//
// NumiCore v1.0 Board Driver Code
// Copyright (c) 2008 by Daniel Corley
// 05/05/08
//

#include <inttypes.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/atomic.h>

#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "delay_x.h"
#include "iocompat.h"
#include "NumiCore.h"
#include "ds3234.h"

/********************************************************************************************************************/
// Prototypes
/********************************************************************************************************************/

/********************************************************************************************************************/
// Miscellany
/********************************************************************************************************************/
enum { UP, DOWN };

// Timer2 Clock Select and Output Compare value
#define TIMER2_TARGET_RATE  200UL   // target rate in microseconds

#define TIMER2_MAX_COUNT    256UL

// Calculate some values for Timer2B.
#if \
     ( ( TIMER2_MAX_COUNT * 1000UL *    1UL ) / ( F_CPU / 1000UL ) ) > TIMER2_TARGET_RATE
#  define TIMER2_PRESCALER 1
#  define TIMER2_CS        ( 0 << CS22 ) | ( 0 << CS21 ) | ( 1 << CS20 )
#elif \
     ( ( TIMER2_MAX_COUNT * 1000UL *    8UL ) / ( F_CPU / 1000UL ) ) > TIMER2_TARGET_RATE
#  define TIMER2_PRESCALER 8
#  define TIMER2_CS        ( 0 << CS22 ) | ( 1 << CS21 ) | ( 0 << CS20 )
#elif \
     ( ( TIMER2_MAX_COUNT * 1000UL *   32UL ) / ( F_CPU / 1000UL ) ) > TIMER2_TARGET_RATE
#  define TIMER2_PRESCALER 32
#  define TIMER2_CS        ( 0 << CS22 ) | ( 1 << CS21 ) | ( 1 << CS20 )
#elif \
     ( ( TIMER2_MAX_COUNT * 1000UL *   64UL ) / ( F_CPU / 1000UL ) ) > TIMER2_TARGET_RATE
#  define TIMER2_PRESCALER 64
#  define TIMER2_CS        ( 1 << CS22 ) | ( 0 << CS21 ) | ( 0 << CS20 )
#elif \
     ( ( TIMER2_MAX_COUNT * 1000UL *  128UL ) / ( F_CPU / 1000UL ) ) > TIMER2_TARGET_RATE
#  define TIMER2_PRESCALER 128
#  define TIMER2_CS        ( 1 << CS22 ) | ( 0 << CS21 ) | ( 1 << CS20 )
#elif \
     ( ( TIMER2_MAX_COUNT * 1000UL *  256UL ) / ( F_CPU / 1000UL ) ) > TIMER2_TARGET_RATE
#  define TIMER2_PRESCALER 256
#  define TIMER2_CS        ( 1 << CS22 ) | ( 1 << CS21 ) | ( 0 << CS20 )
#elif \
     ( ( TIMER2_MAX_COUNT * 1000UL * 1024UL ) / ( F_CPU / 1000UL ) ) > TIMER2_TARGET_RATE
#  define TIMER2_PRESCALER 1024
#  define TIMER2_CS        ( 1 << CS22 ) | ( 1 << CS21 ) | ( 1 << CS20 )
#else
#  error TIMER2_TARGET_RATE not attainable
#endif

#define TIMER2_OUTPUT_COMPARE \
     ( TIMER2_TARGET_RATE * ( F_CPU / 100UL ) ) / \
     ( TIMER2_PRESCALER * 10000UL )

#define TICKS_PER_MS        (1000/TIMER2_TARGET_RATE)

// Terminal strings
#define ESC                 0x1B        // keyboard Esc sequence
#define ESC_SEQ_TIMEOUT     25          // ESC sequence timeout in 1 ms ticks
#define VT100_CLRSCREEN     "\033[2J"   // VT100 Clear Screen Escape Sequence
#define VT100_CURSORHOME    "\033[H"    // VT100 Cursor Home Escape Sequence

// UART Defines
#define BAUDRATE            38400
#define MYUBRR              (F_CPU/16/BAUDRATE - 1)

#define USART_RX_BUFFER_SIZE 32      /* 2,4,8,16,32,64,128 or 256 bytes */
#define USART_TX_BUFFER_SIZE 128     /* 2,4,8,16,32,64,128 or 256 bytes */
#define USART_RX_BUFFER_MASK ( USART_RX_BUFFER_SIZE - 1 )
#define USART_TX_BUFFER_MASK ( USART_TX_BUFFER_SIZE - 1 )
#if ( USART_RX_BUFFER_SIZE & USART_RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif
#if ( USART_TX_BUFFER_SIZE & USART_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif


/********************************************************************************************************************/
// Global Memory Structures
/********************************************************************************************************************/
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));       // Storage for reset cause.

// Display Buffer
displayBuffer_t displayBuffer;

// Time & Date buffer
extern volatile ds3234timeanddate_t timeAndDate;

// Test code
//const uint16_t sequence[10] = { ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE };

// Tick count
volatile uint32_t currentTime = 0;

// Timer flags
volatile bool timerFlag = false;

// Task Flags
volatile bool updateTime = false;

// External Switch data
volatile bool extSwitchChange[3] = { 0 };               // Debounced state change
volatile bool extSwitchState[3]  = { 0 };               // Debounced switch state
volatile uint32_t extSwitchLastChange[3] = { 0 };       // Timestamp (in ticks) of last switch change

// Overall Brightness Control
volatile uint8_t dimmerControl = MAX_DIMMER_STEPS;  // Full brightness by default.

// Operational Mode Control Flags
volatile bool displayTime = true;                   // Select time/date display. (time is the default state)
volatile bool nixieSaver = false;					// Control flag to start a cathode burn off.

// UART Data
static unsigned char USART_RxBuf[USART_RX_BUFFER_SIZE];
static volatile unsigned char USART_RxHead;
static volatile unsigned char USART_RxTail;
static unsigned char USART_TxBuf[USART_TX_BUFFER_SIZE];
static volatile unsigned char USART_TxHead;
static volatile unsigned char USART_TxTail;

// STDIO stream
FILE myStdIo = FDEV_SETUP_STREAM( uartPutChar, uartGetChar, _FDEV_SETUP_RW );

/********************************************************************************************************************/
// Interrupt handler functions
/********************************************************************************************************************/

// Timer/Counter2 Compare Match Interrupt Handler (0.1 ms tick)
ISR( TIMER2_COMPA_vect )
{
    // Increment the global tick count.
    currentTime++;

    // Indicate a timer tick has occurred.
    timerFlag = true;
} // end ISR( TIMER2_COMP_vect )

// External Interrupt 0 (INT0) (runs on rising and falling edges)
ISR( INT0_vect )
{
    // Indicate we should update the time from the RTC.
    updateTime = true;
}

// USART Receive Interrupt Handler
ISR (USART_RX_vect)
{
	unsigned char data;
	unsigned char tmphead;

	// Read the received data 
	data = UDR0;                 

    // Calculate buffer index 
	tmphead = ( USART_RxHead + 1 ) & USART_RX_BUFFER_MASK;

    // Store new index
	USART_RxHead = tmphead;      

	if ( tmphead == USART_RxTail )
	{
		// ERROR! Receive buffer overflow (drop data)
        return;
	}

    // Store received data in buffer
	USART_RxBuf[tmphead] = data; 
}

ISR (USART_UDRE_vect)
{
	unsigned char tmptail;

	// Check if all data is transmitted
	if ( USART_TxHead != USART_TxTail )
	{
		// Calculate buffer index
		tmptail = ( USART_TxTail + 1 ) & USART_TX_BUFFER_MASK;

        // Store new index
		USART_TxTail = tmptail;

        // Start transmition
		UDR0 = USART_TxBuf[tmptail];
	}
	else
	{
        // Disable UDRE interrupt
		UCSR0B &= ~(_BV(UDRIE0));         
	}
} 

/********************************************************************************************************************/
// USART Control Functions
/********************************************************************************************************************/

// Low-level read and write functions
unsigned char USART0_Receive( void )
{
	unsigned char tmptail;

    // Wait for incomming data
	while ( USART_RxHead == USART_RxTail );

    // Calculate buffer index
	tmptail = ( USART_RxTail + 1 ) & USART_RX_BUFFER_MASK;

    // Store new index
	USART_RxTail = tmptail;                

    // Return data
	return USART_RxBuf[tmptail];           
}

void USART0_Transmit( unsigned char data )
{
	unsigned char tmphead;

	// Calculate buffer index	
	tmphead = ( USART_TxHead + 1 ) & USART_TX_BUFFER_MASK; 

    // Wait for free space in buffer
	while ( tmphead == USART_TxTail );

    // Store data in buffer
	USART_TxBuf[tmphead] = data;

    // Store new index
	USART_TxHead = tmphead;                

    // Enable UDRE interrupt
	UCSR0B |= _BV(UDRIE0);                    
}

unsigned char DataInReceiveBuffer( void )
{
    // Return 0 (FALSE) if the receive buffer is empty
	return ( USART_RxHead != USART_RxTail ); 
}

// STDIO Interface functions
int uartGetChar(FILE* stream)
{
    if (DataInReceiveBuffer())
    {
        return USART0_Receive();
    }
    else
    {
        return _FDEV_EOF;
    }
} // end uartGetChar

int uartPutChar(char c, FILE* stream)
{
    // Transmit the character.
    USART0_Transmit(c);

    // Signal success
    return 0;    
} // end uartPutChar

/********************************************************************************************************************/
// SPI Control Functions
/********************************************************************************************************************/

uint8_t spiTranscieve (uint8_t data)
{
    // Transmit character
    SPDR = data;

    // Wait for transmission to complete
    while (!(SPSR & _BV(SPIF))) {};

    // SPI interface clocks data from slave into SPDR while data is simultaneously being clocked out.
    return SPDR;
}

/********************************************************************************************************************/
// Display Control functions
/********************************************************************************************************************/

// Controls the BLANK_O line to the serial chain. (active low)
void blankControl(bool blank)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (blank)
        {
            // Set BLANK pin.
            PORTB |= _BV(PB1);
        }
        else
        {
            // Clear BLANK pin.
            PORTB &= ~(_BV(PB1));
        }
    }
}

// Controls the SRESET lines to the serial chain. (active low)
void resetControl(bool reset)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)    
    {
        if (reset)
        {
            // Set SRESET pin.
            PORTB |= _BV(PB0);
        }
        else
        {
            // Clear SRESET pin.
            PORTB &= ~(_BV(PB0));
        }
    }
}

// Controls the SLATCH line to the serial chain. 
void latchControl(bool latch)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)    
    {
        if (latch)
        {
            // Set SLATCH pin.
            PORTB |= _BV(PB2);
        }
        else
        {
            // Clear SLATCH pin.
            PORTB &= ~(_BV(PB2));
        }
    }
}

// Utilize the SPI hardware to clock data out the MOSI pin
// into the serial chain.
// Note: entire serial chain must be written following
// an access of the RTC!
// Note 2: This is low priority, so it should be interruptable.
void writeDisplayBuffer(uint8_t* data_ptr, char length)
{
    uint8_t i = 0;

    // 'Select' the serial chain.
    latchControl(0);

    // Set the SPI interface for LSB-first operation. (Set the DORD bit)
    SPCR |= _BV(DORD);

    // Write the entire buffer out to the serial chain.
    // We don't care about any data being returned by the slave. (should be none)
    for (i = 0; i < length; i++)
    {
        spiTranscieve(data_ptr[i]);
    }

    // Worst-case set-up delay on 595 between last bit clock edge and latch is 24 ns.
    _delay_ns(25);    

    // Release the serial chain and latch the shift registers and output the data.
    latchControl(1);
}

// Low-level routine to set a single symbol in the display buffer to the requested code.
void setSymbol (uint8_t symbol, uint16_t code)
{
    // This needs to be done atomically to prevent display corruption.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Check to see if this symbol should blink.
        if (displayBuffer.symbol[symbol].blink)
        {
            // Check to see if it is time for a state change
            if ((currentTime % (displayBuffer.symbol[symbol].blink_time * TICKS_PER_MS)) == 0)
            {
                displayBuffer.symbol[symbol].blink_state = !displayBuffer.symbol[symbol].blink_state;
            }

            // Turn the symbol to 'OFF' when blink_state = TRUE.
            if (displayBuffer.symbol[symbol].blink_state)
            {
                code = OFF;
            }
        }
        
        // Only update anything if there is a change to the code.
        if (code != displayBuffer.symbol[symbol].code)
        {            
            // First, move the previous code to the old_code.
            displayBuffer.symbol[symbol].old_code = displayBuffer.symbol[symbol].code;
            
            // Then update with the new code.
            displayBuffer.symbol[symbol].code = code;

            // Reset the threshold
            displayBuffer.symbol[symbol].threshold = 0;
            displayBuffer.symbol[symbol].interval = 0;
            displayBuffer.symbol[symbol].count = 0;
        }
    }        
}

// This function handles masking and shifting to place data in the correct location in the display data buffer.
void setDataBuffer (uint8_t symbol, uint16_t code)
{
    // This needs to be done atomically to prevent display corruption.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        uint8_t index = displayBuffer.symbol[symbol].index;
        uint8_t offset = displayBuffer.symbol[symbol].offset;
        uint16_t dbmask = displayBuffer.symbol[symbol].mask;

        // Check to make sure we stay in the bounds of the buffer.
        if ((index+1) >= sizeof(displayBuffer.data))
        {            
            return;
        }

        // Now update the display data buffer.
        // Set up a bit mask for this symbol, and shift the code word by the appropriate amount.
        uint16_t mask = dbmask << offset;
        code = code << offset;

        // Clear the appropriate bits in the lower & upper bytes in the buffer.
        displayBuffer.data[index] &= ~(mask);
        displayBuffer.data[index+1] &= ~(mask >> 8);

        // Now place the new code into the lower & upper bytes in the buffer.
        displayBuffer.data[index] |= code;
        displayBuffer.data[index+1] |= (code >> 8);
    }    
}

// Converts the requested character into a 16-bit code. Valid input characters
// and the code that is returned is dependent on display technology in use.
// Unsupported characters should return OFF.
uint16_t char2Code (char c)
{
    uint16_t code = OFF;
    
    switch (c)
    {
        case 0:
           code = ZERO;
        break;        
        case 1:
           code = ONE;
        break;
        case 2:
           code = TWO;
        break;
        case 3:
           code = THREE;
        break;
        case 4:
           code = FOUR;
        break;
        case 5:
           code = FIVE;
        break;
        case 6:
           code = SIX;
        break;
        case 7:
           code = SEVEN;
        break;
        case 8:
           code = EIGHT;
        break;
        case 9:
           code = NINE;
        break;
    }

    return code;
}

void refreshGlobalDimmerPWM (void)
{
    // Global Brightness Soft PWM control
    static uint8_t count = 0;

    // Higher Threshold = brighter output
    blankControl(count >= dimmerControl);

    count++;

    if (count > MAX_DIMMER_STEPS)
    {
        count = 0;
    }
}

// This function handles digit cross-fading.
void refreshDisplay (void)
{
    // Loop through all symbols and look for any changes that need to happen.
    for (uint8_t i = 0; i < (NUM_DIGITS+1); i++)
    {
        // See if there is anything to do.
        // If the threshold > FADE_PWM_CYCLES, do nothing
        if (displayBuffer.symbol[i].threshold <= FADE_PWM_CYCLES)
        {
            // Check against the interval threshold.
            if (displayBuffer.symbol[i].count < displayBuffer.symbol[i].threshold)
            {
                // Display the new digit
                setDataBuffer(i, displayBuffer.symbol[i].code);
            }
            else
            {
                // Display the old digit
                setDataBuffer(i, displayBuffer.symbol[i].old_code);                
            } 

            // Increment the interval count
            displayBuffer.symbol[i].count++;

            // Check the other counters to see if the fade operation is complete.
            if (displayBuffer.symbol[i].count > FADE_PWM_CYCLES)
            {
                // Reset PWM cycle counter
                displayBuffer.symbol[i].count = 0;

                // Increment interval count
                displayBuffer.symbol[i].interval++;

                if (displayBuffer.symbol[i].interval > FADE_THRSH_INTVL)
                {
                    displayBuffer.symbol[i].interval = 0;

                    // Increment threshold if we can.
                    if (displayBuffer.symbol[i].threshold <= FADE_PWM_CYCLES)
                    {
                        displayBuffer.symbol[i].threshold++;   
                    }
                }
            }
        }
        else
        {
            // Display the new digit
            setDataBuffer(i, displayBuffer.symbol[i].code);
        }
    }

    // Write display data to the hardware.
    writeDisplayBuffer(&displayBuffer.data[0], sizeof(displayBuffer.data));
}

/********************************************************************************************************************/
// Misc Functions
/********************************************************************************************************************/

// GP LED Control
void ledControl (bool on)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (on)
        {
            PORTD |= _BV(PD7);
        }
        else
        {
            PORTD &= ~(_BV(PD7));
        }
    }
}

// High Voltage Power Supply Control
void hvpsControl (bool on)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        if (on)
        {
            PORTD &= ~(_BV(PD3));
        }
        else
        {
        	PORTD |= _BV(PD3);
        }
    }
}

// This function picks out the MCUSR and clears the Watch Dog Timer interrupt.
void get_mcusr(void) \
    __attribute__((naked)) \
    __attribute__((section(".init3")));
void get_mcusr(void)
{
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

/********************************************************************************************************************/
// Peripheral Initialization Functions
/********************************************************************************************************************/
void wdtInit (void)
{
    // Save off a copy of the reset flag register.    
    get_mcusr();

    // Set the Watchdog for a 30 ms timeout.
    wdt_enable(WDTO_2S);  // formerly WDTO_30MS

    // Give it a kick.
    wdt_reset();
}

void spiMasterInit (void)
{
    // Port pin directions have already been set.
    // Set up the SPI interface (see pg 164 of ATmega48 datasheet)    
    // Enable SPI, Master, Mode 3, set clock rate fck/8 = 2.5 MHz
    // NOTE: RTC works with Mode 1 or 3, Serial Chain requires Mode 3. Can operate no higher than 4 MHz.
    SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA) | _BV(SPR0);

    // Enable SPI2X mode.
    SPSR = _BV(SPI2X);
    
    // Enable SPI interface
    PRR &= ~(_BV(PRSPI));
}

/*
void i2cInit(void)
{
    // There's more to it than this. 
    // Set up bit rate generator, etc and TWI clock.
    // Set up Master mode.
    // Need to write driver for RTC chip.
    
    TWCR |= _BV(TWEN);

    // Enable I2C interface
    PRR &= ~(_BV(PRTWI));
}
*/

void usartInit( void )
{
    // Initialize the USART.
    // Init buffer data structures
	unsigned char x;

    // Set requested baud rate
    UBRR0H = (uint8_t)(MYUBRR >> 8);
    UBRR0L = (uint8_t)(MYUBRR);

    // Enable transmitter and receiver, USART RX interrupt
    UCSR0B = _BV( TXEN0 ) | _BV( RXEN0 ) | _BV( RXCIE0 );

    // Set frame format: 8-N-1
    UCSR0C = _BV( UCSZ01 ) | _BV( UCSZ00 );

	// Flush receive buffer
	x = 0; 			    

	USART_RxTail = x;
	USART_RxHead = x;
	USART_TxTail = x;
	USART_TxHead = x;    

    // Set stdin/stdout streams to the UART interface.
    stdout = stdin = &myStdIo;
}

// Initialize IO Pins
void ioInit (void)
{
    // Set Port Pin I/O and Function
    // Port B Pin Settings
    DDRB = PORTB_DDR;
    PORTB = PORTB_PORT;

    // Port C Pin Settings
    DDRC = PORTC_DDR;
    PORTC = PORTC_PORT;
    
    // Port D Pin Settings
    DDRD = PORTD_DDR;
    PORTD = PORTD_PORT;

    // Turn off the Analog Comparator
    ACSR = (ACSR & (uint8_t) ~(_BV(ACIE))) | _BV(ACD);
    ADCSRB = 0x00;

    spiMasterInit();
//    i2cInit();    
}

void clockInit (void)
{
    // Clear any pre-scale value from the CLKPR register. 
    // According to the datasheet, you must set the CLKPCE bit first, 
    // then write to the CLKPS bits while clearing the CLKPCE bit.
    CLKPR = 0x80;
    CLKPR = 0x00;

    // Note: clock selection is done via the CLKSEL3..0 Fuse Bits.
    // For external 20 MHz crystal, use: 0111, with SUT1..0 set to: 10
}

void extIntInit (void)
{
    // Initialize any external interrupts.

    // INT0 (1Hz)
    // INT0 Mode: Falling Edge
    // INT1 is OFF.
    EICRA = _BV(ISC01);
    EIMSK = _BV(INT0);

    // Clear the Interrupt Flag Register
    EIFR = _BV(INTF0);
}

void timerInit (void)
{
    // Initialize timers.

    // Initialize Timer/Counter2 to generate tenth-millisecond tick:
    // Load OCR2A to contain the value to be compared against.
    OCR2A = TIMER2_OUTPUT_COMPARE - 1;

    // Don't connect external port pin, select CTC mode (WGM22..WGM20 = 010) and Clock Select
    TCCR2A = (1 << WGM21) | (0 << WGM20);
    TCCR2B = (0 << WGM22) | TIMER2_CS;
    TIMSK2 |= _BV( OCIE2A );  // enable Timer/Counter2A Output Compare Match interrupt
}

void displayInit (void)
{
    // Initialize the displayBuffer, then write it out to the serial registers.
    memset((uint8_t *)&displayBuffer.data[0], 0, sizeof(displayBuffer.data));

    // Enable the high-voltage supply and allow it time to stabilize.
    hvpsControl(true);

    // Also set up displayBuffer.symbol info according to how the hardware is connected.
    // Here's the convention: H10 or Symbol 0 (MSB to LSB) -> Next-to-last register bits Q1-0, last register, bits Q7-0.
    // See diagram in notes to understand how this is set up.

    // This particular setup is for a NumiCore v1.0 build using 2 30-pin headers on the main row with 4 bits unused between them.
    // 10-Hours
    displayBuffer.symbol[H10].code      = OFF;
    displayBuffer.symbol[H10].old_code  = OFF;
    displayBuffer.symbol[H10].index     = H10_INDEX;
    displayBuffer.symbol[H10].offset    = H10_SHIFT;
    displayBuffer.symbol[H10].mask      = CODE_MASK;
    displayBuffer.symbol[H10].blink     = false;
    displayBuffer.symbol[H10].blink_time = DEFAULT_BLINK_TIME;
    displayBuffer.symbol[H10].blink_state = false;

    // 1-Hours
    displayBuffer.symbol[H1].code       = OFF;
    displayBuffer.symbol[H1].old_code   = OFF;
    displayBuffer.symbol[H1].index      = H1_INDEX;
    displayBuffer.symbol[H1].offset     = H1_SHIFT;
    displayBuffer.symbol[H1].mask       = CODE_MASK;
    displayBuffer.symbol[H1].blink      = false;
    displayBuffer.symbol[H1].blink_time = DEFAULT_BLINK_TIME;    
    displayBuffer.symbol[H1].blink_state = false;
    
    // 10-Minutes
    displayBuffer.symbol[M10].code      = OFF;
    displayBuffer.symbol[M10].old_code  = OFF;
    displayBuffer.symbol[M10].index     = M10_INDEX;
    displayBuffer.symbol[M10].offset    = M10_SHIFT;
    displayBuffer.symbol[M10].mask      = CODE_MASK;
    displayBuffer.symbol[M10].blink     = false;
    displayBuffer.symbol[M10].blink_time = DEFAULT_BLINK_TIME;
    displayBuffer.symbol[M10].blink_state = false;    
    
    // 1-Minutes    
    displayBuffer.symbol[M1].code       = OFF;
    displayBuffer.symbol[M1].old_code   = OFF;
    displayBuffer.symbol[M1].index      = M1_INDEX;
    displayBuffer.symbol[M1].offset     = M1_SHIFT;
    displayBuffer.symbol[M1].mask       = CODE_MASK;
    displayBuffer.symbol[M1].blink      = false;
    displayBuffer.symbol[M1].blink_time = DEFAULT_BLINK_TIME;
    displayBuffer.symbol[M1].blink_state = false;
    
    // 10-Seconds
    displayBuffer.symbol[S10].code      = OFF;
    displayBuffer.symbol[S10].old_code  = OFF;
    displayBuffer.symbol[S10].index     = S10_INDEX;
    displayBuffer.symbol[S10].offset    = S10_SHIFT;
    displayBuffer.symbol[S10].mask      = CODE_MASK;
    displayBuffer.symbol[S10].blink     = false;
    displayBuffer.symbol[S10].blink_time = DEFAULT_BLINK_TIME;
    displayBuffer.symbol[S10].blink_state = false;

    // 1-Seconds
    displayBuffer.symbol[S1].code       = OFF;
    displayBuffer.symbol[S1].old_code   = OFF;
    displayBuffer.symbol[S1].index      = S1_INDEX;
    displayBuffer.symbol[S1].offset     = S1_SHIFT;
    displayBuffer.symbol[S1].mask       = CODE_MASK;
    displayBuffer.symbol[S1].blink      = false;
    displayBuffer.symbol[S1].blink_time = DEFAULT_BLINK_TIME;
    displayBuffer.symbol[S1].blink_state = false;

    // Pad 0 (16-bits)
    displayBuffer.symbol[P0].code       = OFF;
    displayBuffer.symbol[P0].old_code   = OFF;
    displayBuffer.symbol[P0].index      = P0_INDEX;
    displayBuffer.symbol[P0].offset     = P0_SHIFT;
    displayBuffer.symbol[P0].mask       = 0xFFFF;
    displayBuffer.symbol[P0].blink      = false;
    displayBuffer.symbol[P0].blink_time = DEFAULT_BLINK_TIME;    
    displayBuffer.symbol[P0].blink_state = false;

    // Ensure we're in the correct display mode.
    displayTime = true;
    
    // Disable outputs
    blankControl(true);

    // Toggle reset line (active low)
    resetControl(false);
    resetControl(true);    

    writeDisplayBuffer(&displayBuffer.data[0], sizeof(displayBuffer.data));

    // Enable Outputs (active low)
    blankControl(false);    
}

/********************************************************************************************************************/
// UART Command Interpreter
/********************************************************************************************************************/

void processInput (void)
{
    uint8_t c, len;
    static uint8_t command[ 13 ] = { 0 };    
    bool done = false;

    // Keep getting characters and putting them into the command buffer until:
    // 1) we hit EOF, then simply return
    // 2) we get a CR/LF, then we interpret the command
    do
    {
        // Command entry mode
        c = getchar();

        // Check for invalid character. If so, no data available from the buffer, so we're done.
        if (( c == (uint8_t)EOF ) || (c == (uint8_t)_FDEV_EOF))
        {
            clearerr(stdin);
            return;
        }
        else
        {
            // Check for CR/LF. If so, command entry is complete and we can process the command.
            if ( ( c != '\r' ) && ( c != '\n' ) ) 
            {
                // Calculate the length of the command in the command buffer.
                len = strlen((const char *)command);                

                // Handle backspace characters properly, otherwise append character to command buffer
                // Handle 'hotkeys' here, too.
                switch ( c )
                {
                    // Backspace
                	case '\b':
                    {                
                        if ( len > 0 )
                        {
                            // Remove the last character from the command buffer.
                            command[len-1] = 0;
                            // Echo a backspace to the terminal.
                            printf_P( PSTR( "\b \b" ) );
                        }
                        else
                        {
                            // Complain that we can't go back any further.                            
                            sendBell();
                        }
                    }
                    break;

#if 0
                    case '~':
                    {
                        // Lock up for WDT Reset if we receive 0x7E, i.e. '~'.
                        while(1);
                    }
                    break;
#endif

                    case '+':
                        // Increase brightness by one step.
                        if (dimmerControl < MAX_DIMMER_STEPS)
                        {
                            dimmerControl++;
                            printf_P(PSTR("%d%% Brightness\r\n"), dimmerControl*100/MAX_DIMMER_STEPS);
                        }
                        else
                        {
                            // Complain
                            sendBell();
                        }
                    break;

                    case '-':
                        // Decrease brightness by one step.
                        if (dimmerControl > 0)
                        {
                            dimmerControl--;
                            printf_P(PSTR("%d%% Brightness\r\n"), dimmerControl*100/MAX_DIMMER_STEPS);
                        }            
                        else
                        {
                            // Complain
                            sendBell();
                        }            
                    break;
                    
                    // All other characters
                    default:
                    {
                        // Check for room for another char in the command buffer, then append.
                        // Need to leave room for a '\0' at the end.
                        if ( len < ( sizeof( command ) - 2 ) ) 
                        {
                            // This is a regular character, so echo it.
                            putchar( c );
                            command[len] = c;
                        }
                        else
                        {
                            // Complain and toss the new character.
                            sendBell();
                        }
                    }
                    break;
                } // end switch ( c )
            } // end if CR/LF
            else
            {
                // We've received a CR/LF, so we're done
                done = true;
            }
        }
    } while (!done);

    // Command entry is done for now. Time for command interpretation!
    // Echo a CR/LF.
    printf_P( PSTR("\r\n") );

    switch ( command[0] )
    {
        case '?':
            showDateTime();
            break;

        case 't':
            setTime((char *)command);
            break;

        case 'd':
            setDate((char *)command);
            break;

        case 'y':
            setDay((char *)command);
            break;

        case 'c':
            showTemperature();
            break;

		// Test Code
		// Dump contents of display buffer to screen.
		case 'b':
		{
			uint8_t i;

			for (i = 0; i < sizeof(displayBuffer.data); i++)
			{
				printf_P(PSTR("%d: 0x%02x\r\n"), i, displayBuffer.data[i]);
			}
		}
		break;

        case 'v':
            showVersion();
            break;
            
        case 'h':
            showHelp();
            break;

        default:
        {
            printf_P( PSTR( "Unknown command: " ) );
            printf((const char *)command);
            printf_P( PSTR( "\r\n" ) );            
        }
        break;
    } // end switch    

    // Now zero out the command buffer, since we're done with it.
    memset(command, 0, sizeof(command));

    displayPrompt();
}

/********************************************************************************************************************/
// Terminal Commands
/********************************************************************************************************************/

void showHelp(void)
{
  printf_P( PSTR( "? - Show time, date and day of week\r\n" ) );
  printf_P( PSTR( "t - Set time\r\n" ) );
  printf_P( PSTR( "y - Set day\r\n" ) );
  printf_P( PSTR( "d - Set date\r\n" ) );
  printf_P( PSTR( "c - Show temperature\r\n" ) );
  printf_P( PSTR( "+ - Increase brightness\r\n" ) );
  printf_P( PSTR( "- - Decrease brightness\r\n" ) );
  printf_P( PSTR( "b - Dump display buffer\r\n" ) );
  // Add other options here.
  // Enable/Disable Fading
  // 12/24 hour display
  // etc...
  printf_P( PSTR( "v - Show SW version info\r\n" ) );  
  printf_P( PSTR( "h - Help (this info)\r\n" ) );
} // end showHelp

void sendBell (void)
{
    // Print the BELL character.
    printf_P( PSTR("\a") );
}

void displayPrompt(void)
{
    printf_P(PSTR("> "));
}

void showVersion (void)
{
    printf_P( PSTR( "NumiCore v1.1 by Daniel Corley\r\n" ) );
    printf_P( PSTR( "Built on %s at %s.\r\n" ), __DATE__, __TIME__);
}

/********************************************************************************************************************/
// External Switch Handlers
/********************************************************************************************************************/

// Debounce state machine. This samples and buffers the pin states to provide a stabilized status on each pin.
void debounceSwitches(void)
{
    uint8_t i;
    uint8_t nextState;
    static uint8_t state[3] = { DBNCE_LOW, DBNCE_LOW, DBNCE_LOW };
    static uint8_t swBuff[3] = { 0 };

    // Sample the switch pin port.
    uint8_t data = PINC;

    // Loop through state machine for each state variable.
    for (i = 0; i < sizeof(state); i++)
    {
        // First pick off each bit and shift it into the buffer.
        // Note: it's a lucky coincidence that the port pins are 0, 1 and 2.
        swBuff[i] = (swBuff[i] << 1) | ((data & (1 << i)) ? 1 : 0);    

        // Determine which state to go to next.
        nextState = state[i];
        
        switch (state[i])
        {
            case (DBNCE_GO_LOW):
            {
                switch(swBuff[i])
                {
                    case(0x00):
                        // Looks like a change has occurred.
                        nextState = DBNCE_LOW;
                        extSwitchChange[i] = true;
                        extSwitchState[i] = false;
                    break;
                    case(0xFF):
                        nextState = DBNCE_HIGH;
                    break;
                    default:
                        // No change.
                    break;
                }
            }
            break;
            case (DBNCE_LOW):
            {
                if (swBuff[i] != 0x00)
                {
                    nextState = DBNCE_GO_HIGH;
                }
                // Otherwise, no change.
            }
            break;
            case (DBNCE_GO_HIGH):
            {
                switch(swBuff[i])
                {
                    case(0x00):                        
                        nextState = DBNCE_LOW;                        
                    break;
                    case(0xFF):
                        nextState = DBNCE_HIGH;
                        extSwitchChange[i] = true;
                        extSwitchState[i] = true;
                    break;
                    default:
                        // No change.
                    break;
                }                
            }
            break;
            case (DBNCE_HIGH):
            {
                if (swBuff[i] != 0xFF)
                {
                    nextState = DBNCE_GO_LOW;
                }
                // Otherwise, no change.                
            }            
            break;
        }

        // Update state
        state[i] = nextState;
    }
}

// NOTE: 'true' indicates 'button up' and 'false' indicates the 'button down'.
void scanExtSwitches(void)
{
    // Run the debounce routine.
    debounceSwitches();

    // Spin through the switches to see if any changes have been made.
    if (extSwitchChange[LBUTTON])
    {      
        // Change of state detected. Mark the time this occurred.
        extSwitchLastChange[LBUTTON] = currentTime;

        // Reset the flag.
        extSwitchChange[LBUTTON] = false;
    }
    
    if (extSwitchChange[RBUTTON])
    {
        // Change of state detected. Mark the time this occurred.
        extSwitchLastChange[RBUTTON] = currentTime;

        // Reset the flag.
        extSwitchChange[RBUTTON] = false;
    }
}

/********************************************************************************************************************/
// Front Panel State Machine
// This is polled every 1 milliseconds.
/********************************************************************************************************************/
void runSwitchStateMachine (void)
{
    static uint8_t state = SW_IDLE;
    static bool modePrev = true;
    static bool selPrev = true;
    uint8_t nextState = SW_IDLE;
    uint32_t modeDelta = 0;
    uint32_t selDelta = 0;    
    bool modeState = false;     // True is UP and False is DOWN
    bool selState = false;
    bool modeChange = false;
    bool selChange = false;

    // Calculate the current switch deltas.    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Get the current switch states.
        modeState = extSwitchState[MODE];
        selState = extSwitchState[SEL];        
        
        // Check the switches against their various time rules.
        modeDelta = currentTime - extSwitchLastChange[MODE];
        selDelta = currentTime - extSwitchLastChange[SEL];
    }

    // Determine if any switch state changes have occurred.
    if (modePrev != modeState)
    {
        modeChange = true;
        modePrev = modeState;
    }

    if (selPrev != selState)
    {
        selChange = true;
        selPrev = selState;
    }    

    // Initialize nextState to current state so that if no change is needed, none will occur.
    nextState = state;

    switch (state)
    {
        case (SW_IDLE):
            // Check to see where we should go from here.

            // Wait for the mode button to be pressed.
            if (modeChange && (modeState == BTN_DN))
            {
                nextState = SW_MODE_WAIT;
            }

        break;

        case (SW_MODE_WAIT):
            // Stay in this state until either the button is released or it is held for 5 seconds.
            if (modeState == BTN_UP)
            {
                // Button has been released
                // Toggle the display mode.
                displayTime = !displayTime;
                 
                // Return to idle.
                nextState = SW_IDLE;
            }
            else
            {
                // Check to see how long the button has been pressed.
                if (modeDelta > (5000*TICKS_PER_MS))
                {
                    // Activate time setting mode.

                    // Go to the left-most set of digits first.
                    nextState = SW_SET_LEFT;
                }
            }
        break;

        case (SW_SET_LEFT):
            // Hours/Months should blink.            
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                displayBuffer.symbol[H10].blink = true;
                displayBuffer.symbol[H1].blink = true;
            }

            // If the mode button is pressed again, go to the set minutes/days state.
            if (modeChange && (modeState == BTN_DN))
            {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                {
                    displayBuffer.symbol[H10].blink = false;
                    displayBuffer.symbol[H1].blink = false;
                }
                
                nextState = SW_SET_CENTER;
            }

            // If the select button is pressed, the appropriate value should be incremented.            
            if (selChange && (selState == BTN_DN))
            {
                if (displayTime)
                {
                    // Increment the hour count. Don't forget to update AM/PM as needed.
                    incrementHours();
                }
                else
                {
                    // Increment the month count.
                    incrementMonths();
                }
                
                // Indicate that timeAndDate needs to be refreshed.
                updateTime = true;                
            }

            // Check to see how long the buttons have been released.
            if (modeState && selState)
            {
                if ((modeDelta > (10000*TICKS_PER_MS)) && (selDelta > (10000*TICKS_PER_MS)))
                {
                    // Return to idle mode if no activity for 10 seconds.
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                    {
                        displayBuffer.symbol[H10].blink = false;
                        displayBuffer.symbol[H1].blink = false;
                    }

                    nextState = SW_IDLE;
                }        
            }
        break;

        case (SW_SET_CENTER):
            // Minutes/Days should blink.
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                displayBuffer.symbol[M10].blink = true;
                displayBuffer.symbol[M1].blink = true;
            }

            // If the mode button is pressed again, go to the set seconds/years state.
            if (modeChange && (modeState == BTN_DN))
            {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                {
                    displayBuffer.symbol[M10].blink = false;
                    displayBuffer.symbol[M1].blink = false;
                }
                
                nextState = SW_SET_RIGHT;
            }

            // If the select button is pressed, the appropriate value should be incremented.            
            if (selChange && (selState == BTN_DN))
            {
                if (displayTime)
                {
                    // Increment the minute count.
                    incrementMinutes();
                }
                else
                {
                    // Increment days.
                    incrementDays();
                }                

                // Indicate that timeAndDate needs to be refreshed.
                updateTime = true;                
            }

            // Check to see how long the buttons have been released.
            if (modeState && selState)
            {
                if ((modeDelta > (10000*TICKS_PER_MS)) && (selDelta > (10000*TICKS_PER_MS)))
                {
                    // Return to idle mode if no activity for 10 seconds.
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                    {
                        displayBuffer.symbol[M10].blink = false;
                        displayBuffer.symbol[M1].blink = false;
                    }

                    nextState = SW_IDLE;
                }        
            }            
        break;

        case (SW_SET_RIGHT):
            // Seconds/Year should blink.
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                displayBuffer.symbol[S10].blink = true;
                displayBuffer.symbol[S1].blink = true;
            }

            // If the mode button is pressed again, go to the set left state again, but toggle the display mode.
            if (modeChange && (modeState == BTN_DN))
            {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                {
                    displayBuffer.symbol[S10].blink = false;
                    displayBuffer.symbol[S1].blink = false;
                }

                // Toggle display mode
                displayTime = !displayTime;
                
                nextState = SW_SET_LEFT;
            }

            // If the select button is pressed, the appropriate value should be incremented.            
            if (selChange && (selState == BTN_DN))
            {
                if (displayTime)
                {
                    // Increment the second count.
                    incrementSeconds();
                }
                else
                {
                    // Increment years.
                    incrementYears();
                }

                // Indicate that timeAndDate needs to be refreshed.
                updateTime = true;                
            }            

            // Check to see how long the buttons have been released.
            if (modeState && selState)
            {
                if ((modeDelta > (10000*TICKS_PER_MS)) && (selDelta > (10000*TICKS_PER_MS)))
                {
                    // Return to idle mode if no activity for 10 seconds.
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
                    {
                        displayBuffer.symbol[S10].blink = false;
                        displayBuffer.symbol[S1].blink = false;
                    }

                    nextState = SW_IDLE;
                }
            }
        break;

        default:            
            nextState = SW_IDLE;            
        break;
    }

//  if (state != nextState)
//  {
//      printf_P(PSTR("SM: %d -> %d\r\n"), state, nextState);
//  }

    // Update the state variable.
    state = nextState;
}

/********************************************************************************************************************/
// Display Rendering Function (runs every 10 ms)
/********************************************************************************************************************/
void drawDisplay(void)
{
    static bool colonState = false;
	static uint8_t nsValue = 0;
    uint8_t val = 0;
    uint16_t code = 0;
    
	if (!nixieSaver)
	{
	    // Update the display.
	    // Determine whether to display time or date based on display mode.
	    // Use a display mode switch here?
	    if (displayTime) 
	    {
	        // Update the display with the current time only when it has been updated.        
	        // Suppress zero on the H10 character.
	        val = (timeAndDate.hours.mode12) ? (timeAndDate.hours.hours10 & 0x01) : (timeAndDate.hours.hours10);        

	        setSymbol(H10, (val == 0) ? OFF : (char2Code(val)));
	        setSymbol(H1, char2Code(timeAndDate.hours.hours1));
	        setSymbol(M10, char2Code(timeAndDate.minutes.minutes10));
	        setSymbol(M1, char2Code(timeAndDate.minutes.minutes1));
	        setSymbol(S10, char2Code(timeAndDate.seconds.seconds10));
	        setSymbol(S1, char2Code(timeAndDate.seconds.seconds1));

	        // Determine indicator status.
	        if (timeAndDate.hours.mode12)
	        {
	            // Light AM or PM as appropriate
	            code |= (timeAndDate.hours.hours10 & 0x02) ? PM : AM;
	        }

	        // Leave the Left & Right colons on forever. (for now)
	        code |= (LCOLON | RCOLON);
	        colonState = true;
        
	        setSymbol(P0, code);
	    }
	    else
	    {
	        // Update the display with the current date. (MM/DD/YY)
	        setSymbol(H10, char2Code(timeAndDate.month.month10));
	        setSymbol(H1, char2Code(timeAndDate.month.month1));
	        setSymbol(M10, char2Code(timeAndDate.date.date10));
	        setSymbol(M1, char2Code(timeAndDate.date.date1));
	        setSymbol(S10, char2Code(timeAndDate.year.year10));
	        setSymbol(S1, char2Code(timeAndDate.year.year1));
       
	        // Light the Date indicator
	        code |= DATE;

	        // Blink the colons to indicate the unit is still running.
	        if (colonState)
	        {
	            code |= (LCOLON | RCOLON);
	        }

	        // Toggle colonState every 1000 ms or so.
	        if ((currentTime % (1000*TICKS_PER_MS)) == 0)
	        {
	            colonState = !colonState;
	        }
        
	        setSymbol(P0, code);           
	    }
	}
	else
	{
		// Roll all digits to clean the cathodes.
		if (currentTime % (1000*TICKS_PER_MS) == 0)
		{
			// Write the current value to the display.
	        setSymbol(H10, char2Code(nsValue));
	        setSymbol(H1, char2Code(nsValue));
	        setSymbol(M10, char2Code(nsValue));
	        setSymbol(M1, char2Code(nsValue));
	        setSymbol(S10, char2Code(nsValue));
	        setSymbol(S1, char2Code(nsValue));

			// Increment the current value.
			nsValue++;

			// Check to see if we're done.
			if (nsValue == 10)
			{
				// Reset the flag.
				nixieSaver = false;
			}				
		}			
	}
}

/********************************************************************************************************************/
// Main Function
/********************************************************************************************************************/
int main (void)
{
    // Init Watchdog timer.
    //wdtInit();
    
    // Initialize all the other stuff.
    clockInit();
    ioInit();
    extIntInit();
    timerInit();

    // Setup standard I/O.
    usartInit();

    // Initialize the shift registers.
    displayInit();

    // Enable global interrupts
    sei();

    // Initialize the Real Time Clock
    initRtc();

    // Initialize any other flags that require it.
    // These switches are NO and pulled up, so their default state is HIGH.
    extSwitchState[MODE] = true;
    extSwitchState[SEL] = true;

    // Set up CPU sleep options: keep Timer2 & USART running
    set_sleep_mode(SLEEP_MODE_IDLE);

    // Display some welcome text.
    puts_P( PSTR( VT100_CLRSCREEN VT100_CURSORHOME ) );
    showVersion();
    printf_P( PSTR( "Enter \"h\" for help.\r\n" ) );
       
    displayPrompt();

    // Simply loop forever, the interrupts are doing the rest.
    for (;;)
    {
        sleep_enable();     // Enables the SE bit in the MCUCR
        sleep_mode();       // Put the CPU to sleep. >>> Code will continue execution from here after waking. <<<
        sleep_disable();    // Disable sleep first thing after waking.

        backgroundTasks();  // Since we have a 1 ms timer interrupt, this will execute a minimum of once per ms.
    }

    return (0);
}

/********************************************************************************************************************/
// Background Task Loop
// This should only run when the timer interrupt determines there is something to wake up for.
/********************************************************************************************************************/
void backgroundTasks (void)
{
    // Kick the watchdog.
    //wdt_reset();

    // Update the display buffer at the appropriate time. (once per second)
    if (updateTime)
    {
		updateTime = false;
		rtcRead(RTC_BASE_ADDR, (uint8_t *)&timeAndDate, sizeof(timeAndDate));

		// Check for 3:00:00 AM
		if (((timeAndDate.hours.hours10 & 0x02) == 0) &&
			(timeAndDate.hours.hours1 == 3) &&
			(timeAndDate.minutes.minutes10 == 0) &&
			(timeAndDate.minutes.minutes1 == 0) &&
			(timeAndDate.seconds.seconds10 == 0) &&
			(timeAndDate.seconds.seconds1 == 0) )
		{
			nixieSaver = true;
		}
    }

    // Run the following functions ONLY when the tick timer fires.
    if (timerFlag)
    {
        // Reset the timer flag.
        timerFlag = false;

        // ===== Timed Tasks =====
        // 1 ms task
        if ((currentTime % (TICKS_PER_MS)) == 0)
        {
            // Process terminal input
            processInput();

#if (BUTTONS_PRESENT)
            // Check for any external switch changes.
            scanExtSwitches();
#endif
        }

        // 10 ms task
        if ((currentTime % (10*TICKS_PER_MS)) == 0)
        {
#if (BUTTONS_PRESENT)
            // Run the switch state machine
            runSwitchStateMachine();
#endif
            
            // Update the display.
            drawDisplay();            
        }

        // Calculate display PWM stuff & refresh the display hardware.
        refreshDisplay();

        // Refresh the Global Dimmer.
        refreshGlobalDimmerPWM();

    }
}

/*
    // Echo any received characters.
    unsigned char c = getchar();

    if ((c == (unsigned char)EOF) || (c == (unsigned char)_FDEV_EOF))
    {            
        clearerr(stdin);
    }
    else
    {
        ledControl(true);
        putchar(c);
        //printf("0x%2X ", c);
        ledControl(false);
    }
*/    


// Here is some code that can be put in a background task to perform the 'single digit' fad in/out clock.
//      // 250 ms task
//      if ((currentTime % (500*TICKS_PER_MS)) == 0)
//      {
//          // 250 ms task
//          static uint8_t num = 0;
//   
//          // Read one digit of the current time and display it.
//          switch(num)
//          {
//              case 0:
//                  // Hours (tens)
//                  setSymbol(H10, char2Code((timeAndDate.hours.mode12) ? (timeAndDate.hours.hours10 & 0x01) : timeAndDate.hours.hours10));
//              break;                
//              case 2:
//                  // Hours (ones)
//                  setSymbol(H10, char2Code(timeAndDate.hours.hours1));
//              break;
//              case 4:
//                  // Minutes (tens)
//                  setSymbol(H10, char2Code(timeAndDate.minutes.minutes10));
//              break;
//              case 6:
//                  // Minutes (ones)
//                  setSymbol(H10, char2Code(timeAndDate.minutes.minutes1));
//              break;                
//              default:
//                  // Blank
//                  setSymbol(H10, OFF);
//              break;            
//          }
//          
//          num++;
//    
//          // Wrap around
//          if (num > 10)
//          {
//              num = 0;
//          }
//      }

