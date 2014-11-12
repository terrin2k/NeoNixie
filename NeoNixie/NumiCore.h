//
// NumiCore v1.0 Board Driver Definitions
// by Daniel Corley
// 05/05/08
//

/*************************************************************************************************************/
//
// This section defines the number of bits to be driven in the serial chain.
// This value depends on the type of technology and number of digits making
// up the output display.
//
/*************************************************************************************************************/

#define DADS_CLOCK				0

#define NUMITRON_DISPLAY        0
#define NIXIE_DISPLAY           1
#define VFD_DISPLAY             0
#define DC_PLASMA_DISPLAY       0

#define NUM_REGISTERS           10      // Number of 595's in serial chain

#if (DC_PLASMA_DISPLAY)
#define NUM_DIGITS              16
#else
#define NUM_DIGITS              6
#endif

#if (NUMITRON_DISPLAY)
#define NUM_SEGMENTS            8       // 7-segment digit, plus decimal
#endif
#if (NIXIE_DISPLAY)
//#define NUM_SEGMENTS            12      // 0-9, plus right & left decimal (B5870); depends on tube used.
#define NUM_SEGMENTS            10      // 0-9 only (IN-8)
#endif
#if (VFD_DISPLAY)
#define NUM_SEGMENTS            14      // Typ.
#endif
#if (DC_PLASMA_DISPLAY)
#define NUM_SEGMENTS            16      // 14-segment digit, plus decimal and comma
#endif

#define NUM_EXTRA_BITS          8       // AM/PM indicator, data/time indicator, etc.

// Number of bits in the serial chain. Must be an even multiple of 8. 
#define NUM_DIGIT_BITS          (NUM_DIGITS*NUM_SEGMENTS + NUM_EXTRA_BITS)

#define NUM_DIGIT_BYTES         (NUM_DIGIT_BITS/8)

#define NUM_LEFTOVER_BITS       ((NUM_REGISTERS*8) - (NUM_DIGITS*NUM_SEGMENTS))

/*************************************************************************************************************/
// Display Buffer Stuff
/*************************************************************************************************************/

#define CODE_MASK               (0x03FF)    // 10-bit mask.

// Display Symbol (i.e. nixie digit)
typedef struct
{
    uint16_t    code;                       // Binary code corresponding to new value.
    uint16_t    old_code;                   // Binary code corresponding to old value.
    uint8_t     index;                      // Byte position of the symbol's LSB in the display buffer.
    uint8_t     offset;                     // Bit position of the symbol's LSB in the index byte above.
    uint16_t    mask;                       // Defines how many bits this symbol occupies in the buffer.
    
    uint8_t     threshold;                  // PWM interval threshold. (when in an interval the condition becomes true)
    uint8_t     interval;                   // PWM interval count. (number of elapsed intervals)
    uint8_t     count;                      // PWM sub-interval count. (individual section of an interval)

    // Display Attributes
    bool        blink;                      // Master blink switch
    uint16_t    blink_time;                 // Number of milliseconds per on/off cycle. (50% duty cycle)
    bool        blink_state;                // Current blink state
} displaySymbol_t;

// Byte-wise representation of the display buffer
typedef struct 
{
    displaySymbol_t symbol[NUM_DIGITS+1];   // Symbol data used to generate contents of the data buffer
    uint8_t data[NUM_REGISTERS];            // 10 8-bit registers (80 bits total)
} displayBuffer_t;

// Symbol Indices
#define H10     0       // Hours - Tens
#define H1      1       // Hours - Ones
#define M10     2       // Minutes - Tens
#define M1      3       // Minutes - Ones
#define S10     4       // Seconds - Tens
#define S1      5       // Seconds - Ones
#define P0      6       // Pad 0 (16-bits)

//#define P1      7       // Pad 1 (unused)

/*************************************************************************************************************/
// Character Codes (16-bits)
/*************************************************************************************************************/

#define OFF     0x0000  

#if 0
// IN-8, Test board (scrambled!)
// NOTE: Each value is Endian-swapped for convenience. 
#define ZERO    0x0200
#define ONE     0x0001
#define TWO     0x0004
#define THREE   0x0010
#define FOUR    0x0040
#define FIVE    0x0100
#define SIX     0x0002
#define SEVEN   0x0008
#define EIGHT   0x0020
#define NINE    0x0080
#endif

#if (DADS_CLOCK)

// IN-8, NumiCore v1.0 board (built for Dad)
// This particular instance of the system requires the following
// definitions to operate properly.
// It has AM/PM/DATE indicators, in addition to 2 colon indicators.
// NOTE: Each value is Endian-swapped for convenience.
#define ONE     0x0200
#define TWO     0x0100
#define THREE   0x0080
#define FOUR    0x0040
#define FIVE    0x0020
#define SIX     0x0010
#define SEVEN   0x0008
#define EIGHT   0x0004
#define NINE    0x0002
#define ZERO    0x0001

// Indicator bulb definitions (bit positions in P0)
#define AM      0x0800
#define PM      0x2000
#define DATE    0x8000
#define LCOLON  0x0080
#define RCOLON  0x0200

#else

// IN-8, NumiCore v1.0 board (2nd build)
// This board only has AM/PM indicators and colon indicators. DATE is not used.
// NOTE: Each value is Endian-swapped for convenience. 
#define ONE     0x0200
#define TWO     0x0100
#define THREE   0x0080
#define FOUR    0x0040
#define FIVE    0x0020
#define SIX     0x0010
#define SEVEN   0x0008
#define EIGHT   0x0004
#define NINE    0x0002
#define ZERO    0x0001

// Indicator bulb definitions (bit positions in P0)
#define AM      0x0800
#define PM      0x2000
#define DATE    0x0000
#define LCOLON  0x0080
#define RCOLON  0x0200

#endif

/*************************************************************************************************************/
// Hardware Definitions/Port Pin Functions
/*************************************************************************************************************/

// I/O Config Port B
/*========================================================================
 *
 * Port B pins:                                             PORTB  DDRB 
 *               PB7 -  XTAL2 (GPO temp)                      0      1  
 *               PB6 -  XTAL1 (GPO temp)                      0      1  
 *               PB5 -  SCK                                   0      1  
 *               PB4 -  MISO                                  1      0          (pull-up enabled)
 *                                                           -----------
 *               PB3 -  MOSI                                  0      1  
 *               PB2 -  GPO (SLATCH)                          0      1  
 *               PB1 -  GPO (BLANK)                           0      1  
 *               PB0 -  GPO (SRESET)                          0      1  
 * =======================================================================*/
#define PORTB_DDR       0xEF
#define PORTB_PORT      0x10

// I/O Config Port C
/*========================================================================
 *
 * Port C pins:                                             PORTC  DDRC
 *                                                            0      0
 *               PC6 -  RESET                                 0      0          (Relies on RSTDISBL fuse)
 *               PC5 -  SCL                                   0      1  
 *               PC4 -  SDA                                   0      1
 *                                                           -----------
 *               PC3 -  GPO (!SS_RTC)                         0      1          RTC SPI Chip Select, active low
 *               PC2 -  GPI (EXTSW_C)                         1      0          (pull-up enabled)
 *               PC1 -  GPI (EXTSW_B)                         1      0          (pull-up enabled)  
 *               PC0 -  GPI (EXTSW_A)                         1      0          (pull-up enabled)  
 * =======================================================================*/
#define PORTC_DDR       0x38
#define PORTC_PORT      0x07

// I/O Config Port D
/*========================================================================
 *
 * Port D pins:                                             PORTB  DDRB 
 *               PD7 -  GPO (LED)                             0      1          0 = LED is ON
 *               PD6 -  GPO (unused)                          0      1          
 *               PD5 -  GPO (unused)                          0      1  
 *               PD4 -  GPO (unused)                          0      1
 *                                                           -----------
 *               PD3 -  GPO (!HV_EN)                          1      1			0 = HV Supply Enabled, 1 = Disabled
 *               PD2 -  INT0 (1HZ)                            0      0          Has external pull-up.
 *               PD1 -  TXD                                   0      1  
 *               PD0 -  RXD                                   0      0  
 * =======================================================================*/
#define PORTD_DDR       0xFA
#define PORTD_PORT      0x08

/*************************************************************************************************************/
// Misc Enumerations
/*************************************************************************************************************/
#define DBNCE_LOW       0
#define DBNCE_GO_LOW    1
#define DBNCE_HIGH      2
#define DBNCE_GO_HIGH   3

// Push Button Assignments
#define LBUTTON 1       // ExtSwB
#define RBUTTON 0       // ExtSwA

#define MODE    LBUTTON
#define SEL     RBUTTON

#define BTN_UP  true
#define BTN_DN  false

// Front panel state machine states
enum
{
    SW_IDLE = 0,
    SW_MODE_WAIT,
    SW_SET_LEFT,
    SW_SET_CENTER,
    SW_SET_RIGHT,
};

// Front panel state machine stimuli
#define MODE_STAT   0x01
#define SEL_STAT    0x02

// Time Constants
#define DEFAULT_BLINK_TIME      500     // ms (probably a good idea to keep this > 2 x time for fade to complete)

// Software PWM Constants
#define MAX_DIMMER_STEPS        10

#define FADE_LENGTH             96 //128                        // Number of ticks to complete entire fade
#define FADE_PWM_CYCLES         16                              // Number of ticks in each PWM cycle
#define FADE_THRSH_INTVL        (FADE_LENGTH/FADE_PWM_CYCLES)   // Number of ticks between incrementing PWM threshold.

// Time for fade to complete = FADE_LENGTH*FADE_PWM_CYCLES*0.1 (ms/tick) = 153.6 ms

/*************************************************************************************************************/
// Misc Functions
/*************************************************************************************************************/
void ledControl (bool on);
void hvpsControl (bool on);
void backgroundTasks (void);

/*************************************************************************************************************/
// UART Functions
/*************************************************************************************************************/
unsigned char USART0_Receive (void);
void USART0_Transmit (unsigned char data);

// STDIO interface functions
int uartPutChar (char c, FILE* stream);
int uartGetChar (FILE* stream);

/*************************************************************************************************************/
// Initialization Functions
/*************************************************************************************************************/
void wdtInit (void);
void spiMasterInit (void);
void usartInit (void);
void ioInit (void);
void clockInit (void);
void extIntInit (void);
void timerInit (void);

/*************************************************************************************************************/
// Terminal Functions
/*************************************************************************************************************/
void processInput (void);
void showHelp (void);
void sendBell (void);
void showVersion (void);
void displayPrompt (void);

/*************************************************************************************************************/
// External Switch Handler
/*************************************************************************************************************/
void debounceSwitches (void);
void scanExtSwitches (void);

/*************************************************************************************************************/
// Low-Level SPI Functions
/*************************************************************************************************************/
uint8_t spiTranscieve (uint8_t data);

/*************************************************************************************************************/
// Display Control Function Prototypes
/*************************************************************************************************************/
void blankControl (bool blank); 	// Asserts/de-asserts the BLANK pin.
void resetControl (bool reset);     // Asserts/de-asserts the RESET pin.
void latchControl (bool latch);     // Asserts/de-asserts the SLATCH pin.
void latchDisplay (void);           // Strobes the LATCH pin following a serial write to display chain.

void writeDisplayBuffer (uint8_t *data_ptr, char length); // Writes the display buffer out to the serial chain via the MOSI pin.

void setSymbol (uint8_t symbol, uint16_t code);
void setDataBuffer (uint8_t symbol, uint16_t code);

uint16_t char2Code (char c);
void displayInit (void);

void drawDisplay(void);             // This is the main display control function.

/*************************************************************************************************************/
// Soft PWM Control Function Prototypes
/*************************************************************************************************************/
void refreshGlobalDimmerPWM (void);
void refreshDisplay (void);

/*************************************************************************************************************/
// Time-based Event Functions
/*************************************************************************************************************/
//void timerCheck(void);

void runSwitchStateMachine (void);

