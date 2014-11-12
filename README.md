NeoNixie
========

Code to drive my homemade NixieClock. Yay.

My clock design is based on the Atmel AVR ATMega168 microcontroller and uses a Dallas Semiconductor (aka Maxim) DS3234 battery-backed RTC to keep time. The micro controls the state of the Nixie display (and associated other indicator lights) by writing to a chain of 595 logic chips. The parallel outputs of the 595 registers connect to the base of high voltage transistors, which can selectively ground any or all of the Nixie filaments.

~ D
