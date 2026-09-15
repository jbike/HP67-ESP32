Modify HP67 Calculator to use new electronics

HP-67 ESP32 Retrofit
----------------------------------
----------------------------------
Hardware:

- ESP32 WROOM

- TM1640 display driver

- 74LVC245A level shifter

- Original HP-67 circuit board, keyboard, and LEDs

- RUN/PGM switch implemented

- Vape battery w/ charging system

- DC DC battery to 5v converter
------------------------------------
------------------------------------
Current status:

- Trig functions working

- Program mode working

- R/S working

- Calculator works as original
------------------------------------
------------------------------------
Modifications required: 

- Be sure the traces are cut between keyboard and display, they are separate in this implementation

-Remove card reader

-Remove top portion of circuit board support for wiring access. Do not remove the portion supporting the keyboard and switches.
---------------------------------------
---------------------------------------
To do:

- Improve battery system

- Add a few more of the less used programming keys

