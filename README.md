# CigarBoxRailroad
CBRR - Firmware to control a Railroad inside a Cigar Box.

Size of the Cigar Box is 17.7x13 cm (7x5.1 inch)

# Firmware
Originally the firmware was written in BASCOM-AVR for a AT90S4433.   
The firmware has been rewritten for Arduino and will likely run on any Arduino with at least 13 I/O pins.

## Functionality
There are two tracks, which travel in opposite direction.
The trains can pass each other at the station.

Control Panel:
- Track selector button: Toggles between track 1 and 2.
- Mode select button: Toggles between manual and automatic mode.
- Speed knob: Manually controls the train speed. In automatic mode it controls the maximum train speed.

In Automatic mode a train will automatically start to drive for several laps.
The train slows down when it passes the station to continue for the next round.
On the last round the train will slow down and come to a halt at the station.

After some delay the other track will depart in the opposite direction.

This will continue until the 'Mode' button is pressed. When this happens, any running train will finish its round and stop at the station.


