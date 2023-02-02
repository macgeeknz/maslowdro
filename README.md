// MaslowDROesp32 v1.0 2023-02-02
//
// This sketch is designed to run on a generic ESP32 development board. The RX port of the second UART is used to receive
// a copy of the TTL serial data sent from the Arduino's ATMEGA CPU back to the host PC (Raspberry Pi running Web Control in the author's case)
// 
// The sled position information is parsed out from a subset of the data received and then displayed on a 7-digit display in the form of
// a 4 digit display (for X position) and a 3 digit display (for Y position).
//
// Colour (red/green) is used to indicate the sign of the X and Y positions (positive or negative). The Z axis position is shown immediately for a short time
// whenever it changes and perodically when it does not change. The Z position uses white and blue to show sign and is displayed on the 4-digit part of the display
//
// The display itself is made up of a single string of 147 serially-addressable RGB LEDs, in the form of 7 separate 21-LED displays that are daisy-chained together
//
// The PCB file and BOM for the display boards is made free by the author for any purpose
// 
// Please note, the author of this sketch lives in a country that uses the metric system. Adaption to displaying measurement in bananas is left as an exercise for the user.
//
// The idea of how to do this in the first place was taken from this wonderful project:
//	https://github.com/MaslowCommunityGarden/Big-Z-value-display
//
