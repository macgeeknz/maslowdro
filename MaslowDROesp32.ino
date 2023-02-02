// MaslowDROesp32 v1.0 2023-02-02, pete at mac d0t geek d0t nz
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

#include <Adafruit_NeoPixel.h>    // We stand on the shoulders of giants
#define numberOfDisplays 7        // There are 7 display panels (first 4 for X, remaining 3 for Y)
#define LEDsPerDisplay 21         // 21 LEDs per panel
#define STRIP_LED_COUNT 147       // Therefore 147 LEDs total
#define STRIP_PIN 13                // Connected to IO13 on the ESP32

Adafruit_NeoPixel strip(STRIP_LED_COUNT, STRIP_PIN, NEO_GRB + NEO_KHZ800);     // define the NeoPixel object

// These arrays contains the individual LED numbers that need to be turned on to show each particular digit
int digitZero[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 16, 17, 18, 19, 20, 21 };
int digitOne[] = { 10, 11, 12, 16, 17, 18 };
int digitTwo[] = { 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 19, 20, 21 };
int digitThree[] = { 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
int digitFour[] = { 1, 2, 3, 10, 11, 12, 13, 14, 15, 16, 17, 18 };
int digitFive[] = { 1, 2, 3, 7, 8, 9, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
int digitSix[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
int digitSeven[] = {7, 8, 9, 10, 11, 12, 16, 17, 18 };
int digitEight[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21 };
int digitNine[] = { 1, 2, 3, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18 };
int decimalPoint[] = { 14 };

const int zDisplayTime = 4500;            // a variable to store the time for which to display the Z value after it last changed
char buf[60];                             // a buffer to hold the data received via softwareSerial
char xVal[4];                             // a buffer to hold the new x position data
char lastXVal[4];                         // a buffer to hold last seen x position data 
char yVal[3];                             // a buffer to hold the new y position data
char lastYVal[3];                         // a buffer to hold last seen y position data 

char zVal[2];                             // a buffer to hold the new z position data
char lastZVal[2];                         // a buffer to hold last seen z position data 

char zHundrethsVal[2];                    // a buffer to hold the new z hundreths position data
char lastZHundrethsVal[2];                // a buffer to hold last seen z hundreths position data 

long lastSerialUpdate = 0;                // a variable to store the millis at the last serial update
int minSerialInterval = 4000;             // the minimum rate at which we provide position reports over the debug serial channel
long lastZchange = 0 - zDisplayTime;      // a variable to store the millis each time the Z axis position changes
int zDisplayActive = 0;                   // a variable we use to track whether the Z display is active or not

int xNeg = 0;                     // we set this to 1 if we determine that the x value is negative
int xStartPos = 0;                // the position (offset) in the buffer at which the x value data starts
int xStopPos = 0;                 // the position (offset) in the buffer at which the x value data stops
int xLen = 0;                     // the length of the x value data
int maxZdisplayInterval = 25000; // the longest time we should go without displaying the z height

int yNeg = 0;                     // we set this to 1 if we determine that the y value is negative
int yStartPos = 0;                // the position (offset) in the buffer at which the y value data starts
int yStopPos = 0;                 // the position (offset) in the buffer at which the y value data stop
int yLen = 0;                     // the length of the x value data
  
int zNeg = 0;                     // we set this to 1 if we determine that the z value is negative
int zStartPos = 0;                // the position (offset) in the buffer at which the z value data starts
int zStopPos = 0;                 // the position (offset) in the buffer at which the z value data stop
int zLen = 0;                     // the length of the z value data
int zHundrethsStartPos;           // the position (offset) in the buffer at which the z hundreths value data starts

int posCounter = 0;               // Variable used within iterative loops
int matchAlreadyFound = 0;        // Variable used within iterative loops
int numOfBytesRead = 0;           // Variable used to count number of serial bytes read
int displayNumberOffset = 0;      // a variable used to track any offset necessary for right-justifying digits
  
int r = 32;                       // Red PWM component (display colour)
int g = 32;                       // Red PWM component (display colour)
int b = 32;                       // Blue PWM component  (display colour)

void setup() {
  strip.begin();                                  // Initialise Neopixel library
  strip.show();                                   // Turn OFF all pixels at boot
  Serial2.begin(57600, SERIAL_8N1, 16, 17);       // Initialise Maslow serial on D16
  Serial.begin(57600);                            // Initialise USB/built-in serial (for debug)
  Serial.printf("Startup\n");
  readSerial();
  showXYdisplay();
}

void loop() {
    readSerial();
    writeOutIfValChanged();                          
    if ( millis() - lastSerialUpdate > minSerialInterval ) {           // Update the serial at least every minSerialInterval
          writeOutToSerial();
    }

    
    if ( millis() - lastZchange > maxZdisplayInterval ) { lastZchange = millis(); showZdisplay(); } // Show the Z height at least every maxZdisplayInterval
}

void readSerial() {
      while (Serial2.available() > 0) {                                   // Each time some serial data becomes available in the serial buffer,
          if ( Serial2.read() == '<' ) {                                // if the next available byte is a less than sign then
            numOfBytesRead = Serial2.readBytesUntil('>',buf,80);        // read the following data up until greater than sign or 80 bytes into another buffer
//          for (int i=0; i<sizeof(buf); i++) { Serial.write(buf[i]); } // DEBUG - uncomment this line to copy the entire buffer to the USB serial each time it is read

            extractXposition();
            for (int p=0; p<xLen; p++) { xVal[p] = buf[xStartPos + p];  } // Copy the x position from our buffer of serial-data into it's own buffer
            
            extractYposition();
            for (int p=0; p<yLen; p++) { yVal[p] = buf[yStartPos + p]; } // Copy the y position from our buffer of serial-data into it's own buffer

            extractZposition(); 
            for (int p=0; p<zLen; p++) { zVal[p] = buf[zStartPos + p]; } // Copy the z position from our buffer of serial-data into it's own buffer
            zHundrethsVal[0] = buf[zHundrethsStartPos];
            zHundrethsVal[1] = buf[zHundrethsStartPos+1];
          }
      }
}

void writeOutIfValChanged() {
    int xChanged=0;
    int yChanged=0;
    int zChanged=0;
    for (int i=0; i<xLen; i++) {          // Compare the latest xVal to the last one
      if (lastXVal[i] != xVal[i]) {
        lastXVal[i] = xVal[i];
        xChanged=1;
      }
    }
    for (int i=0; i<yLen; i++) {          // Compare the latest yVal to the last one
      if ( lastYVal[i] != yVal[i] ) {
        lastYVal[i] = yVal[i];
        yChanged=1;
      }
    }
    for (int i=0; i<zLen; i++) {          // Compare the latest zVal to the last one
      if ( lastZVal[i] != zVal[i] ) {
        lastZVal[i] = zVal[i];
        zChanged=1;
      }
    }
    for (int i=0; i<2; i++) {          // Compare the latest zVal to the last one
      if ( lastZHundrethsVal[i] != zHundrethsVal[i] ) {
        lastZHundrethsVal[i] = zHundrethsVal[i];
        zChanged=1;
      }
    }

    if ((zDisplayActive == 1) && (millis() - lastZchange > zDisplayTime)) {    // If the zDisplay has been shown long enough and hasn't changed, revert to XY
      zDisplayActive = 0;
      showXYdisplay();  
    }

    if  ( zChanged == 1 ) {                                 // If the Z axis has changed, start showing that again
      lastZchange = millis();
      showZdisplay();
      writeOutToSerial();
    }
    else {
            if ((xChanged == 1) || ( yChanged == 1 )) {     // otherwise update the XY display only when their positions change
              showXYdisplay();
              writeOutToSerial();
            }
      }
}

void showZdisplay() {
          zDisplayActive = 1;
          for ( int i=0; i<STRIP_LED_COUNT; i++ ) { strip.setPixelColor(i, 0,0,0); }  // Clear the display

          displayNumberOffset = 0;                                                       // Left-justify the Z display
          if ( zNeg == 1 ) { r=24; g=24; b=24; }                                        // Set colour to white when below zero
          else { r=0; g=0; b=64; }                                                      // Otherwise blue
          for (int i=0; i<zLen; i++) {
            if ( zVal[i] == '0' ) { writeNumberToLEDfb(0, i+displayNumberOffset); }
            if ( zVal[i] == '1' ) { writeNumberToLEDfb(1, i+displayNumberOffset); }
            if ( zVal[i] == '2' ) { writeNumberToLEDfb(2, i+displayNumberOffset); }
            if ( zVal[i] == '3' ) { writeNumberToLEDfb(3, i+displayNumberOffset); }
            if ( zVal[i] == '4' ) { writeNumberToLEDfb(4, i+displayNumberOffset); }
            if ( zVal[i] == '5' ) { writeNumberToLEDfb(5, i+displayNumberOffset); }
            if ( zVal[i] == '6' ) { writeNumberToLEDfb(6, i+displayNumberOffset); }
            if ( zVal[i] == '7' ) { writeNumberToLEDfb(7, i+displayNumberOffset); }
            if ( zVal[i] == '8' ) { writeNumberToLEDfb(8, i+displayNumberOffset); }
            if ( zVal[i] == '9' ) { writeNumberToLEDfb(9, i+displayNumberOffset); }
          }
          displayNumberOffset = zLen;                                                    // Position our decimal point
          writeNumberToLEDfb(10, displayNumberOffset);                                   // And write it to the framebuffer
          displayNumberOffset++;
          int zDisplayLength = 2;
          if ( zLen == 2 ) { zDisplayLength = 1; };                                      // If our Z value is in double-digits then we trim the least significant digit off what we are dispaying
          for (int i=0; i<zDisplayLength; i++) {                                         // Position and write our z hundreths digits to the framebuffer
            if ( zHundrethsVal[i] == '0' ) { writeNumberToLEDfb(0, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '1' ) { writeNumberToLEDfb(1, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '2' ) { writeNumberToLEDfb(2, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '3' ) { writeNumberToLEDfb(3, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '4' ) { writeNumberToLEDfb(4, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '5' ) { writeNumberToLEDfb(5, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '6' ) { writeNumberToLEDfb(6, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '7' ) { writeNumberToLEDfb(7, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '8' ) { writeNumberToLEDfb(8, i+displayNumberOffset); }
            if ( zHundrethsVal[i] == '9' ) { writeNumberToLEDfb(9, i+displayNumberOffset); }
          }
          strip.show();                                                                   // Write the framebuffer to the display
}

void showXYdisplay() {
          for ( int i=0; i<STRIP_LED_COUNT; i++ ) { strip.setPixelColor(i, 0,0,0); }  // Clear the display

          int displayNumberOffset = 4 - xLen;                                         // prefix the X value with blank screens based on it's length
          if ( xNeg == 1 ) { r=32; g=0; b=0; }                                        // set the colour to red if the value is negative
          else { r=0; g=32; b=0; }                                                    // otherwise green
          for (int i=0; i<xLen; i++) {                                                // for each digit in the x value, load the display framebuffer with the LED string combination that matches the digit
            if ( xVal[i] == '0' ) { writeNumberToLEDfb(0, i+displayNumberOffset); }
            if ( xVal[i] == '1' ) { writeNumberToLEDfb(1, i+displayNumberOffset); }
            if ( xVal[i] == '2' ) { writeNumberToLEDfb(2, i+displayNumberOffset); }
            if ( xVal[i] == '3' ) { writeNumberToLEDfb(3, i+displayNumberOffset); }
            if ( xVal[i] == '4' ) { writeNumberToLEDfb(4, i+displayNumberOffset); }
            if ( xVal[i] == '5' ) { writeNumberToLEDfb(5, i+displayNumberOffset); }
            if ( xVal[i] == '6' ) { writeNumberToLEDfb(6, i+displayNumberOffset); }
            if ( xVal[i] == '7' ) { writeNumberToLEDfb(7, i+displayNumberOffset); }
            if ( xVal[i] == '8' ) { writeNumberToLEDfb(8, i+displayNumberOffset); }
            if ( xVal[i] == '9' ) { writeNumberToLEDfb(9, i+displayNumberOffset); }
          }

          displayNumberOffset = 7 - yLen;                                             // prefix the Y value with blank screens based on it's length
          if ( yNeg == 1 ) { r=32; g=0; b=0; }                                        // set the colour to red if the value is negative
          else { r=0; g=32; b=0; }                                                    // otherwise green
          for (int i=0; i<yLen; i++) {                                                // for each digit in the x value, load the display framebuffer with the LED string combination that matches the digit
            if ( yVal[i] == '0' ) { writeNumberToLEDfb(0, i+displayNumberOffset); }
            if ( yVal[i] == '1' ) { writeNumberToLEDfb(1, i+displayNumberOffset); }
            if ( yVal[i] == '2' ) { writeNumberToLEDfb(2, i+displayNumberOffset); }
            if ( yVal[i] == '3' ) { writeNumberToLEDfb(3, i+displayNumberOffset); }
            if ( yVal[i] == '4' ) { writeNumberToLEDfb(4, i+displayNumberOffset); }
            if ( yVal[i] == '5' ) { writeNumberToLEDfb(5, i+displayNumberOffset); }
            if ( yVal[i] == '6' ) { writeNumberToLEDfb(6, i+displayNumberOffset); }
            if ( yVal[i] == '7' ) { writeNumberToLEDfb(7, i+displayNumberOffset); }
            if ( yVal[i] == '8' ) { writeNumberToLEDfb(8, i+displayNumberOffset); }
            if ( yVal[i] == '9' ) { writeNumberToLEDfb(9, i+displayNumberOffset); }
          }
          strip.show();
}

void writeOutToSerial() {
          Serial.printf("x:");
          if ( xNeg == 1 ) { Serial.printf("-"); }   // Prefix the number with a minus sign if it was deemed to be negative
          for (int i=0; i<xLen; i++) {
            Serial.write(xVal[i]);
          }    

          Serial.printf(" y:");
          if ( yNeg == 1 ) { Serial.printf("-"); }    // Prefix the number with a minus sign if it was deemed to be negative
          for (int i=0; i<yLen; i++) {
            Serial.write(yVal[i]);
          }
          Serial.printf(" z:");
          if ( zNeg == 1 ) { Serial.printf("-"); }    // Prefix the number with a minus sign if it was deemed to be negative
          for (int i=0; i<zLen; i++) {
            Serial.write(zVal[i]);
          }
          Serial.printf("\n");
          lastSerialUpdate = millis();
}

void extractXposition() {
    xStartPos = 10;                                   // We start with character position 10 in the string
    if ( buf[xStartPos] == '-' ) {                    // And if it's minus sign we note that in xNeg and bump the start position one to the right
      xNeg=1;
      xStartPos = 11;
      }
    else {
      xNeg=0;
    }
    posCounter = 0;
    matchAlreadyFound = 0;
    while ((posCounter < numOfBytesRead) && (matchAlreadyFound == 0)) {
      if (buf[posCounter] == '.') { xStopPos = posCounter; matchAlreadyFound = 1; }   // The xStopPos in the buffer is the char immediately before the first decimal point
      posCounter++;
    }
      xLen = xStopPos - xStartPos;                                                    // Calculate the number of x position digits
}
  
  
  
void extractYposition() {
    posCounter = 0;
    matchAlreadyFound = 0;
    while ((posCounter < numOfBytesRead) && (matchAlreadyFound < 2 )) {               // The yStartPos position in the buffer is immediately after the 2nd 
      if (buf[posCounter] == ',') {                                                   // comma
        yStartPos = posCounter+1;                                         
        matchAlreadyFound++;
      }
      posCounter++;
    }
  
    if ( buf[yStartPos] == '-' ) {                                                    // Check if the Y value is negative and adjust the start position accordingly if it is
      yNeg=1;
      yStartPos++;
    }
    else {
      yNeg=0;
    }
  
    posCounter = 0;
    matchAlreadyFound = 0;
    while ((posCounter < numOfBytesRead) && (matchAlreadyFound < 2)) {               // The yStopPos position in the buffer is immediately before the 2nd decimal point
      if (buf[posCounter] == '.') { yStopPos = posCounter; matchAlreadyFound++; }
      posCounter++;
    }
  
    yLen = yStopPos - yStartPos;                                                      // Calculate the number of y position digits
}
  
void extractZposition() {                                                             // Determine the character position in the buffer for the start of the Z axis position
    posCounter = 0;
    matchAlreadyFound = 0;
    while ((posCounter < numOfBytesRead) && (matchAlreadyFound < 3 )) {               // The zStartPos position in the buffer is immediately after the third comma
      if (buf[posCounter] == ',') {
        zStartPos = posCounter+1;                                         
        matchAlreadyFound++;
      }
      posCounter++;
    }
  
    if ( buf[zStartPos] == '-' ) {                                                    // Check if the Z value is negative and adjust the start position accordingly if it is 
      zNeg=1;
      zStartPos++;
    }
    else {
      zNeg=0;
    }
    
    posCounter = 0;
    matchAlreadyFound = 0;
    while ((posCounter < numOfBytesRead) && (matchAlreadyFound < 3)) {               // The zStopPos position in the buffer is immediately before the 3rd decimal point
      if (buf[posCounter] == '.') { zStopPos = posCounter; matchAlreadyFound++; }
      posCounter++;
    }
    zLen = zStopPos - zStartPos;                                                      // Calculate the number of z position digits
    zHundrethsStartPos = posCounter;
}


void writeNumberToLEDfb(int digit, int displayNumber) {                               // Function to write our decoded digits to the display framebuffer using our digit lookup tables
      int displayOffset = displayNumber * LEDsPerDisplay;
      if ( digit == 0 ) { for (int i=0; i<(sizeof(digitZero)/sizeof(int)); i++) { strip.setPixelColor(((digitZero[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 1 ) { for (int i=0; i<(sizeof(digitOne)/sizeof(int)); i++) { strip.setPixelColor(((digitOne[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 2 ) { for (int i=0; i<(sizeof(digitTwo)/sizeof(int)); i++) { strip.setPixelColor(((digitTwo[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 3 ) { for (int i=0; i<(sizeof(digitThree)/sizeof(int)); i++) { strip.setPixelColor(((digitThree[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 4 ) { for (int i=0; i<(sizeof(digitFour)/sizeof(int)); i++) { strip.setPixelColor(((digitFour[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 5 ) { for (int i=0; i<(sizeof(digitFive)/sizeof(int)); i++) { strip.setPixelColor(((digitFive[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 6 ) { for (int i=0; i<(sizeof(digitSix)/sizeof(int)); i++) { strip.setPixelColor(((digitSix[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 7 ) { for (int i=0; i<(sizeof(digitSeven)/sizeof(int)); i++) { strip.setPixelColor(((digitSeven[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 8 ) { for (int i=0; i<(sizeof(digitEight)/sizeof(int)); i++) { strip.setPixelColor(((digitEight[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 9 ) { for (int i=0; i<(sizeof(digitNine)/sizeof(int)); i++) { strip.setPixelColor(((digitNine[i]-1) + displayOffset), r,g,b); } }
      if ( digit == 10 ) { for (int i=0; i<(sizeof(decimalPoint)/sizeof(int)); i++) { strip.setPixelColor(((decimalPoint[i]-1) + displayOffset), r,g,b); } }
}
