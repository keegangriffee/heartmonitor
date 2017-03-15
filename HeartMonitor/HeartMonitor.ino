// Keegan Griffee #1432621
// Ethan Mayer #1168278
// Lab 8: HeartMonitor Updated
// Uses an amplifier circuit and lcd interface to
// display an electrocardiograph to the user.
// Also every 30 seconds of continuous runtime
// will be saved to an SD card (if available) with the output
// of the ADC that is used to construct the trace.
////////////////////// UPDATES ////////////////////////////////////
// The heart data is now digitally filtered for a cleaner signal
// Now allows SD card data readback and ability to
// scan forwards and backwards through the HRM trace on the display
// Also now has Bluetooth capabilities to display BPM on your phone
// Data written to the SD card will include BPM and the arythmias
// detected during a 30 sec recording
// Arythmias detected, BPM, and QRS are now displayed on the ECG
// while it actively records

// Define pins for using both the display and SD card
#define SD_CS 10
#define TFT_DC  9
#define TFT_CS 20

#include <Arduino.h>
#include <stdlib.h>
#include <string.h> // For memset
#include <Filters.h> // For Digital Filtering to clean up signal
#include <SPI.h>

// Bluetooth libraries and serial printing
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* The service information */
int32_t hrmServiceId;
int32_t hrmMeasureCharId;
int32_t hrmLocationCharId;

///////////////////////////////////////////////////////////
// Display and SD card setup
///////////////////////////////////////////////////////////
#include "SPI.h"
#include "ILI9341_t3.h"

// Uses the SdFat library
#include "SdFat.h"
SdFat sd;
SdFile myFile;
File root;

// Used to determine if we should write to an SD card
boolean hasSDcard = false;

// Setup the display
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

// PDB related defines
#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

////////////////////////////////////////////
// Button definitions for interaction
////////////////////////////////////////////
#define BSTART 23  // Start/Stop Button
#define BDOWN 21  // Scroll Left/Back
#define BUP 22  // Scroll right/up option

/////////////////////////////////////////////
// Bluetooth Configuration
/////////////////////////////////////////////
// Enable Bluetooth when == 1
int enableBluetooth = 0;

////////////////////////////////////////////
// Sample Rate
////////////////////////////////////////////
const int sRate = 250;  // 250 Hz sample rate

////////////////////////////////////////////
// Playback related flags and enabling
////////////////////////////////////////////
int playBackMode = 0;  // Enabled when == 1
uint16_t currPlayBackSamp = 0;
int totPlayBack = 0;
int playBackSamples = sRate * 31;  // For 30 sec of play back

////////////////////////////////////////////
// Sample buffers
////////////////////////////////////////////
// Display tLen seconds of data on display
const int tLen = 2;

// Heartbeat detection and QSR detection related
uint32_t derBuf[sRate * tLen];
int derSamp = 0;
#define THRESHOLD 4500
#define THRESHOLDS 50
#define NUMBEATS 3
#define NUMQRSAVG 4
#define MINWAIT 200  // Not sensing heartbeats above 150
#define MAXWAIT 2  // Max amount of time to wait for next heartbeat (scaler for bps)
#define MAXSSAMP 20  // Max samples to wait before being certain of an S wave
#define MAXQSAMP 15  // Max samples to searh behind for start of Q wave
#define BEATCAL 5    // Number of valid bps's to wait for
#define MAXTACHY 4   // Number of tachy's before tachy trigger
#define MAXBRADY 4   // Number of brady's before brady trigger
#define MAXPAC 4     // Max number of rapid heartbeat changes before PAC
#define MAXPACDIF 0.15  // Max percent change for PAC

// Holds the time of the last peak
int lastPeak = 0;
int currBeat = 0;
int currQRS = 0;

// Holds the current calibration bpss
int calCount = 0;

// Hold data for current time of QRS edges
int qStart = -1;
int sStart = -1;
int sSamples = 0;
float qrsTime = 0;

// Heart problem variables
boolean hasBrady = false;
boolean hasTachy = false;
boolean hasPAC = false;
int currPAC = 0;
int currBrady = 0;
int currTachy = 0;

// Data buffers for storing the
// captured data by the
// PDB triggered ADC
uint16_t samples[sRate * tLen];
uint16_t data[sRate * 31];
// Buffer for storing a recalled
// SD card file of heart rate data
uint16_t playBack[sRate * 31];
// Keep track of samples and current file
// we are saving to
int currSamp = 0;
int totSamp = 0;
int printSamp = 0;
int currFile = 0;

// If we are actively running
boolean isRun;
// Is calibrating at start
boolean isCal = true;
// Is it currently writing to the SD card
boolean isWriting = false;
// Have we attempted to write to the SD card
boolean hasWritten = false;
// Used to store the value of reading buttons
int but;
// The previous state of the system/button press
// for switch debouncing
int prev;

// Filter for baseline wander
#define BASELINE 1800
float bwFreq = 0.5;
FilterOnePole bw(HIGHPASS, bwFreq);

// Filter for high frequency noise
float nFreq = 20;
FilterOnePole nf(LOWPASS, nFreq);

// Screen Constants
#define WIDTH 320
#define HEIGHT 240
#define BOXW 20
#define HLINES (HEIGHT / BOXW)
#define VLINES (WIDTH / BOXW)

// Used for autostopping after 30 seconds of runtime
// and other various timed delays waiting for actions
int startTime;

// Frame numbers for following the flow of the trace
// and actively running screen
int frameNum = 0;

// Used for informing us when to
// clear and redraw the display
boolean drawGraph = true;

// Used for storing and displaying the BPM
// and QRS variables
int lastBeat = -1;
float bps;
int lastbps;
float lastQRS;

//////////////////////////////////////////////////////
// SETUP Initialization
//////////////////////////////////////////////////////
// Asks the user for the which mode to run in and
// whether to enable bluetooth and initializing the
// SD card and display
void setup() {
  Serial.begin(9600);
  // Don't begin until Serial is active
  while (!Serial) {}
  // Setup the pin for capturing heartbeat data
  pinMode(A3, INPUT);
  // Pin for start/stop button
  pinMode(BSTART, INPUT_PULLUP);
  pinMode(BUP, INPUT_PULLUP);
  pinMode(BDOWN, INPUT_PULLUP);
  pinMode(SD_CS, OUTPUT);
  prev  = HIGH;
  // The starting state of running/stopping
  isRun = digitalRead(BSTART);

  // Initialize the SD card
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) {
    // Either an error or no SD card was present
    Serial.println("No SD card detected, proceeding...");
  } else {
    // Success
    hasSDcard = true;
    Serial.println("Initialized SD card");
  }

  tft.begin();
  // Prompt user to select playback or record mode
  chooseMode();
  if (playBackMode == 0) {
    Serial.println("Recording Mode Selected");
  } else {
    Serial.println("Play Back Mode Selected");
  }
  if (!playBackMode) {
    // Ask for bluetooth
    enableBluetooth = askBluetooth();
    if (enableBluetooth) {
      // Initialize Bluetooth
      initializeBluetooth();
    }
    // Begin collecting data
    adcInit();
    pdbInit();
    tft.begin();
    // Halt until the reading is stabalized
    calibrateMonitor();
    drawGrid();
    tft.begin();
  } else {
    // Read from the SD card for playback
    char fBuffer[13];
    // Get the file name from file navigation
    selectFile(fBuffer);
    Serial.print("Reading file: ");
    Serial.println(fBuffer);
    readSD(fBuffer);
  }
}

////////////////////////////////////////////////////
// Select Mode of Operation
////////////////////////////////////////////////////
// Choose either playback mode replaying heart rate
// monitor data from a previous ran from an SD card
// or record mode to display and save the data to
// an SD card
int modeSelect = 0;
void chooseMode() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(0);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_BLACK);
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("Select Mode:");
  while (!modeSelect) {
    if (modeSelect) {
      return;
    }
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(20, 140);
    tft.print("Record [Y]");
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(20, 140);
    tft.print("Playback [Y]");
    delay(500);
    while (!modeSelect && digitalRead(BUP) && digitalRead(BDOWN)) {
      if (!digitalRead(BSTART)) {
        modeSelect = 1;
        playBackMode = 1;
      }
    }
    if (modeSelect) {
      return;
    }
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(20, 140);
    tft.print("Playback [Y]");
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(20, 140);
    tft.print("Record [Y]");
    delay(500);
    while (!modeSelect && digitalRead(BUP) && digitalRead(BDOWN)) {
      if (!digitalRead(BSTART)) {
        modeSelect = 1;
        playBackMode = 0;
      }
    }
  }
}

///////////////////////////////////////////////
// Enable Bluetooth Prompt
///////////////////////////////////////////////
// Prompt to enable bluetooth
// User either selects yes or no
int askBluetooth() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(0);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_BLACK);
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("Bluetooth:");
  while (1) {
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(20, 140);
    tft.print("OFF [Y]");
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(20, 140);
    tft.print("ON [Y]");
    delay(500);
    while (digitalRead(BUP) && digitalRead(BDOWN)) {
      if (!digitalRead(BSTART)) {
        return 1;
      }
    }
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(20, 140);
    tft.print("ON [Y]");
    tft.setTextColor(ILI9341_BLACK);
    tft.setCursor(20, 140);
    tft.print("OFF [Y]");
    delay(500);
    while (digitalRead(BUP) && digitalRead(BDOWN)) {
      if (!digitalRead(BSTART)) {
        return 0;
      }
    }
  }
}

////////////////////////////////////////////////////
// Select File to Open
////////////////////////////////////////////////////
// Allows the user to navigate through valid heart
// rate data files on the SD card and select one
// to replay on the monitor
void selectFile(char* fBuffer) {
  printDirectory();
  tft.fillScreen(ILI9341_WHITE);
  tft.setRotation(0);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLACK);
  tft.setCursor(20, 40);
  tft.print("Press buttons to");
  tft.setCursor(20, 60);
  tft.print("navigate through");
  tft.setCursor(20, 80);
  tft.print("files to open");
  tft.setCursor(20, 100);
  tft.print("File Name: ");
  tft.setCursor(20, 200);
  tft.print("Hold Start To");
  tft.setCursor(20, 220);
  tft.print("Continue");
  tft.setCursor(60, 130);
  int accept = 0;
  sd.vwd()->rewind();
  // Run until the user chooses a file name
  // from the SD card
  while (!accept) {
    // Get the next valid heart rate data file
    // on the SD card
    if (!digitalRead(BUP)) {
      bool flag = false;
      if (myFile.openNext(sd.vwd(), O_READ)) {
        // Ignore hidden files
        while (myFile.isHidden()) {
          myFile.close();
          if (!myFile.openNext(sd.vwd(), O_READ)) {
            // Used to tell if at end of valid files
            flag = true;
            break;
          }
        }
        // Prevents user from going past valid files
        if (!flag) {
          tft.setTextColor(ILI9341_WHITE);
          tft.setCursor(60, 130);
          tft.print(fBuffer);
          tft.setCursor(60, 130);
          tft.setTextColor(ILI9341_BLACK);
          myFile.getName(fBuffer, 13);
          // Checks the validity of the file name
          // then displays it to the user
          if (fBuffer[10] == 't') {
            tft.print(fBuffer);
            myFile.close();
          }
        }
      }
      delay(250);
    }
    // Rewind to start of file names
    if (!digitalRead(BDOWN)) {
      sd.vwd()->rewind();
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(60, 130);
      tft.print(fBuffer);
      delay(250);
    }
    // Accept currently shown file
    // by holding the start button
    // for a long enough period of time
    if (!digitalRead(BSTART)) {
      int butCount = 0;
      while (!digitalRead(BSTART)) {
        butCount++;
        if (butCount > 350000) {
          accept = 1;
          break;
        }
      }
    }
  }
}

////////////////////////////////////////////////////
// Directory Printing
////////////////////////////////////////////////////
// Used for debugging which files are on the SD card
// and printing them to the serial console
void printDirectory() {
  Serial.println("The following are files on the SD Card: ");
  ArduinoOutStream cout(Serial);
  sd.vwd()->rewind();
  while (myFile.openNext(sd.vwd(), O_READ)) {
    if (!myFile.isHidden()) {
      myFile.printName(&Serial);
      cout << endl;
    }
    myFile.close();
  }
  cout << "\nAll files listed!" << endl;
}

///////////////////////////////////////////////
// Draws the grid
///////////////////////////////////////////////
// Clears the display and draws the
// red checkered grid background for the ECG
// and the current frame number to know how many
// frames have been displayed thus far
void drawGrid() {
  // Clear starting derivative buffer for Heart Beat detection
  for (int i = 0; i < 5; i++) {
    derBuf[i] = 0;
  }
  // Draws the grid
  tft.fillScreen(ILI9341_WHITE);
  for (int i = 0; i < HLINES; i++) {
    if (i % 6 == 0) {
      for (int j = 0; j < 3; j++) {
        tft.drawLine(i * BOXW + j, 0, i * BOXW + j, WIDTH, ILI9341_RED);
      }
    } else {
      tft.drawLine(i * BOXW, 0, i * BOXW, WIDTH, ILI9341_RED);
    }
  }
  for (int i = 0; i < VLINES; i++) {
    if (i % 6 == 0) {
      for (int j = 0; j < 3; j++) {
        tft.drawLine(0, i * BOXW + j, HEIGHT, i * BOXW + j, ILI9341_RED);
      }
    } else {
      tft.drawLine(0, i * BOXW, HEIGHT, i * BOXW, ILI9341_RED);
    }
  }
  tft.setTextColor(ILI9341_BLACK);
  tft.setRotation(0);
  tft.setTextSize(3);
  tft.setCursor(200, 20);
  tft.print(frameNum);
}

///////////////////////////////////////////
// Calibrates the HRM
///////////////////////////////////////////
// Prevents data saving to buffer until the input from the
// cardiograph is stabalized
void calibrateMonitor() {
  isCal = true;
  int val;
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(0);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("Calibrating");
  Serial.println("Calibrating");
  int startCal = millis();
  // Finish calibrating when the input signal is steady
  // and in a valid range for 5 continuous seconds
  while (millis() - startCal < 5000) {
    val = samples[currSamp];
    if (val < 400 || val > 3800) {
      startCal = millis();
      // Inform the user they need to stay
      // still because they are moving too much
      tft.setCursor(20, 200);
      tft.setTextColor(ILI9341_WHITE);
      tft.print("Stay Still");
    }
    // Remove the stay still if the user
    // remains still
    if (millis() - startCal > 2000) {
      tft.setCursor(20, 200);
      tft.setTextColor(ILI9341_BLACK);
      tft.print("Stay Still");
    }
  }
  // After calibration is complete set all initial
  // variables to zero and false for a new data run
  calCount = 0;
  currSamp = 0;
  totSamp = 0;
  isCal = false;
  isRun = true;
  hasTachy = false;
  hasBrady = false;
  hasPAC = false;
  currPAC = 0;
  derSamp = 0;
  frameNum = 0;
  bps = 70.0 / 60.0;
  Serial.println("Calibrated");
  startTime = millis();
}

//////////////////////////////////////////////////////////////
// Main Loop
//////////////////////////////////////////////////////////////
// Displays the trace of the cardiograph
// and determines whether we are in a running state
// or stopped state
// Also display the QRS, BPM, and whether we have
// detected tachycardia, bradycardia, or PAC
// Clear the screen when the trace is at the end of the screen
// and begin displaying a new trace
void loop() {
  // Read button value
  but = digitalRead(BSTART);
  // Detect start/stop button and use state transitions
  // to switch debounce
  if (but == LOW && prev == HIGH) {
    isRun = !isRun;
    if (isRun) {
      // About to go into the running state
      // so calibrate and reset counters for
      // storing data and calculating BPM and QRS
      if (!playBackMode) {
        calibrateMonitor();
        hasWritten = false;
        currSamp = 0;
        derSamp = 0;
        totSamp = 0;
        startTime = millis();
      }
    }
  }
  prev = but;
  if (!isRun) {
    // Stop display
    if (playBackMode) {
      // Contol the playback of the screen with 
      // forward and backward buttons
      // and a button to resume the automatic playback
      // Scroll back a screen/frame of data
      if (!digitalRead(BDOWN)) {
        currPlayBackSamp = 0;
        // Goes back one frame of data
        totPlayBack -= (sizeof(samples) / 2);
        frameNum--;
        if (totPlayBack <= 0) {
          frameNum = 0;
          totPlayBack = 0;
        }
        drawGrid();
        Serial.println(totPlayBack);
        Serial.println(currPlayBackSamp );
        delay(500);
      }
      // Scroll forward on current data
      if (!digitalRead(BUP)) {
        currPlayBackSamp = (currPlayBackSamp + 1) % (sizeof(samples) / 2);
        if (currPlayBackSamp == 0) {
          totPlayBack += (sizeof(samples) / 2);
          frameNum++;
          drawGrid();
        } else if (totPlayBack < playBackSamples) {
          // Draw the next data points of the recalled heart monitor data
          tft.drawLine((int)((HEIGHT * playBack[currPlayBackSamp + totPlayBack - 1] / 3095.0)), currPlayBackSamp * ((double)WIDTH / (sRate * tLen)),
                       (int)((HEIGHT * playBack[currPlayBackSamp + totPlayBack] / 3095.0)), (currPlayBackSamp - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLUE);
        } else {
          // Rollback to beginning of data since we reached the end of the replay
          totPlayBack = 0;
          currPlayBackSamp = 1;
        }
        delay(10);
      }
      // Resume automatic playback
      but = digitalRead(BSTART);
      if (but == LOW && prev == HIGH) {
        isRun = true;
      }
      prev = but;
    }
  } else {
    // Actively running
    // Refresh the graph, BPM and QRS detection on rollaround
    if (drawGraph && !playBackMode) {
      lastPeak = -1;
      qStart = -1;
      sStart = -1;
      derSamp = 0;
      drawGrid();
      frameNum++;
      drawGraph = false;
    }

    // Write to our display the trace
    // scaled to emphasize our valid range
    if (playBackMode) {
      for (uint16_t i = currPlayBackSamp; i < (sizeof(samples) / 2); i++) {
        currPlayBackSamp = (currPlayBackSamp + 1) % (sizeof(samples) / 2);
        if (i != 0) {
          tft.drawLine((int)((HEIGHT * playBack[i + totPlayBack] / 3095.0)), i * ((double)WIDTH / (sRate * tLen)),
                       (int)((HEIGHT * playBack[i - 1 + totPlayBack] / 3095.0)), (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLUE);
          uint16_t delayWrite = millis();
          // Slows how fast the data is written
          // and checks to see if the user wants to return to manually
          // scanning through the data
          while (millis() - delayWrite < 10) {
            but = digitalRead(BSTART);
            if (but == LOW && prev == HIGH) {
              isRun = !isRun;
            }
            prev = but;
            if (!isRun) {
              break;
            }
          }
          if (!isRun) {
            break;
          }
        } else {
          totPlayBack += (sizeof(samples) / 2);
          frameNum++;
          if (totPlayBack > playBackSamples) {
            isRun = false;
            totPlayBack = 0;
            frameNum = 0;
            currPlayBackSamp = 0;
          }
          drawGrid();
        }
      }
    } else {
      // Actively recording the data
      // and displaying the trace of the heart rate data
      // Print the updating bpm, QRS
      // and detection of PAC, tachycardia or bradycardia
      tft.setTextColor(ILI9341_PURPLE);
      tft.setTextSize(2);
      tft.setCursor(5, 220);
      tft.print("bpm: ");
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(60, 220);
      tft.print(lastbps);
      tft.setTextColor(ILI9341_PURPLE);
      tft.setCursor(60, 220);
      int printbps = (int)(bps * 60);
      tft.print(printbps);
      lastbps = printbps;
      // Print the updating QRS
      tft.setCursor(5, 240);
      tft.print("QRS: ");
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(60, 240);
      tft.print(lastQRS);
      tft.setTextColor(ILI9341_PURPLE);
      tft.setCursor(60, 240);
      float printQRS = qrsTime;
      tft.print(printQRS);
      lastQRS = printQRS;
      tft.setCursor(5, 260);
      if (hasTachy) {
        tft.print("Tachy: X");
      } else {
        tft.print("Tachy: ");
      }
      tft.setCursor(5, 280);
      if (hasBrady) {
        tft.print("Brady: X");
      } else {
        tft.print("Brady: ");
      }
      tft.setCursor(5, 300);
      if (hasPAC) {
        tft.print("PAC: X");
      } else {
        tft.print("PAC: ");
      }
      for (int i = printSamp; i < currSamp; i++) {
        if (i != 0) {
          tft.drawLine((int)((HEIGHT * samples[i] / 3095.0)), i * ((double)WIDTH / (sRate * tLen)),
                       (int)((HEIGHT * samples[i - 1] / 3095.0)), (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLACK);
        }
      }
      // Get the BPM data, along with QRS, tachycardia and bradycardia
      calculatebps();
      // Automatically stop the data capture and freeze the display while
      // the data is saved to the SD card if a SD card was detected
      // and put into a non-running state
      if (millis() - startTime >= 30 * 1000) {
        lastBeat = 0;
        isRun = false;
        writeToSD();
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////
// Calculate BPM, Arythmias and QRS
////////////////////////////////////////////////////////////////////////////
// Calculates the beats per second which is converted to BPM throughout the 
// program 
// Different arythmias are also detected based on the BPM
void calculatebps() {
  if (currSamp > 3) {
    for (int i = derSamp; i < currSamp; i++) {
      uint16_t *currBuf;
      float curDer = 0;
      currBuf = samples;

      curDer = 0.1 * (2 * currBuf[i] + currBuf[i - 1] - currBuf[i - 3] - (2 * currBuf[i - 4]));
      derBuf[i] = pow(curDer, 2);

      // Test for heartbeats
      int timeDif = i - lastPeak;
      int valToAdd = -1;
      if (derBuf[i] > THRESHOLD && timeDif > MINWAIT) {
        valToAdd = timeDif;
      } else if ((derBuf[i] < THRESHOLD &&  bps != 0 && timeDif > MAXWAIT * sRate / bps) ||
                 (lastPeak == -1 && i == sRate * tLen - 15)) {
        valToAdd = 0;
      }

      if (valToAdd != -1) {
        float prevbps = bps;
        if (currBeat < NUMBEATS) {
          bps *= currBeat;
          currBeat++;
          bps += valToAdd * (1.0 / sRate);
          bps /= currBeat;
        } else {
          bps = ((bps * NUMBEATS) + (valToAdd  * (1.0 / sRate)) - bps) / NUMBEATS;
        }
        if (bps * 60 >= 100) {
          currTachy++;
          if (currTachy >= MAXTACHY) {
            hasTachy = true;
          }
        } else if (bps * 60 <= 60) {
          currBrady++;
          if (currBrady >= MAXBRADY) {
            hasBrady = true;
          }
        }

        if ((bps - prevbps) / bps >= MAXPACDIF) {
          currPAC++;
          if (currPAC >= MAXPAC) {
            hasPAC = true;
          }
        }
        if (isCal) {
          calCount++;
        }
        Serial.print(F("Updating HRM value to "));
        Serial.print((int)(bps * 60));
        Serial.println(F(" bps"));
        // Display the BPM on a bluetooth device if Bluetooth is enabled
        if (enableBluetooth) {
          /* Command is sent when \n (\r) or println is called */
          /* AT+GATTCHAR=CharacteristicID,value */
          ble.print( F("AT+GATTCHAR=") );
          ble.print( hrmMeasureCharId );
          ble.print( F(",00-") );
          ble.println((int)(bps * 60), HEX);

          /* Check if command executed OK */
          if ( !ble.waitForOK() )
          {
            Serial.println(F("Failed to get response!"));
          }
        }
        lastPeak = i;
      }
      // Test for QRS start/end
      if (!isCal) {
        if (qStart == -1 && i >= MAXQSAMP && i <= sRate * tLen - MAXQSAMP) {
          boolean isValidQStart = true;
          for (int j = i - 1; j >= i - MAXQSAMP; j--) {
            if (samples[j] < samples[i]) {
              isValidQStart = false;
            }
          }
          for (int j = i + 1; j < i + MAXQSAMP; j++) {
            if (samples[j] > samples[i]) {
              isValidQStart = false;
            }
          }

          if (isValidQStart) {
            qStart = i;
            sStart = -1;
          }
        } else if (derBuf[i] < THRESHOLDS && qStart != -1 && sStart == -1) {
          // Heartbeat stopped
          sStart = i;
        }
        if (derBuf[i] < THRESHOLDS && qStart != -1 && sStart != -1 && lastPeak > qStart) {
          sSamples++;
          if (sSamples > MAXSSAMP) {
            // S start time is correct
            if (currQRS < NUMQRSAVG) {
              qrsTime *= currQRS;
              currQRS++;
              qrsTime += (sStart - qStart) * (1.0 / sRate);
              qrsTime /= currQRS;
            } else {
              qrsTime = ((qrsTime * NUMQRSAVG) + ((sStart - qStart) * (1.0 / sRate)) - qrsTime) / NUMQRSAVG;
            }

            qStart = -1;
            sStart = -1;
            sSamples = 0;
          }
        }
      }
      // Update the derivative samples
      derSamp = currSamp - 1;
    }
  }
}

////////////////////////////////////////////////////////////////
// SD CARD RECALL
////////////////////////////////////////////////////////////////
// Load the data from a previous HRM run from an SD card
int curPlayBack = 0;
// Read the HRM data to redisplay on the screen
File file;
void readSD(char *fn) {
  file = sd.open(fn, FILE_READ);
  if (!file) {
    Serial.println("open failed");
    return;
  }
  Serial.println("open success");
  // Start at the beginning of the data after the naming and
  // sample rate information
  file.seek(10);
  uint16_t t1;
  // Use space seperated list to grab the data values
  // and put them into our replay data buffer
  while (file.available() && curPlayBack < playBackSamples) {
    if (csvReadUint16(&file, &t1, ' ') == ' ') {
      playBack[curPlayBack] = t1;
      curPlayBack++;
    }
  }
  file.close();
  curPlayBack = 0;
}

// The following are helper methods for processing the space
// seperated data values from the SD card HRM files to
// read as valid displayable data
int csvReadUint32(File * file, uint32_t* num, char delim) {
  char buf[20];
  char* ptr;
  int rtn = csvReadText(file, buf, sizeof(buf), delim);
  if (rtn < 0) return rtn;
  *num = strtoul(buf, &ptr, 10);
  if (buf == ptr) return -3;
  while (isspace(*ptr)) ptr++;
  return *ptr == 0 ? rtn : -4;
}
// More helper functions
int csvReadUint16(File * file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}
// Additional helper function to process the data back into
// a replayable trace of the HRM
int csvReadText(File * file, char* str, size_t size, char delim) {
  char ch;
  int rtn;
  size_t n = 0;
  while (true) {
    // check for EOF
    if (!file->available()) {
      rtn = 0;
      break;
    }
    if (file->read(&ch, 1) != 1) {
      // read error
      rtn = -1;
      break;
    }
    // Delete CR.
    if (ch == '\r') {
      continue;
    }
    if (ch == delim || ch == '\n') {
      rtn = ch;
      break;
    }
    if ((n + 1) >= size) {
      // string too long
      rtn = -2;
      n--;
      break;
    }
    str[n++] = ch;
  }
  str[n] = '\0';
  return rtn;
}

///////////////////////////////////////////////////////////////
// SD Card Writing and Naming Generation
///////////////////////////////////////////////////////////////
// Write the file name as KGEMXXX.txt with increasing file number
// Write the filename, sample rate, HRM data, BPM, and arythmias
// detected during the recording to the SD card
void writeToSD() {
  isRun = false;
  // Prevent writing multiple times in one run or while writing
  if (!hasWritten && !isWriting) {
    isWriting = true;
    // Write to the SD if it is detected
    if (hasSDcard) {
      char file[12] = "KGEM";
      char buf[8] = "KGEM";
      fileGen(buf + 4, currFile);
      fileGen(file + 4, currFile);
      file[7] = '.';
      file[8] = 't';
      file[9] = 'x';
      file[10] = 't';
      file[11] = '\0';
      // Delete the old version of the file if one with the
      // same name already existed
      sd.remove(file);
      if (!myFile.open(file, O_WRITE | O_CREAT)) {
        sd.errorHalt("opening sdcard for write failed");
      }
      Serial.println("opened sd card");
      // Write the header
      myFile.print(buf);
      myFile.print(",");
      myFile.println(sRate);
      // Write the data buffer to the file
      // as space seperated data
      int i = 0;
      while (i < totSamp / 8) {
        for (int j = 0; j < 8; j++) {
          myFile.print(data[8 * i + j]);
          myFile.print(" ");
        }
        myFile.println();
        i++;
      }
      myFile.print("Average BPM: ");
      myFile.println((int)(bps * 60));
      if (hasBrady) {
        myFile.println("Bradycardia detected");
      }
      if (hasTachy) {
        myFile.println("Tachycardia detected");
      }
      if (hasPAC) {
        myFile.println("Premature Atrial Contraction (PAC) detected");
      }
      // Print EOF
      myFile.println("EOF");
      // close the file:
      myFile.close();
      currFile++;
      Serial.println("done.");
    } else {
      // Skip saving data to SD
      Serial.println("No SD card present, skip saving data");
    }
    // Finished writing logic
    hasWritten = true;
    // Reset data buffer
    currSamp = 0;
    totSamp = 0;
    isWriting = false;
  }
}

// Used to generate increasing file name numbering
// to our SD card
void fileGen(char * buf, int i) {
  // put your main code here, to run repeatedly:
  if (i < 1000) {
    int currSpot = 0;
    if (i < 100) {
      buf[currSpot] = '0';
      currSpot++;

      if (i < 10) {
        buf[currSpot] = '0';
        currSpot++;
      } else {
        buf[currSpot] = (i / 10) + '0';
        currSpot++;
        buf[currSpot] = (i % 10) + '0';
      }
    } else {
      buf[currSpot] = (i / 100) + '0';
      currSpot++;
      buf[currSpot] = (i / 10 % 10) + '0';
      currSpot++;
    }
    buf[currSpot] = (i % 10) + '0';
    currSpot++;
    buf[currSpot] = '\0';
    i++;
  }
}

///////////////////////////////////////////////////
// PDB and ADC Configuration and ISR's
//////////////////////////////////////////////////
static const uint8_t channel2sc1a[] = {
  5, 14, 8, 9, 13, 12, 6, 7, 15, 4,
  0, 19, 3, 21, 26, 22
};

/*
  ADC_CFG1_ADIV(2)         Divide ratio = 4 (F_BUS = 48 MHz => ADCK = 12 MHz)
  ADC_CFG1_MODE(2)         Single ended 10 bit mode
  ADC_CFG1_ADLSMP          Long sample time
*/
#define ADC_CONFIG1 (ADC_CFG1_ADIV(1) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP)

/*
  ADC_CFG2_MUXSEL          Select channels ADxxb
  ADC_CFG2_ADLSTS(3)       Shortest long sample time
*/
#define ADC_CONFIG2 (ADC_CFG2_MUXSEL | ADC_CFG2_ADLSTS(3))

void adcInit() {
  ADC0_CFG1 = ADC_CONFIG1;
  ADC0_CFG2 = ADC_CONFIG2;
  // Voltage ref vcc, hardware trigger, DMA
  ADC0_SC2 = ADC_SC2_REFSEL(0) | ADC_SC2_ADTRG | ADC_SC2_DMAEN;

  // Enable averaging, 4 samples
  ADC0_SC3 = ADC_SC3_AVGE | ADC_SC3_AVGS(0);

  adcCalibrate();

  // Enable ADC interrupt, configure pin
  ADC0_SC1A = ADC_SC1_AIEN | channel2sc1a[3];
  NVIC_ENABLE_IRQ(IRQ_ADC0);
}

// Calibrated to be 12 bit resolution for the ADC
void adcCalibrate() {
  uint16_t sum;

  // Begin calibration
  ADC0_SC3 = ADC_SC3_CAL;
  // Wait for calibration
  while (ADC0_SC3 & ADC_SC3_CAL);

  // Plus side gain
  sum = ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 + ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
  sum = (sum / 2) | 0x8000;
  ADC0_PG = sum;

  // Minus side gain (not used in single-ended mode)
  sum = ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 + ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
  sum = (sum / 2) | 0x8000;
  ADC0_MG = sum;
}

/*
  PDB_SC_TRGSEL(15)        Select software trigger
  PDB_SC_PDBEN             PDB enable
  PDB_SC_PDBIE             Interrupt enable
  PDB_SC_CONT              Continuous mode
  PDB_SC_PRESCALER(7)      Prescaler = 128
  PDB_SC_MULT(1)           Prescaler multiplication factor = 10
*/
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
                    | PDB_SC_CONT | PDB_SC_PRESCALER(7) | PDB_SC_MULT(1))

// 48 MHz / 128 / 10 / 1 Hz = 37500
// Set to be 250 Hz sample rate
#define PDB_PERIOD (F_BUS / 128 / 10 / sRate)

// PDB initialization to cause the ADC to trigger at a fixed sample rate
void pdbInit() {
  pinMode(13, OUTPUT);

  // Enable PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  // Timer period
  PDB0_MOD = PDB_PERIOD;
  // Interrupt delay
  PDB0_IDLY = 0;
  // Enable pre-trigger
  PDB0_CH0C1 = PDB_CH0C1_TOS | PDB_CH0C1_EN;
  // PDB0_CH0DLY0 = 0;
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  // Software trigger (reset and restart counter)
  PDB0_SC |= PDB_SC_SWTRIG;
  // Enable interrupt request
  NVIC_ENABLE_IRQ(IRQ_PDB);
}

// Handles getting the ADC reading and saving it to the buffer
// if we are actively running or calibrating
void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF;
}

void adc0_isr() {
  if (isCal || isRun) {
    // Get the ADC value, don't permanently save it if calibrating
    samples[currSamp] = nf.input(bw.input(ADC0_RA)) + BASELINE;
    data[totSamp] = samples[currSamp];
    // If currently running increment the buffer index to save
    // the value to store later on an SD card
    if (isRun) {
      currSamp = (currSamp + 1) % (sizeof(samples) / 2);
      // Flag for knowing when to clear and reset the screen
      // on rollaround in the loop for drawing the heart trace
      if (currSamp == 0) {
        drawGraph = true;
      }
      totSamp++;
    }
  } else {
    // Don't need to use the reading, just read to clear the interrupt flag
    int temp = ADC0_RA;  // Resets the ADCISR flag, preventing infinite loops
  }
}

////////////////////////////////////////////////////
// Bluetooth HRM Setup
////////////////////////////////////////////////////
// Initializes the Bluetooth with a name to search for under KGEM HRM
// Allowing phone applications to display the current BPM of the user
// while the recording data
void initializeBluetooth(void) {
  boolean success;
  Serial.println(F("Adafruit Bluefruit Heart Rate Monitor (HRM) Initializing"));
  Serial.println(F("---------------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    while (1);
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ) {
    Serial.println(F("Couldn't factory reset"));
    while (! ble.factoryReset() ) {
      Serial.println(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'KGEM HRM': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=KGEM HRM")) ) {
    Serial.println(F("Could not set device name?"));
    while (1);
  }

  /* Add the Heart Rate Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Heart Rate Service definition (UUID = 0x180D): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x180D"), &hrmServiceId);
  if (! success) {
    Serial.println(F("Could not add HRM service"));
    while (1);
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
  if (! success) {
    Serial.println(F("Could not add HRM characteristic"));
  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
  if (! success) {
    Serial.println(F("Could not add BSL characteristic"));
  }

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();
}
