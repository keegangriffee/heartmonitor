// Keegan Griffee #1432621
// Ethan Mayer #1168278
// Lab 8: Electrocardiograph Updated
// Uses an amplifier circuit and lcd interface to
// display an electrocardiograph to the user.
// Also every 30 seconds of continuous runtime
// will be saved to an SD card (if available) with the output
// of the ADC that is used to construct the trace.

// TODO: Add method for asking for switching between playback and readback
// TODO: Fix forward and backward scrolling

// Define pins for using both the display and SD card
#define SD_CS 10
#define TFT_DC  9
#define TFT_CS 20

#include <Arduino.h>
#include <stdlib.h>
#include <string.h> // For memset
#include <Filters.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
#include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

//////////////////////////////////////////////////////////
// Bluetooth Configuration
//////////////////////////////////////////////////////////
// Enable Bluetooth when == 1
int enableBluetooth = 0;

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

// The range of the ADC
const float resolution = 4095.0;

// Setup the display
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

////////////////////////////////////////////
// Button definitions
////////////////////////////////////////////
#define BSTART 23  // Start/Stop Button
#define BDOWN 21  // Scroll Left/Back
#define BUP 22  // Scroll right/up option


////////////////////////////////////////////
// Sample Rate
////////////////////////////////////////////
const int sRate = 250;

////////////////////////////////////////////
// PLAYBACK ENABLE
////////////////////////////////////////////
int playBackMode = 0;
int currPlayBackSamp = 0;
int totPlayBack = 0;
int playBackSamples = sRate * 31;
int chooseFile = 0;

////////////////////////////////////////////
// Sample buffers
////////////////////////////////////////////

// Display tLen seconds of data
const int tLen = 2;

//Ethan!!!~!!!!!
const int calLen = 30;
uint16_t calBuf[sRate * calLen];
uint32_t derBuf[sRate * tLen];
int derSamp = 0;
#define THRESHOLD 6000
#define THRESHOLDS 50
#define NUMBEATS 3
#define NUMQRSAVG 4
#define MINWAIT 200  // Not sensing heartbeats above 150
#define MAXWAIT 1.5  // Max amount of time to wait for next heartbeat (scaler for )
#define MAXSSAMP 20  // Max samples to wait before being certain of an S wave
#define MAXQSAMP 15  // Max samples to searh behind for start of Q wave
#define BEATCAL 5    // Number of valid bpm's to wait for
#define MAXPAC 4     // Max number of rapid heartbeat changes before PAC
#define MAXPACDIF 0.15  // Max percent change for PAC

// Holds the time of the last peak
int lastPeak = 0;
int currBeat = 0;
int currQRS = 0;

// Holds the current calibration bpms
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
//ETHAN!@!!!!!

// Data buffer
uint16_t samples[sRate * tLen];
uint16_t data[sRate * 31];
uint16_t playBack[sRate * 31];
// Keep track of samples and files
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
// The previous state of the system
int prev;

// Filter for baseline wander
#define BASELINE 1800
float bwFreq = 0.5;
FilterOnePole bw(HIGHPASS, bwFreq);

// Filter for high frequency noise
float nFreq = 20;
FilterOnePole nf(LOWPASS, nFreq);

// Define constants
#define WIDTH 320
#define HEIGHT 240
#define BOXW 20
#define HLINES (HEIGHT / BOXW)
#define VLINES (WIDTH / BOXW)

// Used for autostopping after 30 seconds of runtime
int startTime;

// Frame numbers for following the flow of the trace
int frameNum = 0;

//////////////////////////////////////////////////////
// Detection Flags for Displaying to File and Screen
/////////////////////////////////////////////////////
int tachycardia = 0;
int brachycardia = 0;

//////////////////////////////////////////////////////
// SETUP Initialization
//////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  // Don't begin until Serial is active
  //while (!Serial) {}
  if (enableBluetooth) {
    // Initialize Bluetooth
    initializeBluetooth();
  }
  // Setup the pin for capturing heartbeat data
  pinMode(A3, INPUT);
  // Pin for start/stop button
  pinMode(23, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  pinMode(SD_CS, OUTPUT);
  prev  = HIGH;

  isRun = digitalRead(23);

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
  chooseMode();
  if (playBackMode == 0) {
    Serial.println("Recording Mode Selected");
  } else {
    Serial.println("Play Back Mode Selected");
  }
  if (!playBackMode) {
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
    // open next file in root.  The volume working directory, vwd, is root
    // define a serial output stream
    int sf = selectFile();
    char file[12] = "KGEM";
    char buf[8] = "KGEM";
    fileGen(buf + 4, sf);
    fileGen(file + 4, sf);
    file[7] = '.';
    file[8] = 't';
    file[9] = 'x';
    file[10] = 't';
    file[11] = '\0';
    Serial.print("Reading file: ");
    Serial.println(file);
    readSD2(file);
  }
}

////////////////////////////////////////////////////
// Select File to Open
////////////////////////////////////////////////////
int selectFile() {
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
  int sf = -1;
  tft.setCursor(60, 130);
  int accept = 0;
  sd.vwd()->rewind();
  char fBuffer[13];
  while (!accept) {
    if (!digitalRead(BUP)) {
      if (myFile.openNext(sd.vwd(), O_READ)) {
        while (myFile.isHidden()) {
          myFile.close();
          if (!myFile.openNext(sd.vwd(), O_READ)) {
            sf--;
            break;
          }
        }
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(60, 130);
        tft.print(fBuffer);
        tft.setCursor(60, 130);
        tft.setTextColor(ILI9341_BLACK);
        myFile.getName(fBuffer, 13);
        tft.print(fBuffer);
        sf++;
        myFile.close();
      }
      delay(250);
    }
    if (!digitalRead(BDOWN)) {
      sf = -1;
      sd.vwd()->rewind();
      tft.setTextColor(ILI9341_WHITE);
      tft.setCursor(60, 130);
      tft.print(fBuffer);
      delay(250);
    }

    if (!digitalRead(BSTART)) {
      int butCount = 0;
      while (!digitalRead(BSTART)) {
        butCount++;
        if (butCount > 300000) {
          accept = 1;
          return sf;
        }
      }
    }
  }
  return sf;
}


////////////////////////////////////////////////////
// Directory Printing
////////////////////////////////////////////////////
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

////////////////////////////////////////////////////
// Select Mode of Operation
////////////////////////////////////////////////////
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
    while (!modeSelect && digitalRead(22) && digitalRead(21)) {
      if (!digitalRead(23)) {
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
    while (!modeSelect && digitalRead(22) && digitalRead(21)) {
      if (!digitalRead(23)) {
        modeSelect = 1;
        playBackMode = 0;
      }
    }
  }
}

///////////////////////////////////////////////
// Draws the grid
///////////////////////////////////////////////
// Draws the red checkered grid on the display
void drawGrid() {
  // ETHAN
  for (int i = 0; i < 5; i++) {
    derBuf[i] = 0;
  }
  // ETHAN
  // Draws the grid
  tft.fillScreen(ILI9341_WHITE);
  for (int i = 0; i < HLINES; i++) {
    if (i % 5 == 0) {
      for (int j = 0; j < 3; j++) {
        tft.drawLine(i * BOXW + j, 0, i * BOXW + j, WIDTH, ILI9341_RED);
      }
    } else {
      tft.drawLine(i * BOXW, 0, i * BOXW, WIDTH, ILI9341_RED);
    }
  }
  for (int i = 0; i < VLINES; i++) {
    if (i % 5 == 0) {
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
  //tft.setCursor(200, 50);
  //setFont(const ILI9341_t3_font_t &f);
  //tft.fillRect(0, 0, 50, WIDTH, ILI9341_WHITE);
}

///////////////////////////////////////////
// Calibrates the HRM
///////////////////////////////////////////
// Prevents data saving to buffer until the input from the
// cardiograph is stabalized
// TODO: Adjust the min and max values based on input
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
  // Finish calibrating when the input signal is steady for
  // 5 continuous seconds
  while (millis() - startCal < 5000) {
    val = samples[currSamp];
    if (val < 400 || val > 3800) {
      startCal = millis();
      tft.setCursor(20, 200);
      tft.setTextColor(ILI9341_WHITE);
      tft.print("Stay Still");
    }
    if (millis() - startCal > 2000) {
      tft.setCursor(20, 200);
      tft.setTextColor(ILI9341_BLACK);
      tft.print("Stay Still");
    }
  }
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
  Serial.println("Calibrated");
  startTime = millis();
}

//// Prevents data saving to buffer until the input from the
//// cardiograph is stabalized
//// TODO: Adjust the min and max values based on input
//void calibrateMonitor() {
//  Serial.println("FUCKING CAL");
//  isCal = true;
//  tft.fillScreen(ILI9341_BLACK);
//  tft.setRotation(0);
//  tft.setTextSize(3);
//  tft.setTextColor(ILI9341_WHITE);
//  tft.setCursor(20, 100);
//  tft.print("Calibrating");
//  Serial.println("Calibrating")
//  int startCal = millis();
//  // Finish calibrating when the input signal is steady for
//  // 5 continuous seconds
//  while (millis() - startCal <= calLen * 1000 && calCount < BEATCAL) {
//    calculateBPM();
//  }
//  if (calCount < BEATCAL) {
//    Serial.println("DIDNT CALIBRATE");
//    // Couldn't calibrate, retry
//    currSamp = 0;
//    derSamp = 0;
//    calCount = 0;
////    calibrateMonitor();    <---------- This is weird
//  }
//  calCount = 0;
//  currSamp = 0;
//  totSamp = 0;
//  isCal = false;
//  isRun = true;
//  hasTachy = false;
//  hasBrady = false;
//  hasPAC = false;
//  currPAC = 0;
//  derSamp = 0;
//  Serial.println("Calibrated");
//  startTime = millis();
//}

int lastBeat = -1;
float bpm;
int but;

// Displays the trace of the cardiograph
// and determines whether we are in a running state
// or stopped state
void loop() {
  noInterrupts();
  // Read button value
  but = digitalRead(23);
  interrupts();
  // Detect start/stop button
  if (but == LOW && prev == HIGH) {
    isRun = !isRun;
    if (isRun) {
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
    noInterrupts();
    if (playBackMode) {
      // Scroll Back a screen
      if (!digitalRead(21)) {
        currPlayBackSamp = 0;
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
      if (!digitalRead(22)) {
        currPlayBackSamp = (currPlayBackSamp + 1) % (sizeof(samples) / 2);
        if (currPlayBackSamp == 0) {
          totPlayBack += (sizeof(samples) / 2);
          frameNum++;
          drawGrid();
        } else if (totPlayBack < playBackSamples) {
          tft.drawLine((int)(HEIGHT - (HEIGHT * playBack[currPlayBackSamp + totPlayBack - 1] / 3595.0)), currPlayBackSamp * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * playBack[currPlayBackSamp + totPlayBack] / 3595.0)), (currPlayBackSamp - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLUE);
        } else {
          currPlayBackSamp = 1;
        }
        delay(5);
      }
      but = digitalRead(23);
      if (but == LOW && prev == HIGH) {
        isRun = 1;
      }
      prev = but;
    }
  } else {
    // Actively running
    interrupts();

    // Refresh the graph on rollaround
    if (currSamp == 0 && !playBackMode) {
      lastPeak = -1;
      qStart = -1;
      sStart = -1;
      derSamp = 0;
      drawGrid();
      frameNum++;
    }

    // Write to our display the trace
    // scaled to emphasize our valid range
    if (playBackMode) {
      for (int i = currPlayBackSamp; i < (sizeof(samples) / 2); i++) {
        currPlayBackSamp = (currPlayBackSamp + 1) % (sizeof(samples) / 2);
        if (i != 0) {
          tft.drawLine((int)(HEIGHT - (HEIGHT * playBack[i + totPlayBack] / 3595.0)), i * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * playBack[i - 1 + totPlayBack] / 3595.0)), (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLUE);
          uint16_t delayWrite = millis();
          while (millis() - delayWrite < 10) {
            but = digitalRead(23);
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
          }
          drawGrid();
        }
      }
    } else {
      tft.setTextColor(ILI9341_PURPLE);
      tft.setTextSize(2);
      tft.setCursor(5, 240);
      if (hasTachy) {
        tft.print("Tachy: X");
      } else {
        tft.print("Tachy: ");
      }
      tft.setCursor(5, 260);
      if (hasBrady) {
        tft.print("Brady: X");
      } else {
        tft.print("Brady: ");
      }
      tft.setCursor(5, 280);
      if (hasPAC) {
        tft.print("PAC: X");
      } else {
        tft.print("PAC: ");
      }
      for (int i = printSamp; i < currSamp; i++) {
        if (i != 0) {
          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)), i * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)), (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLACK);
          //          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 1, i * ((double)WIDTH / (sRate * tLen)),
          //                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 1, (i - 1) * ((double)WIDTH / (sRate * tLen)),
          //                       ILI9341_BLACK);
          //          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 2, i * ((double)WIDTH / (sRate * tLen)),
          //                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 2, (i - 1) * ((double)WIDTH / (sRate * tLen)),
          //                       ILI9341_BLACK);
          //          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 3, i * ((double)WIDTH / (sRate * tLen)),
          //                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 3, (i - 1) * ((double)WIDTH / (sRate * tLen)),
          //                       ILI9341_BLACK);
        }
      }
      calculateBPM();
      if (millis() - startTime >= 30 * 1000) {
        lastBeat = 0;
        isRun = false;
        writeToSD();
      }
    }
  }
}

// ETHANS Calculate BPM
void calculateBPM() {
  if (currSamp > 3) {
    for (int i = derSamp; i < currSamp - 1; i++) {
      uint16_t *currBuf;
      float curDer = 0;
      if (isCal) {
        currBuf = calBuf;
      } else {  // Running
        currBuf = samples;
      }
      curDer = 0.1 * (2 * currBuf[i] + currBuf[i - 1] - currBuf[i - 3] - (2 * currBuf[i - 4]));
      derBuf[i] = pow(curDer, 2);

      // Test for heartbeats
      int timeDif = i - lastPeak;
      int valToAdd = -1;
      if (derBuf[i] > THRESHOLD && timeDif > MINWAIT) {
        valToAdd = timeDif;
      } else if (derBuf[i] < THRESHOLD &&  bpm != 0 && timeDif > MAXWAIT * sRate / bpm) {
        valToAdd = 0;
      }

      if (valToAdd != -1) {
        float prevBPM = bpm;
        if (currBeat < NUMBEATS) {
          bpm *= currBeat;
          currBeat++;
          bpm += valToAdd * (1.0 / sRate);
          bpm /= currBeat;
        } else {
          bpm = ((bpm * NUMBEATS) + (valToAdd  * (1.0 / sRate)) - bpm) / NUMBEATS;
        }
        if (bpm >= 105) {
          hasTachy = true;
        } else if (bpm <= 55) {
          hasBrady = true;
        }

        if ((bpm - prevBPM) / bpm >= MAXPACDIF) {
          currPAC++;
          if (currPAC > MAXPAC) {
            hasPAC = true;
          }
        }
        if (isCal) {
          calCount++;
        }
        Serial.print(F("Updating HRM value to "));
        Serial.print((int)(bpm * 60));
        Serial.println(F(" BPM"));

        if (enableBluetooth) {
          /* Command is sent when \n (\r) or println is called */
          /* AT+GATTCHAR=CharacteristicID,value */
          ble.print( F("AT+GATTCHAR=") );
          ble.print( hrmMeasureCharId );
          ble.print( F(",00-") );
          ble.println((int)(bpm * 60), HEX);

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
            tft.drawLine(0, (int)(i * ((double)WIDTH / (sRate * tLen))),
                         HEIGHT, (int)((i - 1) * ((double)WIDTH / (sRate * tLen))),
                         ILI9341_BLUE);
          }
        } else if (derBuf[i] < THRESHOLDS && qStart != -1 && sStart == -1) {
          // Heartbeat stopped
          sStart = i;
          tft.drawLine(0, (int)(i * ((double)WIDTH / (sRate * tLen))),
                       HEIGHT, (int)((i - 1) * ((double)WIDTH / (sRate * tLen))),
                       ILI9341_GREEN);
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
            Serial.print("QRS: ");
            Serial.println(qrsTime);
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
// OUTDATED VERSION
//void readSD() {
//  int countChar = 0;
//  if (!myFile.open("KGEM000.txt", O_READ)) {
//    sd.errorHalt("Opening data file for read failed");
//  }
//  Serial.println("Opened file for read");
//  int data;
//  while ((data = myFile.read()) >= 0) {
//
//    if (data != ',' && countChar > 10) {
//
//      Serial.write(data);
//    } else {
//      countChar++;
//    }
//  }
//  if (!myFile.remove()) Serial.println("Error file.remove");
//  //myFile.close();
//}

int curPlayBack = 0;

// Add a button to do SD card recall while in a stopped state
File file;
void readSD2(char *fn) {
  file = sd.open(fn, FILE_READ);
  if (!file) {
    Serial.println("open failed");
    return;
  }
  Serial.println("open success");
  file.seek(10);
  uint16_t t1;
  while (file.available() && curPlayBack < playBackSamples) {
    if (csvReadUint16(&file, &t1, ' ') == ' ') {
      playBack[curPlayBack] = t1;
      curPlayBack++;
    }
  }
  file.close();
  curPlayBack = 0;
}

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

int csvReadUint16(File * file, uint16_t* num, char delim) {
  uint32_t tmp;
  int rtn = csvReadUint32(file, &tmp, delim);
  if (rtn < 0) return rtn;
  if (tmp > UINT_MAX) return -5;
  *num = tmp;
  return rtn;
}

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
void writeToSD() {
  isRun = false;
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
      myFile.println((int)(bpm * 60));
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
// to our SD card or recalling a previous file name
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
// if we are not calibrating
// Also handles saving the data buffer to a SD card if one is
// detected
void pdb_isr() {
  PDB0_SC &= ~PDB_SC_PDBIF;
}

void adc0_isr() {
  if (isCal || isRun) {
    samples[currSamp] = nf.input(bw.input(ADC0_RA)) + BASELINE;
    data[totSamp] = samples[currSamp];
    if (isRun) {
      currSamp = (currSamp + 1) % (sizeof(samples) / 2);
      totSamp++;
    }
  } else {
    int temp = ADC0_RA;  // Resets the ADCISR flag, preventing infinite loops
  }
  // Ethan calibrate function
  //  if (isCal || isRun) {
  //    int sampLen = 0;
  //    if (isCal) {
  //      calBuf[currSamp] = nf.input(bw.input(ADC0_RA)) + BASELINE;
  //      sampLen = calLen * sRate;
  //    } else {  // Running
  //      samples[currSamp] = nf.input(bw.input(ADC0_RA)) + BASELINE;
  //      data[totSamp] = samples[currSamp];
  //      totSamp++;
  //      sampLen = tLen * sRate;
  //    }
  //    currSamp = (currSamp + 1) % sampLen;
  //  } else {
  //    int temp = ADC0_RA;  // Resets the ADCISR flag, preventing infinite loops
  //  }
}

////////////////////////////////////////////////////
// Bluetooth HRM Setup
////////////////////////////////////////////////////
void initializeBluetooth(void) {
  boolean success;
  Serial.println(F("Adafruit Bluefruit Heart Rate Monitor (HRM) Example"));
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
