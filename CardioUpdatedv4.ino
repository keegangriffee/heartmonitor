// Keegan Griffee #1432621
// Ethan Mayer #1168278
// Lab 8: Electrocardiograph Updated
// Uses an amplifier circuit and lcd interface to
// display an electrocardiograph to the user.
// Also every 30 seconds of continuous runtime
// will be saved to an SD card (if available) with the output
// of the ADC that is used to construct the trace.

// TODO: Add method for asking for switching between playback
// TODO: Add filebrowsing capabilities
// TODO: Add button for scrolling through file forward and back

// Define pins for using both the display and SD card
#define SD_CS 10
#define TFT_DC  9
#define TFT_CS 20

#include <stdlib.h>
#include <string.h> // For memset

#include "SPI.h"
#include "ILI9341_t3.h"

// Uses the SdFat library
#include "SdFat.h"
SdFat sd;
SdFile myFile;

// Used to determine if we should write to an SD card
boolean hasSDcard = false;

// The range of the ADC
const float resolution = 4095.0;

// Setup the display
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

#define PDB_CH0C1_TOS 0x0100
#define PDB_CH0C1_EN 0x01

static const int filter1_numStages = 2;
static const int filter1_coefficientLength = 10;
extern float filter1_coefficients[10];

typedef struct
{
  float state[8];
  float output;
} filter1Type;

typedef struct
{
  float *pInput;
  float *pOutput;
  float *pState;
  float *pCoefficients;
  short count;
} filter1_executionState;

float filter1_coefficients[10] =
{
  // Scaled for floating point

  0.14302759524247405, 0.2860551904849481, 0.14302759524247405, 0.8923452534610903, -0.22484317957421998,// b0, b1, b2, a1, a2
  0.0625, 0.125, 0.0625, 1.1544493220854493, -0.5846101862882838// b0, b1, b2, a1, a2

};

#define filter1_writeInput( pThis, input )  \
  filter1_filterBlock( pThis, &input, &pThis->output, 1 );

#define filter1_readOutput( pThis )  \
  pThis->output

int filter1_filterBlock( filter1Type * pThis, float * pInput, float * pOutput, unsigned int count );
#define filter1_outputToFloat( output )  \
  (output)

#define filter1_inputFromFloat( input )  \
  (input)

void filter1_filterBiquad( filter1_executionState * pExecState );

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

////////////////////////////////////////////
// Sample buffers
////////////////////////////////////////////

// Display tLen seconds of data
const int tLen = 2;
// Data buffer
uint16_t samples[sRate * tLen];
uint16_t data[sRate * 31];
uint16_t playBack[sRate * 31];
// Keep track of samples and files
int currSamp = 0;
int totSamp = 0;
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
filter1Type* bFilt;

// Define constants
#define WIDTH 320
#define HEIGHT 240
#define BOXW 20
#define HLINES (HEIGHT / BOXW)
#define VLINES (WIDTH / BOXW)

// Used for autostopping after 30 seconds of runtime
int startTime;

//////////////////////////////////////////////////////
// SETUP Initialization
//////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  // Don't begin until Serial is active
  while (!Serial) {}
  // Setup the pin for capturing heartbeat data
  pinMode(A3, INPUT);
  // Pin for start/stop button
  pinMode(23, INPUT_PULLUP);
  pinMode(SD_CS, OUTPUT);
  prev  = HIGH;

  isRun = digitalRead(23);

  // Initialize the SD card
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) {
    // Either an error or no SD card was present
    Serial.println("No SD card detected, proceeding...");
    // sd.initErrorHalt();
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
    bFilt = filter1_create();
  } else {
    // Read from the SD card for playback
    readSD2();
  }
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
  while (!modeSelect) {
    if (modeSelect) {
      return;
    }
    tft.fillScreen(ILI9341_WHITE);
    tft.setCursor(20, 100);
    tft.print("Select Mode:");
    int timer = millis();
    tft.setCursor(20, 140);
    tft.print("Playback [Y]");
    while (!modeSelect && millis() - timer <= 5000) {
      if (!digitalRead(23)) {
        modeSelect = 1;
        playBackMode = 1;
      }
    }
    if (modeSelect) {
      return;
    }
    tft.fillScreen(ILI9341_WHITE);
    timer = millis();
    tft.setCursor(20, 100);
    tft.print("Select Mode:");
    tft.setCursor(20, 140);
    tft.print("Record [Y]");
    while (!modeSelect && millis() - timer <= 5000) {
      if (!digitalRead(23)) {
        modeSelect = 1;
        playBackMode = 0;
      }
    }
  }
}

// Draws the red checkered grid on the display
void drawGrid() {
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
}

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
        tft.print("Stay Still");
    }
  }
  currSamp = 0;
  totSamp = 0;
  isCal = false;
  Serial.println("Calibrated");
  startTime = millis();
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

int lastBeat = 0;
float bpm;

// Displays the trace of the cardiograph
// and determines whether we are in a running state
// or stopped state
void loop() {
  int but;
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
        startTime = millis();
      }
    }
  }
  prev = but;
  if (!isRun) {
    // Stop display
    noInterrupts();
  } else {
    // Actively running
    interrupts();

    // Filter current data
    float filtData[currSamp];

    // Refresh the graph on rollaround
    if (currSamp == 0 && !playBackMode) {
      drawGrid();
    }

    if (currPlayBackSamp == 0 && playBackMode) {
      drawGrid();
      totPlayBack += (sizeof(samples) / 2);
    }

    // Write to our display the trace
    // scaled to emphasize our valid range
    if (playBackMode) {
      if (totPlayBack > playBackSamples) {
        totPlayBack = 0;
      }
      for (int i = 0; i < currPlayBackSamp; i++) {
        if (i != 0) {
          tft.drawLine((int)(HEIGHT - (HEIGHT * playBack[i + totPlayBack] / 3595.0)), i * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * playBack[i - 1 + totPlayBack] / 3595.0)), (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLUE);
          //        tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 1, i * ((double)WIDTH / (sRate * tLen)),
          //                     (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 1, (i - 1) * ((double)WIDTH / (sRate * tLen)),
          //                     ILI9341_BLACK);
          //        tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 2, i * ((double)WIDTH / (sRate * tLen)),
          //                     (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 2, (i - 1) * ((double)WIDTH / (sRate * tLen)),
          //                     ILI9341_BLACK);
          //        tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 3, i * ((double)WIDTH / (sRate * tLen)),
          //                     (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 3, (i - 1) * ((double)WIDTH / (sRate * tLen)),
          //                     ILI9341_BLACK);
        }
      }
      currPlayBackSamp = (currPlayBackSamp + 1) % (sizeof(samples) / 2);
    } else {
      for (int i = 0; i < currSamp; i++) {
        if (i != 0) {
          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)), i * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)), (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLACK);
          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 1, i * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 1, (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLACK);
          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 2, i * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 2, (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLACK);
          tft.drawLine((int)(HEIGHT - (HEIGHT * samples[i] / 3595.0)) + 3, i * ((double)WIDTH / (sRate * tLen)),
                       (int)(HEIGHT - (HEIGHT * samples[i - 1] / 3595.0)) + 3, (i - 1) * ((double)WIDTH / (sRate * tLen)),
                       ILI9341_BLACK);
        }
      }
      calculateBPM();
      if (millis() - startTime >= 10 * 1000) {
        lastBeat = 0;
        isRun = false;
        writeToSD();
      }
    }
  }
}

// TODO: Find a better threshold
// Potentially try a negative value
void calculateBPM() {
  if (totSamp > 0) {
    int val = data[totSamp];
    int prevVal = data[totSamp - 1];
    // Threshold for a heartbeat pulse
    if (prevVal < 2300 && val >= 2300) {
      if (lastBeat != 0) {
        // 60 secs in a min, divide by distance between heartbeats in sec
        // to get bpm
        bpm = 60.0 / ((totSamp - lastBeat) * (1.0 / 250));
        lastBeat = totSamp;
        Serial.println(bpm);
      } else {
        lastBeat = totSamp;
      }
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

// TODO: Play with SD card recall
// Add a button to do SD card recall while in a stopped state
File file;
void readSD2() {
  file = sd.open("KGEM000.txt", FILE_WRITE);
  if (!file) {
    Serial.println("open failed");
    return;
  }
  file.seek(10);
  uint16_t t1, t2, t3, t4;
  char space, result;
  while (file.available() && curPlayBack < playBackSamples) {
    if (csvReadUint16(&file, &t1, ' ') == ' ') {
      playBack[curPlayBack] = t1;
      curPlayBack++;
    }
  }
  file.close();
  // DEBUG FOR READING SD CARD
  //  int i = 0;
  //  while (i < playBackSamples / 8) {
  //    for (int j = 0; j < 8; j++) {
  //      if (j != 0) {
  //        Serial.print(" ");
  //      }
  //      Serial.print(playBack[8 * i + j]);
  //    }
  //    Serial.println();
  //    i++;
  //  }
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
// SD Card Writing
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
      if (!myFile.open(file, O_RDWR | O_CREAT | O_AT_END)) {
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
          //          if (j != 0) {
          //            myFile.print(" ");
          //          }
          myFile.print(data[8 * i + j]);
          myFile.print(" ");
        }
        myFile.println();
        i++;
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
    samples[currSamp] = ADC0_RA;
    data[totSamp] = samples[currSamp];
    if (isRun) {
      currSamp = (currSamp + 1) % (sizeof(samples) / 2);
      totSamp++;
    }
  } else {
    int temp = ADC0_RA;  // Resets the ADCISR flag, preventing infinite loops
  }
}

////////////////////////////////////////////////////////////////////////////
// Filter
////////////////////////////////////////////////////////////////////////////
filter1Type *filter1_create( void )
{
  filter1Type *result = (filter1Type *)malloc( sizeof( filter1Type ) ); // Allocate memory for the object
  filter1_init( result );                     // Initialize it
  return result;                                // Return the result
}

void filter1_destroy( filter1Type * pObject )
{
  free( pObject );
}

void filter1_init( filter1Type * pThis )
{
  filter1_reset( pThis );

}

void filter1_reset( filter1Type * pThis )
{
  memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
  pThis->output = 0;                  // Reset output

}

int filter1_filterBlock( filter1Type * pThis, float * pInput, float * pOutput, unsigned int count )
{
  filter1_executionState executionState;          // The executionState structure holds call data, minimizing stack reads and writes
  if ( ! count ) return 0;                        // If there are no input samples, return immediately
  executionState.pInput = pInput;                 // Pointers to the input and output buffers that each call to filterBiquad() will use
  executionState.pOutput = pOutput;               // - pInput and pOutput can be equal, allowing reuse of the same memory.
  executionState.count = count;                   // The number of samples to be processed
  executionState.pState = pThis->state;                   // Pointer to the biquad's internal state and coefficients.
  executionState.pCoefficients = filter1_coefficients;    // Each call to filterBiquad() will advance pState and pCoefficients to the next biquad

  // The 1st call to filter1_filterBiquad() reads from the caller supplied input buffer and writes to the output buffer.
  // The remaining calls to filterBiquad() recycle the same output buffer, so that multiple intermediate buffers are not required.

  filter1_filterBiquad( &executionState );    // Run biquad #0
  executionState.pInput = executionState.pOutput;         // The remaining biquads will now re-use the same output buffer.

  filter1_filterBiquad( &executionState );    // Run biquad #1
  // At this point, the caller-supplied output buffer will contain the filtered samples and the input buffer will contain the unmodified input samples.
  return count;   // Return the number of samples processed, the same as the number of input samples

}

void filter1_filterBiquad( filter1_executionState * pExecState )
{
  // Read state variables
  float w0, x0;
  float w1 = pExecState->pState[0];
  float w2 = pExecState->pState[1];

  // Read coefficients into work registers
  float b0 = *(pExecState->pCoefficients++);
  float b1 = *(pExecState->pCoefficients++);
  float b2 = *(pExecState->pCoefficients++);
  float a1 = *(pExecState->pCoefficients++);
  float a2 = *(pExecState->pCoefficients++);

  // Read source and target pointers
  float *pInput  = pExecState->pInput;
  float *pOutput = pExecState->pOutput;
  short count = pExecState->count;
  float accumulator;

  // Loop for all samples in the input buffer
  while ( count-- )
  {
    // Read input sample
    x0 = *(pInput++);

    // Run feedback part of filter
    accumulator  = w2 * a2;
    accumulator += w1 * a1;
    accumulator += x0 ;

    w0 = accumulator ;

    // Run feedforward part of filter
    accumulator  = w0 * b0;
    accumulator += w1 * b1;
    accumulator += w2 * b2;

    w2 = w1;    // Shuffle history buffer
    w1 = w0;

    // Write output
    *(pOutput++) = accumulator ;
  }

  // Write state variables
  *(pExecState->pState++) = w1;
  *(pExecState->pState++) = w2;

}