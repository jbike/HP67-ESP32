#include <Arduino.h>
#include <TM1640.h>
#include <Preferences.h>

// --------------------------------------------------
// HP-67 keyboard wiring
// --------------------------------------------------

// 8 rows
const int rowPins[8] = {
  16, 17, 18, 19, 21, 22, 23, 5
};

// 5 columns
const int colPins[5] = {
  26, 27, 14, 12, 13 //13 12 14 27 26
};

// Original HP-67 W/PRGM-RUN slide switch.
// One contact to GPIO 25, the other to GND.
// CLOSED (LOW) = W/PRGM, OPEN (HIGH) = RUN.
const int PROGRAM_RUN_SWITCH_PIN = 25;
const unsigned long PROGRAM_SWITCH_DEBOUNCE_MS = 20;
int programSwitchRawState = HIGH;
int programSwitchStableState = HIGH;
unsigned long programSwitchLastChangeMs = 0;


// --------------------------------------------------
// Confirmed HP-67 keyboard map
//
// Row 1, Col 1 = R/S
// --------------------------------------------------

const char* keyMap[8][5] = {
  // C1     C2      C3      C4       C5
  {"R/S",   ".",    "0",    "/",     ""},
  {"3",     "2",    "1",    "*",     ""},
  {"6",     "5",    "4",    "+",     ""},
  {"9",     "8",    "7",    "-",     ""},
  {"CLX",   "EEX",  "CHS",  "ENTER", ""},
  {"h",     "RCL",  "STO",  "g",     "f"},
  {"SST",   "(i)",  "DSP",  "GTO",   "SIGMA+"},
  {"E",     "D",    "C",    "B",     "A"}
};


// --------------------------------------------------
// RPN stack
//
// stack[0] = X
// stack[1] = Y
// stack[2] = Z
// stack[3] = T
// --------------------------------------------------

double stackReg[4] = {0, 0, 0, 0};


// Number currently being typed
String entry = "";

// True while typing a number
bool enteringNumber = false;

// HP-67 stack-lift state. ENTER and CLX disable the automatic stack
// lift for the very next numeric entry only. Once that entry starts,
// normal automatic stack lift is restored.
bool stackLiftEnabled = true;


// --------------------------------------------------
// Keyboard debounce / contact-chatter handling
// --------------------------------------------------
// Some original HP-67 key contacts momentarily reopen even while held.
// Instead of requiring 35 ms of uninterrupted LOW, qualify a press with
// a few LOW samples, then latch it until the key has been continuously
// released for 50 ms.
byte pressScore[8][5] = {};
bool keyLatched[8][5] = {};
unsigned long releaseStart[8][5] = {};
const unsigned long releaseDebounceTime = 50; // ms

bool exponentMode = false;

// FIX / SCI / ENG display
enum DisplayMode {
  MODE_FIX,
  MODE_SCI,
  MODE_ENG
};

// Trigonometric angle mode, as on the original HP-67.
enum AngleMode {
  ANGLE_DEG,
  ANGLE_RAD,
  ANGLE_GRD
};

AngleMode angleMode = ANGLE_DEG;

DisplayMode displayMode = MODE_FIX;
bool waitingForFormatDigit = false;
DisplayMode pendingDisplayMode = MODE_FIX;

int displayDigits = 4;       // startup default

bool prefixF = false;
bool prefixG = false;
bool prefixH = false;   // black shift key (h)
bool waitingForDSPDigit = false;

// Primary registers R0-R9 and RA-RE.
double storageReg[15] = {0};

// Protected secondary registers RS0-RS9.  Sigma accumulations use RS4-RS9.
double secondaryReg[10] = {0};

// Index register I and LAST X.
double iReg = 0.0;
double lastX = 0.0;

bool waitingForSTODigit = false;
bool waitingForRCLDigit = false;

// --------------------------------------------------
// HP-67 style program memory (first programming pass)
// --------------------------------------------------

enum ProgramOp : uint8_t {
  PGM_EMPTY = 0,
  PGM_KEY   = 1,
  PGM_LBL   = 2,
  PGM_GTO   = 3,
  PGM_GSB   = 4,
  PGM_RTN     = 5,
  PGM_STOP    = 6,
  PGM_KEY_ARG = 7
};

struct ProgramStep {
  uint8_t op;
  char arg;       // label/register operand when used
  char shift;     // 0, 'f', 'g', or 'h' for merged shifted instruction
  char key[8];
};

const int MAX_PROGRAM_STEPS = 224;
ProgramStep programMem[MAX_PROGRAM_STEPS] = {};
int programSize = 0;
int progCursor = 0;
int displayedProgramStep = -1;
int programPC = 0;
bool programMode = false;
bool programRunning = false;
bool waitingForProgramLabel = false;
char pendingProgramCommand = 0;   // 'L' LBL, 'G' GTO, 'S' GSB
bool waitingForProgramOperand = false;
char pendingProgramKey[8] = {0};
char pendingProgramShift = 0;
int callStack[4] = {0};
int callDepth = 0;
unsigned long lastProgramStepTime = 0;
const unsigned long programStepIntervalMs = 10;
Preferences prefs;

// --------------------------------------------------
// HP-67 LED display - TM1640
// --------------------------------------------------

// ESP32 -> LJ245 B side -> LJ245 A side -> TM1640
const int TM1640_DIN = 32;
const int TM1640_CLK = 33;

TM1640 ledDisplay(TM1640_DIN, TM1640_CLK);

// Confirmed HP-67 segment mapping:
// TM SEG1=G, SEG2=F, SEG3=DP, SEG4=E,
// TM SEG5=D, SEG6=B, SEG7=A, SEG8=C
const byte hpDigit[10] = {
  0xFA,   // 0
  0xA0,   // 1
  0x79,   // 2
  0xF1,   // 3
  0xA3,   // 4
  0xD3,   // 5
  0xDB,   // 6
  0xE0,   // 7
  0xFB,   // 8
  0xF3    // 9
};

const byte HP_DP    = 0x04;
const byte HP_MINUS = 0x01;
const byte HP_E     = 0x5B;  // A+D+E+F+G
const byte HP_BLANK = 0x00;

// The HP-67 uses the display positions themselves for the decimal point.
// Mantissa characters start at the left.  In scientific notation the
// exponent occupies the three far-right positions: sign, tens, ones.

byte ledPatternForChar(char c) {
  if (c >= '0' && c <= '9') {
    return hpDigit[c - '0'];
  }

  if (c == '.') return HP_DP;
  if (c == '-') return HP_MINUS;
  if (c == ' ') return HP_BLANK;

  return HP_BLANK;
}

void clearLedDisplay() {
  byte out[15] = {0};
  ledDisplay.setDisplay(out, 15);
}

// Ordinary text starts at the left, as on the HP-67 mantissa field.
void showLedText(const String& text) {
  byte out[15];
  for (byte i = 0; i < 15; i++) out[i] = HP_BLANK;

  int len = text.length();
  if (len > 15) len = 15;

  for (int i = 0; i < len; i++) {
    out[i] = ledPatternForChar(text[i]);
  }

  // One TM1640 auto-increment transfer instead of 15 separate commands.
  // This is much faster and prevents visible intermediate patterns while
  // also keeping the keyboard scanner from being held up by display writes.
  ledDisplay.setDisplay(out, 15);
}

// HP-style scientific display:
//   mantissa begins at the left
//   exponent sign is position 13 (index 12)
//   exponent tens is position 14 (index 13)
//   exponent ones is position 15 (index 14)
// There is no printed "E" character on the display.
void showLedScientific(const String& mantissa, int exponent) {
  byte out[15];
  for (byte i = 0; i < 15; i++) out[i] = HP_BLANK;

  // Leave the last three positions for the exponent.
  int mlen = mantissa.length();
  if (mlen > 12) mlen = 12;

  for (int i = 0; i < mlen; i++) {
    out[i] = ledPatternForChar(mantissa[i]);
  }

  bool negExp = (exponent < 0);
  int e = abs(exponent);
  if (e > 99) e = 99;

  if (negExp) out[12] = HP_MINUS;
  out[13] = hpDigit[(e / 10) % 10];
  out[14] = hpDigit[e % 10];

  // Write all 15 positions in one transaction.
  ledDisplay.setDisplay(out, 15);
}

// Convert between the active HP-67 angle mode and radians used by C math.
double angleToRadians(double a) {
  if (angleMode == ANGLE_RAD) return a;
  if (angleMode == ANGLE_GRD) return a * PI / 200.0;
  return a * PI / 180.0;
}

double radiansToAngle(double r) {
  if (angleMode == ANGLE_RAD) return r;
  if (angleMode == ANGLE_GRD) return r * 200.0 / PI;
  return r * 180.0 / PI;
}

// Engineering notation: exponent is always a multiple of 3.
void makeEngineering(double v, String& mantissa, int& exponent) {
  char buf[48];

  if (v == 0.0) {
    exponent = 0;
    snprintf(buf, sizeof(buf), "%.*f", displayDigits, 0.0);
    mantissa = String(buf);
    return;
  }

  double av = fabs(v);
  int e10 = (int)floor(log10(av));
  exponent = (int)floor(e10 / 3.0) * 3;
  double m = v / pow(10.0, exponent);

  // Account for rounding that turns 999.999... into 1000.000...
  double scale = pow(10.0, displayDigits);
  double rounded = round(m * scale) / scale;
  if (fabs(rounded) >= 1000.0) {
    rounded /= 1000.0;
    exponent += 3;
  }

  snprintf(buf, sizeof(buf), "%.*f", displayDigits, rounded);
  mantissa = String(buf);
}

// During EEX entry, keep the mantissa at the left and put the exponent
// in the right-hand exponent field immediately.
void showLedEntry(const String& s) {
  int ePos = s.indexOf('e');
  if (ePos < 0) ePos = s.indexOf('E');

  if (ePos < 0) {
    showLedText(s);
    return;
  }

  String mantissa = s.substring(0, ePos);
  String expText = s.substring(ePos + 1);

  bool negExp = false;
  if (expText.startsWith("-")) {
    negExp = true;
    expText.remove(0, 1);
  } else if (expText.startsWith("+")) {
    expText.remove(0, 1);
  }

  int e = 0;
  if (expText.length() > 0) {
    e = expText.toInt();
  }
  if (negExp) e = -e;

  showLedScientific(mantissa, e);
}

// Format a completed X-register value for the physical HP-67 display.
void showFormattedLedValue(double v) {
  char buf[48];

  if (displayMode == MODE_FIX) {
    snprintf(buf, sizeof(buf), "%.*f", displayDigits, v);
    String fixed(buf);

    // Decimal point consumes a display position, so the string length
    // is the actual number of GRID positions required.
    if (fixed.length() <= 15) {
      showLedText(fixed);
      return;
    }

    // If FIX will not fit, fall back to scientific notation.
  }

  if (displayMode == MODE_ENG) {
    String mantissa;
    int exponent;
    makeEngineering(v, mantissa, exponent);
    showLedScientific(mantissa, exponent);
    return;
  }

  snprintf(buf, sizeof(buf), "%.*E", displayDigits, v);
  String sci(buf);

  int ePos = sci.indexOf('E');
  if (ePos < 0) {
    showLedText(sci);
    return;
  }

  String mantissa = sci.substring(0, ePos);
  int exponent = sci.substring(ePos + 1).toInt();

  showLedScientific(mantissa, exponent);
}

void updateHp67Display() {
  if (enteringNumber) {
    showLedEntry(entry);
  } else {
    showFormattedLedValue(stackReg[0]);
  }
}

// --------------------------------------------------
// Stack functions
// --------------------------------------------------

void liftStack() {
  stackReg[3] = stackReg[2];
  stackReg[2] = stackReg[1];
  stackReg[1] = stackReg[0];
}


void dropStack() {
  stackReg[0] = stackReg[1];
  stackReg[1] = stackReg[2];
  stackReg[2] = stackReg[3];

  // HP-style T register repeats
  // stackReg[3] stays unchanged
}


void commitEntry() {
  if (enteringNumber) {

    // Complete an unfinished exponent
    if (entry.endsWith("e") || entry.endsWith("e-")) {
      entry += "0";
    }

    stackReg[0] = entry.toDouble();

    enteringNumber = false;
    exponentMode = false;
    entry = "";
  }
}


// --------------------------------------------------
// Display stack on Serial Monitor
// --------------------------------------------------

void showStack() {

  Serial.println();
  Serial.println("----------------------");

  Serial.print("T: ");
  printFormatted(stackReg[3]);
  Serial.println();

  Serial.print("Z: ");
  printFormatted(stackReg[2]);
  Serial.println();

  Serial.print("Y: ");
  printFormatted(stackReg[1]);
  Serial.println();

  Serial.print("X: ");

  if (enteringNumber) {
    Serial.println(entry);
  } else {
    printFormatted(stackReg[0]);
    Serial.println();
  }

  Serial.println("----------------------");

  updateHp67Display();
}

void setFIX() {
  displayMode = MODE_FIX;

  prefixF = false;
  prefixG = false;
  prefixH = false;

  Serial.print("FIX ");
  Serial.println(displayDigits);

  showStack();
}

void setSCI() {
  displayMode = MODE_SCI;

  prefixF = false;
  prefixG = false;
  prefixH = false;

  Serial.print("SCI ");
  Serial.println(displayDigits);

  showStack();
}

void setENG() {
  commitEntry();
  displayMode = MODE_ENG;

  prefixF = false;
  prefixG = false;
  prefixH = false;

  Serial.print("ENG ");
  Serial.println(displayDigits);

  showStack();
}

void setDisplayDigits(int n) {

  if (n < 0) n = 0;
  if (n > 9) n = 9;

  displayDigits = n;

  waitingForDSPDigit = false;

  Serial.print("DSP ");
  Serial.println(displayDigits);

  showStack();
}
void printFormatted(double v) {

  if (displayMode == MODE_FIX) {
    Serial.print(v, displayDigits);
    return;
  }

  if (displayMode == MODE_ENG) {
    String mantissa;
    int exponent;
    makeEngineering(v, mantissa, exponent);
    Serial.print(mantissa);
    Serial.print(" E");
    if (exponent >= 0) Serial.print("+");
    else { Serial.print("-"); exponent = -exponent; }
    if (exponent < 10) Serial.print("0");
    Serial.print(exponent);
    return;
  }

  // SCI mode
  if (v == 0.0) {
    Serial.print("0.");
    for (int i = 0; i < displayDigits; i++) Serial.print("0");
    Serial.print(" E+00");
    return;
  }

  int exponent = floor(log10(fabs(v)));
  double mantissa = v / pow(10.0, exponent);

  Serial.print(mantissa, displayDigits);
  if (exponent >= 0) Serial.print(" E+");
  else { Serial.print(" E-"); exponent = -exponent; }
  if (exponent < 10) Serial.print("0");
  Serial.print(exponent);
}


// --------------------------------------------------
// Digit entry
// --------------------------------------------------

void enterDigit(char digit) {

  if (!enteringNumber) {
    if (stackLiftEnabled) {
      liftStack();
    }

    entry = "";
    enteringNumber = true;
    exponentMode = false;

    // ENTER/CLX suppress stack lift for only this one new entry.
    stackLiftEnabled = true;
  }

  // HP-67 exponent is two digits
  if (exponentMode) {

    int ePos = entry.indexOf('e');

    if (ePos >= 0) {
      int exponentDigits = 0;

      for (int i = ePos + 1; i < entry.length(); i++) {
        if (entry[i] >= '0' && entry[i] <= '9') {
          exponentDigits++;
        }
      }

      if (exponentDigits >= 2) {
        return;
      }
    }
  }

  entry += digit;

  stackReg[0] = entry.toDouble();

  showStack();
}


// --------------------------------------------------
// Decimal point
// --------------------------------------------------

void enterDecimal() {

  // No decimal point in exponent
  if (exponentMode) {
    return;
  }

  if (!enteringNumber) {
    if (stackLiftEnabled) {
      liftStack();
    }

    entry = "0";
    enteringNumber = true;
    stackLiftEnabled = true;
  }

  if (entry.indexOf('.') < 0) {
    entry += ".";
  }

  showStack();
}


// --------------------------------------------------
// ENTER
// --------------------------------------------------

void doEnter() {

  commitEntry();

  // HP-67 ENTER lifts the stack and duplicates X into Y.
  liftStack();
  stackReg[1] = stackReg[0];

  // The next number typed replaces X; it must NOT lift the stack again.
  stackLiftEnabled = false;

  showStack();
}


// --------------------------------------------------
// CHS
// --------------------------------------------------

void doCHS() {

  if (enteringNumber) {

    if (exponentMode) {

      int ePos = entry.indexOf('e');

      if (ePos >= 0) {

        if (ePos + 1 < entry.length() &&
            entry[ePos + 1] == '-') {

          entry.remove(ePos + 1, 1);

        } else {

          entry = entry.substring(0, ePos + 1) +
                  "-" +
                  entry.substring(ePos + 1);
        }
      }

    } else {

      // Normal mantissa sign change
      if (entry.startsWith("-")) {
        entry.remove(0, 1);
      } else {
        entry = "-" + entry;
      }
    }

    stackReg[0] = entry.toDouble();

  } else {

    stackReg[0] = -stackReg[0];
  }

  showStack();
}

void doEEX() {

  // Start a new number if necessary
  if (!enteringNumber) {
    if (stackLiftEnabled) {
      liftStack();
    }

    // HP-style: EEX by itself starts with 1
    entry = "1";
    enteringNumber = true;
    stackLiftEnabled = true;
  }

  // Already entering exponent
  if (exponentMode) {
    return;
  }

  entry += "e";
  exponentMode = true;

  showStack();
}

// --------------------------------------------------
// CLX
// --------------------------------------------------

void doCLX() {

  entry = "";
  enteringNumber = false;
  exponentMode = false;

  stackReg[0] = 0;

  // Like the real HP-67, the next numeric entry replaces X.
  stackLiftEnabled = false;

  showStack();
}


// --------------------------------------------------
// Arithmetic
// --------------------------------------------------

void doOperation(char op) {

  commitEntry();

  double x = stackReg[0];
  double y = stackReg[1];
  lastX = x;

  double result = 0;

  switch (op) {

    case '+':
      result = y + x;
      break;

    case '-':
      result = y - x;
      break;

    case '*':
      result = y * x;
      break;

    case '/':

      if (x == 0) {
        Serial.println("ERROR: Divide by zero");
        return;
      }

      result = y / x;
      break;
  }

  dropStack();

  stackReg[0] = result;

  showStack();
}

// --------------------------------------------------
// Yellow/blue shifted functions on 1, 2, 3
// --------------------------------------------------

// g 1 : rectangular -> polar
// Input:  X = x coordinate, Y = y coordinate
// Output: X = radius, Y = angle in the active DEG/RAD/GRD mode
void doRectToPolar() {
  commitEntry();

  lastX = stackReg[0];
  double x = stackReg[0];
  double y = stackReg[1];

  stackReg[0] = hypot(x, y);
  stackReg[1] = radiansToAngle(atan2(y, x));

  showStack();
}

// f 1 : polar -> rectangular
// Input:  X = radius, Y = angle in the active DEG/RAD/GRD mode
// Output: X = x coordinate, Y = y coordinate
void doPolarToRect() {
  commitEntry();

  lastX = stackReg[0];
  double radius = stackReg[0];
  double angleRad = angleToRadians(stackReg[1]);

  stackReg[0] = radius * cos(angleRad);
  stackReg[1] = radius * sin(angleRad);

  showStack();
}

// g 2 : degrees -> radians
void doDegToRad() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = stackReg[0] * PI / 180.0;
  showStack();
}

// f 2 : radians -> degrees
void doRadToDeg() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = stackReg[0] * 180.0 / PI;
  showStack();
}

// g 3 : decimal hours -> H.MMSS
// Example: 1.5 hours -> 1.3000 (1 hour, 30 minutes, 00 seconds)
void doHoursToHMS() {
  commitEntry();

  double value = stackReg[0];
  double sign = (value < 0.0) ? -1.0 : 1.0;
  double a = fabs(value);

  double hours = floor(a);
  double totalMinutes = (a - hours) * 60.0;
  double minutes = floor(totalMinutes + 1e-10);
  double seconds = (totalMinutes - minutes) * 60.0;

  // Protect against floating-point rollover at 59.999999... seconds.
  if (seconds >= 59.9999995) {
    seconds = 0.0;
    minutes += 1.0;
    if (minutes >= 60.0) {
      minutes = 0.0;
      hours += 1.0;
    }
  }

  stackReg[0] = sign * (hours + minutes / 100.0 + seconds / 10000.0);
  showStack();
}

// f 3 : H.MMSS -> decimal hours
// Example: 1.3000 -> 1.5 hours
void doHMSToHours() {
  commitEntry();

  double value = stackReg[0];
  double sign = (value < 0.0) ? -1.0 : 1.0;
  double a = fabs(value);

  double hours = floor(a);
  double mmss = (a - hours) * 100.0;
  double minutes = floor(mmss + 1e-9);
  double seconds = (mmss - minutes) * 100.0;

  // H.MMSS requires valid minute and second fields.
  if (minutes >= 60.0 || seconds >= 60.0) {
    Serial.println("ERROR: invalid H.MMSS");
    return;
  }

  stackReg[0] = sign * (hours + minutes / 60.0 + seconds / 3600.0);
  showStack();
}

// --------------------------------------------------
// Additional HP-67 shifted math functions
// --------------------------------------------------

// h 2 : pi constant
void doPi() {
  commitEntry();

  // Treat pi like entering a new constant: preserve the old X in Y.
  liftStack();
  stackReg[0] = PI;

  enteringNumber = false;
  exponentMode = false;
  entry = "";

  showStack();
}

// f 7 : natural logarithm
void doLN() {
  commitEntry();

  if (stackReg[0] <= 0.0) {
    Serial.println("ERROR: LN domain");
    return;
  }

  lastX = stackReg[0];
  stackReg[0] = log(stackReg[0]);
  showStack();
}

// g 7 : e^x
void doExp() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = exp(stackReg[0]);
  showStack();
}

// f 8 : common logarithm
void doLOG() {
  commitEntry();

  if (stackReg[0] <= 0.0) {
    Serial.println("ERROR: LOG domain");
    return;
  }

  lastX = stackReg[0];
  stackReg[0] = log10(stackReg[0]);
  showStack();
}

// g 8 : 10^x
void do10toX() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = pow(10.0, stackReg[0]);
  showStack();
}

// g 9 : x^2
void doSquare() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = stackReg[0] * stackReg[0];
  showStack();
}

// trig functions
void doSIN() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = sin(angleToRadians(stackReg[0]));
  showStack();
}

void doCOS() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = cos(angleToRadians(stackReg[0]));
  showStack();
}

void doTAN() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = tan(angleToRadians(stackReg[0]));
  showStack();
}

void doASIN() {
  commitEntry();

  if (stackReg[0] < -1.0 || stackReg[0] > 1.0) {
    Serial.println("ERROR: ASIN domain");
    return;
  }

  lastX = stackReg[0];
  stackReg[0] = radiansToAngle(asin(stackReg[0]));
  showStack();
}

void doACOS() {
  commitEntry();

  if (stackReg[0] < -1.0 || stackReg[0] > 1.0) {
    Serial.println("ERROR: ACOS domain");
    return;
  }

  lastX = stackReg[0];
  stackReg[0] = radiansToAngle(acos(stackReg[0]));
  showStack();
}

void doATAN() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = radiansToAngle(atan(stackReg[0]));
  showStack();
}
// Letter functions
void doReciprocal() {
  commitEntry();

  if (stackReg[0] == 0.0) {
    Serial.println("ERROR: divide by zero");
    return;
  }

  lastX = stackReg[0];
  stackReg[0] = 1.0 / stackReg[0];
  showStack();
}


void doSqrt() {
  commitEntry();

  if (stackReg[0] < 0.0) {
    Serial.println("ERROR: sqrt domain");
    return;
  }

  lastX = stackReg[0];
  stackReg[0] = sqrt(stackReg[0]);
  showStack();
}


void doYtoX() {
  commitEntry();

  double x = stackReg[0];
  double y = stackReg[1];

  lastX = x;
  double result = pow(y, x);

  // Binary operation: consume X and Y
  dropStack();
  stackReg[0] = result;

  showStack();
}


void doRollDown() {
  commitEntry();

  double oldX = stackReg[0];

  stackReg[0] = stackReg[1];
  stackReg[1] = stackReg[2];
  stackReg[2] = stackReg[3];
  stackReg[3] = oldX;

  showStack();
}


void doSwapXY() {
  commitEntry();

  double temp = stackReg[0];
  stackReg[0] = stackReg[1];
  stackReg[1] = temp;

  showStack();
}
// --------------------------------------------------
// Additional front-face functions from the original HP-67
// --------------------------------------------------

void doRollUp() {
  commitEntry();
  double oldT = stackReg[3];
  stackReg[3] = stackReg[2];
  stackReg[2] = stackReg[1];
  stackReg[1] = stackReg[0];
  stackReg[0] = oldT;
  showStack();
}

void doABS() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = fabs(stackReg[0]);
  showStack();
}

void doFactorial() {
  commitEntry();
  double x = stackReg[0];
  if (x < 0.0 || fabs(x - round(x)) > 1e-10 || x > 170.0) {
    Serial.println("ERROR: factorial requires integer 0..170");
    return;
  }
  lastX = x;
  stackReg[0] = tgamma(x + 1.0);
  showStack();
}

void setAngleMode(AngleMode mode) {
  commitEntry();
  angleMode = mode;
  if (mode == ANGLE_DEG) Serial.println("DEG");
  else if (mode == ANGLE_RAD) Serial.println("RAD");
  else Serial.println("GRD");
  showStack();
}

void doStoreI() {
  commitEntry();
  iReg = stackReg[0];
  Serial.print("ST I = "); Serial.println(iReg, 10);
  showStack();
}

void doRecallI() {
  commitEntry();
  liftStack();
  stackReg[0] = iReg;
  enteringNumber = false;
  entry = "";
  Serial.print("RC I = "); Serial.println(iReg, 10);
  showStack();
}

void doSwapXI() {
  commitEntry();
  double t = stackReg[0];
  stackReg[0] = iReg;
  iReg = t;
  showStack();
}

int indirectIndex() {
  return (int)trunc(fabs(iReg));
}

double* indirectRegisterPtr() {
  int n = indirectIndex();
  if (n >= 0 && n <= 9) return &storageReg[n];
  if (n >= 10 && n <= 19) return &secondaryReg[n - 10];
  if (n >= 20 && n <= 24) return &storageReg[10 + (n - 20)];
  if (n == 25) return &iReg;
  return nullptr;
}

void doStoreIndirect() {
  commitEntry();
  double* r = indirectRegisterPtr();
  if (!r) { Serial.println("ERROR: I indirect address out of range"); return; }
  *r = stackReg[0];
  Serial.print("STO(i), I="); Serial.println(iReg, 6);
  showStack();
}

void doRecallIndirect() {
  commitEntry();
  double* r = indirectRegisterPtr();
  if (!r) { Serial.println("ERROR: I indirect address out of range"); return; }
  liftStack();
  stackReg[0] = *r;
  enteringNumber = false;
  entry = "";
  Serial.print("RCL(i), I="); Serial.println(iReg, 6);
  showStack();
}

void doPrimarySecondarySwap() {
  commitEntry();
  for (int i = 0; i < 10; i++) {
    double t = storageReg[i];
    storageReg[i] = secondaryReg[i];
    secondaryReg[i] = t;
  }
  Serial.println("P<>S");
  showStack();
}

void doClearRegisters() {
  // Original HP-67 CL REG behavior: clear PRIMARY storage registers
  // R0-R9, A-E and I.  It does NOT clear the stack, LAST X, or the
  // protected secondary/statistics registers.
  for (int i = 0; i < 15; i++) storageReg[i] = 0.0;
  iReg = 0.0;
  Serial.println("CL REG (primary registers only)");
  showStack();
}

void doSigmaPlus() {
  commitEntry();
  double x = stackReg[0];
  double y = stackReg[1];
  lastX = x;
  secondaryReg[4] += x;
  secondaryReg[5] += x * x;
  secondaryReg[6] += y;
  secondaryReg[7] += y * y;
  secondaryReg[8] += x * y;
  secondaryReg[9] += 1.0;
  stackReg[0] = secondaryReg[9];
  showStack();
}

void doSigmaMinus() {
  commitEntry();
  double x = stackReg[0];
  double y = stackReg[1];
  lastX = x;
  secondaryReg[4] -= x;
  secondaryReg[5] -= x * x;
  secondaryReg[6] -= y;
  secondaryReg[7] -= y * y;
  secondaryReg[8] -= x * y;
  secondaryReg[9] -= 1.0;
  stackReg[0] = secondaryReg[9];
  showStack();
}

void doMean() {
  commitEntry();
  double n = secondaryReg[9];
  if (n == 0.0) { Serial.println("ERROR: no sigma data"); return; }
  lastX = stackReg[0];
  stackReg[0] = secondaryReg[4] / n;
  stackReg[1] = secondaryReg[6] / n;
  showStack();
}

void doStdDev() {
  commitEntry();
  double n = secondaryReg[9];
  if (n <= 1.0) { Serial.println("ERROR: need at least 2 sigma samples"); return; }
  double sx2 = (n * secondaryReg[5] - secondaryReg[4] * secondaryReg[4]) / (n * (n - 1.0));
  double sy2 = (n * secondaryReg[7] - secondaryReg[6] * secondaryReg[6]) / (n * (n - 1.0));
  if (sx2 < 0 && sx2 > -1e-12) sx2 = 0;
  if (sy2 < 0 && sy2 > -1e-12) sy2 = 0;
  if (sx2 < 0 || sy2 < 0) { Serial.println("ERROR: statistics domain"); return; }
  lastX = stackReg[0];
  stackReg[0] = sqrt(sx2);
  stackReg[1] = sqrt(sy2);
  showStack();
}

void doPercent() {
  commitEntry();
  double x = stackReg[0];
  double y = stackReg[1];
  lastX = x;
  stackReg[0] = y * x / 100.0;
  showStack();
}

void doPercentChange() {
  commitEntry();
  double x = stackReg[0];
  double y = stackReg[1];
  if (y == 0.0) { Serial.println("ERROR: percent change divide by zero"); return; }
  lastX = x;
  stackReg[0] = 100.0 * (x - y) / y;
  showStack();
}

void doINT() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = trunc(stackReg[0]);
  showStack();
}

void doFRAC() {
  commitEntry();
  lastX = stackReg[0];
  stackReg[0] = stackReg[0] - trunc(stackReg[0]);
  showStack();
}

void doLastX() {
  commitEntry();
  liftStack();
  stackReg[0] = lastX;
  showStack();
}

void doHMSAdd() {
  commitEntry();
  auto hmsToHoursRaw = [](double value, bool &ok) -> double {
    double sign = value < 0.0 ? -1.0 : 1.0;
    double a = fabs(value);
    double h = floor(a);
    double mmss = (a - h) * 100.0;
    double m = floor(mmss + 1e-9);
    double sec = (mmss - m) * 100.0;
    if (m >= 60.0 || sec >= 60.0) { ok = false; return 0.0; }
    return sign * (h + m / 60.0 + sec / 3600.0);
  };
  auto hoursToHMSRaw = [](double value) -> double {
    double sign = value < 0.0 ? -1.0 : 1.0;
    double a = fabs(value);
    double h = floor(a);
    double totalSec = (a - h) * 3600.0;
    double m = floor(totalSec / 60.0 + 1e-10);
    double sec = totalSec - m * 60.0;
    if (sec >= 59.9999995) { sec = 0; m += 1; }
    if (m >= 60) { m = 0; h += 1; }
    return sign * (h + m / 100.0 + sec / 10000.0);
  };
  bool ok1 = true, ok2 = true;
  double hx = hmsToHoursRaw(stackReg[0], ok1);
  double hy = hmsToHoursRaw(stackReg[1], ok2);
  if (!ok1 || !ok2) { Serial.println("ERROR: invalid H.MMSS"); return; }
  lastX = stackReg[0];
  double result = hoursToHMSRaw(hy + hx);
  dropStack();
  stackReg[0] = result;
  showStack();
}

void doRoundDisplayed() {
  commitEntry();
  lastX = stackReg[0];
  if (displayMode == MODE_FIX) {
    double scale = pow(10.0, displayDigits);
    stackReg[0] = round(stackReg[0] * scale) / scale;
  } else if (displayMode == MODE_ENG) {
    String m; int e; makeEngineering(stackReg[0], m, e);
    stackReg[0] = m.toDouble() * pow(10.0, e);
  } else {
    if (stackReg[0] != 0.0) {
      int e = (int)floor(log10(fabs(stackReg[0])));
      double m = stackReg[0] / pow(10.0, e);
      double scale = pow(10.0, displayDigits);
      m = round(m * scale) / scale;
      stackReg[0] = m * pow(10.0, e);
    }
  }
  showStack();
}

// store and recall
void doStore(int reg) {
  commitEntry();
  if (reg < 0 || reg >= 15) return;

  storageReg[reg] = stackReg[0];

  Serial.print("STO ");
  Serial.print(reg);
  Serial.print(" = ");
  printFormatted(storageReg[reg]);
  Serial.println();

  showStack();
}


void doRecall(int reg) {
  // Recall acts like entering a new number:
  // lift stack, then place recalled value in X
  if (reg < 0 || reg >= 15) return;

  commitEntry();

  liftStack();
  stackReg[0] = storageReg[reg];

  enteringNumber = false;
  entry = "";

  Serial.print("RCL ");
  Serial.print(reg);
  Serial.print(" = ");
  printFormatted(stackReg[0]);
  Serial.println();

  showStack();
}


void processKey(const char* key);
String formatProgramStep(const ProgramStep& step);
void saveProgramMemory();
void loadProgramMemory();
void clearProgramMemory();
void deleteDisplayedProgramStep();
void printProgramPosition(const char* reason);
void showProgramStepDisplay(int index);
int hpMatrixCodeForKey(const char* key);
int hpUnshiftedProgramCode(const char* key);
int hpLabelProgramCode(char label);
bool isProgramLabelKey(const char* key, char &label);
int findProgramLabel(char label);
bool insertProgramStep(const ProgramStep& step);
bool storeProgramKey(const char* key, char shift);
bool storeProgramKeyArg(const char* key, char arg, char shift);
bool storeProgramControl(uint8_t op, char arg = 0);
void startProgramRun();
void startProgramAtLabel(char label);
void stopProgramRun(const char* why);
bool positionProgramAtLabel(char label);
bool executeProgramStep();

// --------------------------------------------------
// Program-memory helpers
// --------------------------------------------------

String formatProgramStep(const ProgramStep& step) {
  String s;
  switch (step.op) {
    case PGM_KEY:
      if (step.shift) { s += step.shift; s += " "; }
      s += step.key;
      return s;
    case PGM_KEY_ARG:
      if (step.shift) { s += step.shift; s += " "; }
      s += step.key;
      s += " ";
      if (step.arg == 'I') s += "(i)";
      else s += step.arg;
      return s;
    case PGM_LBL:  return String("LBL ") + step.arg;
    case PGM_GTO:  return String("GTO ") + step.arg;
    case PGM_GSB:  return String("GSB ") + step.arg;
    case PGM_RTN:  return String("RTN");
    case PGM_STOP: return String("R/S");
    default:       return String("<empty>");
  }
}

int hpMatrixCodeForKey(const char* key) {
  // Original HP-67 row/column positions, top-to-bottom and left-to-right.
  if (strcmp(key, "A") == 0) return 11;
  if (strcmp(key, "B") == 0) return 12;
  if (strcmp(key, "C") == 0) return 13;
  if (strcmp(key, "D") == 0) return 14;
  if (strcmp(key, "E") == 0) return 15;

  if (strcmp(key, "SIGMA+") == 0) return 21;
  if (strcmp(key, "GTO") == 0)    return 22;
  if (strcmp(key, "DSP") == 0)    return 23;
  if (strcmp(key, "(i)") == 0)    return 24;
  if (strcmp(key, "SST") == 0)    return 25;

  if (strcmp(key, "f") == 0)   return 31;
  if (strcmp(key, "g") == 0)   return 32;
  if (strcmp(key, "STO") == 0) return 33;
  if (strcmp(key, "RCL") == 0) return 34;
  if (strcmp(key, "h") == 0)   return 35;

  if (strcmp(key, "ENTER") == 0) return 41;
  if (strcmp(key, "CHS") == 0)   return 42;
  if (strcmp(key, "EEX") == 0)   return 43;
  if (strcmp(key, "CLX") == 0)   return 44;

  if (strcmp(key, "-") == 0) return 51;
  if (strcmp(key, "7") == 0) return 52;
  if (strcmp(key, "8") == 0) return 53;
  if (strcmp(key, "9") == 0) return 54;

  if (strcmp(key, "+") == 0) return 61;
  if (strcmp(key, "4") == 0) return 62;
  if (strcmp(key, "5") == 0) return 63;
  if (strcmp(key, "6") == 0) return 64;

  if (strcmp(key, "*") == 0) return 71;
  if (strcmp(key, "1") == 0) return 72;
  if (strcmp(key, "2") == 0) return 73;
  if (strcmp(key, "3") == 0) return 74;

  if (strcmp(key, "/") == 0)   return 81;
  if (strcmp(key, "0") == 0)   return 82;
  if (strcmp(key, ".") == 0)   return 83;
  if (strcmp(key, "R/S") == 0) return 84;

  return 0;
}

int hpUnshiftedProgramCode(const char* key) {
  // Digits are the HP-67 exception: they display 00 through 09.
  if (strlen(key) == 1 && key[0] >= '0' && key[0] <= '9') {
    return key[0] - '0';
  }
  return hpMatrixCodeForKey(key);
}

int hpLabelProgramCode(char label) {
  if (label >= '0' && label <= '9') return label - '0';
  if (label >= 'A' && label <= 'E') return 11 + (label - 'A');
  return 0;
}

void showProgramStepDisplay(int index) {
  if (!programMode) return;

  if (programSize <= 0 || index < 0 || index >= programSize) {
    displayedProgramStep = -1;
    showLedText("000");
    return;
  }

  displayedProgramStep = index;

  const ProgramStep& step = programMem[index];
  int codes[3] = {0, 0, 0};
  int count = 0;

  switch (step.op) {
    case PGM_KEY:
      if (step.shift) {
        char sh[2] = { step.shift, '\0' };
        codes[count++] = hpMatrixCodeForKey(sh);
        codes[count++] = hpMatrixCodeForKey(step.key);
      } else {
        codes[count++] = hpUnshiftedProgramCode(step.key);
      }
      break;

    case PGM_KEY_ARG:
      if (step.shift) {
        char sh[2] = { step.shift, '\0' };
        codes[count++] = hpMatrixCodeForKey(sh);
      }
      codes[count++] = hpMatrixCodeForKey(step.key);
      if (step.arg == 'I') codes[count++] = hpMatrixCodeForKey("(i)");
      else codes[count++] = hpLabelProgramCode(step.arg);
      break;

    case PGM_LBL:
      codes[count++] = 31; // f
      codes[count++] = 25; // SST key -> LBL
      codes[count++] = hpLabelProgramCode(step.arg);
      break;

    case PGM_GTO:
      codes[count++] = 22;
      codes[count++] = hpLabelProgramCode(step.arg);
      break;

    case PGM_GSB:
      codes[count++] = 31; // f
      codes[count++] = 22; // GTO key -> GSB
      codes[count++] = hpLabelProgramCode(step.arg);
      break;

    case PGM_RTN:
      codes[count++] = 35; // h
      codes[count++] = 22; // GTO key -> RTN
      break;

    case PGM_STOP:
      codes[count++] = 84;
      break;

    default:
      break;
  }

  char line[20];
  int n = snprintf(line, sizeof(line), "%03d", index + 1);
  for (int i = 0; i < count && n < (int)sizeof(line) - 4; i++) {
    n += snprintf(line + n, sizeof(line) - n, " %02d", codes[i]);
  }
  showLedText(String(line));
}

void saveProgramMemory() {
  prefs.begin("hp67pgm", false);
  prefs.putInt("fmt", 12);
  prefs.putInt("size", programSize);
  prefs.putBytes("steps", programMem, sizeof(programMem));
  prefs.end();
}

void loadProgramMemory() {
  memset(programMem, 0, sizeof(programMem));
  prefs.begin("hp67pgm", true);
  int fmt = prefs.getInt("fmt", 0);
  int storedSize = prefs.getInt("size", 0);
  size_t got = prefs.getBytes("steps", programMem, sizeof(programMem));
  prefs.end();

  if (fmt != 12 || got != sizeof(programMem)) {
    memset(programMem, 0, sizeof(programMem));
    programSize = 0;
  } else {
    if (storedSize < 0) storedSize = 0;
    if (storedSize > MAX_PROGRAM_STEPS) storedSize = MAX_PROGRAM_STEPS;
    programSize = storedSize;
  }

  progCursor = programSize;
  displayedProgramStep = (programSize > 0) ? programSize - 1 : -1;
  programPC = 0;
  waitingForProgramLabel = false;
  pendingProgramCommand = 0;
  waitingForProgramOperand = false;
  pendingProgramKey[0] = '\0';
  pendingProgramShift = 0;
  callDepth = 0;
  programRunning = false;
  programMode = false;
}

void clearProgramMemory() {
  memset(programMem, 0, sizeof(programMem));
  programSize = 0;
  progCursor = 0;
  displayedProgramStep = -1;
  programPC = 0;
  waitingForProgramLabel = false;
  pendingProgramCommand = 0;
  waitingForProgramOperand = false;
  pendingProgramKey[0] = '\0';
  pendingProgramShift = 0;
  callDepth = 0;
  programRunning = false;

  // Original HP-67 CL PRGM also returns the calculator to FIX 2 and DEG.
  displayMode = MODE_FIX;
  displayDigits = 2;
  angleMode = ANGLE_DEG;
  prefixF = prefixG = prefixH = false;

  saveProgramMemory();
  Serial.println("Program memory cleared; FIX 2; DEG");
  if (programMode) showLedText("000");
  printProgramPosition("CL PRGM");
}

void deleteDisplayedProgramStep() {
  if (programSize <= 0) {
    displayedProgramStep = -1;
    progCursor = 0;
    showLedText("000");
    Serial.println("DEL: program empty");
    return;
  }

  int target = displayedProgramStep;
  if (target < 0 || target >= programSize) {
    target = progCursor;
    if (target >= programSize) target = programSize - 1;
    if (target < 0) target = 0;
  }

  Serial.print("DEL step ");
  Serial.print(target + 1);
  Serial.print(": ");
  Serial.println(formatProgramStep(programMem[target]));

  for (int i = target; i < programSize - 1; i++) {
    programMem[i] = programMem[i + 1];
  }
  memset(&programMem[programSize - 1], 0, sizeof(ProgramStep));
  programSize--;

  // Leave the cursor at the deleted location so the next entered
  // instruction replaces it, just as expected while editing.
  progCursor = target;

  if (programSize <= 0) {
    displayedProgramStep = -1;
    progCursor = 0;
    showLedText("000");
  } else {
    if (target >= programSize) target = programSize - 1;
    displayedProgramStep = target;
    showProgramStepDisplay(target);
  }

  saveProgramMemory();
  printProgramPosition("DEL");
}

void printProgramPosition(const char* reason) {
  Serial.print("[");
  Serial.print(programMode ? "PRGM" : (programRunning ? "RUN" : "RUN-IDLE"));
  Serial.print("] ");
  Serial.print(reason);
  Serial.print("  size=");
  Serial.print(programSize);
  Serial.print("  cursor=");
  Serial.print(progCursor);
  Serial.print("  pc=");
  Serial.println(programPC);

  if (programMode) {
    int shown = progCursor;
    if (shown >= programSize) shown = programSize - 1;
    if (shown >= 0) {
      Serial.print("Step ");
      Serial.print(shown + 1);
      Serial.print(": ");
      Serial.println(formatProgramStep(programMem[shown]));
      showProgramStepDisplay(shown);
    } else {
      Serial.println("Program empty");
      showLedText("000");
    }
  } else {
    if (programPC < programSize) {
      Serial.print("PC ");
      Serial.print(programPC + 1);
      Serial.print(": ");
      Serial.println(formatProgramStep(programMem[programPC]));
    } else {
      Serial.println("PC at END");
    }
  }
}

bool isProgramLabelKey(const char* key, char &label) {
  if (strlen(key) != 1) return false;
  char c = key[0];
  if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'E')) {
    label = c;
    return true;
  }
  return false;
}

int findProgramLabel(char label) {
  for (int i = 0; i < programSize; i++) {
    if (programMem[i].op == PGM_LBL && programMem[i].arg == label) {
      return i;
    }
  }
  return -1;
}

bool insertProgramStep(const ProgramStep& step) {
  if (programSize >= MAX_PROGRAM_STEPS) {
    Serial.println("ERROR: program memory full");
    return false;
  }

  if (progCursor < 0) progCursor = 0;
  if (progCursor > programSize) progCursor = programSize;

  int insertedAt = progCursor;
  for (int i = programSize; i > progCursor; i--) {
    programMem[i] = programMem[i - 1];
  }

  programMem[progCursor] = step;
  programSize++;
  progCursor++;
  saveProgramMemory();

  Serial.print("Stored step ");
  Serial.print(insertedAt + 1);
  Serial.print(": ");
  Serial.println(formatProgramStep(programMem[insertedAt]));
  showProgramStepDisplay(insertedAt);
  return true;
}

bool storeProgramKey(const char* key, char shift) {
  ProgramStep step = {};
  step.op = PGM_KEY;
  step.shift = shift;
  strncpy(step.key, key, sizeof(step.key) - 1);
  step.key[sizeof(step.key) - 1] = '\0';
  return insertProgramStep(step);
}

bool storeProgramKeyArg(const char* key, char arg, char shift) {
  ProgramStep step = {};
  step.op = PGM_KEY_ARG;
  step.arg = arg;
  step.shift = shift;
  strncpy(step.key, key, sizeof(step.key) - 1);
  step.key[sizeof(step.key) - 1] = '\0';
  return insertProgramStep(step);
}

bool storeProgramControl(uint8_t op, char arg) {
  ProgramStep step = {};
  step.op = op;
  step.arg = arg;
  step.shift = 0;
  step.key[0] = '\0';
  return insertProgramStep(step);
}

void startProgramRun() {
  if (programSize <= 0) {
    Serial.println("No program stored");
    return;
  }

  if (programPC < 0 || programPC >= programSize) {
    programPC = 0;
  }

  programRunning = true;
  lastProgramStepTime = millis();
  printProgramPosition("RUN start");
}

void startProgramAtLabel(char label) {
  int where = findProgramLabel(label);
  if (where < 0) {
    Serial.print("ERROR: label not found: ");
    Serial.println(label);
    return;
  }

  callDepth = 1;
  callStack[0] = -1;
  programPC = where + 1;
  startProgramRun();
}

void stopProgramRun(const char* why) {
  programRunning = false;
  printProgramPosition(why);
}

bool positionProgramAtLabel(char label) {
  int where = findProgramLabel(label);
  if (where < 0) {
    Serial.print("ERROR: label not found: ");
    Serial.println(label);
    return false;
  }

  programPC = where + 1;
  printProgramPosition("label");
  return true;
}

bool executeProgramStep() {
  if (programPC < 0 || programPC >= programSize) {
    stopProgramRun("END");
    return false;
  }

  ProgramStep step = programMem[programPC];
  Serial.print("EXEC ");
  Serial.print(programPC + 1);
  Serial.print(": ");
  Serial.println(formatProgramStep(step));

  programPC++;  // default advance

  switch (step.op) {
    case PGM_KEY: {
      if (step.shift) {
        char sh[2] = { step.shift, '\0' };
        processKey(sh);
      }
      processKey(step.key);
      return true;
    }

    case PGM_KEY_ARG: {
      if (step.shift) {
        char sh[2] = { step.shift, '\0' };
        processKey(sh);
      }
      processKey(step.key);
      if (step.arg == 'I') {
        processKey("(i)");
      } else {
        char a[2] = { step.arg, '\0' };
        processKey(a);
      }
      return true;
    }

    case PGM_LBL:
      return true;

    case PGM_GTO: {
      int where = findProgramLabel(step.arg);
      if (where < 0) {
        Serial.print("ERROR: GTO label not found: ");
        Serial.println(step.arg);
        stopProgramRun("GTO ERROR");
        return false;
      }
      programPC = where + 1;
      return true;
    }

    case PGM_GSB: {
      int where = findProgramLabel(step.arg);
      if (where < 0) {
        Serial.print("ERROR: GSB label not found: ");
        Serial.println(step.arg);
        stopProgramRun("GSB ERROR");
        return false;
      }
      if (callDepth >= 4) {
        Serial.println("ERROR: subroutine nest overflow");
        stopProgramRun("CALL ERROR");
        return false;
      }
      callStack[callDepth++] = programPC;
      programPC = where + 1;
      return true;
    }

    case PGM_RTN:
      if (callDepth > 0) {
        int ret = callStack[--callDepth];
        if (ret < 0) {
          stopProgramRun("RTN");
          return false;
        }
        programPC = ret;
        return true;
      }
      stopProgramRun("RTN");
      return false;

    case PGM_STOP:
      stopProgramRun("R/S stop");
      return false;

    default:
      stopProgramRun("BAD STEP");
      return false;
  }
}

// --------------------------------------------------
// Physical W/PRGM-RUN switch
// --------------------------------------------------

void applyProgramRunSwitchState(int state) {
  bool newProgramMode = (state == LOW);

  if (newProgramMode == programMode) return;

  programMode = newProgramMode;
  programRunning = false;
  waitingForProgramLabel = false;
  pendingProgramCommand = 0;
  waitingForProgramOperand = false;
  pendingProgramKey[0] = '\0';
  pendingProgramShift = 0;
  prefixF = prefixG = prefixH = false;
  callDepth = 0;

  if (programMode) {
    progCursor = programSize;
    Serial.println("W/PRGM-RUN switch -> PRGM");
    printProgramPosition("PRGM switch");
  } else {
    programPC = 0;
    Serial.println("W/PRGM-RUN switch -> RUN");
    showStack();
    printProgramPosition("RUN switch");
  }
}

void updateProgramRunSwitch() {
  int raw = digitalRead(PROGRAM_RUN_SWITCH_PIN);

  if (raw != programSwitchRawState) {
    programSwitchRawState = raw;
    programSwitchLastChangeMs = millis();
  }

  if (raw != programSwitchStableState &&
      (millis() - programSwitchLastChangeMs) >= PROGRAM_SWITCH_DEBOUNCE_MS) {
    programSwitchStableState = raw;
    applyProgramRunSwitchState(programSwitchStableState);
  }
}

// --------------------------------------------------
// Process key
// --------------------------------------------------

void processKey(const char* key) {

  if (key == nullptr || strlen(key) == 0) {
    return;
  }

  Serial.print("KEY: ");
  Serial.println(key);

  // --------------------------------------------------
  // Program label operand pending
  // --------------------------------------------------

  if (waitingForProgramLabel) {
    // If an instruction was started by mistake, a shift key cancels the
    // pending label and becomes the new prefix.  This makes f CLX (CL PRGM)
    // and h CLX (DEL) reliable escape/edit commands even mid-entry.
    if (programMode && (strcmp(key, "f") == 0 || strcmp(key, "g") == 0 || strcmp(key, "h") == 0)) {
      waitingForProgramLabel = false;
      pendingProgramCommand = 0;
      prefixF = (strcmp(key, "f") == 0);
      prefixG = (strcmp(key, "g") == 0);
      prefixH = (strcmp(key, "h") == 0);
      Serial.println("Pending program label cancelled; shift accepted");
      return;
    }

    char label = 0;

    if (isProgramLabelKey(key, label)) {
      if (programMode) {
        if (pendingProgramCommand == 'L') storeProgramControl(PGM_LBL, label);
        else if (pendingProgramCommand == 'G') storeProgramControl(PGM_GTO, label);
        else if (pendingProgramCommand == 'S') storeProgramControl(PGM_GSB, label);
      } else {
        if (pendingProgramCommand == 'G') {
          positionProgramAtLabel(label);
        }
        else if (pendingProgramCommand == 'S') {
          startProgramAtLabel(label);
        }
      }

      waitingForProgramLabel = false;
      pendingProgramCommand = 0;
      prefixF = prefixG = prefixH = false;
      return;
    }

    waitingForProgramLabel = false;
    pendingProgramCommand = 0;
    prefixF = prefixG = prefixH = false;
    Serial.println("Program label entry cancelled");
    return;
  }

  // --------------------------------------------------
  // PRGM mode operand pending (STO/RCL/DSP, FIX/SCI, etc.)
  // --------------------------------------------------

  if (programMode && waitingForProgramOperand) {
    if (strcmp(key, "f") == 0 || strcmp(key, "g") == 0 || strcmp(key, "h") == 0) {
      waitingForProgramOperand = false;
      pendingProgramKey[0] = '\0';
      pendingProgramShift = 0;
      prefixF = (strcmp(key, "f") == 0);
      prefixG = (strcmp(key, "g") == 0);
      prefixH = (strcmp(key, "h") == 0);
      Serial.println("Pending program operand cancelled; shift accepted");
      return;
    }

    char arg = 0;

    if (strcmp(key, "(i)") == 0) {
      arg = 'I';
    } else if (strlen(key) == 1 &&
               ((key[0] >= '0' && key[0] <= '9') ||
                (key[0] >= 'A' && key[0] <= 'E'))) {
      arg = key[0];
    }

    bool valid = (arg != 0);

    // DSP and FIX/SCI take only 0-9 or (i) where applicable.
    if (valid && strcmp(pendingProgramKey, "DSP") == 0) {
      valid = ((arg >= '0' && arg <= '9') || arg == 'I');
    }
    if (valid && strcmp(pendingProgramKey, "STO") == 0 &&
        (pendingProgramShift == 'f' || pendingProgramShift == 'g')) {
      valid = (arg >= '0' && arg <= '9');
    }

    if (valid) {
      storeProgramKeyArg(pendingProgramKey, arg, pendingProgramShift);
    } else {
      Serial.println("Program operand entry cancelled");
    }

    waitingForProgramOperand = false;
    pendingProgramKey[0] = '\0';
    pendingProgramShift = 0;
    prefixF = prefixG = prefixH = false;
    return;
  }

  // --------------------------------------------------
  // PRGM mode: store merged HP-style instructions
  // --------------------------------------------------

  if (programMode) {

    if (strcmp(key, "f") == 0) {
      prefixF = true; prefixG = false; prefixH = false;
      Serial.println("PRGM f");
      return;
    }
    if (strcmp(key, "g") == 0) {
      prefixG = true; prefixF = false; prefixH = false;
      Serial.println("PRGM g");
      return;
    }
    if (strcmp(key, "h") == 0) {
      prefixH = true; prefixF = false; prefixG = false;
      Serial.println("PRGM h");
      return;
    }

    // f SST = LBL on the original HP-67.
    if (prefixF && strcmp(key, "SST") == 0) {
      prefixF = false;
      waitingForProgramLabel = true;
      pendingProgramCommand = 'L';
      Serial.println("LBL: press 0-9 or A-E");
      return;
    }

    // f GTO = GSB on the original HP-67 (keycode 31 22).
    if (prefixF && strcmp(key, "GTO") == 0) {
      prefixF = false;
      waitingForProgramLabel = true;
      pendingProgramCommand = 'S';
      Serial.println("GSB: press 0-9 or A-E");
      return;
    }

    // h GTO = RTN on the original HP-67.
    if (prefixH && strcmp(key, "GTO") == 0) {
      prefixH = false;
      storeProgramControl(PGM_RTN);
      return;
    }

    // h SST = BST while reviewing program memory.
    if (prefixH && strcmp(key, "SST") == 0) {
      prefixH = false;
      if (programSize <= 0) {
        showLedText("000");
      } else {
        if (progCursor > programSize - 1) progCursor = programSize - 1;
        else if (progCursor > 0) progCursor--;
        showProgramStepDisplay(progCursor);
        printProgramPosition("BST");
      }
      return;
    }

    // Original HP-67 legends on the CLX key:
    // f CLX = CL PRGM, h CLX = DEL.
    if (prefixF && strcmp(key, "CLX") == 0) {
      prefixF = false;
      clearProgramMemory();
      return;
    }

    if (prefixH && strcmp(key, "CLX") == 0) {
      prefixH = false;
      deleteDisplayedProgramStep();
      return;
    }

    // SST reviews forward through program memory without storing SST.
    if (strcmp(key, "SST") == 0) {
      prefixF = prefixG = prefixH = false;
      if (programSize <= 0) {
        showLedText("000");
      } else {
        if (progCursor >= programSize) progCursor = 0;
        else if (progCursor < programSize - 1) progCursor++;
        showProgramStepDisplay(progCursor);
        printProgramPosition("SST");
      }
      return;
    }

    // Unshifted GTO stores a merged GTO + label instruction.
    if (strcmp(key, "GTO") == 0) {
      waitingForProgramLabel = true;
      pendingProgramCommand = 'G';
      prefixF = prefixG = prefixH = false;
      Serial.println("GTO: press 0-9 or A-E");
      return;
    }

    // R/S is one merged program step (code 84).
    if (strcmp(key, "R/S") == 0) {
      prefixF = prefixG = prefixH = false;
      storeProgramControl(PGM_STOP);
      return;
    }

    // Register/display instructions with an operand are stored as one step.
    if (!prefixF && !prefixG && !prefixH &&
        (strcmp(key, "STO") == 0 || strcmp(key, "RCL") == 0 || strcmp(key, "DSP") == 0)) {
      waitingForProgramOperand = true;
      strncpy(pendingProgramKey, key, sizeof(pendingProgramKey) - 1);
      pendingProgramKey[sizeof(pendingProgramKey) - 1] = '\0';
      pendingProgramShift = 0;
      Serial.print("PRGM "); Serial.print(key); Serial.println(": enter operand");
      return;
    }

    // In our present calculator mapping, f STO n = FIX n and g STO n = SCI n.
    if ((prefixF || prefixG) && strcmp(key, "STO") == 0) {
      waitingForProgramOperand = true;
      strncpy(pendingProgramKey, key, sizeof(pendingProgramKey) - 1);
      pendingProgramKey[sizeof(pendingProgramKey) - 1] = '\0';
      pendingProgramShift = prefixF ? 'f' : 'g';
      prefixF = prefixG = prefixH = false;
      Serial.println("PRGM format: enter 0-9");
      return;
    }

    // Any other shifted function is merged into one 224-step location.
    if (prefixF || prefixG || prefixH) {
      char shift = prefixF ? 'f' : (prefixG ? 'g' : 'h');
      prefixF = prefixG = prefixH = false;
      storeProgramKey(key, shift);
      return;
    }

    storeProgramKey(key, 0);
    return;
  }


  // --------------------------------------------------
  // Waiting for digit after FIX or SCI
  // --------------------------------------------------

  if (waitingForFormatDigit) {

    if (strlen(key) == 1 &&
        key[0] >= '0' &&
        key[0] <= '9') {

      commitEntry();

      displayMode = pendingDisplayMode;
      displayDigits = key[0] - '0';

      waitingForFormatDigit = false;

      if (displayMode == MODE_FIX) {
        Serial.print("FIX ");
      } else {
        Serial.print("SCI ");
      }

      Serial.println(displayDigits);

      showStack();
      return;
    }

    waitingForFormatDigit = false;
  }


  // --------------------------------------------------
  // DSP waiting for digit
  // --------------------------------------------------

  if (waitingForDSPDigit) {

    if (strlen(key) == 1 && key[0] >= '0' && key[0] <= '9') {
      displayDigits = key[0] - '0';
      waitingForDSPDigit = false;
      Serial.print("DSP ");
      Serial.println(displayDigits);
      showStack();
      return;
    }

    if (strcmp(key, "(i)") == 0) {
      int n = indirectIndex();
      if (n > 9) n = 9;
      displayDigits = n;
      waitingForDSPDigit = false;
      Serial.print("DSP(i) "); Serial.println(displayDigits);
      showStack();
      return;
    }

    waitingForDSPDigit = false;
  }

  // --------------------------------------------------
  // Waiting for STO / RCL register address
  // --------------------------------------------------

  if (waitingForSTODigit) {
    int reg = -1;
    if (strlen(key) == 1 && key[0] >= '0' && key[0] <= '9') reg = key[0] - '0';
    else if (strlen(key) == 1 && key[0] >= 'A' && key[0] <= 'E') reg = 10 + (key[0] - 'A');

    if (reg >= 0) {
      waitingForSTODigit = false;
      doStore(reg);
      return;
    }

    if (strcmp(key, "(i)") == 0) {
      waitingForSTODigit = false;
      doStoreIndirect();
      return;
    }

    waitingForSTODigit = false;
  }

  if (waitingForRCLDigit) {
    int reg = -1;
    if (strlen(key) == 1 && key[0] >= '0' && key[0] <= '9') reg = key[0] - '0';
    else if (strlen(key) == 1 && key[0] >= 'A' && key[0] <= 'E') reg = 10 + (key[0] - 'A');

    if (reg >= 0) {
      waitingForRCLDigit = false;
      doRecall(reg);
      return;
    }

    if (strcmp(key, "(i)") == 0) {
      waitingForRCLDigit = false;
      doRecallIndirect();
      return;
    }

    waitingForRCLDigit = false;
  }

  // f f = mean; g f = sample standard deviation.
  if (strcmp(key, "f") == 0 && prefixF) {
    prefixF = false;
    doMean();
    return;
  }
  if (strcmp(key, "f") == 0 && prefixG) {
    prefixG = false;
    doStdDev();
    return;
  }

  // --------------------------------------------------
  // f prefix
  // --------------------------------------------------

  if (strcmp(key, "f") == 0) {
    prefixF = true;
    prefixG = false;
    prefixH = false;

    Serial.println("f");
    return;
  }

  // --------------------------------------------------
  // g prefix
  // --------------------------------------------------

  if (strcmp(key, "g") == 0) {
    prefixG = true;
    prefixF = false;
    prefixH = false;

    Serial.println("g");
    return;
  }

  // --------------------------------------------------
  // h prefix (black shift)
  // --------------------------------------------------

  if (strcmp(key, "h") == 0) {
    prefixH = true;
    prefixF = false;
    prefixG = false;

    Serial.println("h");
    return;
  }

  // --------------------------------------------------
  // h (black) shifted functions visible on the HP-67 face
  // --------------------------------------------------

  if (prefixH) {
    if (strcmp(key, "R/S") == 0)      { prefixH = false; Serial.println("PRGM mode is controlled by the W/PRGM-RUN slide switch"); return; }
    if (strcmp(key, "SST") == 0)      { prefixH = false; if (programPC > 0) programPC--; printProgramPosition("BST"); return; }
    if (strcmp(key, "GTO") == 0)      {
      prefixH = false;
      if (programRunning) {
        if (callDepth > 0) {
          int ret = callStack[--callDepth];
          if (ret < 0) stopProgramRun("RTN");
          else programPC = ret;
        } else {
          programPC = 0;
          stopProgramRun("RTN");
        }
      } else {
        programPC = 0;
        callDepth = 0;
        Serial.println("RTN: program counter -> 000");
      }
      return;
    }

    if (strcmp(key, "SIGMA+") == 0) { prefixH = false; doSigmaMinus(); return; }
    if (strcmp(key, "DSP") == 0)    { prefixH = false; setENG(); return; }
    if (strcmp(key, "(i)") == 0)    { prefixH = false; doSwapXI(); return; }
    if (strcmp(key, "STO") == 0)    { prefixH = false; doStoreI(); return; }
    if (strcmp(key, "RCL") == 0)    { prefixH = false; doRecallI(); return; }

    if (strcmp(key, "ENTER") == 0)  { prefixH = false; setAngleMode(ANGLE_DEG); return; }
    if (strcmp(key, "CHS") == 0)    { prefixH = false; setAngleMode(ANGLE_RAD); return; }
    if (strcmp(key, "EEX") == 0)    { prefixH = false; setAngleMode(ANGLE_GRD); return; }

    if (strcmp(key, "7") == 0)      { prefixH = false; doSwapXY(); return; }
    if (strcmp(key, "8") == 0)      { prefixH = false; doRollDown(); return; }
    if (strcmp(key, "9") == 0)      { prefixH = false; doRollUp(); return; }
    if (strcmp(key, "4") == 0)      { prefixH = false; doReciprocal(); return; }
    if (strcmp(key, "5") == 0)      { prefixH = false; doYtoX(); return; }
    if (strcmp(key, "6") == 0)      { prefixH = false; doABS(); return; }
    if (strcmp(key, "1") == 0)      { prefixH = false; delay(1000); showStack(); return; } // PAUSE
    if (strcmp(key, "2") == 0)      { prefixH = false; doPi(); return; }
    if (strcmp(key, "0") == 0)      { prefixH = false; doLastX(); return; }
    if (strcmp(key, ".") == 0)      { prefixH = false; doHMSAdd(); return; }
    if (strcmp(key, "/") == 0)      { prefixH = false; doFactorial(); return; }
  }

  // --------------------------------------------------
  // f/g shifted functions on number keys 1 through 9
  // --------------------------------------------------

  if (prefixF) {
    // f GTO = GSB.  In RUN mode this immediately executes the selected
    // subroutine label after its operand is entered.
    if (strcmp(key, "GTO") == 0) {
      prefixF = false;
      waitingForProgramLabel = true;
      pendingProgramCommand = 'S';
      Serial.println("GSB: press 0-9 or A-E");
      return;
    }

    // f CLX is CL PRGM.  In RUN mode the original HP-67 does NOT erase
    // memory; it simply resets the program counter to 000.
    if (strcmp(key, "CLX") == 0) {
      prefixF = false;
      programRunning = false;
      programPC = 0;
      callDepth = 0;
      Serial.println("CL PRGM in RUN mode: program counter -> 000 (program retained)");
      showStack();
      return;
    }

    if (strcmp(key, "SST") == 0) {
      prefixF = false;
      Serial.println("LBL is entered only in PRGM mode");
      return;
    }

    if (strcmp(key, "1") == 0) { prefixF = false; doPolarToRect(); return; }
    if (strcmp(key, "2") == 0) { prefixF = false; doRadToDeg(); return; }
    if (strcmp(key, "3") == 0) { prefixF = false; doHMSToHours(); return; }
    if (strcmp(key, "4") == 0) { prefixF = false; doSIN(); return; }
    if (strcmp(key, "5") == 0) { prefixF = false; doCOS(); return; }
    if (strcmp(key, "6") == 0) { prefixF = false; doTAN(); return; }
    if (strcmp(key, "7") == 0) { prefixF = false; doLN(); return; }
    if (strcmp(key, "8") == 0) { prefixF = false; doLOG(); return; }
    if (strcmp(key, "9") == 0) { prefixF = false; doSqrt(); return; }
  }

  if (prefixG) {
    if (strcmp(key, "1") == 0) { prefixG = false; doRectToPolar(); return; }
    if (strcmp(key, "2") == 0) { prefixG = false; doDegToRad(); return; }
    if (strcmp(key, "3") == 0) { prefixG = false; doHoursToHMS(); return; }
    if (strcmp(key, "4") == 0) { prefixG = false; doASIN(); return; }
    if (strcmp(key, "5") == 0) { prefixG = false; doACOS(); return; }
    if (strcmp(key, "6") == 0) { prefixG = false; doATAN(); return; }
    if (strcmp(key, "7") == 0) { prefixG = false; doExp(); return; }
    if (strcmp(key, "8") == 0) { prefixG = false; do10toX(); return; }
    if (strcmp(key, "9") == 0) { prefixG = false; doSquare(); return; }
  }

  // Other f/g shifted front-face functions.
  if (prefixF && strcmp(key, "CHS") == 0) { prefixF = false; doPrimarySecondarySwap(); return; } // P<>S
  if (prefixF && strcmp(key, "EEX") == 0) { prefixF = false; doClearRegisters(); return; }       // CL REG (primary registers only, as on original HP-67)
  if (prefixF && strcmp(key, "RCL") == 0) { prefixF = false; doRoundDisplayed(); return; }       // RND
  if (prefixF && strcmp(key, "0") == 0)   { prefixF = false; doPercent(); return; }
  if (prefixG && strcmp(key, "0") == 0)   { prefixG = false; doPercentChange(); return; }
  if (prefixF && strcmp(key, ".") == 0)   { prefixF = false; doINT(); return; }
  if (prefixG && strcmp(key, ".") == 0)   { prefixG = false; doFRAC(); return; }

  // --------------------------------------------------
  // STO with prefix
  // --------------------------------------------------

  if (strcmp(key, "STO") == 0) {

    if (prefixF) {
      pendingDisplayMode = MODE_FIX;
      waitingForFormatDigit = true;
      prefixF = false; prefixG = false; prefixH = false;
      Serial.println("FIX: press 0-9");
      return;
    }

    if (prefixG) {
      commitEntry();
      displayMode = MODE_SCI;
      pendingDisplayMode = MODE_SCI;
      waitingForFormatDigit = true;
      prefixF = false; prefixG = false; prefixH = false;
      Serial.println("SCI: press 0-9");
      showStack();
      return;
    }

    waitingForSTODigit = true;
    Serial.println("STO: press 0-9,A-E or (i)");
    return;
  }

  if (strcmp(key, "RCL") == 0) {
    waitingForRCLDigit = true;
    prefixF = false; prefixG = false; prefixH = false;
    Serial.println("RCL: press 0-9,A-E or (i)");
    return;
  }

  // Never let an unimplemented shifted legend fall through and execute
  // the unshifted key by mistake.
  if (prefixF || prefixG || prefixH) {
    Serial.print("Shifted function pending: ");
    if (prefixF) Serial.print("f+");
    else if (prefixG) Serial.print("g+");
    else Serial.print("h+");
    Serial.println(key);
    prefixF = prefixG = prefixH = false;
    return;
  }

  // --------------------------------------------------
  // DSP
  // --------------------------------------------------

  if (strcmp(key, "DSP") == 0) {
    waitingForDSPDigit = true;
    prefixF = false; prefixG = false; prefixH = false;
    Serial.println("DSP: press 0-9");
    return;
  }

  // Direct summation key.
  if (strcmp(key, "SIGMA+") == 0) {
    doSigmaPlus();
    return;
  }

  // (i) alone recalls the register indirectly addressed by I.
  if (strcmp(key, "(i)") == 0) {
    doRecallIndirect();
    return;
  }

  // Program-control keys in RUN mode.
  if (strcmp(key, "GTO") == 0) {
    waitingForProgramLabel = true;
    pendingProgramCommand = 'G';
    Serial.println("GTO: press 0-9 or A-E");
    return;
  }

  if (strcmp(key, "SST") == 0) {
    if (!programRunning) {
      executeProgramStep();
    }
    return;
  }

  if (strcmp(key, "R/S") == 0) {
    if (programRunning) stopProgramRun("R/S halt");
    else startProgramRun();
    return;
  }

  // Any other key cancels unused prefix
  prefixF = false;
  prefixG = false;
  prefixH = false;

  // --------------------------------------------------
  // Digits
  // --------------------------------------------------

  if (strlen(key) == 1 && key[0] >= '0' && key[0] <= '9') {
    enterDigit(key[0]);
    return;
  }

  if (strcmp(key, ".") == 0) {
    enterDecimal();
    return;
  }

  if (strcmp(key, "ENTER") == 0) { doEnter(); return; }
  if (strcmp(key, "CHS") == 0)   { doCHS(); return; }
  if (strcmp(key, "EEX") == 0)   { doEEX(); return; }
  if (strcmp(key, "CLX") == 0)   { doCLX(); return; }
  if (strcmp(key, "+") == 0)     { doOperation('+'); return; }
  if (strcmp(key, "-") == 0)     { doOperation('-'); return; }
  if (strcmp(key, "*") == 0)     { doOperation('*'); return; }
  if (strcmp(key, "/") == 0)     { doOperation('/'); return; }
  if (strcmp(key, "A") == 0 || strcmp(key, "B") == 0 ||
      strcmp(key, "C") == 0 || strcmp(key, "D") == 0 || strcmp(key, "E") == 0) {
    // Original HP-67 top-row behavior:
    //   * With program memory completely empty, A-E are shortcuts for
    //     1/x, sqrt(x), y^x, R-down, and X<>Y.
    //   * As soon as ANY program step exists, A-E become user-program
    //     keys and execute the corresponding LBL A-E routine.
    // When encountered by a running program, A-E act like GSB A-E.
    if (programSize <= 0) {
      if (strcmp(key, "A") == 0) { doReciprocal(); return; }
      if (strcmp(key, "B") == 0) { doSqrt(); return; }
      if (strcmp(key, "C") == 0) { doYtoX(); return; }
      if (strcmp(key, "D") == 0) { doRollDown(); return; }
      if (strcmp(key, "E") == 0) { doSwapXY(); return; }
    }

    char label = key[0];
    int where = findProgramLabel(label);
    if (where < 0) {
      Serial.print("ERROR: user key label not found: ");
      Serial.println(label);
      if (programRunning) stopProgramRun("USER KEY ERROR");
      return;
    }

    if (programRunning) {
      // executeProgramStep() has already advanced programPC, so save that
      // address as the return point exactly as GSB does.
      if (callDepth >= 4) {
        Serial.println("ERROR: subroutine nest overflow");
        stopProgramRun("CALL ERROR");
        return;
      }
      callStack[callDepth++] = programPC;
      programPC = where + 1;
      return;
    }

    // Manual press of A-E in RUN mode starts that user-defined routine.
    startProgramAtLabel(label);
    return;
  }

  Serial.print("Not implemented yet: ");
  Serial.println(key);
}

// --------------------------------------------------
// Scan keyboard
// --------------------------------------------------

// --------------------------------------------------
// Keyboard debounce
// --------------------------------------------------




// --------------------------------------------------
// Scan keyboard
// --------------------------------------------------

void scanKeyboard() {

  for (int r = 0; r < 8; r++) {

    // All rows inactive
    for (int rr = 0; rr < 8; rr++) {
      digitalWrite(rowPins[rr], HIGH);
    }

    // Activate current row
    digitalWrite(rowPins[r], LOW);
    delayMicroseconds(20);

    for (int c = 0; c < 5; c++) {

      bool pressedNow = (digitalRead(colPins[c]) == LOW);

      if (!keyLatched[r][c]) {

        // Integrating press qualifier: does NOT require consecutive LOWs.
        // This tolerates the intermittent 0/1 behavior seen on keys 1,2,3.
        if (pressedNow) {
          if (pressScore[r][c] < 4) pressScore[r][c]++;
        } else {
          if (pressScore[r][c] > 0) pressScore[r][c]--;
        }

        if (pressScore[r][c] >= 3) {
          keyLatched[r][c] = true;
          pressScore[r][c] = 0;
          releaseStart[r][c] = 0;
          processKey(keyMap[r][c]);
        }
      }
      else {
        // Once a key has fired, do not allow another press until it has
        // been continuously released for 50 ms.  Brief contact dropouts
        // while the key is held therefore cannot create duplicate digits.
        if (pressedNow) {
          releaseStart[r][c] = 0;
        } else {
          if (releaseStart[r][c] == 0) {
            releaseStart[r][c] = millis();
          }
          else if ((millis() - releaseStart[r][c]) >= releaseDebounceTime) {
            keyLatched[r][c] = false;
            releaseStart[r][c] = 0;
          }
        }
      }
    }
  }
}



// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup() {

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("HP-67 ESP32 RPN Calculator");
  Serial.println("*** V17 ORIGINAL HP-67 A-E USER-KEY FIX ***");
  Serial.println();

  // TM1640 display
  ledDisplay.begin(true, 2);
  clearLedDisplay();

  loadProgramMemory();

  // Physical W/PRGM-RUN switch: active LOW.
  pinMode(PROGRAM_RUN_SWITCH_PIN, INPUT_PULLUP);
  programSwitchRawState = digitalRead(PROGRAM_RUN_SWITCH_PIN);
  programSwitchStableState = programSwitchRawState;
  programSwitchLastChangeMs = millis();
  programMode = (programSwitchStableState == LOW);
  programRunning = false;
  progCursor = programSize;


  // Rows = outputs

  for (int r = 0; r < 8; r++) {

    pinMode(rowPins[r], OUTPUT);

    digitalWrite(rowPins[r], HIGH);
  }


  // Columns = inputs with pullups

  for (int c = 0; c < 5; c++) {

    pinMode(colPins[c], INPUT_PULLUP);
  }


  if (programMode) {
    Serial.println("Startup switch state: PRGM");
    printProgramPosition("startup");
  } else {
    Serial.println("Startup switch state: RUN");
    showStack();
  }
  Serial.print("Program steps loaded: ");
  Serial.println(programSize);
  Serial.println("Programming controls:");
  Serial.println("  Physical W/PRGM-RUN switch on GPIO 25 controls PRGM/RUN");
  Serial.println("  PRGM f SST + label = LBL");
  Serial.println("  PRGM GTO + label = GTO");
  Serial.println("  PRGM f GTO + label = GSB");
  Serial.println("  PRGM h GTO = RTN");
  Serial.println("  PRGM R/S = store stop step");
  Serial.println("  PRGM SST / h SST = review forward/back (BST)");
  Serial.println("  PRGM f CLX = CL PRGM (clear all program memory)");
  Serial.println("  PRGM h CLX = DEL (delete displayed program step)");
  Serial.println("  RUN A-E = math shortcuts only when program memory is empty; otherwise LBL A-E");
}


// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop() {

  updateProgramRunSwitch();
  scanKeyboard();

  if (programRunning && !programMode) {
    unsigned long now = millis();
    if ((now - lastProgramStepTime) >= programStepIntervalMs) {
      lastProgramStepTime = now;
      executeProgramStep();
    }
  }

  delay(2);
}