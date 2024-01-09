#include <PS3BT.h>
#include <usbhub.h>
#include <Sabertooth.h>
#include <Adafruit_TLC5947.h>
#include <MP3Trigger.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>

// For LED control, define the data ports
//***************************************************

#define clock 5
#define data 4
#define latch 6

// Declare the LED Controller
Adafruit_TLC5947 LEDControl = Adafruit_TLC5947(1, clock, data, latch);

int ledMaxBright = 4000; // 4095 is MAX brightness

// ---------------------------------------------------------------------------------------
//                 Setup for USB, Bluetooth Dongle, & PS3 Controller
// ---------------------------------------------------------------------------------------
USB Usb;
BTD Btd(&Usb);
PS3BT *PS3Controller = new PS3BT(&Btd);

// ---------------------------------------------------------------------------------------
//    Used for PS3 Fault Detection
// ---------------------------------------------------------------------------------------
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;
byte joystickDeadZoneRange = 15;

boolean isPS3ControllerInitialized = false;
boolean mainControllerConnected = false;
boolean WaitingforReconnect = false;
boolean isFootMotorStopped = true;

// ---------------------------------------------------------------------------------------
//    Used for PS3 Controller Request Management
// ---------------------------------------------------------------------------------------
long previousRequestMillis = millis();
boolean extraRequestInputs = false;

// ---------------------------------------------------------------------------------------
//    Request State Machine Variables for PS3 Controller
// ---------------------------------------------------------------------------------------

// Main state varable to determine if a request has been made by the PS3 Controller
boolean reqMade = false;
boolean reqLeftJoyMade = false;
boolean reqRightJoyMade = false;

// LEFT & RIGHT Joystick State Request Values
boolean reqLeftJoyUp = false;
boolean reqLeftJoyDown = false;
int reqLeftJoyYValue = 0;

boolean reqLeftJoyLeft = false;
boolean reqLeftJoyRight = false;
int reqLeftJoyXValue = 0;

boolean reqRightJoyUp = false;
boolean reqRightJoyDown = false;
int reqRightJoyYValue = 0;

boolean reqRightJoyLeft = false;
boolean reqRightJoyRight = false;
int reqRightJoyXValue = 0;

// PS3 Controller Button State Variables
boolean reqArrowUp = false;
boolean reqArrowDown = false;
boolean reqArrowLeft = false;
boolean reqArrowRight = false;
boolean reqCircle = false;
boolean reqCross = false;
boolean reqTriangle = false;
boolean reqSquare = false;
boolean reqL1 = false;
boolean reqL2 = false;
boolean reqR1 = false;
boolean reqR2 = false;
boolean reqSelect = false;
boolean reqStart = false;
boolean reqPS = false;

int driveDeadBandRange = 10;
#define SABERTOOTH_ADDR 128
Sabertooth *ST = new Sabertooth(SABERTOOTH_ADDR, Serial1); //TX1 – Pin#18

int currentSpeed = 0;
int currentTurn = 0;
boolean droidMoving = false;

// Constants for different stages of the routine
const int PICKUP_SIGNAL_STAGE_1 = 1;
const int PICKUP_SIGNAL_STAGE_2 = 2;
const int PICKUP_SIGNAL_STAGE_3 = 3;
const int PICKUP_SIGNAL_STAGE_4 = 4;

// Variable to keep track of the current stage
int currentPickupSignalStage = PICKUP_SIGNAL_STAGE_1;

// Timing variables
unsigned long pickupSignalStartTime;
unsigned long currentPickupSignalTime;

// Duration for each stage in milliseconds
unsigned long stage1Duration = 2000; // Initial value for moving forward
const unsigned long stage2Duration = 1000; // Pause
const unsigned long stage3Duration = 2000; // Move backward
const unsigned long stage4Duration = 1000; // Final pause

// Counter to change stage 1 behavior over time
int stage1VariationCounter = 0;

unsigned long routine1StartTime = 0;
const unsigned long routine1Duration = 60000; // 10 seconds
bool routine1 = false;

// ---------------------------------------------------------------
// SONARS

NewPing frontSonar = NewPing(48,49); //Trig on Pin#34, Echo on Pin#35)
NewPing left1Sonar = NewPing(46,47); //Trig on Pin#46, Echo on Pin#47)
NewPing left2Sonar = NewPing(44,45); //Trig on Pin#46, Echo on Pin#47)
NewPing backSonar = NewPing(42,43); //Trig on Pin#40, Echo on Pin#41)

boolean autoMode = true;
int sonarReadCycle = 1;
long sonarIntervalTimer = millis();
int sonarIntervalTime = 300; // Take a sonar reading every 300ms
int currentFrontDistance = 0; // Distance captured in CM
int currentLeft1Distance = 0; // Distance captured in CM
int currentLeft2Distance = 0; // Distance captured in CM
int currentBackDistance = 0; // Distance captured in CM
int tapeDistance = 0; // Distance from wall to tape in CM

int startLeft1Distance; // Distance captured in CM
int startLeft2Distance; // Distance captured in CM

boolean autoPilot = false;
unsigned long autoPilotStartTime;

boolean backPilot = false;
int pilotCounter = 0;

unsigned long lastTurnTime = 0; // Global variable to store the last turn time
const unsigned long turnDelay = 5000; // Delay time in milliseconds (e.g., 5 seconds)
int turnCount = 0; // Global variable to track the number of turns



// ---------------------------------------------------------------

// ---------------------------------------------------------------------------------------
//    Used for Pin 13 Main Loop Blinker
// ---------------------------------------------------------------------------------------
long blinkMillis = millis();
boolean blinkOn = false;

MP3Trigger MP3Trigger;

long soundTimer;
long songTime;
boolean ambientSound = false;
int randomWait;

int routine = 0;


// =======================================================================================
//                                 Main Program
// =======================================================================================
// =======================================================================================
//                                Setup Function
// =======================================================================================
void setup()
{

  //Initialize Serial @ 115200 baud rate for Serial Monitor Debugging
  Serial.begin(115200);
  while (!Serial);

  //Initialize the USB Dongle and check for errors
  if (Usb.Init() == -1)
  {
    Serial.println("OSC did not start");
    while (1); //halt
  }

  Serial.println("Bluetooth Library Started");

  //PS3 Controller - sets the interrupt function to call when PS3 controller tries to connect
  PS3Controller->attachOnInit(onInitPS3Controller);

  //Setup PIN 13 for Arduino Main Loop Blinker Routine
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // ----------------------------------------------
  // YOUR SETUP CONTROL CODE SHOULD START HERE
  // ----------------------------------------------

  Serial1.begin(9600); //Start TX1 – Pin#18 – Motor Controller
  ST->autobaud();
  ST->setTimeout(200);
  ST->setDeadband(driveDeadBandRange);

  MP3Trigger.setup(&Serial2);
  Serial2.begin(MP3Trigger::serialRate());

  //StarttheLEDController
  LEDControl.begin();


  // ----------------------------------------------
  // YOUR SETUP CONTROL CODE SHOULD END HERE
  // ---------------------------------------------
}

// =======================================================================================
//    Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================
void loop()
{
  // Make sure the PS3 Controller is working - skip main loop if not
  if ( !readUSB() )
  {
    return;
  }

  // If the PS3 controller has been connected - start processing the main controller routines
  if (PS3Controller->PS3Connected) {

    // Read the PS3 Controller and set request state variables for this loop
    readPS3Request();
    //moveDroid();

    if (reqMade) {
      // Toggle autoMode on/off using CROSS button on PS3 Controller
      if (reqCross) {
        if (autoMode) {
          autoMode = false;
        } 
        else {
          autoMode = true;
          sonarIntervalTimer = millis();
        }
      }
    }

    // SELF DRIVING
    selectAutoPilot();
    apMainControl();
    backMainControl();
    
    MP3Trigger.update();

    // Ignore extra inputs from the PS3 Controller for 1/2 second from prior input
    if (extraRequestInputs)
    {
      if ((previousRequestMillis + 500) < millis())
      {
        extraRequestInputs = false;
      }
    }

    // If there was a PS3 request this loop - reset the request variables for next loop
    if (reqMade) {
      resetRequestVariables();
      reqMade = false;
    }

    // If autoMode is true - start taking Sonar Readings
    if (autoMode) {
       takeSonarReadings();
    }

  }

  // Blink to show working heart beat on the Arduino control board
  // If Arduino LED is not blinking - the sketch has crashed
  if ((blinkMillis + 500) < millis()) {
    if (blinkOn) {
      digitalWrite(13, LOW);
      blinkOn = false;
    } else {
      digitalWrite(13, HIGH);
      blinkOn = true;
    }
    blinkMillis = millis();
  }
}

// =======================================================================================
//      ADD YOUR CUSTOM DROID FUNCTIONS STARTING HERE
// =======================================================================================


void takeSonarReadings() {
  if ((sonarIntervalTimer + sonarIntervalTime) > millis()) {
    return;
  } else {
    sonarIntervalTimer = millis();
  }

  if (sonarReadCycle == 1) {
    currentFrontDistance = frontSonar.convert_cm(frontSonar.ping_median(5));
    //Serial.println("***********FRONT Sonar*************");
    //Serial.print("Front Sonar: ");
    //Serial.println(currentFrontDistance);
  }

  if (sonarReadCycle == 2) {
    currentLeft1Distance = left1Sonar.convert_cm(left1Sonar.ping_median(5));
    //Serial.println("***********LEFT 1 Sonar*************");
    //Serial.print("Left 1 Sonar: ");
    //Serial.println(currentLeft1Distance);
  }

  if (sonarReadCycle == 3) {
    currentLeft2Distance = left2Sonar.convert_cm(left2Sonar.ping_median(5));
    //Serial.println("***********LEFT 2 Sonar*************");
    //Serial.print("Left 2 Sonar: ");
    //Serial.println(currentLeft2Distance);
  }

  if (sonarReadCycle == 4) {
    currentBackDistance = backSonar.convert_cm(backSonar.ping_median(5));
    //Serial.println("***********BACK Sonar*************");
    //Serial.print("Back Sonar: ");
    //Serial.println(currentBackDistance);
  }

  sonarReadCycle++;
  if (sonarReadCycle == 5) {
    sonarReadCycle = 1;
  }
}


void selectAutoPilot(){
  if (reqSquare){
    startAutoPilot();
  }
}

void selectBackAutoPilot(){
  if (reqTriangle){
    startBackAutoPilot();
  }
}

void startBackAutoPilot() {
  if (!backPilot) {

    backPilot = true;

    startLeft1Distance = currentLeft1Distance;
    startLeft2Distance = currentLeft2Distance;
    
  }
  else{
    backPilot = false;
  }
}



void startAutoPilot() {
  if (!autoPilot) {
    autoPilotStartTime = millis(); // Record the start time
    autoPilot = true;

    startLeft1Distance = currentLeft1Distance;
    startLeft2Distance = currentLeft2Distance;

    //Serial.println(startLeft1Distance);
    //Serial.println(startLeft2Distance);
    
    Serial.println("***********PILOT TRUE*************");
  }
  else{
    autoPilot = false;
    Serial.println("***********PILOT FALSE*************");
  }
}



void apMainControl(){

  if (autoPilot){

    // Tolerance for alignment in centimeters
    int alignmentTolerance = 3;

    // Distance threshold for the front distance
    int frontDistanceThreshold = startLeft1Distance - 4;

    // Check if the front distance is less than the threshold
    if (currentFrontDistance < frontDistanceThreshold) {
      if (millis() - lastTurnTime > turnDelay) {
        if (turnCount < 3) {
          turnCount++;
          lastTurnTime = millis(); // Update the last turn time
        } else {
          // If turn count is 3 and another turn is needed, stop the autopilot
          autoPilot = false;
          backPilot = true;
          turnCount = 0;
        }
      }
      turnDroid();
    } else {
    if (abs(currentLeft1Distance - currentLeft2Distance) <= alignmentTolerance) {
      // The droid is aligned, move forward
      moveDroidForward();
    } else {
      // The droid is not aligned, adjust its position
      if (currentLeft1Distance > currentLeft2Distance) {
        // The front of the droid is farther from the wall than the back, turn left
        correctLeft();
      } else {
        // The front of the droid is closer to the wall than the back, turn right
        correctRight();
      }
    }

    }
    
  }
  
}



void backMainControl(){

  if (backPilot){


    // Tolerance for alignment in centimeters
    int alignmentTolerance = 3;

    // Distance threshold for the front distance
    int backDistanceThreshold = 60;

    // Check if the front distance is less than the threshold
    if (currentBackDistance < backDistanceThreshold) {
      // If the front distance is too close, turn the droid

      if (millis() - lastTurnTime > turnDelay) {
        if (turnCount < 3) {
          turnCount++;
          lastTurnTime = millis(); // Update the last turn time
        } else {
          // If turn count is 3 and another turn is needed, stop the autopilot
          backPilot = false;
        }
      }
      backturnDroid();

      
    } else {
    if (abs(currentLeft1Distance - currentLeft2Distance) <= alignmentTolerance) {
      // The droid is aligned, move forward
      backmoveDroidForward();
    } else {
      // The droid is not aligned, adjust its position
      if (currentLeft1Distance > currentLeft2Distance) {
        // The front of the droid is farther from the wall than the back, turn left
        backcorrectRight();
      } else {
        // The front of the droid is closer to the wall than the back, turn right
        backcorrectLeft();
      }
    }

    }
    
  }
  
}



void turnDroid(){
  ST->drive(0);
  ST->turn(65);
}

void moveDroidForward(){
  ST->drive(-40);
  ST->turn(0);
}

void correctLeft(){
  ST->drive(-15);
  ST->turn(-20);
}

void correctRight(){
  ST->drive(-15);
  ST->turn(20);
}





void backturnDroid(){
  ST->drive(0);
  ST->turn(-50);
}

void backmoveDroidForward(){
  ST->drive(40);
  ST->turn(0);
}

void backcorrectLeft(){
  ST->drive(10);
  ST->turn(25);
}

void backcorrectRight(){
  ST->drive(10);
  ST->turn(-25);
}



































void selectRoutine(){
  if (reqCross){
    startRoutine1();
  }
}

void startRoutine1() {
  if (!routine1) {
    routine1StartTime = millis(); // Record the start time
    routine1 = true;
  }
}

void soundRoutine1(){
  static bool songStarted = false;  // Tracks if the song has started
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - routine1StartTime;

  // Check if we are within any of the specified time intervals
  bool withinInterval = (elapsedTime > 11000 && elapsedTime < 14000) ||
                        (elapsedTime > 26300 && elapsedTime < 29600) ||
                        (elapsedTime > 41900 && elapsedTime < 45200);

  if (routine1 && withinInterval) {
    if (!songStarted) {
      MP3Trigger.trigger(37);  // Start playing the song
      songStarted = true;
    }

    // Check if 3 seconds have passed since the song started in each interval
    if ((elapsedTime > 14000 && elapsedTime < 16300) ||
        (elapsedTime > 29600 && elapsedTime < 31900) ||
        (elapsedTime > 45200 && elapsedTime < 47500)) {
      MP3Trigger.stop();  // Stop the song
      songStarted = false; // Reset the flag
    }
  } else {
    songStarted = false;  // Reset the flag if the condition is not met
  }
}


// right side 19-23
// left side 0-4
void ledRoutine1(){
  static unsigned long lastBlinkTime = 0;  // Tracks the last time the LED state changed
  static bool ledOn = false;              // Tracks if the LED is currently on
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - routine1StartTime;

  // Check if we are within any of the specified time intervals
  bool withinInterval = (elapsedTime > 11000 && elapsedTime < 15500) ||
                        (elapsedTime > 26300 && elapsedTime < 30800) ||
                        (elapsedTime > 41900 && elapsedTime < 46400);

  if (routine1 && withinInterval) {
    if (currentTime - lastBlinkTime >= 500) {  // 0.5 seconds have passed
      ledOn = !ledOn;  // Toggle LED state
      int brightness = ledOn ? ledMaxBright : 0;
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.write();
      lastBlinkTime = currentTime;  // Update the last blink time
    }
  } else {
    LEDControl.setPWM(19, 0);
    LEDControl.setPWM(20, 0);
    LEDControl.setPWM(21, 0);
    LEDControl.setPWM(22, 0);
    LEDControl.setPWM(23, 0);
    LEDControl.write();
    ledOn = false;  // Reset the state
    lastBlinkTime = currentTime; // Reset the last blink time
  }
}


void moveRoutine1() {
  if (routine1) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - routine1StartTime;

    if (elapsedTime < routine1Duration) {
      if (elapsedTime < 11000) {
        // Turn for 11 seconds
        ST->drive(-25);
        ST->turn(0); // Adjust the turning angle as needed
      } else if (elapsedTime < 14000) {
        // Stop for 3 seconds before moving forward
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 16300) {
        // Turn for 2.3 seconds
        ST->drive(0);
        ST->turn(32); // Adjust the turning angle as needed
      } else if (elapsedTime < 26300) {
        // Move forward for 9 seconds
        ST->drive(-25);
        ST->turn(0);
      } else if (elapsedTime < 29600) {
        // Stop for 3 seconds before turning
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 31900) {
        // Turn for 2.3 seconds
        ST->drive(0);
        ST->turn(32); // Adjust the turning angle as needed
      } else if (elapsedTime < 41900) {
        // Move forward for 8 seconds
        ST->drive(-25);
        ST->turn(0);
      } else if (elapsedTime < 45200) {
        // Stop for 3 seconds before turning
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 47500) {
        // Turn for 2.3 seconds
        ST->drive(0);
        ST->turn(32); // Adjust the turning angle as needed
      } else if (elapsedTime < 56000) {
        // Move forward for 8 seconds
        ST->drive(-25);
        ST->turn(0);
      } else {
        // Stop after completing the sequence
        ST->stop(); // Stop the droid
        routine1 = false;
      }
    } else {
      ST->stop(); // Stop the droid if the duration is exceeded
      routine1 = false;
    }
  }
}




void moveDroid() {
  if (reqLeftJoyMade) {
    if (reqLeftJoyYValue > 70 || reqLeftJoyYValue < -70) {
      currentSpeed = reqLeftJoyYValue / 2;
    }
    else {
      currentSpeed = reqLeftJoyYValue;
    }
    if (reqLeftJoyXValue > 70 || reqLeftJoyXValue < -70) {
      currentTurn = reqLeftJoyXValue / 3;
    } else {
      currentTurn = reqLeftJoyXValue;
    }
    //currentSpeed = reqLeftJoyYValue;
    //currentTurn = reqLeftJoyXValue;
    Serial.println(currentSpeed);
    Serial.println(currentTurn);
    ST->turn(currentTurn);
    ST->drive(currentSpeed);
    if (!droidMoving) {
      droidMoving = true;
    }
  } else {
    if (droidMoving) {
      ST->stop();
      droidMoving = false;
      currentTurn = 0;
      currentSpeed = 0;
    }
  }
}


void ambientControl(){

  if (reqArrowUp) {
      Serial.println(ambientSound);
      if (ambientSound == false){
        ambientSound = true;       
        soundTimer = millis(); 
        MP3Trigger.trigger(random(1,5));
        randomWait = random(5, 11);
      }
      else{
        ambientSound = false;
        MP3Trigger.stop();
      }
  }
  
}

void ambientPlay(){

  if (ambientSound && (soundTimer + (randomWait * 1000)) < millis()){
    MP3Trigger.trigger(random(1,5));
    soundTimer = millis();
    randomWait = random(5, 11);
  }
  
}


// SAMPLE CUSTOM DROID FUNCTION from PS3 REQUEST - REMOVE ONCE YOU UNDERSTAND STRUCTURE
void callMyArrowUpFunction()
{
  Serial.println("Droid is now executing my custom ARROW UP function");
}



// =======================================================================================
//      YOUR CUSTOM DROID FUNCTIONS SHOULD END HERE
// =======================================================================================

// =======================================================================================
//      CORE DROID CONTROL FUNCTIONS START HERE - EDIT WITH CAUTION
// =======================================================================================
// Read the PS3 Controller and set request state variables
void readPS3Request()
{
  if (!extraRequestInputs) {

    if (PS3Controller->getButtonPress(UP))
    {
      Serial.println("Button: UP Selected");

      reqArrowUp = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(DOWN))
    {
      Serial.println("Button: DOWN Selected");

      reqArrowDown = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(LEFT))
    {
      Serial.println("Button: LEFT Selected");

      reqArrowLeft = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(RIGHT))
    {
      Serial.println("Button: RIGHT Selected");

      reqArrowRight = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(CIRCLE))
    {
      Serial.println("Button: CIRCLE Selected");

      reqCircle = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(CROSS))
    {
      Serial.println("Button: CROSS Selected");

      reqCross = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(TRIANGLE))
    {
      Serial.println("Button: TRIANGLE Selected");

      reqTriangle = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }


    if (PS3Controller->getButtonPress(SQUARE))
    {
      Serial.println("Button: SQUARE Selected");

      reqSquare = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;

    }

    if (PS3Controller->getButtonPress(L1))
    {
      Serial.println("Button: LEFT 1 Selected");

      reqL1 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(L2))
    {
      Serial.println("Button: LEFT 2 Selected");

      reqL2 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(R1))
    {
      Serial.println("Button: RIGHT 1 Selected");

      reqR1 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(R2))
    {
      Serial.println("Button: RIGHT 2 Selected");

      reqR2 = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(SELECT))
    {
      Serial.println("Button: SELECT Selected");

      reqSelect = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(START))
    {
      Serial.println("Button: START Selected");

      reqStart = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }

    if (PS3Controller->getButtonPress(PS))
    {
      Serial.println("Button: PS Selected");

      reqPS = true;
      reqMade = true;

      previousRequestMillis = millis();
      extraRequestInputs = true;
    }
  }

  if (((abs(PS3Controller->getAnalogHat(LeftHatY) - 128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(LeftHatX) - 128) > joystickDeadZoneRange)))
  {
    reqLeftJoyUp = false;
    reqLeftJoyDown = false;
    reqLeftJoyLeft = false;
    reqLeftJoyRight = false;
    reqLeftJoyYValue = 0;
    reqLeftJoyXValue = 0;
    reqLeftJoyMade = true;

    int currentValueY = PS3Controller->getAnalogHat(LeftHatY) - 128;
    int currentValueX = PS3Controller->getAnalogHat(LeftHatX) - 128;

    char yString[5];
    itoa(currentValueY, yString, 10);

    char xString[5];
    itoa(currentValueX, xString, 10);

    Serial.print("LEFT Joystick Y Value: ");
    Serial.println(yString);
    Serial.print("LEFT Joystick X Value: ");
    Serial.println(xString);

    if (currentValueY > joystickDeadZoneRange) {
      Serial.println("Left Joystick DOWN");
      reqLeftJoyDown = true;
      reqLeftJoyYValue = currentValueY;
    }

    if (currentValueY < (-1 * joystickDeadZoneRange)) {
      Serial.println("Left Joystick UP");
      reqLeftJoyUp = true;
      reqLeftJoyYValue = currentValueY;
    }

    if (currentValueX > joystickDeadZoneRange) {
      Serial.println("Left Joystick RIGHT");
      reqLeftJoyRight = true;
      reqLeftJoyXValue = currentValueX;
    }

    if (currentValueX < (-1 * joystickDeadZoneRange)) {
      Serial.println("Left Joystick LEFT");
      reqLeftJoyLeft = true;
      reqLeftJoyXValue = currentValueX;
    }
  } else {
    if (reqLeftJoyMade) {
      reqLeftJoyUp = false;
      reqLeftJoyDown = false;
      reqLeftJoyLeft = false;
      reqLeftJoyRight = false;
      reqLeftJoyYValue = 0;
      reqLeftJoyXValue = 0;
      reqLeftJoyMade = false;
    }
  }

  if (((abs(PS3Controller->getAnalogHat(RightHatY) - 128) > joystickDeadZoneRange) || (abs(PS3Controller->getAnalogHat(RightHatX) - 128) > joystickDeadZoneRange)))
  {
    reqRightJoyUp = false;
    reqRightJoyDown = false;
    reqRightJoyLeft = false;
    reqRightJoyRight = false;
    reqRightJoyYValue = 0;
    reqRightJoyXValue = 0;
    reqRightJoyMade = true;

    int currentValueY = PS3Controller->getAnalogHat(RightHatY) - 128;
    int currentValueX = PS3Controller->getAnalogHat(RightHatX) - 128;

    char yString[5];
    itoa(currentValueY, yString, 10);

    char xString[5];
    itoa(currentValueX, xString, 10);

    Serial.print("RIGHT Joystick Y Value: ");
    Serial.println(yString);
    Serial.print("RIGHT Joystick X Value: ");
    Serial.println(xString);

    if (currentValueY > joystickDeadZoneRange) {
      Serial.println("Right Joystick DOWN");
      reqRightJoyDown = true;
      reqRightJoyYValue = currentValueY;
    }

    if (currentValueY < (-1 * joystickDeadZoneRange)) {
      Serial.println("Right Joystick UP");
      reqRightJoyUp = true;
      reqRightJoyYValue = currentValueY;
    }

    if (currentValueX > joystickDeadZoneRange) {
      Serial.println("Right Joystick RIGHT");
      reqRightJoyRight = true;
      reqRightJoyXValue = currentValueX;
    }

    if (currentValueX < (-1 * joystickDeadZoneRange)) {
      Serial.println("Right Joystick LEFT");
      reqRightJoyLeft = true;
      reqRightJoyXValue = currentValueX;
    }
  } else {
    if (reqRightJoyMade) {
      reqRightJoyUp = false;
      reqRightJoyDown = false;
      reqRightJoyLeft = false;
      reqRightJoyRight = false;
      reqRightJoyYValue = 0;
      reqRightJoyXValue = 0;
      reqRightJoyMade = false;
    }
  }
}

// Reset the PS3 request variables on every processing loop when needed
void resetRequestVariables()
{
  reqArrowUp = false;
  reqArrowDown = false;
  reqArrowLeft = false;
  reqArrowRight = false;
  reqCircle = false;
  reqCross = false;
  reqTriangle = false;
  reqSquare = false;
  reqL1 = false;
  reqL2 = false;
  reqR1 = false;
  reqR2 = false;
  reqSelect = false;
  reqStart = false;
  reqPS = false;
}

// Initialize the PS3 Controller Trying to Connect
void onInitPS3Controller()
{
  PS3Controller->setLedOn(LED1);
  isPS3ControllerInitialized = true;
  badPS3Data = 0;

  mainControllerConnected = true;
  WaitingforReconnect = true;

  Serial.println("We have the controller connected");
  Serial.print("Dongle Address: ");
  String dongle_address = String(Btd.my_bdaddr[5], HEX) + ":" + String(Btd.my_bdaddr[4], HEX) + ":" + String(Btd.my_bdaddr[3], HEX) + ":" + String(Btd.my_bdaddr[2], HEX) + ":" + String(Btd.my_bdaddr[1], HEX) + ":" + String(Btd.my_bdaddr[0], HEX);
  Serial.println(dongle_address);
}

// Determine if we are having connection problems with the PS3 Controller
boolean criticalFaultDetect()
{
  if (PS3Controller->PS3Connected)
  {

    currentTime = millis();
    lastMsgTime = PS3Controller->getLastMessageTime();
    msgLagTime = currentTime - lastMsgTime;

    if (WaitingforReconnect)
    {
      if (msgLagTime < 200)
      {
        WaitingforReconnect = false;
      }
      lastMsgTime = currentTime;
    }

    if ( currentTime >= lastMsgTime)
    {
      msgLagTime = currentTime - lastMsgTime;
    } else
    {
      msgLagTime = 0;
    }

    if ( msgLagTime > 5000 )
    {
      Serial.println("It has been 5s since we heard from Controller");
      Serial.println("Disconnecting the controller");

      PS3Controller->disconnect();
      WaitingforReconnect = true;
      return true;
    }

    //Check PS3 Signal Data
    if (!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
    {
      //We don't have good data from the controller.
      //Wait 15ms - try again
      delay(15);
      Usb.Task();
      lastMsgTime = PS3Controller->getLastMessageTime();

      if (!PS3Controller->getStatus(Plugged) && !PS3Controller->getStatus(Unplugged))
      {
        badPS3Data++;
        Serial.println("**Invalid data from PS3 Controller. - Resetting Data**");
        return true;
      }
    }
    else if (badPS3Data > 0)
    {

      badPS3Data = 0;
    }

    if ( badPS3Data > 10 )
    {
      Serial.println("Too much bad data coming from the PS3 Controller");
      Serial.println("Disconnecting the controller");

      PS3Controller->disconnect();
      WaitingforReconnect = true;
      return true;
    }
  }

  return false;
}

// USB Read Function - Supports Main Program Loop
boolean readUSB()
{
  Usb.Task();
  //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
  if (PS3Controller->PS3Connected)
  {
    if (criticalFaultDetect())
    {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      return false;
    }

  }
  return true;
}
