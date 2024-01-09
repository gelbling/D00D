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

#include <Servo.h>
Servo myServo;

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


// ROUTINE 3
unsigned long routine3StartTime = 0;
const unsigned long routine3Duration = 50000; // 10 seconds
bool routine3 = false;
bool isServoOpen3 = false;

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


// ROUTINE 2
bool isServoOpen2 = false;
unsigned long routine2StartTime = 0;
const unsigned long routine2Duration = 50000; // 10 seconds
bool routine2 = false;


// ROUTINE 1
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

  // SERVO PIN
  myServo.attach(8);

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

  myServo.write(0); 


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


    // ----------------------------------------------
    // YOUR MAIN LOOP CONTROL CODE SHOULD START HERE
    // ----------------------------------------------


    // UPDATE SOUND
    MP3Trigger.update();

    // SELF DRIVING
    selectAutoPilot();
    apMainControl();
    backMainControl();

    // ROUTINE 1
    selectRoutine1();
    moveRoutine1();
    soundRoutine1();
    ledRoutine1();

    // ROUTINE 2
    selectRoutine2();
    moveRoutine2();
    soundRoutine2();
    ledRoutine2();
    controlServo2();

    // ROUTINE 3
    selectRoutine3();
    moveRoutine3();
    soundRoutine3();
    ledRoutine3();
    controlServo3();


    // ----------------------------------------------
    // YOUR MAIN LOOP CONTROL CODE SHOULD END HERE
    // ----------------------------------------------

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
    int frontDistanceThreshold = startLeft1Distance;

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
  ST->turn(60);
}

void moveDroidForward(){
  ST->drive(-50);
  ST->turn(0);
}

void correctLeft(){
  ST->drive(-20);
  ST->turn(-30);
}

void correctRight(){
  ST->drive(-20);
  ST->turn(30);
}





void backturnDroid(){
  ST->drive(0);
  ST->turn(-60);
}

void backmoveDroidForward(){
  ST->drive(40);
  ST->turn(0);
}

void backcorrectLeft(){
  ST->drive(20);
  ST->turn(25);
}

void backcorrectRight(){
  ST->drive(20);
  ST->turn(-25);
}







// --------------------------------------------------------------------------------------------------------
// ROUTINE 1

void selectRoutine1(){
  if (reqTriangle){
    startRoutine1();
  }
}

void startRoutine1() {
  if (!routine1) {
    routine1StartTime = millis(); // Record the start time
    routine1 = true;
  }
  else{
    routine1 = false;
  }
}


void soundRoutine1(){
  static bool songStarted = false;  // Tracks if the song has started
  static bool ambientSoundPlaying = false;  // Tracks if the ambient sound is playing
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
      if (ambientSoundPlaying) {
        MP3Trigger.stop();  // Stop the ambient sound
        ambientSoundPlaying = false;
      }
    }

    // Check if 3 seconds have passed since the song started in each interval
    if ((elapsedTime > 14000 && elapsedTime < 16300) ||
        (elapsedTime > 29600 && elapsedTime < 31900) ||
        (elapsedTime > 45200 && elapsedTime < 47500)) {
      MP3Trigger.stop();  // Stop the song
      songStarted = false; // Reset the flag
    }
  } else if (routine1) {
    if (!ambientSoundPlaying) {
      MP3Trigger.trigger(38);  // Start playing ambient sound, assuming 38 is the track number
      ambientSoundPlaying = true;
    }
    songStarted = false;  // Reset the song flag if the condition is not met
  }

  // Check if it's time to stop the ambient sound
  if (ambientSoundPlaying && routine1 && withinInterval) {
    MP3Trigger.stop();  // Stop the ambient sound
    ambientSoundPlaying = false;
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
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);

      LEDControl.write();
      lastBlinkTime = currentTime;  // Update the last blink time
    }
  } else {
    LEDControl.setPWM(19, 0);
    LEDControl.setPWM(20, 0);
    LEDControl.setPWM(21, 0);
    LEDControl.setPWM(22, 0);
    LEDControl.setPWM(23, 0);
    LEDControl.setPWM(9, 0);
    LEDControl.setPWM(8, 0);
    LEDControl.setPWM(10, 0);
    LEDControl.setPWM(11, 0);
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
        ST->drive(-30);
        ST->turn(0); // Adjust the turning angle as needed
      } else if (elapsedTime < 14000) {
        // Stop for 3 seconds before moving forward
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 16300) {
        // Turn for 2.3 seconds
        ST->drive(0);
        ST->turn(45); // Adjust the turning angle as needed
      } else if (elapsedTime < 26300) {
        // Move forward for 9 seconds
        ST->drive(-30);
        ST->turn(0);
      } else if (elapsedTime < 29600) {
        // Stop for 3 seconds before turning
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 31900) {
        // Turn for 2.3 seconds
        ST->drive(0);
        ST->turn(45); // Adjust the turning angle as needed
      } else if (elapsedTime < 41900) {
        // Move forward for 8 seconds
        ST->drive(-30);
        ST->turn(0);
      } else if (elapsedTime < 45200) {
        // Stop for 3 seconds before turning
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 47500) {
        // Turn for 2.3 seconds
        ST->drive(0);
        ST->turn(45); // Adjust the turning angle as needed
      } else if (elapsedTime < 57500) {
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

// ROUTINE 1
// --------------------------------------------------------------------------------------------------------
















// --------------------------------------------------------------------------------------------------------
// ROUTINE 2
void selectRoutine2(){
  if (reqCircle){
    startRoutine2();
  }
}

void startRoutine2() {
  if (!routine2) {
    routine2StartTime = millis(); // Record the start time
    routine2 = true;
  }
  else{
    routine2 = false;
  }
}

void soundRoutine2() {
  static bool soundPlayed[9] = {false}; // Array to track if a sound has been played for each phase

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - routine2StartTime;

  if (routine2) {
    if (elapsedTime < 5000 && !soundPlayed[0]) {
      // FORWARD
      MP3Trigger.trigger(39); // Play sound for FORWARD
      soundPlayed[0] = true;
    } else if (elapsedTime >= 5000 && elapsedTime < 7000 && !soundPlayed[1]) {
      // TURN RIGHT
      MP3Trigger.trigger(41); // Play sound for TURN RIGHT
      soundPlayed[1] = true;
    } else if (elapsedTime >= 7000 && elapsedTime < 9000 && !soundPlayed[2]) {
      // TURN LEFT
      MP3Trigger.trigger(41); // Play sound for TURN LEFT
      soundPlayed[2] = true;
    } else if (elapsedTime >= 9000 && elapsedTime < 14000 && !soundPlayed[3]) {
      // BACKWARDS
      MP3Trigger.trigger(39); // Play sound for BACKWARDS
      soundPlayed[3] = true;
    } else if (elapsedTime >= 14000 && elapsedTime < 16000 && !soundPlayed[4]) {
      // TURN RIGHT
      MP3Trigger.trigger(41); // Play sound for TURN RIGHT
      soundPlayed[4] = true;
    } else if (elapsedTime >= 16000 && elapsedTime < 18000 && !soundPlayed[5]) {
      // TURN LEFT
      MP3Trigger.trigger(41); // Play sound for TURN LEFT
      soundPlayed[5] = true;
    } else if (elapsedTime >= 18000 && elapsedTime < 21000 && !soundPlayed[6]) {
      // TURN RIGHT TO SIGNAL PICKUP
      MP3Trigger.trigger(39); // Play sound for TURN RIGHT TO SIGNAL PICKUP
      soundPlayed[6] = true;
    } else if (elapsedTime >= 21000 && elapsedTime < 31000 && !soundPlayed[7]) {
      // OPEN LATCH, WAIT FOR PICKUP
      MP3Trigger.trigger(40); // Play sound for OPEN LATCH, WAIT FOR PICKUP
      soundPlayed[7] = true;
    } else if (elapsedTime >= 31000 && elapsedTime < 45000 && !soundPlayed[8]) {
      // TURN RIGHT, PICKUP COMPLETE
      MP3Trigger.trigger(42); // Play sound for TURN RIGHT, PICKUP COMPLETE
      soundPlayed[8] = true;
    } 

    // Reset the flags if the movement routine is completed or when it starts again
    if (elapsedTime >= routine2Duration || elapsedTime < 5000) {
      for (int i = 0; i < 9; i++) {
        soundPlayed[i] = false;
      }
    }
  } else {
    // Reset all sound flags if routine2 is not active
    for (int i = 0; i < 9; i++) {
      soundPlayed[i] = false;
    }
  }
}

// right side 19-23
// left side 0-4
void ledRoutine2() {

  static unsigned long lastToggleTime = 0; // Time when the LED was last toggled
  static bool isLedOn = false; // Current state of the LED
  const long flashInterval = 500; // Interval for LED flashing in milliseconds
  const long flashInterval2 = 250; // Interval for LED flashing in milliseconds
  const long flashInterval3 = 100; // Interval for LED flashing in milliseconds
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - routine2StartTime;
  int brightness = 4000; // Adjust this value as needed for your LEDs

  if (routine2) {
    if (elapsedTime < 5000) {
      // FORWARD

      if (currentTime - lastToggleTime > flashInterval) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 7000) {
      // TURN RIGHT
      brightness = 2000;
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(1, 0);
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(4, 0);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 9000) {
      // TURN LEFT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 14000) {
      // TURN LEFT
      if (currentTime - lastToggleTime > flashInterval) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
    }
    else if (elapsedTime < 16000) {
      // TURN RIGHT
      brightness = 2000;
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 18000) {
      // TURN RIGHT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(1, 0);
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(4, 0);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 21000) {
      // TURN RIGHT
      if (currentTime - lastToggleTime > flashInterval2) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 2000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 31000) {
      // TURN RIGHT
      if (currentTime - lastToggleTime > flashInterval2) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, 500);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, 500);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 500);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, 500);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, 500);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 36000) {
      // TURN RIGHT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    }
    else if (elapsedTime < 40000) {
      // TURN RIGHT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(1, 0);
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(4, 0);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    }
    else if (elapsedTime < 45000) {
      // TURN RIGHT
      if (currentTime - lastToggleTime > flashInterval3) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 500;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    }
    // ... Continue this pattern for other movement phases
    else {
      // Default or off state
      brightness = 0; // Turn off or set to default state
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      
      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);
  
      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
    }

    LEDControl.write(); // Update the LED state
  } else {
    // Turn off LEDs if routine2 is not active
    brightness = 0;
    LEDControl.setPWM(19, brightness);
    LEDControl.setPWM(20, brightness);
    LEDControl.setPWM(21, brightness);
    LEDControl.setPWM(22, brightness);
    LEDControl.setPWM(23, brightness);
    LEDControl.setPWM(0, brightness);
    LEDControl.setPWM(1, brightness);
    LEDControl.setPWM(2, brightness);
    LEDControl.setPWM(3, brightness);
    LEDControl.setPWM(4, brightness);

    
    LEDControl.setPWM(9, brightness);
    LEDControl.setPWM(8, brightness);

    LEDControl.setPWM(10, brightness);
    LEDControl.setPWM(11, brightness);
      
    LEDControl.write();
  }
}


void controlServo2() {
  if (routine2) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - routine2StartTime;
  
    if (!isServoOpen2 && elapsedTime >= 21000 && elapsedTime < 31000) {
      // Open the servo at 18 seconds
      myServo.write(180);
      isServoOpen2 = true;
    } else if (isServoOpen2 && elapsedTime >= 31000) {
      // Close the servo at 25 seconds
      myServo.write(0);
      isServoOpen2 = false;
    }
  }
}


void moveRoutine2() {
  if (routine2) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - routine2StartTime;

    if (elapsedTime < routine2Duration) {
      if (elapsedTime < 5000) {
        // FORWARD
        ST->drive(-45);
        ST->turn(0);
      } else if (elapsedTime < 7000) {
        // TURN RIGHT
        ST->drive(0);
        ST->turn(40);
      } else if (elapsedTime < 9000) {
        // TURN LEFT
        ST->drive(0);
        ST->turn(-45);
      } else if (elapsedTime < 14000) {
        // BACKWARDS
        ST->drive(45);
        ST->turn(0);
      } else if (elapsedTime < 16000) {
        // TURN RIGHT
        ST->drive(0);
        ST->turn(-40);
      } else if (elapsedTime < 18000) {
        // TURN LEFT
        ST->drive(0);
        ST->turn(45); // Adjust the turning angle as needed
      } else if (elapsedTime < 21000) {
        // TURN RIGHT TO SIGNAL PICKUP
        ST->drive(0);
        ST->turn(65);
      } else if (elapsedTime < 31000) {
        // OPEN LATCH, WAIT FOR PICKUP, SOUND AND BLINKING LIGHTS
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 36000) {
        // TURN RIGHT, PICKUP CONPLETE SOUND
        ST->drive(-40);
        ST->turn(0);
      } else if (elapsedTime < 40000) {
        // TURN RIGHT, PICKUP CONPLETE SOUND
        ST->drive(0);
        ST->turn(40);
      } else if (elapsedTime < 45000) {
        // TURN RIGHT, PICKUP CONPLETE SOUND
        ST->drive(0);
        ST->turn(-40);
      } 
      else {
        // Stop after completing the sequence
        ST->stop(); // Stop the droid
        routine2 = false;
      }
    } else {
      ST->stop(); // Stop the droid if the duration is exceeded
      routine2 = false;
    }
  }
}

// ROUTINE 2
// --------------------------------------------------------------------------------------------------------




















// --------------------------------------------------------------------------------------------------------
// ROUTINE 3

void selectRoutine3(){
  if (reqCross){
    startRoutine3();
  }
}

void startRoutine3() {
  if (!routine3) {
    routine3StartTime = millis(); // Record the start time
    routine3 = true;
  }
  else{
    routine3 = false;
  }
}

void soundRoutine3() {
  static bool soundPlayed[10] = {false}; // Adjusted array size to track sound for each phase

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - routine3StartTime;

  if (routine3) {
    if (elapsedTime < 2000 && !soundPlayed[0]) {
      // FORWARD
      MP3Trigger.trigger(43); // Play sound for FORWARD
      soundPlayed[0] = true;
    } else if (elapsedTime >= 2000 && elapsedTime < 4000 && !soundPlayed[1]) {
      // BACK
      MP3Trigger.trigger(43); // Play sound for BACK
      soundPlayed[1] = true;
    } else if (elapsedTime >= 4000 && elapsedTime < 9000 && !soundPlayed[2]) {
      // TURN 360
      MP3Trigger.trigger(44); // Play sound for TURN 360
      soundPlayed[2] = true;
    } else if (elapsedTime >= 9000 && elapsedTime < 11000 && !soundPlayed[3]) {
      // FORWARDS
      MP3Trigger.trigger(43); // Play sound for FORWARDS
      soundPlayed[3] = true;
    } else if (elapsedTime >= 11000 && elapsedTime < 13000 && !soundPlayed[4]) {
      // TURN RIGHT
      MP3Trigger.trigger(43); // Play sound for TURN RIGHT
      soundPlayed[4] = true;
    } else if (elapsedTime >= 13000 && elapsedTime < 18000 && !soundPlayed[5]) {
      // TURN 360
      MP3Trigger.trigger(43); // Play sound for TURN 360
      soundPlayed[5] = true;
    } else if (elapsedTime >= 18000 && elapsedTime < 25000 && !soundPlayed[6]) {
      // SOMEONES FOOD IS GETTING COLD - OPEN LATCH
      MP3Trigger.trigger(44); // Play sound for OPEN LATCH
      soundPlayed[6] = true;
    } else if (elapsedTime >= 25000 && elapsedTime < 27000 && !soundPlayed[7]) {
      // OPEN LATCH, WAIT FOR PICKUP, SOUND AND BLINKING LIGHTS
      MP3Trigger.trigger(43); // Additional sound for WAIT FOR PICKUP
      soundPlayed[7] = true;
    } else if (elapsedTime >= 27000 && elapsedTime < 31000 && !soundPlayed[8]) {
      MP3Trigger.trigger(43);
      soundPlayed[8] = true;
    } else if (elapsedTime >= 31000 && elapsedTime < 40000 && !soundPlayed[9]) {
      // Final movement phase
      MP3Trigger.trigger(45); // Choose an appropriate sound for this phase
      soundPlayed[9] = true;
    } 

    // Reset the flags if the movement routine is completed or when it starts again
    if (elapsedTime >= routine3Duration || elapsedTime < 2000) {
      for (int i = 0; i < 10; i++) {
        soundPlayed[i] = false;
      }
    }
  } else {
    // Reset all sound flags if routine3 is not active
    for (int i = 0; i < 10; i++) {
      soundPlayed[i] = false;
    }
  }
}



// right side 19-23
// left side 0-4
void ledRoutine3() {

  static unsigned long lastToggleTime = 0; // Time when the LED was last toggled
  static bool isLedOn = false; // Current state of the LED
  const long flashInterval = 500; // Interval for LED flashing in milliseconds
  const long flashInterval2 = 250; // Interval for LED flashing in milliseconds
  const long flashInterval3 = 100; // Interval for LED flashing in milliseconds
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - routine3StartTime;
  int brightness = 4000; // Adjust this value as needed for your LEDs

  if (routine3) {
    if (elapsedTime < 2000) {
      // FORWARD

      if (currentTime - lastToggleTime > flashInterval) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 4000) {
      // TURN RIGHT
      brightness = 2000;
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(1, 0);
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(4, 0);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 9000) {
      // TURN LEFT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 11000) {
      // TURN LEFT
      if (currentTime - lastToggleTime > flashInterval) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    }
    else if (elapsedTime < 13000) {
      // TURN RIGHT
      brightness = 2000;
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 18000) {
      // TURN RIGHT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(1, 0);
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(4, 0);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 25000) {
      // TURN RIGHT
      if (currentTime - lastToggleTime > flashInterval2) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 2000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 27000) {
      // TURN RIGHT
      if (currentTime - lastToggleTime > flashInterval2) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 0;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, 500);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, 500);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 500);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, 500);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, 500);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    } else if (elapsedTime < 29000) {
      // TURN RIGHT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, 0);
      LEDControl.setPWM(20, 0);
      LEDControl.setPWM(21, 0);
      LEDControl.setPWM(22, 0);
      LEDControl.setPWM(23, 0);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    }
    else if (elapsedTime < 31000) {
      // TURN RIGHT
      brightness = 2000; // Different brightness for TURN LEFT
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, 0);
      LEDControl.setPWM(1, 0);
      LEDControl.setPWM(2, 0);
      LEDControl.setPWM(3, 0);
      LEDControl.setPWM(4, 0);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    }
    else if (elapsedTime < 40000) {
      // TURN RIGHT
      if (currentTime - lastToggleTime > flashInterval3) {
        isLedOn = !isLedOn;
        lastToggleTime = currentTime;
      }
      
      brightness = isLedOn ? 4000 : 500;
      
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
      
    }
    // ... Continue this pattern for other movement phases
    else {
      // Default or off state
      brightness = 0; // Turn off or set to default state
      LEDControl.setPWM(19, brightness);
      LEDControl.setPWM(20, brightness);
      LEDControl.setPWM(21, brightness);
      LEDControl.setPWM(22, brightness);
      LEDControl.setPWM(23, brightness);
      LEDControl.setPWM(0, brightness);
      LEDControl.setPWM(1, brightness);
      LEDControl.setPWM(2, brightness);
      LEDControl.setPWM(3, brightness);
      LEDControl.setPWM(4, brightness);

      LEDControl.setPWM(9, brightness);
      LEDControl.setPWM(8, brightness);

      LEDControl.setPWM(10, brightness);
      LEDControl.setPWM(11, brightness);
    }

    LEDControl.write(); // Update the LED state
  } else {
    // Turn off LEDs if routine2 is not active
    brightness = 0;
    LEDControl.setPWM(19, brightness);
    LEDControl.setPWM(20, brightness);
    LEDControl.setPWM(21, brightness);
    LEDControl.setPWM(22, brightness);
    LEDControl.setPWM(23, brightness);
    LEDControl.setPWM(0, brightness);
    LEDControl.setPWM(1, brightness);
    LEDControl.setPWM(2, brightness);
    LEDControl.setPWM(3, brightness);
    LEDControl.setPWM(4, brightness);

    LEDControl.setPWM(9, brightness);
    LEDControl.setPWM(8, brightness);

    LEDControl.setPWM(10, brightness);
    LEDControl.setPWM(11, brightness);
    
    LEDControl.write();
  }
}


void controlServo3() {
  if (routine3) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - routine3StartTime;
  
    if (!isServoOpen3 && elapsedTime >= 18000 && elapsedTime < 25000) {
      // Open the servo at 18 seconds
      myServo.write(180);
      isServoOpen3 = true;
    } else if (isServoOpen3 && elapsedTime >= 25000) {
      // Close the servo at 25 seconds
      myServo.write(0);
      isServoOpen3 = false;
    }
  }
}


void moveRoutine3() {
  if (routine3) {
    
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - routine3StartTime;

    if (elapsedTime < routine3Duration) {
      if (elapsedTime < 2000) {
        // FORWARD
        ST->drive(-45);
        ST->turn(0);
      } else if (elapsedTime < 4000) {
        // BACK
        ST->drive(45);
        ST->turn(0);
      } else if (elapsedTime < 9000) {
        // TURN 360
        ST->drive(0);
        ST->turn(-45);
      } else if (elapsedTime < 11000) {
        // FORWARDS
        ST->drive(-45);
        ST->turn(0);
      } else if (elapsedTime < 13000) {
        // TURN RIGHT
        ST->drive(45);
        ST->turn(0);
      } else if (elapsedTime < 18000) {
        // TURN 360
        ST->drive(0);
        ST->turn(-45);
      } else if (elapsedTime < 25000) {
        // SOMEONES FOOD IS GETTING COLD
        // OPEN LATCH
        ST->drive(0);
        ST->turn(0);
      } else if (elapsedTime < 27000) {
        // OPEN LATCH, WAIT FOR PICKUP, SOUND AND BLINKING LIGHTS
        ST->drive(-45);
        ST->turn(0);
      } else if (elapsedTime < 29000) {
        // TURN RIGHT, PICKUP CONPLETE SOUND
        ST->drive(45);
        ST->turn(0);
      } else if (elapsedTime < 31000) {
        // TURN RIGHT, PICKUP CONPLETE SOUND
        ST->drive(0);
        ST->turn(40);
      } else if (elapsedTime < 35000) {
        // TURN RIGHT, PICKUP CONPLETE SOUND
        ST->drive(0);
        ST->turn(-40);
      } else if (elapsedTime < 40000) {
        // TURN RIGHT, PICKUP CONPLETE SOUND
        ST->drive(-30);
        ST->turn(0);
      } else {
        // Stop after completing the sequence
        ST->stop(); // Stop the droid
        routine3 = false;
      }
    } else {
      ST->stop(); // Stop the droid if the duration is exceeded
      routine3 = false;
    }
  }
}

// ROUTINE 3
// --------------------------------------------------------------------------------------------------------






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
