// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
//
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

#include "Button.h"

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND 100            // [ms] Sending time interval
#define TIME_SEND_DEBUG 1000     // [ms] Sending time interval

#define SPEED_MAX_MODE0 800  // [-] Maximum speed for testing
#define SPEED_MAX_MODE1 1000 // [-] Maximum speed for testing
#define SPEED_MAX_MODE2 1200 // [-] Maximum speed for testing
#define SPEED_MAX_MODE3 1400 // [-] Maximum speed for testing

#define MODE_COUNT 4

#define SPEED_STEP 100 // [-] Speed step
#define SPEED_STEP_DOWN 300
#define DEBUG_RX // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define BUTTON_PIN 18
#define BUTTON_DEBOUNCE_INTERVAL 100 // Milliseconds

#define POT_MIDDLE 1850
#define POT_DEADBAND 400
#define POT_DEBOUNCE 100

#define DEBUG false

#include <SoftwareSerial.h>
// SoftwareSerial HoverSerial(16,17);        // RX, TX

HardwareSerial HoverSerial(2);

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct
{
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

int mode = 0;

static void onModeBtnSingleClickEventCb(void *button_handle, void *usr_data)
{
  Serial.println("Button single click");
  mode++;
}

static void onModeBtnLongPressStartEventCb(void *button_handle, void *usr_data)
{
  Serial.println("Button long press start");
  mode--;
}

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");
  // 16=rx, 17=tx
  HoverSerial.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17);
  // HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  Button *modeBtn = new Button(BUTTON_PIN, true);
  modeBtn->setParam
      modeBtn->attachSingleClickEventCb(&onModeBtnSingleClickEventCb, NULL);
  modeBtn->attachLongPressStartEventCb(&onModeBtnLongPressStartEventCb, NULL);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.steer = (int16_t)uSteer;
  Command.speed = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available())
  {
    incomingByte = HoverSerial.read();                                  // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
  }
  else
  {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("1: ");
      Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");
      Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");
      Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");
      Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");
      Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");
      Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");
      Serial.println(Feedback.cmdLed);
    }
    else
    {
      Serial.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

unsigned long iTimeSendUart = 0;
unsigned long iTimeSendDebug = 0;
unsigned long iTimePotx = 4294967295;
bool debouncingPotxActive = false;
bool switchedxFlag = false;

int speedCommand = 0;
int speedStep = SPEED_STEP;
int maxSpeed = SPEED_MAX_MODE0;

uint x = 0;
uint y = 0;

bool buttonPressed = false;

void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  Receive();

  // update potis
  x = analogRead(32);
  y = analogRead(33);

  // constrain mode
  constrain(mode, 0, MODE_COUNT - 1);

  switch (mode)
  {
  case 0:
    maxSpeed = SPEED_MAX_MODE0;
    break;
  case 1:
    maxSpeed = SPEED_MAX_MODE1;
    break;
  case 2:
    maxSpeed = SPEED_MAX_MODE2;
    break;
  case 3:
    maxSpeed = SPEED_MAX_MODE3;
    break;
  default:
    mode = 0;
    maxSpeed = SPEED_MAX_MODE0;
  }
  Serial.println("Mode: " + String(mode));

  if (DEBUG)
  {
    if (timeNow > iTimeSendDebug)
    {
      iTimeSendDebug = timeNow + TIME_SEND_DEBUG;
      Serial.println("x=" + String(x));
      Serial.println("y=" + String(y));
    }
  }

  // Send commands
  if (iTimeSendUart > timeNow)
    return;
  iTimeSendUart = timeNow + TIME_SEND;
  // limit speed
  speedCommand = constrain(speedCommand, 0, maxSpeed);
  if (DEBUG)
    Serial.println("speed=" + String(speedCommand));
  Send(0, speedCommand);

  // Calculate test command signal
  if (x > POT_MIDDLE + POT_DEADBAND)
  {
    speedCommand += speedStep;
  }
  else
  {
    speedCommand -= SPEED_STEP_DOWN;
  }
}

// ########################## END ##########################
