//**************************************************************************************************
// INCLUDES
//**************************************************************************************************
#include <Wire.h>
#include <SPI.h>
#include <Pixy2.h> // Pixy2 library
#include <PIDLoop.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
//#include <timer.h>               //Timer library
#include <SharpIR.h>
#include <Commander.h>

//**************************************************************************************************
// DEFINES
//**************************************************************************************************
#define X_AXIS_MID 160 // Middle point in the x-axis
#define Y_AXIS_MID 100 // Middle point in the y-axis

#define X_AXIS_R_FOOT 200 // Point around right foot.

// FSM States for robot
#define ROBO_FSM_FIND 0     // State for finding the ball
#define ROBO_FSM_WALK 1     // State for walking towards the ball
#define ROBO_FSM_POSITION 2 // State for aligning robot with ball
#define ROBO_FSM_KICK 3     // State for kicking the ball

// Serial values received from CM-530
#define CM_530_READY 10

// Define proper RST_PIN if required.
#define RST_PIN -1
//**************************************************************************************************
// PINS
//**************************************************************************************************
int blueLights = 2;
int distanceSensor = 20;

//**************************************************************************************************
// VARIABLES
//**************************************************************************************************
SSD1306AsciiWire oled; // Create display object
Pixy2 pixy;            // Create a Pixy camera object
PIDLoop panLoop(400, 0, 400, true);
PIDLoop tiltLoop(500, 0, 500, true);
SharpIR sensor = SharpIR(distanceSensor, 1080); // Distance sensor
//auto timerElapsed = timer_create_default();   // create a timer with default settings
Commander command = Commander();

double maxDeviation = 0.75 * (X_AXIS_MID / 2);  // Determines the max deviation from the center
double maxDevFoot = 0.97 * (X_AXIS_R_FOOT / 2); // Determines the max deviation from the r foot

uint8_t fsmState; // Defines state in FSM which controls the robot.

long seconds = 1;
long minutes = 0;

int irSensor_value = 0;
uint8_t blocks = 0;
int blueLightsVal = 0;
int receivedCM530 = 0;

//**************************************************************************************************
// CONSTANTS
//**************************************************************************************************
// Instructions to move forward, left, right, and back.
const uint8_t moveFoward[6] = {0xAA, 0x01, 0x0E,
                               0xAA, 0x00, 0x1E};
const uint8_t *buttonU_ptr = moveFoward;
const uint8_t moveLeft[6] = {0xAA, 0x04, 0x5E,
                             0xAA, 0x00, 0x1E};
const uint8_t *buttonL_ptr = moveLeft;
const uint8_t moveRight[6] = {0xAA, 0x08, 0x1E,
                              0xAA, 0x00, 0x1E};
const uint8_t *buttonR_ptr = moveRight;
const uint8_t moveBack[6] = {0xAA, 0x02, 0x7E,
                             0xAA, 0x00, 0x1E};
const uint8_t *buttonD_ptr = moveBack;

// 3 => 64 decimal
const uint8_t button3[12] = {0xAA, 0x40, 0x1E,
                             0xAA, 0x00, 0x1E};
const uint8_t *button3_ptr = button3;

// 1 => 16 decimal
const uint8_t button1[12] = {0xAA, 0x10, 0x1E,
                             0xAA, 0x00, 0x1E};
const uint8_t *button1_ptr = button1;

// 4 => 128 decimal
const uint8_t button4[12] = {0xAA, 0x00, 0x0F,
                             0xAA, 0x00, 0x1E};
const uint8_t *button4_ptr = button4;

// U + 2 => 33 decimal
const uint8_t buttonU2[6] = {0xAA, 0x21, 0x0E,
                             0xAA, 0x00, 0x1E};
const uint8_t *buttonU2_ptr = buttonU2;

// U + 1 => 17 decimal
const uint8_t buttonU1[6] = {0xAA, 0x11, 0x0E,
                             0xAA, 0x00, 0x1E};
const uint8_t *buttonU1_ptr = buttonU1;

// U + 3 => 65 decimal
const uint8_t buttonU3[6] = {0xAA, 0x41, 0x0E,
                             0xAA, 0x00, 0x1E};
const uint8_t *buttonU3_ptr = buttonU3;

// L + 2 => 36 decimal
const uint8_t buttonL2[12] = {0xAA, 0x24, 0x5E,
                              0xAA, 0x00, 0x1E};
const uint8_t *buttonL2_ptr = buttonL2;

// R + 2 => 40 decimal
const uint8_t buttonR2[12] = {0xAA, 0x28, 0x1E,
                              0xAA, 0x00, 0x1E};
const uint8_t *buttonR2_ptr = buttonR2;

// U + R => 9 decimal
const uint8_t buttonUR[6] = {0xAA, 0x09, 0x0E,
                             0xAA, 0x00, 0x1E};
const uint8_t *buttonUR_ptr = buttonUR;

// D + R => 10 decimal
const uint8_t buttonDR[12] = {0xAA, 0x0A, 0x7E,
                              0xAA, 0x00, 0x1E};
const uint8_t *buttonDR_ptr = buttonDR;

// L + R => 12 decimal
const uint8_t buttonLR[12] = {0xAA, 0x0C, 0x5E,
                              0xAA, 0x00, 0x1E};
const uint8_t *buttonLR_ptr = buttonLR;

// 1 + 2 => 48 decimal
const uint8_t button12[6] = {0xAA, 0x30, 0x1E,
                             0xAA, 0x00, 0x1E};
const uint8_t *button12_ptr = button12;

// 1 + 3 => 80 decimal
const uint8_t button13[6] = {0xAA, 0x50, 0x1E,
                             0xAA, 0x00, 0x1E};
const uint8_t *button13_ptr = button13;

// R + 1 => 24 decimal
const uint8_t buttonR1[6] = {0xAA, 0x18, 0x1E,
                             0xAA, 0x00, 0x1E};
const uint8_t *buttonR1_ptr = buttonR1;
//**************************************************************************************************
// FUNCTION PROTOTYPES
//**************************************************************************************************
// Robot states
void walkTowardsObject();
void positionRobot();
void performKick();

// Robot actions
void moveRobotForward();
void moveRobotLeft();
void moveRobotRight();
void moveRobotBack();
void slideRobotLeft();
void slideRobotRight();
void robotKick();
void robotStandUp();

// Other methods
void CheckPixyFramerate();
void PrintDataFromCM530(int received);
int ReceiveDataFromCM530();
void PrintIRSensorValue(int value);
void SendCommand(uint8_t button[0]);
void SendDataMessage(char button[]);
void LightsOn();
void LightsOff();
void PrintPanPosition();

// PIXY2 methods
void Print_Y_Axis_Value(int y_axis);
void PixyDetectObject();

//**************************************************************************************************
//                                         Name: setup
//**************************************************************************************************
void setup()
{
  Serial.begin(9600);
  //pinMode(0, OUTPUT);

  // Head Fan
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);

  // Start serial XBEE/ Commander
  Serial3.begin(38400);

  // Set the blue leds
  pinMode(blueLights, OUTPUT);

  //************************************************************************************************
  // 0.96 OLED Display settings
  //************************************************************************************************
  // This is needed for the OLED to function properly
  Wire.begin();

  //Wire.setClock(400000L);

  oled.begin(&Adafruit128x64, 0x3C); // Initialize display with the I2C address of 0x3C
  oled.setScrollMode(SCROLL_MODE_OFF);
  oled.invertDisplay(false);
  oled.setInvertMode(false);
  oled.setLetterSpacing(3);
  oled.setFont(lcd5x7);
  oled.set1X();
  oled.clear();
  oled.println(F("Initializing OLED"));
  delay(400);

  // start serial to CM-530 (1900 baud)
  Serial1.begin(1900);
  oled.println(F("(CM-530) SERIAL 1900"));
  oled.println();

  delay(2000);

  oled.println(F("Initializing PIXY 2.."));
  oled.println();
  Serial2.begin(115200); // Start serial PIXY2
  pixy.init();           // Initialize Pixy camera
  pixy.setLamp(0, 0);    // Head leds (upper-lights, lower light)
  pixy.changeProg("color_connected_components");

  delay(200);

  // Flash blue leds
  oled.println(F("Checking LIGHTS.."));
  oled.println();
  digitalWrite(blueLights, HIGH);
  delay(30);
  digitalWrite(blueLights, LOW);

  // READY
  delay(400);
  oled.clearField(0, 5, 50);
  oled.setRow(5);
  oled.setCol(40);
  oled.set2X();
  oled.println(F("READY!"));
  delay(1600);
  oled.clear(); // Clear display

  // call the do_count function every 1000 millis (1 second)
  //timerElapsed.every(1000, do_count);

  // Initialize FSM state
  //************************************************************************************************
  // Due to time constrains, the robot will assume that it already has seen the ball. For future
  // work, the robot should be able to find the ball, and should incorperate Hough Transform to
  // find the shape of the ball as well.
  //************************************************************************************************
  fsmState = ROBO_FSM_FIND;

  //Wait untill CM-530 is ready and running.
  // while (receivedCM530 == 0)
  // {
  //   receivedCM530 = ReceiveDataFromCM530();
  // }

  // Get up
  //robotStandUp();
}

//**************************************************************************************************************************************
//                                                                 MAIN LOOP
//**************************************************************************************************************************************
void loop()
{
  irSensor_value = sensor.distance(); // Read Distance in centimeters;
  PixyDetectObject();                 // Read PIXY blocks and Updates servos
  //ReceiveDataFromCM530();             // Read data from CM-530
  CheckPixyFramerate(); // Turns lamp if FPS is < 30

  //***************************
  // PRINT DATA ON OLED DISPLAY
  //***************************
  PrintPanPosition();
  PrintIRSensorValue(irSensor_value);

  //  if (irSensor_value < 8 )
  //  {
  //    // panLoop.m_command - center point is 500
  //    if (panLoop.m_command < 350)
  //    {
  //      RightHandUp();
  //      while (dataFromCM530 != 1)
  //      {
  //        PixyDetectObject();
  //        dataFromCM530 = ReceiveDataFromCM530();
  //      }
  //    }
  //    else if (panLoop.m_command > 650)
  //    {
  //      LeftHandUp();
  //
  //      while ( dataFromCM530 != 1)
  //      {
  //        PixyDetectObject();
  //        dataFromCM530 = ReceiveDataFromCM530();
  //      }
  //    }
  //  }

  //**************************
  // COMMANDER2 remote control
  //**************************
  //  if (command.ReadMsgs() > 0)
  //  {
  //    oled.clearField(100, 4, 8);
  //    oled.set1X();
  //    oled.setRow(4);
  //    oled.setCol(60);
  //    oled.print(F("Input: "));
  //    oled.print(Serial3.read());
  //    if (Serial3.read() == 1) {
  //      ToggleLights();
  //    }
  //    else if (Serial3.read() == 2) {
  //      LeftHandUp();
  //    }
  //    else if (Serial3.read() == 4) {
  //      LeftHandDown();
  //    }
  //  }

  //  timerElapsed.tick();
  //  DisplayMinutes();
  //  DisplaySeconds();

  //**************************************************************************************************
  // Name: Move Robot
  // Run state machine for every loop
  //**************************************************************************************************

  // switch (fsmState)
  // {
  // // Find the ball
  // case ROBO_FSM_FIND:
  //   PrintRobotState(1);

  //   while (blocks == 0)
  //   {
  //     PixyDetectObject();
  //     blocks = pixy.ccc.getBlocks();
  //   }
  //   fsmState = ROBO_FSM_WALK;
  //   break;

  // // Make the robot walk towards the object.
  // case ROBO_FSM_WALK:
  //   PrintRobotState(2);
  //   walkTowardsObject();
  //   break;

  // // Make the robot position to object.
  // case ROBO_FSM_POSITION:
  //   PrintRobotState(3);
  //   positionRobot();
  //   break;

  // // Have the robot kick the ball.
  // case ROBO_FSM_KICK:
  //   PrintRobotState(4);
  //   performKick();
  //   break;

  // // Default case
  // default:
  //   break;
  // }
}

// FUNCTIONS

void PixyDetectObject()
{
  static int i = 0;
  int j;
  char buf[64];
  int32_t panOffset, tiltOffset;

  // get active blocks from Pixy
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    i++;

    // calculate pan and tilt "errors" with respect to first object (blocks[0]),
    // which is the biggest object (they are sorted by size).
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)pixy.ccc.blocks[0].m_x;
    tiltOffset = (int32_t)pixy.ccc.blocks[0].m_y - (int32_t)pixy.frameHeight / 2;

    // update loops
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // set pan and tilt servos
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);

    // PLAY WITH HANDS
    if (irSensor_value < 12)
    {
      // Left hand up
      if (panLoop.m_command > 600)
      {
        SendCommand(buttonU3_ptr);
        char buttonU3[] = "U3";
        SendDataMessage(buttonU3);
        pixy.setServos(800, 750);
      }
      // Right hand up
      else if(panLoop.m_command < 400)
      {
        SendCommand(buttonU2_ptr);
        char buttonU2[] = "U2";
        SendDataMessage(buttonU2);
        pixy.setServos(200, 750);
      }

      delay(2400);

      while (true)
      {
        pixy.ccc.getBlocks();

        if (pixy.ccc.numBlocks)
        {
          char buttonU[] = "U";
          SendDataMessage(buttonU);
          SendCommand(buttonU_ptr);
          break;
        }
      }

      // Standby while we see the block
      while (pixy.ccc.getBlocks() > 0)
      {
      }

      //Open hand
      char buttonD[] = "D";
      SendDataMessage(buttonD);
      SendCommand(buttonD_ptr);

      receivedCM530 = 0;
    }

    //Print information
    oled.clearField(0, 6, 300); // clearField (col, row , int - number of characters to delete)
    oled.setCol(0);
    oled.setRow(6);
    oled.print(F("TRACKING: "));
    oled.print(pixy.ccc.blocks[0].m_signature);

#if 0 // for debugging
    sprintf(buf, "%ld %ld %ld %ld", rotateLoop.m_command, translateLoop.m_command, left, right);
    Serial.println(buf);
#endif
  }
  else // no object detected, go into reset state
  {
    panLoop.reset();
    tiltLoop.reset();
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);

    oled.clearField(0, 6, 100); // clearField (col, row , int - number of characters to delete)
    oled.set1X();
    oled.setCol(0);
    oled.setRow(6);
    oled.print(F("LOST..")); // Print Lost
  }
}
//**************************************************************************************************
//                                     Name: walkTowardsObject
//**************************************************************************************************
void walkTowardsObject()
{
  PixyDetectObject();
  uint16_t y_axis_point;

  // Get the number of blocks and update servos on Pixy2
  blocks = pixy.ccc.getBlocks();
  y_axis_point = pixy.ccc.blocks[0].m_y;
  Print_Y_Axis_Value(y_axis_point); // Print Y axis value from pixy

  // If the object is detected
  if (blocks)
  {
    // If the block is to the left, we want to move left.
    // panLoop.m_command and tiltLoop.m_command we can get the pan and tilt values of the servos
    if (panLoop.m_command > 580)
    {
      moveRobotLeft();
    }
    // If the block is to the right, we want to move right.
    else if (panLoop.m_command < 380)
    {
      moveRobotRight();
    }
    else
    {
      // If the ball is not close, walk towards the ball
      //y_axis_point
      if (irSensor_value > 10)
      {
        moveRobotForward();
      }
      // If the ball is close enough, then it is time to switch to the next state.
      else
      {
        fsmState = ROBO_FSM_POSITION;
      }
    }
  }
  else
  {
    //**********************************************************************************************
    // If the camera(s) can not detect the ball, then the robot will have to switch to the state
    // in which it will search for the ball. Because this is not implemented however, it should
    // be done in the future.
    //**********************************************************************************************
    fsmState = ROBO_FSM_FIND;
  }
}

//**************************************************************************************************
// Name: positionRobot
//**************************************************************************************************
void positionRobot()
{
  uint16_t blocks;
  uint16_t x_left;
  uint16_t x_right;
  uint16_t x_mid;
  uint16_t y_axis_point;

  PixyDetectObject();

  // Get the image details.
  blocks = pixy.ccc.getBlocks();

  // If the image is detected
  if (blocks)
  {
    // Determine where the middle point of the block is
    x_left = pixy.ccc.blocks[0].m_x;
    x_right = x_left + (pixy.ccc.blocks[0].m_width);

    x_mid = ((x_right - x_left) / 2) + x_left;

    y_axis_point = pixy.ccc.blocks[0].m_y;

    //If the ball is close enough, but not aligned, go left or right
    if (x_mid < X_AXIS_R_FOOT)
    {
      moveRobotLeft();
    }
    else if (x_mid > (X_AXIS_R_FOOT + maxDevFoot))
    {
      moveRobotRight();
    }

    // If the block is not close enough, move forward.
    if (irSensor_value > 10)
    {
      moveRobotForward();
    }
    else
    {
      fsmState = ROBO_FSM_KICK;
    }
  }
  else
  {
    //**********************************************************************************************
    // If the robot can not find the ball (although it is assumed it should be right infront of it),
    // then the robot should switch to the find ball mode. For now, however, it will be put into
    // the walk mode.
    //**********************************************************************************************
    fsmState = ROBO_FSM_FIND;
  }
}

//**************************************************************************************************
//                                               Name: performKick
//**************************************************************************************************
void performKick()
{
  robotKick();
  fsmState = ROBO_FSM_FIND;
}

//**************************************************************************************************
//                                            Name: moveRobotForward
//**************************************************************************************************
void moveRobotForward()
{
  char button[] = "U";
  SendDataMessage(button);

  Serial1.write(buttonU_ptr, 3);
  delay(210);
  Serial1.write(buttonU_ptr + 3, 3);
  delay(300);
  Serial1.write(buttonU_ptr, 3);
  delay(210);
  Serial1.write(buttonU_ptr + 3, 3);
}

//**************************************************************************************************
//                                              Name: moveRobotLeft
//**************************************************************************************************
void moveRobotLeft()
{
  char button[] = "L";
  SendDataMessage(button);

  Serial1.write(buttonL_ptr, 3);
  delay(210);
  Serial1.write(buttonL_ptr + 3, 3);
}

//**************************************************************************************************
//                                               Name: moveRobotRight
//**************************************************************************************************
void moveRobotRight()
{
  char button[] = "R";
  SendDataMessage(button);

  Serial1.write(buttonR_ptr, 3);
  delay(210);
  Serial1.write(buttonR_ptr + 3, 3);
}

//**************************************************************************************************
//                                                  Name: moveRobotBack
//**************************************************************************************************
void moveRobotBack()
{
  Serial1.write(buttonD_ptr, 3);
  delay(210);
  Serial1.write(buttonD_ptr + 3, 3);

  char button[] = "D";
  SendDataMessage(button);
}

//**************************************************************************************************
//                                           Name: slideRobotLeft
//**************************************************************************************************
//void slideRobotLeft()
//{
//  Serial1.write(slideLeft_ptr, 3);
//  delay(210);
//  Serial1.write(slideLeft_ptr + 3, 3);
//}

//**************************************************************************************************
//                                              Name: slideRobotRight
//**************************************************************************************************
//void slideRobotRight()
//{
//  Serial1.write(slideRight_ptr, 3);
//  delay(210);
//  Serial1.write(slideRight_ptr + 3, 3);
//}

//**************************************************************************************************
//                                               Name: robotKick
//**************************************************************************************************
void robotKick()
{
  Serial1.write(button4_ptr, 3);
  delay(210);
  Serial1.write(button4_ptr + 3, 3);

  char button[] = "4";
  SendDataMessage(button);
}

void SendCommand(uint8_t button[0])
{
  Serial1.write(button, 3);
  delay(210);
  Serial1.write(button + 3, 3);
}

// Count the minutes and seconds
bool do_count(void *)
{
  seconds++;

  if (seconds == 59)
  {
    minutes++;
    seconds = 0;

    // Clear timer line every minute
    oled.clear(0, 150, 0, 0); // (startCOl, endCOL, startROW, endRow)

    // Blip lights every minute
    LightsOn();
    LightsOff();
  }

  if (seconds == 3)
  {
    // Clear lights ON message
    oled.clear(0, 150, 2, 2); // (startCOl, endCOL, startROW, endRow)
  }
  else if (seconds == 7)
  {
    // Clear lights OFF message
    oled.clear(0, 150, 3, 3); // (startCOl, endCOL, startROW, endRow)
  }
  else if (seconds % 2 == 0)
  {
    oled.clearField(32, 5, 4);  //clear blocks value
    oled.clearField(40, 3, 14); // clearField(col, row , int - number of characters to delete)
  }

  return true; // repeat? true
}
//**************************************************************************************************
//                                            OLED actions
//**************************************************************************************************
// Command send display
void SendDataMessage(char button[])
{
  oled.clearField(15, 3, 10); // clearField(col, row , int - number of characters to delete)
  oled.setCol(0);
  oled.setRow(3);
  oled.print(F("Command:"));
  oled.print(button);
}

void DisplaySeconds()
{
  oled.setRow(0);
  oled.setCol(80);
  oled.print(seconds);
  oled.println(F(" SEC"));
}

void DisplayMinutes()
{
  oled.setRow(0);
  oled.setCol(0);
  oled.set1X();
  oled.print(F("ON: "));
  oled.setCol(30);
  oled.print(minutes);
  oled.print(F(" MIN"));
}

void PrintRobotState(int state)
{

  oled.clearField(56, 4, 20);
  oled.setRow(4);
  oled.setCol(50);
  oled.print(F("State: "));

  switch (state)
  {
  case 1:
    oled.print(F("FIND"));
    break;
  case 2:
    oled.print(F("WALK"));
    break;
  case 3:
    oled.print(F("POSITION"));
    break;
  case 4:
    oled.print(F("KICK!"));
    break;
  default:
    oled.print(F("UKNOWN.."));
    break;
  }
}

void Print_Y_Axis_Value(int y_axis)
{
  oled.clearField(8, 0, 20);
  oled.setCol(0);
  oled.setRow(0);
  oled.print(F("Y-AXIS: "));
  oled.print(y_axis);
}

void PrintPanPosition()
{
  oled.clearField(50, 2, 5); // clearField (col, row , int - number of characters to delete)
  oled.set1X();
  oled.setCol(45);
  oled.setRow(2);
  oled.print(F("PAN Pos: ")); // Print Servo position
  oled.print(panLoop.m_command);
}

void PrintIRSensorValue(int value)
{
  oled.clearField(80, 7, 10);
  oled.setCol(65);
  oled.setRow(7);
  oled.set1X();
  oled.print(F("DIST: "));
  oled.print(value);
}

void XBeePrint()
{
  oled.clearField(87, 4, 8);
  oled.set1X();
  oled.setRow(4);
  oled.setCol(55);
  oled.print(F("XBEE: "));
  if (true)
  {
    //    oled.print(F("..."));
  }
  else
  {
    oled.print(F("Received"));
  }
}

void PrintDataFromCM530(int received)
{
  oled.clearField(12, 5, 20); // (col , row , number of characters)
  oled.set1X();
  oled.setRow(5);
  oled.setCol(0);
  oled.print(F("CM-530: "));
  oled.print(received);
}
//**************************************************************************************************
//                                                  LIGHTS
//**************************************************************************************************
// Toggle all lights
void ToggleLights()
{
  blueLightsVal = digitalRead(blueLights);
  if (blueLightsVal == HIGH)
  {
    digitalWrite(blueLights, LOW);
    pixy.setLamp(0, 0);
  }
  else
  {
    digitalWrite(blueLights, HIGH);
    pixy.setLamp(1, 1);
  }
}

void LightsOn()
{
  oled.clear(0, 150, 2, 2); // (startCOl, endCOL, startROW, endRow)
  oled.set1X();
  oled.setRow(2);
  oled.println(F("Lights: ON"));
  digitalWrite(2, HIGH);
}

void LightsOff()
{
  oled.clear(0, 150, 3, 3); // (startCOl, endCOL, startROW, endRow)
  oled.set1X();
  oled.setRow(3);
  oled.println(F("Lights: OFF"));
  digitalWrite(2, LOW);
}

// Get data from CM-530.
int ReceiveDataFromCM530()
{
  receivedCM530 = 0;

  if (Serial1.available() > 0)
  {
    receivedCM530 = Serial1.parseInt();
  }

  PrintDataFromCM530(receivedCM530);

  return receivedCM530;
}

// Check if frame rate is low then that means the lighting is low, we turn the lights on.
void CheckPixyFramerate()
{
  if (pixy.getFPS() < 20)
  {
    pixy.setLamp(1, 1);
  }
  else
  {
    pixy.setLamp(0, 0);
  }
}

void robotStandUp()
{
  if (sensor.distance() < 20)
  {
    SendCommand(buttonU_ptr);
  }
  else
  {
    SendCommand(buttonD_ptr);
  }
}
