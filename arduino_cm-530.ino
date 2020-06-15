//**************************************************************************************************
// INCLUDES
//**************************************************************************************************
#include <Wire.h>
#include <SPI.h>
#include <Pixy2SPI_SS.h>               // Pixy2 library
#include <PIDLoop.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <timer.h>               //Timer library
#include <XBee.h>
#include <SharpIR.h>

/*
  XBEE
  This example is for Series 1 XBee (802.15.4)
  Receives either a RX16 or RX64 packet and sets a PWM value based on packet data.
  Error led is flashed if an unexpected packet is received
*/

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle
Rx16Response rx16 = Rx16Response();
Rx64Response rx64 = Rx64Response();

int statusLed = 11;
int errorLed = 12;
int dataLed = 10;

uint8_t option = 0;
uint8_t data = 0;

void flashLed(int pin, int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}
// XBEE END LINE

//**************************************************************************************************
// DEFINES
//**************************************************************************************************
#define X_AXIS_MID 160  // Middle point in the x-axis
#define Y_AXIS_MID 100  // Middle point in the y-axis

#define X_AXIS_R_FOOT 200 // Point around right foot.

// FSM States for robot
#define ROBO_FSM_FIND     0 // State for finding the ball
#define ROBO_FSM_WALK     1 // State for walking towards the ball
#define ROBO_FSM_POSITION 2 // State for aligning robot with ball
#define ROBO_FSM_KICK     3 // State for kicking the ball

// Define proper RST_PIN if required.
#define RST_PIN -1
//**************************************************************************************************
// VARIABLES
//**************************************************************************************************
SSD1306AsciiWire oled;      // Create display object

Pixy2SPI_SS pixy;                                           // Create a Pixy camera object
PIDLoop panLoop(400, 0, 400, true);
PIDLoop tiltLoop(500, 0, 500, true);

SharpIR sensor = SharpIR(20, 1080);;

auto timerElapsed = timer_create_default();           // create a timer with default settings

double maxDeviation = 0.75 * (X_AXIS_MID / 2); // Determines the max deviation from the center
double maxDevFoot   = 0.97 * (X_AXIS_R_FOOT / 2); // Determines the max deviation from the r foot

uint8_t fsmState; // Defines state in FSM which controls the robot.

long seconds = 1;
long minutes = 0;

int irSensor_value = 0;
int pixyObjects = 0;

//**************************************************************************************************
// CONSTANTS
//**************************************************************************************************
// Instructions to move forward, left, right, and back.
const uint8_t  moveFoward[6]  = {0xAA, 0x01, 0x0E,
                                 0xAA, 0x00, 0x1E
                                };
const uint8_t* moveFoward_ptr = moveFoward;
const uint8_t  moveLeft[6]    = {0xAA, 0x04, 0x5E,
                                 0xAA, 0x00, 0x1E
                                };
const uint8_t* moveLeft_ptr   = moveLeft;
const uint8_t  moveRight[6]   = {0xAA, 0x08, 0x1E,
                                 0xAA, 0x00, 0x1E
                                };
const uint8_t* moveRight_ptr  = moveRight;
const uint8_t  moveBack[6]    = {0xAA, 0x02, 0x7E,
                                 0xAA, 0x00, 0x1E
                                };
const uint8_t* moveBack_ptr   = moveBack;

// remote button 3 => 64
const uint8_t  button3[12]   = {0xAA, 0x40, 0x1E,
                                0xAA, 0x00, 0x1E
                               };
const uint8_t* button3_ptr   = button3;

// remote button 1 => 16
const uint8_t  button1[12]     = {0xAA, 0x10, 0x1E,
                                  0xAA, 0x00, 0x1E
                                 };
const uint8_t* button1_ptr     = button1;


// button 4 => 128
const uint8_t  button4[12] = {0xAA, 0x00, 0x0F,
                              0xAA, 0x00, 0x1E
                             };
const uint8_t* button4_ptr = button4;

// remote button U + 2 => 33 decimal
const uint8_t buttonU2[6] = {0xAA, 0x21, 0x0E,
                             0xAA, 0x00, 0x1E
                            };
const uint8_t* buttonU2_ptr = buttonU2;

// remote button U + 1 => 17 decimal
const uint8_t buttonU1[6] = {0xAA, 0x11, 0x0E,
                             0xAA, 0x00, 0x1E
                            };
const uint8_t* buttonU1_ptr = buttonU1;

// remote button U + 3 => 65 decimal
const uint8_t buttonU3[6] = {0xAA, 0x41, 0x0E,
                             0xAA, 0x00, 0x1E
                            };
const uint8_t* buttonU3_ptr = buttonU3;

// L + 2 => 36
const uint8_t  buttonL2[12]  = {0xAA, 0x24, 0x5E,
                                0xAA, 0x00, 0x1E
                               };
const uint8_t* buttonL2_ptr  = buttonL2;

// R + 2 => 40
const uint8_t  buttonR2[12] = {0xAA, 0x28, 0x1E,
                               0xAA, 0x00, 0x1E
                              };
const uint8_t* buttonR2_ptr = buttonR2;
//**************************************************************************************************
// FUNCTION PROTOTYPES
//**************************************************************************************************
// Robot states
void walkTowardsBall();
void positionRobot();
void performKick();

// Robot actions
void moveRobotForward();
void moveRobotLeft();
void moveRobotRight();
void moveRobotBack();
void slideRobotLeft();
void slideRobotRight();
void robotLookUp();
void robotLookDown();
void robotKick();

//**************************************************************************************************
//                                         Name: setup
//**************************************************************************************************
void setup() {

  // XBEE SETUP
  //  pinMode(statusLed, OUTPUT);
  //  pinMode(errorLed, OUTPUT);
  //  pinMode(dataLed,  OUTPUT);

  // start serial XBEE
  //  Serial1.begin(9600);
  //  xbee.setSerial(Serial);

  flashLed(statusLed, 3, 50);
  //END EXBEE SETUP

  //IR Setup
  //pinMode(20, INPUT);

  Serial.begin(9600);

  // Set the blue leds
  pinMode(2, OUTPUT);

  // This is needed for the OLED to function properly
  Wire.begin();
  Wire.setClock(400000L);

  //************************************************************************************************
  // 0.96 OLED Display settings
  //************************************************************************************************
  oled.begin(&Adafruit128x64, 0x3C);       // Initialize display with the I2C address of 0x3C
  oled.setScrollMode(SCROLL_MODE_OFF);
  oled.invertDisplay(true);
  oled.setInvertMode(false);
  oled.setLetterSpacing(3);
  oled.setFont(lcd5x7);
  oled.set1X();
  oled.clear();
  oled.println(F("Initializing OLED"));
  delay(400);

  // Start serial PIXY2
  Serial2.begin(115200);

  // start serial to CM-530 (1900 baud)
  Serial1.begin(1900);
  oled.println(F("Start SERIAL 1900"));
  oled.println();
  delay(100);

  // Initialize Pixy camera
  delay(4000);
  // Send command to initialize CM-530
  pixy.init();
  Serial1.write(moveFoward_ptr, 3);
  delay(210);
  Serial1.write(moveFoward_ptr + 3, 3);
  pixy.setLamp(0, 0);    // Flash head leds (upper-lights, lower light)
  pixy.changeProg("color_connected_components");
  oled.println(F("Initializing PIXY 2.."));
  oled.println();
  delay(300);

  // Flash up blue leds
  oled.println(F("Checking LIGHTS.."));
  oled.println();
  CheckLights();

  delay(400);
  oled.clearField(0, 5, 50);
  oled.setRow(5);
  oled.setCol(40);
  oled.set2X();
  oled.println(F("READY!"));
  delay(900);
  oled.clear();               // reset display

  // call the do_count function every 1000 millis (1 second)
  timerElapsed.every(1000, do_count);

  // Initialize FSM state
  //************************************************************************************************
  // Due to time constrains, the robot will assume that it already has seen the ball. For future
  // work, the robot should be able to find the ball, and should incorperate Hough Transform to
  // find the shape of the ball as well.
  //************************************************************************************************
  fsmState = ROBO_FSM_FIND ;
}

//**************************************************************************************************
// Name: loop
//**************************************************************************************************
void loop()
{
  // IR value in centimeters
  irSensor_value = sensor.distance(); //Calculate the distance in centimeters;
  Serial.println(irSensor_value);
  delay(200);
  oled.clearField(60, 5 , 5);
  oled.setCol(60);
  oled.setCol(5);
  oled.set1X();
  oled.print(F("DIST: "));
  oled.print(irSensor_value);

  // Get the number of objects
  pixyObjects = PixyDetectObject();

  // TESTING
  //  while (irSensor_value < 10 && pixyObjects == 1) {
  //    pixyObjects = PixyDetectObject();
  //    irSensor_value = sensor.distance();
  //
  //    LeftHandUp();
  //  }

  LeftHandUp();
  delay(5000);
  LeftHandDown();

  // XBEE CODE
  //RunXBee();
  //XBeePrint();

  //**************************************************************************************************
  // Name: DISPLAY
  //**************************************************************************************************
  timerElapsed.tick();
  DisplayMinutes();
  DisplaySeconds();

  //**************************************************************************************************
  // Name: Move Robot
  // Run state machine for every loop
  //**************************************************************************************************
  //  switch (fsmState)
  //  {
  //    // Find the ball
  //    case ROBO_FSM_FIND:
  //      uint8_t blocks = PixyDetectObject();
  //
  //      oled.clearField(32, 5, 4);   //print blocks value for debugging
  //      oled.setCol(0);
  //      oled.setRow(5);
  //      oled.print(F("Blocks: "));
  //      oled.print(blocks);
  //
  //      if (blocks > 1) {
  //        fsmState = ROBO_FSM_WALK;
  //      }
  //      break;
  //
  //    // Make the robot walk towards the ball.
  //    case ROBO_FSM_WALK:
  //      walkTowardsBall();
  //      break;
  //
  //    // Make the robot position the ball.
  //    case ROBO_FSM_POSITION:
  //      positionRobot();
  //      break;
  //
  //    // Have the robot kick the ball.
  //    case ROBO_FSM_KICK:
  //      performKick();
  //      break;
  //
  //    // Default case
  //    default:
  //      break;
  //  }
}

// FUNCTIONS

//**************************************************************************************************
// Name: walkTowardsBall
//**************************************************************************************************
void walkTowardsBall()
{
  uint16_t blocks; // Number of blocks found by Pixy camera
  uint16_t x_left;
  uint16_t x_right;
  uint16_t x_mid;
  uint16_t y_axis_point;

  // Get the number of blocks from the Pixy camera
  blocks = PixyDetectObject();

  // If the object is detected
  if (blocks)
  {
    // Determine where the middle point of the block is
    x_left = pixy.ccc.blocks[0].m_x;
    x_right = x_left + (pixy.ccc.blocks[0].m_width);
    x_mid = ((x_right - x_left) / 2) + x_left;

    y_axis_point = pixy.ccc.blocks[0].m_y;

    // If the block is to the left, we want to move left.
    if (x_mid < (X_AXIS_MID - maxDeviation))
    {
      moveRobotLeft();
    }
    // If the block is to the right, we want to move right.
    else if (x_mid > (X_AXIS_MID + maxDeviation))
    {
      moveRobotRight();
    }
    else
    {
      // If the ball is not close, walk towards the ball
      if ( y_axis_point < 183 )
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
  //delay(250);
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


  // Get the image details.
  blocks = PixyDetectObject();

  // If the image is detected
  if (blocks)
  {
    // Determine where the middle point of the block is
    x_left = pixy.ccc.blocks[0].m_x;
    x_right = x_left + (pixy.ccc.blocks[0].m_width);
    x_mid = ((x_right - x_left) / 2) + x_left;

    y_axis_point = pixy.ccc.blocks[0].m_y;

    // If the ball is close enough, but not aligned, shuffle
    //    if (x_mid < X_AXIS_R_FOOT)
    //    {
    //      slideRobotLeft();
    //    }
    //    else if(x_mid > (X_AXIS_R_FOOT + maxDevFoot))
    //    {
    //      slideRobotRight();
    //    }

    // If the block is not close enough, move forward.
    if (y_axis_point < 90)
    {
      moveRobotForward();
    }
    // If the block is to the right, we want to move right.
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
// Name: performKick
//**************************************************************************************************
void performKick()
{
  robotKick();
  fsmState = ROBO_FSM_FIND;
}


//**************************************************************************************************
// Name: moveRobotForward
//**************************************************************************************************
void moveRobotForward()
{
  Serial1.write(moveFoward_ptr, 3);
  delay(210);
  Serial1.write(moveFoward_ptr + 3, 3);
  SendDataMessage();
}

//**************************************************************************************************
// Name: moveRobotLeft
//**************************************************************************************************
void moveRobotLeft()
{
  Serial1.write(moveLeft_ptr, 3);
  delay(210);
  Serial1.write(moveLeft_ptr + 3, 3);
  SendDataMessage();
}

//**************************************************************************************************
// Name: moveRobotRight
//**************************************************************************************************
void moveRobotRight()
{
  Serial1.write(moveRight_ptr, 3);
  delay(210);
  Serial1.write(moveRight_ptr + 3, 3);
  SendDataMessage();
}

//**************************************************************************************************
// Name: moveRobotBack
//**************************************************************************************************
void moveRobotBack()
{
  Serial1.write(moveBack_ptr, 3);
  delay(210);
  Serial1.write(moveBack_ptr + 3, 3);
  SendDataMessage();
}


//**************************************************************************************************
// Name: slideRobotLeft
//**************************************************************************************************
//void slideRobotLeft()
//{
//  Serial1.write(slideLeft_ptr, 3);
//  delay(210);
//  Serial1.write(slideLeft_ptr + 3, 3);
//}

//**************************************************************************************************
// Name: slideRobotRight
//**************************************************************************************************
//void slideRobotRight()
//{
//  Serial1.write(slideRight_ptr, 3);
//  delay(210);
//  Serial1.write(slideRight_ptr + 3, 3);
//}

//**************************************************************************************************
// Name: robotKick
//**************************************************************************************************
void robotKick()
{
  Serial1.write(button4_ptr, 3);
  delay(210);
  Serial1.write(button4_ptr + 3, 3);
  SendDataMessage();
}

void LeftHandUp() {
  Serial1.write(buttonU3_ptr, 3);
  delay(210);
  int byteSend = Serial1.write(buttonU3_ptr + 3, 3);

  oled.setRow(7);
  oled.setCol(50);
  oled.print(F("Bytes:"));
  oled.print(byteSend);
}

void LeftHandDown() {
  Serial1.write(buttonL2_ptr, 3);
  delay(210);
  int byteSend = Serial1.write(buttonL2_ptr + 3, 3);

  oled.setRow(7);
  oled.setCol(50);
  oled.print(F("Bytes:"));
  oled.print(byteSend);
}

void DisplaySeconds() {
  oled.setRow(1);
  oled.setCol(80);
  oled.print(seconds);
  oled.println(F(" SEC"));
}

void DisplayMinutes() {
  oled.setRow(1);
  oled.setCol(0);
  oled.set1X();
  oled.print(F("ON: "));
  oled.setCol(30);
  oled.print(minutes);
  oled.print(F(" MIN"));

}

bool XBeePrint() {
  oled.clearField(87, 4, 8);
  oled.set1X();
  oled.setRow(4);
  oled.setCol(55);
  oled.print(F("XBEE: "));
  if (true) {
    //    oled.print(F("..."));
  }
  else {
    oled.print(F("Received"));
  }
}

// Count the minutes and seconds
bool do_count(void *) {
  seconds++;

  if (seconds == 59) {
    minutes++;
    seconds = 0;

    // Clear timer line every minute
    oled.clear(0, 150, 0, 0); // (startCOl, endCOL, startROW, endRow)

    // Blip lights every minute
    LightsOn();
    LightsOff();
  }

  if (seconds == 3) {
    // Clear lights ON message
    oled.clear(0, 150, 2, 2); // (startCOl, endCOL, startROW, endRow)
  }
  else if (seconds == 7) {
    // Clear lights OFF message
    oled.clear(0, 150, 3, 3); // (startCOl, endCOL, startROW, endRow)
  }
  else if (seconds % 2 == 0) {
    oled.clearField(32, 5, 4);   //clear blocks value
    oled.clearField(40, 3, 14);  // clearField(col, row , int - number of characters to delete)
  }

  return true; // repeat? true
}

// Command send display
void SendDataMessage() {
  oled.setCol(40);
  oled.setRow(3);
  oled.clearField(40, 3, 14);  // clearField(col, row , int - number of characters to delete)
  oled.print(F("Command:"));
  oled.print(F(" SEND!"));
}


// Lights up blue leds
void CheckLights() {
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  delay(300);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  delay(300);
}

void LightsOn() {
  oled.clear(0, 150, 2, 2); // (startCOl, endCOL, startROW, endRow)
  oled.set1X();
  oled.setRow(2);
  oled.println(F("Lights: ON"));
  digitalWrite(2, HIGH);
}

void LightsOff() {
  oled.clear(0, 150, 3, 3); // (startCOl, endCOL, startROW, endRow)
  oled.set1X();
  oled.setRow(3);
  oled.println(F("Lights: OFF"));
  digitalWrite(2, LOW);
}

void RunXBee() {
  // continuously reads packets, looking for RX16 or RX64

  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) {
    // got something

    if (xbee.getResponse().getApiId() == RX_16_RESPONSE || xbee.getResponse().getApiId() == RX_64_RESPONSE) {
      // got a rx packet

      if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
        xbee.getResponse().getRx16Response(rx16);
        option = rx16.getOption();
        data = rx16.getData(0);
      } else {
        xbee.getResponse().getRx64Response(rx64);
        option = rx64.getOption();
        data = rx64.getData(0);
      }

      // TODO check option, rssi bytes
      flashLed(statusLed, 1, 10);

      // set dataLed PWM to value of the first byte in the data
      analogWrite(dataLed, data);
    } else {
      // not something we were expecting
      flashLed(errorLed, 1, 25);
    }
  } else if (xbee.getResponse().isError()) {
    //nss.print("Error reading packet.  Error code: ");
    //nss.println(xbee.getResponse().getErrorCode());
    // or flash error led
  }
}

uint8_t PixyDetectObject() {
  static int i = 0;
  int j;
  char buf[64];
  int32_t panOffset, tiltOffset;

  // get active blocks from Pixy
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {
    i++;

    if (i % 60 == 0) {
      Serial.println(i);

      //      oled.setCol(0);
      //      oled.setRow(6);
      //      oled.print(F("Tracking.."));
    }

    // calculate pan and tilt "errors" with respect to first object (blocks[0]),
    // which is the biggest object (they are sorted by size).
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)pixy.ccc.blocks[0].m_x;
    tiltOffset = (int32_t)pixy.ccc.blocks[0].m_y - (int32_t)pixy.frameHeight / 2;

    // update loops
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // set pan and tilt servos
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);

#if 0 // for debugging
    sprintf(buf, "%ld %ld %ld %ld", rotateLoop.m_command, translateLoop.m_command, left, right);
    Serial.println(buf);
#endif

  }
  else // no object detected, go into reset state
  {
    //    oled.clearField(0, 6, 10);    // clearField(col, row , int - number of characters to delete)
    //    oled.set1X();
    //    oled.setCol(0);
    //    oled.setRow(6);
    //    oled.print(F("Lost.."));

    panLoop.reset();
    tiltLoop.reset();
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);
  }

  return  pixy.ccc.getBlocks();
}
