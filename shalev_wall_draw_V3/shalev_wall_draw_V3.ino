// polargraph code edited by shalev: 


#include <stdlib.h>

#include <stdio.h>

#include <time.h>


#define X_DIR_PIN 2

#define X_STEP_PIN 5

#define Y_DIR_PIN 3

#define Y_STEP_PIN 6

#define STEPS_PER_REVOLUTION 200


// main motors regular movement:
//------------------------------------------------------------------------------
const int speedOFbuttons = 1000; // set speed each motor movement
const int stepsPerMotor = 10; // set the number of steps for each motor movement
//------------------------------------------------------------------------------

// movement for the homing procces 
//------------------------------------------------------------------------------
#define SpeedOfHoming 8000 // set speed to the motors 
#define NUM_STEPS 20    // chage this to change the number of steps per push of a button
//------------------------------------------------------------------------------
//set motor speed: MoveTo command !!
int motorSpeed = 300; //homing
int motorSpeed1 = 100; // movment of the head
//------------------------------------------------------------------------------

#define X_SEPARATION  1470    //The horizontal distance above the two ropes mm       

#define LIMXMAX       ( X_SEPARATION*0.5)  //x-axis maximum value 0 is at the center of the artboard

#define LIMXMIN       (-X_SEPARATION*0.5)  //x-axis minimum

#define LIMYMAX         (-520)   //The maximum value of the y-axis is at the bottom of the drawing board

#define LIMYMIN         (520)  //The minimum value of the y-axis is the vertical distance from the fixed point of the left and right lines to the pen at the top of the drawing board. 
//Try to measure and place it accurately, and there will be distortion if the error is too large
//When the value is reduced, the drawing becomes thinner and taller, and when the value is increased, the drawing becomes short and fat

// the following variables are unsigned longs because the time, measured in

// milliseconds, will quickly become a bigger number than can be stored in an int.
//------------------------------------------------------------------------------

#define LS_LEFT_PIN  (A0) // switch for homing left motor

#define LS_RIGHT_PIN  (A1)// switch for homing right motor


#define DEBOUNCE_DELAY 3000

#define STEP_DELAY 600


int xDirection = 1;

int yDirection = 1;

int xPotValue = 0;

int yPotValue = 0;

unsigned long debounceDelay1 = 2000;

unsigned long lastDebounceTime1 = 0;


//int buttonPins[numButtons] = {12, 9, A2, A3, 10, 11};

const int buttonPins[] = {12, 9, A2, A3, 10, 11}; // define button pins
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]); // get the number of buttons


// Define constants for motor direction

#define CLOCKWISE 0

#define COUNTER_CLOCKWISE 1

unsigned long previousMillis = 0;

const long interval = 2000;

#define STEPS_PER_TURN  (2048)  

#define SPOOL_DIAMETER  (35)    

#define SPOOL_CIRC      (SPOOL_DIAMETER * 3.1416)  // 35*3.14=109.956

#define TPS             (SPOOL_CIRC / STEPS_PER_TURN)  

#define step_delay      1  

#define TPD             300  

#define M1_REEL_OUT     1    

#define M1_REEL_IN      -1     

#define M2_REEL_OUT     -1      

#define M2_REEL_IN      1     

static long laststep1, laststep2; 

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

#define PEN_UP_ANGLE    160 

#define PEN_DOWN_ANGLE  100  

#define PEN_DOWN 1  

#define PEN_UP 0    

struct point { 

  float x; 

  float y; 

  float z; 

};
//------------------------------------------------------------------------------

struct point actuatorPos;

// plotter position 

static float posx;

static float posy;

static float posz;  // pen state

static float feed_rate = 0;

// pen state

static int ps;

#define BAUD            (115200)    

#define MAX_BUF         (64)    

//Servo pen;

int penstatenow = 0;

int lastpenstate = 1;

int xstate = 0;

int ystate = 0;

int movment = 5;

//------------------------------------------------------------------------------

void move_step(int stepCount, int stepPin,int dirPin)
{

  // set the axis direction  

  if (stepCount > 0){

      digitalWrite(dirPin, HIGH);

    }

  else{

    digitalWrite(dirPin, LOW);

    }

    for (int i = 0; i < abs(stepCount); i++) {

      

      // These four lines result in 1 step:

      digitalWrite(stepPin, HIGH);

      delayMicroseconds(motorSpeed);

      digitalWrite(stepPin, LOW);

      delayMicroseconds(motorSpeed);

    }

        //Serial.print("moved!");

  }



//------------------------------------------------------------------------------


// theta = acos((a*a+b*b-c*c)/(2*a*b));

void FK(float l1, float l2,float &x,float &y) {

  float a=l1 * TPS;

  float b=X_SEPARATION;

  float c=l2 * TPS;

  float theta = acos((a*a+b*b-c*c)/(2.0*a*b));

  x = cos(theta)*l1 + LIMXMIN;

  y = sin(theta)*l1 + LIMYMIN;          

/*   float theta = (a*a+b*b-c*c)/(2.0*a*b);

  x = theta*l1 + LIMXMIN;

  y = sqrt (1.0 - theta * theta ) * l1 + LIMYMIN;*/

}

//------------------------------------------------------------------------------
void IK(float x,float y,long &l1, long &l2) {

  float dy = y - LIMYMIN;

  float dx = x - LIMXMIN;

  l1 = round(sqrt(dx*dx+dy*dy) / TPS);

  dx = x - LIMXMAX;

  l2 = round(sqrt(dx*dx+dy*dy) / TPS);

}

//------------------------------------------------------------------------------
void pen_state(int pen_st) {

  if(pen_st==PEN_DOWN) {

        ps=PEN_DOWN_ANGLE;

        // Serial.println("Pen down");

      } else {

        ps=PEN_UP_ANGLE;

        //Serial.println("Pen up");

      }

}
//------------------------------------------------------------------------------

void pen_down()

{

  if (ps==PEN_UP_ANGLE)

  {

    ps=PEN_DOWN_ANGLE;

//    pen.write(ps);

    delay(TPD);

  }

}
//------------------------------------------------------------------------------

void pen_up()

{

  if (ps==PEN_DOWN_ANGLE)

  {

    ps=PEN_UP_ANGLE;

//    pen.write(ps);

  }

}
//------------------------------------------------------------------------------

// returns angle of dy/dx as a value from 0...2PI

static float atan3(float dy, float dx) {

  float a = atan2(dy, dx);

  if (a < 0) a = (PI * 2.0) + a;

  return a;

}

//------------------------------------------------------------------------------
// instantly move the virtual plotter position
// does not validate if the move is valid
static void teleport(float x, float y) {

  posx = x;

  posy = y;

  xstate = x;

  ystate = y;

  long l1,l2;

  IK(posx, posy, l1, l2);

  laststep1 = l1;

  laststep2 = l2;

}

//------------------------------------------------------------------------------

void moveto(float x,float y) {

    long l1,l2;

    IK(x,y,l1,l2);

    long d1 = l1 - laststep1;

    long d2 = l2 - laststep2;

    long ad1=abs(d1);

    long ad2=abs(d2);

    int dir1=d1>0 ? M1_REEL_IN : M1_REEL_OUT;

    int dir2=d2>0 ? M2_REEL_IN : M2_REEL_OUT;

    long over=0;

    long i;

  if(ad1>ad2) {

    for(i=0;i<ad1;++i) {

      move_step(dir1,X_STEP_PIN,X_DIR_PIN);

      over+=ad2;

      if(over>=ad1) {

        over-=ad1;

          move_step(dir2,Y_STEP_PIN,Y_DIR_PIN);

      }

      delayMicroseconds(motorSpeed);

     }

  } 

  else {

    for(i=0;i<ad2;++i) {

        move_step(dir2,Y_STEP_PIN,Y_DIR_PIN);

      over+=ad1;

      if(over>=ad2) {

        over-=ad2;

        move_step(dir1,X_STEP_PIN,X_DIR_PIN);

      }

      delayMicroseconds(motorSpeed);

    }

  }

  laststep1=l1;

  laststep2=l2;

  posx=x;

  posy=y;

}
//------------------------------------------------------------------------------

static void line_safe(float x,float y) {

  // split up long lines to make them straighter?

  float dx=x-posx;

  float dy=y-posy;

  float len=sqrt(dx*dx+dy*dy);

  if(len<=TPS) {

    moveto(x,y);

    return;

  }

  long pieces=floor(len/TPS);

  float x0=posx;

  float y0=posy;

  float a;

  for(long j=0;j<=pieces;++j) {

    a=(float)j/(float)pieces;

    moveto((x-x0)*a+x0,(y-y0)*a+y0);

  }

  moveto(x,y);

}
//------------------------------------------------------------------------------

void line(float x,float y) 

{

  line_safe(x,y);

}
//------------------------------------------------------------------------------

void box(float xx,float yy,float dx,float dy)

{

  pen_up();

  line_safe(xx , yy);

  pen_down();

  delay(TPD);

  line_safe(xx + dx, yy);

  delay(TPD);

  line_safe(xx + dx, yy+ dy);

  delay(TPD);

  line_safe(xx , yy + dy);

  delay(TPD);

  line_safe(xx , yy);

  pen_up();



}
//------------------------------------------------------------------------------

void moveDown(int steps, int delayTime) 

{

  digitalWrite(X_DIR_PIN, LOW); // Set direction to move down

  digitalWrite(Y_DIR_PIN, HIGH); // Set direction to move down

  for (int i = 0; i < steps; i++) {

    digitalWrite(X_STEP_PIN, HIGH); // Trigger a step

    delayMicroseconds(delayTime); // Wait for the specified delay

    digitalWrite(Y_STEP_PIN, HIGH); // Trigger a step

    delayMicroseconds(delayTime); // Wait for the specified delay

    

    digitalWrite(X_STEP_PIN, LOW); // Complete the step

    delayMicroseconds(delayTime); // Wait for the specified delay

    digitalWrite(Y_STEP_PIN, LOW); // Complete the step

    delayMicroseconds(delayTime); // Wait for the specified delay

  }

}
//------------------------------------------------------------------------------
void homing() {

  // Move the left motor clockwise until the left microswitch is pressed or timeout occurs

  unsigned long timeout = millis() + 5000; // Set a timeout of 5 seconds

  while (digitalRead(LS_LEFT_PIN) == LOW && millis() < timeout) {

    setDirectionLeft(COUNTER_CLOCKWISE);

    stepMotorLeft();

  }



  // Move the right motor counter-clockwise until the right microswitch is pressed or timeout occurs

  timeout = millis() + 5000; // Reset the timeout

  while (digitalRead(LS_RIGHT_PIN) == LOW && millis() < timeout) {

    setDirectionRight(CLOCKWISE);

    stepMotorRight();

  }



  // Set the motor positions to the home position

  setDirectionLeft(CLOCKWISE);

  setDirectionRight(COUNTER_CLOCKWISE);

  // Move the motors slightly to ensure they are at the home position

  timeout = millis() + 1000; // Set a timeout of 1 second

  while ((digitalRead(LS_LEFT_PIN) == HIGH || digitalRead(LS_RIGHT_PIN) == HIGH) && millis() < timeout) {

    stepMotorLeft();

    stepMotorRight();

  }

  teleport(0, -400);

}

//------------------------------------------------------------------------------

void setDirectionRight(int direction) {

  digitalWrite(X_DIR_PIN, direction);

}

//------------------------------------------------------------------------------

void setDirectionLeft(int direction) {

  digitalWrite(Y_DIR_PIN, direction);

}

//------------------------------------------------------------------------------

void stepMotorRight() {

  digitalWrite(X_STEP_PIN, HIGH);

  delayMicroseconds(motorSpeed1); // Adjust this value to change motor speed

  digitalWrite(X_STEP_PIN, LOW);

  delayMicroseconds(motorSpeed1); // Adjust this value to change motor speed

}

//------------------------------------------------------------------------------

void stepMotorLeft() {

  digitalWrite(Y_STEP_PIN, HIGH);

  delayMicroseconds(motorSpeed1); // Adjust this value to change motor speed

  digitalWrite(Y_STEP_PIN, LOW);

  delayMicroseconds(motorSpeed1); // Adjust this value to change motor speed

}

//------------------------------------------------------------------------------

void demo1()

{

 box(100,0,90,90);

 //line(15,0);

}
//------------------------------------------------------------------------------

// Generate random (x,y) coordinates within the given range and send them to the printer

void sendRandomCoordinates(int xMin, int xMax, int yMin, int yMax) {

  int x = random(xMin, xMax);

  int y = random(yMin, yMax);

 
  // Check if the generated x and y values are within range

  if (x >= xMin && x <= xMax && y >= yMin && y <= yMax) {

    moveto(x, y);

  }

}
//------------------------------------------------------------------------------

void moveMotor(int dirPin, int stepPin, int steps, bool clockwise) {
  digitalWrite(dirPin, clockwise ? HIGH : LOW); // set motor direction
  for(int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(SpeedOfHoming);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(SpeedOfHoming);
  }
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void setup() {
  
  Serial.begin(BAUD);

  Serial.println("Booting...");

  pinMode(Y_DIR_PIN, OUTPUT);

  pinMode(Y_STEP_PIN, OUTPUT);

  pinMode(X_DIR_PIN, OUTPUT);

  pinMode(X_STEP_PIN, OUTPUT);

  pinMode(LS_LEFT_PIN, INPUT_PULLUP);

  pinMode(LS_RIGHT_PIN, INPUT_PULLUP);

  Serial.println("Booting complete, start homing...");


  // Start homing

  // Move down by 100 steps with a delay of 700 microseconds between each step
  
/*
  moveDown(1400, 700);

  homing();

  moveto(0 ,120); // move to middle of the board

  teleport(0, 0); // set position to (0,0) 

  //line_safe(0 ,100);
*/ 

  Serial.println("homing done");
  
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }


  //moveto(0 ,0);

  //Serial.println("Test OK!");

  //demo1();

  // line_safe(0 , 0);

  // moveto(0 , 0);

  

}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void loop()

{

    //sendRandomCoordinates(0,30,-30,30);
      

      // read button states and move motors accordingly
          for (int i = 0; i < numButtons; i++) {
            int state = digitalRead(buttonPins[i]);
            if (state == LOW) {
              if (i == 0) {
                moveMotor(Y_DIR_PIN, Y_STEP_PIN, -stepsPerMotor); // move Y motor counter-clockwise - down
              } else if (i == 1) {
                moveMotor(Y_DIR_PIN, Y_STEP_PIN, stepsPerMotor); // move Y motor clockwise - up
              } else if (i == 2) {
                moveMotor(X_DIR_PIN, X_STEP_PIN, -stepsPerMotor); // move X motor counter-clockwise - left
              } else if (i == 3) {
                moveMotor(X_DIR_PIN, X_STEP_PIN, stepsPerMotor); // move X motor clockwise - right
              }
              else if (i == 4) { //triangle
                   teleport(0, 0);
                    moveto(15,0);
                    moveto(14,2);
                    moveto(14,5);
                    moveto(12,7);
                    moveto(11,9);
                    moveto(9,11);
                    moveto(7,12);
                    moveto(5,14);
                    moveto(2,14);
                    moveto(0,15);
                    moveto(-2,14);
                    moveto(-5,14);
                    moveto(-7,12);
                    moveto(-9,11);
                    moveto(-11,9);
                    moveto(-12,7);
                    moveto(-14,5);
                    moveto(-14,2);
                    moveto(-15,0);
                    moveto(-14,-2);
                    moveto(-14,-5);
                    moveto(-12,-7);
                    moveto(-11,-9);
                    moveto(-9,-11);
                    moveto(-7,-12);
                    moveto(-5,-14);
                    moveto(-2,-14);
                    moveto(0,-15);
                    moveto(2,-14);
                    moveto(5,-14);
                    moveto(7,-12);
                    moveto(9,-11);
                    moveto(11,-9);
                    moveto(12,-7);
                    moveto(14,-5);
                    moveto(14,-2);
                    moveto(15,0);
                    moveto(0,0);

              }

              else if (i == 5) { // box
                   teleport(0, 0);
                   moveto(0 ,0);
                   moveto(30 ,0);
                   moveto(30 ,30);
                   moveto(-30 ,30);
                   moveto(-30 ,-30);
                   moveto(30 ,-30);
                   moveto(30 ,0);
                   moveto(0 ,0);
               }
            }
          }
        }
        
        void moveMotor(int dirPin, int stepPin, int steps) {
          digitalWrite(dirPin, steps > 0 ? HIGH : LOW); // set motor direction based on sign of steps
          for (int i = 0; i < abs(steps); i++) {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(speedOFbuttons);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(speedOFbuttons);
  }
}



  
