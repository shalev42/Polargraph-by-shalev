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
const int SpeedOFMotors = 500; // set speed each motor movement
const int stepsPerMotor = 1; // set the number of steps for each motor movement
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#define X_SEPARATION  1300    //The horizontal distance above the two ropes mm       

#define LIMXMAX       ( X_SEPARATION*0.5)  //x-axis maximum value 0 is at the center of the artboard

#define LIMXMIN       (-X_SEPARATION*0.5)  //x-axis minimum

#define LIMYMAX         (-500)   //The maximum value of the y-axis is at the bottom of the drawing board

#define LIMYMIN         (500)  //The minimum value of the y-axis is the vertical distance from the fixed point of the left and right lines to the pen at the top of the drawing board. 
//Try to measure and place it accurately, and there will be distortion if the error is too large
//When the value is reduced, the drawing becomes thinner and taller, and when the value is increased, the drawing becomes short and fat

// the following variables are unsigned longs because the time, measured in

// milliseconds, will quickly become a bigger number than can be stored in an int.
//------------------------------------------------------------------------------

#define LS_LEFT_PIN  (A1) // switch for homing left motor

#define LS_RIGHT_PIN  (A0)// switch for homing right motor


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

      delayMicroseconds(SpeedOFMotors);

      digitalWrite(stepPin, LOW);

      delayMicroseconds(SpeedOFMotors);

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

      delayMicroseconds(SpeedOFMotors);

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

      delayMicroseconds(SpeedOFMotors);

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

  unsigned long timeout = millis() + 6000; // Set a timeout of 5 seconds

  while (digitalRead(LS_LEFT_PIN) == LOW && millis() < timeout) {

    setDirectionLeft(COUNTER_CLOCKWISE);

    stepMotorLeft();

  }



  // Move the right motor counter-clockwise until the right microswitch is pressed or timeout occurs

  timeout = millis() + 6000; // Reset the timeout

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

  teleport(0, -200);

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

void where() {
  Serial.print("X,Y=  ");
  Serial.print(posx);
  Serial.print(",");
  Serial.print(posy);
  Serial.print("\t");
  Serial.print("Lst1,Lst2=  ");
  Serial.print(laststep1);
  Serial.print(",");
  Serial.println(laststep2);
  Serial.println("");
}

//------------------------------------------------------------------------------

void stepMotorRight() {

  digitalWrite(X_STEP_PIN, HIGH);

  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed

  digitalWrite(X_STEP_PIN, LOW);

  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed

}

//------------------------------------------------------------------------------

void stepMotorLeft() {

  digitalWrite(Y_STEP_PIN, HIGH);

  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed

  digitalWrite(Y_STEP_PIN, LOW);

  delayMicroseconds(SpeedOFMotors); // Adjust this value to change motor speed

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
    delayMicroseconds(SpeedOFMotors);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(SpeedOFMotors);
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
  

  //moveDown(1400, 700);

  homing();

  moveto(0 ,0); // move to middle of the board

  teleport(0, 0); // set position to (0,0) 

  //line_safe(0 ,100);


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
      where(); 
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
              else if (i == 4) { //logo of the museume 
                   teleport(0, 0);
                    moveto(128.6423,75.0304);
                    moveto(108.1619,16.4679);
                    moveto(108.0501,16.21454);
                    moveto(107.914,16.00098);
                    moveto(107.7618,15.82404);
                    moveto(107.6019,15.68054);
                    moveto(107.4423,15.56738);
                    moveto(107.2914,15.48145);
                    moveto(107.1576,15.41962);
                    moveto(107.0491,15.37866);
                    moveto(106.8253,15.31909);
                    moveto(106.6261,15.29474);
                    moveto(106.4511,15.2984);
                    moveto(106.3002,15.32294);
                    moveto(106.0696,15.40588);
                    moveto(105.9329,15.48608);
                    moveto(105.6587,15.71411);
                    moveto(105.4721,15.98816);
                    moveto(105.4094,16.14722);
                    moveto(105.3657,16.32355);
                    moveto(105.3401,16.51904);
                    moveto(105.3318,16.7356);
                    moveto(105.3318,59.14166);
                    moveto(105.2999,60.21869);
                    moveto(105.205,61.27795);
                    moveto(105.0482,62.31763);
                    moveto(104.8302,63.33575);
                    moveto(104.5524,64.33038);
                    moveto(104.2156,65.29962);
                    moveto(103.8209,66.24158);
                    moveto(103.3693,67.15424);
                    moveto(102.8619,68.03583);
                    moveto(102.2997,68.88434);
                    moveto(101.6837,69.69781);
                    moveto(101.0149,70.47443);
                    moveto(100.2944,71.21216);
                    moveto(99.52313,71.90912);
                    moveto(98.70215,72.56348);
                    moveto(97.83252,73.17322);
                    moveto(97.02167,73.67316);
                    moveto(96.18976,74.12268);
                    moveto(95.33881,74.52167);
                    moveto(94.47076,74.86969);
                    moveto(93.58752,75.16656);
                    moveto(92.6911,75.41217);
                    moveto(91.78351,75.60614);
                    moveto(90.86664,75.74823);
                    moveto(89.9425,75.8382);
                    moveto(89.01312,75.87579);
                    moveto(88.08044,75.86078);
                    moveto(87.14642,75.79297);
                    moveto(86.21307,75.672);
                    moveto(85.28229,75.49768);
                    moveto(84.35602,75.26978);
                    moveto(83.4364,74.98804);
                    moveto(82.51062,74.64648);
                    moveto(81.61279,74.25635);
                    moveto(80.74432,73.81903);
                    moveto(79.90668,73.33594);
                    moveto(79.10126,72.80847);
                    moveto(78.32959,72.2381);
                    moveto(77.59308,71.62616);
                    moveto(76.89313,70.97412);
                    moveto(76.23126,70.28339);
                    moveto(75.60895,69.55536);
                    moveto(75.02753,68.7915);
                    moveto(74.48853,67.99316);
                    moveto(73.99335,67.1618);
                    moveto(73.54352,66.29877);
                    moveto(73.1405,65.40558);
                    moveto(72.78564,64.48358);
                    moveto(55.948,16.4679);
                    moveto(55.83624,16.21454);
                    moveto(55.70013,16.00098);
                    moveto(55.54797,15.82404);
                    moveto(55.388,15.68054);
                    moveto(55.22845,15.56738);
                    moveto(55.0777,15.48145);
                    moveto(54.94385,15.41962);
                    moveto(54.83533,15.37866);
                    moveto(54.61157,15.31909);
                    moveto(54.41229,15.29474);
                    moveto(54.23724,15.2984);
                    moveto(54.08636,15.32294);
                    moveto(53.85583,15.40588);
                    moveto(53.71912,15.48608);
                    moveto(53.44489,15.71411);
                    moveto(53.25824,15.98816);
                    moveto(53.1955,16.14722);
                    moveto(53.15179,16.32355);
                    moveto(53.12628,16.51904);
                    moveto(53.11792,16.7356);
                    moveto(53.11792,59.14166);
                    moveto(53.08606,60.21869);
                    moveto(52.99115,61.27808);
                    moveto(52.83429,62.31781);
                    moveto(52.61633,63.33594);
                    moveto(52.33844,64.33063);
                    moveto(52.00165,65.29987);
                    moveto(51.60693,66.24182);
                    moveto(51.15533,67.15454);
                    moveto(50.64789,68.03607);
                    moveto(50.08569,68.88458);
                    moveto(49.46967,69.69806);
                    moveto(48.80084,70.47461);
                    moveto(48.08026,71.21234);
                    moveto(47.30908,71.9093);
                    moveto(46.48816,72.56354);
                    moveto(45.61865,73.17322);
                    moveto(44.8078,73.67316);
                    moveto(43.97589,74.12268);
                    moveto(43.12494,74.52167);
                    moveto(42.2569,74.86969);
                    moveto(41.37366,75.16656);
                    moveto(40.47723,75.41217);
                    moveto(39.56964,75.60614);
                    moveto(38.65289,75.74823);
                    moveto(37.72882,75.8382);
                    moveto(36.7995,75.87579);
                    moveto(35.86688,75.86078);
                    moveto(34.93292,75.79297);
                    moveto(33.99957,75.672);
                    moveto(33.06879,75.49768);
                    moveto(32.14252,75.26978);
                    moveto(31.2229,74.98804);
                    moveto(30.29712,74.64648);
                    moveto(29.39923,74.25635);
                    moveto(28.5307,73.81903);
                    moveto(27.69299,73.336);
                    moveto(26.88751,72.80853);
                    moveto(26.11584,72.23822);
                    moveto(25.37933,71.6264);
                    moveto(24.67944,70.97449);
                    moveto(24.01764,70.28387);
                    moveto(23.39532,69.55609);
                    moveto(22.81403,68.79242);
                    moveto(22.27509,67.99432);
                    moveto(21.78009,67.16327);
                    moveto(21.33038,66.3006);
                    moveto(20.92743,65.40784);
                    moveto(20.57275,64.48633);
                    moveto(0,5.896362);
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
            delayMicroseconds(SpeedOFMotors);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(SpeedOFMotors);
  }
}



  
