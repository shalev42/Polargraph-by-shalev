// polargraph code edited by shalev: 

//libraries for us to use in this code: (compatable on pi)
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <Servo.h>
#include <Arduino.h>

// pins for motors:
#define X_DIR_PIN 2
#define X_STEP_PIN 5
#define Y_DIR_PIN 3
#define Y_STEP_PIN 6
#define STEPS_PER_REVOLUTION 200

//------------------------------------------------------------------------------
// settings for the movment of the motors:

const int SpeedOFMotors = 800; // set speed each motor movement
const int stepsPerMotor = 25; // set the number of steps for each motor movement
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
// homing and buttons defenition:

#define LS_LEFT_PIN  (A1) // switch for homing left motor

#define LS_RIGHT_PIN  (A0)// switch for homing right motor

int xDirection = 1;

int yDirection = 1;

const int buttonPins[] = {12, 13}; // define button pins

const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]); // get the number of buttons
//------------------------------------------------------------------------------

// Define constants for motor direction:

#define CLOCKWISE 0

#define COUNTER_CLOCKWISE 1

unsigned long previousMillis = 0;

const long interval = 2000;
//------------------------------------------------------------------------------

// old code stuff for the neew code to work:


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
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//code for encoders:
const int ENCODER_A_BIT_0 = 11;
const int ENCODER_A_BIT_1 = 10;
const int ENCODER_B_BIT_0 = A2;
const int ENCODER_B_BIT_1 = A3;
int16_t old_encoder_A_count = 0;
int16_t old_encoder_B_count = 0;
int16_t encoder_A_count = 0;
int16_t encoder_B_count = 0;
int rangLimit = 100;

int readEncoder(int bit0, int bit1) {
  return (digitalRead(bit0) << 1) | digitalRead(bit1);
}

void readEncoders() {
  static int old_encoder_A_state = readEncoder(ENCODER_A_BIT_0, ENCODER_A_BIT_1);
  static int old_encoder_B_state = readEncoder(ENCODER_B_BIT_0, ENCODER_B_BIT_1);

  int new_encoder_A_state = readEncoder(ENCODER_A_BIT_0, ENCODER_A_BIT_1);
  int new_encoder_B_state = readEncoder(ENCODER_B_BIT_0, ENCODER_B_BIT_1);

  if (new_encoder_A_state != old_encoder_A_state) {
    if ((old_encoder_A_state == 0b01 && new_encoder_A_state == 0b00) || (old_encoder_A_state == 0b10 && new_encoder_A_state == 0b11)) {
      if (encoder_A_count < rangLimit) {
        encoder_A_count++;
      }
    } else if ((old_encoder_A_state == 0b00 && new_encoder_A_state == 0b01) || (old_encoder_A_state == 0b11 && new_encoder_A_state == 0b10)) {
      if (encoder_A_count > -rangLimit) {
        encoder_A_count--;
      }
    }
    old_encoder_A_state = new_encoder_A_state;
  }

  if (new_encoder_B_state != old_encoder_B_state) {
    if ((old_encoder_B_state == 0b01 && new_encoder_B_state == 0b00) || (old_encoder_B_state == 0b10 && new_encoder_B_state == 0b11)) {
      if (encoder_B_count < rangLimit) {
        encoder_B_count++;
      }
    } else if ((old_encoder_B_state == 0b00 && new_encoder_B_state == 0b01) || (old_encoder_B_state == 0b11 && new_encoder_B_state == 0b10)) {
      if (encoder_B_count > -rangLimit) {
        encoder_B_count--;
      }
    }
    old_encoder_B_state = new_encoder_B_state;
  }
}


//------------------------------------------------------------------------------

// parameters for manual coordinates:
long disx = 0;
long disy = 0;

/*
long disx = 75;
long disy = 60;
long alfa = 38.659;
long beta = 11.31;

  // Calculate new alfa
  double tan_alfa = disy / (disx - 70.0);
  double radian_alfa = atan(tan_alfa);
  alfa = radian_alfa * 180 / PI;

  // Calculate new beta
  double tan_beta = disy / (130.0 - disx);
  double radian_beta = atan(tan_beta);
  beta = radian_beta * 180 / PI;
  
*/
//------------------------------------------------------------------------------

//Define iterator value for chaging drawings:
int b = 0;
int c = 0;
unsigned long timerStart = 0;
const unsigned long timerDuration = 1000 *(60000); // 1 minute in milliseconds delay between random drawings
//------------------------------------------------------------------------------

// servo stuff:
Servo myservo;  // create servo object to control a servo
int countServo = 0;
// debounce for the button:
int buttonState = HIGH;  // Current state of the button (HIGH means not pressed)
int lastButtonState = HIGH;  // Previous state of the button (HIGH means not pressed)
unsigned long lastDebounceTime = 0;  // Last time the button state was toggled
unsigned long debounceDelay = 50;    // Debounce time in milliseconds
int test = 0;  // Variable to keep track of the current direction of the servo

//------------------------------------------------------------------------------

struct point { 

  float x; 

  float y; 

  float z; 

};
//------------------------------------------------------------------------------

struct point actuatorPos;
// plotter position 
float posx;
float posy;
static float posz;  // pen state
static float feed_rate = 0;
// pen state
static int ps;
#define BAUD            (115200)    
#define MAX_BUF         (64)    
int xstate;
int ystate;
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
  posx = x;
  posy = y;
  Serial.println("move to: ");
  Serial.println(posx);
  Serial.println(posy);  
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
  Serial.print("X,Y = ");
  Serial.print(posx);
  Serial.print(",");
  Serial.print(posy);
  Serial.print("\t");
  Serial.print("LastStep1, LastStep2 = ");
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
void executeCode() {
  // Code to be executed after one minute
  if (c == 0) {
    moveto(0 ,0);
    moveto(30 ,0);
    moveto(30 ,30);
    moveto(-30 ,30);
    moveto(-30 ,-30);
    moveto(30 ,-30);
    moveto(30 ,0);
    moveto(0 ,0);
    c = 1;  
  }
  else if (c == 1) {
    moveto(0 ,0);
    moveto(20, 0);
    moveto(40, 14.64);
    moveto(40, 34.64);
    moveto(20, 49.29);
    moveto(0, 34.64);
    moveto(0, 14.64);
    moveto(20, 0);
    moveto(0 ,0);
    c = 2;
   }

  else if (c == 2) {
    moveto(0, 0);       // Start point from (0, 0)
    moveto(4.14, 7.36);
    moveto(9.39, 14.14);
    moveto(15.36, 19.39);
    moveto(21.64, 22.64);
    moveto(27.86, 23.85);
    moveto(33.57, 22.83);
    moveto(38.38, 19.57);
    moveto(41.96, 14.35);
    moveto(44.12, 7.58);
    moveto(44.68, 0.77);
    moveto(43.60, -5.10);
    moveto(41.00, -10.31);
    moveto(36.99, -15.00);
    moveto(31.81, -18.67);
    moveto(25.76, -20.95);
    moveto(19.27, -21.58);
    moveto(12.84, -20.47);
    moveto(7.00, -17.74);
    moveto(2.28, -13.60);
    moveto(0, 0);       // Return to the start point
    c = 0;
   }

   
}               


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void setup() {
  Serial.begin(BAUD);
  Serial.println("Booting...");
  // motors:
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  // homing knobs:
  pinMode(LS_LEFT_PIN, INPUT_PULLUP);
  pinMode(LS_RIGHT_PIN, INPUT_PULLUP);
  //encoders:
  pinMode(ENCODER_A_BIT_0, INPUT_PULLUP);
  pinMode(ENCODER_A_BIT_1, INPUT_PULLUP);
  pinMode(ENCODER_B_BIT_0, INPUT_PULLUP);
  pinMode(ENCODER_B_BIT_1, INPUT_PULLUP); 
   
  Serial.println("Booting complete, start homing...");
  myservo.attach(A5);  // attaches the servo on pin A5 to the servo object

  
  // Start homing
  //moveDown(1400, 700);
  //homing();
  //moveto(0 ,0); // move to middle of the board
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
void moveMotor(int dirPin, int stepPin, int steps) {
          digitalWrite(dirPin, steps > 0 ? HIGH : LOW); // set motor direction based on sign of steps
          for (int i = 0; i < abs(steps); i++) {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(SpeedOFMotors);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(SpeedOFMotors);
  }
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

void loop(){
      
     //checks for inactivity = button pressing if not - start a timer of 1 minute 
    if (millis() - timerStart >= timerDuration) {
    // Timer duration reached, execute the code
    executeCode();
    // Reset the timer
    timerStart = millis();
  }

  readEncoders();

  int count_A = encoder_A_count / 2;
  int count_B = encoder_B_count / 2;

  // Check if the encoder count increased by one step
  if (encoder_A_count > old_encoder_A_count) {
    // Move Y motor counter-clockwise - right up by one step
    moveMotor(Y_DIR_PIN, Y_STEP_PIN, -stepsPerMotor);
  } else if (encoder_A_count < old_encoder_A_count) {
    // Move Y motor clockwise - right down by one step
    moveMotor(Y_DIR_PIN, Y_STEP_PIN, stepsPerMotor);
  }

  // Update the old encoder value for the next iteration
  old_encoder_A_count = encoder_A_count;

  // Check if the encoder count increased by one step
  if (encoder_B_count > old_encoder_B_count) {
    // Move X motor counter-clockwise - right up by one step
    moveMotor(X_DIR_PIN, X_STEP_PIN, stepsPerMotor);
  } else if (encoder_B_count < old_encoder_B_count) {
    // Move X motor clockwise - right down by one step
    moveMotor(X_DIR_PIN, X_STEP_PIN, -stepsPerMotor);
  }

  // Update the old encoder value for the next iteration
  old_encoder_B_count = encoder_B_count;

    Serial.print("Encoder A Count: ");
    Serial.print(encoder_A_count/2);
    Serial.print(", Encoder B Count: ");
    Serial.println(encoder_B_count/2);
  
/*
      // Move Y motor based on encoder_A_count
    if (encoder_A_count > 0) {
      moveMotor(Y_DIR_PIN, Y_STEP_PIN, -motor_steps_A); // Move Y motor counter-clockwise - right up
    } else if (encoder_A_count < 0) {
      moveMotor(Y_DIR_PIN, Y_STEP_PIN, motor_steps_A); // Move Y motor clockwise - right down
    }
  
    // Move X motor based on encoder_B_count
    if (encoder_B_count > 0) {
      moveMotor(X_DIR_PIN, X_STEP_PIN, -motor_steps_B); // Move X motor counter-clockwise - left down
    } else if (encoder_B_count < 0) {
      moveMotor(X_DIR_PIN, X_STEP_PIN, motor_steps_B); // Move X motor clockwise - left up
    }
              

              // Print encoder values
    Serial.print("Encoder A Count: ");
    Serial.print(encoder_A_count/2);
    Serial.print(", Encoder B Count: ");
    Serial.println(encoder_B_count/2);
    
      // read button states and move motors accordingly
          for (int i = 0; i < numButtons; i++) {
            int state = digitalRead(buttonPins[i]);
            if (i == 0) { 
                  // If the button is pressed (LOW), change the direction of the servo and set 'test' variable accordingly
                  if (buttonState == HIGH) {
                    static int test = 0;  // Variable to keep track of the current direction of the servo
                    if (test == 0) {
                      myservo.write(40);
                        delay(500);
                      test = 1;
                    } else {
                      myservo.write(120);
                        delay(500);
                      test = 0;
                    }
                    delay(50); // Optional delay to avoid rapid servo movements due to button's mechanical noise
                  }
              }
              
              else if (i == 0) {
                timerStart = millis();// box
                  if (b == 0) {
                       moveto(0 ,0);
                       moveto(30 ,0);
                       moveto(30 ,30);
                       moveto(-30 ,30);
                       moveto(-30 ,-30);
                       moveto(30 ,-30);
                       moveto(30 ,0);
                       moveto(0 ,0);
                       b = 1;                  
                       }
                    else if (b == 1) {
                      moveto(0 ,0);
                      moveto(20 ,0);
                      moveto(20 ,20);
                      moveto(-20 ,20);
                      moveto(-20 ,-20);
                      moveto(20 ,-20);
                      moveto(20 ,0);
                      moveto(0 ,0);
                      b = 2;  
                    }
                    else if (b == 2) {
                      moveto(0 ,0);
                      moveto(10 ,0);
                      moveto(10 ,10);
                      moveto(-10 ,10);
                      moveto(-10 ,-10);
                      moveto(10 ,-10);
                      moveto(10 ,0);
                      moveto(0 ,0);
                      b = 0;  
                    }
                    else {
                      Serial.println("Invalid value of i");
                   }
                }
                else {
                    Serial.println("end of if");
                  }
              }
              
            
*/
     
}







        




  
