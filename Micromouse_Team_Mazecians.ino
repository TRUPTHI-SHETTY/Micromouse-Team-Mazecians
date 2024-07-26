//Authors: Trupthi T. Shetty and Nishmitha C. V.

#include <Wire.h>
#include <EEPROM.h>
#include <Keypad.h>

//KeyPad Switch Logic (KeyPad Switch is used for switching b/w Initial and Fast/Final Run)
const byte ROW PROGMEM= 2; 
const byte COL PROGMEM= 2; 
char keys[ROW][COL] = {
  {'1','2'},
  {'3','4'},
};
byte rowPins[ROW] = {3,4,}; //r1,r2//connect to the row pinouts of the keypad
byte colPins[COL] = {11, 12,}; //l1,l2//connect to the column pinouts of the keypad
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROW, COL );


//GyroScope variables (GyroScope is used for making 90/180 degree turns based on detected angles)
float RatePitch;
float RateCalibrationPitch;
int RateCalibrationNumber;
float elapsedTime, currentTime, previousTime;
int16_t GyroX,GyroY,GyroZ;


//Motor Driver Variables
#define in1 7
#define in2 10
#define in3 8
#define in4 9
#define enA 5
#define enB 6

//IR sensor variables
const int FrontIRSensor PROGMEM = A2;
const int LeftIRSensor PROGMEM = A0;
const int RightIRSensor PROGMEM = A1;

//Variables with constant value(To maintain proper speed and adjust angles for turns)
const float distance PROGMEM=18;
float speedValue=65.0;
float vSpeed=speedValue/1000.0;
float targetAngle=160.0;//Fixed
float targetAngleTurn=75.0;
float wait=((distance/vSpeed)*2)+135;
float waitStop=150.0;
float waitTurn=0;

//Variables used for logic
int wallType=100;
int directionValue = 0;
int x=0;
int y=0;
int xvar=100;
int yvar=100;
int var=0;

int leftWall=1;
int rightWall=1;
int frontWall=1;

int flagLeftCount=0;
int flagRightCount=0;
int fx=0;
int fy=0;


const int rows PROGMEM= 16;
const int cols PROGMEM= 16;

// Temp Array value is used to make a new turn every time a cell is visited
int temp[rows][cols] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
// Direction Array to store direction of the micromouse on each cell 
int directionStore[rows][cols] = {{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100},{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100}};

//Variables for Final Run
int directionSt=0;
int directionCurrent=0;
int directionFinal=255;
bool flagFinal;
bool flag;


void gyro_signals(void) {
  //Gyroscope function
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  GyroX=Wire.read()<<8 | Wire.read();
  GyroY=Wire.read()<<8 | Wire.read();
  GyroZ=Wire.read()<<8 | Wire.read();
  RatePitch=(float)GyroY/65.5;
  
}


void forwardMotor()
{
digitalWrite(in1,HIGH);
digitalWrite(in2,LOW);
digitalWrite(in3,HIGH);
digitalWrite(in4,LOW);
analogWrite(enA,speedValue);
analogWrite(enB,speedValue);
}

void Stop()
{
digitalWrite(in1,LOW);
digitalWrite(in2,LOW);
digitalWrite(in4,LOW);
digitalWrite(in3,LOW);
analogWrite(enA,0);
analogWrite(enB,0);
}

void leftMotor()
{
digitalWrite(in1,LOW);//backward
digitalWrite(in2,HIGH);
digitalWrite(in3,HIGH);
digitalWrite(in4,LOW);
analogWrite(enA,speedValue);
analogWrite(enB,speedValue);
}

void rightMotor()
{
digitalWrite(in1,HIGH);
digitalWrite(in2,LOW);
digitalWrite(in3,LOW);
digitalWrite(in4,HIGH);
analogWrite(enA,speedValue);
analogWrite(enB,speedValue);
}


void forward()
{  
  //Moves the micomouse forward for a certain amount of constant wait time
  float startTime=micros();
  float timeLoop=0;
  float pitch;
  while(timeLoop<wait)
  {   

  currentTime = micros();
  timeLoop=(currentTime-startTime)/1000; //ms

  forwardMotor(); 

  }

}
void rightTurn()
{
  //Moves the micomouse right side for a certain angle(90 degrees)
  float targetAngleRight=-targetAngleTurn;
  float pitch;
  float thetaError;
  
  do
  { 
  gyro_signals(); 
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  RatePitch-=RateCalibrationPitch;
  pitch += RatePitch * elapsedTime;
  thetaError=targetAngleRight-pitch;
  
  rightMotor();

  }while(thetaError<0);

}

void leftTurn()
{
   //Moves the micomouse left side for a certain angle(90 degrees)
  float targetAngleLeft=targetAngleTurn;
  float thetaError;
  float pitch;
  //waitStop=80;
  do
  {
  gyro_signals();
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  RatePitch-=RateCalibrationPitch;
  pitch += RatePitch * elapsedTime;
  thetaError=targetAngleLeft-pitch;
  
  leftMotor();
  }while(thetaError>0);

}

void backwardTurn()
{
  //Moves the micomouse right side for a certain angle(180 degrees)
  float targetAngleBack=-targetAngle;
  float thetaError;
   float pitch;
  do
  {
  gyro_signals();
  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
  RatePitch-=RateCalibrationPitch;
  pitch += RatePitch * elapsedTime;
  thetaError=targetAngleBack-pitch;
  
  rightMotor();
  }while(thetaError<0);

}

void movement()
{
    // 6  All walls
    // 31 leftWall & rightWall
    // 30  leftWall & frontWall
    // 10 rightwall & front wall
    // 3 leftWall
    // 1 rightwall
    // 0 frontwall
    // 4  No walls
    
   
    //Determine wallType based on IR sensor input
    
    if (leftWall==0 && rightWall==0 && frontWall==0) {
        wallType = 6; // turn backwards
    } 
    
    else if (leftWall==0 && rightWall==0 && frontWall==1) {
        wallType = 31; // Go forward
    } 
    else if (leftWall==0 && frontWall==0 && rightWall==1) {
        wallType = 30; // turn right
        
    } 
    else if (rightWall==0 && frontWall==0 && leftWall==1) {
        wallType = 10;  // turn left
       
    } 
    
    else if (leftWall==0 && rightWall==1 && frontWall==1) {
        wallType = 3; // turn any one right/front
        
    } 
    else if (rightWall==0 && frontWall==1 && leftWall==1) {
        wallType = 1; // turn any one front/left
        
    } 
    else if (frontWall==0 && leftWall==1 && rightWall==1) {
        wallType = 0; // turn any one right/left
        
    } 

    else if (leftWall==1 && rightWall==1 && frontWall==1) {
        wallType= 4; // turn any one right/left/front
    }

    //Below logic is added to satisfy a game rule (Rule: Avoid micromouse to visit start cell(0,0) again). 
    //If it detects (0,1) or (1,0) cell again,new wallType will be assigned to it which helps to turn in such a way not to visit (0,0).
    
    if (x==xvar && y==yvar){
        if( wallType==31 ||wallType==10 ||wallType==30){
                wallType=6;
	     }
            else if (wallType==1){
                wallType=10;
				}
            else if (wallType==3){
                wallType=30;
				}
            else if (wallType==0){
			
                if (x==1 && y==0){
                   wallType=10;
				   }
                else if (x==0 && y==1){
                   wallType=30;
				   }
				   
			}
     }


    
    
    
    
    // 3 leftWall
    // 1 rightwall
    
    
    
    // 30  leftWall & frontWall(1 Choice:Turn Right)
     if(wallType  == 30) {  //Turn right
       
        rightTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
        Stop(); //Stops at Next cell
        delay(waitStop);
        updateDirection('R');  
      }
      
    // 10 rightwall & front wall(1 Choice:Turn Left)
    else if(wallType  == 10) {  //Turn left

        leftTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
        Stop();
        delay(waitStop);
        updateDirection('L');
        
     
       
       
    }

   //temp[x][y] is 3 //forward when rightWall
   //temp[x][y] is 6 //leftTurn when rightWall
   //temp[x][y] is 4 // rightTurn when leftWall
   //temp[x][y] is 5 //forward when leftWall
   //temp[x][y] is 1  //leftTurn when frontWall
   //temp[x][y] is 2  //rightTurn when frontWall


   // 1 rightwall (2 choice: LeftTurn or forward)
   //Normal Case: First time rightwall,move forward(3),Second time same rightwall,take leftTurn(6)
   //Here for temp[x][y]==5 is given for a case where if already in that cell mouse has gone forward with leftWall,
   //and now you are detecting the cell again as having right wall(This case is when mouse is coming back to the same cell in opposite direction).
   //As it is rightwall in the second time(2 Choice: LeftTurn or Forward)--> But choosing forward will take the mouse in already travelled path.
   //So,to avoid repetition in this case leftTurn is used.
   
   else if(wallType  == 1) {  //Right wall
      
       if (temp[x][y]==3 || temp[x][y]==5)  // 1(rightWall)
        {
        leftTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
        Stop();
        delay(waitStop);
        updateDirection('L');
		    temp[x][y]=6;
         }
		 else
		 { 
		    forward();
        Stop();
        delay(waitStop);       
		    temp[x][y]=3;
        }
		}

   // 3 leftwall (2 choice: rightTurn or forward)
   //Normal Case: First time leftwall,move forward(5),Second time same leftwall,take rightTurn(4)
   //Here for temp[x][y]==3 is given for a case where if already in that cell mouse has gone forward with rightWall,
   //and now you are detecting the cell again as having left wall(This case is when mouse is coming back to the same cell in opposite direction).
   //As it is leftwall in the second time(2 Choice: rightTurn or Forward)--> But choosing forward will take the mouse in already travelled path.
   //So,to avoid repetition in this case rightTurn is used.
   
	 else if(wallType  == 3) {  //Left wall
      
       if (temp[x][y]==5 || temp[x][y]==3)   
        {
		    rightTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
        Stop();
        delay(waitStop);
        updateDirection('R'); 
        
		    temp[x][y]=4;
         }
		 else
		 { 
		    forward();
        Stop();
        delay(waitStop);      
		    temp[x][y]=5;
        }
		}   


   // 0 frontwall (2 choice: Left or Right)
   //Normal Case: First time frontwall,take leftTurn(1),Second time same frontwall,take rightTurn(2)
   //Here for temp[x][y]==3 is given for a case where if already in that cell mouse has gone forward with rightWall,
   //and now you are detecting the cell again as having front wall(This case is when mouse is coming back to the same cell in opposite direction).
   //As it is frontwall in the second time(2 Choice: rightTurn or leftTurn)--> But choosing leftTurn will take the mouse in already travelled path.
   //So,to avoid repetition in this case rightTurn is used.
   
    else if(wallType == 0) {  //Front wall present
       if (temp[x][y]==1 || temp[x][y]==3)
       {
        rightTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
        Stop();
        delay(waitStop);
        updateDirection('R');  
		    temp[x][y]=2;     
       }
       else
       {
        leftTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
        Stop();
        delay(waitStop);
        updateDirection('L');
		    temp[x][y]=1;
       }
    }

    // 4  No walls (Multiple choice: Choose Forward Always)
    else if(wallType  == 4) // no walls present
    {      
        forward(); 
        Stop();
        delay(waitStop);            
       
    }

    // 31 leftWall & rightWall((1 Choice:forward)
    else if(wallType  == 31) { //left and right Wall
      
        forward();       
        Stop();
        delay(waitStop);
   
    }
    
    // 6  All walls(1 Choice:BackwardTurn)
    else if(wallType  == 6){
        
        backwardTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
        Stop();
        delay(waitStop);
        updateDirection('B');  //BACKWARD Direction 
        
    
      
        
    }
  

  
   
}

void updateDirection(char turn) {

  //directionValue:(Top:0,Right:1,Bottom:2,Left:3)
  
    if (turn == 'L') {
        directionValue -= 1;
        if (directionValue == -1) {
            directionValue = 3;
        }
    } 
    else if (turn == 'R') {
        directionValue += 1;
        if (directionValue == 4) {
            directionValue = 0;
        }
    } 
    else if (turn == 'B') {
        if (directionValue == 0) {
            directionValue = 2;
        } 
        else if (directionValue == 1) {
            directionValue = 3;
        } 
        else if (directionValue == 2) {
            directionValue = 0;
        } 
        else if (directionValue == 3) {
            directionValue = 1;
        }
    }
}

void updateCoordinates() {
    
   
   
    if (directionValue == 0) {
        y += 1;
    } 
    else if (directionValue == 1) {
        x += 1;
    } 
    else if (directionValue == 2) {
        y -= 1;
    } 
    else if (directionValue == 3) {
        x -= 1;
    }
    
    
    
}

void movementDir() {
  if (directionCurrent == 0) {
        if (directionSt == 0) {
            directionFinal=0;

        } else if (directionSt == 1) {
 
            directionFinal=1;

        } else if (directionSt == 3) {

            directionFinal=3;

        } else {

            directionFinal=2;
   
        }
    } else if (directionCurrent == 1) {
        if (directionSt == 0) {

            directionFinal=3;

        } else if (directionSt == 1) {
   
            directionFinal=0;

        } else if (directionSt == 3) {
            
            directionFinal=2;

        } else {
            
            directionFinal=1;
        }
    } else if (directionCurrent == 2) {
        if (directionSt == 0) {
            
            directionFinal=2;

        } else if (directionSt == 1) {
            
            directionFinal=3;

        } else if (directionSt == 3) {
          
            directionFinal=1;

        } else {
          
            directionFinal=0;

        }
    } else if (directionCurrent == 3) {
        if (directionSt == 0) {
          
            directionFinal=1;

        } else if (directionSt == 1) {
            
            directionFinal=2;
        } else if (directionSt == 3) {
          
            directionFinal=0;

        } else {
            
            directionFinal=3;

        }
    }

    
    
}
void storeFinalDirection()
{
  x=0;
  y=0;
  int i=1;
  directionValue=0;
  while((x==fx && y==fy)==false)
  {
  directionSt = directionStore[x][y];
  movementDir();
  directionCurrent = directionSt;
  directionValue=directionFinal;
  updateCoordinates();
  EEPROM.update(i,directionFinal);
  delay(5);
  i++;
  }
  EEPROM.write(0,50);
  delay(5);
  digitalWrite(13,HIGH);
}
void finalRun() {
  x=0;
  y=0;
  
  int i=1;
  int dirFast=EEPROM.read(1);
  delay(5);

  while(dirFast!=255)
  {
  
  if(dirFast==0){
        forward();
      
  }
  else if(dirFast==1){
        rightTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
  }
  else if(dirFast==2){
        backwardTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;

  }
  else if(dirFast==3){
        leftTurn();
        wait=wait+waitTurn;
        forward();
        wait=wait-waitTurn;
  }

  directionValue=dirFast;
  updateCoordinates();
  if(x==fx && y==fy)
  {
     Stop();
  }
  i++;
  dirFast = EEPROM.read(i);
  delay(5);
  }

}


void setup() {

//Serial.begin(115200);

Wire.setClock(400000);
Wire.begin();
Wire.beginTransmission(0x68); 
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission();
 
RateCalibrationPitch=3.45;
currentTime = micros();
delay(1);

pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);

pinMode(enA,OUTPUT);
pinMode(enB,OUTPUT);

pinMode(13, OUTPUT);

pinMode(FrontIRSensor, INPUT); // IR Sensor pin INPUT
pinMode(LeftIRSensor, INPUT); // IR Sensor pin INPUT
pinMode(RightIRSensor, INPUT); // IR Sensor pin INPUT

//CHANGE FLAG VALUES
flag=true;
 if(EEPROM.read(0)==50)
 {
  finalRun();
  flag=false;
 }

}

void loop() {

char key = keypad.getKey();
if (key != NO_KEY){
if(key=='2')
{
for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
    delay(5);
  }
for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 255);
    delay(5);
  }
 digitalWrite(13, HIGH);
}
}
 if(flag==true)
 {  
frontWall = digitalRead(FrontIRSensor);
leftWall = digitalRead(LeftIRSensor);
rightWall = digitalRead(RightIRSensor);

movement();  


directionStore[x][y]=directionValue;
fx=x;
fy=y;
if(var==1)
{
  xvar=x;
  yvar=y;
}
updateCoordinates();


if (flagLeftCount == 3 || flagRightCount == 3) 
  {
      if (frontWall==0 && leftWall==1 && rightWall==1) 
      {
          Stop();
          storeFinalDirection();
          flag=false;
      }
  }

  if (frontWall==0 && leftWall==0 && rightWall==1)
  {
      flagLeftCount += 1;
  } 
  else 
  {
      flagLeftCount = 0;
  }

  if (frontWall==0 && rightWall==0 && leftWall==1) 
  {
      flagRightCount += 1;
  } 
  else 
  {
      flagRightCount = 0;
  }
var+=1;
}

}
