#include <Pixy2.h>
#include <Servo.h>
#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 
#define front_sensor A0
#define back_sensor A1
#define ServoUp 
#define ServoDown 

// This is the main Pixy object 
Pixy2 pixy;
Servo myServo;  // Create a servo object
//Global Variables
int left_turn_count = 0;
int right_turn_count = 0;
int turn_count = 0;
//Function Declarations
void go_Advance(void);
void go_Left(int t=0);
void go_Right(int t=0);
void go_Back(int t=0);
void align_camera(int x, int y);
void stop_Stop();
void set_Motorspeed(int speed_L,int speed_R);
void init_GPIO();
int readDistance(int sensor);
void lineTrack(Pixy2 pixy);
void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pinMode(3,OUTPUT);
  myServo.attach(3);  // Attach the servo to pin 3
  
  pixy.init();
  init_GPIO();
  pixy.changeProg("CCC");
}

void loop()
{ 
 //lineTrack(pixy);
 Serial.print("Front: ");
 Serial.println(readDistance(A0));
 Serial.print("Back: ");
 Serial.println(readDistance(A1));
 delay(1000);


 //myServo.write(15);
 //delay(7000); // Wait for 1 second
 //myServo.write(180); // Adjust the angle as needed
 //delay(7000); // Wait for 1 seco
 

  
}


void align_camera(int x, int y)
{
  int max_x = 315;
  int max_y = 207;

  int ideal_x = max_x/2;
  int ideal_y = max_y/2;

  if((x >= (ideal_x*0.7) && x <= (ideal_x*1.4)) || (y >= (ideal_y*0.9) && y <= (ideal_y*1.1)))
  {
    go_Advance();
    return;
  }
  else
  {
    if(x < (ideal_x*0.7))
    {
      go_Left(2);
    }
    else if(x >= (ideal_x*1.4))
    {
      go_Right(2);
    }
  }
}

/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,120 );
  analogWrite(speedPinR,120);
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,120);
  analogWrite(speedPinR,120);
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,120);
  analogWrite(speedPinR,120);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,120);
  analogWrite(speedPinR,120);
  delay(t);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

//Pins initialize
void init_GPIO()
{
	pinMode(RightMotorDirPin1, OUTPUT); 
	pinMode(RightMotorDirPin2, OUTPUT); 
	pinMode(speedPinL, OUTPUT);  
 
	pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
	stop_Stop();
}

int readDistance(int sensor)
{
  float volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
  int distance = 13*pow(volts, -1); // worked out from datasheet graph
  return distance;

}

void lineTrack(Pixy2 pixy)
{
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    // Serial.print("Detected ");
    // Serial.println(pixy.ccc.numBlocks);
    //Serial.print("Center X: ");
    //Serial.println(pixy.ccc.blocks[0].m_x);
    //Serial.print("Width: ");
    //Serial.println(pixy.ccc.blocks[0].m_width);
    if (pixy.ccc.blocks[0].m_width < 200) 
    {
      //Serial.println("Straight: ");
      align_camera(pixy.ccc.blocks[0].m_x, pixy.ccc.blocks[0].m_y);
    }
    else if(pixy.ccc.blocks[0].m_width > 200)
    {
      if(pixy.ccc.blocks[0].m_x > 150)
      {
        //Serial.println("Turn Right: ");
        go_Right(900);
        go_Advance();
        delay(700); 
        right_turn_count++;
      }
      else if(pixy.ccc.blocks[0].m_x < 150)
      {
        //Serial.println("Turn Left: ");
        go_Left(900);
        go_Advance();
        delay(700); 
        left_turn_count++;

      }
    }
  }
  else 
  {
    //Serial.println("Stop");
    stop_Stop();
    if(right_turn_count > 4 || left_turn_count > 4)
    {
      go_Back(2000);
    }
  }

}
