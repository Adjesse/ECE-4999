#include <Pixy2.h>
#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 

// This is the main Pixy object 
Pixy2 pixy;
int turn_count = 0;
void go_Advance(void);
void go_Left(int t=0);
void go_Right(int t=0);
void go_Back(int t=0);
void stop_Stop();
void set_Motorspeed(int speed_L,int speed_R);
void init_GPIO();
void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  
  pixy.init();
  init_GPIO();
}

void loop()
{ 
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      determine_movement(pixy.ccc.blocks[i].m_width, pixy.ccc.blocks[i].m_height);
      // Serial.print("  block ");
      // Serial.print(i);
      // Serial.print(": ");
      // pixy.ccc.blocks[i].print();
      // Serial.println();
      // Serial.print("Width: ");
      // Serial.println(pixy.ccc.blocks[i].m_width);

      // Serial.print("Angle: ");
      // Serial.println(pixy.ccc.blocks[i].m_angle);
      

    }
  } 

  delay(1000); 
}

void determine_movement(int width, int height)
{
  if (width < 175) 
  {
    go_Advance();
    delay(height);

  }
  else if(width > 175)
  {
    if (turn_count == 0 || turn_count == 5 )
    {
      go_Right(2000);
    }
    else
    {
      go_Left(2000);
    }

    turn_count++;
    
  }


}

/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,255 );
  analogWrite(speedPinR,255);
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,255);
  analogWrite(speedPinR,255);
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,255);
  analogWrite(speedPinR,255);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,255);
  analogWrite(speedPinR,255);
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

