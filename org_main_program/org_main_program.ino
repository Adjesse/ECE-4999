#include <Pixy2.h>
#include <Servo.h>
#include "movement.hpp"
#include <ezButton.h>
#define front_sensor A0
#define back_sensor A1
#define ServoUpPin 3
#define ServoDownPin 4
#define ServoDownLimit 2
#define ServoUpLimit 22


// This is the main Pixy object 
Pixy2 pixy;
Servo ServoUp;  // Create a servo object
Servo ServoDown;  // Create a servo object
ezButton limitSwitch_Cdown(ServoDownLimit);
ezButton limitSwitch_Bdown(ServoUpLimit);
//Global Variables
int left_turn_count = 0;
int right_turn_count = 0;
int turn_count = 0;
int phase1 = 1;
int phase2 = 0;
int phase3 = 0;
int phase4 = 0;
unsigned long startTime;
unsigned long duration_of_phase2;
unsigned long duration_of_phase1;
//Function Declarations
int readDistance(int sensor);
void lineTrack(Pixy2 pixy);
void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pinMode(ServoUpPin,OUTPUT);
  pinMode(ServoDownPin,OUTPUT);
  //pinMode(ServoDownLimit,INPUT);
  ServoUp.attach(ServoUpPin);  // Attach the servo
  ServoDown.attach(ServoDownPin);  // Attach the servo
  limitSwitch_Cdown.setDebounceTime(50);
  limitSwitch_Bdown.setDebounceTime(50);
  
  //pixy.init();
  init_GPIO();
  //pixy.changeProg("CCC");
  //ServoDown.write(180);
  //delay(1000);
  //ServoDown.write(90);
}

void loop()
{ 
 //lineTrack(pixy);
while(phase1)
{
  // limitSwitch.loop();
  // Serial.println(limitSwitch.isPressed());
  // delay(1000);
  // Serial.print("Front: ");
  // Serial.println(readDistance(front_sensor));
  // Serial.print("Back: ");
  // Serial.println(readDistance(back_sensor));
  // delay(1000);

  
  if(readDistance(front_sensor) > 20)
  {

    int descent = 1;
    startTime = millis();
    while(descent)
    { 
      limitSwitch_Cdown.loop();
      ServoDown.write(15);
      Serial.println(limitSwitch_Cdown.isPressed());

      int state = limitSwitch_Cdown.getState();
      if(state == LOW)
      {
        ServoDown.write(90);
        duration_of_phase1 = millis() - startTime;
        descent = 0;
        phase1 = 0;
        phase2 = 1;
        startTime = millis();
      } 
  }
  }
}
while(phase2)
{
  if(readDistance(back_sensor) > 20)
  {
    //stop_Stop();
    phase2 = 0;
    phase3 = 1;
    duration_of_phase2 = millis() - startTime;

  }
  else
  {
    //go_Advance();
  }
}
while(phase3)
{
  limitSwitch_Bdown.loop();
  ServoUp.write(15);
  Servo.ServoDown(180);
  Serial.println(limitSwitch_Bdown.isPressed());

  int state = limitSwitch_Bdown.getState();
  if(state == LOW)
  {
    ServoUp.write(90);
    ServoDown.write(90);
    phase3 = 0;
  } 
}

//Move Body A down
while(phase4)
{
  go_Advance();               //Move Forward for same time as phase 2
  delay(duration_of_phase2);
  stop_Stop();                //Stop to lower
  ServoUp.write(180);         //Lower for same time as phase 1 (body C)
  delay(duration_of_phase1);
  ServoUp.write(90);
  phase4 = 0;

  
}





}  
  
  

// }
 //myServo.write(15);
 //delay(7000); // Wait for 1 second
 //myServo.write(180); // Adjust the angle as needed
 //delay(7000); // Wait for 1 seco
 

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
