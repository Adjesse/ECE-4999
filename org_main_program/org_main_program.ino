#include <Pixy2.h>
#include <Servo.h>
#include "movement.hpp"
#include <ezButton.h>
#define front_sensor A0
#define back_sensor A1
#define ServoUpPin 3  
#define ServoDownPin 4
#define Limit_C_Up 49
#define Limit_C_Down 51
#define Limit_B_Up 53


// This is the main Pixy object 
Pixy2 pixy;
Servo ServoUp;  // Create a servo object
Servo ServoDown;  // Create a servo object
ezButton limitSwitch_C_Up(Limit_C_Up);
ezButton limitSwitch_B_Up(Limit_B_Up);
ezButton limitSwitch_C_Down(Limit_C_Down);
//Global Variables
int left_turn_count = 0;
int right_turn_count = 0;
int turn_count = 0;
int phase1 = 0;
int phase2 = 0;
int phase3 = 0;
int phase4 = 0;
int phase5 = 0;
int phase6 = 0;
int phase7 = 0;
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
  limitSwitch_C_Up.setDebounceTime(50);
  limitSwitch_B_Up.setDebounceTime(50);
  
  pixy.init();
  init_GPIO();
  pixy.changeProg("CCC");
  //ServoDown.write(180);
  //delay(1000);
  //ServoDown.write(90);
  // ServoDown.write(15);                            //ServoUp ascend rack
  // delay(5000);
  // ServoDown.write(90);

}

void loop()
{

  
  // Serial.print("Front: ");
  // Serial.println(readDistance(front_sensor));
  // Serial.print("Back: ");
  // Serial.println(readDistance(back_sensor));
  // delay(1000);
 //lineTrack(pixy);
while(phase1)
{
  Serial.println("Phase 1");
  Serial.print("Front: ");
  Serial.println(readDistance(front_sensor));
  Serial.print("Back: ");
  Serial.println(readDistance(back_sensor));
  delay(1000);

  
  if(readDistance(front_sensor) > 20)
  {

    int descent = 1;
    startTime = millis();
    while(descent)
    { 
      limitSwitch_C_Up.loop();                        //Must call first
      ServoDown.write(15);                             //Descend Body C 
      //Serial.println(limitSwitch_Cdown.isPressed());

      int state = limitSwitch_C_Up.getState();   
      if(state == LOW)                              //If button is pressed or ServoDown is at top of rack
      {
        ServoDown.write(90);                        //Stop ServoDown Motor
        duration_of_phase1 = millis() - startTime;  //Get Elapsed Time
        descent = 0;                                //End descent loop 
        phase1 = 0;                                 //End Phase 1 
        phase2 = 1;                                 //Start Phase 2
        startTime = millis();                       //Start Timer for Phase 2
      } 
  }
  }
}
while(phase2)
{
  Serial.println("Phase 2");
  if(readDistance(back_sensor) > 20)              //If Body B is off table
  {
    stop_Stop();                                //Stop Motors
    phase2 = 0;                                   //End Phase 2
    phase3 = 1;                                   //Start Phase 3
    duration_of_phase2 = millis() - startTime;    //Get Elapsed Time for Phase 2

  }
  else
  {
    go_Advance();
  }
}
//Move Body B down
while(phase3)
{
  Serial.println("Phase 3");
  limitSwitch_B_Up.loop();                     //Must call first
  ServoUp.write(15);                            //ServoUp ascend rack
  ServoDown.write(180);                         //ServoDown descend rack
  //Serial.println(limitSwitch_Bdown.isPressed());

  int state = limitSwitch_B_Up.getState();
  if(state == LOW)                              //If button is pressed or ServoUp is at top of rack
  {
    ServoUp.write(90);                          //Stop ServoUp Motor
    ServoDown.write(90);                        //Stop ServoDown Motor
    phase3 = 0;                                 //End Phase 3
    phase4 = 1;
  } 
}

//Move Body A down
while(phase4)
{
  Serial.println("Phase 4");
  go_Advance();               //Move Forward for same time as phase 2
  delay(duration_of_phase2);
  stop_Stop();                //Stop to lower
  ServoUp.write(180);         //Lower for same time as phase 1 (body C)
  delay(duration_of_phase1);
  ServoUp.write(90);          //Stop Servo
  phase4 = 0;

  
}

while(phase5)
{
  


}

while(phase6)
{
  Serial.println("Phase 6");
  limitSwitch_B_Up.loop();                     //Must call first
  ServoUp.write(0);                            //ServoUp ascend rack
  
  int state = limitSwitch_B_Up.getState();
  if(state == LOW)                              //If button is pressed or ServoUp is at top of rack
  {
    ServoUp.write(90);                          //Stop ServoUp Motor
    phase6 = 0;                                 //End Phase 6
    phase7 = 1;
  }
  
}
while(phase7)
{
  //Move Body B down
  delay(5000);
  Serial.println("Phase 7");
  limitSwitch_B_Up.loop();                        //Must call first
  ServoUp.write(180);                             //ServoUp descend rack
  ServoDown.write(0);                             //ServoDown ascend rack

  int state = limitSwitch_C_Up.getState();
  if(state == LOW)                              //If button is pressed or ServoUp is at top of rack
  {
    ServoUp.write(90);                          //Stop ServoUp Motor
    ServoDown.write(90);                        //Stop ServoDown Motor
    phase7 = 0;                                 //End Phase 7
  } 



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
    Serial.print("Detected ");
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
        go_Right(5000);
        go_Advance();
        delay(1000); 
        right_turn_count++;
      }
      else if(pixy.ccc.blocks[0].m_x < 150)
      {
        //Serial.println("Turn Left: ");
        go_Left(5000);
        go_Advance();
        delay(1000);    
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
