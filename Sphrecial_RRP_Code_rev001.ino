#include <Servo.h>

  Servo Baseservo; // Base (2)PWN
  Servo Shoulderservo; // Shoulder (3) PWN
  Servo Armservo; // Arm (4) PWN
  Servo Gripperservo; // gripper (5) PWN

// Inverse Kinematic Equations for RRP S. Robot

// Lenghts (ax)
float a1 = 4.2876; // This is Z height
float a2 = 1.1022; // This is the distance between Q1 and Q2
float a3 = 4.1483; // Distance between Q2 and End Effector

// Distance
//float d = 1.210;
float X = 3;
float Y = 3;
float Z = 3; 

float T1;
float T2;
float d3;
float T3;
float ConvertD= 1.20;

// Starting values
/*
float T1 = 0;
float r1 = 0;
float r2 = 0;
float d3 = 0;
float T2 = 0;
*/
void setup() 
{
  
Serial.begin(9600);
  Baseservo.attach(2,553,2450);
  Shoulderservo.attach(3,500,2500);
  Armservo.attach(4,553,2450);
  Gripperservo.attach(5,2400,2400);

  HomePosition();

  getInverseKinematics(X,Y,Z,a1,a2,a3);
  
  T3=ConvertToRotational(d3);
  Serial.print("T3=");
  Serial.println(T3);
}

void loop() 
{
  
  //Baseservo.write(T1);
  //Shoulderservo.write(T2);
  //Armservo.write(T3);
  GoTOPosition();
}

void getInverseKinematics(float X03,float Y03, float Z03, float L1, float L2, float L3)
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
    // put your main code here, to run repeatedly:
// Equation 1
  T1 = atan2(Y03,X03)*180.0/PI;
  Serial.print("T1=");
  Serial.println(T1);

 // Equation 2
 float r1 = Z03-L1;
 Serial.print("r1=");
 Serial.println(r1);
 
 // Equation 3
 float r2 = sqrt(pow(X03,2)+pow(Y03,2));
 Serial.print("r2=");
 Serial.println(r2);

 // Equation 4
 // Note: ABS
 float d31 = (pow(r1,2));
 float d32 = (pow(r2,2));
  d3 = sqrt(abs(d31-d32))-L2-L3;
  Serial.print("d3=");
 Serial.println(d3);

 // Equation 5 
 T2 = atan2(r2,r1)*180.0/PI;
 Serial.print("T2=");
 Serial.println(T2);

}


void openGripper()
{
  Gripperservo.write(180);
  delay(1000);
}
void closeGripper()
{
  Gripperservo.write(0);
  delay(1000);
}

int ConvertToRotational(float distance)

{
  float inc = ConvertD/90.0;
 return((int)(distance/inc));
 
}
void HomePosition()
{
  Baseservo.write(0);
  Shoulderservo.write(90);
  Armservo.write(180);
  }

void GoTOPosition()
{
  Baseservo.write(T1);
  Shoulderservo.write(180-T2);
  Armservo.write(180-T3);
}
