/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
float angle, elapsedTime, timer, timePrev,error;
float previous_error= 0;
float previous_angle= 0;
float total_error= 0;
float alpha = 1;
float tau = 0.1;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float rad_to_deg = 180/3.141592654;
float pid_p=0;
float pid_i=0;
float pid_d=0;
float PID = 0;
float kp=11.5;
float kd=2;
float desired_angle = 0;
int mspeed = 10;


void forward(int vel);
void backward (int vel);
void halt ();


void setup() {
  Serial.begin(9600);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } 
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite( 3, LOW);
  digitalWrite(5, LOW);
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
}


void loop() {
  
  mpu.update();
  timer = millis();  
  elapsedTime = (timer - timePrev) / 1000; 
  alpha = tau/(tau+elapsedTime);
  angle = mpu.getAngleX();
  error = angle - desired_angle; 
  
  pid_p = kp*error;
  pid_d = (kd/elapsedTime)*(error-previous_error);
  PID = pid_p+ pid_d; //PID control compute
  mspeed = abs(PID);

   if(angle<=0){
       forward(mspeed);
      }
   else if(angle>0)
      {
       backward(mspeed);
      }
    if(angle>20)
    halt();
    if(angle<-20)
    halt();

  timePrev = timer;
  previous_angle = angle;  
  previous_error = error;
  
}


void forward (int vel){ 
  analogWrite(11, vel);
  digitalWrite(10, LOW);
  analogWrite( 5, vel);
  digitalWrite(3, LOW);
}

void backward (int vel){ 
  analogWrite(10, vel);
  digitalWrite(11, LOW);
  analogWrite( 3, vel);
  digitalWrite(5, LOW);
}
void halt (){ 
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite( 3, LOW);
  digitalWrite(5, LOW);
}
