/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
#include <QMC5883LCompass.h>

MPU6050 mpu(Wire);
QMC5883LCompass compass;
unsigned long timer = 0;

int x_value;
int y_value;
int z_value;
int azimuth;  // 0° - 359°
byte bearing; // 0 - 15 (N, NNE, NE, ENE, E, ...)
char direction[strlen("NNE") + 1];
char buffer[strlen("X=-99999 | Y=-99999") + 1]; 

int firstAngle, currentAngle;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  compass.init();
  compass.setCalibration(-837, 1296, -1968, 121, -1193, 890);
  compass.read();
  firstAngle = compass.getAzimuth();

}

void loop() {
  mpu.update();
  compass.read();
  azimuth   = compass.getAzimuth();
  currentAngle = azimuth;
  int tiltedAngle = currentAngle - firstAngle;
  if((millis()-timer)>10){ // print data every 10ms
 Serial.print("X : ");
 Serial.print(mpu.getAngleZ());
 Serial.print("\tZ : ");
 Serial.println(tiltedAngle);
 timer = millis();  
  }
}