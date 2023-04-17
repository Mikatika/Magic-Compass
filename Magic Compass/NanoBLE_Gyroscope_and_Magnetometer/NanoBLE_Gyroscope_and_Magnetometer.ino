#include <Arduino_LSM9DS1.h>
float magX, magY, magZ, ledvalue;
float gyX, gyY, gyZ;

void setup() {
  // put your setup code here, to run once:
  IMU.begin();
  Serial.begin(9600);
}

void loop() {
  // really only x important for compass application (?)
  if (IMU.magneticFieldAvailable()){
    IMU.readMagneticField(magX, magY, magZ);
    Serial.print("Magnetic Field x = ");
    Serial.print(magX);
    Serial.print(", y = ");
    Serial.print(magY);
    Serial.print(", z = ");
    Serial.print(magZ);

    double headRad = atan2(magY, magX);
    Serial.print(", Heading = ");
    Serial.println(headRad);
  }
//  else{
//    Serial.println("Magnetometer Failed!");    
//  }
  if (IMU.gyroscopeAvailable()){
    IMU.readGyroscope(gyX, gyY, gyZ);
    Serial.print("Gyroscope values: x = ");
    Serial.print(gyX);
    Serial.print(", y = ");
    Serial.print(gyY);
    Serial.print(", z = ");
    Serial.println(gyZ);
  }
// 
delay(250);
}
