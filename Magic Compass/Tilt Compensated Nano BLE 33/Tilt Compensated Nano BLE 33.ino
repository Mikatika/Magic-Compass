//FemmeVerbeek's XY compass example from LSM9DS1 examples, but with Sparkfun Tilt compass variations

#include <Arduino_LSM9DS1.h>
//const float sensorRate = 400.00; //Madgwick stuff
boolean viewInSerialPlotter=true; 
float p[] = {-1, -1, 0};  //Y marking on sensor board points toward yaw = 0
int head = 0;
// local magnetic declination in degrees
float declination = -13.14;

void setup() {
   Serial.begin(115200);
     while(!Serial);                   // wait till the serial monitor connects
     delay(1);
     if (!IMU.begin()) {               // initialize the magnetometer
        Serial.println("Failed to initialize IMU!");  
        while (1); }
        
/*****************   For a proper functioning of the compass the magnetometer needs to be calibrated    ********************
*****************   Replace the lines below by the output of the DIY_Calibration_Magnetometer sketch   ********************/
   IMU.setMagnetFS(0);  
   IMU.setMagnetODR(8); 
   IMU.setMagnetOffset(25.717773, 16.282349, -10.903320);
   IMU.setMagnetSlope (1.483619, 1.412389, 1.557052);
   IMU.setGyroODR(5);
   IMU.setAccelFS(3);
   IMU.setAccelODR(5);
   IMU.setAccelOffset(-0.004536, -0.009128, -0.023915);
   IMU.setAccelSlope (0.998573, 1.000814, 1.000433);
/******************************************************************************************************************************     
****  FS  Full Scale        range (0=±400 | 1=±800 | 2=±1200 | 3=±1600  (µT)                                              *****     
****  ODR Output Data Rate  range (6,7,8)=(40,80,400)Hz | not available on all chips (0..5): (0.625,1.25,2.5,5.0,10,20)Hz *****
*******************************************************************************************************************************/     

   IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA
   if (viewInSerialPlotter) Serial.println(" Heading \t Inclination \t Strength \t mag.X \t mag.Y \t mag.Z ");  
   //filter.begin(sensorRate); //Madgwick Stuff

}

void loop() {
  float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data
  if (IMU.accelAvailable()) doNAccMeasurements(50, Axyz[0], Axyz[1], Axyz[2]);   //  Accelerometer returns G Force (ms-2)
  if (IMU.magnetAvailable()) doNMagMeasurements(50, Mxyz[0],Mxyz[1],Mxyz[2]);
  vector_normalize(Axyz);   
  vector_normalize(Mxyz);

  Axyz[0] = -Axyz[0]; //fix accel handedness

  head = get_heading(Axyz, Mxyz, p);
  Serial.print("Heading: ");
  Serial.println(head);
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(float acc[3], float mag[3], float p[3])
{
  float W[3], N[3]; //derived direction vectors

  // cross "Up" (acceleration vector, g) with magnetic vector (magnetic north + inclination) with  to produce "West"
  vector_cross(acc, mag, W);
  vector_normalize(W);

  // cross "West" with "Up" to produce "North" (parallel to the ground)
  vector_cross(W, acc, N);
  vector_normalize(N);

  // compute heading in horizontal plane, correct for local magnetic declination
  
  int heading = round(atan2(vector_dot(W, p), vector_dot(N, p)) * 180 / M_PI + declination);
  heading = -heading; //conventional nav, heading increases North to East
  heading = (heading + 720)%360; //apply compass wrap
  return heading;
}


void doNMagMeasurements(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0; averZ =0;
     for (int i=1;i<=N;i++)
     { while (!IMU.magnetAvailable());
        IMU.readMagnet(x, y, z);
        averX += x; averY += y;  averZ += z;
     } 
     averX /= N;    averY /= N;  averZ /= N;
}
 
void doNAccMeasurements(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0; averZ =0;
     for (int i=1;i<=N;i++)
     { while (!IMU.accelAvailable());
        IMU.readAccel(x, y, z);
        averX += x; averY += y;  averZ += z;
     } 
     averX /= N;    averY /= N;  averZ /= N;
}

void vector_cross(float a[3], float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

void vector_normalize(float a[3])
{
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

float vector_dot(float a[3], float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
