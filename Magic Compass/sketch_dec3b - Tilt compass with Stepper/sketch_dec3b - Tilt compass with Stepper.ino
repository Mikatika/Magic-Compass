//FemmeVerbeek's XY compass example from LSM9DS1 examples, but with Sparkfun Tilt compass variations
#include <Stepper.h>
#include <Arduino_LSM9DS1.h>

#define STEPS 400

//const float sensorRate = 400.00; //Madgwick stuff
boolean viewInSerialPlotter=true; 
float p[] = {-1, -1, 0};  //Y marking on sensor board points toward yaw = 0
int head = 0;
// local magnetic declination in degrees
float declination = -63.133;

Stepper stepper(STEPS, 4, 5, 6, 7);
//int stepDeg = 0; //Where the stepper motor is pointing in degrees (0 = N) (maybe need later? rn redundant with direction) probably important when switching directions
int lastHead = 0; //the last heading
int direction = 0; //Direction to point in
float stepAngle = 1.8; 
double curCoord[] = {37.8655782, -122.2521208}; //home
double desCoord[] = {38.8655782, -122.2521208}; //Straight North

//double desCoord[] = {37.8720616, -122.2578123}; //Berkeley campanile
//double desCoord[] = {37.840830, -122.251650}; //Chocolate Dragon Bittersweet Cafe (south ish)

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
  //  IMU.setMagnetOffset(36.484375, 26.000366, -21.262817); //Last accurate with on/off stepper callibration
  //  IMU.setMagnetSlope (1.195592, 1.231558, 1.173107);
  //  IMU.setMagnetOffset(33.415527, 30.151978, -22.955933);
  //  IMU.setMagnetSlope (1.310207, 1.259418, 1.190821);
  IMU.setMagnetOffset(38.242798, -0.313110, -11.228638); //in box
   IMU.setMagnetSlope (1.150282, 1.334535, 1.144575);



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

   stepper.setSpeed(20);
   float IAxyz[3], IMxyz[3]; //centered and scaled accel/mag data
   if (IMU.accelAvailable()) doNAccMeasurements(50, IAxyz[0], IAxyz[1], IAxyz[2]);   //  Accelerometer returns G Force (ms-2)
   if (IMU.magnetAvailable()) doNMagMeasurements(50, IMxyz[0],IMxyz[1],IMxyz[2]);
   vector_normalize(IAxyz);   
   vector_normalize(IMxyz);

   IAxyz[0] = -IAxyz[0]; //fix accel handedness

  //Make stepper arrow point forward and figure out what that heading is
   lastHead = get_heading(IAxyz, IMxyz, p);

   //Now find difference between current point forward and destination, and move stepper to point to destination
   get_direction(curCoord,desCoord);

   //figure out how to incorporate into move_stepper function later
  int shift = direction - lastHead; 
  if (shift > 180) shift = shift - 360;
  if (shift < -180) shift = 360 + shift;
  int numSteps = shift / stepAngle;
  stepper.step(numSteps);

}

void loop() {
  float Axyz[3], Mxyz[3]; //centered and scaled accel/mag data
  if (IMU.accelAvailable()) doNAccMeasurements(100, Axyz[0], Axyz[1], Axyz[2]);   //  Accelerometer returns G Force (ms-2)
  if (IMU.magnetAvailable()) doNMagMeasurements(100, Mxyz[0],Mxyz[1],Mxyz[2]);
  vector_normalize(Axyz);   
  vector_normalize(Mxyz);

  Axyz[0] = -Axyz[0]; //fix accel handedness

  head = get_heading(Axyz, Mxyz, p);
  Serial.print("Heading: ");
  Serial.print(head);

  move_stepper();

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

  // Serial.print("W: ");
  // Serial.print(W);
  // Serial.print("N: ");
  // Serial.print(N);
  // compute heading in horizontal plane, correct for local magnetic declination
  
  float headRad = atan2(vector_dot(W, p), vector_dot(N, p));
  int heading = round(headRad * 180 / M_PI + declination);
  heading = -heading; //conventional nav, heading increases North to East
  heading = (heading + 720) % 360; //apply compass wrap
  return heading;
}

void get_direction(double curr[2], double dest[2]){
  double latDif = dest[0] - curr[0];
  double lonDif = dest[1] - curr[1];
  direction = round(atan2(lonDif, latDif)*180/M_PI);
  }

void move_stepper(){
  //Difference between heading and where the stepper is pointed
  int shift = -head + lastHead; //lastHead is last position of header
  Serial.print(", Shift: ");
  Serial.print(shift);
  if (shift > 180) shift = shift - 360;
  if (shift < -180) shift = 360 + shift;
  Serial.print(", shift2: ");
  Serial.println(shift);
  int numSteps = shift / stepAngle;
  if (numSteps >= 2 || numSteps <= -2){
    lastHead = head;
    stepper.step(numSteps);
  }
}


void doNMagMeasurements(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     digitalWrite(4, LOW);
     digitalWrite(5, LOW);
     digitalWrite(6, LOW);
     digitalWrite(7, LOW);

     averX=0; averY =0; averZ =0;
     for (int i=1;i<=N;i++)
     { while (!IMU.magnetAvailable());
        IMU.readMagnet(x, y, z);
        averX += x; averY += y;  averZ += z;
     } 
     averX /= N;    averY /= N;  averZ /= N;

     digitalWrite(4, HIGH);
     digitalWrite(5, HIGH);
     digitalWrite(6, HIGH);
     digitalWrite(7, HIGH);
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
