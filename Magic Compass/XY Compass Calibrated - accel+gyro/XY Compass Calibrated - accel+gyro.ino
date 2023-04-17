#include <MadgwickAHRS.h>

//FemmeVerbeek's XY compass example from LSM9DS1 examples, but with Magpie flight controller corrections

#include <Arduino_LSM9DS1.h>
const float sensorRate = 400.00;
//  Read IMU

boolean viewInSerialPlotter=true;      // true optimises for serial plotter, false for serial monitor
Madgwick filter;

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
   IMU.setMagnetOffset(17.674561, 29.237671, -12.265015);  //  calibrated, first Nano33BLE
   IMU.setMagnetSlope (1.193672, 1.226216, 1.241961);  //  calibrated
   IMU.setAccelODR(5);
   IMU.setGyroODR(5);
/******************************************************************************************************************************     
****  FS  Full Scale        range (0=±400 | 1=±800 | 2=±1200 | 3=±1600  (µT)                                              *****     
****  ODR Output Data Rate  range (6,7,8)=(40,80,400)Hz | not available on all chips (0..5): (0.625,1.25,2.5,5.0,10,20)Hz *****
*******************************************************************************************************************************/     

   IMU.magnetUnit = MICROTESLA;  //   GAUSS   MICROTESLA   NANOTESLA
   if (viewInSerialPlotter) Serial.println(" Heading \t Inclination \t Strength \t mag.X \t mag.Y \t mag.Z ");  
   filter.begin(sensorRate);
}

void loop ()
{  float magY,magX,magZ,accX,accY,accZ,gyrX,gyrY,gyrZ;
   float gyrRollAngle,gyrPitchAngle,gyrYawAngle;
   float accRollAngle,accPitchAngle;
   doNMagMeasurements (50,magX,magY,magZ);                                  //Average measurements to reduce noise
  //  doNAccMeasurements (50,accX,accY,accZ);
  //  doNAccMeasurements (50,gyrX,gyrY,gyrZ);
   IMU.readAccel(accX, accY, accZ);   //  Accelerometer returns G Force (ms-2)
   IMU.readGyro(gyrX, gyrY, gyrZ);    //  Gyro rates are Degrees Per Second (DPS)
  
  filter.updateIMU(gyrX, gyrY, gyrZ, accX, accY, accZ);
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float pitchRadians = pitch * (PI / 180);
  float rollRadians = roll * (PI / 180);

  float magX_compensated = magX * cos(pitchRadians) + magZ * sin(pitchRadians);
  float magY_compensated = magX * sin(rollRadians) * sin(pitchRadians) + magY * cos(rollRadians) - magZ * sin(rollRadians) * cos(pitchRadians);


   float heading = atan2(magY_compensated,magX_compensated)*180/(PI)+ 180;
   float fieldStrength = sqrt(magX*magX +magY*magY+magZ*magZ);
   float inclination = atan(-magZ/sqrt(magX*magX +magY*magY)) *180/PI;    // by definition a positive inclination means the Z component is negative

   if (!viewInSerialPlotter) Serial.print("Heading ");
   dump(heading,"°  Inclination ");
   dump(inclination,"° Intensity ");
   dump(fieldStrength,"µT  X ");
   dump(magX,"µT  Y ");
   dump(magY,"µT  Z ");
   dump(magZ,"µT");
   Serial.println();
}

void dump (float Value, char txt[])
{
  Serial.print(Value);
  if (!viewInSerialPlotter) Serial.print(txt);
  else Serial.print('\t');
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

void doNGyrMeasurements(unsigned int N, float& averX, float& averY, float& averZ) 
{    float x, y, z;
     averX=0; averY =0; averZ =0;
     for (int i=1;i<=N;i++)
     { while (!IMU.gyroAvailable());
        IMU.readGyro(x, y, z);
        averX += x; averY += y;  averZ += z;
     } 
     averX /= N;    averY /= N;  averZ /= N;
}

