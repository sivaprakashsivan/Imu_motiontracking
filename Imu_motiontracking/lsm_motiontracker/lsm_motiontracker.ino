#include <Arduino_LSM9DS1.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <math.h>

float thetaM ;
float phiM ;
float psi ;
float thetaOld = 0;
float phiOld = 0 ;

float thetaFnew ;
float phiFnew ;

float thetaG = 1 ;
float phiG = 1; 
float dt ;


float gx , gy ,gz ;





// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);


void setupSensor()
{
  // 1.) Set the accelerometer range
  // lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G, lsm.LSM9DS1_ACCELDATARATE_10HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G, lsm.LSM9DS1_ACCELDATARATE_119HZ);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G, lsm.LSM9DS1_ACCELDATARATE_476HZ);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G, lsm.LSM9DS1_ACCELDATARATE_952HZ);
  
  // 2.) Set the magnetometer sensitivity
  // lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  // lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() 
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
 

  // helper to just set the default scaling we want, see above!
  setupSensor();
 
}




unsigned long preTime = 0;

void loop() 
{
  lsm.read();  /* ask it to read in the data */ 

  unsigned long currenTime = millis();
  

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  IMU.readGyroscope(gx,gy,gz) ;
  
//printning the measured values of pitch and roll by using accelerometer values

 thetaM = atan2(a.acceleration.y / 9.8 , a.acceleration.z / 9.8 ) / 2 / 3.14 * 360 ;
 phiM =   atan2(a.acceleration.x / 9.8 , a.acceleration.z / 9.8 ) / 2 / 3.14 * 360 ;

//using the LOW pass filter to avoid the errors during vibration 

 thetaFnew = 0.75 * thetaOld + thetaM * 0.25 ;
 phiFnew = 0.75* phiOld + phiM * 0.25;

 thetaOld = thetaFnew ; //re_assiginging the old theta value to new one for the next iteration .
 phiOld = phiFnew ;

// getting the values of pitch and roll by using gyro values 
    
    float dt = (currenTime - preTime)/100;
   
  
    


thetaG = thetaG + gy * dt ; 
phiG = phiG + gx * dt ;












//  Serial.print(a.acceleration.x);
//  Serial.print(",");
//  Serial.print(a.acceleration.y);
//  Serial.print(",");    
//  Serial.print(a.acceleration.z); 
//  Serial.print(",");    
//  Serial.print (thetaM) ;
//  Serial.print(",");   
//  Serial.print(phiM);
//  Serial.print(",");
//  Serial.print(thetaFnew);
//  Serial.print(",");
//  Serial.print(phiFnew);
//  Serial.print(",");
//  Serial.print(thetaG);
//  Serial.print(",");
//  Serial.println(phiG);

 Serial.print(gx);
 Serial.print(",");
 Serial.println(gy);





 preTime = currenTime;








  // Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
  // Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
  // Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

  
  
  delay(1000);
}
