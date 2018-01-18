#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050_9Axis_MotionApps41.h>

#include <Wire.h>
#include <DHT.h> // library for AM2302 sensor

#define DHTPIN 7 //humidity_temperature connetced pin: digital 7
#define DHTTYPE DHT22 // AM2302
int sound=A0; // sound sensor connected pin: analog A0
int light=A2; // light sensor connected pin: analog A2
int magnet=10; // magnetic sensor digital connected pin: digital 10
int pir=2; // pir sensor connected pin: digital 2
const int MPU=0x68; // I2C standard address of MPU6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

DHT dht(DHTPIN, DHTTYPE);
// initialize DHT sensor for normal 16mhz Arduino

void setup()
{
  Wire.begin(); // initialize Wire library  
  Wire.beginTransmission(MPU); // Data transmission to MPU
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // Start mode of MPU6050
  Wire.endTransmission(true);
  dht.begin();
  Serial.begin(9600);
}

void loop() 
{
  float hum=dht.readHumidity(); // read humidity from sensor
  float temp=dht.readTemperature(); // read temperature from sensor
  
  float magnetic=digitalRead(magnet);

  Wire.beginTransmission(MPU); 
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) 
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU,14,true); // request a total of 14 registers 
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L) 
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L) 
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L) 
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L) 
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
 
  if(isnan(hum)||isnan(temp))
  {  // if any fails to read, end function
    Serial.println("Failed to read from AM2302 sensor");
    return;
  }
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print("\t");
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print("C");
  Serial.print("\t");
  
  Serial.print("Sound: ");
  Serial.print(analogRead(sound));
  Serial.print("\t");
  
  Serial.print("Light: ");
  Serial.print(analogRead(light));
  Serial.print("\t");
  
  Serial.print("Magnetic: ");
  Serial.print(magnetic);
  Serial.print("\t");

  Serial.print("PIR: ");
  Serial.println(digitalRead(pir));
  
  Serial.print("AcX = ");
  Serial.print(AcX); 
  Serial.print(" | AcY = ");
  Serial.print(AcY); 
  Serial.print(" | AcZ = ");
  Serial.print(AcZ); 
  Serial.print(" | Tmp = ");
  Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet 
  Serial.print(" | GyX = ");
  Serial.print(GyX); 
  Serial.print(" | GyY = ");
  Serial.print(GyY); 
  Serial.print(" | GyZ = ");
  Serial.println(GyZ); */
  delay(333); 
  
}

