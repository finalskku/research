#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include <DHT.h> // library for AM2302 sensor

Adafruit_AMG88xx amg;

#define DHTPIN 7 //humidity_temperature connetced to pin 7
#define DHTTYPE DHT22 // AM2302
#define S0 8 //color sensor S0 to pin 8
#define S1 9 //color sensor S1 to pin 9
#define S2 12//color sensor S2 to pin 12
#define S3 11//color sensor S3 to pin 11
#define color_out 10 // color sensor OUT to pin 10

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
int r, g, b; // color sensor variables for Red, Green, Blue
int sound=A0; // sound sensor connected pin: analog A0
int light=A2; // light sensor connected pin: analog A2
int magnet=4; // magnetic sensor digital connected pin: digital 4
int pir=13; // pir sensor connected pin: digital 13
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;


DHT dht(DHTPIN, DHTTYPE);
// initialize DHT sensor for normal 16mhz Arduino

void setup()
{
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(color_out, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  //set fequency scaling to 20%

  Wire.begin(); // initialize Wire library  
  dht.begin();
  Serial.begin(9600);
  bool status = amg.begin();
  setupMPU();
}

void setupMPU()
 {
  Wire.beginTransmission(0b1101000); // I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet)
  Wire.write(0x6B); // accessing the register 6B - Power Management
  Wire.write(0b00000000); // setting SLEEP register to 0.
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x1B); // accessing the register 1B - Gyroscope configuration
  Wire.write(0x00000000); // setting the gyro to full scale +/- 250deg
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); // I2C address of the MPU
  Wire.write(0x1C); // accessing the register 1C - Acccelerometer Configuration
  Wire.write(0b00000000); // setting the accel to +/- 2g
  Wire.endTransmission();
 }

int normalization(int color) // normalization of color values by averaging collected data
{
  int data[10]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  color=0;
  for(int i=0; i<10; i++)
  {
    data[i]=255-pulseIn(color_out, LOW);
    color+=data[i];
  }
  color/=10;
  return color;
}

 void recordAccelRegisters() // Accel recording for MPU
 {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); // starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
 }
 
 void processAccelData()
 {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
 }
 
 void recordGyroRegisters() // Gyro recording for MPU
 {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
 }
 
 void processGyroData() 
 {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
 }
 

void loop() 
{
  float hum=dht.readHumidity(); // read humidity from sensor
  float temp=dht.readTemperature(); // read temperature from sensor

  recordAccelRegisters();
  recordGyroRegisters();

  // setting Red filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  r=normalization(r);
  // setting Green filtered photodiodes to be read
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  g=normalization(g);
  // setting Blue filtered photodiodes to be read
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  b=normalization(b);

  amg.readPixels(pixels);
 
  Serial.print(hum);
  Serial.print(", ");
  Serial.print(temp);
  Serial.print(", ");
  
  Serial.print(analogRead(sound));
  Serial.print(", ");
  
  Serial.print(analogRead(light));
  Serial.print(", ");
  
  Serial.print(digitalRead(magnet));
  Serial.print(", ");

  Serial.print(digitalRead(pir));
  Serial.print(", ");

  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.print(b);
  Serial.print(", ");
    
  Serial.print(rotX); // gyro
  Serial.print(", ");
  Serial.print(rotY);
  Serial.print(", ");
  Serial.print(rotZ);
  Serial.print(", ");
  
  Serial.print(gForceX); // acc
  Serial.print(", ");
  Serial.print(gForceY);
  Serial.print(", ");
  Serial.print(gForceZ);
  Serial.print(", ");
 
  for(int i=0; i<AMG88xx_PIXEL_ARRAY_SIZE; i++)
  {
      Serial.print(pixels[i]);
      Serial.print(", ");
  }
  Serial.println("");
  delay(400); 
  
}

