#include <ADS1115_WE.h> 
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32-TWAI-CAN.hpp>

//CAN TX-RX Pins
#define CAN_RX 33
#define CAN_TX 34

//Defins I2C addresses for ADS ADC and MPU IMU
#define ADS1115_I2C_ADDRESS 0x48
#define MPU6050_I2C_ADDRESS 0x69 //nice

//Defines I2C pins on ESP
#define I2C_SDA 13
#define I2C_SCL 12

//Sets accel and gyro float variables
//Because the board is mounted 90 degrees against the firewall
//From driver pov
//X is up (+) & down (-), ground to sky
//Y is left (+) to right (-)
//Z is front (+) to back (-)
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

//Similar to accels, described from sitting in the drivers seat
//X - yaw, turning right (+) to left (-)
//Y - pitch, nose up (+) nose down (-)
//Z - roll, roll right (-) roll left (+)
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

//Sets up ADC floats
float adc0 = 0.0;
float adc1 = 0.0;
float adc2 = 0.0;
float adc3 = 0.0;

//Declares ADS and MPU objects and what not
ADS1115_WE adc = ADS1115_WE(ADS1115_I2C_ADDRESS);
Adafruit_MPU6050 mpu;

void setup() {

  //Begins I2C communication on corresponding pins
  Wire.begin(I2C_SDA, I2C_SCL);

  //Begins serial comms
  Serial.begin(115200);

  //Initialize ADS1115 library stuff
  if(!adc.init()){
    Serial.println("ADS1115 not connected!");
  }

  //Initialize MPU6050 on given address and stuff
  //Address is 0x69 because the MPU6050 ADDR pin is pulled high
  if (!mpu.begin(MPU6050_I2C_ADDRESS, &Wire, 0)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //Set IMU accelerometer measuring range to +/- 4G
  //Car will generally see around 1.5-1.8g in lateral accel (I think) with the occasional 2-2.4g or so
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  
  //Set IMU gyro to have +/- 250deg/s range
  //Typical yaw rates of the vehicle do not exceed 100 deg/s
  //With the pitch and roll rates being sub 10 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  //TBD
  //Sets low pass filter, available freqs are 260, 184, 94, 44, 21, 10, 5
  //In hertz
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Sets ADC voltage range of ADS1115
  //Measuring 5V sensors so max 6144 range is selected
  //Choices are 6144, 4096, 2048, 1024, 0512, 0256 mV
  adc.setVoltageRange_mV(ADS1115_RANGE_6144); //comment line/change parameter to change range

  //Sets sampling rate of ADC
  //Possible SPS are 8, 16, 32, 64, 128, 250, 475, 860
  adc.setConvRate(ADS1115_128_SPS);

  //Sets ADC to measure continously as opposed to single shot measurements
  adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode
}

void loop() {
  //Grabs MPU IMU sensor events
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Acceleration is given in m/s^2
  //Gets converted to g with division
  //Added offsets based on rough flat surface calibration that idk if is even remotely correct
  accelX = (a.acceleration.x - 0.27 ) / 9.81;
  accelY = (a.acceleration.y - 0.09) / 9.81;
  accelZ = (a.acceleration.z + 0.89) / 9.81;

  //Gyro values give in rad/s
  //Did same rough flat surface calc thing
  gyroX = (g.gyro.x + 0.06);
  gyroY = (g.gyro.y + 0.03);
  gyroZ = (g.gyro.z + 0.05);

  //Gets millivolt reading of ADS1115 ADCs
  adc0 = readChannel(ADS1115_COMP_0_GND);
  adc1 = readChannel(ADS1115_COMP_1_GND);
  adc2 = readChannel(ADS1115_COMP_2_GND);
  adc3 = readChannel(ADS1115_COMP_3_GND);

}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}
