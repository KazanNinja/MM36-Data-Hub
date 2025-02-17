#include <ADS1115_WE.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32-TWAI-CAN.hpp>

//CAN TX-RX Pins
#define CAN_RX 33
#define CAN_TX 34

//Defines I2C addresses for ADS ADC and MPU IMU
#define ADS1115_I2C_ADDRESS 0x48
#define MPU6050_I2C_ADDRESS 0x69  //nice

//Defines I2C pins on ESP
#define I2C_SDA 13
#define I2C_SCL 12

//Defines status LED pin
#define StatusLED 35

//Defines ADC pins for 5V > 3.3V and 12V > 3.3V 
#define ADC_5V_5 3
#define ADC_5V_6 4
#define ADC_12V_1 5
#define ADC_12V_2 6

//CAN Recieving frame
CanFrame rxFrame;

//Defining Tasks
TaskHandle_t CAN_Task;
TaskHandle_t IMU_ADC_Task;

//Sets accel int16 variables for psuedo voltage readings
//Because the board is mounted 90 degrees against the firewall
//From driver pov
//X is up (+) & down (-), ground to sky
//Y is left (+) to right (-)
//Z is front (+) to back (-)
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;

//Similar to accels, described from sitting in the drivers seat
//X - yaw, turning right (+) to left (-)
//Y - pitch, nose up (+) nose down (-)
//Z - roll, roll right (-) roll left (+)
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;

//Sets up ADC floats
int16_t adc0 = 0;
int16_t adc1 = 0;
int16_t adc2 = 0;
int16_t adc3 = 0;

//Sets up voltage divider adc int16_t
//Currently, 5V voltage divider ADC are not used
uint16_t adc_5v_4 = 0;
uint16_t adc_5v_5 = 0;
uint16_t adc_12v_1 = 0;
uint16_t adc_12v_2 = 0;

//Declares ADS and MPU objects and what not
ADS1115_WE adc = ADS1115_WE(ADS1115_I2C_ADDRESS);
Adafruit_MPU6050 mpu;

void setup() {

  //Begins I2C communication on corresponding pins
  Wire.begin(I2C_SDA, I2C_SCL);

  //Begins serial comms
  Serial.begin(115200);

  //Sets up StatusLED to be an output for diagnosis stuff
  pinMode(StatusLED, OUTPUT);

  //Sets 5V > 3.3V voltage divider pins as inputs
  pinMode(ADC_5V_5, INPUT);
  pinMode(ADC_5V_6, INPUT);

  //Sets 12V > 3.3V divider pins
  pinMode(ADC_12V_1, INPUT);
  pinMode(ADC_12V_2, INPUT);

  //Initialize ADS1115 library stuff
  if (!adc.init()) {
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
  adc.setVoltageRange_mV(ADS1115_RANGE_6144);  //comment line/change parameter to change range

  //Sets sampling rate of ADC
  //Possible SPS are 8, 16, 32, 64, 128, 250, 475, 860
  adc.setConvRate(ADS1115_128_SPS);

  //Sets ADC to measure continously as opposed to single shot measurements
  adc.setMeasureMode(ADS1115_CONTINUOUS);  //comment line/change parameter to change mode

  //Setting up task for CAN bus stuffz, grabs data and stuff
  xTaskCreatePinnedToCore(
    CAN_Task_Code,  //Function to implement the task
    "CAN Task",     //Name of the task
    10000,          //Stack size in words
    NULL,           //Task input parameter
    0,              //Priority of the task
    &CAN_Task,      //Task handle
    0               //Core where the task should run
  );

  //Setting up task for IMU and ADC stuff
  xTaskCreatePinnedToCore(
    IMU_ADC_Code,    //Function to implement the task
    "IMU ADC Task",  //Name of the task
    10000,           //Stack size in words
    NULL,            //Task input parameter
    1,               //Priority of the task
    &IMU_ADC_Task,   //Task handle
    1                //Core where the task should run
  );

  //Delay for stuff
  delay(200);

  //CAN setup
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(5);
  ESP32Can.setTxQueueSize(5);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));

  // You can also just use .begin()..
  if (ESP32Can.begin()) {
    Serial.println("CAN bus started!");
    digitalWrite(StatusLED, HIGH);
    delay(75);
    digitalWrite(StatusLED, LOW);
    delay(75);
    digitalWrite(StatusLED, HIGH);
    delay(75);
    digitalWrite(StatusLED, LOW);
  } else {
    Serial.println("CAN bus failed!");
    digitalWrite(StatusLED, LOW);
  }
}

void loop() {
  //Do nuffin
}

void CAN_Task_Code(void *parameter) {
  Serial.println("Running CAN Task");

  while (true) {

    //Sets StatusLED off until CAN talk begins
    digitalWrite(StatusLED, LOW);

    //Shitasses
    static uint32_t lastStamp = 0;
    uint32_t currentStamp = millis();

    //CAN TX-ing, sends data to the bus
    if (currentStamp - lastStamp > 50) {
      digitalWrite(StatusLED, HIGH);
      lastStamp = currentStamp;

      sendIMU_ADC(0x7F0, 0x7F3, 0x7F4);
    }

    //Delay to yield to other tasks
    vTaskDelay(1);
  }
}

void IMU_ADC_Code(void *parameter) {

  while (true) {
    //Grabs MPU IMU sensor events
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Acceleration is given in m/s^2
    //Gets converted to g with division
    //Added offsets based on rough flat surface calibration that idk if is even remotely correct
    //Scaled by 1000 for mV conversion in CAN transfer
    accelX = ((a.acceleration.x - 0.27) / 9.81) * 1000;
    accelY = ((a.acceleration.y - 0.09) / 9.81) * 1000;
    accelZ = ((a.acceleration.z + 0.89) / 9.81) * 1000;

    //Gyro values give in rad/s
    //Did same rough flat surface calc thing
    //Scaled by 1000 for mV conversion in CAN transfer
    gyroX = (g.gyro.x + 0.06) * 1000;
    gyroY = (g.gyro.y + 0.03) * 1000;
    gyroZ = (g.gyro.z + 0.05) * 1000;

    //Gets millivolt reading of ADS1115 ADCs
    adc0 = readChannel(ADS1115_COMP_0_GND);
    adc1 = readChannel(ADS1115_COMP_1_GND);
    adc2 = readChannel(ADS1115_COMP_2_GND);
    adc3 = readChannel(ADS1115_COMP_3_GND);

    //Gets ADC value from 12V ADC input thingy, not finished
    adc_12v_1 = analogRead(ADC_12V_1);
    adc_12v_2 = analogRead(ADC_12V_2);

    //Bit shift demo testing thing
    // uint8_t highByte = (gyroX >> 8) & 0xFF;
    // uint8_t lowByte = gyroX & 0xFF;
    // gyroX = (highByte << 8 | lowByte);
    // Serial.println(gyroX);

    vTaskDelay(1);
  }
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_mV();  // alternative: getResult_mV for Millivolt
  return voltage;
}

void sendGPS(int frameID) {
  //GPS Stuff
}

void sendIMU_ADC(int frameID, int frameID2, int frameID3) {

  //First CAN frame that sends the accelerometer data in all 3-DOF
  //Sends X gyroscope data on last two bytes
  CanFrame IMUframe1 = { 0 };
  IMUframe1.identifier = frameID;
  IMUframe1.extd = 0;
  IMUframe1.data_length_code = 8;
  IMUframe1.data[0] = accelX & 0xFF;
  IMUframe1.data[1] = (accelX >> 8) & 0xFF;
  IMUframe1.data[2] = accelY & 0xFF;
  IMUframe1.data[3] = (accelY >> 8) & 0xFF;
  IMUframe1.data[4] = accelZ & 0xFF;
  IMUframe1.data[5] = (accelZ >> 8) & 0xFF;
  IMUframe1.data[6] = gyroX & 0xFF;
  IMUframe1.data[7] = (gyroX >> 8) & 0xFF;
  ESP32Can.writeFrame(IMUframe1);

  //Second CAN frame that sends the rest of the gyroscope data (Y and Z)
  //Sends first two ADC values from AD1115 (Not voltage divider ADC)
  CanFrame IMU_ADC_Frame1 = { 0 };
  IMU_ADC_Frame1.identifier = frameID2;
  IMU_ADC_Frame1.extd = 0;
  IMU_ADC_Frame1.data_length_code = 8;
  IMU_ADC_Frame1.data[0] = gyroY & 0xFF;
  IMU_ADC_Frame1.data[1] = (gyroY >> 8) & 0xFF;
  IMU_ADC_Frame1.data[2] = gyroZ & 0xFF;
  IMU_ADC_Frame1.data[3] = (gyroZ >> 8) & 0xFF;
  IMU_ADC_Frame1.data[4] = adc0 & 0xFF;
  IMU_ADC_Frame1.data[5] = (adc0 >> 8) & 0xFF;
  IMU_ADC_Frame1.data[6] = adc1 & 0xFF;
  IMU_ADC_Frame1.data[7] = (adc1 >> 8) & 0xFF;
  ESP32Can.writeFrame(IMU_ADC_Frame1);

  //Third CAN frame that sends rest of ADS1115 ADC data
  //Sends data from voltage divider ADCS
  CanFrame ADCFrame = { 0 };
  ADCFrame.identifier = frameID3;
  ADCFrame.extd = 0;
  ADCFrame.data_length_code = 8;
  ADCFrame.data[0] = adc2 & 0xFF;
  ADCFrame.data[1] = (adc2 >> 8) & 0xFF;
  ADCFrame.data[2] = adc3 & 0xFF;
  ADCFrame.data[3] = (adc3 >> 8) & 0xFF;
  ADCFrame.data[4] = adc_12v_1 & 0xFF;
  ADCFrame.data[5] = (adc_12v_1 >> 8) & 0xFF;
  ADCFrame.data[6] = adc_12v_2 & 0xFF;
  ADCFrame.data[7] = (adc_12v_2 >> 8) & 0xFF;
  ESP32Can.writeFrame(ADCFrame);
}
