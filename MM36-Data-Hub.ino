#include <ADS1115_WE.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <freertos/queue.h>
#include <math.h>  // For sin()
#include <ESP32-TWAI-CAN.hpp>
#include "esp_wifi.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//CAN TX-RX Pins
#define CAN_RX 1
#define CAN_TX 2

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

//GPS Power/GND pins
#define GPS_3V3 9
#define GPS_GND 8

//CAN Recieving frame
CanFrame rxFrame;
CanFrame gpsFrame0;
CanFrame gpsFrame1;
CanFrame gpsFrame2;
CanFrame gpsFrame3;

//Defining Tasks
TaskHandle_t CAN_Task;
TaskHandle_t IMU_ADC_Task;
TaskHandle_t ADC_Task;
TaskHandle_t GPS_Task;

SemaphoreHandle_t i2cMutex;

//Sets accel int16 variables for psuedo voltage readings
//Because the board is mounted 90 degrees against the firewall
//From driver pov
//Declares average dongers
//X is up (+) & down (-), ground to sky
//Y is left (+) to right (-)
//Z is front (+) to back (-)
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;
int16_t AxAVG = 0;
int16_t AyAVG = 0;
int16_t AzAVG = 0;

//Similar to accels, described from sitting in the drivers seat
//Declares average dongers
//X - yaw, turning right (+) to left (-)
//Y - pitch, nose up (+) nose down (-)
//Z - roll, roll right (-) roll left (+)
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
int16_t GxAVG = 0;
int16_t GyAVG = 0;
int16_t GzAVG = 0;

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

//ESP-NOW Live Telemetry Vars
int v_speed = 0;
int e_speed = 0;
int throttle = 0;
int brake = 0;
int gear = 0;
int lapTime = 0;
float temp = 0;

// MAC address of the receiver (replace with the actual MAC address of your receiver)
uint8_t receiverMAC[] = { 0xA0, 0xB7, 0x65, 0x27, 0x16, 0x08 };

QueueHandle_t messageQueue;
char currentMessage[128];  // Increased buffer size to accommodate timestamp

//Declares ADS and MPU objects and what not
ADS1115_WE adc = ADS1115_WE(ADS1115_I2C_ADDRESS);
Adafruit_MPU6050 mpu;

//Local timer for GPS querey
long lastTime = 0;
int32_t latitude;
int32_t longitude;
uint32_t UTC;
uint32_t heading;
uint32_t date;
uint16_t groundSpeed;
uint16_t millisecondz;
uint16_t year;
uint16_t HDOP;
int16_t altitude;
uint8_t month;
uint8_t day;
uint8_t hour;
uint8_t minute;
uint8_t second;
uint8_t SIV;
int8_t validPosition;
uint8_t fixType;
bool getFixOK;

void setup() {

  //Begins I2C communication on corresponding pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  //Begins serial comms
  Serial.begin(115200);

  //GPS M9N Begin
  if (myGNSS.begin() == false)  //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
  } else {
    Serial.println(F("u-blox GNSS detected."));
    myGNSS.setI2COutput(COM_TYPE_UBX);                  //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR
    myGNSS.setNavigationFrequency(25);
    myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
    myGNSS.setAutoPVT(true);
  }

  //Delay for stuff
  delay(200);

  // Set device in STA mode
  WiFi.mode(WIFI_STA);
  Serial.println("WiFi initialized in STA mode");

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }
  Serial.println("ESP-NOW initialized successfully");

  // Register the send callback
  esp_now_register_send_cb(onSent);

  // Configure peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);  // Set receiver MAC address
  peerInfo.channel = 0;                        // Use the same Wi-Fi channel
  peerInfo.encrypt = false;                    // Disable encryption

  // Check if the peer exists, add it if not
  if (esp_now_is_peer_exist(receiverMAC)) {
    Serial.println("Peer already exists");
  } else if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else {
    Serial.println("Peer added successfully");
  }

  esp_wifi_set_max_tx_power(84);  // Set max power to 21 dBm

  // Create the queue to hold messages
  messageQueue = xQueueCreate(500, sizeof(char) * 128);  // Increased buffer size for longer messages

  //Sets up StatusLED to be an output for diagnosis stuff
  pinMode(StatusLED, OUTPUT);

  //Sets 5V > 3.3V voltage divider pins as inputs
  pinMode(ADC_5V_5, INPUT);
  pinMode(ADC_5V_6, INPUT);

  //Sets 12V > 3.3V divider pins
  pinMode(ADC_12V_1, INPUT);
  pinMode(ADC_12V_2, INPUT);

  //Delay for stuff
  delay(200);

  //Initialize ADS1115 library stuff
  if (adc.init()) {
    Serial.println("ADS1115 connected!");
  } else {
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
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  //Serial.println(mpu.getFsyncSampleOutput());

  //Sets ADC voltage range of ADS1115
  //Measuring 5V sensors so max 6144 range is selected
  //Choices are 6144, 4096, 2048, 1024, 0512, 0256 mV
  adc.setVoltageRange_mV(ADS1115_RANGE_6144);  //comment line/change parameter to change range

  //Sets sampling rate of ADC
  //Possible SPS are 8, 16, 32, 64, 128, 250, 475, 860
  adc.setConvRate(ADS1115_860_SPS);

  //Sets ADC to measure continously as opposed to single shot measurements
  adc.setMeasureMode(ADS1115_CONTINUOUS);  //comment line/change parameter to change mode

  //CAN setup
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(64);
  ESP32Can.setTxQueueSize(64);
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

  i2cMutex = xSemaphoreCreateMutex();

  //Setting up task for CAN bus stuffz, grabs data and stuff
  xTaskCreatePinnedToCore(
    CAN_Task_Code,  //Function to implement the task
    "CAN Task",     //Name of the task
    4096,           //Stack size in words
    NULL,           //Task input parameter
    20,             //Priority of the task
    &CAN_Task,      //,      //Task handle
    1               //Core where the task should run
  );

  // // //Setting up task for IMU and ADC stuff
  xTaskCreatePinnedToCore(
    IMU__ADC_Code,  //Function to implement the task
    "IMU Task",     //Name of the task
    8192,           //Stack size in words
    NULL,           //Task input parameter
    15,             //Priority of the task
    &IMU_ADC_Task,  //Task handle
    1               //Core where the task should run
  );

  // //Setting up task for IMU and ADC stuff
  // xTaskCreate(
  //   ADC_Code,    //Function to implement the task
  //   "ADC Task",  //Name of the task
  //   8192,        //Stack size in words
  //   NULL,        //Task input parameter
  //   10,          //Priority of the task
  //   &ADC_Task    //Task handle
  //   //1          //Core where the task should run
  // );

  // //Setting up task for IMU and ADC stuff
  xTaskCreatePinnedToCore(
    GPS_CODE,    //Function to implement the task
    "GPS Task",  //Name of the task
    8192,        //Stack size in words
    NULL,        //Task input parameter
    16,          //Priority of the task
    &GPS_Task,   //Task handle
    1            //Core where the task should run
  );

  xTaskCreatePinnedToCore(addMessageToQueueTask, "addMessageToQueueTask", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(sendMessageFromQueueTask, "sendMessageFromQueueTask", 4096, NULL, 4, NULL, 0);

  //Delay for stuff
  delay(200);
}

// Define a buffer to hold task info
char taskListBuffer[2048];  // Buffer to hold task information (increase if needed)

void printTaskList() {
  // Print the task list (this can be printed to serial or a display)
  vTaskList(taskListBuffer);
  Serial.println(taskListBuffer);  // Or any other method to print
}

void loop() {
  // printTaskList();
  // delay(1000);
  vTaskDelete(NULL);
}

void CAN_Task_Code(void *parameter) {

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20;

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  while (true) {

    //Sets StatusLED off until CAN talk begins
    //digitalWrite(StatusLED, LOW);

    //Shitasses
    static uint32_t lastStamp = 0;
    uint32_t currentStamp = millis();
    uint32_t beginning = micros();

    //CAN TX-ing, sends data to the bus
    //digitalWrite(StatusLED, HIGH);
    lastStamp = currentStamp;

    sendIMU_ADC(0x7F0, 0x7F3, 0x7F4);  //Runs function to set IMU data into CAN frame and write frames
    //Serial.print("Sending CAN at ");
    //Serial.println(micros());
    sendGPS();  //Runs function to set GPS data into CAN frame and write frames
    //digitalWrite(StatusLED, LOW);

    //throttle=60;
    //Checks if there are any frames to read
    if (ESP32Can.readFrame(rxFrame, 0)) {

      //Vehicle Speed CAN Frame
      //v_speed = 0;
      if (rxFrame.identifier == 0x648) {
        byte vSpeedLow = rxFrame.data[2];
        byte vSpeedHigh = rxFrame.data[3];
        v_speed = (vSpeedLow << 8) + vSpeedHigh;
      }

      //Engine Speed CAN Frame
      if (rxFrame.identifier == 0x640) {
        byte rpmLow = rxFrame.data[0];
        byte rpmHigh = rxFrame.data[1];
        e_speed = (rpmLow << 8) + rpmHigh;
      }

      //Throttle CAN Frame
      if (rxFrame.identifier == 0x640) {
        byte tpsLow = rxFrame.data[6];
        byte tpsHigh = rxFrame.data[7];
        throttle = (tpsLow << 8) + tpsHigh;
        throttle = throttle / 10;
        //Serial.println(throttle);
      }

      //Brake CAN Frame
      if (rxFrame.identifier == 0x655) {
        byte brakeLow = rxFrame.data[0];
        byte brakeHigh = rxFrame.data[1];
        brake = (brakeLow << 8) + brakeHigh;
        brake = brake / 10;
        //Serial.println(brake);
      }

      //Gear position CAN Frame
      if (rxFrame.identifier == 0x64D) {
        gear = rxFrame.data[6] & 0b00001111;
      }

      //Coolant CAN Frame
      if (rxFrame.identifier == 0x649) {
        temp = rxFrame.data[0] - 40;
        temp = temp * 1.8 + 32;
      }

      //Lap Time
      if (rxFrame.identifier == 0x65B) {
        byte lapLow = rxFrame.data[0];
        byte lapHigh = rxFrame.data[1];
        lapTime = (lapLow << 8) + lapHigh;
        //Serial.println(lapTime);
      }
    }

    uint32_t ending = micros();
    uint32_t timeTaken = ending - beginning;
    //Serial.println(timeTaken);
    //Delay to yield to other tasks
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void IMU__ADC_Code(void *parameter) {

  /*
  Exponential Moving average at 50Hz sampling rate
  */

  static TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20;
  float alpha = 0.25;

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  while (true) {

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5))) {
      uint64_t beginning = millis();

      //digitalWrite(StatusLED, HIGH);
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

      // //Gyro values give in rad/s
      // //Did same rough flat surface calc thing
      // //Scaled by 1000 for mV conversion in CAN transfer
      gyroX = (g.gyro.x + 0.06) * 100;
      gyroY = (g.gyro.y + 0.03) * 100;
      gyroZ = (g.gyro.z + 0.05) * 100;

      //Exponential moving average
      AxAVG = alpha * accelX + (1 - alpha) * AxAVG;
      AyAVG = alpha * accelY + (1 - alpha) * AyAVG;
      AzAVG = alpha * accelZ + (1 - alpha) * AzAVG;
      GxAVG = alpha * gyroX + (1 - alpha) * GxAVG;
      GyAVG = alpha * gyroY + (1 - alpha) * GyAVG;
      GzAVG = alpha * gyroZ + (1 - alpha) * GzAVG;

      adc0 = readChannel(ADS1115_COMP_0_GND);
      // adc1 = readChannel(ADS1115_COMP_1_GND);
      // adc2 = readChannel(ADS1115_COMP_2_GND);
      // adc3 = readChannel(ADS1115_COMP_3_GND);

      uint64_t ending = millis();
      uint64_t timeTaken = ending - beginning;
      //Serial.println(timeTaken);
      //Serial.print("IMU: ");
      //Serial.println(millis());
      //Serial.println(AxAVG);
      //Serial.println(configTICK_RATE_HZ);
      xSemaphoreGive(i2cMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void GPS_CODE(void *parameter) {
  static TickType_t xLastWakeTime;
  const TickType_t xFrequency = 40;

  xLastWakeTime = xTaskGetTickCount();

  while (true) {

    long beginning = millis();

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(0))) {
      if (myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false)) {

        lastTime = millis();  //Update the timer

        //The goods
        latitude = myGNSS.getLatitude();
        longitude = myGNSS.getLongitude();
        altitude = myGNSS.getAltitude();
        SIV = myGNSS.getSIV();
        year = myGNSS.getYear();
        month = myGNSS.getMonth();
        day = myGNSS.getDay();
        hour = myGNSS.getHour();
        minute = myGNSS.getMinute();
        second = myGNSS.getSecond();
        millisecondz = myGNSS.getMillisecond();
        heading = myGNSS.getHeading();
        HDOP = myGNSS.getPDOP();
        fixType = myGNSS.getFixType();
        getFixOK = myGNSS.getGnssFixOk();

        //Conversion of variables to decimal place of Motec requirements
        altitude = altitude / 10;
        heading = heading / 10000;
        double groundSpeedDouble = (double)myGNSS.getGroundSpeed();
        groundSpeed = (groundSpeedDouble / 1000000 * 3600) * 10;

        // //Checks fix type, GPS/GNSS, dead reckoning, differential station, etc
        if (fixType == 3) {
          fixType = 1;
        } else if (fixType == 4) {
          fixType = 6;
        } else if (fixType == 0) {
          fixType = 0;
        } else if (myGNSS.getDiffSoln()) {
          fixType = 2;
        } else {
          fixType = -1;
        }

        // //Checks if GPS fix is valid
        if (getFixOK) {
          validPosition = 1;
        } else if (!getFixOK) {
          validPosition = 0;
        } else {
          validPosition = -1;
        }

        // //Converts ints to strings for joining
        String yearS = (String)year;
        String monthS = (String)month;
        String dayS = (String)day;

        // //Adding of leading zero to day or month
        // //Substring of the year to get last two digits
        // //Joins strings together then converts to int
        if (month < 10) monthS = "0" + monthS;
        if (day < 10) dayS = "0" + dayS;
        yearS = yearS.substring(2, 4);
        String dateResult = dayS + monthS + yearS;
        date = dateResult.toInt();

        // //UTC string concatination and shit
        // //Converts time vars to strings
        // //Concatenates strings together, adds leading zeros if min, sec, or milli is single digit
        // //Joins strings together then converts result into an int for UTC time
        String hourS = String(hour);
        String minuteS = String(minute);
        String secondS = String(second);
        String millisecondzS = String(millisecondz);
        if (minute < 10) minuteS = "0" + minuteS;
        if (second < 10) secondS = "0" + secondS;
        if (millisecondz < 100) millisecondzS = "00" + millisecondzS;
        else if (millisecondz < 10) millisecondzS = "0" + millisecondzS;
        String result = hourS + minuteS + secondS + millisecondzS;

        // UTC = (uint32_t)result.toInt();
        long ending = millis();
        long timeTaken = ending - beginning;
        //Serial.println(timeTaken);
        //Serial.println(millis());
        //Serial.println(latitude);
      }
      xSemaphoreGive(i2cMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void sendIMU_ADC(int frameID, int frameID2, int frameID3) {

  //First CAN frame that sends the accelerometer data in all 3-DOF
  //Sends X gyroscope data on last two bytes
  CanFrame IMUframe1 = { 0 };
  IMUframe1.identifier = frameID;
  IMUframe1.extd = 0;
  IMUframe1.data_length_code = 8;
  IMUframe1.data[0] = (AxAVG >> 8) & 0xFF;
  IMUframe1.data[1] = AxAVG & 0xFF;
  IMUframe1.data[2] = (AyAVG >> 8) & 0xFF;
  IMUframe1.data[3] = AyAVG & 0xFF;
  IMUframe1.data[4] = (AzAVG >> 8) & 0xFF;
  IMUframe1.data[5] = AzAVG & 0xFF;
  IMUframe1.data[6] = (GxAVG >> 8) & 0xFF;
  IMUframe1.data[7] = GxAVG & 0xFF;

  //Second CAN frame that sends the rest of the gyroscope data (Y and Z)
  //Sends first two ADC values from AD1115 (Not voltage divider ADC)
  CanFrame IMU_ADC_Frame1 = { 0 };
  IMU_ADC_Frame1.identifier = frameID2;
  IMU_ADC_Frame1.extd = 0;
  IMU_ADC_Frame1.data_length_code = 8;
  IMU_ADC_Frame1.data[0] = (GyAVG >> 8) & 0xFF;
  IMU_ADC_Frame1.data[1] = GyAVG & 0xFF;
  IMU_ADC_Frame1.data[2] = (GzAVG >> 8) & 0xFF;
  IMU_ADC_Frame1.data[3] = GzAVG & 0xFF;
  IMU_ADC_Frame1.data[4] = (adc0 >> 8) & 0xFF;
  IMU_ADC_Frame1.data[5] = adc0 & 0xFF;
  IMU_ADC_Frame1.data[6] = (adc1 >> 8) & 0xFF;
  IMU_ADC_Frame1.data[7] = adc1 & 0xFF;


  //Third CAN frame that sends rest of ADS1115 ADC data
  //Sends data from voltage divider ADCS
  CanFrame ADCFrame = { 0 };
  ADCFrame.identifier = frameID3;
  ADCFrame.extd = 0;
  ADCFrame.data_length_code = 8;
  ADCFrame.data[0] = (adc2 >> 8) & 0xFF;
  ADCFrame.data[1] = adc2 & 0xFF;
  ADCFrame.data[2] = (adc3 >> 8) & 0xFF;
  ADCFrame.data[3] = adc3 & 0xFF;
  ADCFrame.data[4] = (adc_12v_1 >> 8) & 0xFF;
  ADCFrame.data[5] = adc_12v_1 & 0xFF;
  ADCFrame.data[6] = (adc_12v_2 >> 8) & 0xFF;
  ADCFrame.data[7] = adc_12v_2 & 0xFF;

  ESP32Can.writeFrame(IMUframe1);
  ESP32Can.writeFrame(IMU_ADC_Frame1);
  ESP32Can.writeFrame(ADCFrame);
}

void sendGPS() {

  //Main CAN frame, containts latitiude and longitude
  //Does 4 byte 32 bit shift thing across for the 4 byte long messages that Motec requires
  gpsFrame0 = { 0 };
  gpsFrame0.identifier = 0x680;
  gpsFrame0.extd = 0;
  gpsFrame0.data_length_code = 8;
  gpsFrame0.data[0] = (latitude >> 24) & 0xFF;
  gpsFrame0.data[1] = (latitude >> 16) & 0xFF;
  gpsFrame0.data[2] = (latitude >> 8) & 0xFF;
  gpsFrame0.data[3] = (latitude >> 0) & 0xFF;
  gpsFrame0.data[4] = (longitude >> 24) & 0xFF;
  gpsFrame0.data[5] = (longitude >> 16) & 0xFF;
  gpsFrame0.data[6] = (longitude >> 8) & 0xFF;
  gpsFrame0.data[7] = (longitude >> 0) & 0xFF;

  //Time Frame, sent over 4 bytes in UTC time
  //Ground speed and altitude across 2 bytes
  gpsFrame1 = { 0 };
  gpsFrame1.identifier = 0x681;
  gpsFrame1.extd = 0;
  gpsFrame1.data_length_code = 8;
  gpsFrame1.data[0] = (UTC >> 24) & 0xFF;
  gpsFrame1.data[1] = (UTC >> 16) & 0xFF;
  gpsFrame1.data[2] = (UTC >> 8) & 0xFF;
  gpsFrame1.data[3] = (UTC >> 0) & 0xFF;
  gpsFrame1.data[4] = (groundSpeed >> 8) & 0xFF;
  gpsFrame1.data[5] = (groundSpeed >> 0) & 0xFF;
  gpsFrame1.data[6] = (altitude >> 8) & 0xFF;
  gpsFrame1.data[7] = (altitude >> 0) & 0xFF;

  //Date sent over 3 bytes
  //Valid position value sent over 1 byte
  //Heading over 2 bytes
  //HDOP and SIV sent over 1 byte
  gpsFrame2 = { 0 };
  gpsFrame2.identifier = 0x682;
  gpsFrame2.extd = 0;
  gpsFrame2.data_length_code = 8;
  gpsFrame2.data[0] = (date >> 16) & 0xFF;
  gpsFrame2.data[1] = (date >> 8) & 0xFF;
  gpsFrame2.data[2] = (date >> 0) & 0xFF;
  gpsFrame2.data[3] = validPosition;
  gpsFrame2.data[4] = (heading >> 8) & 0xFF;
  gpsFrame2.data[5] = (heading >> 0) & 0xFF;
  gpsFrame2.data[6] = HDOP;
  gpsFrame2.data[7] = SIV;

  //Valid position only gets sent on this frame
  //AFAIK we don't have the functions to get the other items
  //That Motec wants to see
  gpsFrame3 = { 0 };
  gpsFrame3.identifier = 0x683;
  gpsFrame3.extd = 0;
  gpsFrame3.data_length_code = 8;
  gpsFrame3.data[0] = 0;
  gpsFrame3.data[1] = 0;
  gpsFrame3.data[2] = 0;
  gpsFrame3.data[3] = 0;
  gpsFrame3.data[4] = validPosition;
  gpsFrame3.data[5] = validPosition;
  gpsFrame3.data[6] = 0;
  gpsFrame3.data[7] = 0;

  //Writes/sends frames to CAN bus
  ESP32Can.writeFrame(gpsFrame0);
  ESP32Can.writeFrame(gpsFrame1);
  ESP32Can.writeFrame(gpsFrame2);
  ESP32Can.writeFrame(gpsFrame3);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_mV();  // alternative: getResult_mV for Millivolt
  return voltage;
}

void addMessageToQueueTask(void *pvParameters) {
  char localMessage[128];

  while (true) {

    // Get the current timestamp in milliseconds
    unsigned long time = millis();

    // Format the message with the timestamp, temperature, and humidity
    snprintf(localMessage, sizeof(localMessage), "time:%.3f|v_speed:%d|e_speed:%d|throttle:%d|brake:%d|gear:%d|temp:%.1f|lat:%.6f|long:%.6f|lp:%d", (double)time / 1000, v_speed, e_speed, throttle, brake, gear, temp, ((double)latitude) / 10000000, ((double)longitude) / 10000000, lapTime);

    // Add the message to the queue
    if (xQueueSend(messageQueue, localMessage, portMAX_DELAY) != pdPASS) {
      Serial.println("Failed to add message to queue");
    }

    // Delay for 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void sendMessageFromQueueTask(void *pvParameters) {
  char finalMessage[128];
  while (true) {
    if (xQueueReceive(messageQueue, currentMessage, 0) == pdPASS) {
      // Get the current queue size
      UBaseType_t queueSize = uxQueueMessagesWaiting(messageQueue);

      // Format the final message with queue size included
      snprintf(finalMessage, sizeof(finalMessage), "%s|queue_capacity:%.1f", currentMessage, (double)queueSize / 5);

      // Send the message
      if (esp_now_send(receiverMAC, (const uint8_t *)finalMessage, strlen(finalMessage) + 1) != ESP_OK) {
        // If sending fails, requeue the message in the `onSent` callback
        xQueueSendToFront(messageQueue, currentMessage, 0);
      } else {
        //Serial.print("Sent message: ");
        //Serial.println(finalMessage);
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay for 50ms
  }
}

void onSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  //Serial.print("Send Status: ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    //Serial.println("Success");
  } else {
    //Serial.println("Fail");
    // Requeue the failed message to the front of the queue
    if (xQueueSendToFront(messageQueue, currentMessage, 0) != pdPASS) {
      //Serial.println("Failed to requeue message");
    }
  }
}
