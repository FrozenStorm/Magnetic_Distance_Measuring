/* -------------------- Include --------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_HMC5883_U.h>

/* -------------------- Defines --------------------------------------------------------------------------------*/
#define PIN_ACCELEROMETER_SCL 17
#define PIN_ACCELEROMETER_SDA 18

#define PIN_POWER_EN          15
#define PIN_LCD_BL            38

#define DISP_WIDTH            170
#define DISP_HEIGHT           320

#define LOOP_DELAY_MS         20

/* -------------------- Variable -------------------------------------------------------------------------------*/
TFT_eSPI                          tft = TFT_eSPI(); 
Adafruit_MPU6050                  mpu;
Adafruit_BMP085                   bmp;
Adafruit_HMC5883_Unified          mag;
uint32_t                          targetTime = 0;         
/* -------------------- Functions Prototypes -------------------------------------------------------------------*/

/* -------------------- Setup ----------------------------------------------------------------------------------*/
void setup() {
  sleep(3000);
  Serial.begin(115200);
  Serial.println("Init started");

  // Power Enable (for LCD Backlight without VBUS from USB)
  pinMode(PIN_POWER_EN,OUTPUT);
  digitalWrite(PIN_POWER_EN,HIGH);
  //ledcSetup(0, 10000, 8);
  //ledcAttachPin(PIN_LCD_BL, 0);
  //ledcWrite(0, 10); // 0..255 Backlight intensity

  // Display
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 2);
  tft.setTextColor(TFT_WHITE,TFT_BLACK,true);  
  tft.setTextSize(1);
  Serial.println("Display ready");

  // MPU6050 Senosr Board
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  bmp.begin();
  Serial.println("Sensor ready");

  mag = Adafruit_HMC5883_Unified(12345);
  
  // Loop Delay
  targetTime = millis() + LOOP_DELAY_MS; 
  Serial.println("Init done");
}

/* -------------------- Main -----------------------------------------------------------------------------------*/
void loop() {
  uint32_t startTime = millis();
  char myString[20];
  static uint32_t deltaTime = 0;
  static uint32_t maxHold = 0;
  
  if (targetTime < millis()) {    
    Serial.println(".");
    targetTime += LOOP_DELAY_MS;
  
    tft.setTextDatum(TL_DATUM);
    tft.setCursor(5, 5, 2);

    printf("%3d,%3d/%3d", deltaTime, maxHold, LOOP_DELAY_MS-deltaTime);


    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp); // 3ms

    printf("%3d,%3d,%3d", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    printf("%3d,%3d,%3d", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    printf("%3d,%4d,%4d", temp.temperature, bmp.readPressure(), bmp.readAltitude(1013.25));

    sensors_event_t magnetic;
    mag.getEvent(&magnetic);
    printf("%3d,%3d,%3d", magnetic.magnetic.x, magnetic.magnetic.y, magnetic.magnetic.z);

    deltaTime = millis()-startTime;
    if(deltaTime > maxHold) maxHold = deltaTime;
  }
}