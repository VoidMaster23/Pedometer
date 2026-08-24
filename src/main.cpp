#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "pedometer_algo.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define MPU_ADDRESS 0x68

#define SDA 21
#define SCL 20

#define LED 8

constexpr unsigned long SAMPLE_RATE_HZ = 100;
constexpr unsigned long SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ;
unsigned long last_sample_time = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
sensors_event_t accel, temp, gyro;

int16_t acc_x, acc_y, acc_z;

void setup()
{
  Serial.begin(115200);

  Wire.setPins(SDA, SCL);
  Wire.begin();

  // wake that bih up. TODO: extract this to a lib later
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B); // access the PWR_MGMT_1 register
  Wire.write(0x00); // sleep, cycle, temperature disable set to 0 and defaults to internal clock
  Wire.endTransmission(true);

  // setting the accelerometer range
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C); // ACCEL_CONFIG Reg
  Wire.write(0x10); // setting the accelerometer range to +-8g
  Wire.endTransmission(true);
  Serial.println("Yay! sensor is hereeee");

  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("YOOOO NIGGAA"));
    display.display();
  }

  PedometerAlgo::initGlobals(); // pedometer
  pinMode(LED, OUTPUT);
}

void loop()
{
  unsigned long current_time = millis();

  if (current_time - last_sample_time >= SAMPLE_INTERVAL_MS)
  {
    last_sample_time = current_time;

    display.clearDisplay();

    // get accelerometer readings from register 0x3B onwards
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true); // request 6 bytes of data. Each reading is split into two 8 bit registers - high and low when taking big endian into account

    // now we put this into our shit
    acc_x = (Wire.read() << 8) | Wire.read();
    acc_y = (Wire.read() << 8) | Wire.read();
    acc_z = (Wire.read() << 8) | Wire.read();

    Serial.print("AccX: ");
    Serial.print(acc_x);
    Serial.print(" | AccY: ");
    Serial.print(acc_y);
    Serial.print(" | AccZ: ");
    Serial.println(acc_z);

    int32_t step_count = PedometerAlgo::count_steps(acc_x, acc_y, acc_z);
    Serial.println(step_count);

    char buffer[64];

    sprintf(buffer, "step count: %d", step_count);

    // not good code will defs clean up later, trust
    display.setCursor(0, 0);
    display.print(buffer);
    display.display();
  }
}
