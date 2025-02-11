#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
#include "AS5600.h"
#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AD9833.h>

#ifndef ESP32
#error ESP32 only example, please select appropriate board
#endif

//  HSPI uses default   SCLK=14, MISO=12, MOSI=13, SELECT=15
//  VSPI uses default   SCLK=18, MISO=19, MOSI=23, SELECT=5
SPIClass *myspi = new SPIClass(VSPI);
AD9833 AD(5, myspi);

#define LED_PIN 13
#define NUM_LEDS_1 60
#define NUM_LEDS_2 48
#define NUM_LEDS_3 40
#define TOTAL_LEDS (NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3)
#define BRIGHTNESS 12
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[TOTAL_LEDS];

#define PWM_OUT 25
#define PWM_OUT2 26
#define PWM_OUT3 27
#define ANALOG_PIN 34

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

AS5600 as5600;

float currentAngle = 0;
float previousAngle = 0;
float angularVelocity = 0;

const float movementThreshold = 0.005;
int analogValue;
const int pwmValues[4] = {0, 8, 32, 128}; // Adjusted PWM values for ESP32

const uint8_t hues[4] = {16, 192, 48, 96};
const int frontSize = 1;
const int maxTailLength = 8;

#line 52 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void setup();
#line 89 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void loop();
#line 137 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void updateLEDs();
#line 145 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void updateRing(int startIndex, int numLeds);
#line 189 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void updateDisplay();
#line 52 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void setup()
{
  Serial.begin(115200);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("PRRRRRRA!!!"));
  display.display();
  delay(600);

  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, TOTAL_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  pinMode(PWM_OUT, OUTPUT);
  pinMode(PWM_OUT2, OUTPUT);
  pinMode(PWM_OUT3, OUTPUT);

  myspi->begin();

  AD.begin();
  AD.setFrequency(200, 0);

  AD.setWave(AD9833_SQUARE1);
  Serial.println(AD.getWave());
}

void loop()
{
  static uint8_t displayUpdateCounter = 0;
  static uint32_t lastTime = 0;
  uint32_t currentTime = micros();
  float deltaTime = (currentTime - lastTime) * 1e-6;
  lastTime = currentTime;
  float rawAngle = as5600.rawAngle();
  float targetAngleRad = (rawAngle / 4096.0) * TWO_PI;
  float angleDiff = targetAngleRad - currentAngle;

  if (abs(angleDiff) > PI)
  {
    angleDiff += (angleDiff > 0) ? -TWO_PI : TWO_PI;
  }

  if (abs(angleDiff) > movementThreshold)
  {
    previousAngle = currentAngle;
    currentAngle += angleDiff * 1; // closer to 1 is more direct response
    if (currentAngle < 0)
      currentAngle += TWO_PI;
    if (currentAngle >= TWO_PI)
      currentAngle -= TWO_PI;
    angularVelocity = angleDiff / deltaTime;
  }
  else
  {
    angularVelocity *= 0.2; // closer to 0 more faster decay, closer to 1 slower decay
  }

  int pwmValue = map(constrain(abs(angularVelocity), 0, 96), 0, 100, 0, 255);
  dacWrite(PWM_OUT, pwmValue);
  int segment = (int)(currentAngle / (TWO_PI / 4)); // Determine the segment (0-3)
  dacWrite(PWM_OUT2, pwmValues[segment]);

  analogValue = analogRead(ANALOG_PIN);
  int pwmValue2 = map(analogValue, 0, 4095, 0, 64);
  analogWrite(PWM_OUT3, pwmValue2);

  float mappedFreq = map(constrain(abs(angularVelocity), 0, 20), 0, 20, 0, 600);
  AD.setFrequency(mappedFreq, 0);

  updateLEDs();
  FastLED.show();
  updateDisplay();
}

void updateLEDs()
{
  FastLED.clear();
  updateRing(0, NUM_LEDS_1);
  updateRing(NUM_LEDS_1, NUM_LEDS_2);
  updateRing(NUM_LEDS_1 + NUM_LEDS_2, NUM_LEDS_3);
}

void updateRing(int startIndex, int numLeds)
{
  int currentLed = (int)((currentAngle / TWO_PI) * numLeds) % numLeds;
  int segment = (int)(currentAngle / (TWO_PI / 4)); // Determine the segment (0-3)

  // Set front LEDs (Brighter and more saturated)
  for (int i = 0; i < frontSize; i++)
  {
    int frontLed = (currentLed - i + numLeds) % numLeds;
    leds[startIndex + frontLed] = CHSV(hues[segment], 255, 255); // Max brightness
  }

  // Set tail LEDs (Dimmer, fade out more aggressively)
  if (abs(angularVelocity) > 0.1)
  {
    int tailLength = min((int)(abs(angularVelocity) * 5), maxTailLength);
    float angleDiff = currentAngle - previousAngle;

    if (abs(angleDiff) > PI)
    {
      angleDiff += (angleDiff > 0) ? -TWO_PI : TWO_PI;
    }

    for (int i = 1; i <= tailLength; i++)
    {
      float t = (float)i / tailLength;
      float tailAngle = currentAngle - angleDiff * t * tailLength;

      if (tailAngle < 0)
        tailAngle += TWO_PI;
      if (tailAngle >= TWO_PI)
        tailAngle -= TWO_PI;

      int tailLed = (int)((tailAngle / TWO_PI) * numLeds) % numLeds;
      uint8_t tailBrightness = 180 * (tailLength - i + 1) / tailLength; // More aggressive fade

      if (tailLed != currentLed)
      {
        leds[startIndex + tailLed] = CHSV(hues[segment], 255, tailBrightness);
      }
    }
  }
}

void updateDisplay()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print(F("Angle: "));
  display.println(currentAngle, 2);

  display.print(F("Velo: "));
  display.println(angularVelocity, 2);

  display.print(F("Raw Angle: "));
  display.println(as5600.rawAngle());

  display.print(F("Seg: "));
  display.println((int)(currentAngle / (TWO_PI / 4)));

  display.print(F("PWM1: "));
  display.println(map(constrain(abs(angularVelocity), 0, 96), 0, 100, 0, 255));

  display.print(F("PWM2: "));
  display.println(pwmValues[(int)(currentAngle / (TWO_PI / 4))]);

  display.print(F("AnlgIN: "));
  display.println(analogValue);

  display.display();
}
