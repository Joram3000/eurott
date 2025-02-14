#include "AS5600.h"           // MAGNETIC ENCODER
#include <FastLED.h>          // LED CONTROLLER
#include <Wire.h>             // I2C
#include <Adafruit_GFX.h>     // OLED DISPLAY
#include <Adafruit_SSD1306.h> // OLED DISPLAY
#include <Adafruit_MCP4728.h> // DAC

// MCP4728 setup
Adafruit_MCP4728 mcp;

// AS5600 setup
AS5600 as5600;
#define ANALOG_PIN 34 // Analog pin for AS5600

// LED setup
#define LED_PIN 13 // LED data pin
#define NUM_LEDS_1 60
#define NUM_LEDS_2 48
#define NUM_LEDS_3 40
#define TOTAL_LEDS (NUM_LEDS_1 + NUM_LEDS_2 + NUM_LEDS_3)
#define BRIGHTNESS 12
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[TOTAL_LEDS];

// OLED display setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Variables for angle and velocity
float currentAngle = 0;
float previousAngle = 0;
float angularVelocity = 0;
const float movementThreshold = 0.005;

// Variables for analog and PWM values
int analogValue;
const int pwmValues[4] = {0, 8, 32, 128}; // Adjusted PWM values for ESP32
const uint8_t hues[4] = {16, 192, 48, 96};
const int frontSize = 1;
const int maxTailLength = 8;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("PRRRRRRA!!!"));
  display.display();
  delay(600);

  // Initialize AS5600
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);

  // Initialize LEDs
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, TOTAL_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  // Initialize MCP4728
  if (!mcp.begin())
  {
    Serial.println("Failed to find MCP4728 chip");
    while (1)
    {
      delay(10);
    }
  }
}

void loop()
{
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

  display.display();
}