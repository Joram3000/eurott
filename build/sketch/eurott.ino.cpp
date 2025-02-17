#include <Arduino.h>
#line 1 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
#include "AS5600.h"
#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP4728.h>
#include <cmath>

// MCP4728 setup
Adafruit_MCP4728 mcp;

// AS5600 setup
AS5600 as5600;
#define ANALOG_PIN 34

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

// Constants
const float MOVEMENT_THRESHOLD = 0.004;

// Variables for angle and velocity
float currentAngle = 0;
float previousAngle = 0;
float angularVelocity = 0;
float minMaxVelocity[2] = {0, 0};

// Variables for analog and PWM values
int segmentIndex = 0;
const int segments[4] = {1000, 2000, 3000, 4000};
const uint8_t hues[4] = {20, 200, 54, 100};
const int frontSize = 1;
const int maxTailLength = 8;

#line 50 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void setup();
#line 87 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void loop();
#line 146 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void updateLEDs();
#line 154 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void updateRing(int startIndex, int numLeds);
#line 195 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
void updateDisplay();
#line 50 "/Users/joram/Documents/Arduino/eurott/eurott/eurott.ino"
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
  if (abs(angleDiff) > MOVEMENT_THRESHOLD)
  {
    previousAngle = currentAngle;
    currentAngle += angleDiff * 0.6;
    if (currentAngle < 0)
      currentAngle += TWO_PI;
    if (currentAngle >= TWO_PI)
      currentAngle -= TWO_PI;
    angularVelocity = angleDiff / deltaTime;
  }
  else
  {
    angularVelocity *= 0.2;
  }

  // // **Nieuwe RPM-berekening**
  // float rpm = abs(angularVelocity) * (60.0 / TWO_PI); // Omega → RPM
  // float logVelocity = log2(rpm / 30.0) + 3.0;         // Log-schaal voor octaven
  // int mappedVelocity = constrain((logVelocity / 10.0) * 4045, 0, 4045); // Schalen naar DAC

  // **Nieuwe RPM-berekening**
  float rpm = abs(angularVelocity) * (60.0 / TWO_PI);             // Omega → RPM
  float logVelocity = log2(rpm / 30.0) + 2.0;                     // Log-schaal voor octaven
  float scaledVelocity = (logVelocity / 10.0) * 4045.0;           // Schalen naar DAC
  int mappedVelocity = constrain(round(scaledVelocity), 0, 4045); // Round and constrain

  // **DAC-uitgangen**
  mcp.setChannelValue(MCP4728_CHANNEL_A, mappedVelocity);
  mcp.setChannelValue(MCP4728_CHANNEL_B, rawAngle);
  segmentIndex = (int)(currentAngle / (TWO_PI / 4));
  mcp.setChannelValue(MCP4728_CHANNEL_C, segments[segmentIndex]);
  mcp.setChannelValue(MCP4728_CHANNEL_D, rawAngle);

  // **Min/max tracking**
  if (angularVelocity > minMaxVelocity[1])
    minMaxVelocity[1] = angularVelocity;
  if (angularVelocity < minMaxVelocity[0])
    minMaxVelocity[0] = angularVelocity;

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

  // Determine the hue adjustment based on the direction
  int hueAdjustment = (angularVelocity > 0) ? 12 : -12;

  // Set front LEDs
  for (int i = 0; i < frontSize; i++)
  {
    int frontLed = (currentLed - i + numLeds) % numLeds;
    leds[startIndex + frontLed] = CHSV(hues[segmentIndex] + hueAdjustment, 255, 255);
  }

  // Set tail LEDs
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
      uint8_t tailBrightness = 180 * (tailLength - i + 1) / tailLength;
      if (tailLed != currentLed)
      {
        leds[startIndex + tailLed] = CHSV(hues[segmentIndex] + hueAdjustment, 255, tailBrightness);
      }
    }
  }
}

void updateDisplay()
{
  static float lastVelocity = 0;
  static float lastRawAngle = 0;
  static int lastSegmentIndex = -1;

  if (true)
  {
    // Display the graph
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Draw the graph
    float rpm = abs(angularVelocity) * (60.0 / TWO_PI);             // Omega → RPM
    float logVelocity = log2(rpm / 30.0) + 2.0;                     // Log-schaal voor octaven
    float scaledVelocity = (logVelocity / 10.0) * 4045.0;           // Schalen naar DAC
    int mappedVelocity = constrain(round(scaledVelocity), 0, 4045); // Round and constrain

    // Draw the graph (example)
    int graphHeight = map(mappedVelocity, 0, 4045, 0, SCREEN_HEIGHT);
    display.fillRect(0, SCREEN_HEIGHT - graphHeight, SCREEN_WIDTH, graphHeight, SSD1306_WHITE);

    display.display();
  }
  else
  {
    // Display the current numbers
    if (abs(angularVelocity - lastVelocity) > 0.01 || as5600.rawAngle() != lastRawAngle || segmentIndex != lastSegmentIndex)
    {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);

      display.setCursor(0, 0);
      display.print(F("Velo: "));
      display.println(angularVelocity, 2);
      display.print(F("Max:"));
      display.print(minMaxVelocity[1], 1);
      display.print(F(" Min: "));
      display.println(minMaxVelocity[0], 1);

      display.print(F("Raw: "));
      display.println(as5600.rawAngle());

      display.print(F("Seg: "));
      display.println(segmentIndex);

      display.display();

      lastVelocity = angularVelocity;
      lastRawAngle = as5600.rawAngle();
      lastSegmentIndex = segmentIndex;
    }
  }
}
