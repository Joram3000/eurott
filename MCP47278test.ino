#include <Wire.h>
#include <Adafruit_MCP4728.h>

// Create the MCP4728 object
Adafruit_MCP4728 mcp;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // wait for serial port to open

  Serial.println("Adafruit MCP4728 test");

  // Initialize I2C with custom pins
  Wire.begin(21, 22);

  // Initialize the MCP4728
  if (!mcp.begin())
  {
    Serial.println("Failed to find MCP4728 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MCP4728 Found!");

  // Set the DAC values
  mcp.setChannelValue(MCP4728_CHANNEL_A, 2048); // 50% of 12-bit range
  mcp.setChannelValue(MCP4728_CHANNEL_B, 1024); // 25% of 12-bit range
  mcp.setChannelValue(MCP4728_CHANNEL_C, 3072); // 75% of 12-bit range
  mcp.setChannelValue(MCP4728_CHANNEL_D, 4095); // 100% of 12-bit range

  // Update the DAC outputs
  mcp.saveToEEPROM();
}

void loop()
{
  // Nothing to do here
}