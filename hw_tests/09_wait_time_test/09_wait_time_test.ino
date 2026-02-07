/*!
 * @file 09_wait_time_test.ino
 * @brief Hardware test for AS7343 wait time settings
 *
 * Tests WTIME register and enableWait:
 * - setWaitTime() / getWaitTime() readback verification
 * - enableWait(true/false) doesn't crash
 * - Optional timing test: wait-enabled takes longer than wait-disabled
 *
 * Wait time formula: t_wait = (WTIME + 1) × 2.78 ms
 * Range: 2.78ms (WTIME=0) to 711.68ms (WTIME=255)
 *
 * Hardware: AS7343 on I2C, NeoPixel ring (16 LEDs) on pin 6
 */

#include <Adafruit_AS7343.h>
#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 6
#define NEOPIXEL_COUNT 16

Adafruit_AS7343 as7343;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Test WTIME values
const uint8_t testWTIME[] = {0, 100, 255};
const uint8_t NUM_WTIME_TESTS = sizeof(testWTIME) / sizeof(testWTIME[0]);

// Calculate wait time in ms from WTIME register value
float calcWaitTime(uint8_t wtime) { return (wtime + 1) * 2.78; }

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println(F("AS7343 Wait Time Test"));
  Serial.println(F("====================="));

  // Initialize NeoPixels
  pixels.begin();
  pixels.clear();
  pixels.show();

  // Initialize sensor
  if (!as7343.begin()) {
    Serial.println(F("ERROR: AS7343 not found!"));
    Serial.println(F("RESULT: FAIL"));
    while (1)
      delay(100);
  }

  Serial.println(F("NeoPixels ON..."));
  Serial.println();

  // Turn NeoPixels ON at moderate brightness
  for (int i = 0; i < NEOPIXEL_COUNT; i++) {
    pixels.setPixelColor(i, pixels.Color(64, 64, 64));
  }
  pixels.show();
  delay(100);

  // ==========================================
  // Test 1: Wait Time Readback
  // ==========================================
  Serial.println(F("Wait Time Readback:"));
  Serial.println(F("WTIME    Readback  Calc Time(ms)  Status"));
  Serial.println(F("-----    --------  -------------  ------"));

  bool allReadbacksOK = true;

  for (uint8_t i = 0; i < NUM_WTIME_TESTS; i++) {
    uint8_t wtime = testWTIME[i];

    // Set wait time
    as7343.setWaitTime(wtime);
    delay(10);

    // Read back
    uint8_t readback = as7343.getWaitTime();

    // Calculate expected wait time
    float calcTime = calcWaitTime(wtime);

    // Verify
    bool pass = (readback == wtime);
    if (!pass)
      allReadbacksOK = false;

    // Print formatted output
    Serial.print(wtime);
    if (wtime < 10)
      Serial.print(F("        "));
    else if (wtime < 100)
      Serial.print(F("       "));
    else
      Serial.print(F("      "));

    Serial.print(readback);
    if (readback < 10)
      Serial.print(F("         "));
    else if (readback < 100)
      Serial.print(F("        "));
    else
      Serial.print(F("       "));

    Serial.print(calcTime, 2);
    if (calcTime < 10)
      Serial.print(F("          "));
    else if (calcTime < 100)
      Serial.print(F("         "));
    else
      Serial.print(F("        "));

    Serial.println(pass ? F("PASS") : F("FAIL"));
  }

  Serial.println();

  // ==========================================
  // Test 2: Enable Wait Test
  // ==========================================
  Serial.println(F("Enable Wait test:"));

  // Test enableWait(true)
  as7343.enableWait(true);
  delay(10);
  Serial.println(F("  enableWait(true): OK"));

  // Test enableWait(false)
  as7343.enableWait(false);
  delay(10);
  Serial.println(F("  enableWait(false): OK"));

  Serial.println();

  // ==========================================
  // Summary
  // ==========================================
  Serial.print(F("Wait time readback: "));
  Serial.println(allReadbacksOK ? F("PASS") : F("FAIL"));

  // Turn NeoPixels OFF
  pixels.clear();
  pixels.show();

  // Final result
  Serial.print(F("RESULT: "));
  Serial.println(allReadbacksOK ? F("PASS") : F("FAIL"));
}

void loop() {
  // Nothing to do
}
