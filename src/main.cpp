#include <Arduino.h>
#include "IRRangeSensor.h"

char output[102] = {0};

// some other stuff, just to make sure all the pins are used.

// A0 is used for the diodes
const int IN_SENSE = 0;
// A1 is used for the center voltage reference
const int IN_VREF = 1;

// Pin to light up more when the higher photodiode conducts more
const int OUT_HIGHER = 5;
// Pin to light up more when the lower photodiode conducts more
const int OUT_LOWER = 6;

void setupDiodeDifferential()
{
  pinMode(OUT_HIGHER, OUTPUT);
  pinMode(OUT_LOWER, OUTPUT);
}

void updateDiodeDifferential()
{
  // put your main code here, to run repeatedly:
  int diodeValue = analogRead(IN_SENSE);
  int refValue = analogRead(IN_VREF);
  int difference = (diodeValue - refValue) / 2;

  if (difference > 0)
  {
    analogWrite(OUT_HIGHER, difference);
    analogWrite(OUT_LOWER, 0);
  }
  else if (difference < 0)
  {
    analogWrite(OUT_HIGHER, 0);
    analogWrite(OUT_LOWER, 0 - difference);
  }
  else
  {
    analogWrite(OUT_HIGHER, 0);
    analogWrite(OUT_LOWER, 0);
  }
}

void printProgress()
{
  Serial.print("Progress: ");
  Serial.print(
    irRangeSensor.progress & PROGRESS_BEGUN
      ? "BEGUN "
      : " "
  );
  Serial.print(
    irRangeSensor.progress & PROGRESS_RECV_FALLING
      ? "RECV_FALLING "
      : " "
  );
  Serial.print(
    irRangeSensor.progress & PROGRESS_RECV_RISING
      ? "RECV_RISING "
      : " "
  );
  Serial.print(
    irRangeSensor.progress & PROGRESS_BURST_END
      ? "BURST_END "
      : " "
  );
  Serial.print(
    irRangeSensor.progress & PROGRESS_BURST_END_NO_RECV_FALLING
      ? "BURST_END_NO_RECV_FALLING "
      : " "
  );
  Serial.print(
    irRangeSensor.progress & PROGRESS_RECV_TIMEOUT_IN_BURST_OVF_INTERRUPT
      ? "RECV_TIMEOUT_IN_BURST_OVF_INTERRUPT "
      : " "
  );
  Serial.print(
    irRangeSensor.progress & PROGRESS_END
      ? "END "
      : " "
  );
  Serial.println(".");
}

void doReadingTimesAndPrint(uint8_t readingCount, long compValue)
{
  uint16_t lastReading;
  float average = 0.0;
  uint8_t timesValid = 0;

  compValue = constrain(compValue, 0, 255);

  Serial.print("Reading with OCR2A of ");
  Serial.print(compValue);
  Serial.print(" (");
  Serial.print(16e6 / (2.0 * (float)compValue) / 1e3, 1);
  Serial.print("kHz)... ");

  for (uint8_t i = 0; i < readingCount; ++i)
  {
    lastReading = irRangeSensor.read(compValue);

    if (irRangeSensor.wasReadingValid())
    {
      timesValid += 1;
      average += lastReading;
    }
  }

  if (timesValid == 0)
  {
    Serial.println("(n/a)");
  }
  else
  {
    Serial.print(average / (float)timesValid, 2);
    Serial.print(", ");
    Serial.print(timesValid);
    Serial.print("/");
    Serial.print(readingCount);
    Serial.println(" valid reads");
  }
}

void doSweepAcrossReadingTimes(uint8_t readingCount)
{
  for (long ocr2a = 255; ocr2a >= 210; --ocr2a)
  {
    doReadingTimesAndPrint(readingCount, ocr2a);
  }
}

void singleReading()
{
  while (Serial.available() > 0)
  {
    Serial.print("Enter OCR2A: ");
    long compValue = Serial.parseInt();

    if (Serial.read() == '\n')
    {
      compValue = constrain(compValue, 0, 255);

      Serial.print("Reading with OCR2A of ");
      Serial.print(compValue);
      Serial.println("...");

      uint16_t reading = irRangeSensor.read(compValue);

      if (irRangeSensor.wasReadingValid() == true)
      {
        Serial.print("Reading: ");
        Serial.println(reading);
      }
      else
      {
        Serial.println("Reading was not valid!");
      }

      printProgress();
    }
  }
}

void multipleReadings(uint8_t readingCount)
{
  while (Serial.available() > 0)
  {
    long compValue = Serial.parseInt();

    if (Serial.read() == '\n')
    {
      doReadingTimesAndPrint(readingCount, compValue);
      // printProgress();
    }
    Serial.print("Enter OCR2A: ");
  }
}

void distanceSense()
{
  int8_t distance = irRangeSensor.readDistance();

  if (distance == -1)
  {
    Serial.println("-------- nothing detected! --------");
  }
  else
  {
    // This is a bit easier to graph.
    // now 100 = far away, 0 = up close
    distance = 100 - distance;

    for (int8_t ci = 0; ci < 101; ++ci)
    {
      if (ci < distance)
      {
        output[ci] = ' ';
      }
      else if (ci == distance)
      {
        output[ci] = '|';
      }
      else
      {
        output[ci] = '=';
      }
    }
    output[100] = '|';

    distance = 100 - distance;

    if (distance < 10)
    {
      Serial.print("  ");
    }
    else if (distance < 100)
    {
      Serial.print(" ");
    }
    Serial.print(distance);
    Serial.print(": ");
    Serial.println(output);
  }
}

void sweepOnLF(uint8_t readingCount)
{
  while (Serial.available() > 0)
  {
    if (Serial.read() == '\n')
    {
      doSweepAcrossReadingTimes(readingCount);
    }
  }
}

void setup() {
  setupDiodeDifferential();

  // Keep it off center to avoid non-monotonicity.
  irRangeSensor.carrier.f0 = 40e3;
  // Above 48.8kHz, the TSOP38238 just suddenly cannot even the signal.
  irRangeSensor.carrier.f1 = 48.8e3;
  irRangeSensor.carrier.burstCount = CARRIER_BURST_COUNT_DEFAULT;
  irRangeSensor.setup();

  Serial.begin(9600);
  while (!Serial);
  Serial.println("hi!");
  Serial.print("Enter OCR2A: ");
}

void loop() {
  // attempting to see... well, anything.
  // platformio device monitor --echo --eol LF --port /whatever/thing

  // singleReading();
  // multipleReadings(64);
  // sweepOnLF(64);
  // updateDiodeDifferential();
  distanceSense();
}
