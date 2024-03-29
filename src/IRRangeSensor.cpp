#include "IRRangeSensor.h"
// #define IR_RANGE_SENSOR_READ_DISTANCE_DEBUG true

struct IRRangeSensor irRangeSensor;

void IRRangeSensor::Carrier::setup()
{
  // f0 = carrierF0;
  // burstCount = carrierBurstCount;

  // // For now, using 1.3x instead of 0.56x because that lets us use
  // // 38kHz, at the cost of some precision and range in our results.
  // f1 = f0 * 1.3;
  burstCountLowerBound = burstCount - CARRIER_BURST_COUNT_LOWER_BOUND_MARGIN;
  burstCountUpperBound = burstCount + CARRIER_BURST_COUNT_UPPER_BOUND_MARGIN;
}

void IRRangeSensor::setup()
{
  carrier.setup();

  // First, setting up Timer 2
  // -------------------------

  // This is basically copied verbatim from the Slow Proximity note.
  // Granted, it does exactly what I want, and there's
  // only so many ways to twiddle bits.

  // - Set Wave Gen Mode to Mode 5: PWM Phase Correct with Top = OCR2A
  //   (TCCR2A.WGM20 & TCCR2B.WGM22)
  // - Use System Clock with No Prescaler (TCCR2B.CS20)
  //   NOTE: We won't use OC2A for output, but we will use OC2B
  //   in 1,0 mode (clear on match counting up, set on match counting down).
  TCCR2A = (1 << WGM20); // | (0<<COM2B1);
  TCCR2B = (1 << WGM22) | (1 << CS20);

  // set arduino D3, AVR PD3 as output pin
  // it can be hard-disabled as an output by setting it as an input pin,
  // but I think twiddling COM2B1 is fine.
  DDRD |= (1 << DDD3);

  // Update the halfPeriod and dutyCompValue.
  // updateCarrierDerivedValues();

  // Next, setting up Timer 1
  // ------------------------

  // Here, we do the following:
  // - Enable Input Capture Noise Cancelation (TCCR1B.ICNC1)
  // - Set the Prescaler to 64 (TCCR1B.CS11 & TCCR1B.CS10)
  TCCR1B = (1 << ICNC1) | (1 << CS11) | (1 << CS10);
  // Set to Rising Edge since that's what it's going to be set to
  // if a successful signal detection has occurred.
  // We explicitly expect this behavior, and take advantage of that
  // in other places.
  TCCR1B &= ~(1 << ICES1);

  // We'll set OCR1A and OCR1B to appropriate values later,
  // then enable TIMSK1.ICIE1, etc.
}

/**
 * Does a rough range-find by attempting to detect the carrier
 * at different frequencies.
 *
 * @returns A value 0 to 100 for range, or -1 for no detection at all.
 *   0 means our carrier had to stay near f0, meaning the object was far.
 *   100 means our carrier could stray from f0, meaning the object was near.
 */
int8_t IRRangeSensor::readDistance()
{
  // NOTE: This includes some added conditional logic to switch the math
  // depending on if OCR2A(f0) < OCR2A(f1) or vice versa.

  // state...
  /**
   * Lower-bound Timer 2 Compare value.
   * Half-period because Timer 2 is in mode 5.
   *
   * Initialized to: (Clock / Timer-2-Premultiplier) / 2 / f0
   *
   * NOTE: We actually start slightly off-center to avoid potential
   * non-monotonicity in relative sensitivity around the center frequency.
   */
  uint8_t compFarInit = (uint8_t)(16e6 / (2.0 * carrier.f0));
  /**
   * Upper-bound Timer 2 Compare value.
   * Half-period because Timer 2 is in mode 5.
   *
   * Initialized to: (Clock / Timer-2-Premultiplier) / 2 / f1
   */
  uint8_t compNearInit = (uint8_t)(16e6 / (2.0 * carrier.f1));

  /**
   * Is near greater than far?
   * Used to switch math later to stay positive.
   */
  bool nearGtFar = compNearInit > compFarInit;

  /**
   * How many iterations to perform.
   */
  uint8_t iterationsRemaining = 5;

  /**
   * Current upper-bound of search range.
   *
   * Initialized to upper-bound of input space.
   */
  uint8_t compFar = compFarInit;
  /**
   * Current lower-bound of search range.
   *
   * Initialized to lower-bound of input space.
   */
  uint8_t compNear = compNearInit;
  /**
   * Current value to try.
   *
   * Initialized to: Closest to center frequency to, so as to detect
   * if something at the farthest extreme of our sensor's range.
   */
  uint8_t compTry = compFarInit;

  // binary search of input space...

  // Initial read:
  {
    read(compTry);

    if (wasReadingValid() != true)
    {
      #ifdef IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
      Serial.println("Nothing!");
      #endif // IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
      return -1;
    }
  }

  #ifdef IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
  Serial.print(compNearInit, HEX);
  Serial.print("~");
  Serial.print(compFarInit, HEX);
  Serial.print(": ");
  Serial.print(compTry, HEX);
  Serial.print(" (");
  Serial.print(16e6 / (2.0 * (float)compTry), 1);
  Serial.print(") |< ");
  #endif // IR_RANGE_SENSOR_READ_DISTANCE_DEBUG

  // Main loop:
  while (iterationsRemaining != 0)
  {
    // compTry = compNear + ((compFar - compNear) >> 1);
    compTry = nearGtFar
      ? ((compNear - compFar) >> 1) + compFar
      : ((compFar - compNear) >> 1) + compNear;
    iterationsRemaining -= 1;

    read(compTry);

    #ifdef IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
    Serial.print(compTry, HEX);
    Serial.print(" (");
    Serial.print(16e6 / (2.0 * (float)compTry), 1);
    Serial.print(")");
    #endif // IR_RANGE_SENSOR_READ_DISTANCE_DEBUG

    if (wasReadingValid() == true)
    {
      #ifdef IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
      Serial.print(" |< ");
      #endif // IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
      // Range closer by shifting the working upper bound downwards.
      compFar = compTry;
    }
    else
    {
      #ifdef IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
      Serial.print(" |> ");
      #endif // IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
      // Range farther by shifting the working lower bound upwards.
      compNear = compTry;
    }

    #ifdef IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
    Serial.print("=");
    Serial.print(compNear, HEX);
    Serial.print("~");
    Serial.print(compFar, HEX);
    Serial.print(";  ");
    #endif // IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
  }

  compTry = nearGtFar
    ? ((compNear - compFar) >> 1) + compFar
    : ((compFar - compNear) >> 1) + compNear;

  #ifdef IR_RANGE_SENSOR_READ_DISTANCE_DEBUG
  Serial.print(": ");
  Serial.println(compTry, HEX);
  #endif // IR_RANGE_SENSOR_READ_DISTANCE_DEBUG

  return (
    ((nearGtFar ? compTry - compFarInit : compFarInit - compTry) * 100)
    / (nearGtFar ? compNearInit - compFarInit : compFarInit - compNearInit)
  );
}

uint16_t IRRangeSensor::read(uint8_t carrierComp)
{
  // Since the OVF interrupt is acting as both carrier control
  // and receiver timeout, we need to take into account both the
  // maximum acceptable receiver activation time (tpo < t + 6/f0) and
  // the maximum expected receiver pre-activation dead-time (td < 15/f0)
  carrier.burstPulsesRemaining = (
    carrier.burstCountUpperBound
    + CARRIER_BURST_COUNT_DEAD_TIME_UPPER_BOUND
  );

  receiver.didReceive = false;
  receiver.ticksElapsed = 0;
  senseEnd = false;
  progress = 0;

  // Set to falling edge.
  TCCR1B &= ~(1 << ICES1);

  // Set our frequency.
  OCR2A = carrierComp;
  // Set our duty, which is just 50%.
  OCR2B = carrierComp >> 1;

  // Then the fun begins.
  // Reset Timer 2's counter to ensure a good first pulse.
  TCNT2 = 0;

  // start switching on the emitter.
  TCCR2A |= (1 << COM2B1);
  // Enable Timer 2's overflow interrupt so we can start counting pulses.
  TIMSK2 |= (1 << TOIE2);

  // Enable Timer 1's input capture event interrupt to listen for
  // IR sensor changes.
  TIMSK1 |= (1 << ICIE1);

  progress |= PROGRESS_BEGUN;

  // Then... we wait.
  while (irRangeSensor.senseEnd != true);

  progress |= PROGRESS_END;

  // Just return, we stored actual validity elsewhere.
  // I mean, we stored the value returned too, but hey.
  return irRangeSensor.receiver.ticksElapsed;
}

bool IRRangeSensor::wasReadingValid()
{
  if (irRangeSensor.receiver.didReceive != true)
  {
    return false;
  }

  float ticksPerPulse = 16e6 / (64e0 * carrier.f0);
  uint16_t tickCountLowerBound = (uint16_t)(ticksPerPulse * carrier.burstCountLowerBound);
  uint16_t tickCountUpperBound = (uint16_t)(ticksPerPulse * carrier.burstCountUpperBound);

  return (
    receiver.ticksElapsed > tickCountLowerBound
    && receiver.ticksElapsed < tickCountUpperBound
  );
}

/**
 * Timer 1 Input Capture Event (ICE) Interrupt
 */
ISR(TIMER1_CAPT_vect)
{
  if ((TCCR1B & (1 << ICES1)) == 0)
  {
    // We were called on a falling edge.
    // The IR sensor picked up a signal!
    // Zero-out the timer counter,
    TCNT1 = 0;
    // then set edge detection to rising edge.
    TCCR1B |= 1 << ICES1;

    irRangeSensor.progress |= PROGRESS_RECV_FALLING;
  }
  else
  {
    // We were called on a rising edge.
    // The IR sensor has stopped receiving a signal!

    // Disable input capture interrupt,
    TIMSK1 &= ~(1 << ICIE1);

    // Disconnect OC2B, stopping pulsing the output.
    TCCR2A &= ~(1 << COM2B1);
    // Disable the overflow interrupt,
    TIMSK2 &= ~(1 << TOIE2);

    // Read the count from the Input Capture Register
    // rather than the Timer Counter Register TCNT1,
    // as this hopefully reduces the chance of corruption,
    // (although given we have a premultiplier of 64,
    // that's probably not a problem?)
    irRangeSensor.receiver.ticksElapsed = ICR1;
    irRangeSensor.receiver.didReceive = true;

    // And signal that we're done.
    irRangeSensor.senseEnd = true;
    irRangeSensor.progress |= PROGRESS_RECV_RISING;
  }
}

/**
 * Timer 2 Overflow Interrupt
 */
ISR(TIMER2_OVF_vect)
{
  irRangeSensor.carrier.burstPulsesRemaining -= 1;

  // Check if we should stop the carrier...
  if (irRangeSensor.carrier.burstPulsesRemaining == CARRIER_BURST_COUNT_UPPER_BOUND_MARGIN + CARRIER_BURST_COUNT_DEAD_TIME_UPPER_BOUND)
  {
    // Once we've counted down...

    // Disconnect OC2B, stopping pulsing the output.
    TCCR2A &= ~(1 << COM2B1);

    irRangeSensor.progress |= PROGRESS_BURST_END;
  }
  else if (irRangeSensor.carrier.burstPulsesRemaining == 0)
  {
    // Disable the overflow interrupt,
    TIMSK2 &= ~(1 << TOIE2);

    if (irRangeSensor.senseEnd != true)
    {
      // Disable the ICE interrupt, just in case.
      TIMSK1 &= ~(1 << ICIE1);

      // then set edge detection to rising edge, since that's how
      // a successful detection would end,
      TCCR1B &= ~(1 << ICES1);

      irRangeSensor.senseEnd = true;
      irRangeSensor.progress |= PROGRESS_RECV_TIMEOUT_IN_BURST_OVF_INTERRUPT;
    }
  }
}
