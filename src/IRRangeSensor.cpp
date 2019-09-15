#include "IRRangeSensor.h"

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
  // NOTE: This is currently inverted from the app notes since I'm working
  // in 38kHz, and the lower bound would've ended up at 21.3kHz which would
  // have an OCR2A value of 375, which is a bit bigger than 255...
  // So I went upwards instead.
  // On 38kHz, this gives 40kHz ~ 49.4kHz, so an OCR2A range of 200 ~ 161.

  // In the future, I'll probably want to be able to switch implementations.
  // Shouldn't be too bad since this isn't the timing critical part.

  // state...
  /**
   * Lower-bound Timer 2 Compare value.
   * Half-period because Timer 2 is in mode 5.
   *
   * Initialized to: (Clock / Timer-2-Premultiplier) / 2 / (f0 * 1.05)
   *
   * We start slightly off-center to avoid potential non-monotonicity
   * in relative sensitivity around the center frequency.
   */
  uint8_t compUpperBound = (uint8_t)(16e6 / (2.0 * carrier.f0 * 1.05));
  /**
   * Upper-bound Timer 2 Compare value.
   * Half-period because Timer 2 is in mode 5.
   *
   * Initialized to: (Clock / Timer-2-Premultiplier) / 2 / f1
   */
  uint8_t compLowerBound = (uint8_t)(16e6 / (2.0 * carrier.f1));

  /**
   * How many iterations to perform.
   */
  uint8_t iterationsRemaining = 8;

  /**
   * Current upper-bound of search range.
   *
   * Initialized to upper-bound of input space.
   */
  uint8_t compUpper = compUpperBound;
  /**
   * Current lower-bound of search range.
   *
   * Initialized to lower-bound of input space.
   */
  uint8_t compLower = compLowerBound;
  /**
   * Current value to try.
   *
   * Initialized to: Closest to center frequency to, so as to detect
   * if something at the farthest extreme of our sensor's range.
   */
  uint8_t compTry = compUpperBound;

  // binary search of input space...

  // Initial read:
  {
    read(compTry);

    if (wasReadingValid() != true)
    {
      return -1;
    }
  }

  // Main loop:
  while (iterationsRemaining != 0)
  {
    compTry = compLower + ((compUpper - compLower) >> 1);
    iterationsRemaining -= 1;

    read(compTry);

    if (wasReadingValid() == true)
    {
      // Range closer by shifting the working upper bound downwards.
      compUpper = compTry;
    }
    else
    {
      // Range farther by shifting the working lower bound upwards.
      compLower = compTry;
    }
  }

  compTry = compLower + ((compUpper - compLower) >> 1);

  // If upper bound is set by f0, use this.
  // If lower bound is set by f0, remove the "100 - " at the beginning.
  return (
    100 - (compTry - compLowerBound) * 100 / (compUpperBound - compLowerBound)
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
