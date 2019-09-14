#ifndef __IR_RANGE_SENSOR_H__
#define __IR_RANGE_SENSOR_H__ yey

#include <Arduino.h>

#define CARRIER_F0_38KHZ 38e3
#define CARRIER_F0_56KHZ 56e3

#define CARRIER_BURST_COUNT_DEFAULT 25
#define CARRIER_BURST_COUNT_LOWER_BOUND_MARGIN 15
#define CARRIER_BURST_COUNT_UPPER_BOUND_MARGIN 6
#define CARRIER_BURST_COUNT_DEAD_TIME_UPPER_BOUND 25

#define PROGRESS_BEGUN 1
#define PROGRESS_RECV_FALLING 1 << 1
#define PROGRESS_RECV_RISING 1 << 2
#define PROGRESS_BURST_END 1 << 3
#define PROGRESS_BURST_END_NO_RECV_FALLING 1 << 4
#define PROGRESS_RECV_TIMEOUT_IN_BURST_OVF_INTERRUPT 1 << 5
#define PROGRESS_END 1 << 7

/**
 * Encapsulates the logic and state for doing IR Range Sensing using
 * a an IR sensor found in the typical TV remote, such as the TSOP38238,
 * and an IR LED.  Can be configured for any center-frequency,
 * though 38kHz and 56kHz are common.
 *
 * NOTE: This takes over the following things!
 *
 * - Timer 1, used to modulate the carrier emitter.
 *     - Pin 8 (PB0/ICP1), signal edge detector, connected to the output of your IR sensor.
 * - Timer 2, used to time the receiver activity.
 *     - Pin 3 (PD3/OC2B), carrier output modulator, connected to your IR LED(s) or to the switch controlling it/them.
 *     - Pin 11 (OC2A), not used and not usable.
 *
 * The following interrupt vectors are set:
 *
 * - TIMER1_CAPT_vect
 * - TIMER2_OVF_vect
 */
struct IRRangeSensor {
  // Maybe-TODO: Make most things private and add a report method
  // if people want to log things out to serial?  Hm.

  struct Carrier {
    /**
     * Center frequency of the carrier.
     *
     * Unit: Hz
     */
    float f0;
    /**
     * Off-Center frequency of the carrier.
     *
     * Currently hard-coded to f0 * 0.7, where typical sensors
     * such as the TSOP38238 or TSSP4056 are about 0.15x the sensitivity
     * relative to their sensitivity at f0.
     *
     * Unit: Hz
     */
    float f1;
    /**
     * How many pulses of the carrier to emit per burst.
     *
     * Unit: Carrier-Pulses
     */
    uint8_t burstCount;

    /**
     * Upper bound for number of pulses that can elapse during the received
     * signal duration after which we declare a signal invalid.
     *
     * Currently hardcoded to burstCount + 6.
     *
     * This is used to later derive the actual time value in Timer1 Counts.
     *
     * Unit: Carrier-Pulses
     */
    uint8_t burstCountUpperBound;
    /**
     * Lower bound for the number of pulses that can elapse during the received
     * signal duration after before which we declare a signal invalid.
     *
     * This is used to later derive the actual time value in Timer1 Counts.
     *
     * Currently hardcoded to burstCount - 15.  Ordinarily it would be
     * burstCount - 5, but we're going very far off the center frequency.
     *
     * Unit: Carrier-Pulses
     */
    uint8_t burstCountLowerBound;

    /**
     * How many pulses are there left to fire in this burst.
     * Countdown variable used by the interrupt.
     */
    volatile uint8_t burstPulsesRemaining;

    void setup();
  } carrier;

  struct Receiver {
    volatile bool didReceive;
    volatile uint16_t ticksElapsed;
  } receiver;

  volatile bool senseEnd;
  volatile uint8_t progress;

  void setup();
  int8_t readDistance();
  uint16_t read(uint8_t);
  bool wasReadingValid();
};

extern struct IRRangeSensor irRangeSensor;

#endif // __IR_RANGE_SENSOR_H__
