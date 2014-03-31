// LDR.h

#ifndef _LDR_h
#define _LDR_h

#include <stdint.h>
#include <Arduino.h>

#define LDR__AVG_SAMPLE_COUNT 10

class LDR
{
 private:
	 int pin;
	 uint16_t min;
	 uint16_t max;
	 int32_t offset;

 public:
	 LDR(int pin) : pin(pin) {
		this->min = 0x3FF;
		this->max = 0;
		this->offset = 0;
	 }

	// Samples the ADC using analogRead and averages over LDR__AVG_SAMPLE_COUNT samples.
	uint16_t sample();

	// Samples the ADC and adjusts according to the offset
	int32_t adjustedSample() { (int32_t)this->sample() + this->offset; }

	// updates the LDR offset
	void updateOffset(int32_t offset) { this->offset = offset; }

	// Returns the min value of the LDR seen so far
	__inline__ uint16_t getMin() { return this->min; }

	// Sets the minimum value of the LDR seen so far, internally masked with 10 bits.
	__inline__ void setMin(uint16_t value) { this->min = value & 0x3FF; }

	// Sets the minimum value if the new value is lower than the previous one
	__inline__ void updateMin(uint16_t value) { this->min = min(this->min, value & 0x3FF); }


	// Returns the max value of the LDR seen so far
	__inline__ uint16_t getMax() { return this->max; } 

	// Sets the minimum value of the LDR seen so far, internally masked with 10 bits.
	__inline__ void setMax(uint16_t value) { this->max = value & 0x3FF; }

	// Sets the maximum value if the new value is greater than the previous one
	__inline__ void updateMax(uint16_t value) { this->max = max(this->max, value & 0x3FF); }


	// Maps a previously read value from min/max to from/to
	int32_t mapMinMax(int32_t value, long from, long to);

};

#endif

