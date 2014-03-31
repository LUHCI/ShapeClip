// 
// 
// 

#include "LDR.h"

uint16_t LDR::sample() {

	uint32_t value = 0;
	for(uint8_t i = 0; i < LDR__AVG_SAMPLE_COUNT; i++) value += analogRead(this->pin);
	value /= LDR__AVG_SAMPLE_COUNT;

	this->updateMin(value);
	this->updateMax(value);

	return value;
}

int32_t LDR::mapMinMax(int32_t value, long from, long to) {
	return constrain(map(value, this->min, this->max, from, to), from, to);
}
