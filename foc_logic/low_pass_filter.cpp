#ifndef LOW_PASS_FILTER
#define LOW_PASS_FILTER
#include "constants.hpp"
class LowPassFilter
{
private:
	float y_prev = 0.0f;
	float alpha = 0.1f; // Lower = more filtering, 0-1

public:
	void setCutoff(float cutoff_hz, float dt)
	{
		// alpha = dt / (dt + 1/(2*PI*cutoff))
		float rc = 1.0f / (math_TWO_PI * cutoff_hz);
		alpha = dt / (dt + rc);
	}

	float filter(float input)
	{
		y_prev = alpha * input + (1.0f - alpha) * y_prev;
		return y_prev;
	}

	void reset() { y_prev = 0.0f; }
};
#endif