#ifndef PI_CONTROLLER_HPP
#define PI_CONTROLLER_HPP
#include "constants.hpp"

class PIController
{
private:
	float Kp_ = PIController_Kp;
	float Ki_ = PIController_Ki;
	float integral_ = 0.0f;
	float integral_max_ = PIController_integral_max;

public:
	void setGains(float kp, float ki)
	{
		Kp_ = kp;
		Ki_ = ki;
	}
	void reset() { integral_ = 0.0f; }

	float update(float error, float dt)
	{
		float p_term = Kp_ * error;
		integral_ += (error * Ki_ * dt);

		// Simple Anti-windup clamp
		integral_ = std::clamp(integral_, -integral_max_, integral_max_);

		return p_term + integral_;
	}
};

#endif