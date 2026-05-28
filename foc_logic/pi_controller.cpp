#ifndef PI
#define PI
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
		integral_ += error * Ki_ * dt;

		// Anti-windup clamp
		if (integral_ > integral_max_)
			integral_ = integral_max_;
		if (integral_ < -integral_max_)
			integral_ = -integral_max_;

		return Kp_ * error + integral_;
	}
};

#endif