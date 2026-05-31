#ifndef FOC_CONTROLLER
#define FOC_CONTROLLER

#include "iostream"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "constants.hpp"
#include "pi_controller.cpp"
#include "low_pass_filter.cpp"

/***************************************************/
/*                  priv func impl                 */
/***************************************************/

//////////////////////////////////////////////////////

class FOCController
{
private:
	// desired torque currents
	float i_q_ref_ = 0;
	float i_d_ref_ = 0; // should be 0, max torque

	// measured phase currents
	float i_a_ = 0;
	float i_b_ = 0;
	float i_c_ = 0;

	float i_q_ = 0;
	float i_d_ = 0;

	// ADC Offsets (Calibrated at startup)
	float offset_ia_ = 0, offset_ib_ = 0, offset_ic_ = 0;
	bool calibrated_ = false;

	float v_q_ = 0;
	float v_d_ = 0;
	float v_max_ = dc_bus_voltage * math_ONE_BY_SQRT3; // SVPWM limit (Vdc / sqrt(3))

	// Low-pass filters for current measurements
	LowPassFilter lpf_a_{}, lpf_b_{}, lpf_c_{};

	float v_a_ = 0.0f;
	float v_b_ = 0.0f;
	float v_c_ = 0.0f;

	// duties
	float duty_a_ = 0;
	float duty_b_ = 0;
	float duty_c_ = 0;

	// dt
	float dt_ = 0;

	// PI
	PIController pi_q_{}, pi_d_{};

	float error_q_ = 0;
	float error_d_ = 0;

	// angle
	float angle_rad_ = 0;

public:
	FOCController(/* args */);

	// delta time
	void setDt(float dt);

	// run FOC algorithm
	void run();

	// to set the desired torque currents
	void setDesiredTorque(float i_q);

	// set current angle, should be from encoder
	// need in park transfom
	void updateAngle(float angle_rad);

	// set measured currents, read actual phase currents (from the inverter circuit)
	// need to a feedback system, clarke, park transfoms
	void updateCurrents(float i_a, float i_b, float i_c);

	void calibrateCurrents(float i_a, float i_b, float i_c);

	// getters

	float getV_a() const { return v_a_; }
	float getV_b() const { return v_b_; }
	float getV_c() const { return v_c_; }

	float getDutyA() const { return duty_a_; }
	float getDutyB() const { return duty_b_; }
	float getDutyC() const { return duty_c_; }

	float getIqRef() const { return i_q_ref_; }
	float getIq() const { return i_q_; }
	float getErrorIq() const { return error_q_; }
	float getVq() const { return v_q_; }

	~FOCController();
};

/*****************************************/
/*                  impl                 */
/*****************************************/

FOCController::FOCController(/* args */)
{
	// Initialize PI gains (to be tuned)
	pi_q_.setGains(PIController_Kp, PIController_Ki);
	pi_d_.setGains(PIController_Kp, PIController_Ki);

	// Setup low-pass filters (cutoff 1000 Hz)
	lpf_a_.setCutoff(lpf_cutoffHz, dt_);
	lpf_b_.setCutoff(lpf_cutoffHz, dt_);
	lpf_c_.setCutoff(lpf_cutoffHz, dt_);
}

FOCController::~FOCController()
{
}

void FOCController::setDt(float dt)
{
	// time step, delta time
	dt_ = dt;

	lpf_a_.setCutoff(lpf_cutoffHz, dt);
	lpf_b_.setCutoff(lpf_cutoffHz, dt);
	lpf_c_.setCutoff(lpf_cutoffHz, dt);
}

void FOCController::calibrateCurrents(float i_a, float i_b, float i_c)
{
	// Simple moving average or just assign for offset nulling
	// Important when using ADC reading, because of noise
	offset_ia_ = i_a;
	offset_ib_ = i_b;
	offset_ic_ = i_c;
	calibrated_ = true;
}

void FOCController::updateCurrents(float i_a, float i_b, float i_c)
{
	// Subtract offsets before filtering
	// Important when using ADC reading, because of noise
	i_a_ = lpf_a_.filter(i_a - offset_ia_);
	i_b_ = lpf_b_.filter(i_b - offset_ib_);
	i_c_ = lpf_c_.filter(i_c - offset_ic_);
}

void FOCController::updateAngle(float angle_rad)
{
	angle_rad_ = angle_rad;
}
void FOCController::setDesiredTorque(float i_q)
{
	i_q_ref_ = i_q;
}

void FOCController::run()
{

	// step 1: clarke transform
	// need the measurd phase currents

	float i_alpha = i_a_;
	float i_beta = (i_b_ - i_c_) * math_ONE_BY_SQRT3;

	// step 2: Park transform

	// need Hardware Optimization: compute sin/cos once
	// On real hardware, use sinf/cosf or a LUT
	float cos_theta = std::cos(angle_rad_);
	float sin_theta = std::sin(angle_rad_);

	i_q_ = -i_alpha * sin_theta + i_beta * cos_theta;
	i_d_ = i_alpha * cos_theta + i_beta * sin_theta;

	// step 3: PI
	// currents -> volts

	error_q_ = i_q_ref_ - i_q_;
	error_d_ = i_d_ref_ - i_d_;

	v_q_ = pi_q_.update(error_q_, dt_);
	v_d_ = pi_d_.update(error_d_, dt_);

	// vector clamping, mag and theta, more accurate

	float v_mag = sqrtf(v_q_ * v_q_ + v_d_ * v_d_);
	if (v_mag > v_max_)
	{
		float scale = v_max_ / v_mag;
		v_q_ *= scale;
		v_d_ *= scale;
	}

	// step 4: Inverse Park
	float v_alpha = v_d_ * cos_theta - v_q_ * sin_theta;
	float v_beta = v_d_ * sin_theta + v_q_ * cos_theta;

	// step 5: inverse Clarke
	v_a_ = v_alpha;
	v_b_ = -0.5f * v_alpha + math_SQRT3_BY_2 * v_beta;
	v_c_ = -0.5f * v_alpha - math_SQRT3_BY_2 * v_beta;

	// use SVPWM, better than SPWM

	// --- SVPWM (Min-Max Injection) ---
	// This shifts the neutral point to allow 15% more voltage utilization
	float v_min = std::fmin(v_a_, std::fmin(v_b_, v_c_));
	float v_max = std::fmax(v_a_, std::fmax(v_b_, v_c_));
	float v_offset = (v_min + v_max) * 0.5f;

	v_a_ -= v_offset;
	v_b_ -= v_offset;
	v_c_ -= v_offset;

	// Apply PWM (convert voltage to duty cycle)
	duty_a_ = v_a_ / dc_bus_voltage + 0.5f;
	duty_b_ = v_b_ / dc_bus_voltage + 0.5f;
	duty_c_ = v_c_ / dc_bus_voltage + 0.5f;

	// should make sure the duty in its safe range
	// ex: avoids 1.000001 value

	duty_a_ = std::clamp(duty_a_, 0.0f, 1.0f);
	duty_b_ = std::clamp(duty_b_, 0.0f, 1.0f);
	duty_c_ = std::clamp(duty_c_, 0.0f, 1.0f);
}

#endif