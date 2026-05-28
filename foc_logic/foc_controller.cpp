#ifndef FOC_CONTROLLER
#define FOC_CONTROLLER

#include "constants.hpp"
#include <cmath>
#include "pi_controller.cpp"

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

	// volts, get them from inverse transfoms then will be PWMs
	float v_a_ = 0;
	float v_b_ = 0;
	float v_c_ = 0;

	// dt
	float dt_ = 0;
	// PI
	PIController pi_q_{};
	PIController pi_d_{};

	// angle
	float angle_rad_ = 0;

	// BLDC motor
	// BLDCMotor motor;

public:
	FOCController(/* args */);

	void setCutOffFrq(float hz);

	void setDt(float dt);

	// run FOC algorithm
	void run();

	// to set the desired torque currents
	void setDesiredTorque(float i_q, float i_d = 0);

	// set current angle, should be from encoder
	// need in park transfom
	void updateAngle(float angle_rad);

	// set measured currents, read actual phase currents (from the inverter circuit)
	// need to a feedback system, clarke, park transfoms
	void updateCurrents(float i_a, float i_b, float i_c);

	~FOCController();
};

/*****************************************/
/*                  impl                 */
/*****************************************/

FOCController::FOCController(/* args */)
{
}

FOCController::~FOCController()
{
}

void FOCController::setDt(float dt)
{
	// time step, delta time
	dt_ = dt;
}
void FOCController::setCutOffFrq(float hz)
{
	// time step, delta time
	// dt_ = dt;
}

void FOCController::updateCurrents(float i_a, float i_b, float i_c)
{
	i_a_ = i_a;
	i_b_ = i_b;
	i_c_ = i_c;
}

void FOCController::updateAngle(float angle_rad)
{
	angle_rad_ = angle_rad;
}
void FOCController::setDesiredTorque(float i_q, float i_d = 0)
{
	i_q_ref_ = i_q;
	i_d_ref_ = i_d;
}

void FOCController::run()
{

	// step 1: clarke transform
	// need the measurd phase currents

	float i_alpha = i_a_;
	float i_beta = (i_b_ - i_c_) / 1.7320508f; // 1/√3

	// step 2: Park transform
	// need the motor angle

	float cos_theta = std::cos(angle_rad_);
	float sin_theta = std::sin(angle_rad_);

	float i_q = i_alpha * sin_theta + i_beta * cos_theta;
	float i_d = i_alpha * cos_theta - i_beta * sin_theta;

	// TODO: low pass filter

	// step 3: PI
	// currents -> volts

	float error_q = i_q_ref_ - i_q;
	float error_d = i_d_ref_ - i_d;

	float v_q = pi_q_.update(error_q, dt_);
	float v_d = pi_d_.update(error_d, dt_);

	// should make sure the volts in its safe range
	// TODO: need to read about it

	// step 4: Inverse Park
	float v_alpha = v_d * cos_theta - v_q * sin_theta;
	float v_beta = v_d * sin_theta + v_q * cos_theta;

	// step 5: inverse Clarke
	v_a_ = v_alpha;
	v_b_ = -0.5f * v_alpha + 0.8660254f * v_beta;
	v_c_ = -0.5f * v_alpha - 0.8660254f * v_beta;

	// should make sure the volts in its safe range
	// TODO: need to read about it

	// Apply PWM (convert voltage to duty cycle)
	// bipolar PWM, -dc to +dc

	float duty_a = (v_a_ / dc_bus_voltage + 1.0f) / 2.0f;
	float duty_b = (v_b_ / dc_bus_voltage + 1.0f) / 2.0f;
	float duty_c = (v_c_ / dc_bus_voltage + 1.0f) / 2.0f;

	// should make sure the duty in its safe range
	// ex: avoids 1.000001 value

	duty_a = clamp(duty_a, 0.0f, 1.0f);
	duty_b = clamp(duty_b, 0.0f, 1.0f);
	duty_c = clamp(duty_c, 0.0f, 1.0f);

	/////////// set PWM or log for simulation
	/////////// set PWM or log for simulation
	/////////// set PWM or log for simulation
}

/***************************************************/
/*                  priv func impl                 */
/***************************************************/

float clamp(float value, float min_val, float max_val)
{
	if (value < min_val)
		return min_val;
	if (value > max_val)
		return max_val;
	return value;
}

#endif