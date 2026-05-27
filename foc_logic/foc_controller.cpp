#include "constants.hpp"
#include "BLDC_motor.cpp"

class FOCController
{
private:
	// desired torque currents
	float i_q_ref_ = 0;
	float i_d_ref_ = 0; // should be 0, max torque

	// measured torque currents
	float i_q_measure_ = 0;
	float i_d_measure_ = 0;

	// measured phase currents
	float i_a_ = 0;
	float i_b_ = 0;
	float i_c_ = 0;

	// volts, get them from inverse transfoms then will be PWMs
	float v_a_ = 0;
	float v_b_ = 0;
	float v_c_ = 0;

	// angle
	float angle_rad_ = 0;

	// BLDC motor
	// BLDCMotor motor;

public:
	FOCController(/* args */);

	// run FOC algorithm
	void run();

	// to set the desired torque currents
	void setDesiredTorque(float i_q, float i_d = 0);

	// set current angle, should be from encoder
	// need in park transfom
	void updateAngle(float angle_rad);

	// set measured currents, read actual phase currents
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
	/*PI controller*/

	/*IPark, IClarke*/

	/*PWMs*/

	/*Clarke, Park*/

	/*low pass-filter*/
}