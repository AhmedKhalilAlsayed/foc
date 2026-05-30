#ifndef MOTOR_MODEL
#define MOTOR_MODEL

// #pragma once
#include <cmath>

class MotorModel
{
private:
	// Electrical parameters
	float R = 0.5f;	  // Phase resistance (Ohms)
	float L = 0.001f; // Phase inductance (Henries)
	float Ke = 0.1f;  // Back-EMF constant (V/(rad/s)) also torque constant (Nm/A)

	// Mechanical parameters
	float J = 0.001f;		  // Rotor inertia (kg·m²)
	float B = 0.0001f;		  // Damping coefficient (Nm·s/rad)
	float load_torque = 0.0f; // External load torque (Nm)

	// State variables
	float theta = 0.0f; // Rotor angle (rad)
	float omega = 0.0f; // Rotor speed (rad/s)

	// Electrical state (currents)
	float i_a = 0.0f, i_b = 0.0f, i_c = 0.0f;

	float dt = 0.00005f; // Simulation timestep (50 us)

public:
	void setDt(float dt_sample) { dt = dt_sample; }
	void setLoadTorque(float Tl) { load_torque = Tl; }

	// Apply three-phase voltages (V) and update state one step
	void step(float v_a, float v_b, float v_c);

	// Getters for feedback to FOC controller
	float getAngle() const { return theta; }
	float getSpeed() const { return omega; }
	void getCurrents(float &a, float &b, float &c) const
	{
		a = i_a;
		b = i_b;
		c = i_c;
	}

	// Optional: reset state
	void reset(float theta0 = 0.0f, float omega0 = 0.0f);
};

void MotorModel::step(float v_a, float v_b, float v_c)
{
	// 1. Back-EMF voltages (simplified: sinusoidal, phase shifted by 120°)
	float emf_a = Ke * omega * std::sin(theta);
	float emf_b = Ke * omega * std::sin(theta - 2.0f * M_PI / 3.0f);
	float emf_c = Ke * omega * std::sin(theta - 4.0f * M_PI / 3.0f);

	// 2. Account for floating neutral point (Vn)
	// In a Y-connected motor, Vn = (Va + Vb + Vc - (emfa + emfb + emfc)) / 3
	// Since emfa+emfb+emfc = 0 in a balanced motor:
	float v_neutral = (v_a + v_b + v_c) / 3.0f;

	// 3. Voltage equation: v_phase - v_neutral = R*i + L*di/dt + emf
	//    => di/dt = (v_terminal - v_neutral - R*i - emf) / L
	float di_a_dt = (v_a - v_neutral - R * i_a - emf_a) / L;
	float di_b_dt = (v_b - v_neutral - R * i_b - emf_b) / L;
	float di_c_dt = (v_c - v_neutral - R * i_c - emf_c) / L;

	// 4. Euler integration
	i_a += di_a_dt * dt;
	i_b += di_b_dt * dt;
	i_c += di_c_dt * dt;

	// 4. Electromagnetic torque: Te = Ke * i_q (assuming i_d=0)
	//    For simplicity, compute i_q via Clarke+Park (or use Ke * (some current))
	//    We'll compute using measured currents and angle.
	float i_alpha = i_a;
	float i_beta = (i_b - i_c) / 1.7320508f;
	float cos_theta = std::cos(theta);
	float sin_theta = std::sin(theta);
	float i_q = -i_alpha * sin_theta + i_beta * cos_theta;
	float Te = Ke * i_q; // Nm

	// 5. Mechanical equation: J * dω/dt = Te - B*ω - T_load
	float domega_dt = (Te - B * omega - load_torque) / J;
	omega += domega_dt * dt;

	// 6. Update angle
	theta += omega * dt;

	// Keep theta in [0, 2π)
	if (theta >= 2.0f * M_PI)
		theta -= 2.0f * M_PI;
	if (theta < 0.0f)
		theta += 2.0f * M_PI;
}

void MotorModel::reset(float theta0, float omega0)
{
	theta = theta0;
	omega = omega0;
	i_a = i_b = i_c = 0.0f;
}

#endif