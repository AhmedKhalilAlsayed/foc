#include <iostream>
#include <fstream>
#include <cmath>
#include "foc_logic/foc_controller.cpp"
#include "HAL/motor_model.cpp"

int main()
{
	/*
		// step 1: clarke transform
		// need the measurd phase currents

		float i_alpha = 1; // i_a
		float i_beta = (-0.5f - -0.5f) / 1.7320508f; // i_b - i_c

		// step 2: Park transform
		// need the motor angle

		float cos_theta = std::cos(0);
		float sin_theta = std::sin(0);

		std::cout << (-i_alpha * sin_theta + i_beta * cos_theta) << std::endl;

		// i_d_ = i_alpha * cos_theta - i_beta * sin_theta;
	*/

	// Simulation parameters
	const float DT = 0.00005f;				// 50 us (20 kHz)
	const float SIMULATION_TIME = 3.0f;		// seconds
	const int STEPS = SIMULATION_TIME / DT; // how many deltas
	const int LOG_EVERY = 1000;				// log every 1000 steps (50 ms)

	// Create controller and motor
	FOCController foc{};
	MotorModel motor{};

	foc.setDt(DT);
	motor.setDt(DT);

	// Set initial conditions
	motor.reset(0.0f, 0.0f);	// start at angle 0, speed 0
	motor.setLoadTorque(0.05f); // small constant load (Nm)

	// Open CSV file for logging
	std::ofstream log("foc_simulation.csv");
	log << "time_ms,theta_rad,omega_radps,i_a,i_b,i_c,i_q_ref,error_q,v_q\n";
	// log << "time_ms,i_q_ref,v_q\n";

	// Main simulation loop
	for (int step = 0; step < STEPS; ++step)
	{
		float time_sec = step * DT;

		// ----- Command torque profile (example) -----
		float torque_cmd = 0.0f;
		if (time_sec < 0.5f)
		{
			torque_cmd = 0.0f; // coast
		}
		else if (time_sec < 1.0f)
		{
			torque_cmd = 1.0f; // positive torque (1 A)
		}
		else if (time_sec < 1.5f)
		{
			torque_cmd = -0.8f; // braking torque
		}
		else
		{
			torque_cmd = 0.0f; // coast again
		}
		foc.setDesiredTorque(torque_cmd);

		// ----- Update FOC with measured feedback (from motor model) -----
		float angle = motor.getAngle();
		float i_a, i_b, i_c;
		motor.getCurrents(i_a, i_b, i_c);

		foc.updateAngle(angle);
		foc.updateCurrents(i_a, i_b, i_c);
		// Run FOC calculation (produces v_a, v_b, v_c internally)
		foc.run();

		// Apply voltages to motor model
		// motor drive
		motor.step(foc.getV_a(), foc.getV_b(), foc.getV_c());

		// ----- Logging -----
		if (step % LOG_EVERY == 0)
		{
			log << time_sec * 1000.0 << ","
				<< angle << ","
				<< motor.getSpeed() << ","
				<< i_a << ","
				<< i_b << ","
				<< i_c << ","
				<< foc.getIqRef() << ","
				<< foc.getErrorIq() << ","
				<< foc.getVq()
				<< "\n";
		}
	}

	log.close();
	std::cout << "Simulation finished. Data written to foc_simulation.csv\n";

	return 0;
}
