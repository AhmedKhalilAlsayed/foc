#ifndef constants_h
#define constants_h

// math
const float math_PI = 3.14159265359f;
const float math_TWO_PI = 2.0f * math_PI;

// elec
const float dc_bus_voltage = 24.0f;

// low pass filter
const float lpf_cutoffHz = 1000.0f;

// PI controller
// TODO: should learn more about these
const float PIController_Kp = 6.0f;
const float PIController_Ki = 3000.0f;
const float PIController_integral_max = 12.0f; // Allow up to half Vdc for the integral term

#endif