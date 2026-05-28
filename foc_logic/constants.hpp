#ifndef constants_h
#define constants_h

// math
const float math_PI = 3.14159265359f;
const float math_TWO_PI = 2.0f * math_PI;

// elec
const float dc_bus_voltage = 5.0f;

// low pass filter
const float lpf_cutoffHz = 1000.0f;

// PI controller
const float PIController_Kp = 0.01f;
const float PIController_Ki = 0.1f;
const float PIController_integral_max = 1.0f;

#endif