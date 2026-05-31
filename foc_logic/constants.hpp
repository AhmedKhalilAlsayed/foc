#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// math
constexpr float math_PI = 3.14159265359f;
constexpr float math_TWO_PI = 2.0f * math_PI;
constexpr float math_SQRT3 = 1.73205080757f;
constexpr float math_ONE_BY_SQRT3 = 0.57735026919f;
constexpr float math_SQRT3_BY_2 = 0.86602540378f;

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