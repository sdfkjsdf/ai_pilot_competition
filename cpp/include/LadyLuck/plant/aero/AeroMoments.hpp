#pragma once

#include "LadyLuck/plant/aero/AeroLookup.hpp"

namespace LadyLuck
{
namespace plant
{
namespace aero
{

struct AeroInput
{
	double alpha_rad;
	double beta_rad;
	double mach;
	double qbar_pa;
	double aileron_rad;
	double elevator_rad;
	double rudder_rad;
	double p_rad_s;
	double q_rad_s;
	double r_rad_s;
	double true_airspeed_mps;

	bool has_flaperon_mix;
	double flaperon_mix_rad;
	bool has_lef;
	double lef_rad;
	bool has_speedbrake;
	double speedbrake_rad;
	bool has_gear;
	double gear_pos_norm;

	AeroInput() noexcept;
};

struct AeroForces
{
	double fx_n;
	double fy_n;
	double fz_n;
};

// Mirrors estimator_module2.plant.aero.aero_moments.AeroModel. Despite the
// historical Python filename, the active estimator path requests forces only.
class AeroModel
{
public:
	AeroForces Forces(const AeroInput& input) const noexcept;
	double FlaperonForceZ(const AeroInput& input) const noexcept;

	static double LefPosRad(
		double alpha_rad,
		double mach,
		double gear_pos_norm = 0.0) noexcept;

private:
	AeroProperties BuildProperties(const AeroInput& input) const noexcept;

	AeroLookup lookup_;
};

} // namespace aero
} // namespace plant
} // namespace LadyLuck
