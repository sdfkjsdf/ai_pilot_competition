#include "LadyLuck/plant/aero/AeroMoments.hpp"

#include <algorithm>
#include <cmath>

namespace LadyLuck
{
namespace plant
{
namespace aero
{
namespace
{

const double kFootM = 0.3048;
const double kPaToPsf = 1.0 / 47.880259;
const double kLbfToN = 4.4482216;
const double kWingAreaSqft = 300.0;
const double kWingSpanFt = 30.0;
const double kMeanChordFt = 11.32;

} // namespace

AeroInput::AeroInput() noexcept
	: alpha_rad(0.0),
	beta_rad(0.0),
	mach(0.0),
	qbar_pa(0.0),
	aileron_rad(0.0),
	elevator_rad(0.0),
	rudder_rad(0.0),
	p_rad_s(0.0),
	q_rad_s(0.0),
	r_rad_s(0.0),
	true_airspeed_mps(0.0),
	has_flaperon_mix(false),
	flaperon_mix_rad(0.0),
	has_lef(false),
	lef_rad(0.0),
	has_speedbrake(false),
	speedbrake_rad(0.0),
	has_gear(false),
	gear_pos_norm(0.0)
{
}

double AeroModel::LefPosRad(
	double alpha_rad,
	double mach,
	double gear_pos_norm) noexcept
{
	(void)gear_pos_norm;
	double lef = 0.0;
	if (mach > 0.9)
	{
		lef = -0.0349;
	}
	if (alpha_rad > 0.0873)
	{
		lef = 0.262;
	}
	if (alpha_rad > 0.2618)
	{
		lef = 0.436;
	}
	return lef;
}

AeroProperties AeroModel::BuildProperties(const AeroInput& input) const noexcept
{
	const double true_airspeed_fps = std::max(input.true_airspeed_mps / kFootM, 1.0e-3);
	AeroProperties properties;
	properties.qbar_psf = input.qbar_pa * kPaToPsf;
	properties.wing_area_sqft = kWingAreaSqft;
	properties.alpha_rad = input.alpha_rad;
	properties.beta_rad = input.beta_rad;
	properties.mach = input.mach;
	properties.elevator_rad = input.elevator_rad;
	properties.aileron_rad = input.aileron_rad;
	properties.rudder_rad = input.rudder_rad;
	properties.p_aero_rad_s = input.p_rad_s;
	properties.q_aero_rad_s = input.q_rad_s;
	properties.r_aero_rad_s = input.r_rad_s;
	properties.bi2vel = kWingSpanFt / (2.0 * true_airspeed_fps);
	properties.ci2vel = kMeanChordFt / (2.0 * true_airspeed_fps);
	properties.kcl_ground_effect = 1.0;
	properties.lef_rad = input.has_lef
		? input.lef_rad
		: LefPosRad(input.alpha_rad, input.mach);
	properties.flaperon_mix_rad = input.has_flaperon_mix ? input.flaperon_mix_rad : 0.0;
	properties.speedbrake_rad = input.has_speedbrake ? input.speedbrake_rad : 0.0;
	properties.gear_pos_norm = input.has_gear ? input.gear_pos_norm : 0.0;
	return properties;
}

AeroForces AeroModel::Forces(const AeroInput& input) const noexcept
{
	const AeroProperties properties = BuildProperties(input);
	const AeroAxisSums sums = lookup_.AxisSums(properties);
	const double cosine_alpha = std::cos(properties.alpha_rad);
	const double sine_alpha = std::sin(properties.alpha_rad);
	const double cosine_beta = std::cos(properties.beta_rad);
	const double sine_beta = std::sin(properties.beta_rad);
	AeroForces result;
	result.fx_n = (
		-sums.drag * cosine_alpha * cosine_beta
		- sums.side * cosine_alpha * sine_beta
		+ sums.lift * sine_alpha) * kLbfToN;
	result.fy_n = (-sums.drag * sine_beta + sums.side * cosine_beta) * kLbfToN;
	result.fz_n = (
		-sums.drag * sine_alpha * cosine_beta
		- sums.side * sine_alpha * sine_beta
		- sums.lift * cosine_alpha) * kLbfToN;
	return result;
}

double AeroModel::FlaperonForceZ(const AeroInput& input) const noexcept
{
	const AeroProperties properties = BuildProperties(input);
	double drag_flaperon = 0.0;
	double lift_flaperon = 0.0;
	const bool drag_found = lookup_.TryEvalFunction("CDDflaps", properties, drag_flaperon);
	const bool lift_found = lookup_.TryEvalFunction("CLDflaps", properties, lift_flaperon);
	if (!drag_found || !lift_found)
	{
		return 0.0;
	}
	const double cosine_alpha = std::cos(properties.alpha_rad);
	const double sine_alpha = std::sin(properties.alpha_rad);
	const double cosine_beta = std::cos(properties.beta_rad);
	return (
		-drag_flaperon * sine_alpha * cosine_beta
		- lift_flaperon * cosine_alpha) * kLbfToN;
}

} // namespace aero
} // namespace plant
} // namespace LadyLuck
