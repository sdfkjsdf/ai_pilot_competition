#pragma once

#include <cstddef>

namespace LadyLuck
{
namespace plant
{
namespace aero
{

enum class AeroAxis
{
	Drag,
	Side,
	Lift
};

// Scalar property vector consumed by the frozen f16.xml aerodynamic functions.
// Units follow the upstream metadata names exactly (psf, ft, radians, seconds).
struct AeroProperties
{
	double qbar_psf;
	double wing_area_sqft;
	double alpha_rad;
	double beta_rad;
	double mach;
	double elevator_rad;
	double aileron_rad;
	double rudder_rad;
	double p_aero_rad_s;
	double q_aero_rad_s;
	double r_aero_rad_s;
	double bi2vel;
	double ci2vel;
	double kcl_ground_effect;
	double lef_rad;
	double flaperon_mix_rad;
	double speedbrake_rad;
	double gear_pos_norm;

	AeroProperties() noexcept;
};

struct AeroAxisSums
{
	double drag;
	double side;
	double lift;

	AeroAxisSums() noexcept;
};

// Runtime-free evaluator for the active DRAG/SIDE/LIFT closure of the frozen
// aero_meta.json graph and aero_tables.npz arrays. No JSON, NPZ, Python, heap,
// or filesystem access is performed by this class.
class AeroLookup
{
public:
	// Returns false and writes zero for a null, unknown, or intentionally pruned
	// function key. The active path has no throwing lookup API.
	bool TryEvalFunction(
		const char* key,
		const AeroProperties& properties,
		double& value) const noexcept;

	AeroAxisSums AxisSums(const AeroProperties& properties) const noexcept;

	static std::size_t TableCount() noexcept;
	static std::size_t FunctionCount() noexcept;
	static const char* AeroMetaSha256() noexcept;
	static const char* AeroTablesSha256() noexcept;
	static const char* UpstreamCommit() noexcept;
};

} // namespace aero
} // namespace plant
} // namespace LadyLuck
