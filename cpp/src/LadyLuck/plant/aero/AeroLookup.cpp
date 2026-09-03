#include "LadyLuck/plant/aero/AeroLookup.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace LadyLuck
{
namespace plant
{
namespace aero
{
namespace
{

template <typename T, std::size_t N>
constexpr std::size_t ArrayCount(const T (&)[N]) noexcept
{
	return N;
}

enum class PropertyId : unsigned short
{
	QbarPsf,
	WingAreaSqft,
	AlphaRad,
	BetaRad,
	Mach,
	ElevatorRad,
	AileronRad,
	RudderRad,
	PAeroRadS,
	QAeroRadS,
	RAeroRadS,
	Bi2Vel,
	Ci2Vel,
	KclGroundEffect,
	LefRad,
	FlaperonMixRad,
	SpeedbrakeRad,
	GearPosNorm,
	Count
};

enum class FactorKind : unsigned char
{
	Property,
	Constant,
	Table
};

struct Factor
{
	FactorKind kind;
	unsigned short index;
	double constant;
};

struct StaticTable
{
	const char* key;
	int dimensions;
	PropertyId row_property;
	PropertyId column_property;
	const double* row;
	std::size_t row_count;
	const double* column;
	std::size_t column_count;
	const double* data;
};

struct StaticFunction
{
	const char* key;
	AeroAxis axis;
	const Factor* factors;
	std::size_t factor_count;
};

#include "AeroStaticTables.generated.inc"

double GetProperty(const AeroProperties& properties, PropertyId id) noexcept
{
	switch (id)
	{
	case PropertyId::QbarPsf: return properties.qbar_psf;
	case PropertyId::WingAreaSqft: return properties.wing_area_sqft;
	case PropertyId::AlphaRad: return properties.alpha_rad;
	case PropertyId::BetaRad: return properties.beta_rad;
	case PropertyId::Mach: return properties.mach;
	case PropertyId::ElevatorRad: return properties.elevator_rad;
	case PropertyId::AileronRad: return properties.aileron_rad;
	case PropertyId::RudderRad: return properties.rudder_rad;
	case PropertyId::PAeroRadS: return properties.p_aero_rad_s;
	case PropertyId::QAeroRadS: return properties.q_aero_rad_s;
	case PropertyId::RAeroRadS: return properties.r_aero_rad_s;
	case PropertyId::Bi2Vel: return properties.bi2vel;
	case PropertyId::Ci2Vel: return properties.ci2vel;
	case PropertyId::KclGroundEffect: return properties.kcl_ground_effect;
	case PropertyId::LefRad: return properties.lef_rad;
	case PropertyId::FlaperonMixRad: return properties.flaperon_mix_rad;
	case PropertyId::SpeedbrakeRad: return properties.speedbrake_rad;
	case PropertyId::GearPosNorm: return properties.gear_pos_norm;
	case PropertyId::Count: break;
	}
	return 0.0;
}

std::size_t ClampedCell(const double* axis, std::size_t count, double value) noexcept
{
	const double* upper = std::lower_bound(axis, axis + count, value);
	if (upper == axis)
	{
		return 0;
	}
	if (upper == axis + count)
	{
		return count - 2;
	}
	return static_cast<std::size_t>(upper - axis - 1);
}

double Interp1D(const StaticTable& table, double x) noexcept
{
	if (std::isnan(x))
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	if (x <= table.row[0])
	{
		return table.data[0];
	}
	if (x >= table.row[table.row_count - 1])
	{
		return table.data[table.row_count - 1];
	}
	const double* upper = std::lower_bound(table.row, table.row + table.row_count, x);
	const std::size_t high = static_cast<std::size_t>(upper - table.row);
	if (*upper == x)
	{
		return table.data[high];
	}
	const std::size_t low = high - 1;
	const double fraction = (x - table.row[low]) / (table.row[high] - table.row[low]);
	return table.data[low] + (table.data[high] - table.data[low]) * fraction;
}

double Interp2D(const StaticTable& table, double x, double y) noexcept
{
	if (std::isnan(x) || std::isnan(y))
	{
		return std::numeric_limits<double>::quiet_NaN();
	}
	const double xc = std::max(table.row[0], std::min(table.row[table.row_count - 1], x));
	const double yc = std::max(
		table.column[0],
		std::min(table.column[table.column_count - 1], y));
	const std::size_t i = ClampedCell(table.row, table.row_count, xc);
	const std::size_t j = ClampedCell(table.column, table.column_count, yc);
	const double x0 = table.row[i];
	const double x1 = table.row[i + 1];
	const double y0 = table.column[j];
	const double y1 = table.column[j + 1];
	const double tx = x1 > x0 ? (xc - x0) / (x1 - x0) : 0.0;
	const double ty = y1 > y0 ? (yc - y0) / (y1 - y0) : 0.0;
	const std::size_t stride = table.column_count;
	const double d00 = table.data[i * stride + j];
	const double d10 = table.data[(i + 1) * stride + j];
	const double d01 = table.data[i * stride + j + 1];
	const double d11 = table.data[(i + 1) * stride + j + 1];
	return d00 * (1.0 - tx) * (1.0 - ty)
		+ d10 * tx * (1.0 - ty)
		+ d01 * (1.0 - tx) * ty
		+ d11 * tx * ty;
}

double EvalTable(const StaticTable& table, const AeroProperties& properties) noexcept
{
	const double row_value = GetProperty(properties, table.row_property);
	if (table.dimensions == 1)
	{
		return Interp1D(table, row_value);
	}
	return Interp2D(
		table,
		row_value,
		GetProperty(properties, table.column_property));
}

double EvalStaticFunction(
	const StaticFunction& function,
	const AeroProperties& properties) noexcept
{
	double result = 1.0;
	for (std::size_t index = 0; index < function.factor_count; ++index)
	{
		const Factor& factor = function.factors[index];
		switch (factor.kind)
		{
		case FactorKind::Property:
			result *= GetProperty(properties, static_cast<PropertyId>(factor.index));
			break;
		case FactorKind::Constant:
			result *= factor.constant;
			break;
		case FactorKind::Table:
			result *= EvalTable(kTables[factor.index], properties);
			break;
		}
	}
	return result;
}

void AddAxisValue(AeroAxisSums& sums, AeroAxis axis, double value) noexcept
{
	switch (axis)
	{
	case AeroAxis::Drag: sums.drag += value; break;
	case AeroAxis::Side: sums.side += value; break;
	case AeroAxis::Lift: sums.lift += value; break;
	}
}

} // namespace

AeroProperties::AeroProperties() noexcept
	: qbar_psf(0.0),
	wing_area_sqft(0.0),
	alpha_rad(0.0),
	beta_rad(0.0),
	mach(0.0),
	elevator_rad(0.0),
	aileron_rad(0.0),
	rudder_rad(0.0),
	p_aero_rad_s(0.0),
	q_aero_rad_s(0.0),
	r_aero_rad_s(0.0),
	bi2vel(0.0),
	ci2vel(0.0),
	kcl_ground_effect(0.0),
	lef_rad(0.0),
	flaperon_mix_rad(0.0),
	speedbrake_rad(0.0),
	gear_pos_norm(0.0)
{
}

AeroAxisSums::AeroAxisSums() noexcept
	: drag(0.0), side(0.0), lift(0.0)
{
}

bool AeroLookup::TryEvalFunction(
	const char* key,
	const AeroProperties& properties,
	double& value) const noexcept
{
	value = 0.0;
	if (key == nullptr)
	{
		return false;
	}
	for (std::size_t index = 0; index < ArrayCount(kFunctions); ++index)
	{
		if (std::strcmp(kFunctions[index].key, key) == 0)
		{
			value = EvalStaticFunction(kFunctions[index], properties);
			return true;
		}
	}
	return false;
}

AeroAxisSums AeroLookup::AxisSums(const AeroProperties& properties) const noexcept
{
	AeroAxisSums sums;
	for (std::size_t index = 0; index < ArrayCount(kFunctions); ++index)
	{
		AddAxisValue(
			sums,
			kFunctions[index].axis,
			EvalStaticFunction(kFunctions[index], properties));
	}
	return sums;
}

std::size_t AeroLookup::TableCount() noexcept
{
	return ArrayCount(kTables);
}

std::size_t AeroLookup::FunctionCount() noexcept
{
	return ArrayCount(kFunctions);
}

const char* AeroLookup::AeroMetaSha256() noexcept
{
	return kAeroMetaSha256;
}

const char* AeroLookup::AeroTablesSha256() noexcept
{
	return kAeroTablesSha256;
}

const char* AeroLookup::UpstreamCommit() noexcept
{
	return kUpstreamCommit;
}

} // namespace aero
} // namespace plant
} // namespace LadyLuck
