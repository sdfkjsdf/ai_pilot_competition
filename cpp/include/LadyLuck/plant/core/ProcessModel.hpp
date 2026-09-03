#pragma once

#include "LadyLuck/plant/aero/AeroMoments.hpp"
#include "LadyLuck/plant/core/AuxiliaryState.hpp"
#include "LadyLuck/plant/types/PlantState.hpp"
#include "LadyLuck/plant/types/PlantStatus.hpp"

#include <array>

namespace LadyLuck
{
namespace plant
{
namespace core
{

struct IsaState
{
    double density_kg_m3 = 0.0;
    double speed_of_sound_mps = 0.0;
};

struct DerivedAirData
{
    double true_airspeed_mps = 0.0;
    double alpha_rad = 0.0;
    double beta_rad = 0.0;
};

// Python's omega_aero_radps=None is a real interface state: FBW consumes the
// interval body rate stored in PlantState, and the aerodynamic model falls
// back to that same rate.  A zero-filled std::array cannot distinguish an
// omitted endpoint estimate from a valid endpoint estimate of [0, 0, 0].
//
// Keep the value and its presence bit in one type so a caller cannot assign an
// endpoint vector while forgetting to mark it present.  This is the C++14
// equivalent of a narrowly scoped optional<std::array<double, 3>>.
class OptionalBodyRates
{
public:
    OptionalBodyRates() noexcept = default;

    explicit OptionalBodyRates(
        const std::array<double, 3>& rates_rad_s) noexcept
    {
        Set(rates_rad_s);
    }

    OptionalBodyRates& operator=(
        const std::array<double, 3>& rates_rad_s) noexcept
    {
        Set(rates_rad_s);
        return *this;
    }

    void Set(const std::array<double, 3>& rates_rad_s) noexcept
    {
        rates_rad_s_ = rates_rad_s;
        has_value_ = true;
    }

    void Reset() noexcept
    {
        rates_rad_s_ = {{0.0, 0.0, 0.0}};
        has_value_ = false;
    }

    bool has_value() const noexcept
    {
        return has_value_;
    }

    const std::array<double, 3>* Get() const noexcept
    {
        return has_value_ ? &rates_rad_s_ : nullptr;
    }

private:
    bool has_value_ = false;
    std::array<double, 3> rates_rad_s_{{0.0, 0.0, 0.0}};
};

struct ProcessStepInput
{
    PlantState state{};
    AuxState auxiliary{};
    std::array<double, 4> command{{0.0, 0.0, 0.0, 0.0}};
    double thrust_n = 0.0;
    double dt_s = 0.0;
    double gear_position_normalized = 0.0;
    double pitch_trim = 0.0;
    double yaw_trim = 0.0;
    double fuel_flow_lb_s = 0.0;
    OptionalBodyRates omega_aero_rad_s{};
};

struct ProcessStepOutput
{
    AuxState next_auxiliary{};
    double force_z_n = 0.0;
    double force_y_n = 0.0;
    double mass_kg = 0.0;
    double elevator_rad = 0.0;
    double flaperon_force_z_n = 0.0;
};

class ProcessModel
{
public:
    ProcessModel() noexcept;

    AuxState InitialAuxiliaryState(double mass_kg) const noexcept;
    PlantResult<DerivedAirData> Derived(
        const PlantState& state) const noexcept;
    PlantResult<ProcessStepOutput> StepEstimateModern(
        const ProcessStepInput& input) const noexcept;

    static PlantResult<IsaState> Isa(double altitude_m) noexcept;

private:
    AuxiliaryStateModel auxiliary_;
    aero::AeroModel aero_;
};

} // namespace core
} // namespace plant
} // namespace LadyLuck
