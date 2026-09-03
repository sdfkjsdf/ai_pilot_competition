#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/control/route5/CommandEnvelope.hpp"
#include "LadyLuck/control/route5/Route5Guidance.hpp"
#include "LadyLuck/control/tecs_cis/TecsCisControl.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace runtime
{

namespace current_cis_v4_projection_math
{

// Total finite arithmetic used by the production current-CIS projection.
// Every false result leaves +0.0 and performs no overflowing operation.  The
// named functions are exposed only so the focused boundary runner can call
// the exact production implementation under the floating-point status flags.
bool SafeAdd(double left, double right, double& output) noexcept;
bool SafeSubtract(double left, double right, double& output) noexcept;
bool SafeMultiply(double left, double right, double& output) noexcept;
bool SafeDivide(double numerator, double denominator, double& output) noexcept;

} // namespace current_cis_v4_projection_math

// Typed result of the exact add/main@45abc current-sample SPACING energy
// projection.  It describes raw guidance and the TECS/CIS-admitted explicit
// energy bias; it is not an aircraft-response or thrust-tracking receipt.
enum class CurrentCisV4EnergyProjectionReason : std::uint8_t
{
    NotEvaluated = 0U,
    WithinCurrentAuthority = 1U,
    LowerReferenceProjected = 2U,
    UpperReferenceProjected = 3U,
    InvalidRawIntent = 4U,
    FrameIdentityMismatch = 5U,
    UnsupportedRoute = 6U,
    DirectEnergyBiasPresent = 7U,
    RawRoutePreviewUnavailable = 8U,
    ZeroBiasRoutePreviewUnavailable = 9U,
    RawTecsPreviewUnavailable = 10U,
    ZeroBiasTecsPreviewUnavailable = 11U,
    RawControlUnbounded = 12U,
    MeasuredEnergyRateUnavailable = 13U,
    AuthorityInputsUnavailable = 14U,
    AuthorityArithmeticUnavailable = 15U,
    SaturatedReferenceNotLimited = 16U,
    BoundaryNotForwardRepresentable = 17U,
    InverseForwardClosureExceeded = 18U,
    ProjectionRequiresPositiveBias = 19U,
    ProjectedRoutePreviewUnavailable = 20U,
    ProjectedTecsPreviewUnavailable = 21U,
    ProjectedControlUnbounded = 22U,
    ProjectedCommandStillSaturated = 23U,
    NonEnergyFieldMutation = 24U,
    LiveControllerStateMutation = 25U,
    ReferenceEquationMismatch = 26U
};

const char* CurrentCisV4EnergyProjectionReasonLabel(
    CurrentCisV4EnergyProjectionReason reason) noexcept;

struct CurrentCisV4EnergyProjectionReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool admitted = false;
    bool contract_fault = false;
    CurrentCisV4EnergyProjectionReason reason =
        CurrentCisV4EnergyProjectionReason::NotEvaluated;

    bool all_nonenergy_fields_unchanged = false;
    bool raw_control_bounded = false;
    bool projected_control_bounded = false;
    bool raw_lower_saturated = false;
    bool raw_upper_saturated = false;
    bool projected_lower_saturated = false;
    bool projected_upper_saturated = false;
    bool authority_available = false;
    std::uint32_t closure_ulp_steps = 0U;

    double raw_bias_m2ps3 = 0.0;
    double admitted_bias_m2ps3 = 0.0;
    double base_reference_m2ps3 = 0.0;
    double raw_reference_m2ps3 = 0.0;
    // `admitted_reference` is the exact inverse/forward boundary target;
    // `projected_reference` is the TECS reference actually reconstructed from
    // base + admitted_bias.  They can differ by representable roundoff.
    double admitted_reference_m2ps3 = 0.0;
    double projected_reference_m2ps3 = 0.0;

    double energy_error_gain_per_s = 0.0;
    double energy_integral_gain_per_s2 = 0.0;
    double energy_rate_feedback_gain = 0.0;
    double total_energy_error_m2ps2 = 0.0;
    double energy_integral_before_m2ps = 0.0;
    double specific_energy_rate_measured_m2ps3 = 0.0;
    double speed_for_inverse_mps = 0.0;
    double mass_kg = 0.0;
    double drag_estimate_n = 0.0;
    double thrust_velocity_projection = 0.0;
    double thrust_min_n = 0.0;
    double thrust_max_n = 0.0;
    double reference_min_m2ps3 = 0.0;
    double reference_max_m2ps3 = 0.0;
    double raw_thrust_cmd_n = 0.0;
    double projected_thrust_cmd_n = 0.0;
    // Same-current values already produced by the Route-5/TECS preview.  H09
    // consumes them only after the ordinary projection is admitted; exposing
    // them here adds no controller, estimator, or tactical authority.
    bool raw_rate_measurement_valid = false;
    double raw_specific_energy_rate_measured_m2ps3 = 0.0;
    double raw_thrust_cmd_limited_n = 0.0;
    double projected_thrust_cmd_limited_n = 0.0;
    double route_k_gamma_per_s = 0.0;
    double route_gamma_rate_limit_radps = 0.0;
};

class ICurrentCisV4EnergyProjectionPort
{
public:
    virtual ~ICurrentCisV4EnergyProjectionPort() noexcept = default;

    // The port has stack lifetime for exactly one accepted estimator frame.
    // output is copied from raw and changes only its explicit energy bias on
    // admission.  Every non-admission is typed and returns StatusCode::Ok so
    // the provider can publish its already-validated base fallback.
    virtual void Project(
        const ControlIntent& raw,
        ControlIntent& output,
        CurrentCisV4EnergyProjectionReceipt& receipt,
        Status& status) const noexcept = 0;
};

// Synchronous adapter over the current pre-Step Route-5 and TECS/CIS owners.
// It stores references only for its caller-owned stack lifetime and is never
// placed in TacticalCommandBuildInput or retained by the control core.
class CurrentCisV4EnergyProjectionPort final
    : public ICurrentCisV4EnergyProjectionPort
{
public:
    CurrentCisV4EnergyProjectionPort(
        const control::route5::Route5Guidance& route5,
        const control::tecs_cis::TecsCisControl& tecs_cis,
        const PlaneState& ownship,
        const EstimatorOutputV6& estimate,
        const control::route5::CommandEnvelope& envelope,
        double dt_s,
        bool gcas_energy_handoff_active,
        bool previous_transmitted_auto_gcas_active) noexcept;

    void Project(
        const ControlIntent& raw,
        ControlIntent& output,
        CurrentCisV4EnergyProjectionReceipt& receipt,
        Status& status) const noexcept override;

private:
    const control::route5::Route5Guidance& route5_;
    const control::tecs_cis::TecsCisControl& tecs_cis_;
    const PlaneState& ownship_;
    const EstimatorOutputV6& estimate_;
    const control::route5::CommandEnvelope& envelope_;
    double dt_s_ = 0.0;
    bool gcas_energy_handoff_active_ = false;
    bool previous_transmitted_auto_gcas_active_ = false;
};

static_assert(
    std::is_trivially_copyable<
        CurrentCisV4EnergyProjectionReceipt>::value,
    "current-CIS projection receipt must remain allocation-free");

} // namespace runtime
} // namespace LadyLuck
