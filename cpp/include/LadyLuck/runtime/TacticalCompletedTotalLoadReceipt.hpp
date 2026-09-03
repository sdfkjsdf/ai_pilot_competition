#pragma once

#include "LadyLuck/contracts/FrameContext.hpp"
#include "LadyLuck/contracts/Status.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace runtime
{

// Exact source identity carried by d90 ControlFeedbackSample.total_load_source.
// A source enum replaces the Python diagnostic string without collapsing the
// three numerically different controller paths.
enum class TacticalCompletedTotalLoadSource : std::uint8_t
{
    Unavailable = 0U,
    Route5NCommand = 1U,
    DirectH09AerodynamicTargetVector = 2U,
    DirectNedVelocityNormalForce = 3U
};

// One accepted controller frame's raw load magnitude, governed magnitude and
// physical envelope limit.  This is controller output evidence, not measured
// aircraft load and not simulator truth.
struct TacticalCompletedTotalLoadReceipt
{
    bool valid = false;
    ControlFrameIdentity source_frame_identity{};
    double raw_total_load_g = 0.0;
    double governed_total_load_g = 0.0;
    double physical_limit_g = 0.0;
    TacticalCompletedTotalLoadSource source =
        TacticalCompletedTotalLoadSource::Unavailable;
};

class TacticalCompletedTotalLoadReceiptBuilder final
{
public:
    TacticalCompletedTotalLoadReceiptBuilder() noexcept = default;

    void Build(
        const ControlFrameIdentity& source_frame_identity,
        double raw_total_load_g,
        double governed_total_load_g,
        double physical_limit_g,
        TacticalCompletedTotalLoadSource source,
        TacticalCompletedTotalLoadReceipt& output,
        Status& status) const noexcept;
};

static_assert(
    std::is_trivially_copyable<TacticalCompletedTotalLoadReceipt>::value,
    "Completed total-load evidence must remain allocation-free.");

} // namespace runtime
} // namespace LadyLuck
