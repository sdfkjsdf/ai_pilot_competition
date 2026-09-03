#include "LadyLuck/runtime/TacticalCompletedTotalLoadReceipt.hpp"

#include <cmath>

namespace
{

bool ValidSource(
    const LadyLuck::runtime::TacticalCompletedTotalLoadSource source) noexcept
{
    using LadyLuck::runtime::TacticalCompletedTotalLoadSource;
    switch (source)
    {
    case TacticalCompletedTotalLoadSource::Route5NCommand:
    case TacticalCompletedTotalLoadSource::DirectH09AerodynamicTargetVector:
    case TacticalCompletedTotalLoadSource::DirectNedVelocityNormalForce:
        return true;
    case TacticalCompletedTotalLoadSource::Unavailable:
    default:
        return false;
    }
}

} // namespace

namespace LadyLuck
{
namespace runtime
{

void TacticalCompletedTotalLoadReceiptBuilder::Build(
    const ControlFrameIdentity& source_frame_identity,
    const double raw_total_load_g,
    const double governed_total_load_g,
    const double physical_limit_g,
    const TacticalCompletedTotalLoadSource source,
    TacticalCompletedTotalLoadReceipt& output,
    Status& status) const noexcept
{
    output = TacticalCompletedTotalLoadReceipt{};
    status = Status{};
    if (!IsValidControlFrameIdentity(source_frame_identity)
        || !ValidSource(source))
    {
        status.code = StatusCode::InvalidConfiguration;
        return;
    }
    if (!std::isfinite(raw_total_load_g)
        || !std::isfinite(governed_total_load_g)
        || !std::isfinite(physical_limit_g))
    {
        status.code = StatusCode::NonFiniteInput;
        return;
    }
    // d90's completed-feedback adapter publishes this triplet only when every
    // member is strictly positive.  Governor consistency is deliberately not
    // checked here; tactical consumers own that separate admission decision.
    if (raw_total_load_g <= 0.0
        || governed_total_load_g <= 0.0
        || physical_limit_g <= 0.0)
    {
        status.code = StatusCode::InvalidArgument;
        return;
    }

    output.valid = true;
    output.source_frame_identity = source_frame_identity;
    output.raw_total_load_g = raw_total_load_g;
    output.governed_total_load_g = governed_total_load_g;
    output.physical_limit_g = physical_limit_g;
    output.source = source;
}

} // namespace runtime
} // namespace LadyLuck
