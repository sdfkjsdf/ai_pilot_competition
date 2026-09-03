#pragma once

#include "LadyLuck/contracts/TacticalCommand.hpp"
#include "LadyLuck/guidance/GunDefenseControlIntent.hpp"

#include <cstdint>

namespace LadyLuck
{

struct GunDefenseOutput
{
    OptionalValue<TacticalCommand> command{};
    bool threat_active = false;
    bool entered = false;
    bool cleared = false;
    OptionalValue<std::int32_t> side_sign{};
    std::uint64_t entry_count = 0U;
    bool toward_side_candidate_held = false;
    bool missile_branch_enabled = false;
};

Result<TacticalCommand> BuildHorizontalBreakCommand(
    const DogfightGeometryFrame& frame,
    std::int32_t side_sign) noexcept;

class GunDefensePolicy
{
public:
    GunDefensePolicy() noexcept;

    void Reset() noexcept;
    std::int32_t NextSideSign() const noexcept;
    GunDefenseSnapshot Snapshot() const noexcept;

    // admitted_threat_active is owned by the future Root/Service layer.  This
    // numerical module neither infers admission nor publishes a Blackboard
    // command.  It has no frame-index, dt, or gap state.
    Result<GunDefenseOutput> Step(
        const DogfightGeometryFrame& frame,
        bool admitted_threat_active,
        const OptionalValue<std::int32_t>& entry_side_sign = {}) noexcept;

private:
    bool active_ = false;
    std::int32_t side_sign_ = 1;
    std::uint64_t entry_count_ = 0U;
    bool toward_side_candidate_held_ = false;
};
}
