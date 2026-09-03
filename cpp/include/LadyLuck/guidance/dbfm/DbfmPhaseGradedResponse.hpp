#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/GunDefenseControlIntent.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

// Command-neutral focused seam for the exact finite rounded-boundary family.
bool DbfmPhaseGradedArithmeticBoundarySafeForTesting() noexcept;

enum class DbfmPhaseGradedResponseBranch : std::uint8_t
{
    BaseBreak = 0U,
    HardTurn = 1U
};

enum class DbfmPhaseGradedResponseReason : std::uint8_t
{
    NotEvaluated = 0U,
    NonOwner = 1U,
    BaseBreakContractInvalid = 2U,
    BaseBreakRouteUnavailable = 3U,
    HeldSideUnavailable = 4U,
    NotOfficialScratch = 5U,
    RearGeometryUnavailable = 6U,
    AttackerNotStrictRear = 7U,
    HeldSideNoLongerTowardAttacker = 8U,
    HardTurnMaterializationUnavailable = 9U,
    HardTurnSelected = 10U
};

// ImmediateGun response receipt. It owns only the replacement of one
// already-valid current-frame Gun BREAK guidance reference. Body-rate, Nz,
// surface, thrust, estimator, and aircraft-response authority remain outside
// this provider.
struct DbfmPhaseGradedResponseReceipt
{
    ControlFrameIdentity frame_identity{};
    bool evaluated = false;
    bool official_scratch = false;
    bool strict_rear_geometry_available = false;
    bool attacker_in_strict_rear = false;
    bool current_toward_side_available = false;
    std::int32_t current_toward_side_sign = 0;
    bool held_side_preserved = false;
    bool outer_admitted = false;
    DbfmPhaseGradedResponseBranch selected_branch =
        DbfmPhaseGradedResponseBranch::BaseBreak;
    DbfmPhaseGradedResponseReason reason =
        DbfmPhaseGradedResponseReason::NotEvaluated;
    bool replacement_available = false;
    bool base_break_preserved = false;
};

class DbfmPhaseGradedResponseProvider
{
public:
    void Reset() noexcept;

    // The output starts as base_break and changes only after every admission
    // and materialization predicate succeeds.  Therefore every normal finite
    // non-owner, geometric degeneracy, or unavailable real-valued command
    // preserves the already-validated same-frame BREAK exactly once.
    void Evaluate(
        const DogfightGeometryFrame& frame,
        bool root_gun_owner_selected,
        const GunDefenseSnapshot& gun_episode,
        const ControlIntent& base_break,
        DbfmPhaseGradedResponseReceipt& receipt,
        ControlIntent& output,
        Status& status) noexcept;

};

static_assert(
    std::is_trivially_copyable<DbfmPhaseGradedResponseReceipt>::value,
    "DBFM phase-graded response receipt must remain allocation-free.");

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
