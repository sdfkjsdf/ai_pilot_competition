#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/guidance/dbfm/DbfmDefenseSpeedControlIntent.hpp"
#include "LadyLuck/guidance/dbfm/DbfmHardTurnControlIntent.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace dbfm
{

// Exact d90 selector priority for the ordinary DBFM subtree.  None is only
// the finite reset/no-mode backing value; it never owns a command.
enum class DbfmOrdinaryBranch : std::uint8_t
{
    None = 0U,
    Break = 1U,
    AltitudeSeparated = 2U,
    Escape = 3U,
    Extend = 4U,
    HardTurn = 5U
};

enum class DbfmOrdinaryUnavailability : std::uint8_t
{
    None = 0U,
    RootGunOwner = 1U,
    ProductionConfigDisabled = 2U
};

// A branch condition may be observed but false, or explicitly unavailable.
// selected=true is admissible only when evidence_available=true.
struct DbfmOrdinaryBranchAdmission
{
    bool evidence_available = false;
    bool selected = false;
    DbfmOrdinaryUnavailability unavailability =
        DbfmOrdinaryUnavailability::None;
};

// Same-frame Service receipt for the selector conditions.  The current d90
// competition configuration marks every higher-priority ordinary-DBFM branch
// unavailable: actual gun BREAK is Root-owned, and altitude/escape/extend are
// default-OFF.  That makes HARD_TURN the production-effective fallback.
struct DbfmOrdinaryAdmissionReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    DbfmOrdinaryBranchAdmission break_branch{};
    DbfmOrdinaryBranchAdmission altitude_separated_branch{};
    DbfmOrdinaryBranchAdmission escape_branch{};
    DbfmOrdinaryBranchAdmission extend_branch{};
};

// The corner interval may be unadmitted, but its receipt must still be bound
// to the current frame.  Unadmitted sustained speed preserves HARD_TURN's
// current-speed echo exactly.
struct DbfmOrdinarySpeedReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    DbfmCornerSpeedControlEvidence evidence{};
};

struct DbfmOrdinarySelectionReceipt
{
    ControlFrameIdentity frame_identity{};
    bool valid = false;
    bool mode_selected = false;
    DbfmOrdinaryBranch selected_branch = DbfmOrdinaryBranch::None;
    std::uint32_t selection_count = 0U;
    bool command_ready = false;
};

class DbfmOrdinaryModeWriter final
{
public:
    DbfmOrdinaryModeWriter() noexcept;

    void Reset() noexcept;

    void ObserveCurrentProductionAdmissions(
        const DogfightGeometryFrame& frame,
        DbfmOrdinaryAdmissionReceipt& output,
        Status& status) const noexcept;

    void Build(
        const DogfightGeometryFrame& frame,
        DoctrineModeId selected_mode,
        const DbfmOrdinaryAdmissionReceipt& admissions,
        const DbfmOrdinarySpeedReceipt& speed_receipt,
        const DbfmHardTurnGeometryReceipt& hard_turn_geometry,
        ControlIntent& output,
        DbfmOrdinarySelectionReceipt& selection,
        Status& status) noexcept;

private:
    bool active_ = false;
    DbfmOrdinaryBranch active_branch_ = DbfmOrdinaryBranch::None;
    ControlFrameIdentity last_frame_identity_{};
};

static_assert(
    std::is_trivially_copyable<DbfmOrdinaryAdmissionReceipt>::value,
    "DBFM admission receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<DbfmOrdinarySpeedReceipt>::value,
    "DBFM speed receipt must remain allocation-free.");
static_assert(
    std::is_trivially_copyable<DbfmOrdinarySelectionReceipt>::value,
    "DBFM selection receipt must remain allocation-free.");

} // namespace dbfm
} // namespace guidance
} // namespace LadyLuck
