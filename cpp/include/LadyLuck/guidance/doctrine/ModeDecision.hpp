#pragma once

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace doctrine
{

// Total current-situation values used by the production tree.  Every accepted
// tactical frame resolves to OBFM, HABFM, or DBFM; Safety is a senior bypass.
enum class TacticalMode : std::uint8_t
{
    Safety = 1U,
    Obfm = 2U,
    Habfm = 3U,
    Dbfm = 4U
};

enum class ModeDecisionBypassReason : std::uint8_t
{
    None = 0U,
    SafetyRequired = 1U,
    ActualGunThreat = 2U
};

struct OptionalTacticalMode
{
    bool has_value = false;
    TacticalMode value = TacticalMode::Habfm;
};

struct ModeDecision
{
    // `valid` is false only when Status reports a caller/configuration fault.
    // All backing values remain finite even in that case.
    bool valid = false;
    TacticalMode raw_mode = TacticalMode::Habfm;
    OptionalTacticalMode mode{};
    bool admitted = false;
    ModeDecisionBypassReason bypass_reason =
        ModeDecisionBypassReason::None;
};
static_assert(
    std::is_trivially_copyable<ModeDecision>::value,
    "ModeDecision must remain allocation-free.");

} // namespace doctrine
} // namespace guidance
} // namespace LadyLuck
