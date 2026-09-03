#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

using BtNodeId = std::uint32_t;
using BtStageId = std::uint32_t;
using BtReasonId = std::uint32_t;
using BtWriterId = std::uint32_t;
using BtNodeIndex = std::uint32_t;

// A multi-tick maneuver owner is not a leaf node.  Several phase and terminal
// leaves may share one lifecycle owner while retaining distinct source-node
// identities for diagnostics.
struct BtLifecycleOwnerId
{
    std::uint32_t value = 0U;
};

constexpr BtNodeId BtNodeIdInvalid = 0U;
constexpr BtStageId BtStageIdInvalid = 0U;
constexpr BtReasonId BtReasonIdNone = 0U;
constexpr BtWriterId BtWriterIdNone = 0U;
constexpr BtLifecycleOwnerId BtLifecycleOwnerIdNone{};

constexpr BtLifecycleOwnerId MakeBtLifecycleOwnerId(
    const BtWriterId writer_id) noexcept
{
    return BtLifecycleOwnerId{writer_id};
}

constexpr bool SameBtLifecycleOwnerId(
    const BtLifecycleOwnerId lhs,
    const BtLifecycleOwnerId rhs) noexcept
{
    return lhs.value == rhs.value;
}

constexpr bool HasBtLifecycleOwnerId(
    const BtLifecycleOwnerId owner_id) noexcept
{
    return owner_id.value != BtWriterIdNone;
}

// Normal control-flow outcomes and internal faults are intentionally disjoint.
// Callers must compare the typed value; numeric sign and ordering have no
// semantic meaning.
enum class BtReturnCode : std::int32_t
{
    Selected = 0,
    NotApplicable = 1,
    Running = 2,
    Completed = 3,
    Released = 4,

    InvalidInput = 100,
    InvalidPort = 101,
    InvalidTopology = 102,
    CapacityExceeded = 103,
    InternalContractFault = 104,
    InvalidRoot = 105,
    InvalidNode = 106,
    InvalidChild = 107,
    DuplicateNodeId = 108,
    DuplicateWriter = 109,
    CycleDetected = 110,
    UnreachableNode = 111,
    MissingRequiredInput = 112,
    InvalidNodeKind = 113,
    WriterContractFault = 114
};

struct BtTickResult
{
    BtReturnCode code = BtReturnCode::InvalidInput;
    BtNodeId node_id = BtNodeIdInvalid;
    BtStageId stage_id = BtStageIdInvalid;
    std::uint64_t frame_index = 0U;
    BtReasonId reason = BtReasonIdNone;
};

constexpr bool IsBtReturnCodeError(const BtReturnCode code) noexcept
{
    switch (code)
    {
    case BtReturnCode::Selected:
    case BtReturnCode::NotApplicable:
    case BtReturnCode::Running:
    case BtReturnCode::Completed:
    case BtReturnCode::Released:
        return false;
    default:
        return true;
    }
}

constexpr bool IsBtReturnCodeTerminal(const BtReturnCode code) noexcept
{
    return code == BtReturnCode::Completed ||
           code == BtReturnCode::Released;
}

constexpr BtTickResult MakeBtTickResult(
    const BtReturnCode code,
    const BtNodeId node_id,
    const BtStageId stage_id,
    const std::uint64_t frame_index,
    const BtReasonId reason) noexcept
{
    return BtTickResult{code, node_id, stage_id, frame_index, reason};
}

static_assert(
    std::is_same<std::underlying_type<BtReturnCode>::type,
                 std::int32_t>::value,
    "BtReturnCode must retain its fixed-width ABI type");
static_assert(std::is_standard_layout<BtLifecycleOwnerId>::value,
              "Lifecycle owner ID must remain standard-layout");
static_assert(std::is_trivially_copyable<BtLifecycleOwnerId>::value,
              "Lifecycle owner ID must remain trivially copyable");
static_assert(sizeof(BtLifecycleOwnerId) == 4U,
              "Lifecycle owner ID x64 ABI size changed");
static_assert(alignof(BtLifecycleOwnerId) == 4U,
              "Lifecycle owner ID x64 ABI alignment changed");
static_assert(offsetof(BtLifecycleOwnerId, value) == 0U,
              "Lifecycle owner ID value offset changed");
static_assert(std::is_standard_layout<BtTickResult>::value,
              "BtTickResult must remain a fixed-width receipt");
static_assert(std::is_trivially_copyable<BtTickResult>::value,
              "BtTickResult must remain trivially copyable");
static_assert(sizeof(BtTickResult) == 32U,
              "BtTickResult x64 ABI size changed");
static_assert(alignof(BtTickResult) == 8U,
              "BtTickResult x64 ABI alignment changed");
static_assert(offsetof(BtTickResult, code) == 0U,
              "BtTickResult.code offset changed");
static_assert(offsetof(BtTickResult, node_id) == 4U,
              "BtTickResult.node_id offset changed");
static_assert(offsetof(BtTickResult, stage_id) == 8U,
              "BtTickResult.stage_id offset changed");
static_assert(offsetof(BtTickResult, frame_index) == 16U,
              "BtTickResult.frame_index offset changed");
static_assert(offsetof(BtTickResult, reason) == 24U,
              "BtTickResult.reason offset changed");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
