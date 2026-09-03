#pragma once

#include "LadyLuck/transport/DogFightProtocol.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace transport
{

// This module preserves the provided Python client's receive chronology. It
// does not interpret coordinate frames, shape guidance, or produce a control
// command.
struct DogFightPlaneSnapshot
{
    bool is_valid = false;
    std::int32_t plane_id = -1;
    std::uint64_t frame_index = 0U;
    PlaneInfo plane_info{};
};

struct DogFightPairingSnapshot
{
    std::int32_t assigned_plane_id = -1;
    std::uint64_t context_frame_index = 0U;
    bool has_initial_state = false;
    Init initial_state{};
    bool has_game_control = false;
    GameControl game_control{};
    DogFightPlaneSnapshot own_plane{};
    DogFightPlaneSnapshot enemy_plane{};
    bool own_info_received = false;
    bool enemy_info_received = false;
};

struct DogFightReadyPair
{
    std::int32_t assigned_plane_id = -1;
    // Matches client.py: this is the second accepted arrival's index, not a
    // same-index synthesis and not necessarily the ownship index.
    std::uint64_t command_frame_index = 0U;
    bool has_initial_state = false;
    Init initial_state{};
    bool has_game_control = false;
    GameControl game_control{};
    DogFightPlaneSnapshot own_plane{};
    DogFightPlaneSnapshot enemy_plane{};
};

enum class DogFightPairingReceiptCode : std::int32_t
{
    SetPlaneIdCommitted = 0,
    InitCommitted = 1,
    PlaneAcceptedWaiting = 2,
    PairReady = 3,
    StalePlaneDropped = 4,
    GameControlCommitted = 5,
    PrestartPlaceholderSuppressed = 6
};

struct DogFightPairingReceipt
{
    DogFightPairingReceiptCode code =
        DogFightPairingReceiptCode::PlaneAcceptedWaiting;
    bool state_committed = false;
    // SetPlaneID and Init are the exact provided-client policy-reset receipts.
    bool policy_reset_required = false;
    // Each true receipt is exactly one action-repeat=1 policy-call boundary.
    bool has_ready_pair = false;
    DogFightReadyPair ready_pair{};
};

class DogFightPairingState final
{
public:
    DogFightPairingState() noexcept = default;

    DogFightPairingReceipt OnSetPlaneID(
        const SetPlaneID& packet) noexcept;
    DogFightPairingReceipt OnInit(const Init& packet) noexcept;
    DogFightPairingReceipt OnGameControl(
        const GameControl& packet) noexcept;
    DogFightPairingReceipt OnPlaneInfo(
        const PlaneInfo& packet) noexcept;

    DogFightPairingSnapshot Snapshot() const noexcept;

private:
    DogFightPairingSnapshot state_{};
};

} // namespace transport
} // namespace LadyLuck
