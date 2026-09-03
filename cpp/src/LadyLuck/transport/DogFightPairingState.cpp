#include "LadyLuck/transport/DogFightPairingState.hpp"

#include <cstring>
#include <type_traits>

namespace LadyLuck
{
namespace transport
{
namespace
{

template <typename T>
void CopyObjectBits(const T& source, T& destination) noexcept
{
    static_assert(
        std::is_trivially_copyable<T>::value,
        "pairing values must remain trivially copyable");
    std::memcpy(&destination, &source, sizeof(T));
}

void UpdatePlaneSnapshot(
    const PlaneInfo& packet,
    DogFightPlaneSnapshot& snapshot) noexcept
{
    snapshot.is_valid = true;
    snapshot.plane_id = packet.plane_id;
    snapshot.frame_index = packet.index;
    CopyObjectBits(packet, snapshot.plane_info);
}

DogFightPairingReceipt ResetReceipt(
    const DogFightPairingReceiptCode code) noexcept
{
    DogFightPairingReceipt receipt{};
    receipt.code = code;
    receipt.state_committed = true;
    receipt.policy_reset_required = true;
    return receipt;
}

bool ExactZeroVelocity(const PlaneInfo& plane) noexcept
{
    return plane.velocity.x == 0.0F
        && plane.velocity.y == 0.0F
        && plane.velocity.z == 0.0F;
}

} // namespace

DogFightPairingReceipt DogFightPairingState::OnSetPlaneID(
    const SetPlaneID& packet) noexcept
{
    // client.py deliberately retains both latest snapshots and the context
    // frame here. Only the role identity and pair-receipt flags are reset.
    state_.assigned_plane_id = packet.plane_id;
    state_.own_info_received = false;
    state_.enemy_info_received = false;
    return ResetReceipt(DogFightPairingReceiptCode::SetPlaneIdCommitted);
}

DogFightPairingReceipt DogFightPairingState::OnInit(
    const Init& packet) noexcept
{
    CopyObjectBits(packet, state_.initial_state);
    state_.has_initial_state = true;
    state_.own_plane = DogFightPlaneSnapshot{};
    state_.enemy_plane = DogFightPlaneSnapshot{};
    state_.own_info_received = false;
    state_.enemy_info_received = false;
    // client.py does not reset context.frame_index or assigned_plane_id here.
    return ResetReceipt(DogFightPairingReceiptCode::InitCommitted);
}

DogFightPairingReceipt DogFightPairingState::OnGameControl(
    const GameControl& packet) noexcept
{
    CopyObjectBits(packet, state_.game_control);
    state_.has_game_control = true;

    DogFightPairingReceipt receipt{};
    receipt.code = DogFightPairingReceiptCode::GameControlCommitted;
    receipt.state_committed = true;
    return receipt;
}

DogFightPairingReceipt DogFightPairingState::OnPlaneInfo(
    const PlaneInfo& packet) noexcept
{
    DogFightPairingReceipt receipt{};

    const bool is_ownship =
        state_.assigned_plane_id != -1
        && packet.plane_id == state_.assigned_plane_id;
    DogFightPlaneSnapshot& destination =
        is_ownship ? state_.own_plane : state_.enemy_plane;

    // Strictly older packets are the only rejected PlaneInfo packets. The
    // early return is atomic: latest state, context index, and receipt flags
    // are all unchanged. Equality, duplicates, and index gaps are admitted.
    if (destination.is_valid && packet.index < destination.frame_index)
    {
        receipt.code = DogFightPairingReceiptCode::StalePlaneDropped;
        return receipt;
    }

    UpdatePlaneSnapshot(packet, destination);
    if (is_ownship)
    {
        state_.own_info_received = true;
    }
    else
    {
        state_.enemy_info_received = true;
    }
    state_.context_frame_index = packet.index;

    receipt.state_committed = true;
    if (!(state_.own_info_received && state_.enemy_info_received))
    {
        receipt.code = DogFightPairingReceiptCode::PlaneAcceptedWaiting;
        return receipt;
    }

    const bool prestart_placeholder =
        ExactZeroVelocity(state_.own_plane.plane_info)
        && ExactZeroVelocity(state_.enemy_plane.plane_info);
    receipt.code = prestart_placeholder
        ? DogFightPairingReceiptCode::PrestartPlaceholderSuppressed
        : DogFightPairingReceiptCode::PairReady;
    receipt.has_ready_pair = !prestart_placeholder;
    receipt.ready_pair.assigned_plane_id = state_.assigned_plane_id;
    receipt.ready_pair.command_frame_index = state_.context_frame_index;
    receipt.ready_pair.has_initial_state = state_.has_initial_state;
    CopyObjectBits(state_.initial_state, receipt.ready_pair.initial_state);
    receipt.ready_pair.has_game_control = state_.has_game_control;
    CopyObjectBits(state_.game_control, receipt.ready_pair.game_control);
    CopyObjectBits(state_.own_plane, receipt.ready_pair.own_plane);
    CopyObjectBits(state_.enemy_plane, receipt.ready_pair.enemy_plane);

    // Match client.py's transaction boundary: clear both receipts before an
    // external policy call. A downstream failure cannot cause this pair to be
    // issued a second time.
    state_.own_info_received = false;
    state_.enemy_info_received = false;
    return receipt;
}

DogFightPairingSnapshot DogFightPairingState::Snapshot() const noexcept
{
    DogFightPairingSnapshot snapshot{};
    CopyObjectBits(state_, snapshot);
    return snapshot;
}

} // namespace transport
} // namespace LadyLuck
