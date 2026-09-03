#pragma once

#include "LadyLuck/contracts/ControlIntent.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"
#include "LadyLuck/runtime/RootGunPreTaskEvidenceProvider.hpp"
#include "LadyLuck/safety/AutoGcas.hpp"

#include <cstdint>

namespace LadyLuck
{
namespace runtime
{

// Allocation-free leaf writers for the currently representable Root subset.
// A BehaviorTree Service observes the official threat, its Decorator selects
// one branch, and exactly one selected leaf calls one Build method. This class
// never selects a branch and never publishes a fallback for a failed writer.
class RootTacticalCommandWriters final
{
public:
    RootTacticalCommandWriters() noexcept;

    // Full provider/episode reset.  Clears both the Gun episode and the
    // causal last-observed horizontal course.
    void Reset() noexcept;
    // Safety preemption clears only Gun episode ownership.  The immediately
    // preceding same-episode observed course remains a last fallback after
    // current nose, velocity, and measured body-up are unobservable.
    void ResetGunThreatEpisode() noexcept;
    void ObserveOfficialGunThreat(
        const DogfightGeometryFrame& frame,
        bool& output,
        Status& status) noexcept;
    void ObserveAdmittedGunThreat(
        bool admitted_threat_active,
        Status& status) noexcept;
    // Shared Python GunDefensePolicy entry parity.  Predictive DBFM margin
    // may read this side without activating or incrementing the episode;
    // the later official Root Gun entry is the sole commit owner.
    std::int32_t NextGunSideSign() const noexcept
    {
        return gun_entry_count_ % 2U == 0U ? 1 : -1;
    }
    void ObserveRootGunPreTaskEvidence(
        const DogfightGeometryFrame& frame,
        RootGunPreTaskEvidence& output,
        Status& status) noexcept;
    void BuildGunDefense(
        const DogfightGeometryFrame& frame,
        const RootGunPreTaskEvidence& evidence,
        bool admitted_threat_active,
        bool entry_side_sign_valid,
        std::int32_t entry_side_sign,
        ControlIntent& output,
        Status& status) noexcept;
    // Commit only the narrow Gun force-direction history after the Root
    // contract has actually published this frame.  A commit mismatch clears
    // history but never revokes an already valid command.
    void CommitPublishedGunDirection(
        const ControlFrameIdentity& frame_identity) noexcept;
    void BuildRootAutoGcasRecovery(
        const DogfightGeometryFrame& frame,
        const safety::AutoGcasEntryReceipt& safety,
        ControlIntent& output,
        Status& status) noexcept;

private:
    bool gun_threat_active_ = false;
    std::int32_t gun_side_sign_ = 1;
    std::uint64_t gun_entry_count_ = 0U;
    bool gun_break_direction_valid_ = false;
    Vector3 gun_break_direction_ned_{};
    ControlFrameIdentity gun_break_direction_frame_{};
    bool pending_gun_break_direction_valid_ = false;
    Vector3 pending_gun_break_direction_ned_{};
    ControlFrameIdentity pending_gun_break_direction_frame_{};
    bool horizontal_course_history_valid_ = false;
    Vector3 horizontal_course_history_ned_{};
    ControlFrameIdentity horizontal_course_history_frame_{};
    RootGunPreTaskEvidenceProvider pre_task_evidence_provider_{};
};

} // namespace runtime
} // namespace LadyLuck
