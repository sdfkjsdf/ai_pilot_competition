#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>

namespace LadyLuck
{
// Allocation-free representation of Python's float | None return values.
// value remains finite and is ignored while has_value is false.
struct FrontalPassOptionalScalar
{
    bool has_value = false;
    double value = 0.0;
};

struct FrontalPassBandClearingPair
{
    bool has_value = false;
    double safe_abeam_m = 0.0;
    double compressed_abeam_m = 0.0;
};

// Exact enum representation of the non-empty Python reason strings. None is
// used only while Status reports a caller/internal fault and the output is not
// publishable.
enum class FrontalPassReason : std::uint8_t
{
    None = 0U,
    OpeningGeometryNoPendingFrontalPass = 1U,
    DegenerateHorizontalLineOfSight = 2U,
    NoUnlockedOfficialPhase = 3U,
    OffsetTowardTheAlreadyOpenSide = 4U,
    ExactHeadOnUsesCallerEntrySide = 5U,
    SideLatchedFromFirstAdmission = 6U
};

struct FrontalPassEvidence
{
    bool admitted = false;
    FrontalPassReason reason = FrontalPassReason::None;

    // These three finite backing values are valid together iff admitted.
    double safe_abeam_m = 0.0;
    double compressed_abeam_m = 0.0;
    std::int32_t side_sign = 0;

    // Observation only. This is the official enemy-cone match and never gates
    // admission, exactly as in the Python authority.
    bool threatened_now = false;
};

// d90e929 frontal_pass_observer.py semantic counterparts. All routines reset
// their out parameters on entry. A non-negative Status plus has_value=false is
// Python None; a negative Status represents a Python caller-contract fault.
void OfficialConeAbeamM(
    double t_sec,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept;

void FrameObservationErrorM(
    const Vector3& own_position_m,
    const Vector3& adversary_position_m,
    double& output_m,
    Status& status) noexcept;

void EffectiveCorridorHalfWidthM(
    double cone_half_width_m,
    double observation_error_m,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept;

void BandClearingAbeamPairM(
    double t_sec,
    double range_m,
    double lateral_m,
    double observation_error_m,
    FrontalPassBandClearingPair& output,
    Status& status) noexcept;

void BandClearingAbeamM(
    double t_sec,
    double range_m,
    double lateral_m,
    double observation_error_m,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept;

void CompressedBandClearingAbeamM(
    double t_sec,
    double range_m,
    double lateral_m,
    double observation_error_m,
    FrontalPassOptionalScalar& output,
    Status& status) noexcept;

void EvaluateFrontalPass(
    const DogfightGeometryFrame& frame,
    std::int32_t fallback_side_sign,
    FrontalPassEvidence& output,
    Status& status) noexcept;

class FrontalPassTracker
{
public:
    void Reset() noexcept;

    void GetLatchedSideSign(
        std::int32_t& side_sign,
        bool& has_value) const noexcept;

    void Update(
        const DogfightGeometryFrame& frame,
        std::int32_t fallback_side_sign,
        FrontalPassEvidence& output,
        Status& status) noexcept;

private:
    bool latched_side_valid_ = false;
    std::int32_t latched_side_sign_ = 0;
};
}
