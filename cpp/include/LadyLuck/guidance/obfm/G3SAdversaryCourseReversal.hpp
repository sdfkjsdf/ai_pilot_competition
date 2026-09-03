#pragma once

#include "LadyLuck/contracts/Status.hpp"
#include "LadyLuck/geometry/DogfightGeometryFrame.hpp"

#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace guidance
{
namespace obfm
{

// Exact binary64 result of Python math.radians(1.0), the smallest
// official WEZ cone half-angle in d90.
constexpr double G3SReversalScaleRad = 0.017453292519943295;

struct G3SOptionalDouble
{
    bool has_value = false;
    double value = 0.0;
};

enum class AdversaryReversalObservationReason : std::uint8_t
{
    FrameStateNotFinite = 0U,
    AdversaryCourseNotResolved = 1U,
    FirstCourseSample = 2U,
    CourseRotationNotResolved = 3U,
    CourseRotationResolved = 4U,
    ObserverContractRejected = 5U
};

const char* AdversaryReversalObservationReasonLabel(
    AdversaryReversalObservationReason reason) noexcept;

struct AdversaryReversalObservation
{
    bool valid = false;
    AdversaryReversalObservationReason reason =
        AdversaryReversalObservationReason::FrameStateNotFinite;
    bool reversal_current = false;
    std::int32_t current_sign = 0;
    G3SOptionalDouble current_run_start_t{};
    G3SOptionalDouble current_run_last_t{};
    double current_run_net_rad = 0.0;
    std::int32_t previous_run_sign = 0;
    G3SOptionalDouble previous_run_duration_s{};
    double reversal_scale_rad = G3SReversalScaleRad;
};

class AdversaryCourseReversalObserver final
{
public:
    AdversaryCourseReversalObserver() noexcept;

    void Reset() noexcept;
    void Update(
        const DogfightGeometryFrame& frame,
        AdversaryReversalObservation& output,
        Status& status) noexcept;

    G3SOptionalDouble AlternationEpisodeStartT() const noexcept;

private:
    void BuildObservation(
        bool valid,
        AdversaryReversalObservationReason reason,
        double now_t,
        AdversaryReversalObservation& output) const noexcept;

    bool previous_course_valid_ = false;
    double previous_course_x_ = 0.0;
    double previous_course_y_ = 0.0;
    bool previous_course_error_valid_ = false;
    double previous_course_error_rad_ = 0.0;
    std::int32_t current_sign_ = 0;
    G3SOptionalDouble current_start_t_{};
    G3SOptionalDouble current_last_t_{};
    double current_net_rad_ = 0.0;
    std::int32_t challenger_sign_ = 0;
    G3SOptionalDouble challenger_start_t_{};
    G3SOptionalDouble challenger_last_t_{};
    double challenger_net_rad_ = 0.0;
    std::int32_t previous_sign_ = 0;
    G3SOptionalDouble previous_duration_s_{};
    G3SOptionalDouble previous_start_t_{};
};

bool ScissorsSituationResolved(
    const AdversaryReversalObservation* observation) noexcept;

static_assert(
    std::is_trivially_copyable<AdversaryReversalObservation>::value,
    "G3-S reversal observation must remain allocation-free");

} // namespace obfm
} // namespace guidance
} // namespace LadyLuck
