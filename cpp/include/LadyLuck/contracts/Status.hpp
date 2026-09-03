#pragma once

#include <cstdint>

namespace LadyLuck
{
enum class StatusCode : std::int32_t
{
    Ok = 0,
    Seeded = 1,
    ObservationInvalid = 2,
    FrameGap = 3,
    InvalidArgument = -1,
    NonFiniteInput = -2,
    InvalidDt = -3,
    AmbiguousRotation = -4,
    InvalidConfiguration = -5
};

struct Status
{
    StatusCode code = StatusCode::Ok;

    // Non-negative codes are normal, accepted observer receipts. Seeded,
    // ObservationInvalid, and FrameGap deliberately carry an invalid sample
    // with an explanatory gate; they are not API or transaction failures.
    bool ok() const noexcept
    {
        return static_cast<std::int32_t>(code) >= 0;
    }

    bool sample_valid() const noexcept
    {
        return code == StatusCode::Ok;
    }
};

template <typename T>
struct Result
{
    Status status{};
    T value{};

    bool ok() const noexcept
    {
        return status.ok();
    }

    bool sample_valid() const noexcept
    {
        return status.sample_valid();
    }
};
}
