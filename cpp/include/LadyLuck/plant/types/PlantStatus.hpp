#pragma once

namespace LadyLuck
{
namespace plant
{

enum class PlantStatusCode
{
    Ok = 0,
    InvalidArgument,
    InvalidState,
    NonFiniteResult,
    DependencyFailure
};

struct PlantStatus
{
    PlantStatusCode code = PlantStatusCode::Ok;
    const char* message = "ok";

    bool ok() const noexcept
    {
        return code == PlantStatusCode::Ok;
    }

    static PlantStatus Success() noexcept
    {
        return PlantStatus{};
    }

    static PlantStatus Failure(
        const PlantStatusCode failure_code,
        const char* failure_message) noexcept
    {
        PlantStatus status;
        status.code = failure_code;
        status.message = failure_message;
        return status;
    }
};

template <typename T>
struct PlantResult
{
    PlantStatus status{};
    T value{};

    bool ok() const noexcept
    {
        return status.ok();
    }
};

} // namespace plant
} // namespace LadyLuck
