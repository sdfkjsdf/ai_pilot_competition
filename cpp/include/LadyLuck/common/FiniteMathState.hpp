#pragma once

#include <cstdint>

namespace LadyLuck
{
namespace common
{

enum class FiniteMathState : std::uint8_t
{
    Available = 0U,
    Degenerate = 1U,
    ArithmeticUnavailable = 2U
};

} // namespace common
} // namespace LadyLuck
