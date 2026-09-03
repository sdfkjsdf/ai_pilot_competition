#pragma once

#include "LadyLuck/contracts/Enums.hpp"

#include <cstdint>

namespace LadyLuck
{
struct OptionalFrameIndex
{
    bool has_value = false;
    std::uint64_t value = 0U;
};

struct OptionalEpoch
{
    bool has_value = false;
    std::uint64_t value = 0U;
};

struct OptionalSeconds
{
    bool has_value = false;
    double value = 0.0;
};

// Allocation-free causal identity shared by same-frame control products.
// A valid token binds a product to one accepted observation frame and its
// source time; consumers must not combine products with different tokens.
struct ControlFrameIdentity
{
    bool valid = false;
    std::uint64_t episode_epoch = 0U;
    std::uint64_t frame_index = 0U;
    double source_time_s = 0.0;
};

bool IsValidControlFrameIdentity(
    const ControlFrameIdentity& identity) noexcept;
bool SameControlFrameIdentity(
    const ControlFrameIdentity& left,
    const ControlFrameIdentity& right) noexcept;

struct FrameContext
{
    std::uint64_t episode_epoch = 0U;
    std::uint64_t measurement_frame_index = 0U;
    OptionalFrameIndex command_frame_index{};
    double source_t_sec = 0.0;
    SourceTimeKind source_time_kind = SourceTimeKind::PlaneInfoIndexDerived;
    double nominal_dt_sec = 1.0 / 60.0;
    double sample_dt_sec = 1.0 / 60.0;
    std::uint64_t gap_count = 1U;
    GapPolicy gap_policy = GapPolicy::FirstSeed;
    OptionalFrameIndex previous_command_frame{};
    double estimator_soft_budget_sec = 0.005;
};
}
