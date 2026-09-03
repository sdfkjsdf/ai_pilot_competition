#pragma once

#include <cstdint>

namespace LadyLuck
{
// Only the ed757e27 production profile is represented here. The Python
// low-pass and alpha-beta observer experiments are intentionally excluded.
enum class GapPolicy : std::int32_t
{
    FirstSeed = 1,
    Normal = 2,
    ObserverReprime = 3,
    GapResync = 4
};

enum class SourceTimeKind : std::int32_t
{
    CallerUnspecified = 0,
    PlaneInfoIndexDerived = 1
};

enum class ActionFeedbackKind : std::int32_t
{
    CallerUnspecified = 0,
    ResetSeed = 1,
    PreviousTransmittedAssumption = 2
};

enum class BodyRateGate : std::int32_t
{
    Uninitialized = 0,
    Reset = 1,
    Init = 2,
    Update = 3,
    InvalidAttitude = 4,
    InvalidDt = 5,
    FrameGap = 6,
    AmbiguousRotation = 7
};

enum class FeatureGate : std::int32_t
{
    Reset = 0,
    Init = 1,
    Update = 2,
    Ready = 3,
    Warmup = 4,
    LowDataQuality = 5,
    InvalidAttitude = 6,
    InvalidDt = 7,
    FrameGap = 8,
    AmbiguousRotation = 9
};

enum class TranslationalGate : std::int32_t
{
    Uninitialized = 0,
    Reset = 1,
    Init = 2,
    Update = 3,
    InvalidSample = 4,
    InvalidVelocity = 5,
    InvalidDt = 6,
    FrameGap = 7
};

enum class EstimatorGate : std::int32_t
{
    Uninitialized = 0,
    SeedUnvalidated = 1,
    ConstructionSeedUnvalidated = 2,
    ExplicitResetSeedUnvalidated = 3,
    GapResyncSeedUnvalidated = 4,
    DependencyPqrIntervalInvalid = 5,
    DependencyPqrEndpointInvalid = 6,
    LiveEndpoint = 7,
    LiveActuatorReplica = 8
};

enum class EstimatorSource : std::int32_t
{
    Uninitialized = 0,
    ModelSeedUnvalidated = 1,
    AeroModelInvalidPqrSentinelKinematicFusionDisabled = 2,
    AeroModelIntervalFallbackKinematicFusionDisabled = 3,
    AeroModelEndpointPqrKinematicFusionDisabled = 4,
    FbwActuatorReplicaSeed = 5,
    FbwActuatorReplicaLive = 6,
    So3Raw = 7,
    ConfigurationPlusFuelDeadReckoning = 8,
    NedVelocityDifferenceShadow = 9
};

enum class TimeAlignment : std::int32_t
{
    IntervalAverageReportedAtK = 1,
    EndpointKSecondOrder = 2
};

enum class EstimatorSchemaVersion : std::int32_t
{
    DeployableEstimatorOutputV6 = 6
};
}
