#include "LadyLuck/guidance/em/EnergyManeuverEnvelope.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace
{
#include "EnergyManeuverTables.generated.inc"

constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kRegressionRidge = 1.0e-6;
constexpr double kTukeyC = 4.685;
constexpr double kMinimumEffectiveSamples = 6.0;
constexpr std::uint64_t kFnvOffsetBasis = 0xCBF29CE484222325ULL;
constexpr std::uint64_t kFnvPrime = 0x100000001B3ULL;
const char kCanonicalTableSource[] = "table/em_tables.npz";
const char kCanonicalTableSha256[] =
    "C808635A214D71CE29B6177C7E22E7C8C81BE7F65D49F20D05F66D4371DF60F8";
const char kCanonicalF16Md5[] = "770FB6E1BD2E2238808E614313A9AA43";
const char kCanonicalConfigurationPrefix[] = "gear up / speedbrake 0";
const char kCanonicalPublishPolicyV52Sha256[] =
    "022C5969A9C02E782F8A69869C2532A56E5E62F00E045C29B5307D9AF13340B1";

bool NonEmpty(const char* const text) noexcept
{
    return text != nullptr && text[0] != '\0';
}

template <std::size_t Size>
bool FiniteArray(const double (&values)[Size]) noexcept
{
    for (std::size_t index = 0U; index < Size; ++index)
    {
        if (!std::isfinite(values[index]))
        {
            return false;
        }
    }
    return true;
}

template <std::size_t Size>
bool StrictlyIncreasing(const double (&axis)[Size]) noexcept
{
    if (!FiniteArray(axis))
    {
        return false;
    }
    for (std::size_t index = 1U; index < Size; ++index)
    {
        if (!(axis[index] > axis[index - 1U]))
        {
            return false;
        }
    }
    return true;
}

void FnvByte(std::uint64_t& value, const std::uint8_t byte) noexcept
{
    value ^= static_cast<std::uint64_t>(byte);
    value *= kFnvPrime;
}

void FnvDouble(std::uint64_t& value, const double scalar) noexcept
{
    std::uint64_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(scalar), "double width mismatch");
    std::memcpy(&bits, &scalar, sizeof(bits));
    for (unsigned int shift = 0U; shift < 64U; shift += 8U)
    {
        FnvByte(
            value,
            static_cast<std::uint8_t>((bits >> shift) & 0xFFULL));
    }
}

void FnvCString(std::uint64_t& value, const char* const text) noexcept
{
    for (const char* current = text; *current != '\0'; ++current)
    {
        FnvByte(value, static_cast<std::uint8_t>(*current));
    }
}

template <std::size_t Size>
void FnvDoubleArray(
    std::uint64_t& value,
    const double (&values)[Size]) noexcept
{
    for (std::size_t index = 0U; index < Size; ++index)
    {
        FnvDouble(value, values[index]);
    }
}

std::uint64_t TableContentHash() noexcept
{
    std::uint64_t hash = kFnvOffsetBasis;
    FnvDoubleArray(hash, kEmSpeedAxisMps);
    FnvDoubleArray(hash, kEmAltitudeAxisM);
    FnvDoubleArray(hash, kEmPsMaxPubMps);
    FnvDoubleArray(hash, kEmPsMinPubMps);
    FnvDoubleArray(hash, kEmNInstPubG);
    FnvDoubleArray(hash, kEmNSustainedPubG);
    FnvDoubleArray(hash, kEmNInstRawG);
    FnvDoubleArray(hash, kEmNSustainedRawG);
    for (std::size_t index = 0U; index < kEmGridCount; ++index)
    {
        FnvByte(hash, kEmSoundnessMask[index]);
    }
    for (std::size_t index = 0U; index < kEmGridCount; ++index)
    {
        FnvByte(hash, kEmSoundnessMaskN[index]);
    }
    FnvDoubleArray(hash, kEmCornerInstantaneousMps);
    FnvDoubleArray(hash, kEmCornerSustainedMps);
    FnvDouble(hash, kEmReferenceWeightN);
    FnvDoubleArray(hash, kEmAbAnchor);
    FnvDoubleArray(hash, kEmRegressionBandwidth);
    FnvCString(hash, kEmPublishPolicyV52);
    return hash;
}

bool BinaryMasksAndMonotoneTrust() noexcept
{
    for (std::size_t index = 0U; index < kEmGridCount; ++index)
    {
        if (kEmSoundnessMask[index] != 0U
            && kEmSoundnessMask[index] != 1U)
        {
            return false;
        }
        if (kEmSoundnessMaskN[index] != 0U
            && kEmSoundnessMaskN[index] != 1U)
        {
            return false;
        }
        // Zero means trusted.  v5.2 may open N-only cells, but must never
        // close a point already trusted by the shared Ps-inclusive mask.
        if (kEmSoundnessMask[index] == 0U
            && kEmSoundnessMaskN[index] != 0U)
        {
            return false;
        }
    }
    return true;
}

bool PositiveArray(const double* const values, const std::size_t count) noexcept
{
    for (std::size_t index = 0U; index < count; ++index)
    {
        if (!std::isfinite(values[index]) || values[index] <= 0.0)
        {
            return false;
        }
    }
    return true;
}

LadyLuck::guidance::em::EmTableAuthority BuildAuthority() noexcept
{
    LadyLuck::guidance::em::EmTableAuthority authority{};
    authority.table_source = kCanonicalTableSource;
    authority.table_sha256 = kEmSourceSha256;
    authority.f16_md5 = kEmF16Md5;
    authority.baseline_configuration = kEmBaselineConfiguration;
    authority.publish_policy_v52_sha256 = kEmPublishPolicyV52Sha256;
    authority.table_identity_valid =
        std::strcmp(kEmSourceSha256, kCanonicalTableSha256) == 0
        && std::strcmp(
            kEmPublishPolicyV52Sha256,
            kCanonicalPublishPolicyV52Sha256) == 0
        && TableContentHash() == kEmContentFnv1a64;
    authority.schema_valid =
        kEmSpeedCount >= 2U
        && kEmAltitudeCount >= 2U
        && kEmGridCount == kEmSpeedCount * kEmAltitudeCount
        && kEmAnchorWidth == 4U
        && StrictlyIncreasing(kEmSpeedAxisMps)
        && StrictlyIncreasing(kEmAltitudeAxisM)
        && FiniteArray(kEmPsMaxPubMps)
        && FiniteArray(kEmPsMinPubMps)
        && FiniteArray(kEmNInstPubG)
        && FiniteArray(kEmNSustainedPubG)
        && FiniteArray(kEmNInstRawG)
        && FiniteArray(kEmNSustainedRawG)
        && BinaryMasksAndMonotoneTrust()
        && PositiveArray(kEmCornerInstantaneousMps, kEmAltitudeCount)
        && PositiveArray(kEmCornerSustainedMps, kEmAltitudeCount)
        && std::isfinite(kEmReferenceWeightN)
        && kEmReferenceWeightN > 0.0
        && FiniteArray(kEmAbAnchor)
        && PositiveArray(kEmRegressionBandwidth, 3U)
        && NonEmpty(kEmPublishPolicyV52);
    authority.provenance_valid = authority.table_identity_valid
        && authority.schema_valid
        && std::strcmp(kEmF16Md5, kCanonicalF16Md5) == 0
        && std::strncmp(
            kEmBaselineConfiguration,
            kCanonicalConfigurationPrefix,
            sizeof(kCanonicalConfigurationPrefix) - 1U) == 0;
    return authority;
}

const LadyLuck::guidance::em::EmTableAuthority& CachedAuthority() noexcept
{
    static const LadyLuck::guidance::em::EmTableAuthority authority =
        BuildAuthority();
    return authority;
}

std::size_t CellIndex(
    const double* const axis,
    const std::size_t count,
    const double value) noexcept
{
    const double* const found = std::lower_bound(axis, axis + count, value);
    std::ptrdiff_t index = found - axis - 1;
    if (index < 0)
    {
        index = 0;
    }
    const std::ptrdiff_t maximum = static_cast<std::ptrdiff_t>(count - 2U);
    if (index > maximum)
    {
        index = maximum;
    }
    return static_cast<std::size_t>(index);
}

std::size_t GridIndex(
    const std::size_t speed_index,
    const std::size_t altitude_index) noexcept
{
    return speed_index * kEmAltitudeCount + altitude_index;
}

bool AllFourCornersZero(
    const std::uint8_t* const mask,
    const std::size_t speed_index,
    const std::size_t altitude_index) noexcept
{
    return mask[GridIndex(speed_index, altitude_index)] == 0U
        && mask[GridIndex(speed_index + 1U, altitude_index)] == 0U
        && mask[GridIndex(speed_index, altitude_index + 1U)] == 0U
        && mask[GridIndex(speed_index + 1U, altitude_index + 1U)] == 0U;
}

void PopulateCellEvidence(
    const std::size_t speed_index,
    const std::size_t altitude_index,
    LadyLuck::guidance::em::EmCellEvidence& cell) noexcept
{
    cell = LadyLuck::guidance::em::EmCellEvidence{};
    cell.available = true;
    cell.speed_index = speed_index;
    cell.altitude_index = altitude_index;
    cell.speed_lower_mps = kEmSpeedAxisMps[speed_index];
    cell.speed_upper_mps = kEmSpeedAxisMps[speed_index + 1U];
    cell.altitude_lower_m = kEmAltitudeAxisM[altitude_index];
    cell.altitude_upper_m = kEmAltitudeAxisM[altitude_index + 1U];
    cell.mask_corners[0] =
        kEmSoundnessMask[GridIndex(speed_index, altitude_index)];
    cell.mask_corners[1] =
        kEmSoundnessMask[GridIndex(speed_index + 1U, altitude_index)];
    cell.mask_corners[2] =
        kEmSoundnessMask[GridIndex(speed_index, altitude_index + 1U)];
    cell.mask_corners[3] = kEmSoundnessMask[
        GridIndex(speed_index + 1U, altitude_index + 1U)];
    cell.trusted = AllFourCornersZero(
        kEmSoundnessMask,
        speed_index,
        altitude_index);
    cell.trusted_n = AllFourCornersZero(
        kEmSoundnessMaskN,
        speed_index,
        altitude_index);
}

double Bilinear(
    const double* const field,
    const std::size_t speed_index,
    const std::size_t altitude_index,
    const double speed_mps,
    const double altitude_m) noexcept
{
    const double tv = (speed_mps - kEmSpeedAxisMps[speed_index])
        / (kEmSpeedAxisMps[speed_index + 1U]
            - kEmSpeedAxisMps[speed_index]);
    const double th = (altitude_m - kEmAltitudeAxisM[altitude_index])
        / (kEmAltitudeAxisM[altitude_index + 1U]
            - kEmAltitudeAxisM[altitude_index]);
    return field[GridIndex(speed_index, altitude_index)]
            * (1.0 - tv) * (1.0 - th)
        + field[GridIndex(speed_index + 1U, altitude_index)]
            * tv * (1.0 - th)
        + field[GridIndex(speed_index, altitude_index + 1U)]
            * (1.0 - tv) * th
        + field[GridIndex(speed_index + 1U, altitude_index + 1U)]
            * tv * th;
}

double BilinearMaskN(
    const std::size_t speed_index,
    const std::size_t altitude_index,
    const double speed_mps,
    const double altitude_m) noexcept
{
    const double tv = (speed_mps - kEmSpeedAxisMps[speed_index])
        / (kEmSpeedAxisMps[speed_index + 1U]
            - kEmSpeedAxisMps[speed_index]);
    const double th = (altitude_m - kEmAltitudeAxisM[altitude_index])
        / (kEmAltitudeAxisM[altitude_index + 1U]
            - kEmAltitudeAxisM[altitude_index]);
    return static_cast<double>(
            kEmSoundnessMaskN[GridIndex(speed_index, altitude_index)])
            * (1.0 - tv) * (1.0 - th)
        + static_cast<double>(
            kEmSoundnessMaskN[GridIndex(
                speed_index + 1U,
                altitude_index)])
            * tv * (1.0 - th)
        + static_cast<double>(
            kEmSoundnessMaskN[GridIndex(
                speed_index,
                altitude_index + 1U)])
            * (1.0 - tv) * th
        + static_cast<double>(
            kEmSoundnessMaskN[GridIndex(
                speed_index + 1U,
                altitude_index + 1U)])
            * tv * th;
}

struct CharacterizedRawInterpolation
{
    double value = 0.0;
    double mask_n = 0.0;
};

CharacterizedRawInterpolation InterpolateCharacterizedRawN(
    const double* const field,
    const std::size_t speed_index,
    const std::size_t altitude_index,
    const double speed_mps,
    const double altitude_m) noexcept
{
    // Preserve the Python numeric value and interpolated-mask diagnostic
    // exactly: compute the four weights first, then accumulate in source order.
    const double tv = (speed_mps - kEmSpeedAxisMps[speed_index])
        / (kEmSpeedAxisMps[speed_index + 1U]
            - kEmSpeedAxisMps[speed_index]);
    const double th = (altitude_m - kEmAltitudeAxisM[altitude_index])
        / (kEmAltitudeAxisM[altitude_index + 1U]
            - kEmAltitudeAxisM[altitude_index]);
    const double weights[4] =
    {
        (1.0 - tv) * (1.0 - th),
        tv * (1.0 - th),
        (1.0 - tv) * th,
        tv * th
    };
    CharacterizedRawInterpolation output{};
    output.value =
        field[GridIndex(speed_index, altitude_index)] * weights[0]
        + field[GridIndex(speed_index + 1U, altitude_index)] * weights[1]
        + field[GridIndex(speed_index, altitude_index + 1U)] * weights[2]
        + field[GridIndex(speed_index + 1U, altitude_index + 1U)]
            * weights[3];
    output.mask_n =
        static_cast<double>(
            kEmSoundnessMaskN[GridIndex(speed_index, altitude_index)])
            * weights[0]
        + static_cast<double>(
            kEmSoundnessMaskN[GridIndex(
                speed_index + 1U,
                altitude_index)])
            * weights[1]
        + static_cast<double>(
            kEmSoundnessMaskN[GridIndex(
                speed_index,
                altitude_index + 1U)])
            * weights[2]
        + static_cast<double>(
            kEmSoundnessMaskN[GridIndex(
                speed_index + 1U,
                altitude_index + 1U)])
            * weights[3];
    return output;
}

struct RegressionWorkspace
{
    std::array<double, kEmAnchorCount> weights{};
    std::array<double, kEmAnchorCount> residuals{};
    std::array<double, kEmAnchorCount> scratch{};
};

struct RegressionSystem
{
    double matrix[4][4] = {};
    double target[4] = {};
};

void AnchorRow(
    const std::size_t index,
    const double speed_mps,
    const double altitude_m,
    const double load_factor_g,
    double (&row)[4],
    double& measured_ps_mps) noexcept
{
    const std::size_t base = index * kEmAnchorWidth;
    row[0] = 1.0;
    row[1] = kEmAbAnchor[base + 1U] - speed_mps;
    row[2] = kEmAbAnchor[base] - altitude_m;
    row[3] = kEmAbAnchor[base + 2U] - load_factor_g;
    measured_ps_mps = kEmAbAnchor[base + 3U];
}

double Sum(const std::array<double, kEmAnchorCount>& values) noexcept
{
    double sum = 0.0;
    for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
    {
        sum += values[index];
    }
    return sum;
}

RegressionSystem BuildRegressionSystem(
    const std::array<double, kEmAnchorCount>& weights,
    const double speed_mps,
    const double altitude_m,
    const double load_factor_g) noexcept
{
    RegressionSystem system{};
    for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
    {
        double row[4] = {};
        double measured_ps_mps = 0.0;
        AnchorRow(
            index,
            speed_mps,
            altitude_m,
            load_factor_g,
            row,
            measured_ps_mps);
        for (std::size_t left = 0U; left < 4U; ++left)
        {
            const double weighted_left = row[left] * weights[index];
            system.target[left] += weighted_left * measured_ps_mps;
            for (std::size_t right = 0U; right < 4U; ++right)
            {
                system.matrix[left][right] += weighted_left * row[right];
            }
        }
    }
    for (std::size_t diagonal = 0U; diagonal < 4U; ++diagonal)
    {
        system.matrix[diagonal][diagonal] += kRegressionRidge;
    }
    return system;
}

bool SolveRegression(
    RegressionSystem system,
    double (&solution)[4]) noexcept
{
    // np.linalg.solve dispatches this 4x4 system to LAPACK dgesv.  Preserve
    // its unblocked DGETF2/DGETRS operation order: factor the matrix without
    // touching b, scale each multiplier by one reciprocal, apply rank-one
    // updates column-major, pivot b, then perform column-update triangular
    // solves.  A conventional in-place Gaussian solve is mathematically
    // equivalent but changes the final Ps mass correction by up to a few ULP.
    std::size_t pivots[4] = {0U, 0U, 0U, 0U};
    for (std::size_t column = 0U; column < 4U; ++column)
    {
        std::size_t pivot_row = column;
        double pivot_magnitude = std::fabs(system.matrix[column][column]);
        for (std::size_t row = column + 1U; row < 4U; ++row)
        {
            const double candidate = std::fabs(system.matrix[row][column]);
            if (candidate > pivot_magnitude)
            {
                pivot_magnitude = candidate;
                pivot_row = row;
            }
        }
        if (!std::isfinite(pivot_magnitude)
            || pivot_magnitude <= (std::numeric_limits<double>::min)())
        {
            return false;
        }
        pivots[column] = pivot_row;
        if (pivot_row != column)
        {
            for (std::size_t entry = 0U; entry < 4U; ++entry)
            {
                std::swap(
                    system.matrix[column][entry],
                    system.matrix[pivot_row][entry]);
            }
        }
        if (column < 3U)
        {
            const double reciprocal = 1.0 / system.matrix[column][column];
            for (std::size_t row = column + 1U; row < 4U; ++row)
            {
                system.matrix[row][column] *= reciprocal;
            }
            for (std::size_t entry = column + 1U; entry < 4U; ++entry)
            {
                for (std::size_t row = column + 1U; row < 4U; ++row)
                {
                    system.matrix[row][entry] -=
                        system.matrix[row][column]
                        * system.matrix[column][entry];
                }
            }
        }
    }

    for (std::size_t column = 0U; column < 4U; ++column)
    {
        if (pivots[column] != column)
        {
            std::swap(system.target[column], system.target[pivots[column]]);
        }
    }
    for (std::size_t column = 0U; column < 4U; ++column)
    {
        for (std::size_t row = column + 1U; row < 4U; ++row)
        {
            system.target[row] -=
                system.matrix[row][column] * system.target[column];
        }
    }
    for (std::size_t reverse = 0U; reverse < 4U; ++reverse)
    {
        const std::size_t column = 3U - reverse;
        const double diagonal = system.matrix[column][column];
        if (!std::isfinite(diagonal)
            || std::fabs(diagonal) <= (std::numeric_limits<double>::min)())
        {
            return false;
        }
        system.target[column] /= diagonal;
        if (!std::isfinite(system.target[column]))
        {
            return false;
        }
        for (std::size_t row = 0U; row < column; ++row)
        {
            system.target[row] -=
                system.matrix[row][column] * system.target[column];
        }
    }
    for (std::size_t index = 0U; index < 4U; ++index)
    {
        solution[index] = system.target[index];
    }
    return true;
}

double Median(std::array<double, kEmAnchorCount>& values) noexcept
{
    std::sort(values.begin(), values.end());
    const std::size_t upper = kEmAnchorCount / 2U;
    return (values[upper - 1U] + values[upper]) / 2.0;
}

bool FitLocalMean(
    const double speed_mps,
    const double altitude_m,
    const double load_factor_g,
    RegressionWorkspace& workspace,
    double& mean_mps) noexcept
{
    for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
    {
        double row[4] = {};
        double measured_ps_mps = 0.0;
        AnchorRow(
            index,
            speed_mps,
            altitude_m,
            load_factor_g,
            row,
            measured_ps_mps);
        const double scaled_speed = row[1] / kEmRegressionBandwidth[0];
        const double scaled_altitude = row[2] / kEmRegressionBandwidth[1];
        const double scaled_load = row[3] / kEmRegressionBandwidth[2];
        workspace.weights[index] = std::exp(
            -0.5 * (scaled_speed * scaled_speed
                + scaled_altitude * scaled_altitude
                + scaled_load * scaled_load));
    }
    if (!(Sum(workspace.weights) > 1.0e-12))
    {
        return false;
    }

    double coefficients[4] = {};
    if (!SolveRegression(
        BuildRegressionSystem(
            workspace.weights,
            speed_mps,
            altitude_m,
            load_factor_g),
        coefficients))
    {
        return false;
    }

    for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
    {
        double row[4] = {};
        double measured_ps_mps = 0.0;
        AnchorRow(
            index,
            speed_mps,
            altitude_m,
            load_factor_g,
            row,
            measured_ps_mps);
        workspace.residuals[index] = measured_ps_mps
            - (row[0] * coefficients[0]
                + row[1] * coefficients[1]
                + row[2] * coefficients[2]
                + row[3] * coefficients[3]);
        workspace.scratch[index] = workspace.residuals[index];
    }
    const double residual_median = Median(workspace.scratch);
    for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
    {
        workspace.scratch[index] = std::fabs(
            workspace.residuals[index] - residual_median);
    }
    const double median_absolute_deviation = Median(workspace.scratch);
    const double robust_scale =
        1.4826 * median_absolute_deviation + 1.0e-9;
    for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
    {
        const double normalized = workspace.residuals[index]
            / (kTukeyC * robust_scale);
        double robust_weight = 0.0;
        if (std::fabs(normalized) < 1.0)
        {
            const double base = 1.0 - normalized * normalized;
            robust_weight = base * base;
        }
        workspace.scratch[index] =
            workspace.weights[index] * robust_weight;
    }
    if (Sum(workspace.scratch) > 1.0e-12)
    {
        double robust_coefficients[4] = {};
        if (SolveRegression(
            BuildRegressionSystem(
                workspace.scratch,
                speed_mps,
                altitude_m,
                load_factor_g),
            robust_coefficients))
        {
            for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
            {
                workspace.weights[index] = workspace.scratch[index];
            }
            for (std::size_t index = 0U; index < 4U; ++index)
            {
                coefficients[index] = robust_coefficients[index];
            }
        }
    }

    double weight_sum = 0.0;
    double weight_square_sum = 0.0;
    for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
    {
        weight_sum += workspace.weights[index];
        weight_square_sum +=
            workspace.weights[index] * workspace.weights[index];
    }
    const double effective_samples = weight_sum * weight_sum
        / (std::max)(weight_square_sum, 1.0e-30);
    if (effective_samples < kMinimumEffectiveSamples)
    {
        double weighted_sum = 0.0;
        for (std::size_t index = 0U; index < kEmAnchorCount; ++index)
        {
            const double measured_ps_mps =
                kEmAbAnchor[index * kEmAnchorWidth + 3U];
            weighted_sum += workspace.weights[index] * measured_ps_mps;
        }
        mean_mps = weighted_sum / (std::max)(weight_sum, 1.0e-30);
    }
    else
    {
        mean_mps = coefficients[0];
    }
    return std::isfinite(mean_mps);
}

bool LocalLoadCorrection(
    const double speed_mps,
    const double altitude_m,
    const double effective_load_g,
    double& correction_mps) noexcept
{
    correction_mps = 0.0;
    RegressionWorkspace workspace{};
    double effective_mean = 0.0;
    const bool effective_valid = FitLocalMean(
        speed_mps,
        altitude_m,
        effective_load_g,
        workspace,
        effective_mean);
    double reference_mean = 0.0;
    const bool reference_valid = FitLocalMean(
        speed_mps,
        altitude_m,
        1.0,
        workspace,
        reference_mean);
    const double difference = effective_mean - reference_mean;
    if (!effective_valid || !reference_valid || !std::isfinite(difference))
    {
        return false;
    }
    correction_mps = difference;
    return true;
}

bool BasicCornerTableAvailable() noexcept
{
    return StrictlyIncreasing(kEmAltitudeAxisM)
        && PositiveArray(kEmCornerInstantaneousMps, kEmAltitudeCount)
        && PositiveArray(kEmCornerSustainedMps, kEmAltitudeCount);
}

bool BasicEnergyResolutionTableAvailable() noexcept
{
    // The Python provider's first gate is construction availability, not the
    // later speed-axis length check.  Compiled data has no runtime loader, so
    // the generated source/provenance receipts are its construction receipt.
    return NonEmpty(kEmSourceSha256)
        && NonEmpty(kEmEnergyResolutionProvenance);
}

std::size_t PublishedSpeedCount() noexcept
{
    // Keep this non-constexpr so the explicit degenerate-axis branch below is
    // preserved as a branch in strict warning-enabled C++14 builds.
    return kEmSpeedCount;
}

double InterpolateCorner(
    const double* const values,
    const double altitude_m) noexcept
{
    if (altitude_m == kEmAltitudeAxisM[0])
    {
        return values[0];
    }
    if (altitude_m == kEmAltitudeAxisM[kEmAltitudeCount - 1U])
    {
        return values[kEmAltitudeCount - 1U];
    }
    const double* const right = std::lower_bound(
        kEmAltitudeAxisM,
        kEmAltitudeAxisM + kEmAltitudeCount,
        altitude_m);
    const std::size_t right_index =
        static_cast<std::size_t>(right - kEmAltitudeAxisM);
    if (*right == altitude_m)
    {
        return values[right_index];
    }
    const std::size_t left_index = right_index - 1U;
    const double slope =
        (values[right_index] - values[left_index])
        / (kEmAltitudeAxisM[right_index] - kEmAltitudeAxisM[left_index]);
    return slope * (altitude_m - kEmAltitudeAxisM[left_index])
        + values[left_index];
}

LadyLuck::guidance::em::MergeCornerInterval CornerInterval(
    const double altitude_m,
    const double* const values) noexcept
{
    using LadyLuck::guidance::em::CornerIntervalStatus;
    using LadyLuck::guidance::em::MergeCornerInterval;
    MergeCornerInterval result{};
    if (!BasicCornerTableAvailable())
    {
        result.status = CornerIntervalStatus::TableUnavailable;
        return result;
    }
    if (!std::isfinite(altitude_m))
    {
        result.status = CornerIntervalStatus::AltitudeNotFinite;
        return result;
    }
    if (altitude_m < kEmAltitudeAxisM[0]
        || altitude_m > kEmAltitudeAxisM[kEmAltitudeCount - 1U])
    {
        result.status = CornerIntervalStatus::AltitudeOutsidePublishedAxis;
        return result;
    }
    const double corner_mps = InterpolateCorner(values, altitude_m);
    if (!std::isfinite(corner_mps) || corner_mps <= 0.0)
    {
        result.status = CornerIntervalStatus::PublishedCornerNotFinitePositive;
        return result;
    }
    result.status = CornerIntervalStatus::Admitted;
    result.lower_mps.has_value = true;
    result.lower_mps.value = corner_mps;
    result.upper_mps.has_value = true;
    result.upper_mps.value = corner_mps;
    return result;
}

LadyLuck::guidance::em::EnergyResolution EnergyResolutionFor(
    const double altitude_m,
    const double speed_mps) noexcept
{
    using LadyLuck::guidance::em::EnergyResolution;
    using LadyLuck::guidance::em::EnergyResolutionStatus;
    EnergyResolution result{};
    if (!BasicEnergyResolutionTableAvailable())
    {
        result.status = EnergyResolutionStatus::TableUnavailable;
        return result;
    }
    if (!std::isfinite(altitude_m) || !std::isfinite(speed_mps))
    {
        result.status = EnergyResolutionStatus::OperatingPointNotFinite;
        return result;
    }
    if (altitude_m < kEmAltitudeAxisM[0]
        || altitude_m > kEmAltitudeAxisM[kEmAltitudeCount - 1U])
    {
        result.status = EnergyResolutionStatus::AltitudeOutsidePublishedAxis;
        return result;
    }
    const std::size_t speed_count = PublishedSpeedCount();
    if (speed_count < 2U)
    {
        result.status = EnergyResolutionStatus::PublishedSpeedAxisDegenerate;
        return result;
    }
    // Validate the schema before either endpoint is indexed.  The compiled
    // competition table has many speed points, but this ordering keeps a
    // malformed/generated one-point schema from evaluating speed_count - 1
    // on the 60 Hz path before it can be classified as unavailable.
    if (speed_mps < kEmSpeedAxisMps[0]
        || speed_mps > kEmSpeedAxisMps[speed_count - 1U])
    {
        result.status = EnergyResolutionStatus::SpeedOutsidePublishedAxis;
        return result;
    }

    // Match Python's first inclusive bracket exactly.  An interior axis knot
    // therefore belongs to the interval on its left.
    std::size_t bracket = 0U;
    bool bracket_found = false;
    for (std::size_t index = 0U; index + 1U < speed_count; ++index)
    {
        if (kEmSpeedAxisMps[index] <= speed_mps
            && speed_mps <= kEmSpeedAxisMps[index + 1U])
        {
            bracket = index;
            bracket_found = true;
            break;
        }
    }
    if (!bracket_found)
    {
        result.status = EnergyResolutionStatus::SpeedGridGapUnresolved;
        return result;
    }

    // Preserve Python's (bracket-1, bracket, bracket+1) candidate order.
    std::size_t gap_indices[3] = {0U, 0U, 0U};
    std::size_t gap_count = 0U;
    if (bracket > 0U)
    {
        gap_indices[gap_count++] = bracket - 1U;
    }
    gap_indices[gap_count++] = bracket;
    if (bracket + 1U < speed_count - 1U)
    {
        gap_indices[gap_count++] = bracket + 1U;
    }
    double dv_local = kEmSpeedAxisMps[gap_indices[0] + 1U]
        - kEmSpeedAxisMps[gap_indices[0]];
    for (std::size_t index = 1U; index < gap_count; ++index)
    {
        const std::size_t gap_index = gap_indices[index];
        const double gap = kEmSpeedAxisMps[gap_index + 1U]
            - kEmSpeedAxisMps[gap_index];
        dv_local = (std::max)(dv_local, gap);
    }
    if (!std::isfinite(dv_local) || dv_local <= 0.0)
    {
        result.status =
            EnergyResolutionStatus::PublishedGridGapNotFinitePositive;
        return result;
    }

    const double resolution_m =
        speed_mps * dv_local / kStandardGravityMps2;
    if (!std::isfinite(resolution_m) || resolution_m <= 0.0)
    {
        result.status = EnergyResolutionStatus::ResolutionNotFinitePositive;
        return result;
    }
    result.status = EnergyResolutionStatus::Admitted;
    result.resolution_m = LadyLuck::guidance::em::EmValue{true, resolution_m};
    return result;
}

bool EvaluatePursuitOvershootCandidate(
    const LadyLuck::guidance::em::StrictEnergyManeuverEnvelope& envelope,
    const std::size_t altitude_index,
    const double clamped_altitude_m,
    const double mass_ratio,
    const double candidate_mps,
    double& best_mps2,
    bool& band_trusted) noexcept
{
    const double clamped_speed_mps = (std::max)(
        kEmSpeedAxisMps[0],
        (std::min)(candidate_mps,
            kEmSpeedAxisMps[kEmSpeedCount - 1U]));
    const std::size_t speed_index = CellIndex(
        kEmSpeedAxisMps,
        kEmSpeedCount,
        clamped_speed_mps);
    const double load_g = Bilinear(
        kEmNInstPubG,
        speed_index,
        altitude_index,
        clamped_speed_mps,
        clamped_altitude_m) / mass_ratio;
    const double ps_min_mps = Bilinear(
        kEmPsMinPubMps,
        speed_index,
        altitude_index,
        clamped_speed_mps,
        clamped_altitude_m) / mass_ratio;
    LadyLuck::guidance::em::EmCellTrustReceipt trust{};
    envelope.ObserveCellTrust(clamped_speed_mps, clamped_altitude_m, trust);
    if (!trust.lookup_valid())
    {
        return false;
    }
    if (!trust.cell.trusted || !trust.cell.trusted_n)
    {
        band_trusted = false;
    }
    if (!std::isfinite(load_g) || !std::isfinite(ps_min_mps))
    {
        return false;
    }
    const double bounded_load_g = (std::max)(load_g, 1.0);
    const double lateral_mps2 = kStandardGravityMps2
        * std::sqrt(bounded_load_g * bounded_load_g - 1.0);
    const double deceleration_mps2 = kStandardGravityMps2
        * (std::max)(0.0, -ps_min_mps) / candidate_mps;
    const double arrest_mps2 = lateral_mps2 + deceleration_mps2;
    if (!std::isfinite(arrest_mps2))
    {
        return false;
    }
    best_mps2 = (std::max)(best_mps2, arrest_mps2);
    return true;
}
}

namespace LadyLuck
{
namespace guidance
{
namespace em
{
const char* StrictEmStatusText(const StrictEmStatus status) noexcept
{
    switch (status)
    {
    case StrictEmStatus::LookupTrusted:
        return "lookup_trusted";
    case StrictEmStatus::TableIdentityMismatch:
        return "table_identity_mismatch";
    case StrictEmStatus::TableSchemaOrProvenanceInvalid:
        return "table_schema_or_provenance_invalid";
    case StrictEmStatus::StateOrMassInvalid:
        return "state_or_mass_invalid";
    case StrictEmStatus::ConfigurationUnverifiedOrMismatch:
        return "configuration_unverified_or_mismatch";
    case StrictEmStatus::OperatingPointOutsideAxes:
        return "operating_point_outside_axes";
    case StrictEmStatus::SoundnessCellNotComplete:
        return "soundness_cell_not_complete";
    case StrictEmStatus::PublishedLookupFailed:
        return "published_lookup_failed";
    case StrictEmStatus::PublishedLookupNonfinite:
        return "published_lookup_nonfinite";
    case StrictEmStatus::PublishedLookupSemanticsInvalid:
        return "published_lookup_semantics_invalid";
    default:
        return "unknown_strict_em_status";
    }
}

const char* PublishedSustainedNStatusText(
    const PublishedSustainedNStatus status) noexcept
{
    switch (status)
    {
    case PublishedSustainedNStatus::LookupCompleted:
        return "lookup_completed";
    case PublishedSustainedNStatus::TableUnavailable:
        return "table_unavailable";
    case PublishedSustainedNStatus::StateOrMassInvalid:
        return "state_or_mass_invalid";
    case PublishedSustainedNStatus::PublishedLookupNonfinite:
        return "published_lookup_nonfinite";
    default:
        return "unknown_published_sustained_n_status";
    }
}

EmTableAuthority StrictEnergyManeuverEnvelope::Authority() noexcept
{
    return CachedAuthority();
}

void StrictEnergyManeuverEnvelope::ObserveCellTrust(
    const double speed_mps,
    const double altitude_m,
    EmCellTrustReceipt& output) const noexcept
{
    output = EmCellTrustReceipt{};
    const EmTableAuthority authority = Authority();
    if (!authority.table_identity_valid
        || !authority.schema_valid
        || !authority.provenance_valid)
    {
        output.status = EmCellTrustStatus::TableUnavailable;
        return;
    }
    if (!std::isfinite(speed_mps) || !std::isfinite(altitude_m))
    {
        output.status = EmCellTrustStatus::OperatingPointNotFinite;
        return;
    }
    if (speed_mps < kEmSpeedAxisMps[0]
        || speed_mps > kEmSpeedAxisMps[kEmSpeedCount - 1U]
        || altitude_m < kEmAltitudeAxisM[0]
        || altitude_m > kEmAltitudeAxisM[kEmAltitudeCount - 1U])
    {
        output.status = EmCellTrustStatus::OperatingPointOutsideAxes;
        return;
    }

    const std::size_t speed_index = CellIndex(
        kEmSpeedAxisMps,
        kEmSpeedCount,
        speed_mps);
    const std::size_t altitude_index = CellIndex(
        kEmAltitudeAxisM,
        kEmAltitudeCount,
        altitude_m);
    PopulateCellEvidence(speed_index, altitude_index, output.cell);
    output.status = EmCellTrustStatus::LookupCompleted;
}

EnergyManeuverCapability StrictEnergyManeuverEnvelope::Observe(
    const StrictEmInput& input) const noexcept
{
    EnergyManeuverCapability result{};
    const EmTableAuthority authority = Authority();
    result.table_identity_valid = authority.table_identity_valid;
    result.provenance_valid = authority.provenance_valid;
    result.configuration_valid = input.gear_pos_norm == 0.0
        && input.speedbrake_pos_norm == 0.0
        && input.speedbrake_valid
        && NonEmpty(input.configuration_source);
    if (!authority.table_identity_valid)
    {
        result.status = StrictEmStatus::TableIdentityMismatch;
        return result;
    }
    if (!authority.schema_valid || !authority.provenance_valid)
    {
        result.status = StrictEmStatus::TableSchemaOrProvenanceInvalid;
        return result;
    }
    if (!std::isfinite(input.speed_mps)
        || !std::isfinite(input.altitude_m)
        || !std::isfinite(input.mass_kg)
        || input.mass_kg <= 0.0
        || !input.mass_valid
        || !NonEmpty(input.mass_source))
    {
        result.status = StrictEmStatus::StateOrMassInvalid;
        return result;
    }
    if (!result.configuration_valid)
    {
        result.status = StrictEmStatus::ConfigurationUnverifiedOrMismatch;
        return result;
    }
    if (input.speed_mps < kEmSpeedAxisMps[0]
        || input.speed_mps > kEmSpeedAxisMps[kEmSpeedCount - 1U]
        || input.altitude_m < kEmAltitudeAxisM[0]
        || input.altitude_m > kEmAltitudeAxisM[kEmAltitudeCount - 1U])
    {
        result.status = StrictEmStatus::OperatingPointOutsideAxes;
        return result;
    }

    const std::size_t speed_index = CellIndex(
        kEmSpeedAxisMps,
        kEmSpeedCount,
        input.speed_mps);
    const std::size_t altitude_index = CellIndex(
        kEmAltitudeAxisM,
        kEmAltitudeCount,
        input.altitude_m);
    PopulateCellEvidence(speed_index, altitude_index, result.cell);

    const double reference_mass_kg =
        kEmReferenceWeightN / kStandardGravityMps2;
    const double mass_ratio = input.mass_kg / reference_mass_kg;
    if (!std::isfinite(mass_ratio) || mass_ratio <= 0.0)
    {
        result.status = StrictEmStatus::PublishedLookupFailed;
        return result;
    }

    // v5.2 is additive: a shared-mask refusal remains the public status and
    // keeps Ps absent.  Only a separately trusted, finite, semantically valid
    // N pair is exposed through n_channel_trusted.
    if (!result.cell.trusted)
    {
        result.status = StrictEmStatus::SoundnessCellNotComplete;
        if (!result.cell.trusted_n)
        {
            return result;
        }
        const double n_inst = Bilinear(
            kEmNInstPubG,
            speed_index,
            altitude_index,
            input.speed_mps,
            input.altitude_m) / mass_ratio;
        const double n_sustained = Bilinear(
            kEmNSustainedPubG,
            speed_index,
            altitude_index,
            input.speed_mps,
            input.altitude_m) / mass_ratio;
        if (std::isfinite(n_inst)
            && std::isfinite(n_sustained)
            && n_inst > 0.0
            && n_sustained > 0.0
            && n_inst >= n_sustained)
        {
            result.n_inst_g = EmValue{true, n_inst};
            result.n_sus_g = EmValue{true, n_sustained};
            result.n_channel_trusted = true;
        }
        return result;
    }

    const double ps_min = Bilinear(
        kEmPsMinPubMps,
        speed_index,
        altitude_index,
        input.speed_mps,
        input.altitude_m) / mass_ratio;
    double ps_max = Bilinear(
        kEmPsMaxPubMps,
        speed_index,
        altitude_index,
        input.speed_mps,
        input.altitude_m);
    if (mass_ratio != 1.0)
    {
        double correction_mps = 0.0;
        if (!LocalLoadCorrection(
                input.speed_mps,
                input.altitude_m,
                mass_ratio,
                correction_mps))
        {
            // A failed local fit is missing characterization, not a measured
            // zero correction.  Keep the lookup command-neutral instead of
            // promoting an unproven Ps/load value as trusted authority.
            result.status = StrictEmStatus::PublishedLookupFailed;
            return result;
        }
        ps_max = (ps_max + correction_mps) / mass_ratio;
    }
    const double n_inst = Bilinear(
        kEmNInstPubG,
        speed_index,
        altitude_index,
        input.speed_mps,
        input.altitude_m) / mass_ratio;
    const double n_sustained = Bilinear(
        kEmNSustainedPubG,
        speed_index,
        altitude_index,
        input.speed_mps,
        input.altitude_m) / mass_ratio;
    if (!std::isfinite(ps_min)
        || !std::isfinite(ps_max)
        || !std::isfinite(n_inst)
        || !std::isfinite(n_sustained))
    {
        result.status = StrictEmStatus::PublishedLookupNonfinite;
        return result;
    }
    if (!(ps_min < 0.0
        && n_inst > 0.0
        && n_sustained > 0.0
        && n_inst >= n_sustained))
    {
        result.status = StrictEmStatus::PublishedLookupSemanticsInvalid;
        return result;
    }

    result.ps_min_1g_idle_mps = EmValue{true, ps_min};
    result.ps_max_1g_ab_mps = EmValue{true, ps_max};
    result.n_inst_g = EmValue{true, n_inst};
    result.n_sus_g = EmValue{true, n_sustained};
    result.status = StrictEmStatus::LookupTrusted;
    result.lookup_trusted = true;
    result.n_channel_trusted = true;
    result.load_comparison_valid = std::isfinite(input.nz_g)
        && input.nz_valid
        && NonEmpty(input.nz_source);
    return result;
}

void StrictEnergyManeuverEnvelope::ObservePublishedSustainedN(
    const double speed_mps,
    const double altitude_m,
    const double mass_kg,
    PublishedSustainedNLookup& output) const noexcept
{
    output = PublishedSustainedNLookup{};
    const EmTableAuthority authority = Authority();
    if (!authority.table_identity_valid
        || !authority.schema_valid
        || !authority.provenance_valid)
    {
        output.status = PublishedSustainedNStatus::TableUnavailable;
        return;
    }
    if (!std::isfinite(speed_mps)
        || !std::isfinite(altitude_m)
        || !std::isfinite(mass_kg)
        || mass_kg <= 0.0)
    {
        output.status = PublishedSustainedNStatus::StateOrMassInvalid;
        return;
    }

    // numpy.clip is part of EMEnvelope._bilinear's public behavior.  The HABFM
    // caller separately refuses out-of-axis altitude before this lookup, but
    // preserving the lookup itself prevents this API from acquiring a subtly
    // different boundary contract.
    const double clamped_speed_mps = (std::max)(
        kEmSpeedAxisMps[0],
        (std::min)(speed_mps, kEmSpeedAxisMps[kEmSpeedCount - 1U]));
    const double clamped_altitude_m = (std::max)(
        kEmAltitudeAxisM[0],
        (std::min)(altitude_m, kEmAltitudeAxisM[kEmAltitudeCount - 1U]));
    const std::size_t speed_index = CellIndex(
        kEmSpeedAxisMps,
        kEmSpeedCount,
        clamped_speed_mps);
    const std::size_t altitude_index = CellIndex(
        kEmAltitudeAxisM,
        kEmAltitudeCount,
        clamped_altitude_m);

    const double reference_mass_kg =
        kEmReferenceWeightN / kStandardGravityMps2;
    const double mass_ratio = mass_kg / reference_mass_kg;
    const double load_factor_g = Bilinear(
        kEmNSustainedPubG,
        speed_index,
        altitude_index,
        clamped_speed_mps,
        clamped_altitude_m) / mass_ratio;
    const double interpolated_mask_n = BilinearMaskN(
        speed_index,
        altitude_index,
        clamped_speed_mps,
        clamped_altitude_m);
    if (!std::isfinite(mass_ratio)
        || mass_ratio <= 0.0
        || !std::isfinite(load_factor_g)
        || !std::isfinite(interpolated_mask_n))
    {
        output.status = PublishedSustainedNStatus::PublishedLookupNonfinite;
        return;
    }

    output.status = PublishedSustainedNStatus::LookupCompleted;
    output.load_factor_g = EmValue{true, load_factor_g};
    output.interpolated_mask_n = EmValue{true, interpolated_mask_n};
    EmCellTrustReceipt trust{};
    ObserveCellTrust(clamped_speed_mps, clamped_altitude_m, trust);
    output.trusted = trust.lookup_valid() && trust.cell.trusted_n;
}

void StrictEnergyManeuverEnvelope::ObserveCharacterizedRawN(
    const double speed_mps,
    const double altitude_m,
    const bool sustained,
    CharacterizedRawNLookup& output) const noexcept
{
    output = CharacterizedRawNLookup{};
    const EmTableAuthority authority = Authority();
    if (!authority.table_identity_valid
        || !authority.schema_valid
        || !authority.provenance_valid)
    {
        output.status = CharacterizedRawNStatus::TableUnavailable;
        return;
    }
    if (!std::isfinite(speed_mps) || !std::isfinite(altitude_m))
    {
        output.status = CharacterizedRawNStatus::OperatingPointNotFinite;
        return;
    }

    output.in_table_domain =
        speed_mps >= kEmSpeedAxisMps[0]
        && speed_mps <= kEmSpeedAxisMps[kEmSpeedCount - 1U]
        && altitude_m >= kEmAltitudeAxisM[0]
        && altitude_m <= kEmAltitudeAxisM[kEmAltitudeCount - 1U];
    const double clamped_speed_mps = (std::max)(
        kEmSpeedAxisMps[0],
        (std::min)(speed_mps, kEmSpeedAxisMps[kEmSpeedCount - 1U]));
    const double clamped_altitude_m = (std::max)(
        kEmAltitudeAxisM[0],
        (std::min)(altitude_m, kEmAltitudeAxisM[kEmAltitudeCount - 1U]));
    const std::size_t speed_index = CellIndex(
        kEmSpeedAxisMps,
        kEmSpeedCount,
        clamped_speed_mps);
    const std::size_t altitude_index = CellIndex(
        kEmAltitudeAxisM,
        kEmAltitudeCount,
        clamped_altitude_m);
    const CharacterizedRawInterpolation interpolation =
        InterpolateCharacterizedRawN(
            sustained ? kEmNSustainedRawG : kEmNInstRawG,
            speed_index,
            altitude_index,
            clamped_speed_mps,
            clamped_altitude_m);
    if (!std::isfinite(interpolation.value)
        || !std::isfinite(interpolation.mask_n))
    {
        output.status = CharacterizedRawNStatus::LookupNonfinite;
        return;
    }

    output.status = CharacterizedRawNStatus::LookupCompleted;
    output.load_factor_g = EmValue{true, interpolation.value};
    output.interpolated_mask_n = EmValue{true, interpolation.mask_n};
    EmCellTrustReceipt trust{};
    ObserveCellTrust(clamped_speed_mps, clamped_altitude_m, trust);
    output.trusted = output.in_table_domain
        && trust.lookup_valid()
        && trust.cell.trusted_n;
}

void StrictEnergyManeuverEnvelope::ObservePursuitOvershootComposite(
    const double speed_mps,
    const double altitude_m,
    const bool mass_available,
    const double mass_kg,
    PursuitOvershootEmQuery& output) const noexcept
{
    output = PursuitOvershootEmQuery{};
    const EmTableAuthority authority = Authority();
    if (!authority.table_identity_valid
        || !authority.schema_valid
        || !authority.provenance_valid)
    {
        output.status = PursuitOvershootEmStatus::TableUnavailable;
        return;
    }
    if (!std::isfinite(speed_mps) || !std::isfinite(altitude_m))
    {
        output.status = PursuitOvershootEmStatus::OperatingPointNotFinite;
        return;
    }

    const double clamped_altitude_m = (std::max)(
        kEmAltitudeAxisM[0],
        (std::min)(altitude_m, kEmAltitudeAxisM[kEmAltitudeCount - 1U]));
    const std::size_t altitude_index = CellIndex(
        kEmAltitudeAxisM,
        kEmAltitudeCount,
        clamped_altitude_m);

    // EMEnvelope.V_corner_sus uses numpy.interp, including endpoint clamps.
    const double sustained_corner_mps = InterpolateCorner(
        kEmCornerSustainedMps,
        clamped_altitude_m);
    if (std::isfinite(sustained_corner_mps))
    {
        output.sustained_corner_mps = EmValue{true, sustained_corner_mps};
    }

    const double reference_mass_kg =
        kEmReferenceWeightN / kStandardGravityMps2;
    const double mass_ratio = mass_available
        ? mass_kg / reference_mass_kg
        : 1.0;
    const double requested_load_g = mass_ratio;

    // Exact first crossing of EMEnvelope.v_stall(h, 1, mass=...).  A NaN
    // requested load intentionally compares false at every point and therefore
    // selects the final speed knot, as NumPy's nonzero(col >= NaN) path does.
    std::size_t crossing_index = kEmSpeedCount;
    double crossing_value_g = 0.0;
    double previous_value_g = 0.0;
    for (std::size_t index = 0U; index < kEmSpeedCount; ++index)
    {
        const std::size_t speed_index = CellIndex(
            kEmSpeedAxisMps,
            kEmSpeedCount,
            kEmSpeedAxisMps[index]);
        const double value_g = Bilinear(
            kEmNInstPubG,
            speed_index,
            altitude_index,
            kEmSpeedAxisMps[index],
            clamped_altitude_m);
        if (value_g >= requested_load_g)
        {
            crossing_index = index;
            crossing_value_g = value_g;
            break;
        }
        previous_value_g = value_g;
    }

    double stall_floor_mps = kEmSpeedAxisMps[kEmSpeedCount - 1U];
    if (crossing_index == 0U)
    {
        stall_floor_mps = kEmSpeedAxisMps[0];
    }
    else if (crossing_index < kEmSpeedCount)
    {
        double weight = crossing_value_g == previous_value_g
            ? 0.0
            : (requested_load_g - previous_value_g)
                / (crossing_value_g - previous_value_g);
        weight = (std::max)(0.0, (std::min)(weight, 1.0));
        stall_floor_mps = kEmSpeedAxisMps[crossing_index - 1U]
            + weight * (kEmSpeedAxisMps[crossing_index]
                - kEmSpeedAxisMps[crossing_index - 1U]);
    }
    if (std::isfinite(stall_floor_mps))
    {
        output.stall_1g_mps = EmValue{true, stall_floor_mps};
    }
    output.status = PursuitOvershootEmStatus::QueryCompleted;

    double best_mps2 = 0.0;
    bool band_trusted = true;
    if (!EvaluatePursuitOvershootCandidate(
            *this,
            altitude_index,
            clamped_altitude_m,
            mass_ratio,
            speed_mps,
            best_mps2,
            band_trusted))
    {
        output.status = PursuitOvershootEmStatus::PublishedLookupNonfinite;
        return;
    }
    if (stall_floor_mps < speed_mps)
    {
        if (!EvaluatePursuitOvershootCandidate(
                *this,
                altitude_index,
                clamped_altitude_m,
                mass_ratio,
                stall_floor_mps,
                best_mps2,
                band_trusted))
        {
            output.status = PursuitOvershootEmStatus::PublishedLookupNonfinite;
            return;
        }
        for (std::size_t index = 0U; index < kEmSpeedCount; ++index)
        {
            const double candidate_mps = kEmSpeedAxisMps[index];
            if (candidate_mps > stall_floor_mps
                && candidate_mps < speed_mps
                && !EvaluatePursuitOvershootCandidate(
                    *this,
                    altitude_index,
                    clamped_altitude_m,
                    mass_ratio,
                    candidate_mps,
                    best_mps2,
                    band_trusted))
            {
                output.status =
                    PursuitOvershootEmStatus::PublishedLookupNonfinite;
                return;
            }
        }
    }
    output.optimistic_arrest_mps2 = EmValue{true, best_mps2};
    output.arrest_band_trusted = band_trusted;
}

const char* CornerIntervalStatusText(
    const CornerIntervalStatus status) noexcept
{
    switch (status)
    {
    case CornerIntervalStatus::Admitted:
        return "interval_admitted";
    case CornerIntervalStatus::TableUnavailable:
        return "table_unavailable";
    case CornerIntervalStatus::AltitudeNotFinite:
        return "altitude_not_finite";
    case CornerIntervalStatus::AltitudeOutsidePublishedAxis:
        return "altitude_outside_published_axis";
    case CornerIntervalStatus::PublishedCornerNotFinitePositive:
        return "published_corner_not_finite_positive";
    default:
        return "unknown_corner_interval_status";
    }
}

const char* EnergyResolutionStatusText(
    const EnergyResolutionStatus status) noexcept
{
    switch (status)
    {
    case EnergyResolutionStatus::Admitted:
        return kEmEnergyResolutionProvenance;
    case EnergyResolutionStatus::TableUnavailable:
        return "table_unavailable";
    case EnergyResolutionStatus::OperatingPointNotFinite:
        return "operating_point_not_finite";
    case EnergyResolutionStatus::AltitudeOutsidePublishedAxis:
        return "altitude_outside_published_axis";
    case EnergyResolutionStatus::SpeedOutsidePublishedAxis:
        return "speed_outside_published_axis";
    case EnergyResolutionStatus::PublishedSpeedAxisDegenerate:
        return "published_speed_axis_degenerate";
    case EnergyResolutionStatus::SpeedGridGapUnresolved:
        return "speed_grid_gap_unresolved";
    case EnergyResolutionStatus::PublishedGridGapNotFinitePositive:
        return "published_grid_gap_not_finite_positive";
    case EnergyResolutionStatus::ResolutionNotFinitePositive:
        return "resolution_not_finite_positive";
    default:
        return "unknown_energy_resolution_status";
    }
}

MergeCornerInterval MergeIntentCornerProvider::InstantaneousInterval(
    const double altitude_m) const noexcept
{
    return CornerInterval(altitude_m, kEmCornerInstantaneousMps);
}

MergeCornerInterval MergeIntentCornerProvider::SustainedInterval(
    const double altitude_m) const noexcept
{
    return CornerInterval(altitude_m, kEmCornerSustainedMps);
}

EnergyResolution MergeIntentCornerProvider::EnergyResolutionAt(
    const double altitude_m,
    const double speed_mps) const noexcept
{
    return EnergyResolutionFor(altitude_m, speed_mps);
}
}
}
}
