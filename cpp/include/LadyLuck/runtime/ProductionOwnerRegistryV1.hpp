#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace runtime
{

constexpr std::uint32_t ProductionOwnerRegistryV1MaxOwners = 128U;

enum class ProductionOwnerRegistryV1Code : std::int32_t
{
    Created = 1,
    AlreadyExistsSameForce = 2,
    Found = 3,
    Removed = 4,
    Reset = 5,
    ForceMismatch = -1,
    InvalidOwner = -2,
    InvalidForce = -3,
    NotFound = -4
};

struct ProductionOwnerRecordV1
{
    std::int32_t owner_id = -1;
    std::int32_t force_side = 0;
    std::uint32_t generation = 0U;
    std::uint8_t occupied = 0U;
    std::uint8_t reserved[3]{};
};

using ProductionOwnerRecordArrayV1 = std::array<
    ProductionOwnerRecordV1,
    static_cast<std::size_t>(ProductionOwnerRegistryV1MaxOwners)>;

struct ProductionOwnerRegistryStorageV1
{
    ProductionOwnerRecordArrayV1 records{};
    std::uint32_t generation = 0U;
    std::uint32_t occupied_count = 0U;
};

struct ProductionOwnerRegistryResultV1
{
    ProductionOwnerRegistryV1Code code =
        ProductionOwnerRegistryV1Code::NotFound;
    ProductionOwnerRecordV1 record{};
    std::uint32_t registry_generation = 0U;
    std::uint32_t occupied_count = 0U;
};

class ProductionOwnerRegistryV1 final
{
public:
    static constexpr std::uint32_t MaxOwners =
        ProductionOwnerRegistryV1MaxOwners;

    ProductionOwnerRegistryResultV1 Create(
        const std::int32_t owner_id,
        const std::int32_t force_side) noexcept
    {
        const ProductionOwnerRegistryResultV1 inspection =
            InspectCreate(owner_id, force_side);
        if (inspection.code != ProductionOwnerRegistryV1Code::NotFound)
        {
            return inspection;
        }

        ProductionOwnerRecordV1& record =
            storage_.records[static_cast<std::size_t>(owner_id)];
        storage_.generation = NextGeneration(storage_.generation);
        record.owner_id = owner_id;
        record.force_side = force_side;
        record.generation = storage_.generation;
        record.occupied = 1U;
        ++storage_.occupied_count;
        return Result(ProductionOwnerRegistryV1Code::Created, record);
    }

    ProductionOwnerRegistryResultV1 InspectCreate(
        const std::int32_t owner_id,
        const std::int32_t force_side) const noexcept
    {
        if (!ValidOwner(owner_id))
        {
            return Result(ProductionOwnerRegistryV1Code::InvalidOwner);
        }
        if (!ValidForce(force_side))
        {
            return Result(ProductionOwnerRegistryV1Code::InvalidForce);
        }

        const ProductionOwnerRecordV1& record =
            storage_.records[static_cast<std::size_t>(owner_id)];
        if (record.occupied != 0U)
        {
            return Result(
                record.force_side == force_side
                    ? ProductionOwnerRegistryV1Code::AlreadyExistsSameForce
                    : ProductionOwnerRegistryV1Code::ForceMismatch,
                record);
        }
        return Result(ProductionOwnerRegistryV1Code::NotFound);
    }

    ProductionOwnerRegistryResultV1 Find(
        const std::int32_t owner_id) const noexcept
    {
        if (!ValidOwner(owner_id))
        {
            return Result(ProductionOwnerRegistryV1Code::InvalidOwner);
        }

        const ProductionOwnerRecordV1& record =
            storage_.records[static_cast<std::size_t>(owner_id)];
        return record.occupied != 0U
            ? Result(ProductionOwnerRegistryV1Code::Found, record)
            : Result(ProductionOwnerRegistryV1Code::NotFound);
    }

    ProductionOwnerRegistryResultV1 Remove(
        const std::int32_t owner_id) noexcept
    {
        if (!ValidOwner(owner_id))
        {
            return Result(ProductionOwnerRegistryV1Code::InvalidOwner);
        }

        ProductionOwnerRecordV1& record =
            storage_.records[static_cast<std::size_t>(owner_id)];
        if (record.occupied == 0U)
        {
            return Result(ProductionOwnerRegistryV1Code::NotFound);
        }

        const ProductionOwnerRecordV1 removed = record;
        storage_.generation = NextGeneration(storage_.generation);
        record = ProductionOwnerRecordV1{};
        --storage_.occupied_count;
        return Result(ProductionOwnerRegistryV1Code::Removed, removed);
    }

    ProductionOwnerRegistryResultV1 Reset() noexcept
    {
        const std::uint32_t next_generation =
            NextGeneration(storage_.generation);
        storage_ = ProductionOwnerRegistryStorageV1{};
        storage_.generation = next_generation;
        return Result(ProductionOwnerRegistryV1Code::Reset);
    }

    std::uint32_t Count() const noexcept
    {
        return storage_.occupied_count;
    }

    std::uint32_t Generation() const noexcept
    {
        return storage_.generation;
    }

private:
    static bool ValidOwner(const std::int32_t owner_id) noexcept
    {
        return owner_id >= 0
            && static_cast<std::uint32_t>(owner_id) < MaxOwners;
    }

    // V1 observation/runtime admission reserves zero as "force unavailable".
    static bool ValidForce(const std::int32_t force_side) noexcept
    {
        return force_side != 0;
    }

    static std::uint32_t NextGeneration(
        const std::uint32_t generation) noexcept
    {
        return generation == UINT32_MAX ? 1U : generation + 1U;
    }

    ProductionOwnerRegistryResultV1 Result(
        const ProductionOwnerRegistryV1Code code) const noexcept
    {
        ProductionOwnerRegistryResultV1 result{};
        result.code = code;
        result.registry_generation = storage_.generation;
        result.occupied_count = storage_.occupied_count;
        return result;
    }

    ProductionOwnerRegistryResultV1 Result(
        const ProductionOwnerRegistryV1Code code,
        const ProductionOwnerRecordV1& record) const noexcept
    {
        ProductionOwnerRegistryResultV1 result = Result(code);
        result.record = record;
        return result;
    }

    ProductionOwnerRegistryStorageV1 storage_{};
};

static_assert(
    ProductionOwnerRegistryV1::MaxOwners == 128U,
    "Production owner domain changed.");
static_assert(
    std::tuple_size<ProductionOwnerRecordArrayV1>::value == 128U,
    "Production owner registry capacity changed.");

static_assert(
    std::is_standard_layout<ProductionOwnerRecordV1>::value,
    "Production owner record must be standard-layout.");
static_assert(
    std::is_trivially_copyable<ProductionOwnerRecordV1>::value,
    "Production owner record must be trivially copyable.");
static_assert(sizeof(ProductionOwnerRecordV1) == 16U, "Owner record size changed.");
static_assert(alignof(ProductionOwnerRecordV1) == 4U, "Owner record alignment changed.");
static_assert(offsetof(ProductionOwnerRecordV1, owner_id) == 0U, "Owner record layout changed.");
static_assert(offsetof(ProductionOwnerRecordV1, force_side) == 4U, "Owner record layout changed.");
static_assert(offsetof(ProductionOwnerRecordV1, generation) == 8U, "Owner record layout changed.");
static_assert(offsetof(ProductionOwnerRecordV1, occupied) == 12U, "Owner record layout changed.");
static_assert(offsetof(ProductionOwnerRecordV1, reserved) == 13U, "Owner record layout changed.");

static_assert(
    std::is_standard_layout<ProductionOwnerRegistryStorageV1>::value,
    "Production owner storage must be standard-layout.");
static_assert(
    std::is_trivially_copyable<ProductionOwnerRegistryStorageV1>::value,
    "Production owner storage must be trivially copyable.");
static_assert(sizeof(ProductionOwnerRegistryStorageV1) == 2056U, "Owner storage size changed.");
static_assert(alignof(ProductionOwnerRegistryStorageV1) == 4U, "Owner storage alignment changed.");
static_assert(offsetof(ProductionOwnerRegistryStorageV1, records) == 0U, "Owner storage layout changed.");
static_assert(offsetof(ProductionOwnerRegistryStorageV1, generation) == 2048U, "Owner storage layout changed.");
static_assert(offsetof(ProductionOwnerRegistryStorageV1, occupied_count) == 2052U, "Owner storage layout changed.");

static_assert(
    std::is_standard_layout<ProductionOwnerRegistryResultV1>::value,
    "Production owner result must be standard-layout.");
static_assert(
    std::is_trivially_copyable<ProductionOwnerRegistryResultV1>::value,
    "Production owner result must be trivially copyable.");
static_assert(sizeof(ProductionOwnerRegistryResultV1) == 28U, "Owner result size changed.");
static_assert(alignof(ProductionOwnerRegistryResultV1) == 4U, "Owner result alignment changed.");
static_assert(offsetof(ProductionOwnerRegistryResultV1, code) == 0U, "Owner result layout changed.");
static_assert(offsetof(ProductionOwnerRegistryResultV1, record) == 4U, "Owner result layout changed.");
static_assert(offsetof(ProductionOwnerRegistryResultV1, registry_generation) == 20U, "Owner result layout changed.");
static_assert(offsetof(ProductionOwnerRegistryResultV1, occupied_count) == 24U, "Owner result layout changed.");

static_assert(
    std::is_standard_layout<ProductionOwnerRegistryV1>::value,
    "Production owner registry must be standard-layout.");
static_assert(
    std::is_trivially_copyable<ProductionOwnerRegistryV1>::value,
    "Production owner registry must be trivially copyable.");
static_assert(sizeof(ProductionOwnerRegistryV1) == 2056U, "Owner registry size changed.");
static_assert(alignof(ProductionOwnerRegistryV1) == 4U, "Owner registry alignment changed.");

} // namespace runtime
} // namespace LadyLuck
