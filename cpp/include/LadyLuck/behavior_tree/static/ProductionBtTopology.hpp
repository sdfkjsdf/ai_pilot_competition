#pragma once

#include "LadyLuck/behavior_tree/static/StaticBtResult.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace LadyLuck
{
namespace behavior_tree
{
namespace static_bt
{

enum class ProductionBtNodeKind : std::uint8_t
{
    Root = 0U,
    Sequence = 1U,
    Fallback = 2U,
    Dispatcher = 3U,
    Service = 4U,
    Condition = 5U,
    CommandTask = 6U,
    LifecycleOwner = 7U
};

enum class ProductionBtTopologyReason : BtReasonId
{
    None = 0U,
    EmptyTopology = 1U,
    RootIndexOutOfRange = 2U,
    RootKindMismatch = 3U,
    InvalidNodeIdentity = 4U,
    DuplicateNodeIdentity = 5U,
    ChildSpanOutOfRange = 6U,
    BranchHasNoChildren = 7U,
    LeafHasChildren = 8U,
    BranchOwnsWriter = 9U,
    CommandOwnerMissingWriter = 10U,
    NonCommandLeafOwnsWriter = 11U,
    ChildIndexOutOfRange = 12U,
    DuplicateWriterIdentity = 13U,
    RootHasParent = 14U,
    MultipleParents = 15U,
    Cycle = 16U,
    Unreachable = 17U,
    InvalidKind = 18U
};

struct ProductionBtNodeDescriptor
{
    BtNodeId node_id = BtNodeIdInvalid;
    BtStageId stage_id = BtStageIdInvalid;
    ProductionBtNodeKind kind = ProductionBtNodeKind::Condition;
    BtNodeIndex first_child = 0U;
    std::uint32_t child_count = 0U;
    BtWriterId writer_id = BtWriterIdNone;
};

template <std::size_t NodeCount, std::size_t ChildCount>
struct ProductionBtTopology
{
    static_assert(NodeCount > 0U,
                  "ProductionBtTopology requires at least one node");

    std::array<ProductionBtNodeDescriptor, NodeCount> nodes{};
    std::array<BtNodeIndex, ChildCount> child_indices{};
    BtNodeIndex root_index = 0U;
};

struct ProductionBtTopologyValidation
{
    bool valid = false;
    BtTickResult diagnostic{};
};

constexpr BtReasonId ToReasonId(
    const ProductionBtTopologyReason reason) noexcept
{
    return static_cast<BtReasonId>(reason);
}

constexpr bool IsBranchKind(const ProductionBtNodeKind kind) noexcept
{
    return kind == ProductionBtNodeKind::Root ||
           kind == ProductionBtNodeKind::Sequence ||
           kind == ProductionBtNodeKind::Fallback ||
           kind == ProductionBtNodeKind::Dispatcher;
}

constexpr bool IsCommandOwnerKind(const ProductionBtNodeKind kind) noexcept
{
    return kind == ProductionBtNodeKind::CommandTask ||
           kind == ProductionBtNodeKind::LifecycleOwner;
}

constexpr bool IsKnownNodeKind(const ProductionBtNodeKind kind) noexcept
{
    return IsBranchKind(kind) || kind == ProductionBtNodeKind::Service ||
           kind == ProductionBtNodeKind::Condition ||
           IsCommandOwnerKind(kind);
}

constexpr ProductionBtTopologyValidation MakeTopologyFailure(
    const BtReturnCode code,
    const ProductionBtNodeDescriptor& node,
    const ProductionBtTopologyReason reason) noexcept
{
    return ProductionBtTopologyValidation{
        false,
        MakeBtTickResult(code,
                         node.node_id,
                         node.stage_id,
                         0U,
                         ToReasonId(reason))};
}

constexpr ProductionBtTopologyValidation MakeRootTopologyFailure(
    const BtReturnCode code,
    const ProductionBtTopologyReason reason) noexcept
{
    return ProductionBtTopologyValidation{
        false,
        MakeBtTickResult(code,
                         BtNodeIdInvalid,
                         BtStageIdInvalid,
                         0U,
                         ToReasonId(reason))};
}

template <std::size_t NodeCount, std::size_t ChildCount>
constexpr ProductionBtTopologyValidation ValidateProductionBtTopology(
    const ProductionBtTopology<NodeCount, ChildCount>& topology) noexcept
{
    if (topology.root_index >= NodeCount)
    {
        return MakeRootTopologyFailure(
            BtReturnCode::InvalidRoot,
            ProductionBtTopologyReason::RootIndexOutOfRange);
    }

    const ProductionBtNodeDescriptor& root = topology.nodes[topology.root_index];
    if (root.kind != ProductionBtNodeKind::Root)
    {
        return MakeTopologyFailure(BtReturnCode::InvalidRoot,
                                   root,
                                   ProductionBtTopologyReason::RootKindMismatch);
    }

    for (std::size_t i = 0U; i < NodeCount; ++i)
    {
        const ProductionBtNodeDescriptor& node = topology.nodes[i];
        if (node.node_id == BtNodeIdInvalid ||
            node.stage_id == BtStageIdInvalid)
        {
            return MakeTopologyFailure(
                BtReturnCode::InvalidNode,
                node,
                ProductionBtTopologyReason::InvalidNodeIdentity);
        }
        if (!IsKnownNodeKind(node.kind))
        {
            return MakeTopologyFailure(BtReturnCode::InvalidNodeKind,
                                       node,
                                       ProductionBtTopologyReason::InvalidKind);
        }
        for (std::size_t prior = 0U; prior < i; ++prior)
        {
            if (topology.nodes[prior].node_id == node.node_id)
            {
                return MakeTopologyFailure(
                    BtReturnCode::DuplicateNodeId,
                    node,
                    ProductionBtTopologyReason::DuplicateNodeIdentity);
            }
        }

        if (node.first_child > ChildCount ||
            node.child_count > ChildCount - node.first_child)
        {
            return MakeTopologyFailure(
                BtReturnCode::InvalidChild,
                node,
                ProductionBtTopologyReason::ChildSpanOutOfRange);
        }

        if (IsBranchKind(node.kind))
        {
            if (node.child_count == 0U)
            {
                return MakeTopologyFailure(
                    BtReturnCode::InvalidTopology,
                    node,
                    ProductionBtTopologyReason::BranchHasNoChildren);
            }
            if (node.writer_id != BtWriterIdNone)
            {
                return MakeTopologyFailure(
                    BtReturnCode::WriterContractFault,
                    node,
                    ProductionBtTopologyReason::BranchOwnsWriter);
            }
        }
        else
        {
            if (node.child_count != 0U)
            {
                return MakeTopologyFailure(
                    BtReturnCode::InvalidTopology,
                    node,
                    ProductionBtTopologyReason::LeafHasChildren);
            }
            if (IsCommandOwnerKind(node.kind))
            {
                if (node.writer_id == BtWriterIdNone)
                {
                    return MakeTopologyFailure(
                        BtReturnCode::WriterContractFault,
                        node,
                        ProductionBtTopologyReason::CommandOwnerMissingWriter);
                }
            }
            else if (node.writer_id != BtWriterIdNone)
            {
                return MakeTopologyFailure(
                    BtReturnCode::WriterContractFault,
                    node,
                    ProductionBtTopologyReason::NonCommandLeafOwnsWriter);
            }
        }
    }

    for (std::size_t i = 0U; i < NodeCount; ++i)
    {
        const ProductionBtNodeDescriptor& node = topology.nodes[i];
        if (!IsCommandOwnerKind(node.kind))
        {
            continue;
        }
        for (std::size_t prior = 0U; prior < i; ++prior)
        {
            const ProductionBtNodeDescriptor& prior_node = topology.nodes[prior];
            if (IsCommandOwnerKind(prior_node.kind) &&
                prior_node.writer_id == node.writer_id)
            {
                return MakeTopologyFailure(
                    BtReturnCode::DuplicateWriter,
                    node,
                    ProductionBtTopologyReason::DuplicateWriterIdentity);
            }
        }
    }

    std::uint32_t indegree[NodeCount] = {};
    for (std::size_t parent_index = 0U; parent_index < NodeCount; ++parent_index)
    {
        const ProductionBtNodeDescriptor& parent = topology.nodes[parent_index];
        for (std::uint32_t offset = 0U; offset < parent.child_count; ++offset)
        {
            const BtNodeIndex child_index =
                topology.child_indices[parent.first_child + offset];
            if (child_index >= NodeCount)
            {
                return MakeTopologyFailure(
                    BtReturnCode::InvalidChild,
                    parent,
                    ProductionBtTopologyReason::ChildIndexOutOfRange);
            }
            ++indegree[child_index];
        }
    }

    if (indegree[topology.root_index] != 0U)
    {
        return MakeTopologyFailure(BtReturnCode::InvalidRoot,
                                   root,
                                   ProductionBtTopologyReason::RootHasParent);
    }

    // Kahn's algorithm establishes acyclicity without recursion or heap use.
    std::uint32_t remaining_indegree[NodeCount] = {};
    BtNodeIndex queue[NodeCount] = {};
    std::size_t queue_head = 0U;
    std::size_t queue_tail = 0U;
    for (std::size_t i = 0U; i < NodeCount; ++i)
    {
        remaining_indegree[i] = indegree[i];
        if (remaining_indegree[i] == 0U)
        {
            queue[queue_tail++] = static_cast<BtNodeIndex>(i);
        }
    }

    std::size_t visited_count = 0U;
    while (queue_head < queue_tail)
    {
        const BtNodeIndex parent_index = queue[queue_head++];
        ++visited_count;
        const ProductionBtNodeDescriptor& parent = topology.nodes[parent_index];
        for (std::uint32_t offset = 0U; offset < parent.child_count; ++offset)
        {
            const BtNodeIndex child_index =
                topology.child_indices[parent.first_child + offset];
            --remaining_indegree[child_index];
            if (remaining_indegree[child_index] == 0U)
            {
                queue[queue_tail++] = child_index;
            }
        }
    }
    if (visited_count != NodeCount)
    {
        return MakeRootTopologyFailure(BtReturnCode::CycleDetected,
                                       ProductionBtTopologyReason::Cycle);
    }

    for (std::size_t i = 0U; i < NodeCount; ++i)
    {
        if (i != topology.root_index && indegree[i] > 1U)
        {
            return MakeTopologyFailure(BtReturnCode::InvalidTopology,
                                       topology.nodes[i],
                                       ProductionBtTopologyReason::MultipleParents);
        }
    }

    bool reached[NodeCount] = {};
    queue_head = 0U;
    queue_tail = 0U;
    queue[queue_tail++] = topology.root_index;
    reached[topology.root_index] = true;
    while (queue_head < queue_tail)
    {
        const BtNodeIndex parent_index = queue[queue_head++];
        const ProductionBtNodeDescriptor& parent = topology.nodes[parent_index];
        for (std::uint32_t offset = 0U; offset < parent.child_count; ++offset)
        {
            const BtNodeIndex child_index =
                topology.child_indices[parent.first_child + offset];
            if (!reached[child_index])
            {
                reached[child_index] = true;
                queue[queue_tail++] = child_index;
            }
        }
    }
    for (std::size_t i = 0U; i < NodeCount; ++i)
    {
        if (!reached[i])
        {
            return MakeTopologyFailure(BtReturnCode::UnreachableNode,
                                       topology.nodes[i],
                                       ProductionBtTopologyReason::Unreachable);
        }
    }

    return ProductionBtTopologyValidation{
        true,
        MakeBtTickResult(BtReturnCode::Completed,
                         root.node_id,
                         root.stage_id,
                         0U,
                         BtReasonIdNone)};
}

template <std::size_t NodeCount, std::size_t ChildCount>
constexpr bool IsValidProductionBtTopology(
    const ProductionBtTopology<NodeCount, ChildCount>& topology) noexcept
{
    return ValidateProductionBtTopology(topology).valid;
}

static_assert(std::is_standard_layout<ProductionBtNodeDescriptor>::value,
              "Topology nodes must remain fixed-layout descriptors");
static_assert(std::is_trivially_copyable<ProductionBtNodeDescriptor>::value,
              "Topology nodes must remain trivially copyable");
static_assert(sizeof(ProductionBtNodeDescriptor) == 24U,
              "Topology node x64 ABI size changed");
static_assert(alignof(ProductionBtNodeDescriptor) == 4U,
              "Topology node x64 ABI alignment changed");
static_assert(offsetof(ProductionBtNodeDescriptor, node_id) == 0U,
              "Topology node_id offset changed");
static_assert(offsetof(ProductionBtNodeDescriptor, stage_id) == 4U,
              "Topology stage_id offset changed");
static_assert(offsetof(ProductionBtNodeDescriptor, kind) == 8U,
              "Topology kind offset changed");
static_assert(offsetof(ProductionBtNodeDescriptor, first_child) == 12U,
              "Topology first_child offset changed");
static_assert(offsetof(ProductionBtNodeDescriptor, child_count) == 16U,
              "Topology child_count offset changed");
static_assert(offsetof(ProductionBtNodeDescriptor, writer_id) == 20U,
              "Topology writer_id offset changed");
static_assert(std::is_standard_layout<ProductionBtTopologyValidation>::value,
              "Topology diagnostics must remain fixed-layout receipts");
static_assert(std::is_trivially_copyable<ProductionBtTopologyValidation>::value,
              "Topology diagnostics must remain trivially copyable");
static_assert(sizeof(ProductionBtTopologyValidation) == 40U,
              "Topology validation x64 ABI size changed");
static_assert(alignof(ProductionBtTopologyValidation) == 8U,
              "Topology validation x64 ABI alignment changed");
static_assert(offsetof(ProductionBtTopologyValidation, valid) == 0U,
              "Topology validation valid offset changed");
static_assert(offsetof(ProductionBtTopologyValidation, diagnostic) == 8U,
              "Topology validation diagnostic offset changed");

} // namespace static_bt
} // namespace behavior_tree
} // namespace LadyLuck
