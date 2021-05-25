#include "math_helpers.h"
#include <array>
#include <tuple>
#include <optional>
#include <glm/gtx/extended_min_max.hpp>

enum class BrickVoxelPosition {
    NodeCenter,
    NodeCorner
};

enum class BrickLayout {
    Linear,
    Morton
};

struct OctreeNode {
    static constexpr u32 DATA_CONSTANT_COLOR = 0;
    static constexpr u32 DATA_BRICK_ADDRESS = 1;

    u32 max_subdivision : 1; //
    u32 data_type_flag : 1; // node/constant
    u32 address : 30;

    union {
        struct {
            u32 r : 8;
            u32 g : 8;
            u32 b : 8;
            u32 a : 8;
        };
        struct {
            // u32 x : 10;
            // u32 y : 10;
            // u32 z : 10;
            // u32 _ : 2;
            u32 brick_address;
        };
    };
};

template<typename T>
constexpr T square(T x) {
    return x * x;
}

template<typename T>
constexpr T cube(T x) {
    return x * x * x;
}

template<i32 BRICK_SIZE, BrickLayout BRICK_LAYOUT = BrickLayout::Linear> 
struct BrickPayload {
    static constexpr i32 Size = BRICK_SIZE;
    std::array<Vec4, cube(BRICK_SIZE)> colors;

    void init() {
        colors.fill({});
    }

    i32 get_texel_index(Vec3i coord) const {
        if constexpr (BRICK_LAYOUT == BrickLayout::Linear)
        {
            return coord.z * square(BRICK_SIZE) +
                   coord.y * BRICK_SIZE +
                   coord.x;
        }
        else if(BRICK_LAYOUT == BrickLayout::Morton) {
            
        }
    }

    void set_voxel_color(Vec3i coord, Vec4 color) {
        colors.at(get_texel_index(coord)) = color;
    }

    Vec4 fetch(Vec3i coord) const {
        return colors.at(get_texel_index(coord));
    }

    // integers are texel centers
    Vec4 sample_trilinear(Vec3 location) const {
        // this is for sampling at the upper edge
        constexpr f32 EPS = 0.000001f;
        location = glm::min(location, Vec3{BRICK_SIZE - 1 - EPS});
        Vec3i ilocation = location;
        Vec4 p000 = fetch(ilocation);
        Vec4 p100 = fetch(ilocation + Vec3i{1, 0, 0});
        Vec4 p010 = fetch(ilocation + Vec3i{0, 1, 0});
        Vec4 p001 = fetch(ilocation + Vec3i{0, 0, 1});
        Vec4 p110 = fetch(ilocation + Vec3i{1, 1, 0});
        Vec4 p011 = fetch(ilocation + Vec3i{0, 1, 1});
        Vec4 p101 = fetch(ilocation + Vec3i{1, 0, 1});
        Vec4 p111 = fetch(ilocation + Vec3i{1, 1, 1});
        Vec4 c0 = p000;
        Vec4 c1 = p100 - p000;
        Vec4 c2 = p010 - p000;
        Vec4 c3 = p001 - p000;
        Vec4 c4 = p110 - p010 - p100 + p000;
        Vec4 c5 = p011 - p001 - p010 + p000;
        Vec4 c6 = p101 - p001 - p100 + p000;
        Vec4 c7 = p111 - p011 - p101 - p110 + p100 + p001 + p010 - p000;
        Vec3 delta = location - Vec3{ilocation};

        return c0 + c1 * delta.x + c2 * delta.y + c3 * delta.z + c4 * delta.x * delta.y
            + c5 * delta.y * delta.z + c6 * delta.z * delta.x + c7 * delta.x * delta.y * delta.z;
    }
};

constexpr f32 TRACE_EPS = 0.000001f;

namespace Brick {
    struct RayHit {
        Vec4 color;
        f32 t;
    };

    template<typename BrickT>
    // trace in normalised brick space [0..1]
    // each voxel is considered uniform in color (no interpolation)
    static std::optional<RayHit> trace_ray_start_in_brick(
        BrickT const & brick,
        Vec3 const & ray_origin,
        Vec3 const & inv_ray_dir
    ) 
    {
        static constexpr f32 inv_brick_size = 1.f / BrickT::Size;
        Vec3i voxel = glm::min(Vec3i{ray_origin * Vec3{BrickT::Size}}, Vec3i{BrickT::Size - 1});
        const Vec3i step = glm::sign(inv_ray_dir);
        f32 current_t = 0;

        while(true) {
            Vec4 sample = brick.fetch(voxel);
            bool collision = glm::any(glm::greaterThan(sample, Vec4{}));
            if(collision) {
                return RayHit{ .color = sample, .t = current_t };
            }

            Vec3 exit_planes = Vec3(voxel + Vec3i{glm::greaterThan(inv_ray_dir, Vec3{0})}) * Vec3{inv_brick_size};
            Vec3 t3 = ((exit_planes - ray_origin) * inv_ray_dir);
            // for rays parallel to an axis, we might have a negative inf
            t3 = glm::abs(t3);
            current_t = glm::fmin(t3.x, t3.y, t3.z);

            voxel += step * Vec3i{glm::equal(t3, Vec3{current_t})};

            if(voxel != glm::clamp(voxel, {}, {BrickT::Size - 1})) {
                break;
            }
        }

        return std::nullopt;
    }

    // trace in normalised brick space [0..1]
    template<typename BrickT>
    static std::optional<RayHit> trace_ray(
        BrickT const & brick,
        Vec3 ray_origin,
        Vec3 const & ray_dir,
        Vec3 const & inv_ray_dir
    )
    {
        f32 t = 0.f;
        if(glm::any(glm::lessThan(ray_origin, Vec3{})) || glm::any(glm::greaterThan(ray_origin, Vec3{1}))) {
            // find t till hit the front plane
            Vec3 enter_planes = glm::lessThan(inv_ray_dir, Vec3{0});
            Vec3 t3 = ((enter_planes - ray_origin) * inv_ray_dir);
            f32 max_t = glm::fmax(t3.x, t3.y, t3.z);
            t = glm::max(0.f, max_t + TRACE_EPS);
            if(t > 0) {
                // move rayo to t, start trace in the brick
                ray_origin += t * ray_dir;
            }

            // if not in the brick - return immediatelly 
            if(glm::any(glm::lessThan(ray_origin, Vec3{})) || glm::any(glm::greaterThan(ray_origin, Vec3{1}))) {
                return std::nullopt;
            }   
        }

        auto hit = trace_ray_start_in_brick(brick, ray_origin, inv_ray_dir);
        if(hit) {
            hit->t += t;
            return hit;
        }
        return std::nullopt;
    }
}

template<i32 BRICK_SIZE, BrickVoxelPosition BRICK_VOXEL_POS, BrickLayout BRICK_LAYOUT = BrickLayout::Linear>
struct SvoPool {
    using BrickType = BrickPayload<BRICK_SIZE, BRICK_LAYOUT>;
    static constexpr BrickVoxelPosition BRICK_VOXEL_POS = BRICK_VOXEL_POS;
    static constexpr i32 BRICK_SIZE = BRICK_SIZE;

    std::vector<OctreeNode> nodes_pool_;
    std::vector<BrickType> brick_pool_;
    i32 next_node_ = 0;
    i32 next_brick_ = 0;

    SvoPool() {
        if constexpr(BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            static_assert(BRICK_SIZE >= 3);
        }
        else if(BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            static_assert(BRICK_SIZE >= 2);
        }
    }

    void reset(i32 reserve_nodes, i32 reserve_bricks) {
        next_node_ = 0;
        nodes_pool_.resize(reserve_nodes);
        brick_pool_.resize(reserve_bricks);
    }

    i32 alloc_nodes(i32 num) {
        i32 offset = next_node_;
        next_node_ += num;
        return offset;
    }

    i32 alloc_brick() {
        return next_brick_++;
    }

    OctreeNode& get_node(i32 index) {
        return nodes_pool_.at(index);
    }

    const OctreeNode& get_node(i32 index) const {
        return nodes_pool_.at(index);
    }

    BrickType& get_brick(i32 index) {
        return brick_pool_.at(index);
    }

    const BrickType& get_brick(i32 index) const {
        return brick_pool_.at(index);
    }
};

template<typename TPool>
struct Svo {
    TPool & pool_;
    const Obb obb_;
    const i32 root_node_;
    const i32 max_depth_;

    static constexpr i32 MAX_DEPTH = 10;

    std::unordered_map<Vec4i, i32> brick_to_address_;

    struct BrickInfo {
        Vec3i id;
        i32 address;
    };
    std::array<std::vector<BrickInfo>, MAX_DEPTH> bricks_at_depth_; 

    Svo(TPool & pool, Obb const& obb, i32 max_depth) : 
        pool_{pool}, 
        obb_{obb}, 
        max_depth_{max_depth},
        root_node_{pool.alloc_nodes(1)}
    {
        reset_node(root_node_);
    }

    i32 get_bricks_num_per_side(const i32 depth) const {
        return 1 << depth;
    }

    i32 get_voxel_res(const i32 depth) const {
        const i32 brick_res = get_bricks_num_per_side(depth);
        // voxels on the "border" are not included
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            return brick_res * (TPool::BRICK_SIZE - 2);
        }
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            return brick_res * (TPool::BRICK_SIZE - 1) + 1;
        }
    }

    f32 get_voxel_world_size() const {
        f32 span = obb_.get_size().x;
        i32 res = get_voxel_res(max_depth_);
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            return span / res;
        }
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            return span / (res - 1);
        }
    }

    f32 get_voxel_normalised_size(const i32 depth) const {
        i32 res = get_voxel_res(depth);
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            return 1.f / res;
        }
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            return 1.f / (res - 1);
        }
    }

    // [0, 1] cube
    Vec3 get_local_normalised_position(Vec3 world_position) const {
        return obb_.to_local(world_position) / obb_.half_extent * 0.5f + 0.5f;
    }

    std::tuple<Vec3i, Vec3i> get_brick_id_and_brick_coord(Vec3i voxel_coord, const i32 depth) const {
        assert(glm::all(glm::greaterThanEqual(voxel_coord, {})));
        assert(glm::all(glm::lessThanEqual(voxel_coord, Vec3i{get_voxel_res(depth)})));

        // *-|-*--*-|-*
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            Vec3i brick_id = voxel_coord / (TPool::BRICK_SIZE - 2);
            brick_id = glm::min(brick_id, get_bricks_num_per_side(depth) - 1);
            return std::make_tuple(brick_id, voxel_coord - brick_id * (TPool::BRICK_SIZE - 2) + 1);
        }
        // |*--*--*|
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            Vec3i brick_id = voxel_coord / (TPool::BRICK_SIZE - 1);
            // this is to address the last voxel within the last brick
            brick_id = glm::min(brick_id, get_bricks_num_per_side(depth) - 1);
            return std::make_tuple(brick_id, voxel_coord - brick_id * (TPool::BRICK_SIZE - 1));
        }
    }

    Vec3 get_first_voxel_world_offset() const {
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            return get_voxel_world_size() * Vec3{0.5f};
        }
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            return {};
        }
    }

    static constexpr Vec3 get_normalised_position_to_sample_offset() {
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            return Vec3{-0.5f};
        }
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            return {};
        }
    }

    void set_color_at_location(Vec3 location, Vec4 color) {
        Vec3 local_normalised_position = get_local_normalised_position(location);

        assert(glm::all(glm::greaterThanEqual(local_normalised_position, Vec3{})));
        assert(glm::all(glm::lessThanEqual(local_normalised_position, Vec3{1.f})));

        // calculate voxel coord
        const f32 sample_distance = get_voxel_normalised_size(max_depth_);
        const Vec3 voxel_coord = local_normalised_position / sample_distance + get_normalised_position_to_sample_offset();
        Vec3i voxel_icoord = voxel_coord + 0.5f; // rounding
        // if we hit voxel at 1., we need to clamp so it's stored within non-border voxel
        voxel_icoord = glm::min(voxel_icoord, get_voxel_res(max_depth_) - 1);

        // we need to round the proximity of the sample to that sample index
        // it's not necessary when sampling later (as we have the border anyway)
        store_voxel_level(voxel_icoord, max_depth_, color);
    }

    Vec4 sample_color_at_location_level(Vec3 location, i32 depth) {
        Vec3 local_normalised_position = get_local_normalised_position(location);

        assert(glm::all(glm::greaterThanEqual(local_normalised_position, Vec3{})));
        assert(glm::all(glm::lessThanEqual(local_normalised_position, Vec3{1.f})));

        // calculate voxel coord
        const f32 sample_distance = get_voxel_normalised_size(depth);
        const Vec3 voxel_coord = local_normalised_position / sample_distance + get_normalised_position_to_sample_offset();
        const Vec3i voxel_icoord = voxel_coord;

        auto [brick_id, _] = get_brick_id_and_brick_coord(voxel_icoord, depth);
        i32 brick_address = request_committed_brick_mem(brick_id, depth);

        // TODO: depends on brick mode, we need the coord of the leftermost voxel sample?
        Vec3 brick_texcoord;
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            brick_texcoord = voxel_coord - Vec3{brick_id * (TPool::BRICK_SIZE - 2)} + 1.f;
        }
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            brick_texcoord = voxel_coord - Vec3{brick_id * (TPool::BRICK_SIZE - 1)};
        }

        return pool_.get_brick(brick_address).sample_trilinear(brick_texcoord);
    }

    Vec4 sample_color_at_location(Vec3 location) {
        return sample_color_at_location_level(location, max_depth_);
    }

    void reset_node(i32 address) {
        OctreeNode &node = pool_.get_node(address);
        node.max_subdivision = 0;
        //node.data_type_flag = OctreeNode::DATA_CONSTANT_COLOR;
        //node.data_type_flag = OctreeNode::DATA_BRICK_ADDRESS;
        node.address = 0;
        // node.r = 0;
        // node.g = 0;
        // node.b = 0;
        // node.a = 0;
        node.brick_address = pool_.alloc_brick();
    }

    struct BorderCopySpan
    {
        Vec3i voxels_begin;
        Vec3i voxels_end;
        Vec3i neighbour_offset;
    };

    void copy_border(Vec3i brick_id, i32 depth, BorderCopySpan const & span) 
    {
        auto const & src_brick = pool_.get_brick(request_committed_brick_mem(brick_id, depth));
        bool needs_copy = false;
        for(i32 z=span.voxels_begin.z; z<span.voxels_end.z; z++) {
            for(i32 y=span.voxels_begin.y; y<span.voxels_end.y; y++) {
                for(i32 x=span.voxels_begin.x; x<span.voxels_end.x; x++) {
                    if(src_brick.fetch({x, y, z}) != Vec4{}) {
                        needs_copy = true;
                        break;
                    }
                }
            }
        }

        if (!needs_copy)
        {
            return;
        }

        Vec3i neighbour_id = brick_id + span.neighbour_offset;

        const bool dst_brick_outside_octree = glm::any(glm::lessThan(neighbour_id, Vec3i{})) || glm::any(glm::greaterThanEqual(neighbour_id, Vec3i{get_bricks_num_per_side(depth)}));

        if (dst_brick_outside_octree)
        {
            if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter)
            {
                // copy to the local border
                auto & dst_brick = pool_.get_brick(request_committed_brick_mem(brick_id, depth));
                for(i32 z=span.voxels_begin.z; z<span.voxels_end.z; z++) {
                    for(i32 y=span.voxels_begin.y; y<span.voxels_end.y; y++) {
                        for(i32 x=span.voxels_begin.x; x<span.voxels_end.x; x++) {
                            Vec3i offset = span.neighbour_offset;
                            Vec3i dst_texel = Vec3i{x, y, z} + offset;
                            dst_brick.set_voxel_color(dst_texel, src_brick.fetch({x, y, z}));
                        }
                    }
                }
            }

            return;
        }

        auto & dst_brick = pool_.get_brick(request_committed_brick_mem(neighbour_id, depth));
        for(i32 z=span.voxels_begin.z; z<span.voxels_end.z; z++) {
            for(i32 y=span.voxels_begin.y; y<span.voxels_end.y; y++) {
                for(i32 x=span.voxels_begin.x; x<span.voxels_end.x; x++) {
                    Vec3i offset = span.neighbour_offset;
                    if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
                        offset *= 2; // the border is 2 voxels wide
                    }
                    Vec3i dst_texel = (Vec3i{x, y, z} + offset + TPool::BRICK_SIZE) % TPool::BRICK_SIZE;
                    dst_brick.set_voxel_color(dst_texel, src_brick.fetch({x, y, z}));
                }
            }
        }
    }

    static Vec3i child_index_to_spatial_offset(const i32 index) {
        return { index % 2, (index / 2) % 2, index / 4 };
    }

    i32 get_child_index_and_refine_range(const Vec3i brick_id, Vec3i & begin, Vec3i & end) {
        assert(all(glm::greaterThanEqual(brick_id, begin)));
        assert(all(glm::lessThan(brick_id, end)));

        Vec3i span = (end - begin) / 2;
        Vec3i mid = span + begin;

        auto v_greater_eq = glm::greaterThanEqual(brick_id, mid);
        begin += Vec3i{v_greater_eq} * span;
        end -= Vec3i{glm::not_(v_greater_eq)} * span;
        
        return v_greater_eq.x * 1 + v_greater_eq.y * 2 + v_greater_eq.z * 4;
    }

    i32 request_committed_brick_mem(const Vec3i brick_id, const i32 depth) {
        assert(glm::all(glm::greaterThanEqual(brick_id, {})));
        assert(glm::all(glm::lessThan(brick_id, Vec3i{get_voxel_res(depth)})));

        // walk down the tree
        i32 current_depth = 0;
        OctreeNode * current_node = &pool_.get_node(root_node_);
        //
        Vec3i node_bricks_begin{};
        Vec3i node_bricks_end{get_bricks_num_per_side(depth)};

        while(current_depth != depth) {
            i32 address = current_node->address;
            if(address == 0) {
                current_node->address = address = pool_.alloc_nodes(8);
                for(i32 i=0; i<8; i++) {
                    reset_node(address + i);
                }
            }

            i32 child_index = get_child_index_and_refine_range(brick_id, node_bricks_begin, node_bricks_end);
            current_node = &pool_.get_node(address + child_index);
            current_depth++;
        }

        return current_node->brick_address;
    }

    // skips border voxels
    std::optional<Vec4> try_load_voxel_level(const Vec3i voxel, const i32 depth) {
        const bool valid_sample = glm::all(glm::greaterThanEqual(voxel, {})) && glm::all(glm::lessThan(voxel, Vec3i{get_voxel_res(depth)}));        
        if(!valid_sample) {
            return std::nullopt;
        }
        auto [brick_id, brick_coord] = get_brick_id_and_brick_coord(voxel, depth);
        i32 brick_address = request_committed_brick_mem(brick_id, depth);
        return pool_.get_brick(brick_address).fetch(brick_coord);
    }

    Vec4 load_voxel_level(const Vec3i voxel, const i32 depth) {
        auto [brick_id, brick_coord] = get_brick_id_and_brick_coord(voxel, depth);
        i32 brick_address = request_committed_brick_mem(brick_id, depth);
        return pool_.get_brick(brick_address).fetch(brick_coord);
    }

    // skips border voxels
    void store_voxel_level(const Vec3i voxel, const i32 depth, const Vec4 color) {
        auto [brick_id, brick_coord] = get_brick_id_and_brick_coord(voxel, depth);
        i32 brick_address = request_committed_brick_mem(brick_id, depth);
        pool_.get_brick(brick_address).set_voxel_color(brick_coord, color);
    }

    void gather_bricks_at_level(const OctreeNode * traverse_node, const Vec3i traverse_brick_id, const i32 traverse_depth, const i32 requested_depth, std::vector<Vec3i> & acc) const
    {
        if(traverse_depth == requested_depth)
        {
            acc.push_back(traverse_brick_id);
            return;
        }

        if(traverse_node->address == 0)
        {
            return;
        }

        for(i32 c=0; c<8; c++) {
            gather_bricks_at_level(
                &pool_.get_node(traverse_node->address + c),
                traverse_brick_id * 2 + child_index_to_spatial_offset(c),
                traverse_depth + 1,
                requested_depth,
                acc
            );
        }
    }

    void build_tree() {
        for (i32 depth = max_depth_; depth >= 0; depth--)
        {
            std::vector<Vec3i> bricks;
            gather_bricks_at_level(&pool_.get_node(root_node_), {}, 0, depth, bricks);

            if(depth != max_depth_)
            {
                // downsample
                for (Vec3i brick_id : bricks)
                {
                    const Vec3i neighbour_id = brick_id + Vec3i{1, 1, 1};
                    const auto neighbour_outside = glm::greaterThanEqual(neighbour_id, Vec3i{get_bricks_num_per_side(depth)});

                    // the border voxels will be calculated in the neighbour and copies (except if there's no neighbour)

                    if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
                        for (i32 z = 0; z < TPool::BRICK_SIZE - 2; z++)
                        {
                            for (i32 y = 0; y < TPool::BRICK_SIZE - 2; y++)
                            {
                                for (i32 x = 0; x < TPool::BRICK_SIZE - 2; x++)
                                {
                                    Vec3i dst_voxel = brick_id * (TPool::BRICK_SIZE - 2) + Vec3i{x, y, z};
                                    Vec3i src_voxel = dst_voxel * 2;

                                    Vec4 downsampled{0};
                                    f32 w{0};

                                    for (i32 i = 0; i < 2; i++)
                                    {
                                        for (i32 j = 0; j < 2; j++)
                                        {
                                            for (i32 k = 0; k < 2; k++)
                                            {
                                                auto maybe_sample = try_load_voxel_level(src_voxel + Vec3i{i, j, k}, depth + 1);
                                                if(maybe_sample)
                                                {
                                                    downsampled += (*maybe_sample);
                                                    w += 1.f;
                                                }
                                            }
                                        }
                                    }

                                    // to avoid NaN
                                    if(w == 0) {
                                        w = 1.f;
                                    }
                                    store_voxel_level(dst_voxel, depth, downsampled / w);
                                }
                            }
                        }
                    }
                    else if constexpr (TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner)
                    {
                        for (i32 z = 0; z < TPool::BRICK_SIZE - 1 + neighbour_outside.x; z++)
                        {
                            for (i32 y = 0; y < TPool::BRICK_SIZE - 1 + neighbour_outside.y; y++)
                            {
                                for (i32 x = 0; x < TPool::BRICK_SIZE - 1 + neighbour_outside.z; x++)
                                {
                                    Vec3i dst_voxel = brick_id * (TPool::BRICK_SIZE - 1) + Vec3i{x, y, z};
                                    Vec3i src_voxel = dst_voxel * 2;
                                    
                                    Vec4 downsampled{0};
                                    
                                    std::array<f32, 3 * 3> kernel_02 {
                                        1.f, 2.f, 1.f,
                                        2.f, 4.f, 2.f,
                                        1.f, 2.f, 1.f
                                    };
                                    std::array<f32, 3 * 3> kernel_1 {
                                        2.f, 4.f, 2.f,
                                        4.f, 8.f, 4.f,
                                        2.f, 4.f, 2.f,
                                    };
                                    f32 w = 0.f;
                                    for(i32 i=0; i<3; i++) {
                                        for (i32 j= 0; j<3; j++)
                                        {
                                            auto maybe_sample = try_load_voxel_level(src_voxel + Vec3i{i - 1, j - 1, -1}, depth + 1);
                                            if(maybe_sample)
                                            {
                                                downsampled += (*maybe_sample) * Vec4{kernel_02.at(i * 3 + j)};
                                                w += kernel_02.at(i * 3 + j);
                                            }
                                        }
                                    }
                                    for(i32 i=0; i<3; i++) {
                                        for (i32 j= 0; j<3; j++)
                                        {
                                            auto maybe_sample = try_load_voxel_level(src_voxel + Vec3i{i - 1, j - 1, 0}, depth + 1);
                                            if(maybe_sample)
                                            {
                                                downsampled += (*maybe_sample) * Vec4{kernel_1.at(i * 3 + j)};
                                                w += kernel_1.at(i * 3 + j);
                                            }
                                        }
                                    }
                                    for(i32 i=0; i<3; i++) {
                                        for (i32 j= 0; j<3; j++)
                                        {
                                            auto maybe_sample = try_load_voxel_level(src_voxel + Vec3i{i - 1, j - 1, 1}, depth + 1);
                                            if(maybe_sample)
                                            {
                                                downsampled += (*maybe_sample) * Vec4{kernel_02.at(i * 3 + j)};
                                                w += kernel_02.at(i * 3 + j);
                                            }
                                        }
                                    }

                                    // to avoid NaN
                                    if(w == 0) {
                                        w = 1.f;
                                    }
                                    store_voxel_level(dst_voxel, depth, downsampled / w);
                                }
                            }
                        }
                    }
                }
            }

            for (Vec3i brick_id : bricks)
            {
                if constexpr (TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter)
                {
                    // 6 sides
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, 1}, .voxels_end = {2, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, 0, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, 2, TPool::BRICK_SIZE - 1}, .neighbour_offset = {0, -1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, 2}, .neighbour_offset = {0, 0, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, 1, 1}, .voxels_end = {TPool::BRICK_SIZE -1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {1, 0, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, TPool::BRICK_SIZE - 2, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE -1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {0, 1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, TPool::BRICK_SIZE - 2}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {0, 0, 1}});
                    // 12 edges
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, 1}, .voxels_end = {2, 2, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, -1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, 1}, .voxels_end = {2, TPool::BRICK_SIZE - 1, 2}, .neighbour_offset = {-1, 0, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, 2, 2}, .neighbour_offset = {0, -1, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, 1, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, 2, TPool::BRICK_SIZE - 1}, .neighbour_offset = {1, -1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, 1, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, 2}, .neighbour_offset = {1, 0, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, TPool::BRICK_SIZE - 2, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, 2}, .neighbour_offset = {0, 1, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, TPool::BRICK_SIZE - 2, 1}, .voxels_end = {2, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, 1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, TPool::BRICK_SIZE - 2}, .voxels_end = {2, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, 0, 1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, TPool::BRICK_SIZE - 2}, .voxels_end = {TPool::BRICK_SIZE - 1, 2, TPool::BRICK_SIZE - 1}, .neighbour_offset = {0, -1, 1}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, TPool::BRICK_SIZE - 2, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {1, 1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, 1, TPool::BRICK_SIZE - 2}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {1, 0, 1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, TPool::BRICK_SIZE - 2, TPool::BRICK_SIZE - 2}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {0, 1, 1}});
                    // 8 corners
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, 1}, .voxels_end = {2, 2, 2}, .neighbour_offset = {-1, -1, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, 1, TPool::BRICK_SIZE - 2}, .voxels_end = {2, 2, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, -1, 1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, TPool::BRICK_SIZE - 2, 1}, .voxels_end = {2, TPool::BRICK_SIZE - 1, 2}, .neighbour_offset = {-1, 1, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {1, TPool::BRICK_SIZE - 2, TPool::BRICK_SIZE - 2}, .voxels_end = {2, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, 1, 1}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, 1, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, 2, 2}, .neighbour_offset = {1, -1, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, 1, TPool::BRICK_SIZE - 2}, .voxels_end = {TPool::BRICK_SIZE - 1, 2, TPool::BRICK_SIZE - 1}, .neighbour_offset = {1, -1, 1}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, TPool::BRICK_SIZE - 2, 1}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, 2}, .neighbour_offset = {1, 1, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {TPool::BRICK_SIZE - 2, TPool::BRICK_SIZE - 2, TPool::BRICK_SIZE - 2}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {1, 1, 1}});
                }
                else if constexpr (TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner)
                {
                    // 3 sides
                    copy_border(brick_id, depth, {.voxels_begin = {}, .voxels_end = {1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, 0, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {}, .voxels_end = {TPool::BRICK_SIZE - 1, 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {0, -1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {}, .voxels_end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, 1}, .neighbour_offset = {0, 0, -1}});
                    // 3 edges
                    copy_border(brick_id, depth, {.voxels_begin = {}, .voxels_end = {1, 1, TPool::BRICK_SIZE - 1}, .neighbour_offset = {-1, -1, 0}});
                    copy_border(brick_id, depth, {.voxels_begin = {}, .voxels_end = {1, TPool::BRICK_SIZE - 1, 1}, .neighbour_offset = {-1, 0, -1}});
                    copy_border(brick_id, depth, {.voxels_begin = {}, .voxels_end = {TPool::BRICK_SIZE - 1, 1, 1}, .neighbour_offset = {0, -1, -1}});
                    // 1 corner
                    copy_border(brick_id, depth, {.voxels_begin = {}, .voxels_end = {1, 1, 1}, .neighbour_offset = {-1, -1, -1}});
                }
            }
        }
    }
};

struct TraceResult
{
    Vec4 color;
    f32 t;
};

/*
test scenarios:
*) non-aabb obb
*) 
*/

// constexpr f32 k_traverse_eps = 0.00000001f;

// template<typename TSvo>
// std::optional<TraceResult> trace_ray(TSvo const & svo, Ray const &ray) 
// {
//     // convert ray to local position
//     Ray local_ray = svo.obb_.to_local(ray);
//     Vec3 inv_local_ray_dir = Vec3{1.f} / local_ray.dir;
//     Vec3 half_extent = svo.obb_.half_extent;

//     Vec3 enter_local_vt = (local_ray.origin - Vec3{-half_extent}) * inv_local_ray_dir;
//     f32 enter_local_t = glm::max(enter_local_vt.x, enter_local_vt.y, enter_local_vt.z);
//     Vec3 exit_local_vt = (local_ray.origin - Vec3{half_extent}) * inv_local_ray_dir;
//     f32 exit_local_t = glm::min(exit_local_vt.x, exit_local_vt.y, exit_local_vt.z);

//     if(exit_local_t < 0.f) {
//         return std::nullopt;
//     }

//     const f32 voxel_wsize = get_voxel_world_size();

//     local_ray.origin += glm::min(enter_local_t + k_traverse_eps, 0.f) * local_ray.direction;
//     // find the target node of the current position
//     Vec3i voxel_coord = (local_ray.origin - half_extent) / get_voxel_world_size();

//     const i32 max_depth = svo.max_depth_;
//     f32 current_t = enter_local_t > 0 ? 0 : enter_local_t;
//     Vec3 current_origin = local_ray.origin + current_t * local_ray.direction;
//     int current_depth = 0;
//     const OctreeNode * current_node = &svo.pool_.get_node(svo.root_node_);
//     Vec3i current_node_coord{0};
//     Vec3i step = glm::sign(local_ray.direction);
    
//     while(true) {
//         //not supported yet
//         assert(current_node.data_type_flag != OctreeNode::DATA_CONSTANT_COLOR);
//         i32 children_address = current_node->address;
//         const i32 width_in_max_depth_nodes = 1 << (max_depth - current_depth);

//         // subdivide if possible
//         if(current_depth < max_depth && children_address) {
//             const Vec3i current_node_mid = current_node_coord * width_in_max_depth_nodes + width_in_max_depth_nodes / 2;
//             auto vec_ge_mid = glm::greaterThanEqual(voxel_coord, current_node_mid);
            
//             i32 child_index = vec_ge_mid.x + vec_ge_mid.y * 2 + vec_ge_mid.z * 4;

//             current_node = &svo.pool_.get_node(children_address + child_index);
//             current_node_coord = current_node_coord * 2 + Vec3i{vec_ge_mid};
//             current_depth++;
//             continue;
//         }

//         // couldn't subdivide, do I have a brick
//         if(current_node->brick_address) {
//             // raymarch the brick, return on hit
//         }

//         // time to move to the end of the cell
//         // find the id of the neighbour on the same level

//         // TODO: domain
//         Vec3 exit_planes = (current_node_coord + step) * width_in_max_depth_nodes;
//         exit_planes *= voxel_wsize;

//         Vec3 t_3 = (local_ray.origin - exit_planes) * inv_local_ray_dir;
//         current_t = std::min(t_3.x, t_3.y, t_3.z);
//         current_position = local_ray.origin + current_t * local_ray.direction;

//         Vec3i neighbour_node_id = current_node_coord + glm::equals(t_3, current_t) * step;
//         // find common ancestor, update current_node to the common ancestor
//         // Vec3i common_ancestor = glm::highestBitValue(neighbour_node_id & current_node_id);
//         i32 how_many_steps_to_common_ancestor;
//         current_node_coord << how_many_steps_to_common_ancestor; // 
//         // update current node from stack

//         // do stuff to

//     }

//     return std::nullopt;
// }