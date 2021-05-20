#include "math_helpers.h"
#include <array>
#include <tuple>

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
            return brick_res * (TPool::BRICK_SIZE - 1);
        }
    }

    f32 get_voxel_world_size() const {
        f32 span = obb_.get_size().x;
        i32 res = get_voxel_res(max_depth_);
        // border voxels are outside of the represented volume
        // obb space spans from the first non-border voxel center (0) to the last non-border voxel center (1)
        return span / (res - 1);
    }

    f32 get_voxel_normalised_size(const i32 depth) const {
        i32 res = get_voxel_res(depth);
        return 1.f / (res - 1);
    }

    // [0, 1] cube
    Vec3 get_local_normalised_position(Vec3 world_position) const {
        return obb_.to_local(world_position) / obb_.half_extent * 0.5f + 0.5f;
    }

    std::tuple<Vec3i, Vec3i> get_brick_id_and_brick_coord(Vec3i voxel_coord, const i32 depth) const {
        voxel_coord = glm::min(voxel_coord, get_voxel_res(depth) - 1);
        // *-|-*--*-|-*
        if constexpr(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter) {
            Vec3i brick_id = voxel_coord / (TPool::BRICK_SIZE - 2);
            return std::make_tuple(brick_id, voxel_coord - brick_id * (TPool::BRICK_SIZE - 2) + 1);
        }
        // |*--*--*|
        else if(TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner) {
            Vec3i brick_id = voxel_coord / (TPool::BRICK_SIZE - 1);
            return std::make_tuple(brick_id, voxel_coord - brick_id * (TPool::BRICK_SIZE - 1));
        }
    }

    void set_color_at_location(Vec3 location, Vec4 color) {
        Vec3 local_voxel_position = get_local_normalised_position(location);

        assert(glm::all(glm::greaterThanEqual(local_voxel_position, Vec3{})));
        assert(glm::all(glm::lessThanEqual(local_voxel_position, Vec3{1.f})));

        // calculate voxel coord
        const f32 sample_distance = get_voxel_normalised_size(max_depth_);
        const Vec3 voxel_coord = local_voxel_position / sample_distance;
        const Vec3i voxel_icoord = voxel_coord + 0.5f;

        // we need to round the proximity of the sample to that sample index
        // it's not necessary when sampling later (as we have the border anyway)
        store_voxel_level(voxel_icoord, max_depth_, color);
    }

    Vec4 sample_color_at_location(Vec3 location) {
        Vec3 local_voxel_position = get_local_normalised_position(location);

        assert(glm::all(glm::greaterThanEqual(local_voxel_position, Vec3{})));
        assert(glm::all(glm::lessThanEqual(local_voxel_position, Vec3{1.f})));

        // calculate voxel coord
        const f32 sample_distance = get_voxel_normalised_size(max_depth_);
        const Vec3 voxel_coord = local_voxel_position / sample_distance;

        auto [brick_id, _] = get_brick_id_and_brick_coord(voxel_coord, max_depth_);
        // TODO: depends on brick mode, we need the coord of the leftermost voxel sample?
        const Vec3 brick_texcoord = voxel_coord - Vec3{brick_id * (TPool::BRICK_SIZE - 1)};

        // walk down the tree
        i32 current_depth = 0;
        OctreeNode * current_node = &pool_.get_node(root_node_);
        //
        Vec3i node_bricks_begin{};
        Vec3i node_bricks_end{get_bricks_num_per_side(max_depth_)};

        while(current_depth != max_depth_) {
            i32 address = current_node->address;
            assert(address);

            i32 child_index = get_child_index_and_refine_range(brick_id, node_bricks_begin, node_bricks_end);
            current_node = &pool_.get_node(address + child_index);
            current_depth++;
        }

        return pool_.get_brick(current_node->brick_address).sample_trilinear(brick_texcoord); 
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

    // TPool::BrickType & get_or_insert_brick(Vec4i id_depth) {
    //     // todo: don't create bricks outside of borders
    //     auto iter = brick_to_address_.find(id_depth);
    //     if(iter == brick_to_address_.end()) {
    //         iter = brick_to_address_.insert(std::make_pair(id_depth, pool_.alloc_brick())).first;
    //         bricks_at_depth_[id_depth.w].push_back({Vec3i{id_depth}, iter->second});
    //         pool_.get_brick(iter->second).init();
    //     }
    //     return pool_.get_brick(iter->second);
    // }

    // struct CopySpan
    // {
    //     Vec3i begin;
    //     Vec3i end;
    //     Vec3i brick_neighbour_offset;
    // };

    // void copy_border(Vec4i current_brick, i32 current_address, CopySpan const & span) 
    // {
    //     auto const & src_brick = pool_.get_brick(current_address);
    //     bool needs_copy = false;
    //     for(i32 z=span.begin.z; z<span.end.z; z++) {
    //         for(i32 y=span.begin.y; y<span.end.y; y++) {
    //             for(i32 x=span.begin.x; x<span.end.x; x++) {
    //                 if(src_brick.fetch({x, y, z}) != Vec4{}) {
    //                     needs_copy = true;
    //                     break;
    //                 }
    //             }
    //         }
    //     }

    //     if(!needs_copy) {
    //         return;
    //     }

    //     Vec3i neighbour = Vec3i{current_brick} + span.brick_neighbour_offset;

    //     // todo: this can fail if outside borders!
    //     // also if we created: update parent!
    //     auto & dst_brick = get_or_insert_brick({neighbour, current_brick.w});
    //     for(i32 z=span.begin.z; z<span.end.z; z++) {
    //         for(i32 y=span.begin.y; y<span.end.y; y++) {
    //             for(i32 x=span.begin.x; x<span.end.x; x++) {
    //                 Vec3i dst_texel = (Vec3i{x, y, z} + span.brick_neighbour_offset) % TPool::BRICK_SIZE;
    //                 dst_brick.set_voxel_color(dst_texel, src_brick.fetch({x, y, z}));
    //             }
    //         }
    //     }
    // }

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
        // walk down the tree
        i32 current_depth = 0;
        OctreeNode * current_node = &pool_.get_node(root_node_);
        //
        Vec3i node_bricks_begin{};
        Vec3i node_bricks_end{get_bricks_num_per_side(depth)};

        while(current_depth != depth) {
            i32 address = 0;
            if(current_node->address == 0) {
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
    Vec4 load_voxel_level(const Vec3i voxel, const i32 depth) {
        auto [brick_id, brick_coord] = get_brick_id_and_brick_coord(voxel, depth);
        i32 brick_address = request_committed_brick_mem(brick_id, depth);
        pool_.get_brick(brick_address).fetch(brick_coord);
    }

    // skips border voxels
    void store_voxel_level(const Vec3i voxel, const i32 depth, const Vec4 color) {
        auto [brick_id, brick_coord] = get_brick_id_and_brick_coord(voxel, depth);
        i32 brick_address = request_committed_brick_mem(brick_id, depth);
        pool_.get_brick(brick_address).set_voxel_color(brick_coord, color);
    }

    void gather_bricks_at_level(const OctreeNode * traverse_node, const Vec3i traverse_brick_id, const i32 traverse_depth, const i32 requested_depth, std::vector<Vec3i> & acc)
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

    // todo: this doesn't build the tree any more
    // transfer_border_voxels
    void build_tree() {
        for (i32 depth = max_depth_; depth >= 0; depth--)
        {
            std::vector<Vec3i> bricks;
            gather_bricks_at_level(&pool_.get_node(root_node_), {}, 0, depth, bricks);


            // walk the tree to build the vector of bricks at max_depth
            // foreach brick
            {
                // iterate edge voxels
                // find neighbour brick address with request_committed_brick_mem
            }
        }
    }

    // void build_tree() {
    //     // create tree structure for the leaf nodes
    //     std::array<Vec3i, MAX_DEPTH> stack;
        
    //     for(BrickInfo brick_info : bricks_at_depth_[max_depth_]) {
    //         i32 depth = max_depth_;
    //         Vec3i brick_id = brick_info.id;
    //         i32 address = brick_info.address;
    //         while(depth >= 0) {
    //             stack.at(depth) = brick_id;
    //             brick_id /= 2;
    //             depth--;
    //         }

    //         if(root_ == -1) {
    //             root_ = pool_.alloc_nodes(1);
    //             reset_node(root_);
    //         }

    //         OctreeNode & node = pool_.get_node(root_);
    //         for (depth = 1; depth <= max_depth_; depth++)
    //         {
    //             if(node.max_subdivision) {
    //                 node.max_subdivision = 0;
    //                 node.address = pool_.alloc_nodes(8);

    //                 for(i32 i=0; i<8; i++) {
    //                     reset_node(node.address + i);
    //                 }
    //             }

    //             i32 child_offset = (stack[depth].x % 2) + (stack[depth].y % 2) * 2 + (stack[depth].z % 2) * 4;
    //             node = pool_.get_node(node.address + child_offset);
    //         }
    //     }

    //     // iterate leaf bricks
    //     for (i32 depth = max_depth_; depth >= 0; depth--)
    //     {
    //         if(depth < max_depth_) 
    //         {
    //             // todo: upsample from children
    //             for (BrickInfo brick_info : bricks_at_depth_[depth])
    //             {
    //                 Vec3i brick_id = brick_info.id;
    //                 i32 address = brick_info.address;

    //                 if constexpr (TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter)
    //                 {
    //                     // todo
    //                 }
    //                 else if (TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner)
    //                 {
    //                     for (i32 z = 0; z < TPool::BRICK_SIZE - 1; z++)
    //                     {
    //                         for (i32 y = 0; y < TPool::BRICK_SIZE - 1; y++)
    //                         {
    //                             for (i32 x = 0; x < TPool::BRICK_SIZE - 1; x++)
    //                             {
    //                                 Vec3i dst_voxel = brick_id * (TPool::BRICK_SIZE - 1) + Vec3i{x, y, z};
    //                                 Vec3i src_voxel = dst_voxel * 2;
                                    
    //                                 Vec4 downsampled{0};
                                    
    //                                 std::array<f32, 3 * 3> kernel_02 {
    //                                     1.f / 64.f, 2.f / 64.f, 1.f/ 64.f,
    //                                     2.f / 64.f, 4.f / 64.f, 2.f/ 64.f,
    //                                     1.f / 64.f, 2.f / 64.f, 1.f/ 64.f
    //                                 };
    //                                 std::array<f32, 3 * 3> kernel_1 {
    //                                     2.f / 64.f, 4.f / 64.f, 2.f/ 64.f,
    //                                     4.f / 64.f, 8.f / 64.f, 4.f/ 64.f,
    //                                     2.f / 64.f, 4.f / 64.f, 2.f/ 64.f,
    //                                 };
    //                                 for(i32 i=0; i<3; i++) {
    //                                     for (i32 j= 0; j<3; j++)
    //                                     {
    //                                         downsampled += load_voxel_level(src_voxel - Vec3i{i - 1, j - 1, -1}, depth + 1) * kernel_02.at(i * 3 + j);
    //                                     }
    //                                 }
    //                                 for(i32 i=0; i<3; i++) {
    //                                     for (i32 j= 0; j<3; j++)
    //                                     {
    //                                         downsampled += load_voxel_level(src_voxel - Vec3i{i - 1, j - 1, 0}, depth + 1) * kernel_1.at(i * 3 + j);
    //                                     }
    //                                 }
    //                                 for(i32 i=0; i<3; i++) {
    //                                     for (i32 j= 0; j<3; j++)
    //                                     {
    //                                         downsampled += load_voxel_level(src_voxel - Vec3i{i - 1, j - 1, 1}, depth + 1) * kernel_02.at(i * 3 + j);
    //                                     }
    //                                 }

    //                                 store_voxel_level(dst_voxel, depth, downsampled);
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //         }

    //         for (BrickInfo brick_info : bricks_at_depth_[depth])
    //         {
    //             Vec3i brick_id = brick_info.id;
    //             i32 address = brick_info.address;

    //             // copy edges to neighbour bricks! (might need to allocate new nodes!)
    //             if constexpr (TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCenter)
    //             {
    //                 // iterate all sides
    //                 //static_cast(false);
    //             }
    //             else if (TPool::BRICK_VOXEL_POS == BrickVoxelPosition::NodeCorner)
    //             {
    //                 copy_border({brick_id, depth}, address,
    //                             {.begin = Vec3i{},
    //                              .end = {1, TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1},
    //                              .brick_neighbour_offset = {-1, 0, 0}});
    //                 copy_border({brick_id, depth}, address, {.begin = {}, .end = {TPool::BRICK_SIZE - 1, 1, TPool::BRICK_SIZE - 1}, .brick_neighbour_offset = {0, -1, 0}});
    //                 copy_border({brick_id, depth}, address, {.begin = {}, .end = {TPool::BRICK_SIZE - 1, TPool::BRICK_SIZE - 1, 1}, .brick_neighbour_offset = {0, 0, -1}});
    //                 //
    //                 copy_border({brick_id, depth}, address, {.begin = {}, .end = {1, 1, TPool::BRICK_SIZE - 1}, .brick_neighbour_offset = {-1, -1, 0}});
    //                 copy_border({brick_id, depth}, address, {.begin = {}, .end = {1, TPool::BRICK_SIZE - 1, 1}, .brick_neighbour_offset = {-1, 0, -1}});
    //                 copy_border({brick_id, depth}, address, {.begin = {}, .end = {TPool::BRICK_SIZE - 1, 1, 1}, .brick_neighbour_offset = {0, -1, -1}});
    //                 //
    //                 copy_border({brick_id, depth}, address, {.begin = {}, .end = {1, 1, 1}, .brick_neighbour_offset = {-1, -1, -1}});
    //             }
    //         }

    //         // todo: pass that replaces constant voxels will single color and frees the brick
    //     }
    // }
};

// void generate_plane(SvoPool & pool, Plane plane, Obb cube, i32 max_octree_depth, Vec4 color) {
//     std::unordered_map<Vec3i, BrickPayload> bricks;

//     const Plane local_plane = cube.transform_to_local(plane);
//     const f32 cube_side_len = cube.get_size().x;
//     const i32 bricks_num_max_depth = (1 << max_octree_depth);

//     f32 voxel_size;
//     i32 voxel_res;
//     if( SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCenter ) {
//         voxel_res = bricks_num_max_depth * (SparseOctreeBrickConfig::brick_size - 2);
//         // *-|-*--*-|-*
//         voxel_size = cube_side_len / voxel_res;
//     }
//     else if( SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCorner ) {
//         voxel_res = bricks_num_max_depth * (SparseOctreeBrickConfig::brick_size - 1) + 1;
//         // |*--*--*|
//         voxel_size = cube_side_len / (voxel_res - 1);
//     }
    
//     Ray ray_setup{ .origin = {}, .direction = {1, 0, 0} };
//     Vec3 ray_offset_dim0{0, 1, 0};
//     Vec3 ray_offset_dim1{0, 0, 1};

//     if(glm::dot(ray_setup.direction, local_plane.normal) == 0) {
//         ray_setup.direction = {0, 1, 0};
//         ray_offset_dim0 = {1, 0, 0};
//     }

//     if(SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCenter) {
//         ray_setup.origin += (ray_offset_dim0 + ray_offset_dim1) * voxel_size * 0.5f;
//     }

//     for(i32 y=0; y<voxel_res; y++) {
//         for(i32 x=0; x<voxel_res; x++) {
//             Ray ray {.origin = ray_setup.origin + ray_offset_dim0 * (f32)x + ray_offset_dim1 * (f32)y, .direction = ray_setup.direction}; 
//             f32 t = -1.f;
//             if(glm::intersectRayPlane(ray.origin, ray.direction, local_plane.origin, local_plane.normal, t)) {
//                 Vec3 hit = ray.origin + ray.direction * t;
//                 Vec3i voxel_coord = (hit - ray_setup.origin) / voxel_size;
//                 Vec3i brick_id;
//                 Vec3i in_brick_coord;
//                 if constexpr(SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCenter) {
//                     brick_id = voxel_coord / (SparseOctreeBrickConfig::brick_size - 2);
//                     in_brick_coord = voxel_coord - brick_id * (SparseOctreeBrickConfig::brick_size - 2);
//                 }
//                 else if constexpr(SparseOctreeBrickConfig::brick_voxel_position == SparseOctreeBrickConfig::BrickVoxelNodeCorner) {
//                     brick_id = voxel_coord / (SparseOctreeBrickConfig::brick_size - 1);
//                     in_brick_coord = voxel_coord - brick_id * (SparseOctreeBrickConfig::brick_size - 1);
//                 }

//                 auto brick_iter = bricks.find(brick_id);
//                 if(brick_iter == bricks.end()) {
//                     brick_iter = bricks.insert({brick_id, BrickPayload{}}).first;
//                     brick_iter->second.init();
//                 }
//                 BrickPayload& brick = brick_iter->second;
//                 brick.set_voxel_color(in_brick_coord, color);
//             }
//         }
//     }

//     // build 
//     // traverse the bricks bottom-top to build the nodes structure
// }

