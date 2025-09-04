const std = @import("std");
const math = std.math;
const spatial = @import("spatial.zig");
const reset_module = @import("reset.zig");
const generational = @import("generational.zig");
const mouse = @import("mouse.zig");
const physics = @import("physics.zig");

pub const GRID_COUNT = 5;
pub const GRID_PARTICLE_SIZE = 12;
pub const PARTICLES_PER_GRID = GRID_PARTICLE_SIZE * GRID_PARTICLE_SIZE;
pub const TOTAL_GRID_PARTICLES = GRID_COUNT * PARTICLES_PER_GRID;
pub const FREE_AGENT_COUNT = 1000;
pub const EXTRA_PARTICLE_SLOTS = 10000;
pub const PARTICLE_COUNT = TOTAL_GRID_PARTICLES + FREE_AGENT_COUNT + EXTRA_PARTICLE_SLOTS;

pub const PARTICLE_SIZE = 5.0;
const PARTICLE_MASS = 1.0;
pub const WORLD_SIZE = 1.95;

var world_width: f32 = WORLD_SIZE;
var world_height: f32 = WORLD_SIZE;

const XPBD_ITERATIONS = 6;
const XPBD_SUBSTEPS = 6;

const DISTANCE_STIFFNESS = 10000000.0;
const COLLISION_STIFFNESS = 1.0;
const MOUSE_STIFFNESS = 50000.0;

const AIR_DAMPING = 0.99;
const GRAVITY = 25.0;

pub const SPRING_REST_LENGTH = PARTICLE_SIZE * 3.1;
const SEPARATION_RADIUS = PARTICLE_SIZE * 1.5;
pub const GRID_SPACING = PARTICLE_SIZE * 2.6;

const BOIDS_SEPARATION_RADIUS = PARTICLE_SIZE * 3.0;
const BOIDS_ALIGNMENT_RADIUS = PARTICLE_SIZE * 5.0;
const BOIDS_COHESION_RADIUS = PARTICLE_SIZE * 7.0;
const BOIDS_SEPARATION_STIFFNESS = 1.0;
const BOIDS_ALIGNMENT_STIFFNESS = 10.0;
const BOIDS_COHESION_STIFFNESS = 50.0;

const SPRING_STRENGTH = DISTANCE_STIFFNESS;
const SEPARATION_STRENGTH = COLLISION_STIFFNESS;

const GRID_MAX_SPRINGS = GRID_COUNT * PARTICLES_PER_GRID * 4;
const USER_ADDED_MAX_SPRINGS = EXTRA_PARTICLE_SLOTS * 2;
pub const MAX_SPRINGS = GRID_MAX_SPRINGS + USER_ADDED_MAX_SPRINGS;

const ParticleArena = generational.GenerationalArena(Particle, PARTICLE_COUNT);
const SpringArena = generational.GenerationalArena(Spring, MAX_SPRINGS);

pub const ParticleHandle = ParticleArena.Handle;
pub const SpringHandle = SpringArena.Handle;

pub const Particle = struct {
    x: f32,
    y: f32,
    predicted_x: f32,
    predicted_y: f32,
    vx: f32,
    vy: f32,
    mass: f32,
    grid_id: u8,
    desired_valence: u8,
    current_valence: u8,

    const Self = @This();

    pub fn init(x: f32, y: f32, grid: u8) Self {
        return Self{
            .x = x,
            .y = y,
            .predicted_x = x,
            .predicted_y = y,
            .vx = (x * 0.1),
            .vy = (y * 0.1),
            .mass = PARTICLE_MASS,
            .grid_id = grid,
            .desired_valence = 2,
            .current_valence = 0,
        };
    }

    pub fn initWithValence(x: f32, y: f32, grid: u8, valence: u8) Self {
        return Self{
            .x = x,
            .y = y,
            .predicted_x = x,
            .predicted_y = y,
            .vx = (x * 0.1),
            .vy = (y * 0.1),
            .mass = PARTICLE_MASS,
            .grid_id = grid,
            .desired_valence = valence,
            .current_valence = 0,
        };
    }

    pub fn predictPosition(self: *Self, dt: f32) void {
        self.vy -= GRAVITY * dt;

        self.vx *= AIR_DAMPING;
        self.vy *= AIR_DAMPING;

        self.predicted_x = self.x + self.vx * dt;
        self.predicted_y = self.y + self.vy * dt;

        const border_x = world_width / 2.0;
        const border_y = world_height / 2.0;
        if (self.predicted_x > border_x) {
            self.predicted_x = border_x;
            self.vx *= -0.8;
        }
        if (self.predicted_x < -border_x) {
            self.predicted_x = -border_x;
            self.vx *= -0.8;
        }
        if (self.predicted_y > border_y) {
            self.predicted_y = border_y;
            self.vy *= -0.8;
        }
        if (self.predicted_y < -border_y) {
            self.predicted_y = -border_y;
            self.vy *= -0.8;
        }
    }

    pub fn updateFromPrediction(self: *Self, dt: f32) void {
        self.vx = (self.predicted_x - self.x) / dt;
        self.vy = (self.predicted_y - self.y) / dt;

        self.x = self.predicted_x;
        self.y = self.predicted_y;
    }

    pub fn applyBoundaryConstraints(self: *Self) void {
        const border_x = world_width / 2.0;
        const border_y = world_height / 2.0;

        if (self.predicted_x > border_x) {
            self.predicted_x = border_x;
        }
        if (self.predicted_x < -border_x) {
            self.predicted_x = -border_x;
        }
        if (self.predicted_y > border_y) {
            self.predicted_y = border_y;
        }
        if (self.predicted_y < -border_y) {
            self.predicted_y = -border_y;
        }
    }
};

pub const Spring = struct {
    particle_a: ParticleHandle,
    particle_b: ParticleHandle,
    rest_length: f32,

    pub fn init(particle_a: ParticleHandle, particle_b: ParticleHandle, rest_length: f32) Spring {
        return Spring{
            .particle_a = particle_a,
            .particle_b = particle_b,
            .rest_length = rest_length,
        };
    }
};

const MAX_DISTANCE_CONSTRAINTS = MAX_SPRINGS;
const MAX_COLLISION_CONSTRAINTS = 50000;
const MAX_BOIDS_SEPARATION_CONSTRAINTS = FREE_AGENT_COUNT * 16;
const MAX_BOIDS_ALIGNMENT_CONSTRAINTS = FREE_AGENT_COUNT;
const MAX_BOIDS_COHESION_CONSTRAINTS = FREE_AGENT_COUNT;

var distance_constraints: [MAX_DISTANCE_CONSTRAINTS]physics.DistanceConstraint = undefined;
var collision_constraints: [MAX_COLLISION_CONSTRAINTS]physics.CollisionConstraint = undefined;
var boids_separation_constraints: [MAX_BOIDS_SEPARATION_CONSTRAINTS]physics.SeparationConstraint = undefined;
var boids_alignment_constraints: [MAX_BOIDS_ALIGNMENT_CONSTRAINTS]physics.AlignmentConstraint = undefined;
var boids_cohesion_constraints: [MAX_BOIDS_COHESION_CONSTRAINTS]physics.CohesionConstraint = undefined;

var particle_arena: generational.GenerationalArena(Particle, PARTICLE_COUNT) = undefined;
var particles_initialized = false;

var spring_arena: generational.GenerationalArena(Spring, MAX_SPRINGS) = undefined;
var springs_initialized = false;

pub const MAX_CONNECTIONS_PER_PARTICLE = 6;
var particle_connections: [PARTICLE_COUNT][MAX_CONNECTIONS_PER_PARTICLE]SpringHandle = undefined;
var particle_connection_counts: [PARTICLE_COUNT]u8 = undefined;

var particle_bulk_buffer: [PARTICLE_COUNT * 4]f32 = undefined;
var spring_bulk_buffer: [MAX_SPRINGS * 4]f32 = undefined;

var physics_system: physics.PhysicsSystem = undefined;

extern fn emscripten_webgpu_get_device() u32;
extern fn console_log(ptr: [*]const u8, len: usize) void;

var device_handle: u32 = 0;

fn log(comptime fmt: []const u8, args: anytype) void {
    var buffer: [1024]u8 = undefined;
    const message = std.fmt.bufPrint(buffer[0..], fmt, args) catch "Log message too long";
    console_log(message.ptr, message.len);
}

fn initializeParticleSystems() void {
    particle_arena = ParticleArena.init();
}

fn initializeSpringSystems() void {
    spring_arena = SpringArena.init();
}

// Physics system callback functions
fn getParticlePtrCallback(handle: ParticleHandle) ?*anyopaque {
    if (particle_arena.getMut(handle)) |particle| {
        return @ptrCast(particle);
    }
    return null;
}

fn getSpringPtrCallback(handle: SpringHandle) ?*anyopaque {
    if (spring_arena.getMut(handle)) |spring| {
        return @ptrCast(spring);
    }
    return null;
}

fn destroySpringCallback(handle: SpringHandle) void {
    destroySpring(handle);
}

fn getDenseParticleIndexCallback(handle: ParticleHandle) ?u32 {
    return particle_arena.getDenseIndex(handle);
}

fn initializePhysicsSystem() void {
    physics_system = physics.PhysicsSystem.init(
        &distance_constraints,
        &collision_constraints,
        &boids_separation_constraints,
        &boids_alignment_constraints,
        &boids_cohesion_constraints,
        getParticlePtrCallback,
        getSpringPtrCallback,
        destroySpringCallback,
        getDenseParticleIndexCallback,
    );
}

pub fn spawnParticle(particle: Particle) ParticleHandle {
    return particle_arena.spawn(particle);
}

pub fn destroyParticle(handle: ParticleHandle) void {
    particle_arena.destroy(handle);
    if (handle.index < PARTICLE_COUNT) {
        particle_connection_counts[handle.index] = 0;
    }
}

pub fn getParticlePtr(handle: ParticleHandle) ?*Particle {
    return particle_arena.getMut(handle);
}

pub fn getParticle(handle: ParticleHandle) ?Particle {
    if (particle_arena.get(handle)) |particle| {
        return particle.*;
    }
    return null;
}

pub fn spawnSpring(spring: Spring) SpringHandle {
    return spring_arena.spawn(spring);
}

pub fn destroySpring(handle: SpringHandle) void {
    spring_arena.destroy(handle);
}

pub fn getSpringPtr(handle: SpringHandle) ?*Spring {
    return spring_arena.getMut(handle);
}

pub fn getSpring(handle: SpringHandle) ?Spring {
    if (spring_arena.get(handle)) |spring| {
        return spring.*;
    }
    return null;
}

fn predictPositionsForAliveParticles(dt: f32) void {
    const dense_data = particle_arena.getDenseDataMut();
    const dense_handles = particle_arena.getDenseHandles();
    for (dense_data, 0..) |*particle, i| {
        if (mouse.isMouseParticle(dense_handles[i])) {
            continue;
        }
        particle.predictPosition(dt);
    }
}

fn updatePositionsForAliveParticles(dt: f32) void {
    const dense_data = particle_arena.getDenseDataMut();
    const dense_handles = particle_arena.getDenseHandles();
    for (dense_data, 0..) |*particle, i| {
        if (mouse.isMouseParticle(dense_handles[i])) {
            continue;
        }
        particle.updateFromPrediction(dt);
    }
}

fn resetValenceForAliveParticles() void {
    const dense_data = particle_arena.getDenseDataMut();
    for (dense_data) |*particle| {
        particle.current_valence = 0;
    }
}

fn countValenceFromAlivesprings() void {
    spring_arena.forEachDense(struct {
        fn countValence(spring_handle: SpringHandle, spring: *const Spring) void {
            _ = spring_handle;

            if (particle_arena.getMut(spring.particle_a)) |particle_a| {
                particle_a.current_valence += 1;
            }
            if (particle_arena.getMut(spring.particle_b)) |particle_b| {
                particle_b.current_valence += 1;
            }
        }
    }.countValence);
}

pub fn getParticleHandleByIndex(index: u32) ?ParticleHandle {
    const dense_handles = particle_arena.getDenseHandles();
    const dense_count = particle_arena.getDenseCount();

    if (index >= dense_count) return null;
    return dense_handles[index];
}

pub fn getParticleByIndex(index: u32) ?Particle {
    const dense_particles = particle_arena.getDenseData();
    const dense_count = particle_arena.getDenseCount();

    if (index >= dense_count) return null;
    return dense_particles[index];
}

pub fn findClosestParticleIndex(target_x: f32, target_y: f32) u32 {
    var closest_index: u32 = 0;
    var closest_distance_sq: f32 = std.math.inf(f32);

    const dense_particles = particle_arena.getDenseData();
    const dense_count = particle_arena.getDenseCount();

    for (0..dense_count) |i| {
        const particle = &dense_particles[i];
        const dx = target_x - particle.x;
        const dy = target_y - particle.y;
        const distance_sq = dx * dx + dy * dy;

        if (distance_sq < closest_distance_sq) {
            closest_distance_sq = distance_sq;
            closest_index = @intCast(i);
        }
    }

    return closest_index;
}

fn rebuildDenseArrays() void {
    particle_arena.rebuildDenseArrays();
}

fn writeDenseToSparse() void {
    particle_arena.writeDenseToSparse();
}

pub fn getSpringCount() u32 {
    return spring_arena.getAliveCount();
}

pub fn getAliveParticleCount() u32 {
    return particle_arena.getAliveCount();
}

pub fn getParticleConnectionCount(particle_index: u32) u8 {
    return particle_connection_counts[particle_index];
}

pub fn setParticleConnectionCount(particle_index: u32, count: u8) void {
    particle_connection_counts[particle_index] = count;
}

pub fn setParticleConnection(particle_index: u32, connection_index: u8, spring_handle: SpringHandle) void {
    particle_connections[particle_index][connection_index] = spring_handle;
}

fn updateValenceBonds() void {
    const dense_particles = particle_arena.getDenseDataMut();
    const dense_particle_handles = particle_arena.getDenseHandles();
    const dense_particle_count = particle_arena.getDenseCount();

    for (0..dense_particle_count) |i| {
        const handle_a = dense_particle_handles[i];
        const particle_a = &dense_particles[i];

        if (particle_a.current_valence >= particle_a.desired_valence) continue;

        for ((i + 1)..dense_particle_count) |j| {
            const handle_b = dense_particle_handles[j];
            const particle_b = &dense_particles[j];

            if (particle_b.current_valence >= particle_b.desired_valence) continue;

            const dx = particle_b.x - particle_a.x;
            const dy = particle_b.y - particle_a.y;
            const distance = @sqrt(dx * dx + dy * dy);

            const min_bond_distance = SPRING_REST_LENGTH * 0.9;
            const max_bond_distance = SPRING_REST_LENGTH * 1.1;

            if (distance >= min_bond_distance and distance <= max_bond_distance) {
                var already_connected = false;
                const dense_springs = spring_arena.getDenseData();
                const spring_count = spring_arena.getDenseCount();

                for (0..spring_count) |s| {
                    const spring = &dense_springs[s];
                    if ((spring.particle_a.eql(handle_a) and spring.particle_b.eql(handle_b)) or
                        (spring.particle_a.eql(handle_b) and spring.particle_b.eql(handle_a)))
                    {
                        already_connected = true;
                        break;
                    }
                }

                if (!already_connected and spring_arena.getAliveCount() < MAX_SPRINGS) {
                    const new_spring = Spring{
                        .particle_a = handle_a,
                        .particle_b = handle_b,
                        .rest_length = SPRING_REST_LENGTH,
                    };
                    const spring_handle = spawnSpring(new_spring);
                    if (spring_handle.isValid()) {
                        reset_module.addParticleConnection(handle_a.index, spring_handle);
                        reset_module.addParticleConnection(handle_b.index, spring_handle);

                        dense_particles[i].current_valence += 1;
                        dense_particles[j].current_valence += 1;

                        if (dense_particles[i].current_valence >= dense_particles[i].desired_valence) break;
                    }
                }
            }
        }
    }
}

fn initializeValenceCounts() void {
    resetValenceForAliveParticles();
    countValenceFromAlivesprings();
}

export fn init() void {
    log("Initializing enhanced particle system: {} grids + {} free agents...", .{ GRID_COUNT, FREE_AGENT_COUNT });
    device_handle = emscripten_webgpu_get_device();
    log("WebGPU device initialized: {}", .{device_handle});

    spatial.initializeGrid();

    if (!particles_initialized) {
        initializeParticleSystems();
        initializeSpringSystems();
        initializePhysicsSystem();
        reset_module.initializeGridParticles();
        reset_module.initializeFreeAgents();
        particles_initialized = true;
        log("Initialized {} total particles: {} grid + {} free agents", .{ PARTICLE_COUNT, TOTAL_GRID_PARTICLES, FREE_AGENT_COUNT });
    }

    if (!springs_initialized) {
        reset_module.initializeSprings();
        springs_initialized = true;
        log("Created {} spring connections for {} grids", .{ getSpringCount(), GRID_COUNT });
    }

    mouse.initMouseSystem();
    log("Mouse interaction system initialized", .{});
}

export fn reset() void {
    log("Resetting particle system...", .{});

    mouse.initMouseSystem();

    initializeParticleSystems();
    initializeSpringSystems();
    initializePhysicsSystem();

    reset_module.initializeGridParticles();
    reset_module.initializeFreeAgents();

    initializeValenceCounts();

    log("Particle system reset complete", .{});
}

export fn update_particles(dt: f32) void {
    if (!particles_initialized) return;

    rebuildDenseArrays();

    predictPositionsForAliveParticles(dt);

    mouse.updateMousePhysics();

    updateValenceBonds();

    // Use physics system to generate and solve constraints
    const dense_springs = spring_arena.getDenseData();
    const dense_spring_handles = spring_arena.getDenseHandles();
    const spring_count = spring_arena.getDenseCount();
    const dense_particles = particle_arena.getDenseDataMut();
    const dense_particle_handles = particle_arena.getDenseHandles();
    const dense_particle_count = particle_arena.getDenseCount();

    physics_system.generateConstraints(
        dt,
        dense_springs,
        dense_spring_handles,
        spring_count,
        dense_particles,
        dense_particle_handles,
        dense_particle_count,
        DISTANCE_STIFFNESS,
        COLLISION_STIFFNESS,
    );

    for (0..XPBD_ITERATIONS) |_| {
        physics_system.solveConstraints(dt, particle_arena.getDenseDataMut());
    }

    for (0..XPBD_SUBSTEPS) |_| {
        physics_system.solveCollisionConstraintsOnly(dt, particle_arena.getDenseDataMut());
    }

    updatePositionsForAliveParticles(dt);

    writeDenseToSparse();
}

export fn get_particle_count() i32 {
    return PARTICLE_COUNT;
}

export fn get_world_size() f32 {
    return WORLD_SIZE;
}

pub export fn get_world_width() f32 {
    return world_width;
}

pub export fn get_world_height() f32 {
    return world_height;
}

pub export fn set_world_dimensions(width: f32, height: f32) void {
    world_width = width;
    world_height = height;
    spatial.updateGridDimensions();
    log("World dimensions updated: {d:.2} x {d:.2}", .{ world_width, world_height });
}

export fn get_grid_size() i32 {
    return @as(i32, @intCast(@max(spatial.grid_size_x, spatial.grid_size_y)));
}

export fn get_grid_dimensions_x() i32 {
    return @as(i32, @intCast(spatial.grid_size_x));
}

export fn get_grid_dimensions_y() i32 {
    return @as(i32, @intCast(spatial.grid_size_y));
}

export fn get_world_width_debug() f32 {
    return world_width;
}

export fn get_world_height_debug() f32 {
    return world_height;
}

export fn get_max_particles() i32 {
    return PARTICLE_COUNT + 1000;
}

export fn get_spatial_max_occupancy() i32 {
    return @as(i32, @intCast(spatial.getMaxOccupancy()));
}

export fn get_max_springs() i32 {
    return MAX_SPRINGS;
}

export fn get_particle_size() f32 {
    return PARTICLE_SIZE;
}

export fn get_spring_count() i32 {
    return @intCast(getSpringCount());
}

export fn get_spring_particle_a(spring_index: i32) i32 {
    if (spring_index < 0) return -1;

    const current_spring_index: i32 = 0;
    _ = current_spring_index;
    return -1;
}

export fn get_spring_particle_b(spring_index: i32) i32 {
    _ = spring_index;
    return -1;
}

export fn get_particle_data(index: i32) f32 {
    if (!particles_initialized or index < 0 or index >= @as(i32, @intCast(getAliveParticleCount())) * 2) {
        return 0.0;
    }

    const particle_index = @divFloor(@as(usize, @intCast(index)), 2);
    const coord_index = @mod(@as(usize, @intCast(index)), 2);

    _ = particle_index;
    _ = coord_index;
    return 0.0;
}

export fn get_particle_valence(particle_index: i32) i32 {
    _ = particle_index;
    return 0;
}

export fn get_particle_current_valence(particle_index: i32) i32 {
    _ = particle_index;
    return 0;
}

export fn add_particle(x: f32, y: f32, valence: u32) void {
    if (!particles_initialized) return;

    if (particle_arena.getAliveCount() >= PARTICLE_COUNT) {
        log("Cannot add more particles - reached limit of {}", .{PARTICLE_COUNT});
        return;
    }

    const new_particle = Particle.initWithValence(x, y, 255, @intCast(valence));
    const handle = spawnParticle(new_particle);

    if (!handle.isValid()) {
        log("Failed to spawn particle", .{});
        return;
    }
}

export fn destroy_particle_by_index(particle_index: i32) void {
    _ = particle_index;
}

export fn get_alive_particle_count() i32 {
    return @intCast(getAliveParticleCount());
}

export fn get_alive_spring_count() i32 {
    return @intCast(getSpringCount());
}

export fn get_particle_data_bulk() [*]f32 {
    rebuildDenseArrays();

    const dense_particles = particle_arena.getDenseData();
    const dense_particle_count = particle_arena.getDenseCount();
    var write_index: u32 = 0;
    for (0..dense_particle_count) |i| {
        const particle = &dense_particles[i];

        if (write_index + 3 < PARTICLE_COUNT * 4) {
            particle_bulk_buffer[write_index] = particle.x;
            particle_bulk_buffer[write_index + 1] = particle.y;
            particle_bulk_buffer[write_index + 2] = @floatFromInt(particle.desired_valence);
            particle_bulk_buffer[write_index + 3] = @floatFromInt(particle.current_valence);
            write_index += 4;
        }
    }

    return &particle_bulk_buffer;
}

export fn get_spring_data_bulk() [*]f32 {
    rebuildDenseArrays();
    spring_arena.rebuildDenseArrays();

    const dense_springs = spring_arena.getDenseData();
    const spring_count = spring_arena.getDenseCount();
    var write_index: u32 = 0;

    for (0..spring_count) |i| {
        const spring = &dense_springs[i];

        const idx_a = particle_arena.getDenseIndex(spring.particle_a);
        const idx_b = particle_arena.getDenseIndex(spring.particle_b);

        if (idx_a != null and idx_b != null and write_index + 3 < MAX_SPRINGS * 4) {
            const dense_particles = particle_arena.getDenseData();
            const particle_a = &dense_particles[idx_a.?];
            const particle_b = &dense_particles[idx_b.?];

            spring_bulk_buffer[write_index] = particle_a.x;
            spring_bulk_buffer[write_index + 1] = particle_a.y;
            spring_bulk_buffer[write_index + 2] = particle_b.x;
            spring_bulk_buffer[write_index + 3] = particle_b.y;
            write_index += 4;
        }
    }

    return &spring_bulk_buffer;
}

export fn get_bulk_particle_count() i32 {
    return @intCast(particle_arena.getDenseCount());
}

export fn get_bulk_spring_count() i32 {
    return @intCast(getSpringCount());
}

export fn set_mouse_interaction(x: f32, y: f32, pressed: bool) void {
    mouse.updateMousePosition(x, y);
    mouse.setMousePressed(pressed);
}

export fn get_mouse_connected_particle() i32 {
    if (mouse.getGrabbedParticle(0)) |handle| {
        return @intCast(handle.index);
    }
    return -1;
}

export fn get_mouse_position_x() f32 {
    return mouse.getMousePositionX();
}

export fn get_mouse_position_y() f32 {
    return mouse.getMousePositionY();
}

export fn get_mouse_grab_count() i32 {
    return @intCast(mouse.getGrabCount());
}

export fn is_mouse_pressed() bool {
    return mouse.isMousePressed();
}