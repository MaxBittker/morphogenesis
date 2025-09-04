const std = @import("std");
const math = std.math;
const spatial = @import("spatial.zig");
const reset_module = @import("reset.zig");
const generational = @import("generational.zig");
const mouse = @import("mouse.zig");

// XPBD particle system: grids + free agents
pub const GRID_COUNT = 5;
pub const GRID_PARTICLE_SIZE = 12;
pub const PARTICLES_PER_GRID = GRID_PARTICLE_SIZE * GRID_PARTICLE_SIZE;
pub const TOTAL_GRID_PARTICLES = GRID_COUNT * PARTICLES_PER_GRID;
pub const FREE_AGENT_COUNT = 1000; // Initial free agents
pub const EXTRA_PARTICLE_SLOTS = 10000; // Extra slots for user-added particles
pub const PARTICLE_COUNT = TOTAL_GRID_PARTICLES + FREE_AGENT_COUNT + EXTRA_PARTICLE_SLOTS;

// Particle physical properties
pub const PARTICLE_SIZE = 5.0; // Physical radius in pixels
const PARTICLE_MASS = 1.0; // Normalized particle mass
pub const WORLD_SIZE = 1.95; // Base world size (height for portrait, width for landscape)

// Aspect-ratio aware world dimensions (updated from JavaScript)
var world_width: f32 = WORLD_SIZE;
var world_height: f32 = WORLD_SIZE;

// XPBD solver parameters
const XPBD_ITERATIONS = 6; // Constraint solver iterations per timestep
const XPBD_SUBSTEPS = 6; // Number of substeps per frame

// Constraint parameters (compliance = 1/(stiffness * dt²))
const DISTANCE_STIFFNESS = 10000000.0; // Very stiff rods
const COLLISION_STIFFNESS = 1.0; // Much stiffer for snappy collisions
const MOUSE_STIFFNESS = 50000.0;

// Constraint distances
pub const SPRING_REST_LENGTH = PARTICLE_SIZE * 3.1; // Natural spring length between connected particles
const SEPARATION_RADIUS = PARTICLE_SIZE * 1.5; // Minimum distance between particles
pub const GRID_SPACING = PARTICLE_SIZE * 2.6; // Base spacing between grid particles (avoid overlaps)

// Boids constraint parameters
const BOIDS_SEPARATION_RADIUS = PARTICLE_SIZE * 3.0; // Separation distance for boids
const BOIDS_ALIGNMENT_RADIUS = PARTICLE_SIZE * 5.0; // Alignment radius for boids
const BOIDS_COHESION_RADIUS = PARTICLE_SIZE * 7.0; // Cohesion radius for boids
const BOIDS_SEPARATION_STIFFNESS = 1.0;
const BOIDS_ALIGNMENT_STIFFNESS = 10.0;
const BOIDS_COHESION_STIFFNESS = 50.0;

// Air resistance
const AIR_DAMPING = 0.99; // Slight damping to prevent infinite acceleration
const GRAVITY = 25.0; // Gravity acceleration (in pixels/s²)

// Legacy constants for compatibility
const SPRING_STRENGTH = DISTANCE_STIFFNESS;
const SEPARATION_STRENGTH = COLLISION_STIFFNESS;

// Spatial partitioning (now in spatial.zig with dynamic sizing)

// XPBD Constraint Types
const ConstraintType = enum {
    distance,
    collision,
    mouse_spring,
};

// XPBD Distance Constraint (replaces springs)
const DistanceConstraint = struct {
    particle_a: ParticleHandle,
    particle_b: ParticleHandle,
    rest_length: f32,
    compliance: f32, // α = 1/(k * dt²)
    lagrange_multiplier: f32, // λ accumulated during solving

    const Self = @This();

    pub fn init(a: ParticleHandle, b: ParticleHandle, length: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .rest_length = length,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }

    pub fn evaluate(self: *const Self) f32 {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return 0.0;
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return 0.0;

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);
        return distance - self.rest_length;
    }

    pub fn getGradients(self: *const Self) struct { ga: Vec2, gb: Vec2, valid: bool } {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);

        if (distance < 1e-6) {
            return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        }

        const norm_x = dx / distance;
        const norm_y = dy / distance;

        return .{
            .ga = Vec2{ .x = -norm_x, .y = -norm_y },
            .gb = Vec2{ .x = norm_x, .y = norm_y },
            .valid = true,
        };
    }
};

// XPBD Collision Constraint (replaces separation forces)
const CollisionConstraint = struct {
    particle_a: ParticleHandle,
    particle_b: ParticleHandle,
    min_distance: f32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, b: ParticleHandle, min_dist: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .min_distance = min_dist,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }

    pub fn evaluate(self: *const Self) f32 {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return 0.0;
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return 0.0;

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);
        return @min(0.0, distance - self.min_distance); // Only active when overlapping
    }

    pub fn getGradients(self: *const Self) struct { ga: Vec2, gb: Vec2, valid: bool } {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);

        if (distance < 1e-6) {
            return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        }

        const norm_x = dx / distance;
        const norm_y = dy / distance;

        return .{
            .ga = Vec2{ .x = -norm_x, .y = -norm_y },
            .gb = Vec2{ .x = norm_x, .y = norm_y },
            .valid = true,
        };
    }
};

// XPBD Boids Constraints (replaces force-based boids)
const SeparationConstraint = struct {
    particle_a: ParticleHandle,
    particle_b: ParticleHandle,
    target_distance: f32, // Desired separation distance
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, b: ParticleHandle, target_dist: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .target_distance = target_dist,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }
};

const AlignmentConstraint = struct {
    particle_a: ParticleHandle,
    neighbor_particles: [16]ParticleHandle, // Max neighbors for alignment
    neighbor_count: u32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .neighbor_particles = undefined,
            .neighbor_count = 0,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }
};

const CohesionConstraint = struct {
    particle_a: ParticleHandle,
    target_x: f32, // Center of mass X
    target_y: f32, // Center of mass Y
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, target_x: f32, target_y: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .target_x = target_x,
            .target_y = target_y,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }
};

// Use the clean generational arena system
const ParticleArena = generational.GenerationalArena(Particle, PARTICLE_COUNT);
const SpringArena = generational.GenerationalArena(Spring, MAX_SPRINGS);

// Handle type aliases for convenience and compatibility
pub const ParticleHandle = ParticleArena.Handle;
pub const SpringHandle = SpringArena.Handle;

// Vector2 helper
pub const Vec2 = struct {
    x: f32,
    y: f32,
};

// XPBD Particle structure
pub const Particle = struct {
    x: f32,
    y: f32,
    predicted_x: f32, // Position prediction for XPBD
    predicted_y: f32,
    vx: f32,
    vy: f32,
    mass: f32,
    grid_id: u8, // Which grid this particle belongs to (0-4, or 255 for free agents)
    desired_valence: u8, // How many connections this particle wants
    current_valence: u8, // How many connections this particle currently has

    const Self = @This();

    pub fn init(x: f32, y: f32, grid: u8) Self {
        return Self{
            .x = x,
            .y = y,
            .predicted_x = x,
            .predicted_y = y,
            .vx = (x * 0.1), // Simple deterministic initial velocity
            .vy = (y * 0.1),
            .mass = PARTICLE_MASS,
            .grid_id = grid,
            .desired_valence = 2, // Default valence
            .current_valence = 0, // Start with no connections
        };
    }

    pub fn initWithValence(x: f32, y: f32, grid: u8, valence: u8) Self {
        return Self{
            .x = x,
            .y = y,
            .predicted_x = x,
            .predicted_y = y,
            .vx = (x * 0.1), // Simple deterministic initial velocity
            .vy = (y * 0.1),
            .mass = PARTICLE_MASS,
            .grid_id = grid,
            .desired_valence = valence,
            .current_valence = 0, // Start with no connections
        };
    }

    pub fn predictPosition(self: *Self, dt: f32) void {
        // Apply gravity (downward acceleration)
        self.vy -= GRAVITY * dt;

        // Apply air damping to velocity
        self.vx *= AIR_DAMPING;
        self.vy *= AIR_DAMPING;

        // Predict position using current velocity (explicit Euler prediction)
        self.predicted_x = self.x + self.vx * dt;
        self.predicted_y = self.y + self.vy * dt;

        // Handle boundary collision in prediction with aspect-aware bounds
        const border_x = world_width / 2.0;
        const border_y = world_height / 2.0;
        if (self.predicted_x > border_x) {
            self.predicted_x = border_x;
            self.vx *= -0.8; // Bounce damping
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
        // Update velocity based on position correction
        self.vx = (self.predicted_x - self.x) / dt;
        self.vy = (self.predicted_y - self.y) / dt;

        // Update position
        self.x = self.predicted_x;
        self.y = self.predicted_y;
    }

    pub fn applyBoundaryConstraints(self: *Self) void {
        // Aspect-aware boundary constraints for XPBD
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

// XPBD constraint arrays
const MAX_DISTANCE_CONSTRAINTS = MAX_SPRINGS; // Match spring capacity for distance constraints
const MAX_COLLISION_CONSTRAINTS = 50000; // Handle more particles with collisions

var distance_constraints: [MAX_DISTANCE_CONSTRAINTS]DistanceConstraint = undefined;
var distance_constraint_count: u32 = 0;

var collision_constraints: [MAX_COLLISION_CONSTRAINTS]CollisionConstraint = undefined;
var collision_constraint_count: u32 = 0;

// Boids constraint arrays
const MAX_BOIDS_SEPARATION_CONSTRAINTS = FREE_AGENT_COUNT * 16; // Max 16 neighbors per boid
const MAX_BOIDS_ALIGNMENT_CONSTRAINTS = FREE_AGENT_COUNT; // One per free agent
const MAX_BOIDS_COHESION_CONSTRAINTS = FREE_AGENT_COUNT; // One per free agent

var boids_separation_constraints: [MAX_BOIDS_SEPARATION_CONSTRAINTS]SeparationConstraint = undefined;
var boids_separation_constraint_count: u32 = 0;

var boids_alignment_constraints: [MAX_BOIDS_ALIGNMENT_CONSTRAINTS]AlignmentConstraint = undefined;
var boids_alignment_constraint_count: u32 = 0;

var boids_cohesion_constraints: [MAX_BOIDS_COHESION_CONSTRAINTS]CohesionConstraint = undefined;
var boids_cohesion_constraint_count: u32 = 0;

// Global particle system using GenerationalArena
var particle_arena: generational.GenerationalArena(Particle, PARTICLE_COUNT) = undefined;
var particles_initialized = false;

// Spring system - maximum springs for hexagonal lattice (6-connectivity) + user additions
// Each particle has up to 6 connections, but springs are shared between particles
// In practice, each particle contributes ~3 unique springs to avoid double-counting
// Safety margin: use 4 springs per particle to ensure we don't hit the limit
// User-added particles: each gets 2 springs (to nearest neighbors)
const GRID_MAX_SPRINGS = GRID_COUNT * PARTICLES_PER_GRID * 4;
const USER_ADDED_MAX_SPRINGS = EXTRA_PARTICLE_SLOTS * 2; // 2 springs per user-added particle
pub const MAX_SPRINGS = GRID_MAX_SPRINGS + USER_ADDED_MAX_SPRINGS; // ~20,000+ springs total

// Spring structure using generational handles
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

var spring_arena: generational.GenerationalArena(Spring, MAX_SPRINGS) = undefined;
var springs_initialized = false;

// Connection lookup optimization: store spring handles for each particle
pub const MAX_CONNECTIONS_PER_PARTICLE = 6; // Maximum 6 springs per grid particle (triangular lattice)
var particle_connections: [PARTICLE_COUNT][MAX_CONNECTIONS_PER_PARTICLE]SpringHandle = undefined;
var particle_connection_counts: [PARTICLE_COUNT]u8 = undefined;

// Initialize arena systems
fn initializeParticleSystems() void {
    particle_arena = ParticleArena.init();
}

fn initializeSpringSystems() void {
    spring_arena = SpringArena.init();
}

pub fn spawnParticle(particle: Particle) ParticleHandle {
    return particle_arena.spawn(particle);
}

pub fn destroyParticle(handle: ParticleHandle) void {
    particle_arena.destroy(handle);
    // Clear connections
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

// Mock WebGPU bindings (not actually used for rendering)
extern fn emscripten_webgpu_get_device() u32;

// Console logging
extern fn console_log(ptr: [*]const u8, len: usize) void;

fn log(comptime fmt: []const u8, args: anytype) void {
    var buffer: [1024]u8 = undefined;
    const message = std.fmt.bufPrint(buffer[0..], fmt, args) catch "Log message too long";
    console_log(message.ptr, message.len);
}

var device_handle: u32 = 0;

// Use the arena's dense arrays for performance

// The arena now handles dense to sparse writing

// Fast helper functions using dense arrays
fn predictPositionsForAliveParticles(dt: f32) void {
    const dense_data = particle_arena.getDenseDataMut();
    const dense_handles = particle_arena.getDenseHandles();
    for (dense_data, 0..) |*particle, i| {
        // Skip mouse particle - it should not be affected by physics
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
        // Skip mouse particle - it maintains its own position
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

            // Count valence for both connected particles
            if (particle_arena.getMut(spring.particle_a)) |particle_a| {
                particle_a.current_valence += 1;
            }
            if (particle_arena.getMut(spring.particle_b)) |particle_b| {
                particle_b.current_valence += 1;
            }
        }
    }.countValence);
}

// Critical functions that need to be restored temporarily
pub fn getParticleHandleByIndex(index: u32) ?ParticleHandle {
    // Use dense arrays to find particle handle by iteration index
    const dense_handles = particle_arena.getDenseHandles();
    const dense_count = particle_arena.getDenseCount();

    if (index >= dense_count) return null;
    return dense_handles[index];
}

pub fn getParticleByIndex(index: u32) ?Particle {
    // Use dense arrays to find particle by iteration index
    const dense_particles = particle_arena.getDenseData();
    const dense_count = particle_arena.getDenseCount();

    if (index >= dense_count) return null;
    return dense_particles[index];
}

// Find the closest particle to a given position using arena dense arrays
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

// XPBD constraint generation - creates distance, collision, and boids constraints
fn generateConstraints(dt: f32) void {
    distance_constraint_count = 0;
    collision_constraint_count = 0;
    boids_separation_constraint_count = 0;
    boids_alignment_constraint_count = 0;
    boids_cohesion_constraint_count = 0;

    // Generate distance constraints from alive springs (and check for overstretching)
    var springs_to_remove: [MAX_SPRINGS]SpringHandle = undefined;
    var remove_count: u32 = 0;

    // Iterate through dense springs for performance
    const dense_springs = spring_arena.getDenseData();
    const dense_spring_handles = spring_arena.getDenseHandles();
    const spring_count = spring_arena.getDenseCount();

    for (0..spring_count) |i| {
        const spring = &dense_springs[i];
        const spring_handle = dense_spring_handles[i];

        const particle_a = getParticlePtr(spring.particle_a);
        const particle_b = getParticlePtr(spring.particle_b);

        // Validate both particles are alive
        if (particle_a != null and particle_b != null) {
            // Check if spring is overstretched - mouse springs get higher threshold
            const dx = particle_b.?.x - particle_a.?.x;
            const dy = particle_b.?.y - particle_a.?.y;
            const current_length = @sqrt(dx * dx + dy * dy);

            // Mouse springs get much higher break threshold (10x vs 2x)
            const is_mouse_spring = mouse.isMouseParticle(spring.particle_a) or mouse.isMouseParticle(spring.particle_b);
            const max_allowed_length = if (is_mouse_spring) spring.rest_length * 10.0 else spring.rest_length * 2.0;

            if (current_length > max_allowed_length) {
                // Mark spring for removal and refund valence
                if (remove_count < MAX_SPRINGS) {
                    springs_to_remove[remove_count] = spring_handle;
                    remove_count += 1;

                    // Refund valence to both particles
                    particle_a.?.current_valence = @max(0, particle_a.?.current_valence - 1);
                    particle_b.?.current_valence = @max(0, particle_b.?.current_valence - 1);
                }
            } else if (distance_constraint_count < MAX_DISTANCE_CONSTRAINTS) {
                // Spring is fine, create constraint
                const spring_compliance = 1.0 / (DISTANCE_STIFFNESS * dt * dt);
                distance_constraints[distance_constraint_count] = DistanceConstraint{
                    .particle_a = spring.particle_a,
                    .particle_b = spring.particle_b,
                    .rest_length = spring.rest_length,
                    .compliance = spring_compliance,
                    .lagrange_multiplier = 0.0,
                };
                distance_constraint_count += 1;
            }
        }
    }

    // Remove overstretched springs
    for (0..remove_count) |j| {
        destroySpring(springs_to_remove[j]);
    }

    // Note: Valence changes are made to sparse storage during spring destruction.
    // Dense arrays will be rebuilt at the start of the next frame, preserving
    // the current physics state (velocities, positions) until then.

    // Generate collision constraints using spatial grid (use dense arrays)
    const dense_particles = particle_arena.getDenseDataMut();
    const dense_particle_handles = particle_arena.getDenseHandles();
    const dense_particle_count = particle_arena.getDenseCount();

    spatial.populateGrid(dense_particles, dense_particle_count);

    for (0..dense_particle_count) |i| {
        // Skip mouse particle - it should not participate in collisions
        if (!mouse.isMouseParticle(dense_particle_handles[i])) {
            generateCollisionConstraintsForParticle(dense_particle_handles[i], dt, dense_particle_handles, dense_particle_count);
        }
    }

    // Generate boids constraints for initial free agents only (not user-added particles)
    // For now, skip boids constraints until we update them properly
    // TODO: Update boids constraint generation for generational system
}

// Generate collision constraints for a single particle using spatial grid
fn generateCollisionConstraintsForParticle(particle_handle: ParticleHandle, dt: f32, handles_array: []const ParticleHandle, handles_count: u32) void {
    const particle = getParticlePtr(particle_handle) orelse return;

    // Check current cell and 8 surrounding cells
    const gx = spatial.worldToGridX(particle.predicted_x);
    const gy = spatial.worldToGridY(particle.predicted_y);

    var dy: i32 = -1;
    while (dy <= 1) : (dy += 1) {
        var dx: i32 = -1;
        while (dx <= 1) : (dx += 1) {
            const check_x = gx + dx;
            const check_y = gy + dy;

            if (check_x >= 0 and check_x < @as(i32, @intCast(spatial.grid_size_x)) and check_y >= 0 and check_y < @as(i32, @intCast(spatial.grid_size_y))) {
                const cell = &spatial.spatial_grid[@intCast(check_x)][@intCast(check_y)];

                for (0..cell.count) |i| {
                    const neighbor_index = cell.particles[i];

                    // Get handle for the neighbor particle from the handles array
                    if (neighbor_index >= handles_count) continue;
                    const neighbor_handle = handles_array[neighbor_index];

                    // Avoid duplicates by comparing handle indices and only process if current < neighbor
                    // Also skip if either particle is the mouse particle
                    if (!neighbor_handle.eql(particle_handle) and
                        particle_handle.index < neighbor_handle.index and
                        !mouse.isMouseParticle(neighbor_handle))
                    {
                        const other = getParticlePtr(neighbor_handle) orelse continue;
                        const dx_pred = particle.predicted_x - other.predicted_x;
                        const dy_pred = particle.predicted_y - other.predicted_y;
                        const dist_sq = dx_pred * dx_pred + dy_pred * dy_pred;

                        // Billiard ball collision: only when actually overlapping
                        const ball_radius = PARTICLE_SIZE;
                        const contact_distance = ball_radius * 2.0;
                        const current_distance = @sqrt(dist_sq);

                        // Only create collision constraint when balls overlap (billiard ball style)
                        if (current_distance < contact_distance) {
                            if (collision_constraint_count < MAX_COLLISION_CONSTRAINTS) {
                                // Very high stiffness for billiard ball collisions (10x stronger)
                                const billiard_stiffness = COLLISION_STIFFNESS * 10.0;
                                const collision_compliance = 1.0 / (billiard_stiffness * dt * dt);

                                collision_constraints[collision_constraint_count] = CollisionConstraint{
                                    .particle_a = particle_handle,
                                    .particle_b = neighbor_handle,
                                    .min_distance = contact_distance,
                                    .compliance = collision_compliance,
                                    .lagrange_multiplier = 0.0,
                                };
                                collision_constraint_count += 1;
                            }
                        }
                    }
                }
            }
        }
    }
}

// TODO: Reimplement boids constraint generation for generational system

// XPBD constraint solver - solves all constraints using Lagrange multipliers
fn solveConstraints(dt: f32) void {
    // Solve distance constraints
    for (0..distance_constraint_count) |i| {
        solveDistanceConstraint(&distance_constraints[i], dt);
    }

    // Solve collision constraints
    for (0..collision_constraint_count) |i| {
        solveCollisionConstraint(&collision_constraints[i], dt);
    }

    // Solve boids constraints
    for (0..boids_separation_constraint_count) |i| {
        solveSeparationConstraint(&boids_separation_constraints[i], dt);
    }

    for (0..boids_alignment_constraint_count) |i| {
        solveAlignmentConstraint(&boids_alignment_constraints[i], dt);
    }

    for (0..boids_cohesion_constraint_count) |i| {
        solveCohesionConstraint(&boids_cohesion_constraints[i], dt);
    }
}

// Solve only collision constraints for extra snappy collision response
fn solveCollisionConstraintsOnly(dt: f32) void {
    // Solve collision constraints only
    for (0..collision_constraint_count) |i| {
        solveCollisionConstraint(&collision_constraints[i], dt);
    }
}

// Fast particle lookup in dense array using arena's lookup
inline fn getDenseParticleIndex(handle: ParticleHandle) ?u32 {
    return particle_arena.getDenseIndex(handle);
}

// Solve individual distance constraint using XPBD - optimized version
fn solveDistanceConstraint(constraint: *DistanceConstraint, dt: f32) void {
    const idx_a = getDenseParticleIndex(constraint.particle_a) orelse return;
    const idx_b = getDenseParticleIndex(constraint.particle_b) orelse return;

    const dense_particles = particle_arena.getDenseDataMut();
    const particle_a = &dense_particles[idx_a];
    const particle_b = &dense_particles[idx_b];

    // Calculate constraint value C(x) = |x_a - x_b| - rest_length
    const dx = particle_a.predicted_x - particle_b.predicted_x;
    const dy = particle_a.predicted_y - particle_b.predicted_y;
    const current_distance = @sqrt(dx * dx + dy * dy);

    if (current_distance < 0.001) return; // Avoid division by zero

    const constraint_value = current_distance - constraint.rest_length;

    // Calculate constraint gradient ∇C
    const grad_x = dx / current_distance;
    const grad_y = dy / current_distance;

    // Calculate denominator for Lagrange multiplier
    // w_a * |∇C_a|² + w_b * |∇C_b|² + α/dt²
    const w_a = 1.0 / particle_a.mass;
    const w_b = 1.0 / particle_b.mass;
    const grad_length_sq = grad_x * grad_x + grad_y * grad_y;
    const denominator = w_a * grad_length_sq + w_b * grad_length_sq + constraint.compliance / (dt * dt);

    if (denominator < 0.001) return; // Avoid division by zero

    // Calculate delta Lagrange multiplier
    const delta_lambda = -(constraint_value + constraint.compliance * constraint.lagrange_multiplier / (dt * dt)) / denominator;

    // Update Lagrange multiplier
    constraint.lagrange_multiplier += delta_lambda;

    // Apply position corrections
    const correction_a_x = w_a * delta_lambda * grad_x;
    const correction_a_y = w_a * delta_lambda * grad_y;
    const correction_b_x = -w_b * delta_lambda * grad_x;
    const correction_b_y = -w_b * delta_lambda * grad_y;

    // Update predicted positions
    particle_a.predicted_x += correction_a_x;
    particle_a.predicted_y += correction_a_y;
    particle_b.predicted_x += correction_b_x;
    particle_b.predicted_y += correction_b_y;
}

// Solve individual collision constraint using billiard ball physics - optimized version
fn solveCollisionConstraint(constraint: *CollisionConstraint, dt: f32) void {
    _ = dt; // Mark as intentionally unused
    const idx_a = getDenseParticleIndex(constraint.particle_a) orelse return;
    const idx_b = getDenseParticleIndex(constraint.particle_b) orelse return;

    const dense_particles = particle_arena.getDenseDataMut();
    const particle_a = &dense_particles[idx_a];
    const particle_b = &dense_particles[idx_b];

    // Calculate constraint value C(x) = min_distance - |x_a - x_b|
    const dx = particle_a.predicted_x - particle_b.predicted_x;
    const dy = particle_a.predicted_y - particle_b.predicted_y;
    const current_distance = @sqrt(dx * dx + dy * dy);

    if (current_distance < 0.001) {
        // Handle edge case of particles at same position
        particle_a.predicted_x += 0.01;
        particle_b.predicted_x -= 0.01;
        return;
    }

    const constraint_value = constraint.min_distance - current_distance;

    // Only solve if constraint is violated (particles overlapping)
    if (constraint_value <= 0) return;

    // Calculate collision normal (from b to a)
    const normal_x = dx / current_distance;
    const normal_y = dy / current_distance;

    // POSITION CORRECTION: Separate overlapping particles
    const separation = constraint_value * 0.5; // Each particle moves half the overlap distance
    particle_a.predicted_x += normal_x * separation;
    particle_a.predicted_y += normal_y * separation;
    particle_b.predicted_x -= normal_x * separation;
    particle_b.predicted_y -= normal_y * separation;

    // ELASTIC COLLISION: Apply velocity changes for billiard ball physics
    // Calculate relative velocity along collision normal
    const rel_vx = particle_a.vx - particle_b.vx;
    const rel_vy = particle_a.vy - particle_b.vy;
    const rel_vel_normal = rel_vx * normal_x + rel_vy * normal_y;

    // Only resolve if objects are approaching (not separating)
    if (rel_vel_normal > 0) return;

    // Calculate impulse for elastic collision
    const restitution = 0.9; // Slightly less than perfectly elastic for stability
    const impulse_magnitude = -(1.0 + restitution) * rel_vel_normal / (1.0 / particle_a.mass + 1.0 / particle_b.mass);

    // Apply impulse to velocities (billiard ball momentum conservation)
    const impulse_x = impulse_magnitude * normal_x;
    const impulse_y = impulse_magnitude * normal_y;

    // Apply impulse correctly: F = dp/dt, so dv = F/m
    particle_a.vx += impulse_x / particle_a.mass;
    particle_a.vy += impulse_y / particle_a.mass;
    particle_b.vx -= impulse_x / particle_b.mass;
    particle_b.vy -= impulse_y / particle_b.mass;
}

// Solve boids separation constraint (keep particles apart)
fn solveSeparationConstraint(constraint: *SeparationConstraint, dt: f32) void {
    const particle_a = getParticlePtr(constraint.particle_a) orelse return;
    const particle_b = getParticlePtr(constraint.particle_b) orelse return;

    // Calculate constraint value C(x) = target_distance - |x_a - x_b|
    const dx = particle_a.predicted_x - particle_b.predicted_x;
    const dy = particle_a.predicted_y - particle_b.predicted_y;
    const current_distance = @sqrt(dx * dx + dy * dy);

    if (current_distance < 0.001) return; // Avoid division by zero

    // Only enforce if particles are too close (current_distance < target_distance)
    if (current_distance >= constraint.target_distance) return;

    const constraint_value = constraint.target_distance - current_distance;

    // Calculate constraint gradient ∇C (negative because we subtract distance)
    const grad_x = -dx / current_distance;
    const grad_y = -dy / current_distance;

    // Calculate denominator for Lagrange multiplier
    const w_a = 1.0 / particle_a.mass;
    const w_b = 1.0 / particle_b.mass;
    const grad_length_sq = grad_x * grad_x + grad_y * grad_y;
    const denominator = w_a * grad_length_sq + w_b * grad_length_sq + constraint.compliance / (dt * dt);

    if (denominator < 0.001) return; // Avoid division by zero

    // Calculate delta Lagrange multiplier
    const delta_lambda = -(constraint_value + constraint.compliance * constraint.lagrange_multiplier / (dt * dt)) / denominator;

    // Update Lagrange multiplier
    constraint.lagrange_multiplier += delta_lambda;

    // Apply position corrections
    const correction_a_x = w_a * delta_lambda * grad_x;
    const correction_a_y = w_a * delta_lambda * grad_y;
    const correction_b_x = -w_b * delta_lambda * grad_x;
    const correction_b_y = -w_b * delta_lambda * grad_y;

    // Update predicted positions
    particle_a.predicted_x += correction_a_x;
    particle_a.predicted_y += correction_a_y;
    particle_b.predicted_x += correction_b_x;
    particle_b.predicted_y += correction_b_y;
}

// Solve alignment constraint (align velocities with neighbors)
fn solveAlignmentConstraint(constraint: *AlignmentConstraint, dt: f32) void {
    const particle = getParticlePtr(constraint.particle_a) orelse return;

    if (constraint.neighbor_count == 0) return;

    // Calculate average neighbor velocity
    var avg_vx: f32 = 0;
    var avg_vy: f32 = 0;
    var valid_neighbors: u32 = 0;

    for (0..constraint.neighbor_count) |i| {
        if (getParticlePtr(constraint.neighbor_particles[i])) |neighbor| {
            avg_vx += neighbor.vx;
            avg_vy += neighbor.vy;
            valid_neighbors += 1;
        }
    }

    if (valid_neighbors == 0) return;

    avg_vx /= @as(f32, @floatFromInt(valid_neighbors));
    avg_vy /= @as(f32, @floatFromInt(valid_neighbors));

    // Apply velocity alignment as position correction (XPBD style)
    const velocity_diff_x = avg_vx - particle.vx;
    const velocity_diff_y = avg_vy - particle.vy;

    // Convert velocity difference to position correction
    const alignment_strength = 1.0 / (constraint.compliance + dt * dt);
    const correction_x = velocity_diff_x * dt * alignment_strength * 0.01; // Small factor for stability
    const correction_y = velocity_diff_y * dt * alignment_strength * 0.01;

    // Apply position correction
    particle.predicted_x += correction_x;
    particle.predicted_y += correction_y;
}

// Solve cohesion constraint (attract to center of mass)
fn solveCohesionConstraint(constraint: *CohesionConstraint, dt: f32) void {
    const particle = getParticlePtr(constraint.particle_a) orelse return;

    // Calculate constraint value C(x) = |x_particle - center_of_mass|
    const dx = particle.predicted_x - constraint.target_x;
    const dy = particle.predicted_y - constraint.target_y;
    const distance_to_center = @sqrt(dx * dx + dy * dy);

    if (distance_to_center < 0.001) return; // Already at center

    // We want to minimize distance to center, so constraint is just the distance
    const constraint_value = distance_to_center;

    // Calculate constraint gradient ∇C
    const grad_x = dx / distance_to_center;
    const grad_y = dy / distance_to_center;

    // Calculate correction (simple attraction)
    const cohesion_strength = 1.0 / (constraint.compliance + dt * dt);

    const correction_x = -grad_x * constraint_value * cohesion_strength * 0.001; // Very small factor for stability
    const correction_y = -grad_y * constraint_value * cohesion_strength * 0.001;

    // Apply position correction
    particle.predicted_x += correction_x;
    particle.predicted_y += correction_y;
}

export fn init() void {
    log("Initializing enhanced particle system: {} grids + {} free agents...", .{ GRID_COUNT, FREE_AGENT_COUNT });
    device_handle = emscripten_webgpu_get_device();
    log("WebGPU device initialized: {}", .{device_handle});

    // Initialize spatial grid
    spatial.initializeGrid();

    // Initialize particle and spring arenas
    if (!particles_initialized) {
        initializeParticleSystems();
        initializeSpringSystems();
        reset_module.initializeGridParticles();
        reset_module.initializeFreeAgents();
        particles_initialized = true;
        log("Initialized {} total particles: {} grid + {} free agents", .{ PARTICLE_COUNT, TOTAL_GRID_PARTICLES, FREE_AGENT_COUNT });
    }

    // Initialize spring connections
    if (!springs_initialized) {
        reset_module.initializeSprings();
        springs_initialized = true;
        log("Created {} spring connections for {} grids", .{ getSpringCount(), GRID_COUNT });
    }

    // Initialize mouse system
    mouse.initMouseSystem();
    log("Mouse interaction system initialized", .{});
}

export fn reset() void {
    log("Resetting particle system...", .{});

    // Reset mouse interaction system
    mouse.initMouseSystem();

    // Reset generational storage
    initializeParticleSystems();
    initializeSpringSystems();

    // Reinitialize all particles to starting positions
    reset_module.initializeGridParticles();
    reset_module.initializeFreeAgents();

    // Initialize valence counts for existing springs
    initializeValenceCounts();

    log("Particle system reset complete", .{});
}

export fn update_particles(dt: f32) void {
    if (!particles_initialized) return;

    // PERFORMANCE OPTIMIZATION: Rebuild dense arrays once per frame
    rebuildDenseArrays();

    // XPBD Integration Steps:
    // 1. Position prediction (using velocity)
    // 2. Constraint generation and solving
    // 3. Velocity update from position changes
    // 4. Position finalization

    // Step 1: Predict positions for all alive particles (using dense arrays)
    predictPositionsForAliveParticles(dt);

    // Update mouse particle physics (ensure it stays locked to cursor)
    mouse.updateMousePhysics();

    // Step 2: Update valence-based bonds (check for new connections)
    updateValenceBonds();

    // Generate constraints and solve them iteratively
    generateConstraints(dt);

    // Solve constraints for multiple iterations for stability
    for (0..XPBD_ITERATIONS) |_| {
        solveConstraints(dt);
    }

    // Extra collision-only passes for snappy collision response
    for (0..XPBD_SUBSTEPS) |_| {
        solveCollisionConstraintsOnly(dt);
    }

    // Step 3: Update velocities and positions from predictions (using dense arrays)
    updatePositionsForAliveParticles(dt);

    // PERFORMANCE OPTIMIZATION: Write back from dense to sparse arrays
    writeDenseToSparse();
}

export fn get_particle_count() i32 {
    return PARTICLE_COUNT;
}

// Export important constants to avoid duplication in JavaScript
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
    return PARTICLE_COUNT + 1000; // Buffer for safety
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

    // Iterate through alive springs to find the spring at the given index
    const current_spring_index: i32 = 0;
    // TODO: Reimplement with arena iteration
    _ = current_spring_index;
    return -1; // Stubbed for refactoring
}

export fn get_spring_particle_b(spring_index: i32) i32 {
    // TODO: Reimplement with arena iteration
    _ = spring_index;
    return -1; // Stubbed for refactoring
}

// Get the index of a particle in the alive particle list (for rendering)

export fn get_particle_data(index: i32) f32 {
    if (!particles_initialized or index < 0 or index >= @as(i32, @intCast(getAliveParticleCount())) * 2) {
        return 0.0;
    }

    const particle_index = @divFloor(@as(usize, @intCast(index)), 2);
    const coord_index = @mod(@as(usize, @intCast(index)), 2);

    // TODO: Reimplement with arena dense arrays
    _ = particle_index;
    _ = coord_index;
    return 0.0; // Stubbed for refactoring
}

// Get particle desired valence for rendering
export fn get_particle_valence(particle_index: i32) i32 {
    // TODO: Reimplement with arena dense arrays
    _ = particle_index;
    return 0; // Stubbed for refactoring
}

// Get particle current valence for rendering
export fn get_particle_current_valence(particle_index: i32) i32 {
    // TODO: Reimplement with arena dense arrays
    _ = particle_index;
    return 0; // Stubbed for refactoring
}

// Add a new particle at position with springs to k nearest particles
export fn add_particle(x: f32, y: f32, valence: u32) void {
    if (!particles_initialized) return;

    // Check if we have room for more particles
    if (particle_arena.getAliveCount() >= PARTICLE_COUNT) {
        log("Cannot add more particles - reached limit of {}", .{PARTICLE_COUNT});
        return;
    }

    // Create new particle and spawn it
    const new_particle = Particle.initWithValence(x, y, 255, @intCast(valence));
    const handle = spawnParticle(new_particle);

    if (!handle.isValid()) {
        log("Failed to spawn particle", .{});
        return;
    }
}

// Destroy a particle by its index in the active particle array
export fn destroy_particle_by_index(particle_index: i32) void {
    // TODO: Reimplement with arena iteration
    _ = particle_index;
    // Stubbed for refactoring
}

// Destroy all springs connected to a particle

// Get count of alive particles for JavaScript
export fn get_alive_particle_count() i32 {
    return @intCast(getAliveParticleCount());
}

// Get count of alive springs for JavaScript
export fn get_alive_spring_count() i32 {
    return @intCast(getSpringCount());
}

// BULK DATA TRANSFER FUNCTIONS FOR PERFORMANCE

// Get pointer to particle data for bulk transfer (x, y, desired_valence, current_valence per particle)
export fn get_particle_data_bulk() [*]f32 {
    // Ensure dense arrays are up to date
    rebuildDenseArrays();

    // Pack particle data: x, y, desired_valence, current_valence
    const dense_particles = particle_arena.getDenseData();
    const dense_particle_count = particle_arena.getDenseCount();
    var write_index: u32 = 0;
    for (0..dense_particle_count) |i| {
        const particle = &dense_particles[i];

        // Reuse the existing particlePositionsBuffer for bulk transfer
        if (write_index + 3 < PARTICLE_COUNT * 4) {
            // Use a static buffer for bulk transfer
            particle_bulk_buffer[write_index] = particle.x;
            particle_bulk_buffer[write_index + 1] = particle.y;
            particle_bulk_buffer[write_index + 2] = @floatFromInt(particle.desired_valence);
            particle_bulk_buffer[write_index + 3] = @floatFromInt(particle.current_valence);
            write_index += 4;
        }
    }

    return &particle_bulk_buffer;
}

// Get pointer to spring data for bulk transfer (x1, y1, x2, y2 per spring)
export fn get_spring_data_bulk() [*]f32 {
    // Ensure dense arrays are up to date
    rebuildDenseArrays();
    spring_arena.rebuildDenseArrays();

    // Pack spring vertex data using arena dense arrays
    const dense_springs = spring_arena.getDenseData();
    const spring_count = spring_arena.getDenseCount();
    var write_index: u32 = 0;

    for (0..spring_count) |i| {
        const spring = &dense_springs[i];

        // Find particles in dense array
        const idx_a = getDenseParticleIndex(spring.particle_a);
        const idx_b = getDenseParticleIndex(spring.particle_b);

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

// Get actual count of particles in bulk buffer
export fn get_bulk_particle_count() i32 {
    return @intCast(particle_arena.getDenseCount());
}

// Get actual count of springs in bulk buffer
export fn get_bulk_spring_count() i32 {
    return @intCast(getSpringCount());
}

// Static buffers for bulk data transfer
var particle_bulk_buffer: [PARTICLE_COUNT * 4]f32 = undefined;
var spring_bulk_buffer: [MAX_SPRINGS * 4]f32 = undefined;

// Check for and form valence-based bonds between particles (using dense arrays)
fn updateValenceBonds() void {

    // Get current dense arrays from arena
    const dense_particles = particle_arena.getDenseDataMut();
    const dense_particle_handles = particle_arena.getDenseHandles();
    const dense_particle_count = particle_arena.getDenseCount();

    // Check all particle pairs for potential bonds (using dense arrays)
    for (0..dense_particle_count) |i| {
        const handle_a = dense_particle_handles[i];
        const particle_a = &dense_particles[i];

        // Skip if this particle is already satisfied
        if (particle_a.current_valence >= particle_a.desired_valence) continue;

        for ((i + 1)..dense_particle_count) |j| {
            const handle_b = dense_particle_handles[j];
            const particle_b = &dense_particles[j];

            // Skip if the other particle is already satisfied
            if (particle_b.current_valence >= particle_b.desired_valence) continue;

            // Check if particles are at optimal bonding distance (not too close, not too far)
            const dx = particle_b.x - particle_a.x;
            const dy = particle_b.y - particle_a.y;
            const distance = @sqrt(dx * dx + dy * dy);

            // Only bond if distance is close to desired spring rest length
            const min_bond_distance = SPRING_REST_LENGTH * 0.9; // Not too close
            const max_bond_distance = SPRING_REST_LENGTH * 1.1; // Not too far

            if (distance >= min_bond_distance and distance <= max_bond_distance) {
                // Check if they're not already connected by iterating springs
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

                // Form a bond if not already connected and there's room
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

                        // Update valence counts in dense array
                        dense_particles[i].current_valence += 1;
                        dense_particles[j].current_valence += 1;

                        // Stop checking if both particles are now satisfied
                        if (dense_particles[i].current_valence >= dense_particles[i].desired_valence) break;
                    }
                }
            }
        }
    }
}

// Initialize valence counts based on existing springs
fn initializeValenceCounts() void {
    // Reset all current valence counts for alive particles
    resetValenceForAliveParticles();

    // Count existing springs
    countValenceFromAlivesprings();
}

// Mouse interaction functions - using new mouse system
export fn set_mouse_interaction(x: f32, y: f32, pressed: bool) void {
    mouse.updateMousePosition(x, y);
    mouse.setMousePressed(pressed);
}

export fn get_mouse_connected_particle() i32 {
    // Return first grabbed particle, or -1 if none
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
