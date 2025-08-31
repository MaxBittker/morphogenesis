const std = @import("std");
const math = std.math;

// XPBD particle system: grids + free agents  
const GRID_COUNT = 5;
const GRID_PARTICLE_SIZE = 3;
const PARTICLES_PER_GRID = GRID_PARTICLE_SIZE * GRID_PARTICLE_SIZE;
const TOTAL_GRID_PARTICLES = GRID_COUNT * PARTICLES_PER_GRID;
const FREE_AGENT_COUNT = 500;
const PARTICLE_COUNT = TOTAL_GRID_PARTICLES + FREE_AGENT_COUNT;

// Particle physical properties
const PARTICLE_SIZE = 0.02; // Physical radius of particles
const PARTICLE_MASS = 1.0; // Normalized particle mass
const WORLD_SIZE = 1.95; // Keep particles well within bounds regardless of aspect ratio

// XPBD solver parameters
const XPBD_ITERATIONS = 5; // Constraint solver iterations per timestep
const XPBD_SUBSTEPS = 1; // Number of substeps per frame

// Constraint parameters (compliance = 1/(stiffness * dt²))
const DISTANCE_STIFFNESS = 1000.0;
const COLLISION_STIFFNESS = 100000.0; // Much stiffer for snappy collisions
const MOUSE_STIFFNESS = 50000.0;

// Constraint distances
const SPRING_REST_LENGTH = PARTICLE_SIZE * 1; // Natural spring length between connected particles
const SEPARATION_RADIUS = PARTICLE_SIZE * 1.5; // Minimum distance between particles
const GRID_SPACING = SPRING_REST_LENGTH * 1.1; // Base spacing between grid particles

// Boids constraint parameters
const BOIDS_SEPARATION_RADIUS = PARTICLE_SIZE * 3.0; // Separation distance for boids
const BOIDS_ALIGNMENT_RADIUS = PARTICLE_SIZE * 5.0; // Alignment radius for boids
const BOIDS_COHESION_RADIUS = PARTICLE_SIZE * 7.0; // Cohesion radius for boids
const BOIDS_SEPARATION_STIFFNESS = 5000.0;
const BOIDS_ALIGNMENT_STIFFNESS = 1000.0;
const BOIDS_COHESION_STIFFNESS = 800.0;

// Air resistance
const AIR_DAMPING = 0.99;
const GRAVITY = 9.8; // Gravity acceleration

// Legacy constants for compatibility
const SPRING_STRENGTH = DISTANCE_STIFFNESS;
const SEPARATION_STRENGTH = COLLISION_STIFFNESS;

// Spatial partitioning grid for O(n) performance
const GRID_SIZE = 25;
const GRID_CELL_SIZE = WORLD_SIZE / GRID_SIZE;
const MAX_PARTICLES_PER_CELL = 500;

// Pre-calculated constants for worldToGrid optimization
const GRID_SCALE = @as(f32, GRID_SIZE) / WORLD_SIZE;
const WORLD_HALF = WORLD_SIZE / 2.0;

// XPBD Constraint Types
const ConstraintType = enum {
    distance,
    collision,
    mouse_spring,
};

// XPBD Distance Constraint (replaces springs)
const DistanceConstraint = struct {
    particle_a: u32,
    particle_b: u32,
    rest_length: f32,
    compliance: f32, // α = 1/(k * dt²)
    lagrange_multiplier: f32, // λ accumulated during solving

    const Self = @This();

    pub fn init(a: u32, b: u32, length: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .rest_length = length,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }

    pub fn evaluate(self: *const Self, positions: []Vec2) f32 {
        const pa = positions[self.particle_a];
        const pb = positions[self.particle_b];
        const dx = pb.x - pa.x;
        const dy = pb.y - pa.y;
        const distance = @sqrt(dx * dx + dy * dy);
        return distance - self.rest_length;
    }

    pub fn getGradients(self: *const Self, positions: []Vec2) struct { ga: Vec2, gb: Vec2 } {
        const pa = positions[self.particle_a];
        const pb = positions[self.particle_b];
        const dx = pb.x - pa.x;
        const dy = pb.y - pa.y;
        const distance = @sqrt(dx * dx + dy * dy);
        
        if (distance < 1e-6) {
            return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 } };
        }
        
        const norm_x = dx / distance;
        const norm_y = dy / distance;
        
        return .{
            .ga = Vec2{ .x = -norm_x, .y = -norm_y },
            .gb = Vec2{ .x = norm_x, .y = norm_y },
        };
    }
};

// XPBD Collision Constraint (replaces separation forces)
const CollisionConstraint = struct {
    particle_a: u32,
    particle_b: u32,
    min_distance: f32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: u32, b: u32, min_dist: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .min_distance = min_dist,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }

    pub fn evaluate(self: *const Self, positions: []Vec2) f32 {
        const pa = positions[self.particle_a];
        const pb = positions[self.particle_b];
        const dx = pb.x - pa.x;
        const dy = pb.y - pa.y;
        const distance = @sqrt(dx * dx + dy * dy);
        return @min(0.0, distance - self.min_distance); // Only active when overlapping
    }

    pub fn getGradients(self: *const Self, positions: []Vec2) struct { ga: Vec2, gb: Vec2 } {
        const pa = positions[self.particle_a];
        const pb = positions[self.particle_b];
        const dx = pb.x - pa.x;
        const dy = pb.y - pa.y;
        const distance = @sqrt(dx * dx + dy * dy);
        
        if (distance < 1e-6) {
            return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 } };
        }
        
        const norm_x = dx / distance;
        const norm_y = dy / distance;
        
        return .{
            .ga = Vec2{ .x = -norm_x, .y = -norm_y },
            .gb = Vec2{ .x = norm_x, .y = norm_y },
        };
    }
};

// XPBD Boids Constraints (replaces force-based boids)
const SeparationConstraint = struct {
    particle_a: u32,
    particle_b: u32,
    target_distance: f32, // Desired separation distance
    compliance: f32,
    lagrange_multiplier: f32,
    
    const Self = @This();
    
    pub fn init(a: u32, b: u32, target_dist: f32, stiffness: f32, dt: f32) Self {
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
    particle_a: u32,
    neighbor_particles: [16]u32, // Max neighbors for alignment
    neighbor_count: u32,
    compliance: f32,
    lagrange_multiplier: f32,
    
    const Self = @This();
    
    pub fn init(a: u32, stiffness: f32, dt: f32) Self {
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
    particle_a: u32,
    target_x: f32, // Center of mass X
    target_y: f32, // Center of mass Y
    compliance: f32,
    lagrange_multiplier: f32,
    
    const Self = @This();
    
    pub fn init(a: u32, target_x: f32, target_y: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .target_x = target_x,
            .target_y = target_y,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }
};

// Vector2 helper
const Vec2 = struct {
    x: f32,
    y: f32,
};

// Particle type enumeration
const ParticleType = enum {
    grid_particle, // Part of a spring-connected grid
    free_agent, // Independent Boid
};

// XPBD Particle structure  
const Particle = struct {
    x: f32,
    y: f32,
    predicted_x: f32, // Position prediction for XPBD
    predicted_y: f32,
    vx: f32,
    vy: f32,
    mass: f32,
    particle_type: ParticleType,
    grid_id: u8, // Which grid this particle belongs to (0-4, or 255 for free agents)

    const Self = @This();

    pub fn init(x: f32, y: f32, ptype: ParticleType, grid: u8) Self {
        return Self{
            .x = x,
            .y = y,
            .predicted_x = x,
            .predicted_y = y,
            .vx = (x * 0.1), // Simple deterministic initial velocity
            .vy = (y * 0.1),
            .mass = PARTICLE_MASS,
            .particle_type = ptype,
            .grid_id = grid,
        };
    }

    pub fn predictPosition(self: *Self, dt: f32) void {
        // Apply air damping to velocity
        self.vx *= AIR_DAMPING;
        self.vy *= AIR_DAMPING;
        
        // Predict position using current velocity (explicit Euler prediction)
        self.predicted_x = self.x + self.vx * dt;
        self.predicted_y = self.y + self.vy * dt;
        
        // Handle boundary collision in prediction
        const border = WORLD_SIZE / 2.0;
        if (self.predicted_x > border) {
            self.predicted_x = border;
            self.vx *= -0.8; // Bounce damping
        }
        if (self.predicted_x < -border) {
            self.predicted_x = -border;
            self.vx *= -0.8;
        }
        if (self.predicted_y > border) {
            self.predicted_y = border;
            self.vy *= -0.8;
        }
        if (self.predicted_y < -border) {
            self.predicted_y = -border;
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
        // Simple boundary constraints for XPBD
        const border = WORLD_SIZE / 2.0;
        
        if (self.predicted_x > border) {
            self.predicted_x = border;
        }
        if (self.predicted_x < -border) {
            self.predicted_x = -border;
        }
        if (self.predicted_y > border) {
            self.predicted_y = border;
        }
        if (self.predicted_y < -border) {
            self.predicted_y = -border;
        }
    }
};

// XPBD constraint arrays
const MAX_DISTANCE_CONSTRAINTS = GRID_COUNT * (GRID_PARTICLE_SIZE - 1) * GRID_PARTICLE_SIZE * 2;
const MAX_COLLISION_CONSTRAINTS = 10000; // Dynamic collision detection

var distance_constraints: [MAX_DISTANCE_CONSTRAINTS]DistanceConstraint = undefined;
var distance_constraint_count: u32 = 0;

var collision_constraints: [MAX_COLLISION_CONSTRAINTS]CollisionConstraint = undefined;
var collision_constraint_count: u32 = 0;

// Mouse constraint
var mouse_constraint: ?DistanceConstraint = null;

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

// Spatial grid cell structure
const GridCell = struct {
    particles: [MAX_PARTICLES_PER_CELL]u32,
    count: u32,

    const Self = @This();

    pub fn init() Self {
        return Self{
            .particles = undefined,
            .count = 0,
        };
    }

    pub fn clear(self: *Self) void {
        self.count = 0;
    }

    pub fn add(self: *Self, particle_index: u32) void {
        if (self.count < MAX_PARTICLES_PER_CELL) {
            self.particles[self.count] = particle_index;
            self.count += 1;
        }
    }
};

// Spatial grid for fast neighbor queries
var spatial_grid: [GRID_SIZE][GRID_SIZE]GridCell = undefined;
var grid_initialized = false;

// Global particle system
var particles: [PARTICLE_COUNT]Particle = undefined;
var particles_initialized = false;

// Spring system - maximum springs for 5 16x16 grids with 4-connectivity
const MAX_SPRINGS = GRID_COUNT * (GRID_PARTICLE_SIZE - 1) * GRID_PARTICLE_SIZE * 2; // Horizontal + vertical springs

// Legacy Spring structure for backward compatibility
const Spring = struct {
    particle_a: u32,
    particle_b: u32,
    rest_length: f32,
    
    pub fn init(particle_a: u32, particle_b: u32, rest_length: f32) Spring {
        return Spring{
            .particle_a = particle_a,
            .particle_b = particle_b,
            .rest_length = rest_length,
        };
    }
};

var springs: [MAX_SPRINGS]Spring = undefined;
var spring_count: u32 = 0;
var springs_initialized = false;

// Connection lookup optimization: store spring indices for each particle
const MAX_CONNECTIONS_PER_PARTICLE = 4; // Maximum 4 springs per grid particle (up, down, left, right)
var particle_connections: [PARTICLE_COUNT][MAX_CONNECTIONS_PER_PARTICLE]u32 = undefined;
var particle_connection_counts: [PARTICLE_COUNT]u8 = undefined;

// Mouse interaction state
var mouse_x: f32 = 0.0;
var mouse_y: f32 = 0.0;
var mouse_pressed: bool = false;
var mouse_connected_particle: u32 = 0;
var mouse_has_connection: bool = false;

// Mock WebGPU bindings (not actually used for rendering)
extern fn emscripten_webgpu_get_device() u32;

// Console logging
extern fn console_log(ptr: [*]const u8, len: usize) void;

fn log(comptime fmt: []const u8, args: anytype) void {
    var buffer: [1024]u8 = undefined;
    const message = std.fmt.bufPrint(buffer[0..], fmt, args) catch "Log message too long";
    console_log(message.ptr, message.len);
}

// Color-based flocking helper functions
fn getParticleHue(particle_index: u32) f32 {
    // Same hue calculation as in the vertex shader
    return @as(f32, @floatFromInt(particle_index)) * 0.01745329; // 1 degree per particle in radians
}

fn getHueSimilarity(hue1: f32, hue2: f32) f32 {
    // Calculate the shortest angular distance between two hues (0 to 2π)
    const tau = 6.28318530718; // 2π
    var diff = @abs(hue1 - hue2);

    // Handle wrapping around the color wheel
    if (diff > tau * 0.5) {
        diff = tau - diff;
    }

    // Convert to similarity (0 = opposite colors, 1 = identical colors)
    // Use cosine for smooth falloff: cos(0) = 1, cos(π) = -1
    const similarity = (@cos(diff) + 1.0) * 0.5; // Normalize to 0-1 range
    return similarity;
}

// Spatial grid helper functions
inline fn worldToGrid(world_pos: f32) i32 {
    // Optimized: single multiply instead of add + divide + multiply
    const grid_pos = @as(i32, @intFromFloat((world_pos + WORLD_HALF) * GRID_SCALE));
    return @max(0, @min(GRID_SIZE - 1, grid_pos));
}

fn getGridCell(x: f32, y: f32) *GridCell {
    const gx = worldToGrid(x);
    const gy = worldToGrid(y);
    return &spatial_grid[@intCast(gx)][@intCast(gy)];
}

fn initializeGrid() void {
    if (!grid_initialized) {
        for (0..GRID_SIZE) |i| {
            for (0..GRID_SIZE) |j| {
                spatial_grid[i][j] = GridCell.init();
            }
        }
        grid_initialized = true;
    }
}

fn clearGrid() void {
    for (0..GRID_SIZE) |i| {
        for (0..GRID_SIZE) |j| {
            spatial_grid[i][j].clear();
        }
    }
}

fn populateGrid() void {
    clearGrid();
    for (0..PARTICLE_COUNT) |i| {
        const particle = &particles[i];
        const cell = getGridCell(particle.x, particle.y);
        cell.add(@intCast(i));
    }
}

var device_handle: u32 = 0;

// Initialize grid particles in 5 separate 16x16 grids
fn initializeGridParticles() void {
    // Grid positions in world space (spread them out)
    const grid_positions = [_][2]f32{
        .{ -0.6, -0.6 }, // Top-left
        .{ 0.6, -0.6 }, // Top-right
        .{ 0.0, 0.0 }, // Center
        .{ -0.6, 0.6 }, // Bottom-left
        .{ 0.6, 0.6 }, // Bottom-right
    };

    var particle_index: u32 = 0;

    for (0..GRID_COUNT) |grid_id| {
        const base_x = grid_positions[grid_id][0];
        const base_y = grid_positions[grid_id][1];

        // Create 16x16 grid of particles
        for (0..GRID_PARTICLE_SIZE) |row| {
            for (0..GRID_PARTICLE_SIZE) |col| {
                const x = base_x + (@as(f32, @floatFromInt(col)) - 7.5) * GRID_SPACING;
                const y = base_y + (@as(f32, @floatFromInt(row)) - 7.5) * GRID_SPACING;

                particles[particle_index] = Particle.init(x, y, ParticleType.grid_particle, @intCast(grid_id));
                particle_index += 1;
            }
        }
    }
}

// Initialize free agent Boids with random positions
fn initializeFreeAgents() void {
    const start_index = TOTAL_GRID_PARTICLES;

    for (start_index..PARTICLE_COUNT) |i| {
        const seed = @as(f32, @floatFromInt(i));
        const range = WORLD_SIZE * 0.8;
        const x = ((@sin(seed * 12.9898) + 1.0) * 0.5 - 0.5) * range;
        const y = ((@sin(seed * 78.233) + 1.0) * 0.5 - 0.5) * range;

        particles[i] = Particle.init(x, y, ParticleType.free_agent, 255);
    }
}

// Create spring connections within each 16x16 grid
fn initializeSprings() void {
    spring_count = 0;

    // Initialize connection lookup table
    for (0..PARTICLE_COUNT) |i| {
        particle_connection_counts[i] = 0;
    }

    for (0..GRID_COUNT) |grid_id| {
        const grid_start = grid_id * PARTICLES_PER_GRID;

        // Create horizontal springs (connect adjacent particles in same row)
        for (0..GRID_PARTICLE_SIZE) |row| {
            for (0..GRID_PARTICLE_SIZE - 1) |col| {
                const particle_a = @as(u32, @intCast(grid_start + row * GRID_PARTICLE_SIZE + col));
                const particle_b = @as(u32, @intCast(grid_start + row * GRID_PARTICLE_SIZE + col + 1));

                springs[spring_count] = Spring.init(particle_a, particle_b, SPRING_REST_LENGTH);

                // Update connection lookup for both particles
                addParticleConnection(particle_a, spring_count);
                addParticleConnection(particle_b, spring_count);

                spring_count += 1;
            }
        }

        // Create vertical springs (connect adjacent particles in same column)
        for (0..GRID_PARTICLE_SIZE - 1) |row| {
            for (0..GRID_PARTICLE_SIZE) |col| {
                const particle_a = @as(u32, @intCast(grid_start + row * GRID_PARTICLE_SIZE + col));
                const particle_b = @as(u32, @intCast(grid_start + (row + 1) * GRID_PARTICLE_SIZE + col));

                springs[spring_count] = Spring.init(particle_a, particle_b, SPRING_REST_LENGTH);

                // Update connection lookup for both particles
                addParticleConnection(particle_a, spring_count);
                addParticleConnection(particle_b, spring_count);

                spring_count += 1;
            }
        }
    }
}

// Helper function to add a spring connection to a particle's lookup table
fn addParticleConnection(particle_index: u32, spring_index: u32) void {
    const count = particle_connection_counts[particle_index];
    if (count < MAX_CONNECTIONS_PER_PARTICLE) {
        particle_connections[particle_index][count] = spring_index;
        particle_connection_counts[particle_index] += 1;
    }
}

// Find the closest particle to a given position
fn findClosestParticle(target_x: f32, target_y: f32) u32 {
    var closest_index: u32 = 0;
    var closest_distance_sq: f32 = std.math.inf(f32);

    for (0..PARTICLE_COUNT) |i| {
        const particle = &particles[i];
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

// XPBD constraint generation - creates distance, collision, and boids constraints
fn generateConstraints(dt: f32) void {
    distance_constraint_count = 0;
    collision_constraint_count = 0;
    boids_separation_constraint_count = 0;
    boids_alignment_constraint_count = 0;
    boids_cohesion_constraint_count = 0;
    
    // Generate distance constraints from springs
    for (0..spring_count) |i| {
        const spring = &springs[i];
        if (distance_constraint_count < MAX_DISTANCE_CONSTRAINTS) {
            // Calculate compliance: α = 1/(k * dt²)
            const spring_compliance = 1.0 / (SPRING_STRENGTH * dt * dt);
            
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
    
    // Generate mouse constraint if active
    if (mouse_has_connection) {
        mouse_constraint = DistanceConstraint{
            .particle_a = mouse_connected_particle,
            .particle_b = 0, // Dummy particle index (mouse is external)
            .rest_length = 0.0, // Pull directly to mouse
            .compliance = 1.0 / (SPRING_STRENGTH * 500.0 * dt * dt), // Very strong
            .lagrange_multiplier = 0.0,
        };
    } else {
        mouse_constraint = null;
    }
    
    // Generate collision constraints using spatial grid
    populateGrid();
    for (0..PARTICLE_COUNT) |i| {
        generateCollisionConstraintsForParticle(@intCast(i), dt);
    }
    
    // Generate boids constraints for free agents only
    for (TOTAL_GRID_PARTICLES..PARTICLE_COUNT) |i| {
        generateBoidsConstraintsForParticle(@intCast(i), dt);
    }
}

// Generate collision constraints for a single particle using spatial grid
fn generateCollisionConstraintsForParticle(particle_index: u32, dt: f32) void {
    const particle = &particles[particle_index];
    
    // Check current cell and 8 surrounding cells
    const gx = worldToGrid(particle.predicted_x);
    const gy = worldToGrid(particle.predicted_y);

    var dy: i32 = -1;
    while (dy <= 1) : (dy += 1) {
        var dx: i32 = -1;
        while (dx <= 1) : (dx += 1) {
            const check_x = gx + dx;
            const check_y = gy + dy;

            if (check_x >= 0 and check_x < GRID_SIZE and check_y >= 0 and check_y < GRID_SIZE) {
                const cell = &spatial_grid[@intCast(check_x)][@intCast(check_y)];

                for (0..cell.count) |i| {
                    const neighbor_index = cell.particles[i];
                    if (neighbor_index != particle_index and neighbor_index < particle_index) { // Avoid duplicates
                        const other = &particles[neighbor_index];
                        const dx_pred = particle.predicted_x - other.predicted_x;
                        const dy_pred = particle.predicted_y - other.predicted_y;
                        const dist_sq = dx_pred * dx_pred + dy_pred * dy_pred;
                        
                        const min_distance = PARTICLE_SIZE * 2.2; // Slightly larger separation for better collision detection
                        if (dist_sq < min_distance * min_distance) {
                            if (collision_constraint_count < MAX_COLLISION_CONSTRAINTS) {
                                // High stiffness for collision constraints
                                const collision_compliance = 1.0 / (SEPARATION_STRENGTH * dt * dt);
                                
                                collision_constraints[collision_constraint_count] = CollisionConstraint{
                                    .particle_a = particle_index,
                                    .particle_b = neighbor_index,
                                    .min_distance = min_distance,
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

// Generate boids constraints for a single free agent particle
fn generateBoidsConstraintsForParticle(particle_index: u32, dt: f32) void {
    const particle = &particles[particle_index];
    
    // Only apply to free agents
    if (particle.particle_type != ParticleType.free_agent) return;
    
    var neighbors: [16]u32 = undefined;
    var neighbor_count: u32 = 0;
    var neighbor_positions_x: f32 = 0;
    var neighbor_positions_y: f32 = 0;
    var neighbor_velocities_x: f32 = 0;
    var neighbor_velocities_y: f32 = 0;
    
    // Check current cell and surrounding cells for neighbors
    const gx = worldToGrid(particle.predicted_x);
    const gy = worldToGrid(particle.predicted_y);

    var dy: i32 = -1;
    while (dy <= 1) : (dy += 1) {
        var dx: i32 = -1;
        while (dx <= 1) : (dx += 1) {
            const check_x = gx + dx;
            const check_y = gy + dy;

            if (check_x >= 0 and check_x < GRID_SIZE and check_y >= 0 and check_y < GRID_SIZE) {
                const cell = &spatial_grid[@intCast(check_x)][@intCast(check_y)];

                for (0..cell.count) |i| {
                    const neighbor_index = cell.particles[i];
                    if (neighbor_index != particle_index and neighbor_index >= TOTAL_GRID_PARTICLES) { // Only other free agents
                        const other = &particles[neighbor_index];
                        const dx_pred = particle.predicted_x - other.predicted_x;
                        const dy_pred = particle.predicted_y - other.predicted_y;
                        const dist_sq = dx_pred * dx_pred + dy_pred * dy_pred;
                        
                        // Check if within boids interaction range (use largest radius)
                        const max_radius_sq = BOIDS_COHESION_RADIUS * BOIDS_COHESION_RADIUS;
                        if (dist_sq < max_radius_sq and neighbor_count < 16) {
                            neighbors[neighbor_count] = neighbor_index;
                            neighbor_count += 1;
                            
                            // Accumulate for center of mass and average velocity
                            neighbor_positions_x += other.predicted_x;
                            neighbor_positions_y += other.predicted_y;
                            neighbor_velocities_x += other.vx;
                            neighbor_velocities_y += other.vy;
                            
                            // Generate separation constraints for close neighbors
                            const sep_radius_sq = BOIDS_SEPARATION_RADIUS * BOIDS_SEPARATION_RADIUS;
                            if (dist_sq < sep_radius_sq and boids_separation_constraint_count < MAX_BOIDS_SEPARATION_CONSTRAINTS) {
                                const separation_compliance = 1.0 / (BOIDS_SEPARATION_STIFFNESS * dt * dt);
                                
                                boids_separation_constraints[boids_separation_constraint_count] = SeparationConstraint{
                                    .particle_a = particle_index,
                                    .particle_b = neighbor_index,
                                    .target_distance = BOIDS_SEPARATION_RADIUS,
                                    .compliance = separation_compliance,
                                    .lagrange_multiplier = 0.0,
                                };
                                boids_separation_constraint_count += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // Generate alignment constraint if we have neighbors
    if (neighbor_count > 0 and boids_alignment_constraint_count < MAX_BOIDS_ALIGNMENT_CONSTRAINTS) {
        boids_alignment_constraints[boids_alignment_constraint_count] = AlignmentConstraint.init(
            particle_index, 
            BOIDS_ALIGNMENT_STIFFNESS, 
            dt
        );
        
        // Store neighbor information
        var constraint = &boids_alignment_constraints[boids_alignment_constraint_count];
        constraint.neighbor_count = @min(neighbor_count, 16);
        for (0..constraint.neighbor_count) |i| {
            constraint.neighbor_particles[i] = neighbors[i];
        }
        boids_alignment_constraint_count += 1;
    }
    
    // Generate cohesion constraint if we have neighbors
    if (neighbor_count > 0 and boids_cohesion_constraint_count < MAX_BOIDS_COHESION_CONSTRAINTS) {
        const center_x = neighbor_positions_x / @as(f32, @floatFromInt(neighbor_count));
        const center_y = neighbor_positions_y / @as(f32, @floatFromInt(neighbor_count));
        
        boids_cohesion_constraints[boids_cohesion_constraint_count] = CohesionConstraint.init(
            particle_index,
            center_x,
            center_y,
            BOIDS_COHESION_STIFFNESS,
            dt
        );
        boids_cohesion_constraint_count += 1;
    }
}

// XPBD constraint solver - solves all constraints using Lagrange multipliers
fn solveConstraints(dt: f32) void {
    // Solve distance constraints
    for (0..distance_constraint_count) |i| {
        solveDistanceConstraint(&distance_constraints[i], dt);
    }
    
    // Solve mouse constraint if active
    if (mouse_constraint) |*constraint| {
        solveMouseConstraint(constraint, dt);
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

// Solve individual distance constraint using XPBD
fn solveDistanceConstraint(constraint: *DistanceConstraint, dt: f32) void {
    const particle_a = &particles[constraint.particle_a];
    const particle_b = &particles[constraint.particle_b];
    
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
    particles[constraint.particle_a].predicted_x += correction_a_x;
    particles[constraint.particle_a].predicted_y += correction_a_y;
    particles[constraint.particle_b].predicted_x += correction_b_x;
    particles[constraint.particle_b].predicted_y += correction_b_y;
}

// Solve mouse constraint (particle to mouse position)
fn solveMouseConstraint(constraint: *DistanceConstraint, dt: f32) void {
    const particle = &particles[constraint.particle_a];
    
    // Calculate constraint value C(x) = |x_particle - x_mouse| - rest_length
    const dx = particle.predicted_x - mouse_x;
    const dy = particle.predicted_y - mouse_y;
    const current_distance = @sqrt(dx * dx + dy * dy);
    
    if (current_distance < 0.001) return; // Avoid division by zero
    
    const constraint_value = current_distance - constraint.rest_length;
    
    // Calculate constraint gradient ∇C
    const grad_x = dx / current_distance;
    const grad_y = dy / current_distance;
    
    // Mouse has infinite mass, so only particle moves
    const w_particle = 1.0 / particle.mass;
    const grad_length_sq = grad_x * grad_x + grad_y * grad_y;
    const denominator = w_particle * grad_length_sq + constraint.compliance / (dt * dt);
    
    if (denominator < 0.001) return; // Avoid division by zero
    
    // Calculate delta Lagrange multiplier
    const delta_lambda = -(constraint_value + constraint.compliance * constraint.lagrange_multiplier / (dt * dt)) / denominator;
    
    // Update Lagrange multiplier
    constraint.lagrange_multiplier += delta_lambda;
    
    // Apply position correction only to particle
    const correction_x = w_particle * delta_lambda * grad_x;
    const correction_y = w_particle * delta_lambda * grad_y;
    
    // Update predicted position
    particles[constraint.particle_a].predicted_x += correction_x;
    particles[constraint.particle_a].predicted_y += correction_y;
}

// Solve individual collision constraint using XPBD
fn solveCollisionConstraint(constraint: *CollisionConstraint, dt: f32) void {
    const particle_a = &particles[constraint.particle_a];
    const particle_b = &particles[constraint.particle_b];
    
    // Calculate constraint value C(x) = min_distance - |x_a - x_b|
    const dx = particle_a.predicted_x - particle_b.predicted_x;
    const dy = particle_a.predicted_y - particle_b.predicted_y;
    const current_distance = @sqrt(dx * dx + dy * dy);
    
    if (current_distance < 0.001) return; // Avoid division by zero
    
    const constraint_value = constraint.min_distance - current_distance;
    
    // Only solve if constraint is violated (particles too close)
    if (constraint_value <= 0) return;
    
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
    particles[constraint.particle_a].predicted_x += correction_a_x;
    particles[constraint.particle_a].predicted_y += correction_a_y;
    particles[constraint.particle_b].predicted_x += correction_b_x;
    particles[constraint.particle_b].predicted_y += correction_b_y;
}

// Solve boids separation constraint (keep particles apart)
fn solveSeparationConstraint(constraint: *SeparationConstraint, dt: f32) void {
    const particle_a = &particles[constraint.particle_a];
    const particle_b = &particles[constraint.particle_b];
    
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
    particles[constraint.particle_a].predicted_x += correction_a_x;
    particles[constraint.particle_a].predicted_y += correction_a_y;
    particles[constraint.particle_b].predicted_x += correction_b_x;
    particles[constraint.particle_b].predicted_y += correction_b_y;
}

// Solve alignment constraint (align velocities with neighbors)
fn solveAlignmentConstraint(constraint: *AlignmentConstraint, dt: f32) void {
    const particle = &particles[constraint.particle_a];
    
    if (constraint.neighbor_count == 0) return;
    
    // Calculate average neighbor velocity
    var avg_vx: f32 = 0;
    var avg_vy: f32 = 0;
    for (0..constraint.neighbor_count) |i| {
        const neighbor = &particles[constraint.neighbor_particles[i]];
        avg_vx += neighbor.vx;
        avg_vy += neighbor.vy;
    }
    avg_vx /= @as(f32, @floatFromInt(constraint.neighbor_count));
    avg_vy /= @as(f32, @floatFromInt(constraint.neighbor_count));
    
    // Apply velocity alignment as position correction (XPBD style)
    const velocity_diff_x = avg_vx - particle.vx;
    const velocity_diff_y = avg_vy - particle.vy;
    
    // Convert velocity difference to position correction
    const alignment_strength = 1.0 / (constraint.compliance + dt * dt);
    const correction_x = velocity_diff_x * dt * alignment_strength * 0.01; // Small factor for stability
    const correction_y = velocity_diff_y * dt * alignment_strength * 0.01;
    
    // Apply position correction
    particles[constraint.particle_a].predicted_x += correction_x;
    particles[constraint.particle_a].predicted_y += correction_y;
}

// Solve cohesion constraint (attract to center of mass)
fn solveCohesionConstraint(constraint: *CohesionConstraint, dt: f32) void {
    const particle = &particles[constraint.particle_a];
    
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
    
    const correction_x = -grad_x * constraint_value * cohesion_strength * 0.01; // Small factor for stability
    const correction_y = -grad_y * constraint_value * cohesion_strength * 0.01;
    
    // Apply position correction
    particles[constraint.particle_a].predicted_x += correction_x;
    particles[constraint.particle_a].predicted_y += correction_y;
}

export fn init() void {
    log("Initializing enhanced particle system: {} grids + {} free agents...", .{ GRID_COUNT, FREE_AGENT_COUNT });
    device_handle = emscripten_webgpu_get_device();
    log("WebGPU device initialized: {}", .{device_handle});

    // Initialize spatial grid
    initializeGrid();

    // Initialize particles
    if (!particles_initialized) {
        initializeGridParticles();
        initializeFreeAgents();
        particles_initialized = true;
        log("Initialized {} total particles: {} grid + {} free agents", .{ PARTICLE_COUNT, TOTAL_GRID_PARTICLES, FREE_AGENT_COUNT });
    }

    // Initialize spring connections
    if (!springs_initialized) {
        initializeSprings();
        springs_initialized = true;
        log("Created {} spring connections for {} grids", .{ spring_count, GRID_COUNT });
    }
}

export fn reset() void {
    log("Resetting particle system...", .{});

    // Reset mouse interaction
    mouse_pressed = false;
    mouse_has_connection = false;

    // Reinitialize all particles to starting positions
    initializeGridParticles();
    initializeFreeAgents();

    log("Particle system reset complete", .{});
}

export fn update_particles(dt: f32) void {
    if (!particles_initialized) return;

    // XPBD Integration Steps:
    // 1. Position prediction (using velocity)
    // 2. Constraint generation and solving
    // 3. Velocity update from position changes
    // 4. Position finalization

    // Step 1: Predict positions for all particles
    for (0..PARTICLE_COUNT) |i| {
        var particle = &particles[i];
        particle.predictPosition(dt);
    }

    // Step 2: Generate constraints and solve them iteratively
    generateConstraints(dt);
    
    // Solve constraints for multiple iterations for stability
    const solver_iterations = 5; // More iterations for better stability
    for (0..solver_iterations) |_| {
        solveConstraints(dt);
    }
    
    // Extra collision-only passes for snappy collision response
    const collision_passes = 3;
    for (0..collision_passes) |_| {
        solveCollisionConstraintsOnly(dt);
    }
    
    // Step 3: Update velocities and positions from predictions
    for (0..PARTICLE_COUNT) |i| {
        var particle = &particles[i];
        particle.updateFromPrediction(dt);
    }
}

export fn get_particle_count() i32 {
    return PARTICLE_COUNT;
}

// Export important constants to avoid duplication in JavaScript
export fn get_world_size() f32 {
    return WORLD_SIZE;
}

export fn get_grid_size() i32 {
    return GRID_SIZE;
}

export fn get_max_particles() i32 {
    return PARTICLE_COUNT + 1000; // Buffer for safety
}

export fn get_max_springs() i32 {
    return MAX_SPRINGS;
}

export fn get_particle_size() f32 {
    return PARTICLE_SIZE;
}

export fn get_spring_count() i32 {
    return @intCast(spring_count);
}

export fn get_spring_particle_a(spring_index: i32) i32 {
    if (spring_index < 0 or spring_index >= spring_count) {
        return 0;
    }
    return @intCast(springs[@intCast(spring_index)].particle_a);
}

export fn get_spring_particle_b(spring_index: i32) i32 {
    if (spring_index < 0 or spring_index >= spring_count) {
        return 0;
    }
    return @intCast(springs[@intCast(spring_index)].particle_b);
}

export fn get_particle_data(index: i32) f32 {
    if (!particles_initialized or index < 0 or index >= PARTICLE_COUNT * 2) {
        return 0.0;
    }

    const particle_index = @divFloor(@as(usize, @intCast(index)), 2);
    const coord_index = @mod(@as(usize, @intCast(index)), 2);

    if (coord_index == 0) {
        return particles[particle_index].x;
    } else {
        return particles[particle_index].y;
    }
}

// Mouse interaction functions
export fn set_mouse_interaction(x: f32, y: f32, pressed: bool) void {
    mouse_x = x;
    mouse_y = y;

    if (pressed and !mouse_pressed) {
        // Mouse just pressed - find closest particle and create connection
        mouse_pressed = true;
        mouse_connected_particle = findClosestParticle(x, y);
        mouse_has_connection = true;
    } else if (!pressed and mouse_pressed) {
        // Mouse just released - remove connection
        mouse_pressed = false;
        mouse_has_connection = false;
    } else if (pressed and mouse_pressed) {
        // Mouse still pressed - update position only
        mouse_pressed = true;
    }
}

export fn get_mouse_connected_particle() i32 {
    if (mouse_has_connection) {
        return @intCast(mouse_connected_particle);
    }
    return -1;
}

export fn get_mouse_position_x() f32 {
    return mouse_x;
}

export fn get_mouse_position_y() f32 {
    return mouse_y;
}

// Test functions for debugging
export fn add(a: i32, b: i32) i32 {
    return a + b;
}

export fn compute_demo(input: f32) f32 {
    return input * input;
}
