const std = @import("std");
const math = std.math;

// Boids simulation parameters - adjust this for stress testing
const PARTICLE_COUNT = 2500; // Hue-based flocking with color affinity calculations
const MAX_SPEED = 0.2;
const MAX_FORCE = 0.3;
const SEPARATION_RADIUS = 0.016;
const ALIGNMENT_RADIUS = 0.15;
const COHESION_RADIUS = 0.25;
const WORLD_SIZE = 1.95; // Keep particles well within bounds regardless of aspect ratio

// Force strength multipliers
const SEPARATION_STRENGTH = 20.0;
const ALIGNMENT_STRENGTH = 1.5;
const COHESION_STRENGTH = 1.5;

// Spatial partitioning grid for O(n) performance
const GRID_SIZE = 25; // 25x25 grid
const GRID_CELL_SIZE = WORLD_SIZE / GRID_SIZE;
const MAX_PARTICLES_PER_CELL = 500;

// Pre-calculated constants for worldToGrid optimization
const GRID_SCALE = @as(f32, GRID_SIZE) / WORLD_SIZE;
const WORLD_HALF = WORLD_SIZE / 2.0;

// Particle structure for Boids
const Particle = struct {
    x: f32,
    y: f32,
    vx: f32,
    vy: f32,

    const Self = @This();

    pub fn init(x: f32, y: f32) Self {
        return Self{
            .x = x,
            .y = y,
            .vx = (x * 0.1), // Simple deterministic initial velocity
            .vy = (y * 0.1),
        };
    }

    pub fn updateWithSpatialGrid(self: *Self, particle_index: u32, dt: f32) void {
        // Single-pass optimized Boids calculation with hue-based affinity
        var sep_x: f32 = 0;
        var sep_y: f32 = 0;
        var sep_count: f32 = 0;

        var ali_x: f32 = 0;
        var ali_y: f32 = 0;
        var ali_count: f32 = 0;

        var coh_x: f32 = 0;
        var coh_y: f32 = 0;
        var coh_count: f32 = 0;

        // Pre-calculate squared radii to avoid sqrt comparisons
        const sep_radius_sq = SEPARATION_RADIUS * SEPARATION_RADIUS;
        const ali_radius_sq = ALIGNMENT_RADIUS * ALIGNMENT_RADIUS;
        const coh_radius_sq = COHESION_RADIUS * COHESION_RADIUS;

        // Get this particle's hue for color-based affinity
        const my_hue = getParticleHue(particle_index);

        // Check current cell and 8 surrounding cells in single pass
        const gx = worldToGrid(self.x);
        const gy = worldToGrid(self.y);

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
                        if (neighbor_index != particle_index) {
                            const other = &particles[neighbor_index];
                            const dist_x = self.x - other.x;
                            const dist_y = self.y - other.y;
                            const dist_sq = dist_x * dist_x + dist_y * dist_y;

                            // Calculate hue similarity for color-based affinity
                            const neighbor_hue = getParticleHue(neighbor_index);
                            const hue_similarity = getHueSimilarity(my_hue, neighbor_hue);

                            // Separation (closest interactions) - purely distance-based, no color influence
                            if (dist_sq > 0 and dist_sq < sep_radius_sq) {
                                const inv_dist = 1.0 / @sqrt(dist_sq);
                                sep_x += dist_x * inv_dist;
                                sep_y += dist_y * inv_dist;
                                sep_count += 1.0;
                            }

                            // Alignment (medium range) - strongly affected by color affinity
                            if (dist_sq > 0 and dist_sq < ali_radius_sq) {
                                const ali_strength = hue_similarity * hue_similarity * hue_similarity; // Cubic preference for tighter grouping
                                ali_x += other.vx * ali_strength;
                                ali_y += other.vy * ali_strength;
                                ali_count += ali_strength;
                            }

                            // Cohesion (largest range) - very strongly affected by color affinity
                            if (dist_sq > 0 and dist_sq < coh_radius_sq) {
                                const coh_strength = hue_similarity * hue_similarity * hue_similarity * hue_similarity; // Quartic preference for very tight color grouping
                                coh_x += other.x * coh_strength;
                                coh_y += other.y * coh_strength;
                                coh_count += coh_strength;
                            }
                        }
                    }
                }
            }
        }

        // Calculate final forces with color-weighted averages
        var force_x: f32 = 0;
        var force_y: f32 = 0;

        // Separation force
        if (sep_count > 0) {
            sep_x /= sep_count;
            sep_y /= sep_count;

            const sep_mag = @sqrt(sep_x * sep_x + sep_y * sep_y);
            if (sep_mag > 0) {
                force_x += (sep_x / sep_mag) * MAX_FORCE * SEPARATION_STRENGTH;
                force_y += (sep_y / sep_mag) * MAX_FORCE * SEPARATION_STRENGTH;
            }
        }

        // Alignment force
        if (ali_count > 0) {
            ali_x /= ali_count;
            ali_y /= ali_count;

            const ali_mag = @sqrt(ali_x * ali_x + ali_y * ali_y);
            if (ali_mag > 0) {
                force_x += (ali_x / ali_mag) * MAX_FORCE * ALIGNMENT_STRENGTH;
                force_y += (ali_y / ali_mag) * MAX_FORCE * ALIGNMENT_STRENGTH;
            }
        }

        // Cohesion force
        if (coh_count > 0) {
            coh_x /= coh_count;
            coh_y /= coh_count;

            coh_x -= self.x; // Seek towards center
            coh_y -= self.y;

            const coh_mag = @sqrt(coh_x * coh_x + coh_y * coh_y);
            if (coh_mag > 0) {
                force_x += (coh_x / coh_mag) * MAX_FORCE * COHESION_STRENGTH;
                force_y += (coh_y / coh_mag) * MAX_FORCE * COHESION_STRENGTH;
            }
        }

        // Apply forces
        self.vx += force_x * dt;
        self.vy += force_y * dt;

        // Limit speed
        const speed_sq = self.vx * self.vx + self.vy * self.vy;
        const max_speed_sq = MAX_SPEED * MAX_SPEED;
        if (speed_sq > max_speed_sq) {
            const speed = @sqrt(speed_sq);
            self.vx = (self.vx / speed) * MAX_SPEED;
            self.vy = (self.vy / speed) * MAX_SPEED;
        }

        // Update position
        self.x += self.vx * dt;
        self.y += self.vy * dt;

        // Bounce off edges with damping
        const border = WORLD_SIZE / 2.0;
        const bounce_force = 0.8;

        if (self.x > border) {
            self.x = border;
            self.vx = -@abs(self.vx) * bounce_force;
        }
        if (self.x < -border) {
            self.x = -border;
            self.vx = @abs(self.vx) * bounce_force;
        }
        if (self.y > border) {
            self.y = border;
            self.vy = -@abs(self.vy) * bounce_force;
        }
        if (self.y < -border) {
            self.y = -border;
            self.vy = @abs(self.vy) * bounce_force;
        }
    }

    fn separate(self: *const Self, neighbors: []const Particle) struct { x: f32, y: f32 } {
        var steer_x: f32 = 0;
        var steer_y: f32 = 0;
        var count: i32 = 0;

        for (neighbors) |other| {
            const dx = self.x - other.x;
            const dy = self.y - other.y;
            const dist = @sqrt(dx * dx + dy * dy);

            if (dist > 0 and dist < SEPARATION_RADIUS) {
                steer_x += dx / dist;
                steer_y += dy / dist;
                count += 1;
            }
        }

        if (count > 0) {
            steer_x /= @as(f32, @floatFromInt(count));
            steer_y /= @as(f32, @floatFromInt(count));

            // Normalize and scale
            const mag = @sqrt(steer_x * steer_x + steer_y * steer_y);
            if (mag > 0) {
                steer_x = (steer_x / mag) * MAX_FORCE;
                steer_y = (steer_y / mag) * MAX_FORCE;
            }
        }

        return .{ .x = steer_x, .y = steer_y };
    }

    fn alignment(self: *const Self, neighbors: []const Particle) struct { x: f32, y: f32 } {
        var sum_x: f32 = 0;
        var sum_y: f32 = 0;
        var count: i32 = 0;

        for (neighbors) |other| {
            const dx = self.x - other.x;
            const dy = self.y - other.y;
            const dist = @sqrt(dx * dx + dy * dy);

            if (dist > 0 and dist < ALIGNMENT_RADIUS) {
                sum_x += other.vx;
                sum_y += other.vy;
                count += 1;
            }
        }

        if (count > 0) {
            sum_x /= @as(f32, @floatFromInt(count));
            sum_y /= @as(f32, @floatFromInt(count));

            // Normalize and scale
            const mag = @sqrt(sum_x * sum_x + sum_y * sum_y);
            if (mag > 0) {
                sum_x = (sum_x / mag) * MAX_FORCE;
                sum_y = (sum_y / mag) * MAX_FORCE;
            }
        }

        return .{ .x = sum_x, .y = sum_y };
    }

    fn cohesion(self: *const Self, neighbors: []const Particle) struct { x: f32, y: f32 } {
        var sum_x: f32 = 0;
        var sum_y: f32 = 0;
        var count: i32 = 0;

        for (neighbors) |other| {
            const dx = self.x - other.x;
            const dy = self.y - other.y;
            const dist = @sqrt(dx * dx + dy * dy);

            if (dist > 0 and dist < COHESION_RADIUS) {
                sum_x += other.x;
                sum_y += other.y;
                count += 1;
            }
        }

        if (count > 0) {
            sum_x /= @as(f32, @floatFromInt(count));
            sum_y /= @as(f32, @floatFromInt(count));

            // Seek towards center
            var steer_x = sum_x - self.x;
            var steer_y = sum_y - self.y;

            // Normalize and scale
            const mag = @sqrt(steer_x * steer_x + steer_y * steer_y);
            if (mag > 0) {
                steer_x = (steer_x / mag) * MAX_FORCE;
                steer_y = (steer_y / mag) * MAX_FORCE;
            }

            return .{ .x = steer_x, .y = steer_y };
        }

        return .{ .x = 0, .y = 0 };
    }

    // Spatial grid versions of Boids functions for O(n) performance
    fn separateFromGrid(self: *const Self, particle_index: u32) struct { x: f32, y: f32 } {
        var steer_x: f32 = 0;
        var steer_y: f32 = 0;
        var count: i32 = 0;

        // Check current cell and 8 surrounding cells
        const gx = worldToGrid(self.x);
        const gy = worldToGrid(self.y);

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
                        if (neighbor_index != particle_index) {
                            const other = &particles[neighbor_index];
                            const dist_x = self.x - other.x;
                            const dist_y = self.y - other.y;
                            const dist = @sqrt(dist_x * dist_x + dist_y * dist_y);

                            if (dist > 0 and dist < SEPARATION_RADIUS) {
                                steer_x += dist_x / dist;
                                steer_y += dist_y / dist;
                                count += 1;
                            }
                        }
                    }
                }
            }
        }

        if (count > 0) {
            steer_x /= @as(f32, @floatFromInt(count));
            steer_y /= @as(f32, @floatFromInt(count));

            // Normalize and scale
            const mag = @sqrt(steer_x * steer_x + steer_y * steer_y);
            if (mag > 0) {
                steer_x = (steer_x / mag) * MAX_FORCE;
                steer_y = (steer_y / mag) * MAX_FORCE;
            }
        }

        return .{ .x = steer_x, .y = steer_y };
    }

    fn alignmentFromGrid(self: *const Self, particle_index: u32) struct { x: f32, y: f32 } {
        var sum_x: f32 = 0;
        var sum_y: f32 = 0;
        var count: i32 = 0;

        // Check current cell and 8 surrounding cells
        const gx = worldToGrid(self.x);
        const gy = worldToGrid(self.y);

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
                        if (neighbor_index != particle_index) {
                            const other = &particles[neighbor_index];
                            const dist_x = self.x - other.x;
                            const dist_y = self.y - other.y;
                            const dist = @sqrt(dist_x * dist_x + dist_y * dist_y);

                            if (dist > 0 and dist < ALIGNMENT_RADIUS) {
                                sum_x += other.vx;
                                sum_y += other.vy;
                                count += 1;
                            }
                        }
                    }
                }
            }
        }

        if (count > 0) {
            sum_x /= @as(f32, @floatFromInt(count));
            sum_y /= @as(f32, @floatFromInt(count));

            // Normalize and scale
            const mag = @sqrt(sum_x * sum_x + sum_y * sum_y);
            if (mag > 0) {
                sum_x = (sum_x / mag) * MAX_FORCE;
                sum_y = (sum_y / mag) * MAX_FORCE;
            }
        }

        return .{ .x = sum_x, .y = sum_y };
    }

    fn cohesionFromGrid(self: *const Self, particle_index: u32) struct { x: f32, y: f32 } {
        var sum_x: f32 = 0;
        var sum_y: f32 = 0;
        var count: i32 = 0;

        // Check current cell and 8 surrounding cells
        const gx = worldToGrid(self.x);
        const gy = worldToGrid(self.y);

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
                        if (neighbor_index != particle_index) {
                            const other = &particles[neighbor_index];
                            const dist_x = self.x - other.x;
                            const dist_y = self.y - other.y;
                            const dist = @sqrt(dist_x * dist_x + dist_y * dist_y);

                            if (dist > 0 and dist < COHESION_RADIUS) {
                                sum_x += other.x;
                                sum_y += other.y;
                                count += 1;
                            }
                        }
                    }
                }
            }
        }

        if (count > 0) {
            sum_x /= @as(f32, @floatFromInt(count));
            sum_y /= @as(f32, @floatFromInt(count));

            // Seek towards center
            var steer_x = sum_x - self.x;
            var steer_y = sum_y - self.y;

            // Normalize and scale
            const mag = @sqrt(steer_x * steer_x + steer_y * steer_y);
            if (mag > 0) {
                steer_x = (steer_x / mag) * MAX_FORCE;
                steer_y = (steer_y / mag) * MAX_FORCE;
            }

            return .{ .x = steer_x, .y = steer_y };
        }

        return .{ .x = 0, .y = 0 };
    }
};

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

export fn init() void {
    log("Initializing Boids particle system with spatial partitioning...", .{});
    device_handle = emscripten_webgpu_get_device();
    log("WebGPU device initialized: {}", .{device_handle});

    // Initialize spatial grid
    initializeGrid();

    // Initialize particles
    if (!particles_initialized) {
        for (&particles, 0..) |*particle, i| {
            // More random distribution using simple pseudo-random
            const seed = @as(f32, @floatFromInt(i));
            const range = WORLD_SIZE * 0.8; // Use 80% of world size for initial spread
            const x = ((@sin(seed * 12.9898) + 1.0) * 0.5 - 0.5) * range;
            const y = ((@sin(seed * 78.233) + 1.0) * 0.5 - 0.5) * range;

            particle.* = Particle.init(x, y);
        }
        particles_initialized = true;
        log("Initialized {} particles with {}x{} spatial grid", .{ PARTICLE_COUNT, GRID_SIZE, GRID_SIZE });
    }
}

export fn update_particles(dt: f32) void {
    if (!particles_initialized) return;

    // Populate spatial grid with current particle positions
    populateGrid();

    // Update each particle using spatial grid for O(n) performance
    for (0..PARTICLE_COUNT) |i| {
        var particle = &particles[i];
        particle.updateWithSpatialGrid(@intCast(i), dt);
    }
}

export fn get_particle_count() i32 {
    return PARTICLE_COUNT;
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

// Test functions for debugging
export fn add(a: i32, b: i32) i32 {
    return a + b;
}

export fn compute_demo(input: f32) f32 {
    return input * input;
}
