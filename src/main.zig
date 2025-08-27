const std = @import("std");
const math = std.math;

// Boids simulation parameters - adjust this for stress testing
const PARTICLE_COUNT = 5000; // Optimized with spatial partitioning for stunning visual performance
const MAX_SPEED = 0.8;
const MAX_FORCE = 0.05;
const SEPARATION_RADIUS = 0.12;
const ALIGNMENT_RADIUS = 0.25;
const COHESION_RADIUS = 0.35;
const WORLD_SIZE = 1.8;  // Keep particles well within bounds regardless of aspect ratio

// Spatial partitioning grid for O(n) performance
const GRID_SIZE = 16; // 16x16 grid
const GRID_CELL_SIZE = WORLD_SIZE / GRID_SIZE;
const MAX_PARTICLES_PER_CELL = 100;

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
        const sep = self.separateFromGrid(particle_index);
        const ali = self.alignmentFromGrid(particle_index);
        const coh = self.cohesionFromGrid(particle_index);
        
        // Apply weighted forces for better flocking behavior
        const sep_weight = 2.0; // Stronger separation
        const ali_weight = 1.0; // Moderate alignment
        const coh_weight = 1.5; // Strong cohesion
        
        self.vx += (sep.x * sep_weight + ali.x * ali_weight + coh.x * coh_weight) * dt;
        self.vy += (sep.y * sep_weight + ali.y * ali_weight + coh.y * coh_weight) * dt;
        
        // Limit speed
        const speed = @sqrt(self.vx * self.vx + self.vy * self.vy);
        if (speed > MAX_SPEED) {
            self.vx = (self.vx / speed) * MAX_SPEED;
            self.vy = (self.vy / speed) * MAX_SPEED;
        }
        
        // Update position
        self.x += self.vx * dt;
        self.y += self.vy * dt;
        
        // Bounce off edges with damping
        const border = WORLD_SIZE / 2.0;
        const bounce_force = 0.8; // Bounce damping factor
        
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

// Spatial grid helper functions
fn worldToGrid(world_pos: f32) i32 {
    // Convert world position (-WORLD_SIZE/2 to WORLD_SIZE/2) to grid coordinates (0 to GRID_SIZE-1)
    const normalized = (world_pos + WORLD_SIZE / 2.0) / WORLD_SIZE;
    const grid_pos = @as(i32, @intFromFloat(normalized * GRID_SIZE));
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