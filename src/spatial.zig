const std = @import("std");

// Spatial partitioning grid for O(n) performance
pub const GRID_SIZE = 25;
pub const MAX_PARTICLES_PER_CELL = 500;

// Pre-calculated constants for worldToGrid optimization  
const main_module = @import("main.zig");
const BASE_WORLD_SIZE = main_module.WORLD_SIZE;

// Spatial grid cell structure
pub const GridCell = struct {
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
pub var spatial_grid: [GRID_SIZE][GRID_SIZE]GridCell = undefined;
var grid_initialized = false;

// Spatial grid helper functions with aspect-aware scaling
pub inline fn worldToGridX(world_x: f32) i32 {
    const world_width = main_module.get_world_width();
    const grid_scale_x = @as(f32, GRID_SIZE) / world_width;
    const world_half_x = world_width / 2.0;
    const grid_pos = @as(i32, @intFromFloat((world_x + world_half_x) * grid_scale_x));
    return @max(0, @min(GRID_SIZE - 1, grid_pos));
}

pub inline fn worldToGridY(world_y: f32) i32 {
    const world_height = main_module.get_world_height();
    const grid_scale_y = @as(f32, GRID_SIZE) / world_height;
    const world_half_y = world_height / 2.0;
    const grid_pos = @as(i32, @intFromFloat((world_y + world_half_y) * grid_scale_y));
    return @max(0, @min(GRID_SIZE - 1, grid_pos));
}

// Legacy function for compatibility
pub inline fn worldToGrid(world_pos: f32) i32 {
    return worldToGridX(world_pos); // Default to X-axis behavior
}

pub fn getGridCell(x: f32, y: f32) *GridCell {
    const gx = worldToGridX(x);
    const gy = worldToGridY(y);
    return &spatial_grid[@intCast(gx)][@intCast(gy)];
}

pub fn initializeGrid() void {
    if (!grid_initialized) {
        for (0..GRID_SIZE) |i| {
            for (0..GRID_SIZE) |j| {
                spatial_grid[i][j] = GridCell.init();
            }
        }
        grid_initialized = true;
    }
}

pub fn clearGrid() void {
    for (0..GRID_SIZE) |i| {
        for (0..GRID_SIZE) |j| {
            spatial_grid[i][j].clear();
        }
    }
}

pub fn populateGrid(particles: anytype, particle_count: u32) void {
    clearGrid();
    for (0..particle_count) |i| {
        const particle = &particles[i];
        const cell = getGridCell(particle.x, particle.y);
        cell.add(@intCast(i));
    }
}