const std = @import("std");

// Spatial partitioning grid for O(n) performance
pub const GRID_SIZE = 25;
pub const MAX_PARTICLES_PER_CELL = 500;

// Pre-calculated constants for worldToGrid optimization
const GRID_SCALE = @as(f32, GRID_SIZE) / @import("main.zig").WORLD_SIZE;
const WORLD_HALF = @import("main.zig").WORLD_SIZE / 2.0;

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

// Spatial grid helper functions
pub inline fn worldToGrid(world_pos: f32) i32 {
    // Optimized: single multiply instead of add + divide + multiply
    const grid_pos = @as(i32, @intFromFloat((world_pos + WORLD_HALF) * GRID_SCALE));
    return @max(0, @min(GRID_SIZE - 1, grid_pos));
}

pub fn getGridCell(x: f32, y: f32) *GridCell {
    const gx = worldToGrid(x);
    const gy = worldToGrid(y);
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