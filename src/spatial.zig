const std = @import("std");

// Pre-calculated constants for worldToGrid optimization
const main_module = @import("main.zig");
const BASE_WORLD_SIZE = main_module.WORLD_SIZE;

// Spatial partitioning grid for O(n) performance
// Fixed bin size = 10 particle diameters
pub const BIN_SIZE_PIXELS = 6.0 * (2.0 * main_module.PARTICLE_SIZE); // 10 particle diameters
pub const MAX_PARTICLES_PER_CELL = 500;

// Dynamic grid size based on world dimensions and fixed bin size
pub var grid_size_x: u32 = 0;
pub var grid_size_y: u32 = 0;

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

// Maximum grid dimensions (for static allocation)
const MAX_GRID_SIZE = 100;

// Spatial grid for fast neighbor queries (using max size for static allocation)
pub var spatial_grid: [MAX_GRID_SIZE][MAX_GRID_SIZE]GridCell = undefined;
var grid_initialized = false;
var max_occupancy: u32 = 0;

// Spatial grid helper functions with fixed bin size
pub inline fn worldToGridX(world_x: f32) i32 {
    const world_width = main_module.get_world_width();
    const world_half_x = world_width / 2.0;
    const grid_pos = @as(i32, @intFromFloat((world_x + world_half_x) / BIN_SIZE_PIXELS));
    return @max(0, @min(@as(i32, @intCast(grid_size_x)) - 1, grid_pos));
}

pub inline fn worldToGridY(world_y: f32) i32 {
    const world_height = main_module.get_world_height();
    const world_half_y = world_height / 2.0;
    const grid_pos = @as(i32, @intFromFloat((world_y + world_half_y) / BIN_SIZE_PIXELS));
    return @max(0, @min(@as(i32, @intCast(grid_size_y)) - 1, grid_pos));
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

pub fn updateGridDimensions() void {
    // Calculate grid dimensions based on world size and fixed bin size
    const world_width = main_module.get_world_width();
    const world_height = main_module.get_world_height();

    grid_size_x = @min(MAX_GRID_SIZE, @max(1, @as(u32, @intFromFloat(world_width / BIN_SIZE_PIXELS)) + 1));
    grid_size_y = @min(MAX_GRID_SIZE, @max(1, @as(u32, @intFromFloat(world_height / BIN_SIZE_PIXELS)) + 1));
}

pub fn initializeGrid() void {
    if (!grid_initialized) {
        updateGridDimensions();

        for (0..MAX_GRID_SIZE) |i| {
            for (0..MAX_GRID_SIZE) |j| {
                spatial_grid[i][j] = GridCell.init();
            }
        }
        grid_initialized = true;
    }
}

pub fn clearGrid() void {
    max_occupancy = 0;
    for (0..grid_size_x) |i| {
        for (0..grid_size_y) |j| {
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

        // Track maximum occupancy
        if (cell.count > max_occupancy) {
            max_occupancy = cell.count;
        }
    }
}

// Get current maximum bin occupancy
pub fn getMaxOccupancy() u32 {
    return max_occupancy;
}

// Get grid dimensions
pub fn getGridDimensions() struct { width: u32, height: u32 } {
    return .{ .width = grid_size_x, .height = grid_size_y };
}
