const std = @import("std");
const main = @import("main.zig");

// Import constants and types from main
const GRID_COUNT = main.GRID_COUNT;
const GRID_PARTICLE_SIZE = main.GRID_PARTICLE_SIZE;
const PARTICLES_PER_GRID = main.PARTICLES_PER_GRID;
const TOTAL_GRID_PARTICLES = main.TOTAL_GRID_PARTICLES;
const FREE_AGENT_COUNT = main.FREE_AGENT_COUNT;
const PARTICLE_COUNT = main.PARTICLE_COUNT;
const WORLD_SIZE = main.WORLD_SIZE;
const GRID_SPACING = main.GRID_SPACING;
const SPRING_REST_LENGTH = main.SPRING_REST_LENGTH;
const MAX_SPRINGS = main.MAX_SPRINGS;
const MAX_CONNECTIONS_PER_PARTICLE = main.MAX_CONNECTIONS_PER_PARTICLE;

const Particle = main.Particle;
const Spring = main.Spring;

// Console logging
extern fn console_log(ptr: [*]const u8, len: usize) void;

fn log(comptime fmt: []const u8, args: anytype) void {
    var buffer: [1024]u8 = undefined;
    const message = std.fmt.bufPrint(buffer[0..], fmt, args) catch "Log message too long";
    console_log(message.ptr, message.len);
}

// Initialize grid particles in 5x5 layout of grids
pub fn initializeGridParticles() void {
    // Generate grid positions in a 5x5 layout using pixel coordinates
    var grid_positions: [25][2]f32 = undefined;
    var grid_index: u32 = 0;

    // Space grids 150 pixels apart
    const grid_spacing = 250.0;
    for (0..5) |row| {
        for (0..5) |col| {
            const x = (@as(f32, @floatFromInt(col)) - 2.0) * grid_spacing;
            const y = (@as(f32, @floatFromInt(row)) - 1.0) * grid_spacing;
            grid_positions[grid_index] = [_]f32{ x, y };
            grid_index += 1;
        }
    }

    var particle_index: u32 = 0;

    for (0..GRID_COUNT) |grid_id| {
        const base_x = grid_positions[grid_id][0];
        const base_y = grid_positions[grid_id][1];

        // Create hexagonal grid of particles with offset alternating rows
        for (0..GRID_PARTICLE_SIZE) |row| {
            for (0..GRID_PARTICLE_SIZE) |col| {
                // Offset odd rows by half a column width to create hexagonal pattern
                const col_offset: f32 = if (row % 2 == 1) 0.5 else 0.0;
                const grid_center_offset: f32 = (@as(f32, @floatFromInt(GRID_PARTICLE_SIZE)) - 1.0) / 2.0;
                const x = base_x + (@as(f32, @floatFromInt(col)) + col_offset - grid_center_offset) * GRID_SPACING;
                // Reduce row spacing for hexagonal pattern (cos(30°) ≈ 0.866)
                const y = base_y + (@as(f32, @floatFromInt(row)) - grid_center_offset) * GRID_SPACING * 0.866;

                const particle = Particle.initWithValence(x, y, @intCast(grid_id), 6);
                _ = main.spawnParticle(particle);
                particle_index += 1;
            }
        }
    }
}

// Initialize free agent Boids with random positions
pub fn initializeFreeAgents() void {
    const start_index = TOTAL_GRID_PARTICLES;
    const end_index = TOTAL_GRID_PARTICLES + FREE_AGENT_COUNT; // Only initialize initial free agents

    for (start_index..end_index) |i| {
        const seed = @as(f32, @floatFromInt(i));
        // Spawn across the world dimensions (with fallback for first frame)
        const world_width = main.get_world_width();
        const world_height = main.get_world_height();
        const fallback_width = 1920.0; // Reasonable default
        const fallback_height = 1080.0; // Reasonable default
        const actual_width = if (world_width > 10.0) world_width else fallback_width;
        const actual_height = if (world_height > 10.0) world_height else fallback_height;
        const range_x = actual_width * 0.4; // Use 80% of world width centered
        const range_y = actual_height * 0.4; // Use 80% of world height centered
        const x = ((@sin(seed * 12.9898) + 1.0) * 0.5 - 0.5) * range_x;
        const y = ((@sin(seed * 78.233) + 1.0) * 0.5 - 0.5) * range_y;

        const particle = Particle.initWithValence(x, y, 255, 0);
        _ = main.spawnParticle(particle);
    }
}

// Initialize spring system (no pre-made connections, valence system handles bonding)
pub fn initializeSprings() void {
    // Initialize connection lookup table
    for (0..PARTICLE_COUNT) |i| {
        main.setParticleConnectionCount(@intCast(i), 0);
    }

    // log("Springs system initialized - valence system will handle bond formation");
}

// Helper function to add a spring connection to a particle's lookup table
pub fn addParticleConnection(particle_index: u32, spring_handle: main.SpringHandle) void {
    const count = main.getParticleConnectionCount(particle_index);
    if (count < MAX_CONNECTIONS_PER_PARTICLE) {
        main.setParticleConnection(particle_index, count, spring_handle);
        main.setParticleConnectionCount(particle_index, count + 1);
    }
}

