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
const ParticleType = main.ParticleType;
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
    // Generate grid positions in a 5x5 layout
    var grid_positions: [25][2]f32 = undefined;
    var grid_index: u32 = 0;

    for (0..5) |row| {
        for (0..5) |col| {
            const x = (@as(f32, @floatFromInt(col)) - 2.0) * 0.4;
            const y = (@as(f32, @floatFromInt(row)) - 2.0) * 0.4;
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

                main.setParticle(particle_index, Particle.init(x, y, ParticleType.grid_particle, @intCast(grid_id)));
                particle_index += 1;
            }
        }
    }
}

// Initialize free agent Boids with random positions
pub fn initializeFreeAgents() void {
    const start_index = TOTAL_GRID_PARTICLES;

    for (start_index..PARTICLE_COUNT) |i| {
        const seed = @as(f32, @floatFromInt(i));
        const range = WORLD_SIZE * 0.8;
        const x = ((@sin(seed * 12.9898) + 1.0) * 0.5 - 0.5) * range;
        const y = ((@sin(seed * 78.233) + 1.0) * 0.5 - 0.5) * range;

        main.setParticle(i, Particle.init(x, y, ParticleType.free_agent, 255));
    }
}

// Create hexagonal lattice connections within each grid
pub fn initializeSprings() void {
    main.setSpringCount(0);

    // Initialize connection lookup table
    for (0..PARTICLE_COUNT) |i| {
        main.setParticleConnectionCount(i, 0);
    }

    for (0..GRID_COUNT) |grid_id| {
        const grid_start = grid_id * PARTICLES_PER_GRID;
        const springs_before = main.getSpringCount();

        // Create hexagonal lattice connections (6-connected)
        for (0..GRID_PARTICLE_SIZE) |row| {
            for (0..GRID_PARTICLE_SIZE) |col| {
                const current_particle = @as(u32, @intCast(grid_start + row * GRID_PARTICLE_SIZE + col));
                const is_odd_row = (row % 2 == 1);

                // Connect to left neighbor
                if (col > 0) {
                    const left_particle = @as(u32, @intCast(grid_start + row * GRID_PARTICLE_SIZE + col - 1));
                    addSpringConnection(current_particle, left_particle);
                }

                // Connect to right neighbor
                if (col < GRID_PARTICLE_SIZE - 1) {
                    const right_particle = @as(u32, @intCast(grid_start + row * GRID_PARTICLE_SIZE + col + 1));
                    addSpringConnection(current_particle, right_particle);
                }

                // Connect to neighbors in the row above (hexagonal pattern)
                if (row > 0) {
                    if (is_odd_row) {
                        // For odd rows: connect to upper-left and upper-right
                        // Upper-left
                        const upper_left = @as(u32, @intCast(grid_start + (row - 1) * GRID_PARTICLE_SIZE + col));
                        addSpringConnection(current_particle, upper_left);
                        // Upper-right
                        if (col < GRID_PARTICLE_SIZE - 1) {
                            const upper_right = @as(u32, @intCast(grid_start + (row - 1) * GRID_PARTICLE_SIZE + col + 1));
                            addSpringConnection(current_particle, upper_right);
                        }
                    } else {
                        // For even rows: connect to upper-left and upper-right
                        // Upper-left
                        if (col > 0) {
                            const upper_left = @as(u32, @intCast(grid_start + (row - 1) * GRID_PARTICLE_SIZE + col - 1));
                            addSpringConnection(current_particle, upper_left);
                        }
                        // Upper-right
                        const upper_right = @as(u32, @intCast(grid_start + (row - 1) * GRID_PARTICLE_SIZE + col));
                        addSpringConnection(current_particle, upper_right);
                    }
                }

                // Connect to neighbors in the row below (hexagonal pattern)
                if (row < GRID_PARTICLE_SIZE - 1) {
                    if (is_odd_row) {
                        // For odd rows: connect to lower-left and lower-right
                        // Lower-left
                        const lower_left = @as(u32, @intCast(grid_start + (row + 1) * GRID_PARTICLE_SIZE + col));
                        addSpringConnection(current_particle, lower_left);
                        // Lower-right
                        if (col < GRID_PARTICLE_SIZE - 1) {
                            const lower_right = @as(u32, @intCast(grid_start + (row + 1) * GRID_PARTICLE_SIZE + col + 1));
                            addSpringConnection(current_particle, lower_right);
                        }
                    } else {
                        // For even rows: connect to lower-left and lower-right
                        // Lower-left
                        if (col > 0) {
                            const lower_left = @as(u32, @intCast(grid_start + (row + 1) * GRID_PARTICLE_SIZE + col - 1));
                            addSpringConnection(current_particle, lower_left);
                        }
                        // Lower-right
                        const lower_right = @as(u32, @intCast(grid_start + (row + 1) * GRID_PARTICLE_SIZE + col));
                        addSpringConnection(current_particle, lower_right);
                    }
                }
            }
        }

        const springs_created = main.getSpringCount() - springs_before;
        log("Grid {}: created {} springs (total: {})", .{ grid_id, springs_created, main.getSpringCount() });
    }

    log("Total springs created: {} (max: {})", .{ main.getSpringCount(), MAX_SPRINGS });
}

// Helper function to add a spring connection between two particles
pub fn addSpringConnection(particle_a: u32, particle_b: u32) void {
    const current_spring_count = main.getSpringCount();
    if (current_spring_count >= MAX_SPRINGS) {
        log("WARNING: Hit MAX_SPRINGS limit ({}) when trying to connect {} to {}", .{ MAX_SPRINGS, particle_a, particle_b });
        return;
    }

    main.setSpring(current_spring_count, Spring.init(particle_a, particle_b, SPRING_REST_LENGTH));

    // Update connection lookup for both particles
    addParticleConnection(particle_a, current_spring_count);
    addParticleConnection(particle_b, current_spring_count);

    main.setSpringCount(current_spring_count + 1);
}

// Helper function to add a spring connection to a particle's lookup table
pub fn addParticleConnection(particle_index: u32, spring_index: u32) void {
    const count = main.getParticleConnectionCount(particle_index);
    if (count < MAX_CONNECTIONS_PER_PARTICLE) {
        main.setParticleConnection(particle_index, count, spring_index);
        main.setParticleConnectionCount(particle_index, count + 1);
    }
}

// Find the closest particle to a given position
pub fn findClosestParticle(target_x: f32, target_y: f32) u32 {
    var closest_index: u32 = 0;
    var closest_distance_sq: f32 = std.math.inf(f32);

    for (0..PARTICLE_COUNT) |i| {
        const particle = main.getParticle(i);
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