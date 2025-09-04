const std = @import("std");
const main = @import("main.zig");
const spatial = @import("spatial.zig");

// Mouse system constants
const GRAB_RADIUS = main.PARTICLE_SIZE * 5.0;
const MAX_GRAB_PARTICLES = 5;
const GRAB_SPRING_STIFFNESS = 1000000.0;
const MOUSE_PARTICLE_MASS = std.math.inf(f32);

// Mouse state
pub var mouse_particle: ?main.ParticleHandle = null;
var mouse_position_x: f32 = 0.0;
var mouse_position_y: f32 = 0.0;
var mouse_is_pressed: bool = false;
var grabbed_particles: [MAX_GRAB_PARTICLES]main.ParticleHandle = undefined;
var grabbed_springs: [MAX_GRAB_PARTICLES]main.SpringHandle = undefined;
var grab_count: u32 = 0;

pub fn initMouseSystem() void {
    // Create virtual mouse particle with infinite mass
    const mouse_particle_data = main.Particle{
        .x = 0.0,
        .y = 0.0,
        .predicted_x = 0.0,
        .predicted_y = 0.0,
        .vx = 0.0,
        .vy = 0.0,
        .mass = MOUSE_PARTICLE_MASS,
        .grid_id = 255, // Special ID for mouse particle
        .desired_valence = 0,
        .current_valence = 0,
    };
    
    mouse_particle = main.spawnParticle(mouse_particle_data);
    
    // Initialize grab arrays
    for (0..MAX_GRAB_PARTICLES) |i| {
        grabbed_particles[i] = main.ParticleHandle.invalid();
        grabbed_springs[i] = main.SpringHandle.invalid();
    }
    grab_count = 0;
}

pub fn updateMousePosition(x: f32, y: f32) void {
    mouse_position_x = x;
    mouse_position_y = y;
    
    // Update virtual mouse particle position if it exists
    if (mouse_particle) |handle| {
        if (main.getParticlePtr(handle)) |particle| {
            particle.x = x;
            particle.y = y;
            particle.predicted_x = x;
            particle.predicted_y = y;
            // Ensure velocity stays zero (infinite mass)
            particle.vx = 0.0;
            particle.vy = 0.0;
        }
    }
}

pub fn setMousePressed(pressed: bool) void {
    if (pressed and !mouse_is_pressed) {
        // Mouse just pressed - grab particles
        startGrab();
    } else if (!pressed and mouse_is_pressed) {
        // Mouse just released - release all grabs
        endGrab();
    }
    mouse_is_pressed = pressed;
}

pub fn findParticlesInGrabRadius() [MAX_GRAB_PARTICLES]main.ParticleHandle {
    var nearby_particles: [MAX_GRAB_PARTICLES]main.ParticleHandle = undefined;
    var found_count: u32 = 0;
    
    // Initialize array with invalid handles
    for (0..MAX_GRAB_PARTICLES) |i| {
        nearby_particles[i] = main.ParticleHandle.invalid();
    }
    
    // Use spatial grid to find nearby particles efficiently
    const gx = spatial.worldToGridX(mouse_position_x);
    const gy = spatial.worldToGridY(mouse_position_y);
    
    // Check the mouse cell and 8 surrounding cells
    var dy: i32 = -1;
    while (dy <= 1 and found_count < MAX_GRAB_PARTICLES) : (dy += 1) {
        var dx: i32 = -1;
        while (dx <= 1 and found_count < MAX_GRAB_PARTICLES) : (dx += 1) {
            const check_x = gx + dx;
            const check_y = gy + dy;
            
            // Bounds check
            if (check_x >= 0 and check_x < @as(i32, @intCast(spatial.grid_size_x)) and 
                check_y >= 0 and check_y < @as(i32, @intCast(spatial.grid_size_y))) {
                
                const cell = &spatial.spatial_grid[@intCast(check_x)][@intCast(check_y)];
                
                // Check all particles in this cell
                for (0..cell.count) |i| {
                    const particle_dense_index = cell.particles[i];
                    
                    // Get the particle handle from dense array
                    if (main.getParticleHandleByIndex(particle_dense_index)) |particle_handle| {
                        // Skip if this is the mouse particle itself
                        if (mouse_particle != null and particle_handle.eql(mouse_particle.?)) {
                            continue;
                        }
                        
                        if (main.getParticle(particle_handle)) |particle| {
                            // Calculate distance to mouse
                            const dx_dist = particle.x - mouse_position_x;
                            const dy_dist = particle.y - mouse_position_y;
                            const distance = @sqrt(dx_dist * dx_dist + dy_dist * dy_dist);
                            
                            // Check if within grab radius
                            if (distance <= GRAB_RADIUS and found_count < MAX_GRAB_PARTICLES) {
                                nearby_particles[found_count] = particle_handle;
                                found_count += 1;
                            }
                        }
                    }
                }
            }
        }
    }
    
    return nearby_particles;
}

fn startGrab() void {
    if (mouse_particle == null) return;
    
    // Find particles within grab radius
    const nearby_particles = findParticlesInGrabRadius();
    grab_count = 0;
    
    // Create springs to nearby particles
    for (0..MAX_GRAB_PARTICLES) |i| {
        const particle_handle = nearby_particles[i];
        if (!particle_handle.isValid()) break;
        
        // Calculate rest length (distance at moment of connection)
        if (main.getParticle(particle_handle)) |particle| {
            const dx = particle.x - mouse_position_x;
            const dy = particle.y - mouse_position_y;
            const rest_length = @sqrt(dx * dx + dy * dy);
            
            // Create spring between mouse particle and grabbed particle
            const spring = main.Spring{
                .particle_a = mouse_particle.?,
                .particle_b = particle_handle,
                .rest_length = rest_length,
            };
            
            const spring_handle = main.spawnSpring(spring);
            if (spring_handle.isValid()) {
                grabbed_particles[grab_count] = particle_handle;
                grabbed_springs[grab_count] = spring_handle;
                grab_count += 1;
            }
        }
    }
}

fn endGrab() void {
    // Destroy all grab springs
    for (0..grab_count) |i| {
        if (grabbed_springs[i].isValid()) {
            main.destroySpring(grabbed_springs[i]);
            grabbed_springs[i] = main.SpringHandle.invalid();
        }
        grabbed_particles[i] = main.ParticleHandle.invalid();
    }
    grab_count = 0;
}

pub fn updateMousePhysics() void {
    // Ensure mouse particle stays at cursor position and has zero velocity
    if (mouse_particle) |handle| {
        if (main.getParticlePtr(handle)) |particle| {
            // Lock position to cursor
            particle.x = mouse_position_x;
            particle.y = mouse_position_y;
            particle.predicted_x = mouse_position_x;
            particle.predicted_y = mouse_position_y;
            
            // Ensure zero velocity (infinite mass behavior)
            particle.vx = 0.0;
            particle.vy = 0.0;
        }
    }
}

// Export functions for JavaScript interface
pub fn getMousePositionX() f32 {
    return mouse_position_x;
}

pub fn getMousePositionY() f32 {
    return mouse_position_y;
}

pub fn isMousePressed() bool {
    return mouse_is_pressed;
}

pub fn getGrabCount() u32 {
    return grab_count;
}

pub fn getGrabbedParticle(index: u32) ?main.ParticleHandle {
    if (index >= grab_count) return null;
    return grabbed_particles[index];
}

pub fn isMouseParticle(handle: main.ParticleHandle) bool {
    if (mouse_particle == null) return false;
    return handle.eql(mouse_particle.?);
}