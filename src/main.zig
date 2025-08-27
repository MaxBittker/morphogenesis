const std = @import("std");
const math = std.math;

// Boids simulation parameters  
const PARTICLE_COUNT = 150;
const MAX_SPEED = 0.8;
const MAX_FORCE = 0.05;
const SEPARATION_RADIUS = 0.12;
const ALIGNMENT_RADIUS = 0.25;
const COHESION_RADIUS = 0.35;
const WORLD_SIZE = 1.8;  // Keep particles well within bounds regardless of aspect ratio

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
    
    pub fn update(self: *Self, neighbors: []const Particle, dt: f32) void {
        const sep = self.separate(neighbors);
        const ali = self.alignment(neighbors);
        const coh = self.cohesion(neighbors);
        
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
};

// Global particle system
var particles: [PARTICLE_COUNT]Particle = undefined;
var particles_initialized = false;

// WebGPU bindings for Emscripten
extern fn emscripten_webgpu_get_device() u32;
extern fn emscripten_webgpu_create_render_pass_encoder(device: u32, color_attachment: u32, depth_attachment: u32) u32;
extern fn emscripten_webgpu_render_pass_draw(encoder: u32, vertex_count: u32) void;
extern fn emscripten_webgpu_submit_render_pass(encoder: u32) void;

// Console logging
extern fn console_log(ptr: [*]const u8, len: usize) void;

fn log(comptime fmt: []const u8, args: anytype) void {
    var buffer: [1024]u8 = undefined;
    const message = std.fmt.bufPrint(buffer[0..], fmt, args) catch "Log message too long";
    console_log(message.ptr, message.len);
}

var device_handle: u32 = 0;

export fn init() void {
    log("Initializing Boids particle system...", .{});
    device_handle = emscripten_webgpu_get_device();
    log("WebGPU device initialized: {}", .{device_handle});
    
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
        log("Initialized {} particles", .{PARTICLE_COUNT});
    }
}

export fn update_particles(dt: f32) void {
    if (!particles_initialized) return;
    
    // Update each particle with Boids algorithm
    for (&particles, 0..) |*particle, i| {
        // Create a slice excluding the current particle for neighbor calculations
        var neighbors: [PARTICLE_COUNT - 1]Particle = undefined;
        var neighbor_idx: usize = 0;
        
        for (particles, 0..) |other, j| {
            if (i != j) {
                neighbors[neighbor_idx] = other;
                neighbor_idx += 1;
            }
        }
        
        particle.update(neighbors[0..neighbor_idx], dt);
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

export fn render() void {
    if (device_handle == 0) {
        log("Device not initialized!", .{});
        return;
    }
    
    // Create render pass
    const render_pass = emscripten_webgpu_create_render_pass_encoder(device_handle, 0, 0);
    
    // Draw particles (will be handled in JavaScript)
    emscripten_webgpu_render_pass_draw(render_pass, PARTICLE_COUNT * 3); // 3 vertices per particle triangle
    
    // Submit render pass
    emscripten_webgpu_submit_render_pass(render_pass);
}

export fn add(a: i32, b: i32) i32 {
    const result = a + b;
    log("Computing {} + {} = {}", .{ a, b, result });
    return result;
}

// Compute shader entry point for future implementation
export fn compute_demo(input: f32) f32 {
    // Simple compute operation - square the input
    const result = input * input;
    log("Compute: {} squared = {}", .{ input, result });
    return result;
}