const std = @import("std");
const spatial = @import("spatial.zig");
const mouse = @import("mouse.zig");

pub const Vec2 = struct {
    x: f32,
    y: f32,
};

// Physics constants are now defined in main.zig

const ConstraintType = enum {
    distance,
    collision,
    mouse_spring,
};

pub const DistanceConstraint = struct {
    particle_a: ParticleHandle,
    particle_b: ParticleHandle,
    rest_length: f32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, b: ParticleHandle, length: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .rest_length = length,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }

    pub fn evaluate(self: *const Self, getParticlePtr: anytype) f32 {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return 0.0;
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return 0.0;

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);
        return distance - self.rest_length;
    }

    pub fn getGradients(self: *const Self, getParticlePtr: anytype) struct { ga: Vec2, gb: Vec2, valid: bool } {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);

        if (distance < 1e-6) {
            return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        }

        const norm_x = dx / distance;
        const norm_y = dy / distance;

        return .{
            .ga = Vec2{ .x = -norm_x, .y = -norm_y },
            .gb = Vec2{ .x = norm_x, .y = norm_y },
            .valid = true,
        };
    }
};

pub const CollisionConstraint = struct {
    particle_a: ParticleHandle,
    particle_b: ParticleHandle,
    min_distance: f32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, b: ParticleHandle, min_dist: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .min_distance = min_dist,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }

    pub fn evaluate(self: *const Self, getParticlePtr: anytype) f32 {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return 0.0;
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return 0.0;

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);
        return @min(0.0, distance - self.min_distance);
    }

    pub fn getGradients(self: *const Self, getParticlePtr: anytype) struct { ga: Vec2, gb: Vec2, valid: bool } {
        const particle_a_ptr = getParticlePtr(self.particle_a) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        const particle_b_ptr = getParticlePtr(self.particle_b) orelse return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };

        const dx = particle_b_ptr.predicted_x - particle_a_ptr.predicted_x;
        const dy = particle_b_ptr.predicted_y - particle_a_ptr.predicted_y;
        const distance = @sqrt(dx * dx + dy * dy);

        if (distance < 1e-6) {
            return .{ .ga = Vec2{ .x = 0, .y = 0 }, .gb = Vec2{ .x = 0, .y = 0 }, .valid = false };
        }

        const norm_x = dx / distance;
        const norm_y = dy / distance;

        return .{
            .ga = Vec2{ .x = -norm_x, .y = -norm_y },
            .gb = Vec2{ .x = norm_x, .y = norm_y },
            .valid = true,
        };
    }
};

pub const SeparationConstraint = struct {
    particle_a: ParticleHandle,
    particle_b: ParticleHandle,
    target_distance: f32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, b: ParticleHandle, target_dist: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .particle_b = b,
            .target_distance = target_dist,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }
};

pub const AlignmentConstraint = struct {
    particle_a: ParticleHandle,
    neighbor_particles: [16]ParticleHandle,
    neighbor_count: u32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .neighbor_particles = undefined,
            .neighbor_count = 0,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }
};

pub const CohesionConstraint = struct {
    particle_a: ParticleHandle,
    target_x: f32,
    target_y: f32,
    compliance: f32,
    lagrange_multiplier: f32,

    const Self = @This();

    pub fn init(a: ParticleHandle, target_x: f32, target_y: f32, stiffness: f32, dt: f32) Self {
        return Self{
            .particle_a = a,
            .target_x = target_x,
            .target_y = target_y,
            .compliance = 1.0 / (stiffness * dt * dt),
            .lagrange_multiplier = 0.0,
        };
    }
};

// Generic types for the solver - will be provided by main.zig
const ParticleHandle = @import("main.zig").ParticleHandle;
const SpringHandle = @import("main.zig").SpringHandle;
const MAX_SPRINGS = @import("main.zig").MAX_SPRINGS;
const PARTICLE_SIZE = @import("main.zig").PARTICLE_SIZE;

pub const PhysicsSystem = struct {
    // Constraint arrays
    distance_constraints: []DistanceConstraint,
    distance_constraint_count: u32,
    collision_constraints: []CollisionConstraint,
    collision_constraint_count: u32,
    boids_separation_constraints: []SeparationConstraint,
    boids_separation_constraint_count: u32,
    boids_alignment_constraints: []AlignmentConstraint,
    boids_alignment_constraint_count: u32,
    boids_cohesion_constraints: []CohesionConstraint,
    boids_cohesion_constraint_count: u32,

    // Callbacks for particle access
    getParticlePtr: *const fn (ParticleHandle) ?*anyopaque,
    getSpringPtr: *const fn (SpringHandle) ?*anyopaque,
    destroySpring: *const fn (SpringHandle) void,
    getDenseParticleIndex: *const fn (ParticleHandle) ?u32,

    const Self = @This();

    pub fn init(
        distance_constraints: []DistanceConstraint,
        collision_constraints: []CollisionConstraint,
        boids_separation_constraints: []SeparationConstraint,
        boids_alignment_constraints: []AlignmentConstraint,
        boids_cohesion_constraints: []CohesionConstraint,
        getParticlePtr: *const fn (ParticleHandle) ?*anyopaque,
        getSpringPtr: *const fn (SpringHandle) ?*anyopaque,
        destroySpring: *const fn (SpringHandle) void,
        getDenseParticleIndex: *const fn (ParticleHandle) ?u32,
    ) Self {
        return Self{
            .distance_constraints = distance_constraints,
            .distance_constraint_count = 0,
            .collision_constraints = collision_constraints,
            .collision_constraint_count = 0,
            .boids_separation_constraints = boids_separation_constraints,
            .boids_separation_constraint_count = 0,
            .boids_alignment_constraints = boids_alignment_constraints,
            .boids_alignment_constraint_count = 0,
            .boids_cohesion_constraints = boids_cohesion_constraints,
            .boids_cohesion_constraint_count = 0,
            .getParticlePtr = getParticlePtr,
            .getSpringPtr = getSpringPtr,
            .destroySpring = destroySpring,
            .getDenseParticleIndex = getDenseParticleIndex,
        };
    }

    pub fn generateConstraints(
        self: *Self,
        dt: f32,
        dense_springs: anytype,
        dense_spring_handles: anytype,
        spring_count: u32,
        dense_particles: anytype,
        dense_particle_handles: anytype,
        dense_particle_count: u32,
        distance_stiffness: f32,
        collision_stiffness: f32,
    ) void {
        self.distance_constraint_count = 0;
        self.collision_constraint_count = 0;
        self.boids_separation_constraint_count = 0;
        self.boids_alignment_constraint_count = 0;
        self.boids_cohesion_constraint_count = 0;

        var springs_to_remove: [MAX_SPRINGS]SpringHandle = undefined;
        var remove_count: u32 = 0;

        // Generate distance constraints from springs
        for (0..spring_count) |i| {
            const spring = &dense_springs[i];
            const spring_handle = dense_spring_handles[i];

            const particle_a = self.getParticlePtr(spring.particle_a);
            const particle_b = self.getParticlePtr(spring.particle_b);

            if (particle_a != null and particle_b != null) {
                // Cast back to proper particle types for distance calculation
                const p_a: *const @import("main.zig").Particle = @ptrCast(@alignCast(particle_a.?));
                const p_b: *const @import("main.zig").Particle = @ptrCast(@alignCast(particle_b.?));

                const dx = p_b.x - p_a.x;
                const dy = p_b.y - p_a.y;
                const current_length = @sqrt(dx * dx + dy * dy);

                const is_mouse_spring = mouse.isMouseParticle(spring.particle_a) or mouse.isMouseParticle(spring.particle_b);
                const max_allowed_length = if (is_mouse_spring) spring.rest_length * 10.0 else spring.rest_length * 2.0;
                const min_allowed_length = if (is_mouse_spring) spring.rest_length * 0.5 else spring.rest_length * 0.5;

                if (current_length > max_allowed_length or current_length < min_allowed_length) {
                    if (remove_count < MAX_SPRINGS) {
                        springs_to_remove[remove_count] = spring_handle;
                        remove_count += 1;

                        // Refund valence - need to cast particle pointers
                        const p_a_mut = @constCast(p_a);
                        const p_b_mut = @constCast(p_b);
                        p_a_mut.current_valence = @max(0, p_a_mut.current_valence - 1);
                        p_b_mut.current_valence = @max(0, p_b_mut.current_valence - 1);
                    }
                } else if (self.distance_constraint_count < self.distance_constraints.len) {
                    const spring_compliance = 1.0 / (distance_stiffness * dt * dt);
                    self.distance_constraints[self.distance_constraint_count] = DistanceConstraint{
                        .particle_a = spring.particle_a,
                        .particle_b = spring.particle_b,
                        .rest_length = spring.rest_length,
                        .compliance = spring_compliance,
                        .lagrange_multiplier = 0.0,
                    };
                    self.distance_constraint_count += 1;
                }
            }
        }

        // Remove overstretched springs
        for (0..remove_count) |j| {
            self.destroySpring(springs_to_remove[j]);
        }

        // Generate collision constraints using spatial grid
        spatial.populateGrid(dense_particles, dense_particle_count);

        for (0..dense_particle_count) |i| {
            if (!mouse.isMouseParticle(dense_particle_handles[i])) {
                self.generateCollisionConstraintsForParticle(dense_particle_handles[i], dt, dense_particle_handles, dense_particle_count, collision_stiffness);
            }
        }
    }

    fn generateCollisionConstraintsForParticle(
        self: *Self,
        particle_handle: ParticleHandle,
        dt: f32,
        handles_array: anytype,
        handles_count: u32,
        collision_stiffness: f32,
    ) void {
        const particle_ptr = self.getParticlePtr(particle_handle) orelse return;
        const particle: *const @import("main.zig").Particle = @ptrCast(@alignCast(particle_ptr));

        const gx = spatial.worldToGridX(particle.predicted_x);
        const gy = spatial.worldToGridY(particle.predicted_y);

        var dy: i32 = -1;
        while (dy <= 1) : (dy += 1) {
            var dx: i32 = -1;
            while (dx <= 1) : (dx += 1) {
                const check_x = gx + dx;
                const check_y = gy + dy;

                if (check_x >= 0 and check_x < @as(i32, @intCast(spatial.grid_size_x)) and check_y >= 0 and check_y < @as(i32, @intCast(spatial.grid_size_y))) {
                    const cell = &spatial.spatial_grid[@intCast(check_x)][@intCast(check_y)];

                    for (0..cell.count) |i| {
                        const neighbor_index = cell.particles[i];

                        if (neighbor_index >= handles_count) continue;
                        const neighbor_handle = handles_array[neighbor_index];

                        if (!neighbor_handle.eql(particle_handle) and
                            particle_handle.index < neighbor_handle.index and
                            !mouse.isMouseParticle(neighbor_handle))
                        {
                            const other_ptr = self.getParticlePtr(neighbor_handle) orelse continue;
                            const other: *const @import("main.zig").Particle = @ptrCast(@alignCast(other_ptr));
                            const dx_pred = particle.predicted_x - other.predicted_x;
                            const dy_pred = particle.predicted_y - other.predicted_y;
                            const dist_sq = dx_pred * dx_pred + dy_pred * dy_pred;

                            const ball_radius = PARTICLE_SIZE;
                            const contact_distance = ball_radius * 2.0;
                            const current_distance = @sqrt(dist_sq);

                            if (current_distance < contact_distance) {
                                if (self.collision_constraint_count < self.collision_constraints.len) {
                                    const billiard_stiffness = collision_stiffness * 10.0;
                                    const collision_compliance = 1.0 / (billiard_stiffness * dt * dt);

                                    self.collision_constraints[self.collision_constraint_count] = CollisionConstraint{
                                        .particle_a = particle_handle,
                                        .particle_b = neighbor_handle,
                                        .min_distance = contact_distance,
                                        .compliance = collision_compliance,
                                        .lagrange_multiplier = 0.0,
                                    };
                                    self.collision_constraint_count += 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn solveConstraints(self: *Self, dt: f32, dense_particles: anytype) void {
        for (0..self.distance_constraint_count) |i| {
            self.solveDistanceConstraint(&self.distance_constraints[i], dt, dense_particles);
        }

        for (0..self.collision_constraint_count) |i| {
            self.solveCollisionConstraint(&self.collision_constraints[i], dt, dense_particles);
        }

        for (0..self.boids_separation_constraint_count) |i| {
            self.solveSeparationConstraint(&self.boids_separation_constraints[i], dt);
        }

        for (0..self.boids_alignment_constraint_count) |i| {
            self.solveAlignmentConstraint(&self.boids_alignment_constraints[i], dt);
        }

        for (0..self.boids_cohesion_constraint_count) |i| {
            self.solveCohesionConstraint(&self.boids_cohesion_constraints[i], dt);
        }
    }

    pub fn solveCollisionConstraintsOnly(self: *Self, dt: f32, dense_particles: anytype) void {
        for (0..self.collision_constraint_count) |i| {
            self.solveCollisionConstraint(&self.collision_constraints[i], dt, dense_particles);
        }
    }

    fn solveDistanceConstraint(self: *Self, constraint: *DistanceConstraint, dt: f32, dense_particles: anytype) void {
        const idx_a = self.getDenseParticleIndex(constraint.particle_a) orelse return;
        const idx_b = self.getDenseParticleIndex(constraint.particle_b) orelse return;
        const particle_a = &dense_particles[idx_a];
        const particle_b = &dense_particles[idx_b];

        const dx = particle_a.predicted_x - particle_b.predicted_x;
        const dy = particle_a.predicted_y - particle_b.predicted_y;
        const current_distance = @sqrt(dx * dx + dy * dy);

        if (current_distance < 0.001) return;

        const constraint_value = current_distance - constraint.rest_length;

        const grad_x = dx / current_distance;
        const grad_y = dy / current_distance;

        const w_a = 1.0 / particle_a.mass;
        const w_b = 1.0 / particle_b.mass;
        const grad_length_sq = grad_x * grad_x + grad_y * grad_y;
        const denominator = w_a * grad_length_sq + w_b * grad_length_sq + constraint.compliance / (dt * dt);

        if (denominator < 0.001) return;

        const delta_lambda = -(constraint_value + constraint.compliance * constraint.lagrange_multiplier / (dt * dt)) / denominator;

        constraint.lagrange_multiplier += delta_lambda;

        const correction_a_x = w_a * delta_lambda * grad_x;
        const correction_a_y = w_a * delta_lambda * grad_y;
        const correction_b_x = -w_b * delta_lambda * grad_x;
        const correction_b_y = -w_b * delta_lambda * grad_y;

        particle_a.predicted_x += correction_a_x;
        particle_a.predicted_y += correction_a_y;
        particle_b.predicted_x += correction_b_x;
        particle_b.predicted_y += correction_b_y;
    }

    fn solveCollisionConstraint(self: *Self, constraint: *CollisionConstraint, dt: f32, dense_particles: anytype) void {
        _ = dt;
        const idx_a = self.getDenseParticleIndex(constraint.particle_a) orelse return;
        const idx_b = self.getDenseParticleIndex(constraint.particle_b) orelse return;
        const particle_a = &dense_particles[idx_a];
        const particle_b = &dense_particles[idx_b];

        const dx = particle_a.predicted_x - particle_b.predicted_x;
        const dy = particle_a.predicted_y - particle_b.predicted_y;
        const current_distance = @sqrt(dx * dx + dy * dy);

        if (current_distance < 0.001) {
            particle_a.predicted_x += 0.01;
            particle_b.predicted_x -= 0.01;
            return;
        }

        const constraint_value = constraint.min_distance - current_distance;

        if (constraint_value <= 0) return;

        const normal_x = dx / current_distance;
        const normal_y = dy / current_distance;

        const separation = constraint_value * 0.5;
        particle_a.predicted_x += normal_x * separation;
        particle_a.predicted_y += normal_y * separation;
        particle_b.predicted_x -= normal_x * separation;
        particle_b.predicted_y -= normal_y * separation;

        const rel_vx = particle_a.vx - particle_b.vx;
        const rel_vy = particle_a.vy - particle_b.vy;
        const rel_vel_normal = rel_vx * normal_x + rel_vy * normal_y;

        if (rel_vel_normal > 0) return;

        const restitution = 0.9;
        const impulse_magnitude = -(1.0 + restitution) * rel_vel_normal / (1.0 / particle_a.mass + 1.0 / particle_b.mass);

        const impulse_x = impulse_magnitude * normal_x;
        const impulse_y = impulse_magnitude * normal_y;

        particle_a.vx += impulse_x / particle_a.mass;
        particle_a.vy += impulse_y / particle_a.mass;
        particle_b.vx -= impulse_x / particle_b.mass;
        particle_b.vy -= impulse_y / particle_b.mass;
    }

    fn solveSeparationConstraint(self: *Self, constraint: *SeparationConstraint, dt: f32) void {
        const particle_a_ptr = self.getParticlePtr(constraint.particle_a) orelse return;
        const particle_b_ptr = self.getParticlePtr(constraint.particle_b) orelse return;
        const particle_a: *@import("main.zig").Particle = @ptrCast(@alignCast(particle_a_ptr));
        const particle_b: *@import("main.zig").Particle = @ptrCast(@alignCast(particle_b_ptr));

        const dx = particle_a.predicted_x - particle_b.predicted_x;
        const dy = particle_a.predicted_y - particle_b.predicted_y;
        const current_distance = @sqrt(dx * dx + dy * dy);

        if (current_distance < 0.001) return;

        if (current_distance >= constraint.target_distance) return;

        const constraint_value = constraint.target_distance - current_distance;

        const grad_x = -dx / current_distance;
        const grad_y = -dy / current_distance;

        const w_a = 1.0 / particle_a.mass;
        const w_b = 1.0 / particle_b.mass;
        const grad_length_sq = grad_x * grad_x + grad_y * grad_y;
        const denominator = w_a * grad_length_sq + w_b * grad_length_sq + constraint.compliance / (dt * dt);

        if (denominator < 0.001) return;

        const delta_lambda = -(constraint_value + constraint.compliance * constraint.lagrange_multiplier / (dt * dt)) / denominator;

        constraint.lagrange_multiplier += delta_lambda;

        const correction_a_x = w_a * delta_lambda * grad_x;
        const correction_a_y = w_a * delta_lambda * grad_y;
        const correction_b_x = -w_b * delta_lambda * grad_x;
        const correction_b_y = -w_b * delta_lambda * grad_y;

        particle_a.predicted_x += correction_a_x;
        particle_a.predicted_y += correction_a_y;
        particle_b.predicted_x += correction_b_x;
        particle_b.predicted_y += correction_b_y;
    }

    fn solveAlignmentConstraint(self: *Self, constraint: *AlignmentConstraint, dt: f32) void {
        const particle_ptr = self.getParticlePtr(constraint.particle_a) orelse return;
        const particle: *@import("main.zig").Particle = @ptrCast(@alignCast(particle_ptr));

        if (constraint.neighbor_count == 0) return;

        var avg_vx: f32 = 0;
        var avg_vy: f32 = 0;
        var valid_neighbors: u32 = 0;

        for (0..constraint.neighbor_count) |i| {
            const neighbor_ptr = self.getParticlePtr(constraint.neighbor_particles[i]) orelse continue;
            const neighbor: *const @import("main.zig").Particle = @ptrCast(@alignCast(neighbor_ptr));
            avg_vx += neighbor.vx;
            avg_vy += neighbor.vy;
            valid_neighbors += 1;
        }

        if (valid_neighbors == 0) return;

        avg_vx /= @as(f32, @floatFromInt(valid_neighbors));
        avg_vy /= @as(f32, @floatFromInt(valid_neighbors));

        const velocity_diff_x = avg_vx - particle.vx;
        const velocity_diff_y = avg_vy - particle.vy;

        const alignment_strength = 1.0 / (constraint.compliance + dt * dt);
        const correction_x = velocity_diff_x * dt * alignment_strength * 0.01;
        const correction_y = velocity_diff_y * dt * alignment_strength * 0.01;

        particle.predicted_x += correction_x;
        particle.predicted_y += correction_y;
    }

    fn solveCohesionConstraint(self: *Self, constraint: *CohesionConstraint, dt: f32) void {
        const particle_ptr = self.getParticlePtr(constraint.particle_a) orelse return;
        const particle: *@import("main.zig").Particle = @ptrCast(@alignCast(particle_ptr));

        const dx = particle.predicted_x - constraint.target_x;
        const dy = particle.predicted_y - constraint.target_y;
        const distance_to_center = @sqrt(dx * dx + dy * dy);

        if (distance_to_center < 0.001) return;

        const constraint_value = distance_to_center;

        const grad_x = dx / distance_to_center;
        const grad_y = dy / distance_to_center;

        const cohesion_strength = 1.0 / (constraint.compliance + dt * dt);

        const correction_x = -grad_x * constraint_value * cohesion_strength * 0.001;
        const correction_y = -grad_y * constraint_value * cohesion_strength * 0.001;

        particle.predicted_x += correction_x;
        particle.predicted_y += correction_y;
    }
};