const std = @import("std");

// Generational Index System for safe entity references
// Provides O(1) spawn/destroy with handle-based safety and dense iteration

pub const Generation = u16;
pub const Index = u16;

pub fn GenerationalArena(comptime T: type, comptime capacity: u32) type {
    return struct {
        const Self = @This();

        pub const Handle = packed struct {
            index: Index,
            generation: Generation,

            const HandleSelf = @This();

            pub fn invalid() HandleSelf {
                return HandleSelf{ .index = 0xFFFF, .generation = 0xFFFF };
            }

            pub fn isValid(self: HandleSelf) bool {
                return self.index != 0xFFFF and self.generation != 0xFFFF;
            }

            pub fn eql(self: HandleSelf, other: HandleSelf) bool {
                return self.index == other.index and self.generation == other.generation;
            }
        };

        const Slot = struct {
            data: T,
            generation: Generation,
            is_alive: bool,

            pub fn init() @This() {
                return @This(){
                    .data = undefined,
                    .generation = 0,
                    .is_alive = false,
                };
            }
        };

        // Storage
        slots: [capacity]Slot,
        free_indices: [capacity]u16,
        free_count: u32,
        alive_count: u32,

        // Dense arrays for performance
        dense_data: [capacity]T,
        dense_handles: [capacity]Handle,
        dense_count: u32,
        handle_to_dense: [capacity]u32,

        pub fn init() Self {
            var self = Self{
                .slots = undefined,
                .free_indices = undefined,
                .free_count = capacity,
                .alive_count = 0,
                .dense_data = undefined,
                .dense_handles = undefined,
                .dense_count = 0,
                .handle_to_dense = undefined,
            };

            // Initialize slots and free list
            for (0..capacity) |i| {
                self.slots[i] = Slot.init();
                self.free_indices[i] = @intCast(i);
                self.handle_to_dense[i] = 0xFFFFFFFF;
            }

            return self;
        }

        pub fn spawn(self: *Self, data: T) Handle {
            if (self.free_count == 0) {
                return Handle.invalid();
            }

            // Get next free slot
            self.free_count -= 1;
            const index = self.free_indices[self.free_count];

            // Initialize slot
            var slot = &self.slots[index];
            slot.data = data;
            slot.generation += 1; // Increment generation to invalidate old handles
            slot.is_alive = true;

            self.alive_count += 1;

            return Handle{
                .index = index,
                .generation = slot.generation,
            };
        }

        pub fn destroy(self: *Self, handle: Handle) void {
            if (!handle.isValid()) return;

            const slot = &self.slots[handle.index];
            if (!slot.is_alive or slot.generation != handle.generation) return;

            // Mark as dead
            slot.is_alive = false;
            self.alive_count -= 1;

            // Add to free list
            self.free_indices[self.free_count] = handle.index;
            self.free_count += 1;
        }

        pub fn get(self: *const Self, handle: Handle) ?*const T {
            if (!handle.isValid()) return null;

            const slot = &self.slots[handle.index];
            if (!slot.is_alive or slot.generation != handle.generation) return null;

            return &slot.data;
        }

        pub fn getMut(self: *Self, handle: Handle) ?*T {
            if (!handle.isValid()) return null;

            const slot = &self.slots[handle.index];
            if (!slot.is_alive or slot.generation != handle.generation) return null;

            return &slot.data;
        }

        pub fn rebuildDenseArrays(self: *Self) void {
            self.dense_count = 0;

            // Initialize lookup table
            for (0..capacity) |i| {
                self.handle_to_dense[i] = 0xFFFFFFFF;
            }

            // Build dense arrays
            for (0..capacity) |i| {
                const slot = &self.slots[i];
                if (slot.is_alive) {
                    self.dense_data[self.dense_count] = slot.data;
                    self.dense_handles[self.dense_count] = Handle{ .index = @intCast(i), .generation = slot.generation };
                    self.handle_to_dense[i] = self.dense_count;
                    self.dense_count += 1;
                }
            }
        }

        pub fn writeDenseToSparse(self: *Self) void {
            for (0..self.dense_count) |i| {
                const handle = self.dense_handles[i];
                if (handle.index < capacity) {
                    const slot = &self.slots[handle.index];
                    if (slot.is_alive and slot.generation == handle.generation) {
                        slot.data = self.dense_data[i];
                    }
                }
            }
        }

        pub fn getDenseIndex(self: *const Self, handle: Handle) ?u32 {
            if (handle.index >= capacity) return null;
            const dense_idx = self.handle_to_dense[handle.index];
            if (dense_idx == 0xFFFFFFFF) return null;
            if (dense_idx >= self.dense_count) return null;

            // Verify generation matches
            if (self.dense_handles[dense_idx].generation != handle.generation) return null;

            return dense_idx;
        }

        pub fn getDenseData(self: *const Self) []const T {
            return self.dense_data[0..self.dense_count];
        }

        pub fn getDenseDataMut(self: *Self) []T {
            return self.dense_data[0..self.dense_count];
        }

        pub fn getDenseHandles(self: *const Self) []const Handle {
            return self.dense_handles[0..self.dense_count];
        }

        pub fn getAliveCount(self: *const Self) u32 {
            return self.alive_count;
        }

        pub fn getDenseCount(self: *const Self) u32 {
            return self.dense_count;
        }

        // Iterator for dense data (faster)
        pub fn forEachDense(self: *const Self, func: *const fn (Handle, *const T) void) void {
            for (0..self.dense_count) |i| {
                func(self.dense_handles[i], &self.dense_data[i]);
            }
        }
    };
}
