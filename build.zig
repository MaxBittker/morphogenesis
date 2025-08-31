const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.resolveTargetQuery(.{
        .cpu_arch = .wasm32,
        .os_tag = .freestanding,
    });

    const optimize = b.standardOptimizeOption(.{});

    const exe = b.addExecutable(.{
        .name = "webgpu-demo",
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.root_source_file = b.path("src/main.zig");

    // No entry point for WASM library
    exe.entry = .disabled;
    
    // Export functions for WebAssembly
    exe.rdynamic = true;

    b.installArtifact(exe);
}