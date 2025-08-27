const std = @import("std");

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

// Triangle vertices in normalized device coordinates
const vertices = [_]f32{
    // x,    y,   z,   r,   g,   b
     0.0,  0.8, 0.0, 1.0, 0.0, 0.0, // top - red
    -0.8, -0.8, 0.0, 0.0, 1.0, 0.0, // bottom left - green  
     0.8, -0.8, 0.0, 0.0, 0.0, 1.0, // bottom right - blue
};

var device_handle: u32 = 0;

export fn init() void {
    log("Initializing WebGPU demo...", .{});
    device_handle = emscripten_webgpu_get_device();
    log("WebGPU device initialized: {}", .{device_handle});
}

export fn render() void {
    if (device_handle == 0) {
        log("Device not initialized!", .{});
        return;
    }
    
    // Create render pass
    const render_pass = emscripten_webgpu_create_render_pass_encoder(device_handle, 0, 0);
    
    // Draw triangle
    emscripten_webgpu_render_pass_draw(render_pass, 3);
    
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