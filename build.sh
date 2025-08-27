#!/bin/bash
echo "Building Zig WebGPU WASM Demo..."

# Build WASM module
./zig/zig build

# Copy to project root
cp zig-out/bin/webgpu-demo.wasm .

echo "✅ Build complete! Run 'python3 -m http.server 8000' and visit localhost:8000"