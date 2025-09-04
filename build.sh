#!/bin/bash
echo "Building Zig WebGPU WASM Demo..."

# Default to ReleaseFast if no optimization level specified
OPTIMIZE=${1:-ReleaseFast}

# Use Zig 0.14 for compatibility
ZIG_PATH="/opt/homebrew/opt/zig@0.14/bin/zig"
if [ -f "$ZIG_PATH" ]; then
    ZIG_CMD="$ZIG_PATH"
elif command -v zig &> /dev/null; then
    ZIG_CMD="zig"
else
    echo "❌ Zig not found! Please install Zig 0.14.1 or compatible version"
    echo "   Run: brew install zig@0.14"
    exit 1
fi

echo "Using Zig: $($ZIG_CMD version)"

# Build WASM module with optimizations
echo "Using optimization level: $OPTIMIZE"
$ZIG_CMD build -Doptimize=$OPTIMIZE

# Copy to project root
cp zig-out/bin/webgpu-demo.wasm .

echo "✅ Build complete! "