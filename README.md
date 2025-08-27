# Morphogenesis

A real-time particle simulation system exploring artificial life and morphogenesis, inspired by Michael Levin's work and Alan Turing's work on the chemical basis of morphogenesis.

## Features

- **Zig + WebAssembly**: Core simulation logic compiled from Zig to WASM
- **WebGPU Rendering**: Hardware-accelerated graphics with modern GPU APIs
- **Real-time Animation**: Smooth 60fps particle animations
- **Fullscreen Experience**: Immersive particle simulations

## Prerequisites

- **Zig 0.13.0+** - [Download from ziglang.org](https://ziglang.org/download/)
- **Python 3** - For local development server
- **WebGPU-compatible browser** - Chrome, Firefox, or Safari

## Quick Start

1. **Build the project**:
   ```bash
   ./build.sh
   ```

2. **Serve the demo**:
   ```bash
   python3 -m http.server 8000
   ```

3. **Open in browser**:
   ```
   http://localhost:8000
   ```

## Technical Stack

- **Zig 0.13.0** - Systems programming language
- **WebGPU** - Modern graphics API
- **WebAssembly** - High-performance web execution
- **WGSL Shaders** - WebGPU Shading Language

## Build System

- `build.zig` - Zig build configuration
- `build.sh` - Build helper script
- `src/main.zig` - Core Zig implementation

## Roadmap

**Current**: Animated triangle demo showcasing the complete WebGPU + Zig + WASM pipeline

**Phase 1**: Basic particle simulation with Boids-like flocking behavior
- Particle system with position, velocity, acceleration
- Simple rules: separation, alignment, cohesion
- WebGPU instanced rendering for performance

**Phase 2**: Spring connections and mesh topology
- Spring relationships forming dynamic mesh structures
- Particle repulsion and collision detection
- Morphogenic signal diffusion through connections

**Phase 3**: Growth and evolution mechanisms
- Particles can spawn new nodes using simple rules
- Neural networks for growth/connection decisions
- Complex emergent behaviors and shape formation

## Performance Testing & Best Practices

### Stress Testing Methodology

To find the optimal particle count for 60fps performance:

1. **Edit particle count**: Modify `PARTICLE_COUNT` in `src/main.zig`
   ```zig
   const PARTICLE_COUNT = 1000; // Adjust this value
   ```

2. **Rebuild**: Run `./build.sh` to compile changes

3. **Test with browser MCP**: Use browser automation to capture performance
   - Navigate to `http://localhost:8000` 
   - Take screenshots to verify smooth animation
   - Monitor console for any errors
   - Observe flocking behavior quality

4. **Performance benchmarks** (tested on M3 MacBook Pro):
   - ✅ **150 particles**: Baseline performance, excellent flocking
   - ✅ **500 particles**: Smooth 60fps, beautiful patterns  
   - ✅ **1000 particles**: Excellent performance, complex flocking
   - ✅ **2000 particles**: Still smooth, dense particle interactions
   - 🔄 **Higher counts**: Continue testing to find threshold

### Testing Best Practices

- **Keep it simple**: Use code comments and rebuilds rather than runtime controls
- **Browser MCP integration**: Perfect for automated testing and screenshots
- **Incremental testing**: Start low, increase by 500-1000 particles per test
- **Visual verification**: Screenshots show both performance and behavior quality
- **Git commits**: Document each test configuration for reproducibility

### Buffer Allocation

The JavaScript instance buffer is pre-allocated for 5000 particles. Increase this if testing higher counts:

```javascript
const maxParticles = 5000; // Adjust for higher particle counts
```

## Goal

Build a web-based, high-performance simulation platform that layers interactions incrementally, starting with fundamental particle dynamics and evolving toward complex morphogenesis.