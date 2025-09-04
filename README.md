# Morphogenesis

A real-time particle simulation system exploring artificial life and morphogenesis, inspired by Michael Levin's work and Alan Turing's work on the chemical basis of morphogenesis.

## Features

- **Zig + WebAssembly**: Core simulation logic compiled from Zig to WASM
- **WebGPU Rendering**: Hardware-accelerated graphics with modern GPU APIs
- **Real-time Animation**: Smooth 60fps particle animations
- **Fullscreen Experience**: Immersive particle simulations

## Prerequisites

- **Zig 0.13.0+** - [Download from ziglang.org](https://ziglang.org/download/)
- **WebGPU-compatible browser** - Chrome, Firefox, or Safari
- **node** - For local development server

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

##  Testing & Best Practices

. **Test with browser MCP**: Use browser automation to capture performance
   - Navigate to `http://localhost:8000` 
   - Take screenshots to verify smooth animation
   - Monitor console for any errors
   - Observe flocking behavior quality



### Algorithm Optimization

The Boids system uses **spatial partitioning** for O(n) performance:
- 16x16 spatial grid for fast neighbor queries
- Each particle only checks surrounding 9 grid cells
- Dramatic performance improvement over naive O(n²) approach

### Buffer Allocation

The system uses a sophisticated multi-tier buffer allocation strategy implemented in Zig:

**Particle System:**
- **Grid Particles**: 5 grids × 144 particles each = 720 structured particles
- **Free Agents**: 1,000 boids/autonomous particles  
- **User Expansion**: 10,000 slots for runtime particle addition
- **Total Capacity**: ~11,720 particles with room for growth

**Memory Management:**
- **Generational Arena**: Safe handle-based particle references with O(1) spawn/destroy
- **Dense Arrays**: Optimized iteration using packed alive particles only
- **Spatial Grid**: Dynamic grid sizing with 500 particles per cell maximum
- **Spring Network**: ~20,000+ spring capacity supporting complex connectivity

**Performance Features:**
- Bulk data transfer for WebGL rendering
- Automatic memory defragmentation
- Handle validation prevents use-after-free bugs

## Goal

Build a web-based, high-performance simulation platform that layers interactions incrementally, starting with fundamental particle dynamics and evolving toward complex morphogenesis.