# Morphogenesis

A real-time particle simulation system exploring artificial life and morphogenesis, inspired by Michael Levin's work and Alan Turing's work on the chemical basis of morphogenesis.

## Features

- **Zig + WebAssembly**: Core simulation logic compiled from Zig to WASM
- **WebGPU Rendering**: Hardware-accelerated graphics with modern GPU APIs
- **Real-time Animation**: Smooth 60fps particle animations
- **Fullscreen Experience**: Immersive particle simulations

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

## Goal

Build a web-based, high-performance simulation platform that layers interactions incrementally, starting with fundamental particle dynamics and evolving toward complex morphogenesis.