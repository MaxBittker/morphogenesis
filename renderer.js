// WebGPU Renderer for Morphogenesis Particle System
// Handles all rendering pipelines, shaders, and GPU operations

class MorphogenesisRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.device = null;
    this.context = null;
    
    // Render pipelines
    this.renderPipeline = null;
    this.gridRenderPipeline = null;
    this.springRenderPipeline = null;
    this.mouseSpringRenderPipeline = null;
    
    // Buffers
    this.vertexBuffer = null;
    this.gridVertexBuffer = null;
    this.springVertexBuffer = null;
    this.mouseSpringVertexBuffer = null;
    this.instanceBuffer = null;
    this.uniformBuffer = null;
    
    // Bind groups
    this.bindGroup = null;
    this.gridBindGroup = null;
    this.springBindGroup = null;
    this.mouseSpringBindGroup = null;
    
    // Rendering state
    this.gridLinesCount = 0;
    this.maxSprings = 0;
    this.maxParticles = 0;
    
    // Pre-allocated reusable buffers
    this.particlePositionsBuffer = null;
    this.springVerticesBuffer = null;
    this.aspectRatioBuffer = null;
    
    // WASM constants (fetched from Zig)
    this.worldSize = 1.95;
    this.gridSize = 25;
    
    // Cached data for performance
    this.cachedParticleData = null;
  }

  log(message) {
    console.log(`[Renderer] ${message}`);
  }

  async initWebGPU() {
    if (!navigator.gpu) {
      this.log("WebGPU not supported in this browser!");
      return false;
    }

    try {
      if (!this.device) {
        const adapter = await navigator.gpu.requestAdapter();
        if (!adapter) {
          this.log("Failed to get WebGPU adapter!");
          return false;
        }

        this.device = await adapter.requestDevice();
        this.log("WebGPU device created successfully");
      }
      
      this.context = this.canvas.getContext("webgpu");
      if (!this.context) {
        this.log("Failed to get WebGPU context!");
        return false;
      }

      const format = navigator.gpu.getPreferredCanvasFormat();
      this.context.configure({
        device: this.device,
        format: format,
      });
      
      this.log("WebGPU context configured successfully");
      return true;
    } catch (error) {
      this.log("WebGPU initialization error: " + error.message);
      return false;
    }
  }

  fetchConstantsFromWasm(wasmModule) {
    // Get constants from Zig to avoid duplication
    this.worldSize = wasmModule.exports.get_world_size();
    this.gridSize = wasmModule.exports.get_grid_size();
    this.maxParticles = wasmModule.exports.get_max_particles();
    this.maxSprings = wasmModule.exports.get_max_springs();
    this.particleSize = wasmModule.exports.get_particle_size();
    
    this.log(`Constants from WASM: world=${this.worldSize}, grid=${this.gridSize}, particles=${this.maxParticles}, springs=${this.maxSprings}, particleSize=${this.particleSize}`);
  }
  
  getCurrentWorldDimensions(wasmModule) {
    // Get current world dimensions (may have changed due to aspect ratio)
    if (wasmModule.exports.get_world_width && wasmModule.exports.get_world_height) {
      return {
        width: wasmModule.exports.get_world_width(),
        height: wasmModule.exports.get_world_height()
      };
    }
    // Fallback to square world
    return { width: this.worldSize, height: this.worldSize };
  }

  createShaders() {
    // Particle vertex shader
    const particleVertexShaderCode = `
      struct Uniforms {
          particle_size: f32,
          world_width: f32,
          world_height: f32,
      }
      @group(0) @binding(0) var<uniform> uniforms: Uniforms;

      struct VertexInput {
          @location(0) position: vec2<f32>,
          @location(1) particle_pos: vec2<f32>,
          @location(2) desired_valence: f32,
          @location(3) current_valence: f32,
      }

      struct VertexOutput {
          @builtin(position) position: vec4<f32>,
          @location(0) color: vec3<f32>,
          @location(1) uv: vec2<f32>,
      }

      @vertex
      fn main(input: VertexInput, @builtin(instance_index) instance_id: u32) -> VertexOutput {
          var output: VertexOutput;
          
          // Create square around particle position (in world pixels)
          let scaled_pos = input.position * uniforms.particle_size;
          let world_pos = scaled_pos + input.particle_pos;
          
          // Transform world pixel coordinates to NDC space
          let ndc_x = world_pos.x / (uniforms.world_width * 0.5);
          let ndc_y = world_pos.y / (uniforms.world_height * 0.5);
          
          output.position = vec4<f32>(ndc_x, ndc_y, 0.0, 1.0);
          output.uv = input.position;
          
          // Color based on desired valence - different colors for different bond counts
          let desired_valence = input.desired_valence;
          let current_valence = input.current_valence;
          var base_color: vec3<f32>;
          
          if (desired_valence == 0.0) {
              base_color = vec3<f32>(0.3, 0.3, 0.3); // Gray for inert (valence 0)
          } else if (desired_valence == 1.0) {
              base_color = vec3<f32>(1.0, 0.2, 0.2); // Red for valence 1
          } else if (desired_valence == 2.0) {
              base_color = vec3<f32>(0.2, 1.0, 0.2); // Green for valence 2
          } else if (desired_valence == 3.0) {
              base_color = vec3<f32>(0.2, 0.2, 1.0); // Blue for valence 3
          } else if (desired_valence == 4.0) {
              base_color = vec3<f32>(1.0, 1.0, 0.2); // Yellow for valence 4
          } else if (desired_valence == 5.0) {
              base_color = vec3<f32>(1.0, 0.2, 1.0); // Magenta for valence 5
          } else if (desired_valence == 6.0) {
              base_color = vec3<f32>(0.2, 1.0, 1.0); // Cyan for valence 6
          } else {
              // For valence > 6, use a rainbow based on valence
              let hue = (desired_valence - 6.0) * 0.5;
              base_color = vec3<f32>(
                  0.5 + 0.5 * sin(hue),
                  0.5 + 0.5 * sin(hue + 2.09439510),
                  0.5 + 0.5 * sin(hue + 4.18879020)
              );
          }
          
          // Dim particles when valence is satisfied (current >= desired)
          var brightness_factor: f32 = 1.0;
          if (current_valence >= desired_valence && desired_valence > 0.0) {
              brightness_factor = 0.6; // Dimmer when satisfied
          }
          
          output.color = base_color * brightness_factor;
          
          return output;
      }
    `;

    // Particle fragment shader
    const particleFragmentShaderCode = `
      @fragment
      fn main(@location(0) color: vec3<f32>, @location(1) uv: vec2<f32>) -> @location(0) vec4<f32> {
          let dist = length(uv);
          let circle = 1.0 - smoothstep(0.8, 1.0, dist);
          let glow = 1.0 - smoothstep(0.0, 0.6, dist);
          let intensity = circle * (0.7 + 0.3 * glow);
          
          if (circle < 0.01) {
              discard;
          }
          
          return vec4<f32>(color * intensity, circle);
      }
    `;

    // Grid visualization shader
    const gridVertexShaderCode = `
      struct Uniforms {
          particle_size: f32,
          world_width: f32,
          world_height: f32,
      }
      @group(0) @binding(0) var<uniform> uniforms: Uniforms;
      
      @vertex
      fn main(@location(0) position: vec2<f32>) -> @builtin(position) vec4<f32> {
          // Transform world pixel coordinates to NDC
          let ndc_x = position.x / (uniforms.world_width * 0.5);
          let ndc_y = position.y / (uniforms.world_height * 0.5);
          return vec4<f32>(ndc_x, ndc_y, 0.0, 1.0);
      }
    `;

    const gridFragmentShaderCode = `
      @fragment
      fn main() -> @location(0) vec4<f32> {
          return vec4<f32>(0.2, 0.4, 0.8, 0.15);
      }
    `;

    // Spring visualization shader
    const springVertexShaderCode = `
      struct Uniforms {
          particle_size: f32,
          world_width: f32,
          world_height: f32,
      }
      @group(0) @binding(0) var<uniform> uniforms: Uniforms;
      
      @vertex
      fn main(@location(0) position: vec2<f32>) -> @builtin(position) vec4<f32> {
          // Transform world pixel coordinates to NDC
          let ndc_x = position.x / (uniforms.world_width * 0.5);
          let ndc_y = position.y / (uniforms.world_height * 0.5);
          return vec4<f32>(ndc_x, ndc_y, 0.0, 1.0);
      }
    `;

    const springFragmentShaderCode = `
      @fragment
      fn main() -> @location(0) vec4<f32> {
          return vec4<f32>(1.0, 1.0, 1.0, 0.9); // Bright white, very opaque
      }
    `;

    // Mouse spring visualization shader (bright, prominent color)
    const mouseSpringFragmentShaderCode = `
      @fragment
      fn main() -> @location(0) vec4<f32> {
          return vec4<f32>(1.0, 0.2, 0.2, 0.8); // Bright red, more opaque
      }
    `;

    return {
      particleVertex: particleVertexShaderCode,
      particleFragment: particleFragmentShaderCode,
      gridVertex: gridVertexShaderCode,
      gridFragment: gridFragmentShaderCode,
      springVertex: springVertexShaderCode,
      springFragment: springFragmentShaderCode,
      mouseSpringFragment: mouseSpringFragmentShaderCode
    };
  }

  async createRenderPipelines() {
    const shaders = this.createShaders();
    
    // Create shader modules
    const particleVertexModule = this.device.createShaderModule({ code: shaders.particleVertex });
    const particleFragmentModule = this.device.createShaderModule({ code: shaders.particleFragment });
    const gridVertexModule = this.device.createShaderModule({ code: shaders.gridVertex });
    const gridFragmentModule = this.device.createShaderModule({ code: shaders.gridFragment });
    const springVertexModule = this.device.createShaderModule({ code: shaders.springVertex });
    const springFragmentModule = this.device.createShaderModule({ code: shaders.springFragment });
    const mouseSpringFragmentModule = this.device.createShaderModule({ code: shaders.mouseSpringFragment });

    // Create bind group layout
    const bindGroupLayout = this.device.createBindGroupLayout({
      entries: [{
        binding: 0,
        visibility: GPUShaderStage.VERTEX,
        buffer: { type: "uniform" },
      }],
    });

    // Create particle render pipeline
    this.renderPipeline = this.device.createRenderPipeline({
      layout: this.device.createPipelineLayout({ bindGroupLayouts: [bindGroupLayout] }),
      vertex: {
        module: particleVertexModule,
        entryPoint: "main",
        buffers: [
          {
            arrayStride: 2 * 4,
            stepMode: "vertex",
            attributes: [{ format: "float32x2", offset: 0, shaderLocation: 0 }],
          },
          {
            arrayStride: 4 * 4,
            stepMode: "instance", 
            attributes: [
              { format: "float32x2", offset: 0, shaderLocation: 1 }, // position
              { format: "float32", offset: 8, shaderLocation: 2 }, // desired_valence
              { format: "float32", offset: 12, shaderLocation: 3 }, // current_valence
            ],
          },
        ],
      },
      fragment: {
        module: particleFragmentModule,
        entryPoint: "main",
        targets: [{
          format: navigator.gpu.getPreferredCanvasFormat(),
          blend: {
            color: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
            alpha: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
          },
        }],
      },
      primitive: { topology: "triangle-list" },
    });

    // Create grid render pipeline
    this.gridRenderPipeline = this.device.createRenderPipeline({
      layout: this.device.createPipelineLayout({ bindGroupLayouts: [bindGroupLayout] }),
      vertex: {
        module: gridVertexModule,
        entryPoint: "main",
        buffers: [{
          arrayStride: 2 * 4,
          stepMode: "vertex",
          attributes: [{ format: "float32x2", offset: 0, shaderLocation: 0 }],
        }],
      },
      fragment: {
        module: gridFragmentModule,
        entryPoint: "main",
        targets: [{
          format: navigator.gpu.getPreferredCanvasFormat(),
          blend: {
            color: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
            alpha: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
          },
        }],
      },
      primitive: { topology: "line-list" },
    });

    // Create spring render pipeline
    this.springRenderPipeline = this.device.createRenderPipeline({
      layout: this.device.createPipelineLayout({ bindGroupLayouts: [bindGroupLayout] }),
      vertex: {
        module: springVertexModule,
        entryPoint: "main",
        buffers: [{
          arrayStride: 2 * 4,
          stepMode: "vertex", 
          attributes: [{ format: "float32x2", offset: 0, shaderLocation: 0 }],
        }],
      },
      fragment: {
        module: springFragmentModule,
        entryPoint: "main",
        targets: [{
          format: navigator.gpu.getPreferredCanvasFormat(),
          blend: {
            color: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
            alpha: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
          },
        }],
      },
      primitive: { topology: "line-list" },
    });

    // Create mouse spring render pipeline (same as spring but with different color)
    this.mouseSpringRenderPipeline = this.device.createRenderPipeline({
      layout: this.device.createPipelineLayout({ bindGroupLayouts: [bindGroupLayout] }),
      vertex: {
        module: springVertexModule,
        entryPoint: "main",
        buffers: [{
          arrayStride: 2 * 4,
          stepMode: "vertex", 
          attributes: [{ format: "float32x2", offset: 0, shaderLocation: 0 }],
        }],
      },
      fragment: {
        module: mouseSpringFragmentModule,
        entryPoint: "main",
        targets: [{
          format: navigator.gpu.getPreferredCanvasFormat(),
          blend: {
            color: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
            alpha: { srcFactor: "src-alpha", dstFactor: "one-minus-src-alpha", operation: "add" },
          },
        }],
      },
      primitive: { topology: "line-list" },
    });

    this.log("Render pipelines created successfully");
    return bindGroupLayout;
  }

  createBuffers(bindGroupLayout) {
    // Particle quad vertices
    const quadVertices = new Float32Array([
      -1.0, -1.0, 1.0, -1.0, -1.0, 1.0,  // First triangle
      1.0, -1.0, 1.0, 1.0, -1.0, 1.0     // Second triangle  
    ]);

    this.vertexBuffer = this.device.createBuffer({
      size: quadVertices.byteLength,
      usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
    });
    this.device.queue.writeBuffer(this.vertexBuffer, 0, quadVertices);

    // Instance buffer for particle positions + valences (4 floats per particle: x, y, desired_valence, current_valence)
    this.instanceBuffer = this.device.createBuffer({
      size: this.maxParticles * 4 * 4,
      usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
    });

    // Create grid buffer (will be updated dynamically based on world dimensions)
    const maxGridLines = (this.gridSize + 1) * 2;
    this.gridVertexBuffer = this.device.createBuffer({
      size: maxGridLines * 4 * 4, // Extra space for aspect ratio variations
      usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
    });
    this.gridLinesCount = 0; // Will be set when grid is updated

    // Spring vertex buffer
    this.springVertexBuffer = this.device.createBuffer({
      size: this.maxSprings * 4 * 4,
      usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
    });

    // Mouse spring vertex buffer (for single mouse connection)
    this.mouseSpringVertexBuffer = this.device.createBuffer({
      size: 4 * 4, // One line (2 vertices * 2 coordinates * 4 bytes)
      usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
    });

    // Uniform buffer for shader uniforms
    this.uniformBuffer = this.device.createBuffer({
      size: 12, // 4 bytes each for particle_size, world_width, world_height
      usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
    });

    // Create bind groups
    this.bindGroup = this.device.createBindGroup({
      layout: bindGroupLayout,
      entries: [{ binding: 0, resource: { buffer: this.uniformBuffer } }],
    });

    this.gridBindGroup = this.device.createBindGroup({
      layout: bindGroupLayout,
      entries: [{ binding: 0, resource: { buffer: this.uniformBuffer } }],
    });

    this.springBindGroup = this.device.createBindGroup({
      layout: bindGroupLayout,
      entries: [{ binding: 0, resource: { buffer: this.uniformBuffer } }],
    });

    this.mouseSpringBindGroup = this.device.createBindGroup({
      layout: bindGroupLayout,
      entries: [{ binding: 0, resource: { buffer: this.uniformBuffer } }],
    });

    // Pre-allocate reusable TypedArray buffers (x, y, desired_valence, current_valence per particle)
    this.particlePositionsBuffer = new Float32Array(this.maxParticles * 4);
    this.springVerticesBuffer = new Float32Array(this.maxSprings * 4);
    this.mouseSpringVerticesBuffer = new Float32Array(4); // One line: x1, y1, x2, y2
    this.uniformsBuffer = new Float32Array(3); // particle_size, world_width, world_height

    this.log("Buffers created successfully");
  }

  async initialize(wasmModule) {
    this.log("Initializing renderer...");
    
    // Fetch constants from WASM first
    this.fetchConstantsFromWasm(wasmModule);
    
    // Initialize WebGPU
    if (!await this.initWebGPU()) return false;

    // Enable error reporting
    this.device.addEventListener("uncapturederror", (event) => {
      console.error("WebGPU uncaptured error:", event.error);
    });

    // Create render pipelines
    const bindGroupLayout = await this.createRenderPipelines();
    
    // Create buffers
    this.createBuffers(bindGroupLayout);
    
    this.log("Renderer initialized successfully");
    return true;
  }

  updateGridLines(wasmModule) {
    // Generate grid lines in world space coordinates
    const worldDims = this.getCurrentWorldDimensions(wasmModule);
    const worldHalfX = worldDims.width / 2.0;
    const worldHalfY = worldDims.height / 2.0;
    
    const totalGridLines = (this.gridSize + 1) * 2;
    const gridLinesArray = new Float32Array(totalGridLines * 4);
    let gridIndex = 0;
    
    // Vertical lines (full height of world space)
    for (let i = 0; i <= this.gridSize; i++) {
      const x = (i / this.gridSize) * worldDims.width - worldHalfX;
      gridLinesArray[gridIndex++] = x;
      gridLinesArray[gridIndex++] = -worldHalfY; // Bottom of world space
      gridLinesArray[gridIndex++] = x;
      gridLinesArray[gridIndex++] = worldHalfY;  // Top of world space
    }
    
    // Horizontal lines (full width of world space)
    for (let i = 0; i <= this.gridSize; i++) {
      const y = (i / this.gridSize) * worldDims.height - worldHalfY;
      gridLinesArray[gridIndex++] = -worldHalfX;
      gridLinesArray[gridIndex++] = y;
      gridLinesArray[gridIndex++] = worldHalfX;
      gridLinesArray[gridIndex++] = y;
    }
    
    this.device.queue.writeBuffer(this.gridVertexBuffer, 0, gridLinesArray);
    this.gridLinesCount = totalGridLines;
  }

  updateData(wasmModule) {
    // Update grid lines to match current world dimensions
    this.updateGridLines(wasmModule);
    
    // Update uniforms (particle size and world dimensions)
    const worldDims = this.getCurrentWorldDimensions(wasmModule);
    this.uniformsBuffer[0] = this.particleSize;
    this.uniformsBuffer[1] = worldDims.width;
    this.uniformsBuffer[2] = worldDims.height;
    this.device.queue.writeBuffer(this.uniformBuffer, 0, this.uniformsBuffer);

    // BULK DATA TRANSFER: Get all particle data at once
    const particleCount = wasmModule.exports.get_bulk_particle_count();
    if (particleCount > 0) {
      // Get direct pointer to WASM memory buffer - no copying in JS!
      const particleDataPtr = wasmModule.exports.get_particle_data_bulk();
      const particleDataView = new Float32Array(
        wasmModule.exports.memory.buffer,
        particleDataPtr,
        particleCount * 4
      );
      
      // Direct upload to GPU - single memcpy instead of thousands of function calls
      this.device.queue.writeBuffer(this.instanceBuffer, 0, particleDataView);
      
      // Store reference for mouse spring lookup
      this.cachedParticleData = particleDataView;
    }

    // BULK DATA TRANSFER: Get all spring data at once
    const springCount = wasmModule.exports.get_bulk_spring_count();
    if (springCount > 0) {
      // Get direct pointer to WASM memory buffer - no copying in JS!
      const springDataPtr = wasmModule.exports.get_spring_data_bulk();
      const springDataView = new Float32Array(
        wasmModule.exports.memory.buffer,
        springDataPtr,
        springCount * 4
      );
      
      // Direct upload to GPU - single memcpy instead of loop
      this.device.queue.writeBuffer(this.springVertexBuffer, 0, springDataView);
    }

    // Get mouse spring data if exists (this is minimal, so keep individual calls)
    let hasMouseSpring = false;
    if (wasmModule.exports.get_mouse_connected_particle) {
      const mouseParticle = wasmModule.exports.get_mouse_connected_particle();
      if (mouseParticle >= 0 && this.cachedParticleData) {
        const mouseX = wasmModule.exports.get_mouse_position_x();
        const mouseY = wasmModule.exports.get_mouse_position_y();
        
        // Get connected particle position from cached data
        const particleX = this.cachedParticleData[mouseParticle * 4];
        const particleY = this.cachedParticleData[mouseParticle * 4 + 1];
        
        // Create mouse spring line
        this.mouseSpringVerticesBuffer[0] = particleX;
        this.mouseSpringVerticesBuffer[1] = particleY;
        this.mouseSpringVerticesBuffer[2] = mouseX;
        this.mouseSpringVerticesBuffer[3] = mouseY;
        
        this.device.queue.writeBuffer(this.mouseSpringVertexBuffer, 0, this.mouseSpringVerticesBuffer);
        hasMouseSpring = true;
      }
    }

    return { particleCount, springCount, hasMouseSpring };
  }

  render(wasmModule) {
    const { particleCount, springCount, hasMouseSpring } = this.updateData(wasmModule);

    try {
      const commandEncoder = this.device.createCommandEncoder();
      const textureView = this.context.getCurrentTexture().createView();

      const renderPassDescriptor = {
        colorAttachments: [{
          view: textureView,
          clearValue: { r: 0.1, g: 0.1, b: 0.1, a: 1.0 },
          loadOp: "clear",
          storeOp: "store",
        }],
      };

      const passEncoder = commandEncoder.beginRenderPass(renderPassDescriptor);
      
      // Draw spatial grid
      passEncoder.setPipeline(this.gridRenderPipeline);
      passEncoder.setBindGroup(0, this.gridBindGroup);
      passEncoder.setVertexBuffer(0, this.gridVertexBuffer);
      passEncoder.draw(this.gridLinesCount, 1, 0, 0);
      
      // Draw spring connections
      if (springCount > 0) {
        passEncoder.setPipeline(this.springRenderPipeline);
        passEncoder.setBindGroup(0, this.springBindGroup);
        passEncoder.setVertexBuffer(0, this.springVertexBuffer);
        passEncoder.draw(springCount * 2, 1, 0, 0);
      }
      
      // Draw mouse spring connection (on top of regular springs)
      if (hasMouseSpring) {
        passEncoder.setPipeline(this.mouseSpringRenderPipeline);
        passEncoder.setBindGroup(0, this.mouseSpringBindGroup);
        passEncoder.setVertexBuffer(0, this.mouseSpringVertexBuffer);
        passEncoder.draw(2, 1, 0, 0); // One line = 2 vertices
      }
      
      // Draw particles
      passEncoder.setPipeline(this.renderPipeline);
      passEncoder.setBindGroup(0, this.bindGroup);
      passEncoder.setVertexBuffer(0, this.vertexBuffer);
      passEncoder.setVertexBuffer(1, this.instanceBuffer);
      passEncoder.draw(6, particleCount, 0, 0);
      
      passEncoder.end();
      this.device.queue.submit([commandEncoder.finish()]);
      
    } catch (error) {
      this.log("Render error: " + error.message);
      console.error("Full render error:", error);
    }
  }
}

// Export the renderer class
window.MorphogenesisRenderer = MorphogenesisRenderer;