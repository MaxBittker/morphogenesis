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

  createShaders() {
    // Particle vertex shader
    const particleVertexShaderCode = `
      struct Uniforms {
          aspect_ratio: f32,
          particle_size: f32,
      }
      @group(0) @binding(0) var<uniform> uniforms: Uniforms;

      struct VertexInput {
          @location(0) position: vec2<f32>,
          @location(1) particle_pos: vec2<f32>,
      }

      struct VertexOutput {
          @builtin(position) position: vec4<f32>,
          @location(0) color: vec3<f32>,
          @location(1) uv: vec2<f32>,
      }

      @vertex
      fn main(input: VertexInput, @builtin(instance_index) instance_id: u32) -> VertexOutput {
          var output: VertexOutput;
          
          // Create square around particle position for circle rendering
          var scaled_pos = input.position * uniforms.particle_size;
          
          // Adjust for aspect ratio to keep circles circular
          if (uniforms.aspect_ratio > 1.0) {
              scaled_pos.x = scaled_pos.x / uniforms.aspect_ratio;
          } else {
              scaled_pos.y = scaled_pos.y * uniforms.aspect_ratio;
          }
          
          let world_pos = scaled_pos + input.particle_pos;
          output.position = vec4<f32>(world_pos, 0.0, 1.0);
          output.uv = input.position;
          
          // Color based on particle index for stable identity-based colors
          let hue = f32(instance_id) * 0.01745329;
          output.color = vec3<f32>(
              0.5 + 0.5 * sin(hue),
              0.5 + 0.5 * sin(hue + 2.09439510),
              0.5 + 0.5 * sin(hue + 4.18879020)
          );
          
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
          aspect_ratio: f32,
      }
      @group(0) @binding(0) var<uniform> uniforms: Uniforms;
      
      @vertex
      fn main(@location(0) position: vec2<f32>) -> @builtin(position) vec4<f32> {
          return vec4<f32>(position, 0.0, 1.0);
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
          aspect_ratio: f32,
      }
      @group(0) @binding(0) var<uniform> uniforms: Uniforms;
      
      @vertex
      fn main(@location(0) position: vec2<f32>) -> @builtin(position) vec4<f32> {
          return vec4<f32>(position, 0.0, 1.0);
      }
    `;

    const springFragmentShaderCode = `
      @fragment
      fn main() -> @location(0) vec4<f32> {
          return vec4<f32>(0.8, 0.6, 0.2, 0.3);
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
            arrayStride: 2 * 4,
            stepMode: "instance", 
            attributes: [{ format: "float32x2", offset: 0, shaderLocation: 1 }],
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

    // Instance buffer for particle positions
    this.instanceBuffer = this.device.createBuffer({
      size: this.maxParticles * 2 * 4,
      usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
    });

    // Create grid lines
    const worldHalf = this.worldSize / 2.0;
    const totalGridLines = (this.gridSize + 1) * 2;
    const gridLinesArray = new Float32Array(totalGridLines * 4);
    let gridIndex = 0;
    
    // Vertical lines
    for (let i = 0; i <= this.gridSize; i++) {
      const x = (i / this.gridSize) * this.worldSize - worldHalf;
      gridLinesArray[gridIndex++] = x;
      gridLinesArray[gridIndex++] = -worldHalf;
      gridLinesArray[gridIndex++] = x;
      gridLinesArray[gridIndex++] = worldHalf;
    }
    
    // Horizontal lines
    for (let i = 0; i <= this.gridSize; i++) {
      const y = (i / this.gridSize) * this.worldSize - worldHalf;
      gridLinesArray[gridIndex++] = -worldHalf;
      gridLinesArray[gridIndex++] = y;
      gridLinesArray[gridIndex++] = worldHalf;
      gridLinesArray[gridIndex++] = y;
    }
    
    this.gridVertexBuffer = this.device.createBuffer({
      size: gridLinesArray.byteLength,
      usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
    });
    this.device.queue.writeBuffer(this.gridVertexBuffer, 0, gridLinesArray);
    this.gridLinesCount = totalGridLines;

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

    // Uniform buffer for aspect ratio and particle size
    this.uniformBuffer = this.device.createBuffer({
      size: 8, // 4 bytes for aspect_ratio + 4 bytes for particle_size
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

    // Pre-allocate reusable TypedArray buffers
    this.particlePositionsBuffer = new Float32Array(this.maxParticles * 2);
    this.springVerticesBuffer = new Float32Array(this.maxSprings * 4);
    this.mouseSpringVerticesBuffer = new Float32Array(4); // One line: x1, y1, x2, y2
    this.uniformsBuffer = new Float32Array(2); // aspect_ratio, particle_size

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

  updateData(wasmModule) {
    // Update uniforms (aspect ratio and particle size)
    const aspectRatio = window.aspectRatio || this.canvas.width / this.canvas.height;
    this.uniformsBuffer[0] = aspectRatio;
    this.uniformsBuffer[1] = this.particleSize;
    this.device.queue.writeBuffer(this.uniformBuffer, 0, this.uniformsBuffer);

    // Get particle data
    const particleCount = wasmModule.exports.get_particle_count();
    for (let i = 0; i < particleCount; i++) {
      this.particlePositionsBuffer[i * 2] = wasmModule.exports.get_particle_data(i * 2);
      this.particlePositionsBuffer[i * 2 + 1] = wasmModule.exports.get_particle_data(i * 2 + 1);
    }
    this.device.queue.writeBuffer(this.instanceBuffer, 0, this.particlePositionsBuffer, 0, particleCount * 2);

    // Get spring data
    const springCount = wasmModule.exports.get_spring_count ? wasmModule.exports.get_spring_count() : 0;
    if (springCount > 0) {
      for (let i = 0; i < springCount; i++) {
        const particleA = wasmModule.exports.get_spring_particle_a(i);
        const particleB = wasmModule.exports.get_spring_particle_b(i);
        
        // Reuse already fetched particle data
        const ax = this.particlePositionsBuffer[particleA * 2];
        const ay = this.particlePositionsBuffer[particleA * 2 + 1];
        const bx = this.particlePositionsBuffer[particleB * 2];
        const by = this.particlePositionsBuffer[particleB * 2 + 1];
        
        this.springVerticesBuffer[i * 4] = ax;
        this.springVerticesBuffer[i * 4 + 1] = ay;
        this.springVerticesBuffer[i * 4 + 2] = bx;
        this.springVerticesBuffer[i * 4 + 3] = by;
      }
      this.device.queue.writeBuffer(this.springVertexBuffer, 0, this.springVerticesBuffer, 0, springCount * 4);
    }

    // Get mouse spring data if exists
    let hasMouseSpring = false;
    if (wasmModule.exports.get_mouse_connected_particle) {
      const mouseParticle = wasmModule.exports.get_mouse_connected_particle();
      if (mouseParticle >= 0) {
        const mouseX = wasmModule.exports.get_mouse_position_x();
        const mouseY = wasmModule.exports.get_mouse_position_y();
        
        // Get connected particle position
        const particleX = this.particlePositionsBuffer[mouseParticle * 2];
        const particleY = this.particlePositionsBuffer[mouseParticle * 2 + 1];
        
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