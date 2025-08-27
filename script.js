const canvas = document.getElementById("webgpu-canvas");
let wasmModule = null;
let device = null;
let context = null;
let renderPipeline = null;
let vertexBuffer = null;
let instanceBuffer = null;
let uniformBuffer = null;
let bindGroup = null;
let animationId = null;
let time = 0;
let updateTimeMs = 0;
const timingDisplay = document.getElementById("timing-display");

function log(message) {
  console.log(message);
}

// Set canvas size to match viewport
function resizeCanvas() {
  canvas.width = window.innerWidth * devicePixelRatio;
  canvas.height = window.innerHeight * devicePixelRatio;
  canvas.style.width = window.innerWidth + "px";
  canvas.style.height = window.innerHeight + "px";

  // Store aspect ratio for coordinate transformation
  window.aspectRatio = canvas.width / canvas.height;
}

window.addEventListener("resize", resizeCanvas);
resizeCanvas();

async function initWebGPU() {
  if (!navigator.gpu) {
    log("WebGPU not supported in this browser!");
    return false;
  }

  try {
    // Only create device if not already created
    if (!device) {
      const adapter = await navigator.gpu.requestAdapter();
      if (!adapter) {
        log("Failed to get WebGPU adapter!");
        return false;
      }

      device = await adapter.requestDevice();
      log("WebGPU device created successfully");
    }
    context = canvas.getContext("webgpu");

    if (!context) {
      log("Failed to get WebGPU context!");
      return false;
    }

    const format = navigator.gpu.getPreferredCanvasFormat();
    context.configure({
      device: device,
      format: format,
    });
    log("WebGPU context configured successfully");

    await createRenderPipeline();
    log("WebGPU initialized successfully with render pipeline");
    return true;
  } catch (error) {
    log("WebGPU initialization error: " + error.message);
    return false;
  }
}

async function createRenderPipeline() {
  // Instanced particle vertex shader
  const vertexShaderCode = `
                struct Uniforms {
                    aspect_ratio: f32,
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
                    
                    // Create square around particle position for circle rendering - half size
                    let particle_size = 0.0125; // Half the previous size (0.025 / 2)
                    var scaled_pos = input.position * particle_size;
                    
                    // Adjust for aspect ratio to keep circles circular
                    if (uniforms.aspect_ratio > 1.0) {
                        // Wide screen: compress X
                        scaled_pos.x = scaled_pos.x / uniforms.aspect_ratio;
                    } else {
                        // Tall screen: compress Y  
                        scaled_pos.y = scaled_pos.y * uniforms.aspect_ratio;
                    }
                    
                    let world_pos = scaled_pos + input.particle_pos;
                    output.position = vec4<f32>(world_pos, 0.0, 1.0);
                    output.uv = input.position; // Use quad position as UV for circle calculation
                    
                    // Color based on particle index for stable identity-based colors
                    let hue = f32(instance_id) * 0.01745329; // Convert index to hue (1 degree per particle)
                    output.color = vec3<f32>(
                        0.5 + 0.5 * sin(hue),
                        0.5 + 0.5 * sin(hue + 2.09439510),   // 120 degrees offset
                        0.5 + 0.5 * sin(hue + 4.18879020)    // 240 degrees offset
                    );
                    
                    return output;
                }
            `;

  // Fragment shader with circle rendering
  const fragmentShaderCode = `
                @fragment
                fn main(@location(0) color: vec3<f32>, @location(1) uv: vec2<f32>) -> @location(0) vec4<f32> {
                    // Calculate distance from center of quad
                    let dist = length(uv);
                    
                    // Create circle with smooth edges
                    let circle = 1.0 - smoothstep(0.8, 1.0, dist);
                    
                    // Add inner glow effect
                    let glow = 1.0 - smoothstep(0.0, 0.6, dist);
                    let intensity = circle * (0.7 + 0.3 * glow);
                    
                    // Apply alpha based on circle shape
                    if (circle < 0.01) {
                        discard;
                    }
                    
                    return vec4<f32>(color * intensity, circle);
                }
            `;

  const vertexShaderModule = device.createShaderModule({
    code: vertexShaderCode,
  });

  const fragmentShaderModule = device.createShaderModule({
    code: fragmentShaderCode,
  });

  // Base quad vertices (will be instanced) for circle rendering
  const quadVertices = new Float32Array([
    // First triangle
    -1.0,
    -1.0, // bottom left
    1.0,
    -1.0, // bottom right
    -1.0,
    1.0, // top left

    // Second triangle
    1.0,
    -1.0, // bottom right
    1.0,
    1.0, // top right
    -1.0,
    1.0, // top left
  ]);

  vertexBuffer = device.createBuffer({
    size: quadVertices.byteLength,
    usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
  });

  device.queue.writeBuffer(vertexBuffer, 0, quadVertices);

  // Instance buffer for particle positions (will be updated each frame)
  // Allocate for stress testing - adjust based on PARTICLE_COUNT in main.zig
  const maxParticles = 6000; // Should be >= PARTICLE_COUNT in main.zig
  instanceBuffer = device.createBuffer({
    size: maxParticles * 2 * 4, // 2 floats per particle (x, y)
    usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
  });
  log(`Created instance buffer for up to ${maxParticles} particles`);

  // Uniform buffer for aspect ratio
  uniformBuffer = device.createBuffer({
    size: 4, // 1 f32 for aspect ratio
    usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
  });

  // Create bind group layout
  const bindGroupLayout = device.createBindGroupLayout({
    entries: [
      {
        binding: 0,
        visibility: GPUShaderStage.VERTEX,
        buffer: { type: "uniform" },
      },
    ],
  });

  renderPipeline = device.createRenderPipeline({
    layout: device.createPipelineLayout({
      bindGroupLayouts: [bindGroupLayout],
    }),
    vertex: {
      module: vertexShaderModule,
      entryPoint: "main",
      buffers: [
        {
          arrayStride: 2 * 4, // 2 floats
          stepMode: "vertex",
          attributes: [
            {
              format: "float32x2",
              offset: 0,
              shaderLocation: 0, // position
            },
          ],
        },
        {
          arrayStride: 2 * 4, // 2 floats
          stepMode: "instance",
          attributes: [
            {
              format: "float32x2",
              offset: 0,
              shaderLocation: 1, // particle_pos
            },
          ],
        },
      ],
    },
    fragment: {
      module: fragmentShaderModule,
      entryPoint: "main",
      targets: [
        {
          format: navigator.gpu.getPreferredCanvasFormat(),
          blend: {
            color: {
              srcFactor: "src-alpha",
              dstFactor: "one-minus-src-alpha",
              operation: "add",
            },
            alpha: {
              srcFactor: "src-alpha",
              dstFactor: "one-minus-src-alpha",
              operation: "add",
            },
          },
        },
      ],
    },
    primitive: {
      topology: "triangle-list",
    },
  });

  // Create bind group
  bindGroup = device.createBindGroup({
    layout: bindGroupLayout,
    entries: [
      {
        binding: 0,
        resource: { buffer: uniformBuffer },
      },
    ],
  });

  log("Particle render pipeline created");
}

// Load and instantiate WASM module
async function loadWasm() {
  try {
    // Environment for WASM module
    const env = {
      console_log: (ptr, len) => {
        if (wasmModule && wasmModule.exports.memory) {
          const memory = wasmModule.exports.memory;
          const buffer = new Uint8Array(memory.buffer, ptr, len);
          const message = new TextDecoder().decode(buffer);
          log("[WASM] " + message);
        }
      },
      emscripten_webgpu_get_device: () => (device ? 1 : 0),
    };

    const wasmResponse = await fetch("webgpu-demo.wasm");
    const wasmBytes = await wasmResponse.arrayBuffer();
    const wasmObj = await WebAssembly.instantiate(wasmBytes, {
      env: env,
    });

    wasmModule = wasmObj.instance;
    log("WASM module loaded successfully");

    // Initialize the WASM module
    wasmModule.exports.init();
    return true;
  } catch (error) {
    log("WASM loading error: " + error.message);
    return false;
  }
}

// Render frame with Boids particles
function renderFrame() {
  if (
    !device ||
    !context ||
    !renderPipeline ||
    !vertexBuffer ||
    !instanceBuffer ||
    !uniformBuffer ||
    !bindGroup ||
    !wasmModule
  ) {
    return;
  }

  time += 0.016; // ~60fps timing

  // Update aspect ratio uniform
  const aspectRatio = window.aspectRatio || canvas.width / canvas.height;
  device.queue.writeBuffer(
    uniformBuffer,
    0,
    new Float32Array([aspectRatio])
  );

  // Time the particle update loop
  const updateStart = performance.now();
  wasmModule.exports.update_particles(0.016);
  const updateEnd = performance.now();
  updateTimeMs = Math.round(updateEnd - updateStart);

  // Update timing display
  timingDisplay.textContent = `Update: ${updateTimeMs}ms`;

  // Get particle data from WASM
  const particleCount = wasmModule.exports.get_particle_count();
  const particlePositions = new Float32Array(particleCount * 2);

  for (let i = 0; i < particleCount; i++) {
    particlePositions[i * 2] = wasmModule.exports.get_particle_data(
      i * 2
    ); // x
    particlePositions[i * 2 + 1] = wasmModule.exports.get_particle_data(
      i * 2 + 1
    ); // y
  }

  // Update instance buffer with particle positions
  device.queue.writeBuffer(instanceBuffer, 0, particlePositions);

  try {
    const commandEncoder = device.createCommandEncoder();
    const textureView = context.getCurrentTexture().createView();

    const renderPassDescriptor = {
      colorAttachments: [
        {
          view: textureView,
          clearValue: { r: 0.1, g: 0.1, b: 0.1, a: 1.0 }, // Slightly gray to see if canvas is working
          loadOp: "clear",
          storeOp: "store",
        },
      ],
    };

    const passEncoder =
      commandEncoder.beginRenderPass(renderPassDescriptor);
    passEncoder.setPipeline(renderPipeline);
    passEncoder.setBindGroup(0, bindGroup); // Bind uniform buffer
    passEncoder.setVertexBuffer(0, vertexBuffer); // Base quad vertices
    passEncoder.setVertexBuffer(1, instanceBuffer); // Particle positions
    passEncoder.draw(6, particleCount, 0, 0); // 6 vertices per quad (2 triangles), N instances
    passEncoder.end();

    device.queue.submit([commandEncoder.finish()]);
  } catch (error) {
    log("WebGPU render error: " + error.message);
    console.error("Full WebGPU error:", error);
  }

  // Continue animation
  animationId = requestAnimationFrame(renderFrame);
}

// Initialize everything
async function init() {
  log("Starting WebGPU + Zig WASM demo...");

  const webgpuOk = await initWebGPU();
  if (!webgpuOk) {
    log("Cannot continue without WebGPU support");
    return;
  }

  // Enable WebGPU error reporting
  device.addEventListener("uncapturederror", (event) => {
    console.error("WebGPU uncaptured error:", event.error);
  });

  const wasmOk = await loadWasm();
  if (!wasmOk) {
    log(
      "Cannot continue without WASM module - starting WebGPU-only demo"
    );
    return;
  }

  log(
    `WASM initialized ${wasmModule.exports.get_particle_count()} particles`
  );

  log("Starting animation loop...");
  renderFrame(); // Start the animation
}

// Start when page loads
window.addEventListener("load", init);