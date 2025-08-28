const canvas = document.getElementById("webgpu-canvas");
let wasmModule = null;
let renderer = null;
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

async function initRenderer() {
  // Create and initialize the renderer
  renderer = new MorphogenesisRenderer(canvas);
  return await renderer.initialize(wasmModule);
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
      emscripten_webgpu_get_device: () => (renderer?.device ? 1 : 0),
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

// Render frame with particles, springs, and grid
function renderFrame() {
  if (!renderer || !wasmModule) {
    return;
  }

  time += 0.016; // ~60fps timing

  // Time the particle update loop
  const updateStart = performance.now();
  wasmModule.exports.update_particles(0.016);
  const updateEnd = performance.now();
  updateTimeMs = Math.round(updateEnd - updateStart);

  // Update timing display
  timingDisplay.textContent = `Update: ${updateTimeMs}ms`;

  // Render using the renderer
  renderer.render(wasmModule);

  // Continue animation
  animationId = requestAnimationFrame(renderFrame);
}

// Initialize everything
async function init() {
  log("Starting WebGPU + Zig WASM demo...");

  const wasmOk = await loadWasm();
  if (!wasmOk) {
    log("Cannot continue without WASM module");
    return;
  }

  const rendererOk = await initRenderer();
  if (!rendererOk) {
    log("Cannot continue without WebGPU renderer");
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