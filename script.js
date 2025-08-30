const canvas = document.getElementById("webgpu-canvas");
let wasmModule = null;
let renderer = null;
let animationId = null;
let time = 0;
let updateTimeMs = 0;
const timingDisplay = document.getElementById("timing-display");

// Mouse interaction state
let mousePressed = false;
let mouseX = 0;
let mouseY = 0;

// Simulation control state
let isPaused = false;
let stepRequested = false;

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

// Convert mouse screen coordinates to world coordinates
function screenToWorld(screenX, screenY) {
  const rect = canvas.getBoundingClientRect();
  
  // Convert to canvas coordinates (0 to canvas size)
  const canvasX = (screenX - rect.left) * (canvas.width / rect.width);
  const canvasY = (screenY - rect.top) * (canvas.height / rect.height);
  
  // Convert to normalized device coordinates (-1 to 1)
  const ndcX = (canvasX / canvas.width) * 2 - 1;
  const ndcY = -((canvasY / canvas.height) * 2 - 1); // Flip Y axis
  
  // Convert to world coordinates - simple direct mapping
  const worldSize = wasmModule ? wasmModule.exports.get_world_size() : 1.95;
  const worldX = ndcX * (worldSize / 2);
  const worldY = ndcY * (worldSize / 2);
  
  return { x: worldX, y: worldY };
}

// Mouse event handlers
canvas.addEventListener('pointerdown', (event) => {
  if (!wasmModule) return;
  
  mousePressed = true;
  const worldPos = screenToWorld(event.clientX, event.clientY);
  mouseX = worldPos.x;
  mouseY = worldPos.y;
  
  // Tell WASM about mouse press
  wasmModule.exports.set_mouse_interaction(mouseX, mouseY, true);
});

canvas.addEventListener('pointermove', (event) => {
  if (!wasmModule) return;
  
  const worldPos = screenToWorld(event.clientX, event.clientY);
  mouseX = worldPos.x;
  mouseY = worldPos.y;
  
  // Update mouse position in WASM if pressed
  if (mousePressed) {
    wasmModule.exports.set_mouse_interaction(mouseX, mouseY, true);
  }
});

canvas.addEventListener('pointerup', (event) => {
  if (!wasmModule) return;
  
  mousePressed = false;
  
  // Tell WASM about mouse release
  wasmModule.exports.set_mouse_interaction(mouseX, mouseY, false);
});

// Control button event handlers
document.getElementById('pause-btn').addEventListener('click', () => {
  isPaused = !isPaused;
  const btn = document.getElementById('pause-btn');
  btn.textContent = isPaused ? 'Resume' : 'Pause';
});

document.getElementById('step-btn').addEventListener('click', () => {
  if (isPaused) {
    stepRequested = true;
  }
});

document.getElementById('reset-btn').addEventListener('click', () => {
  if (wasmModule) {
    wasmModule.exports.reset(); // Reset simulation to initial state
  }
});

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

  // Check pause/step state
  const shouldUpdate = !isPaused || stepRequested;
  if (stepRequested) {
    stepRequested = false;
  }

  if (shouldUpdate) {
    time += 0.016; // ~60fps timing

    // Time the particle update loop
    const updateStart = performance.now();
    wasmModule.exports.update_particles(0.016);
    const updateEnd = performance.now();
    updateTimeMs = Math.round(updateEnd - updateStart);
  }

  // Update timing display
  const statusText = isPaused ? "PAUSED" : `Update: ${updateTimeMs}ms`;
  timingDisplay.textContent = statusText;

  // Always render (even when paused) to show current state
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