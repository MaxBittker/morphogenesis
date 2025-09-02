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
  // Get the actual device pixel ratio (respects browser zoom)
  const currentDPR = window.devicePixelRatio;
  
  // Canvas resolution follows device pixel ratio
  canvas.width = window.innerWidth * currentDPR;
  canvas.height = window.innerHeight * currentDPR;
  canvas.style.width = window.innerWidth + "px";
  canvas.style.height = window.innerHeight + "px";

  // Store aspect ratio for reference
  window.aspectRatio = canvas.width / canvas.height;
  
  // Set world dimensions based on CSS pixels (not device pixels)
  // This makes the world size stay constant in logical units while zoom changes
  if (wasmModule && wasmModule.exports.set_world_dimensions) {
    // Use CSS pixel dimensions for world space (zoom-independent logical size)
    const worldWidth = window.innerWidth;
    const worldHeight = window.innerHeight;
    
    wasmModule.exports.set_world_dimensions(worldWidth, worldHeight);
    console.log(`World: ${worldWidth}x${worldHeight} logical pixels | Canvas: ${canvas.width}x${canvas.height} device pixels (DPR: ${currentDPR.toFixed(2)})`);
  }
}

// Listen for both resize and zoom changes
window.addEventListener("resize", resizeCanvas);

// Detect zoom changes by monitoring devicePixelRatio
let lastDPR = window.devicePixelRatio;
function checkZoomChange() {
  if (window.devicePixelRatio !== lastDPR) {
    lastDPR = window.devicePixelRatio;
    resizeCanvas();
  }
}
// Check for zoom changes periodically
setInterval(checkZoomChange, 500);
resizeCanvas();

// Convert mouse screen coordinates to world coordinates
function screenToWorld(screenX, screenY) {
  const rect = canvas.getBoundingClientRect();
  
  // Convert to logical coordinates (CSS pixels)
  const logicalX = screenX - rect.left;
  const logicalY = screenY - rect.top;
  
  // World coordinates use CSS pixels (logical size) with origin at center
  const worldX = logicalX - (window.innerWidth / 2);
  const worldY = (window.innerHeight / 2) - logicalY; // Flip Y axis for standard coordinate system
  
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
    
    // Set initial world dimensions based on current aspect ratio
    resizeCanvas(); // This will call set_world_dimensions
    
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

  // Update timing display with spatial occupancy
  let statusText;
  if (isPaused) {
    statusText = "PAUSED";
  } else {
    const maxOccupancy = wasmModule.exports.get_spatial_max_occupancy();
    const gridX = wasmModule.exports.get_grid_dimensions_x();
    const gridY = wasmModule.exports.get_grid_dimensions_y();
    const worldW = Math.round(wasmModule.exports.get_world_width_debug());
    const worldH = Math.round(wasmModule.exports.get_world_height_debug());
    statusText = `Update: ${updateTimeMs}ms | World: ${worldW}x${worldH} | Grid: ${gridX}x${gridY} | Max bin: ${maxOccupancy}`;
  }
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