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

// Tool states
const TOOLS = {
  DRAG: 'drag',
  SPAWN: 'spawn'
};
let currentTool = TOOLS.DRAG;

// Particle painting state
let lastSpawnX = null;
let lastSpawnY = null;
let spawnValence = 0; // Default spawn valence

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

// Set tool to drag/grab
function setDragTool() {
  currentTool = TOOLS.DRAG;
  updateToolButtons();
  updateToolDisplay();
}

// Set tool to spawn with specified valence
function setSpawnTool(valence) {
  currentTool = TOOLS.SPAWN;
  spawnValence = valence;
  updateToolButtons();
  updateToolDisplay();
}

// Update button visual states
function updateToolButtons() {
  // Remove active class from all buttons
  document.querySelectorAll('.tool-btn').forEach(btn => btn.classList.remove('active'));
  
  // Add active class to current tool button
  if (currentTool === TOOLS.DRAG) {
    document.getElementById('grab-btn').classList.add('active');
  } else if (currentTool === TOOLS.SPAWN) {
    document.getElementById(`spawn-${spawnValence}-btn`).classList.add('active');
  }
}

// Keyboard event handlers for tool switching
document.addEventListener('keydown', (event) => {
  if (!wasmModule) return;
  
  const key = event.key.toLowerCase();
  
  if (key === 'q' || key === 'g') {
    setDragTool();
  } else if (key >= '0' && key <= '6') {
    setSpawnTool(parseInt(key));
  }
});

// Update tool display in UI
function updateToolDisplay() {
  const toolDisplay = document.getElementById('tool-display');
  if (toolDisplay) {
    const toolName = currentTool === TOOLS.DRAG ? 'Grab' : `Spawn valence ${spawnValence}`;
    toolDisplay.textContent = `Tool: ${toolName}`;
  }
}

// Spawn a particle if far enough from the last spawn point
function trySpawnParticle(worldX, worldY) {
  if (!wasmModule) return false;
  
  // Check if we should spawn based on distance from last spawn
  if (lastSpawnX !== null && lastSpawnY !== null) {
    const dx = worldX - lastSpawnX;
    const dy = worldY - lastSpawnY;
    const distance = Math.sqrt(dx * dx + dy * dy);
    
    // Use 2x particle diameter as threshold (diameter = 2 * radius)
    const particleSize = wasmModule.exports.get_particle_size();
    const spawnThreshold = particleSize ;
    
    if (distance < spawnThreshold) {
      return false; // Too close to last spawn point
    }
  }
  
  wasmModule.exports.add_particle(worldX, worldY, spawnValence);
  
  lastSpawnX = worldX;
  lastSpawnY = worldY;
  return true;
}

// Mouse event handlers
canvas.addEventListener('pointerdown', (event) => {
  if (!wasmModule) return;
  
  mousePressed = true;
  const worldPos = screenToWorld(event.clientX, event.clientY);
  mouseX = worldPos.x;
  mouseY = worldPos.y;
  
  // Handle based on current tool
  if (currentTool === TOOLS.DRAG) {
    // Tell WASM about mouse press for dragging
    wasmModule.exports.set_mouse_interaction(mouseX, mouseY, true);
  } else if (currentTool === TOOLS.SPAWN) {
    // Start particle painting - spawn first particle and reset spawn tracking
    const spawned = trySpawnParticle(mouseX, mouseY);
    if (spawned) {
      console.log(`Started painting at (${mouseX.toFixed(1)}, ${mouseY.toFixed(1)})`);
    }
  }
});

canvas.addEventListener('pointermove', (event) => {
  if (!wasmModule) return;
  
  const worldPos = screenToWorld(event.clientX, event.clientY);
  mouseX = worldPos.x;
  mouseY = worldPos.y;
  
  // Handle based on current tool and mouse state
  if (mousePressed) {
    if (currentTool === TOOLS.DRAG) {
      // Update mouse position in WASM for dragging
      wasmModule.exports.set_mouse_interaction(mouseX, mouseY, true);
    } else if (currentTool === TOOLS.SPAWN) {
      // Continue painting particles if we've moved far enough
      trySpawnParticle(mouseX, mouseY);
    }
  }
});

canvas.addEventListener('pointerup', (event) => {
  if (!wasmModule) return;
  
  mousePressed = false;
  
  // Handle based on current tool
  if (currentTool === TOOLS.DRAG) {
    // Tell WASM about mouse release for dragging
    wasmModule.exports.set_mouse_interaction(mouseX, mouseY, false);
  } else if (currentTool === TOOLS.SPAWN) {
    // End painting - reset spawn tracking for next painting session
    lastSpawnX = null;
    lastSpawnY = null;
    console.log('Ended particle painting');
  }
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

// Toolbar button event handlers
document.getElementById('grab-btn').addEventListener('click', () => {
  setDragTool();
});

document.getElementById('spawn-0-btn').addEventListener('click', () => {
  setSpawnTool(0);
});

document.getElementById('spawn-1-btn').addEventListener('click', () => {
  setSpawnTool(1);
});

document.getElementById('spawn-2-btn').addEventListener('click', () => {
  setSpawnTool(2);
});

document.getElementById('spawn-3-btn').addEventListener('click', () => {
  setSpawnTool(3);
});

document.getElementById('spawn-4-btn').addEventListener('click', () => {
  setSpawnTool(4);
});

document.getElementById('spawn-5-btn').addEventListener('click', () => {
  setSpawnTool(5);
});

document.getElementById('spawn-6-btn').addEventListener('click', () => {
  setSpawnTool(6);
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

  // Start timing the entire frame
  const frameStart = performance.now();

  // Check pause/step state
  const shouldUpdate = !isPaused || stepRequested;
  if (stepRequested) {
    stepRequested = false;
  }

  let physicsTimeMs = 0;
  if (shouldUpdate) {
    time += 0.016; // ~60fps timing

    // Time the particle update loop
    const updateStart = performance.now();
    wasmModule.exports.update_particles(0.016);
    const updateEnd = performance.now();
    physicsTimeMs = updateEnd - updateStart;
  }

  // Always render (even when paused) to show current state
  const renderStart = performance.now();
  renderer.render(wasmModule);
  const renderEnd = performance.now();
  const renderTimeMs = renderEnd - renderStart;
  
  // Calculate total frame time
  const frameEnd = performance.now();
  const totalFrameTimeMs = frameEnd - frameStart;

  // Update timing display with detailed breakdown
  let statusText;
  if (isPaused) {
    statusText = "PAUSED";
  } else {
    const maxOccupancy = wasmModule.exports.get_spatial_max_occupancy();
    const gridX = wasmModule.exports.get_grid_dimensions_x();
    const gridY = wasmModule.exports.get_grid_dimensions_y();
    const worldW = Math.round(wasmModule.exports.get_world_width_debug());
    const worldH = Math.round(wasmModule.exports.get_world_height_debug());
    const aliveParticles = wasmModule.exports.get_alive_particle_count();
    const aliveSprings = wasmModule.exports.get_alive_spring_count();
    
    statusText = `${Math.round(totalFrameTimeMs)}ms | P:${aliveParticles} S:${aliveSprings} | ${worldW}x${worldH} | ${gridX}x${gridY} | bin:${maxOccupancy}`;
  }
  timingDisplay.textContent = statusText;

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

  // Initialize UI
  updateToolDisplay();
  updateToolButtons();

  log("Starting animation loop...");
  renderFrame(); // Start the animation
}

// Start when page loads
window.addEventListener("load", init);