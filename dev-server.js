#!/usr/bin/env node
const http = require('http');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const chokidar = require('chokidar');
const WebSocket = require('ws');

const PORT = 8000;
const WS_PORT = 8001;

// MIME types for static file serving
const mimeTypes = {
  '.html': 'text/html',
  '.js': 'text/javascript',
  '.css': 'text/css',
  '.wasm': 'application/wasm',
  '.svg': 'image/svg+xml'
};

// WebSocket server for hot reload
const wss = new WebSocket.Server({ port: WS_PORT });
console.log(`WebSocket server running on port ${WS_PORT}`);

// Broadcast reload signal to all connected clients
function broadcastReload() {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send('reload');
    }
  });
}

// Build function
function build() {
  return new Promise((resolve, reject) => {
    console.log('🔨 Building...');
    const buildProcess = spawn('./build.sh', [], { stdio: 'inherit' });
    
    buildProcess.on('close', (code) => {
      if (code === 0) {
        console.log('✅ Build successful');
        resolve();
      } else {
        console.error('❌ Build failed');
        reject(new Error(`Build failed with exit code ${code}`));
      }
    });
  });
}

// HTTP server for static files
const server = http.createServer((req, res) => {
  let filePath = req.url === '/' ? '/index.html' : req.url;
  filePath = path.join(__dirname, filePath);
  
  const ext = path.extname(filePath);
  const mimeType = mimeTypes[ext] || 'text/plain';
  
  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404);
      res.end('File not found');
      return;
    }
    
    // Inject WebSocket client code into HTML files
    if (ext === '.html') {
      const injectedScript = `
<script>
  (function() {
    const ws = new WebSocket('ws://localhost:${WS_PORT}');
    ws.onmessage = function(event) {
      if (event.data === 'reload') {
        location.reload();
      }
    };
    ws.onopen = function() {
      console.log('🔄 Hot reload connected');
    };
    ws.onclose = function() {
      console.log('❌ Hot reload disconnected');
    };
  })();
</script>
</body>`;
      data = data.toString().replace('</body>', injectedScript);
    }
    
    res.writeHead(200, { 'Content-Type': mimeType });
    res.end(data);
  });
});

// File watcher
const watcher = chokidar.watch([
  'src/**/*.zig',
  'build.zig',
  '*.js',
  '*.css',
  '*.html'
], {
  ignored: ['node_modules', 'zig-out', '.git'],
  ignoreInitial: true
});

watcher.on('change', async (filePath) => {
  console.log(`📝 File changed: ${filePath}`);
  
  // If Zig files changed, rebuild
  if (filePath.endsWith('.zig')) {
    try {
      await build();
      broadcastReload();
    } catch (error) {
      console.error('Build failed, not reloading');
    }
  } else {
    // For other files, just reload
    broadcastReload();
  }
});

// Start server
server.listen(PORT, () => {
  console.log(`🚀 Dev server running at http://localhost:${PORT}`);
  console.log('👀 Watching for file changes...');
  
  // Initial build
  build().catch(() => {
    console.log('Initial build failed, but server is running');
  });
});