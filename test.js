// Quick Node.js test for WASM exports
const fs = require('fs');

async function testWasm() {
    try {
        const wasmBuffer = fs.readFileSync('./webgpu-demo.wasm');
        
        const env = {
            console_log: (ptr, len) => console.log('WASM log called'),
            emscripten_webgpu_get_device: () => 1,
            emscripten_webgpu_create_render_pass_encoder: () => 1,
            emscripten_webgpu_render_pass_draw: () => console.log('Draw called'),
            emscripten_webgpu_submit_render_pass: () => console.log('Submit called'),
        };

        const wasmModule = await WebAssembly.instantiate(wasmBuffer, { env });
        
        console.log('WASM loaded successfully');
        console.log('Available exports:', Object.keys(wasmModule.instance.exports));
        
        // Test function calls if they exist
        if (wasmModule.instance.exports.add) {
            const result = wasmModule.instance.exports.add(5, 3);
            console.log('Add result:', result);
        } else {
            console.log('add function not found');
        }
        
        if (wasmModule.instance.exports.init) {
            wasmModule.instance.exports.init();
            console.log('Init called successfully');
        } else {
            console.log('init function not found');
        }
        
    } catch (error) {
        console.error('Error:', error);
    }
}

testWasm();