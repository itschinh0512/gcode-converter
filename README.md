# Image to G-Code ESP Sender

Static browser app for converting JPEG/PNG/WebP line art into simple plotter G-code and streaming it to an ESP32 running FluidNC or another GRBL-compatible firmware.

## Use

Open `index.html` directly in a browser, or serve this folder with any static web server. For real ESP streaming, prefer an HTTP local server so the browser allows `ws://` connections:

```powershell
python -m http.server 5173
```

Then open `http://localhost:5173`.

## Workflow

1. Upload an image.
2. Tune threshold, invert, machine size, feed rates, and pen commands.
   Keep `Merge adjacent lines` enabled when you want thick or neighboring strokes to collapse into a single centerline path.
   Use `Line smoothing` to round jagged traced segments before G-code is generated without raising `Simplify tolerance`.
3. Inspect the preview and generated G-code.
4. Connect to the ESP WebSocket URL, for example `ws://192.168.4.1:81/`.
   If this fails on FluidNC, use `Auto detect` or try `ws://192.168.4.1:82/` with `FluidNC WebUI v3`.
   For the custom `Máy Vẽ Firmware v2.2-AP` page, use `Máy Vẽ HTTP API`; it connects to `/ws` for status and posts the full job to `/gcode`.
5. Send the job. GRBL WebSocket mode streams one line at a time; Máy Vẽ HTTP mode posts the full G-code job to `/gcode`.

## Notes

- The first implementation intentionally emits `G0/G1` only. Arc fitting with `G2/G3` can be added later.
- Default pen commands are `G0 Z5` and `G1 Z0 F300`. Use the `M3/M5` preset only if the firmware config maps those commands safely.
- Machine calibration, stepper pins, homing, limits, and servo setup belong in FluidNC/GRBL firmware configuration.
