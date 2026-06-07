# LiDAR Performance Viewer (React)

Real-time 3D point-cloud viewer and telemetry HUD for the LiDAR scanner. React
port of the original single-file `index.html`, restructured for clarity and
throughput. Talks to the Python WebSocket bridge (`new_scaner_control.py`).

## Run

```bash
npm install
npm run dev          # http://localhost:5173
# point it at your bridge:
echo "VITE_WS_URL=ws://localhost:8080" > .env
```

Start the Python bridge separately; it relays UDP from the scanner to the
browser over WebSocket.

## Architecture — why it's split this way

The one rule that drives the whole design: **the high-throughput data path
never touches React's render cycle.** Point packets can arrive hundreds of
times per second; re-rendering React on each would be fatal. So:

```
WebSocket frame ──► ScannerController ──► PointCloudManager ──► GPU buffers
   (hot path, plain JS, no React)                         (Three.js)

ScannerController ──(coalesced snapshot every ~200ms)──► React state ──► HUD
   (cold path: temps, fans, status, fps — throttled)
```

- `core/ScannerController.js` — plain-JS orchestrator. Owns the `requestAnimationFrame`
  render loop, routes incoming messages, and emits a *coalesced* UI snapshot at a
  fixed cadence (`UI_FLUSH_MS`). Message rate and React render rate are decoupled.
- `three/PointCloudManager.js` — chunked GPU buffer manager: spatial dedup,
  fixed-size chunks, freezing finished chunks as static geometry, partial buffer
  uploads, and pruning past a hard cap. Framework-agnostic (no DOM, no React).
- `three/createScene.js` — scene/camera/renderer/controls factory.
- `net/WebSocketClient.js` — reconnecting socket with exponential backoff.
- `hooks/useScannerController.js` — the single React⇄controller bridge.
- `components/*` — presentational HUD pieces, re-rendered only on snapshots.

## What changed from the original

- Decoupled data path from UI; the manager no longer pokes the DOM directly.
- Throttled telemetry/stat updates (~5 Hz) instead of per-message DOM writes.
- Auto-reconnecting WebSocket (original died on bridge restart).
- Configurable bridge URL via `VITE_WS_URL` (was hardcoded localhost).
- Cross-version Three buffer updates (`addUpdateRange`/`updateRange`).
- Proper teardown: RAF cancelled, socket closed, GPU resources disposed.

## Possible next step

Scan controls (Start/Stop/Home) would need the Python bridge to accept inbound
WebSocket commands — today it only pushes. `WebSocketClient.send()` is ready for it.
