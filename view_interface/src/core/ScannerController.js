import { createScene } from '../three/createScene.js';
import { PointCloudManager } from '../three/PointCloudManager.js';
import { WebSocketClient } from '../net/WebSocketClient.js';
import { WS_URL, UI_FLUSH_MS } from '../config.js';

// The orchestrator. Owns the render loop, the point manager, and the socket.
// It is plain JS (no React) so the hot path never touches the render cycle.
// It emits a *coalesced* UI snapshot at a fixed cadence; React subscribes to
// that, so message rate and React render rate are fully decoupled.
export class ScannerController {
  constructor(canvas, onUi) {
    // The onUi callback is the only bridge to React, so we can batch updates
    //  and avoid re-rendering on every packet.
    this.onUi = onUi ?? (() => {});
    // Three.js scene and point manager.
    this.three = createScene(canvas);
    this.manager = new PointCloudManager(this.three.scene);

    // Low-frequency UI snapshot (the only thing React ever sees).
    this.ui = {
      connection: 'connecting',
      status: 'Connecting...',
      statusKind: 'connected',
      position: '---',
      temp1: null, temp2: null, fan1: 0, fan2: 0,
      fps: 0, pps: 0,
      totalPoints: 0, chunkCount: 1, prunedPoints: 0,
      autoPrune: true,
    };

    // Counters for the stats loop.
    this._frames = 0;
    this._packets = 0;
    this._lastStatFlush = performance.now();
    this._lastUiFlush = 0;
    this._uiDirty = true;
    this._raf = 0;

    // The WebSocket client, with message and state handlers.
    this.socket = new WebSocketClient(WS_URL, {
      onMessage: (m) => this._onMessage(m),
      onState: (s) => this._onSocketState(s),
    });
    // Resize handler to keep the canvas fitting the window.
    this._onResize = () => this.three.resize();
    window.addEventListener('resize', this._onResize);
  }

  start() {
    this.socket.connect();
    const loop = () => {
      this._raf = requestAnimationFrame(loop);
      this.three.controls.update();
      this.three.renderer.render(this.three.scene, this.three.camera);
      this._frames++;
      this._tickStats();
    };
    this._raf = requestAnimationFrame(loop);
  }

  _tickStats() {
    const now = performance.now();
    const dt = now - this._lastStatFlush;
    if (dt >= 1000) {
      this.ui.fps = Math.round((this._frames * 1000) / dt);
      this.ui.pps = Math.round((this._packets * 1000) / dt);
      this._frames = 0; this._packets = 0; this._lastStatFlush = now;
      const s = this.manager.stats();
      this.ui.totalPoints = s.totalPoints;
      this.ui.chunkCount = s.chunkCount;
      this.ui.prunedPoints = s.prunedPoints;
      this._uiDirty = true;
    }
    // Coalesced flush to React at UI_FLUSH_MS, independent of packet rate.
    if (this._uiDirty && now - this._lastUiFlush >= UI_FLUSH_MS) {
      this._lastUiFlush = now;
      this._uiDirty = false;
      this.onUi({ ...this.ui });
    }
  }

  _onSocketState(state) {
    this.ui.connection = state;
    if (state === 'connected') { this.ui.status = 'Connected'; this.ui.statusKind = 'connected'; }
    else if (state === 'disconnected') { this.ui.status = 'Disconnected'; this.ui.statusKind = 'error'; }
    else { this.ui.status = 'Connecting...'; this.ui.statusKind = 'connected'; }
    this._uiDirty = true;
  }

  _onMessage(msg) {
    switch (msg.type) {
      case 'points':
        // HOT PATH: straight to the GPU buffers, no React involvement.
        this.manager.addPoints(msg.data);
        this._packets++;
        break;
      case 'status': this._onStatus(msg.msg); break;
      case 'telemetry': this._onTelemetry(msg); break;
      case 'connection': this.ui.status = msg.msg ?? this.ui.status; this._uiDirty = true; break;
      case 'scan_complete':
        this.ui.status = 'Scan Complete'; this.ui.statusKind = 'connected';
        this.ui.position = 'Home'; this._uiDirty = true; break;
      default: break; // heartbeat etc. ignored for UI
    }
  }

  _onStatus(msg) {
    if (typeof msg !== 'string') return;
    if (msg.startsWith('ROLL:') || msg.startsWith('PITCH:')) {
      this.ui.position = msg; this.ui.status = 'Scanning'; this.ui.statusKind = 'scanning';
    } else if (msg === 'DONE') {
      this.ui.status = 'Scan Complete'; this.ui.statusKind = 'connected'; this.ui.position = 'Home';
    } else if (msg === 'STARTED') {
      this.ui.status = 'Scanning Started'; this.ui.statusKind = 'scanning';
    } else if (msg.startsWith('CRITICAL')) {
      this.ui.status = 'CRITICAL TEMP'; this.ui.statusKind = 'error';
    }
    this._uiDirty = true;
  }

  _onTelemetry(m) {
    this.ui.temp1 = m.temp1; this.ui.temp2 = m.temp2;
    this.ui.fan1 = m.fan1 ?? m.fan1_speed ?? 0;
    this.ui.fan2 = m.fan2 ?? m.fan2_speed ?? 0;
    this._uiDirty = true;
  }

  // --- actions invoked from React ---
  resetView() { this.three.resetView(); }
  clearAll() { this.manager.clearAll(); this._uiDirty = true; }
  setAutoPrune(enabled) { this.manager.setAutoPrune(enabled); this.ui.autoPrune = enabled; this._uiDirty = true; }
  startScan(mode = 'grid') { this.socket.send({ cmd: 'start', mode }); }
  stopScan() { this.socket.send({ cmd: 'stop' }); }

  dispose() {
    cancelAnimationFrame(this._raf);
    window.removeEventListener('resize', this._onResize);
    this.socket.close();
    this.manager.dispose();
    this.three.dispose();
  }
}
