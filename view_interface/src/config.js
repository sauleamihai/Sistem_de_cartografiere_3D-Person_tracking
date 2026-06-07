// Centralized configuration. Override VITE_WS_URL via .env (see .env.example).
export const WS_URL = import.meta.env.VITE_WS_URL ?? 'ws://localhost:8080';

// Point-cloud manager tuning
export const MAX_POINTS_PER_CHUNK = 500_000; // smaller chunks => smoother pruning
export const MAX_TOTAL_POINTS = 10_000_000;  // hard cap to avoid GPU/RAM blowup
export const DEDUP_GRID_MM = 2.0;            // spatial quantization grid (mm)

// How often (ms) low-frequency UI state is flushed to React. The data/render
// path is decoupled from this — points stream straight to the GPU regardless.
export const UI_FLUSH_MS = 200; // ~5 Hz telemetry/stat refresh

// Camera home position
export const CAMERA_HOME = [2000, 2000, 2000];
