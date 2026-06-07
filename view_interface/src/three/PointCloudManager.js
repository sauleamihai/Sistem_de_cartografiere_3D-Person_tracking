import * as THREE from 'three';
import { MAX_POINTS_PER_CHUNK, MAX_TOTAL_POINTS, DEDUP_GRID_MM } from '../config.js';

// Chunked point-cloud manager. Streams incoming points into fixed-size GPU
// buffers, deduplicates against a spatial grid, freezes finished chunks as
// static geometry, and prunes the oldest chunks past a hard cap.
//
// This class is deliberately framework-agnostic: it touches no DOM and no React
// state. It only manages Three.js geometry and exposes plain counters that the
// controller reads on its own cadence. That separation is what keeps the
// high-throughput ingest path off React's render cycle.
export class PointCloudManager {
  constructor(scene, maxPointsPerChunk = MAX_POINTS_PER_CHUNK) {
    this.scene = scene;
    this.maxPointsPerChunk = maxPointsPerChunk;
    this.chunks = [];
    this.activeChunk = null;
    this.totalPoints = 0;
    this.prunedPoints = 0;
    this.autoPrune = true;

    this.gridSize = DEDUP_GRID_MM;
    this.lastX = this.lastY = this.lastZ = NaN;

    this.createNewChunk();
  }

  setAutoPrune(enabled) {
    this.autoPrune = enabled;
    if (enabled) this.pruneOldChunks();
  }

  createNewChunk() {
    if (this.activeChunk) {
      const prev = this.activeChunk.mesh.geometry;
      prev.attributes.position.usage = THREE.StaticDrawUsage;
      prev.attributes.color.usage = THREE.StaticDrawUsage;
      this.activeChunk.mesh.frustumCulled = true; // re-enable cull on finished chunk
    }

    const geometry = new THREE.BufferGeometry();
    const positions = new Float32Array(this.maxPointsPerChunk * 3);
    const colors = new Float32Array(this.maxPointsPerChunk * 3);
    geometry.setAttribute('position',
      new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage));
    geometry.setAttribute('color',
      new THREE.BufferAttribute(colors, 3).setUsage(THREE.DynamicDrawUsage));

    // Fixed bounding sphere avoids per-frame recomputation while streaming.
    geometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), 50000);

    const material = new THREE.PointsMaterial({ size: 3, vertexColors: true, sizeAttenuation: true });
    const mesh = new THREE.Points(geometry, material);
    mesh.frustumCulled = false; // keep visible while being written

    this.scene.add(mesh);
    this.chunks.push(mesh);
    this.activeChunk = { mesh, positions, colors, index: 0 };
  }

  // Partial-upload helper that works across Three versions (addUpdateRange was
  // introduced ~r159; updateRange is the legacy field).
  static markRange(attr, startFloat, countFloat) {
    attr.needsUpdate = true;
    if (typeof attr.addUpdateRange === 'function') {
      attr.clearUpdateRanges?.();
      attr.addUpdateRange(startFloat, countFloat);
    } else {
      attr.updateRange = { offset: startFloat, count: countFloat };
    }
  }

  addPoints(pointArray) {
    let chunk = this.activeChunk;
    let added = 0;
    const startIndex = chunk.index;

    for (const p of pointArray) {
      // Spatial dedup: snap to a 2mm grid, skip if identical to the last point.
      const qx = Math.round(p.x / this.gridSize) * this.gridSize;
      const qy = Math.round(p.y / this.gridSize) * this.gridSize;
      const qz = Math.round(p.z / this.gridSize) * this.gridSize;
      if (qx === this.lastX && qy === this.lastY && qz === this.lastZ) continue;
      this.lastX = qx; this.lastY = qy; this.lastZ = qz;

      if (chunk.index >= this.maxPointsPerChunk) {
        // Flush the just-filled chunk's range before rotating.
        this._flush(chunk, startIndex, added);
        this.createNewChunk();
        chunk = this.activeChunk;
        added = 0;
      }

      const idx = chunk.index * 3;
      // LiDAR Z-up -> Three.js Y-up.
      chunk.positions[idx] = p.x;
      chunk.positions[idx + 1] = p.z;
      chunk.positions[idx + 2] = -p.y;
      chunk.colors[idx] = p.r / 255;
      chunk.colors[idx + 1] = p.g / 255;
      chunk.colors[idx + 2] = p.b / 255;

      chunk.index++;
      added++;
    }

    if (added > 0) {
      this.totalPoints += added;
      this._flush(chunk, chunk.index - added, added);
    }
    if (this.autoPrune) this.pruneOldChunks();
  }

  _flush(chunk, startPoint, addedPoints) {
    if (addedPoints <= 0) return;
    const geo = chunk.mesh.geometry;
    geo.setDrawRange(0, chunk.index);
    PointCloudManager.markRange(geo.attributes.position, startPoint * 3, addedPoints * 3);
    PointCloudManager.markRange(geo.attributes.color, startPoint * 3, addedPoints * 3);
  }

  pruneOldChunks() {
    while (this.totalPoints > MAX_TOTAL_POINTS && this.chunks.length > 1) {
      const old = this.chunks.shift();
      this.scene.remove(old);
      const removed = old.geometry.drawRange.count || this.maxPointsPerChunk;
      old.geometry.dispose();
      old.material.dispose();
      this.totalPoints -= removed;
      this.prunedPoints += removed;
    }
  }

  clearAll() {
    for (const c of this.chunks) {
      this.scene.remove(c);
      c.geometry.dispose();
      c.material.dispose();
    }
    this.chunks = [];
    this.totalPoints = 0;
    this.prunedPoints = 0;
    this.lastX = this.lastY = this.lastZ = NaN;
    this.createNewChunk();
  }

  // Plain snapshot for the stats loop — no DOM, no React.
  stats() {
    return {
      totalPoints: this.totalPoints,
      chunkCount: this.chunks.length,
      prunedPoints: this.prunedPoints,
    };
  }

  dispose() {
    this.clearAll();
  }
}
