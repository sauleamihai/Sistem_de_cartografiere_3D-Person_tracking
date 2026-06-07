import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { CAMERA_HOME } from '../config.js';

// Builds the Three.js scene, camera, renderer and controls for a given canvas.
// Returns an imperative handle; React never re-renders any of this.
export function createScene(canvas) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x000000);
  scene.fog = new THREE.Fog(0x000000, 1000, 15000);

  scene.add(new THREE.AxesHelper(500));
  scene.add(new THREE.GridHelper(2000, 20, 0x00ff00, 0x003300));

  const camera = new THREE.PerspectiveCamera(
    75, canvas.clientWidth / canvas.clientHeight, 0.1, 50000,
  );
  camera.position.set(...CAMERA_HOME);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({
    canvas, antialias: false, powerPreference: 'high-performance',
  });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(canvas.clientWidth, canvas.clientHeight, false);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.maxDistance = 20000;

  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }

  function resetView() {
    camera.position.set(...CAMERA_HOME);
    camera.lookAt(0, 0, 0);
    controls.reset();
  }

  function dispose() {
    controls.dispose();
    renderer.dispose();
  }

  return { scene, camera, renderer, controls, resize, resetView, dispose };
}
