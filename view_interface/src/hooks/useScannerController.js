import { useEffect, useRef, useState } from 'react';
import { ScannerController } from '../core/ScannerController.js';

// The single bridge between the imperative controller and React. It mounts the
// controller onto a canvas, subscribes to its coalesced UI snapshots, and
// exposes stable action callbacks. React re-renders only when a (throttled)
// snapshot arrives — never per point packet.
export function useScannerController(canvasRef) {
  const controllerRef = useRef(null);
  const [ui, setUi] = useState(null);

  useEffect(() => {
    if (!canvasRef.current) return;
    const controller = new ScannerController(canvasRef.current, setUi);
    controllerRef.current = controller;
    controller.start();
    return () => { controller.dispose(); controllerRef.current = null; };
  }, [canvasRef]);

  const actions = {
    resetView: () => controllerRef.current?.resetView(),
    clearAll: () => controllerRef.current?.clearAll(),
    setAutoPrune: (v) => controllerRef.current?.setAutoPrune(v),
    startScan: (mode) => controllerRef.current?.startScan(mode),
    stopScan: () => controllerRef.current?.stopScan(),
  };

  return { ui, actions };
}
