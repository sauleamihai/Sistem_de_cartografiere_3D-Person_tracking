import { useRef } from 'react';
import { useScannerController } from './hooks/useScannerController.js';
import { InfoPanel } from './components/InfoPanel.jsx';
import { StatsPanel } from './components/StatsPanel.jsx';
import { ControlBar } from './components/ControlBar.jsx';

export default function App() {
  const canvasRef = useRef(null);
  const { ui, actions } = useScannerController(canvasRef);

  return (
    <div className="viewer-root">
      <canvas ref={canvasRef} className="viewer-canvas" />
      {ui && (
        <div className="hud">
          <InfoPanel ui={ui} />
          <StatsPanel ui={ui} />
          <ControlBar autoPrune={ui.autoPrune} actions={actions} />
        </div>
      )}
    </div>
  );
}
