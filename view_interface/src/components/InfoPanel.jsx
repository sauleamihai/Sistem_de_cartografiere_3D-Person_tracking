import { StatusBadge } from './StatusBadge.jsx';
import { TemperatureReadout } from './TemperatureReadout.jsx';
import { FanIndicator } from './FanIndicator.jsx';

function Row({ label, children }) {
  return (
    <div className="info-row">
      <span className="info-label">{label}</span>
      <span className="info-value">{children}</span>
    </div>
  );
}

export function InfoPanel({ ui }) {
  return (
    <section className="panel info-panel" aria-label="Scanner status">
      <h2>LiDAR Performance</h2>
      <Row label="Status"><StatusBadge kind={ui.statusKind} label={ui.status} /></Row>
      <Row label="Total Points">{ui.totalPoints.toLocaleString()}</Row>
      <Row label="Chunks (active)">{ui.chunkCount}</Row>
      <Row label="Position">{ui.position}</Row>
      {ui.autoPrune && <Row label="Auto-pruning">ON</Row>}
      <hr />
      <Row label="Temp 1"><TemperatureReadout value={ui.temp1} /></Row>
      <Row label="Temp 2"><TemperatureReadout value={ui.temp2} /></Row>
      <Row label="Fan 1"><FanIndicator speed={ui.fan1} /></Row>
      <Row label="Fan 2"><FanIndicator speed={ui.fan2} /></Row>
    </section>
  );
}
