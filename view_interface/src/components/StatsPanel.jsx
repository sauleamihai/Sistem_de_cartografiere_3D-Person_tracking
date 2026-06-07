export function StatsPanel({ ui }) {
  return (
    <section className="panel stats-panel" aria-label="Performance stats">
      <div>FPS: <span>{ui.fps}</span></div>
      <div>Packets/s: <span>{ui.pps}</span></div>
      <div>Pruned: <span>{ui.prunedPoints.toLocaleString()}</span></div>
    </section>
  );
}
