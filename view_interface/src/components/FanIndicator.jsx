export function FanIndicator({ speed }) {
  const pct = Math.max(0, Math.min(100, Math.round(speed)));
  return (
    <span className="fan-wrap">
      <span className="fan-track"><span className="fan-bar" style={{ width: `${pct}%` }} /></span>
      <span className="fan-pct">{pct}%</span>
    </span>
  );
}
