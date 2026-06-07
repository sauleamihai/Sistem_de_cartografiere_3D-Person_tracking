const KIND_CLASS = { connected: 'led-connected', scanning: 'led-scanning', error: 'led-error' };

export function StatusBadge({ kind, label }) {
  return (
    <span className="status-value">
      <span className={`led ${KIND_CLASS[kind] ?? 'led-connected'}`} />
      {label}
    </span>
  );
}
