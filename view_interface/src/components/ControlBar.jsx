export function ControlBar({ autoPrune, actions }) {
  return (
    <section className="panel control-bar" aria-label="Controls">
      <button onClick={() => actions.startScan('grid')}>Start Scan</button>
      <button onClick={actions.stopScan}>Stop</button>
      <button onClick={actions.resetView}>Reset View</button>
      <button onClick={actions.clearAll}>Clear All</button>
      <label>
        <input
          type="checkbox"
          checked={autoPrune}
          onChange={(e) => actions.setAutoPrune(e.target.checked)}
        />
        Performance mode (max 10M)
      </label>
    </section>
  );
}
