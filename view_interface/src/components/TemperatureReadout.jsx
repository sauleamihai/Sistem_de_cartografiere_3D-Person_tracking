function tempClass(t) {
  if (t == null) return 'temp-normal';
  if (t < 35) return 'temp-normal';
  if (t < 45) return 'temp-warm';
  if (t < 55) return 'temp-hot';
  return 'temp-critical';
}

export function TemperatureReadout({ value }) {
  return (
    <span className={`temp-pill ${tempClass(value)}`}>
      {value == null ? '--' : value.toFixed(1)}&deg;C
    </span>
  );
}
