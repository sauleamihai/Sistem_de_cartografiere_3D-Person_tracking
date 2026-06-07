// Reconnecting WebSocket with exponential backoff. The original viewer had no
// reconnect — a dropped bridge meant a dead page until manual refresh.
export class WebSocketClient {
  constructor(url, { onMessage, onState } = {}) {
    this.url = url;
    this.onMessage = onMessage ?? (() => {});
    this.onState = onState ?? (() => {});
    this.ws = null;
    this.shouldRun = false;
    this.retry = 0;
    this.retryTimer = null;
  }

  connect() {
    this.shouldRun = true;
    this._open();
  }

  _open() {
    this.onState('connecting');
    let ws;
    try {
      ws = new WebSocket(this.url);
    } catch {
      return this._scheduleReconnect();
    }
    this.ws = ws;

    ws.onopen = () => { this.retry = 0; this.onState('connected'); };
    ws.onclose = () => { this.onState('disconnected'); if (this.shouldRun) this._scheduleReconnect(); };
    ws.onerror = () => { ws.close(); };
    ws.onmessage = (ev) => {
      try { this.onMessage(JSON.parse(ev.data)); } catch { /* ignore malformed frame */ }
    };
  }

  _scheduleReconnect() {
    clearTimeout(this.retryTimer);
    const delay = Math.min(1000 * 2 ** this.retry, 10_000); // cap at 10s
    this.retry++;
    this.retryTimer = setTimeout(() => this.shouldRun && this._open(), delay);
  }

  send(obj) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  close() {
    this.shouldRun = false;
    clearTimeout(this.retryTimer);
    if (this.ws) { this.ws.onclose = null; this.ws.close(); }
    this.ws = null;
  }
}
