/* Optional application timing on the existing socket; no robot commands. */
class VRLatencyClient {
  constructor(socket, scene) {
    this.socket = socket;
    this.scene = scene;
    this.seq = 0;
    this.probeSeq = 0;
    this.pending = new Map();
    this.lastSend = null;
    this.lastReply = null;
    this.rtt = null;
    this.poseAge = null;
    this.closed = false;
    this.timer = setInterval(() => this.probe(), 1000);
    this.probe();
  }
  status() {
    const session = this.scene?.renderer?.xr?.getSession();
    return {page_visibility: document.visibilityState,
      xr_visibility: session?.visibilityState || 'no-xr-session',
      buffered_bytes: this.socket.bufferedAmount,
      send_idle_ms: this.lastSend === null ? null : performance.now() - this.lastSend};
  }
  stamp(packet) {
    if (this.closed) return;
    this.lastSend = performance.now();
    packet._latency = {seq: ++this.seq, sent_ms: this.lastSend, ...this.status(),
      tracking: {left: packet.leftController?.verificationTracking ?? null,
        right: packet.rightController?.verificationTracking ?? null}};
  }
  send(data) {
    if (this.closed || this.socket.readyState !== 1) return false;
    try { this.socket.send(JSON.stringify(data)); return true; }
    catch (_) { return false; }
  }
  probe() {
    if (this.closed) return;
    const now = performance.now();
    for (const [id, sent] of this.pending) {
      if (now - sent > 8000) this.pending.delete(id);
    }
    const id = ++this.probeSeq;
    if (this.send({type: 'latency_ping', id, status: this.status()})) this.pending.set(id, now);
    // Only once per second. A stale reply must not look like a fresh low RTT.
    const hud = document.querySelector('#vrLatencyHud');
    if (hud) {
      const fresh = this.lastReply !== null && now - this.lastReply < 2500;
      const value = fresh
        ? `WS RTT ${this.rtt.toFixed(1)} ms | RX pose age ${this.poseAge === null ? '--' : this.poseAge.toFixed(0)} ms | queue ${this.socket.bufferedAmount} B`
        : 'WS latency: waiting for reply / no recent reply';
      hud.setAttribute('value', value);
      hud.setAttribute('color', fresh && this.rtt < 100 ? '#FFFFFF' : '#FFD54F');
    }
  }
  receive(data) {
    if (this.closed || data?.type !== 'latency_pong' || !this.pending.has(data.id)) return;
    const now = performance.now();
    this.rtt = now - this.pending.get(data.id);
    this.lastReply = now;
    this.poseAge = data.pose_age_ms;
    this.pending.delete(data.id);
    this.send({type: 'latency_result', id: data.id, rtt_ms: this.rtt});
  }
  close() {
    this.closed = true;
    clearInterval(this.timer);
    this.pending.clear();
    const hud = document.querySelector('#vrLatencyHud');
    if (hud) hud.setAttribute('value', 'WS latency: disconnected');
  }
}
