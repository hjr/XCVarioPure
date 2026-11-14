// Web Serial hardware replay. One clock drives the page and a single, backpressured writer.
// No hardware samples are queued ahead of their deadline. Controls invalidate work by generation;
// a write already handed to the OS can finish, but no later stale sentence can start.
(function (root) {
  "use strict";
  // Simulation mode and clock speed as tools/replay_nmea.py sends them: "$PFLAX,S,<speed>" without a checksum. The
  // firmware takes the command only from a sentence of exactly two fields and counts a checksum as one, so it ignores
  // "$PFLAX,S,1*21" and reads "$PFLAX,S*3c" as speed atoi("3c") = 3: a 1x stream then runs 3x fast on the device.
  const command = (speed) => "$PFLAX,S," + speed;
  const speeds = [1, 2, 5, 10]; // conservative target ceiling; never send the offline 300x option to uint8_t firmware
  function allowedSpeeds(timeline, baud) {
    return timeline?.events.length ? speeds.filter(s => timeline.peakBytes * s <= baud / 10 * 0.7) : [];
  }
  function stateAt(timeline, t) {
    let config = timeline.initialConfig, flap = null, index = 0;
    for (; index < timeline.events.length && timeline.events[index].t < t; index++) { /* lower bound */ }
    for (let i = 0; i < timeline.events.length && timeline.events[i].t <= t; i++) {
      const e = timeline.events[i];
      if (e.kind === "config") config = e.line;
      if (e.kind === "flap") flap = e.line;
    }
    return { index, lines: [config, flap].filter(Boolean) };
  }
  const PORT_KEY = "airmass-replay-serial-port";
  const portId = port => JSON.stringify(port.getInfo?.() ?? {});

  class SerialReplay {
    constructor({ serial = root.navigator?.serial, now = () => performance.now(),
      setTimer = (f, ms) => setTimeout(f, ms), clearTimer = id => clearTimeout(id),
      onChange = () => {}, lagMs = 250, writeTimeoutMs = 2000, storage = null } = {}) {
      Object.assign(this, { serial, now, setTimer, clearTimer, onChange, lagMs, writeTimeoutMs, storage });
      this.timeline = null;
      this.port = this.writer = this.lastUsed = null;
      this.running = this.wanted = this.ready = this.connecting = false;
      this.speed = 1; this.position = 0; this.anchor = 0; this.index = 0; this.generation = 0;
      this.queue = Promise.resolve(); this.timer = null; this.message = "Disconnected";
      this.closing = null; this.initialized = false; this.disconnecting = false;
      this.disconnectTask = null;
      this.onDisconnect = e => {
        if (e.target === this.port || e.port === this.port) this.fail(new Error("Serial device disconnected. Reconnect to resume."));
      };
      serial?.addEventListener("disconnect", this.onDisconnect);
    }
    get connected() { return !!this.writer; }
    time() {
      return this.running ? Math.min(this.timeline.end, this.position + (this.now() - this.anchor) * this.speed / 1000) : this.position;
    }
    notify(message = this.message) { this.message = message; this.onChange(this); }
    cancel() {
      this.position = this.time();
      const next = this.timeline?.events[this.index];
      if (this.running && next) this.position = Math.min(this.position, next.t);
      this.running = false;
      this.generation++;
      this.clearTimer(this.timer); this.timer = null;
      return this.generation;
    }
    enqueue(g, task) {
      const work = this.queue.then(async () => {
        if (g !== this.generation || !this.writer) return;
        try { await task(); } catch (e) { if (g === this.generation) this.fail(e); }
      });
      this.queue = work;
      return work;
    }
    async bounded(promise, ms, message) {
      let timer;
      try {
        return await Promise.race([promise, new Promise((_, reject) => {
          timer = this.setTimer(() => reject(new Error(message)), ms);
        })]);
      } finally { this.clearTimer(timer); }
    }
    async send(line, g, onStart = () => {}) {
      const writer = this.writer;
      if (!writer || g !== this.generation) return false;
      // Await ready separately: pause/seek may invalidate this sentence while the stream is backpressured.
      await this.bounded(writer.ready, this.writeTimeoutMs, "Serial backpressure timed out");
      if (writer !== this.writer || g !== this.generation) return false;
      const written = writer.write(new TextEncoder().encode(line + "\r\n"));
      onStart();
      await this.bounded(written, this.writeTimeoutMs, "Serial write timed out");
      return true;
    }
    // The port of the last connection among those the browser still grants: this session's, else the only one with the
    // remembered USB or Bluetooth identity.
    async lastPort() {
      const ports = await this.serial.getPorts?.() ?? [];
      if (ports.includes(this.lastUsed)) return this.lastUsed;
      let saved = null;
      try { saved = this.storage?.getItem(PORT_KEY); } catch (_) { /* no storage */ }
      const same = ports.filter(p => portId(p) === saved);
      return same.length === 1 ? same[0] : null;
    }
    async connect(baud = 115200, choose = false) {
      if (this.connected || this.connecting) return;
      if (!this.serial) throw new Error("Web Serial is unavailable. Use desktop Chrome over HTTPS or localhost.");
      if (!allowedSpeeds(this.timeline, baud).includes(1)) throw new Error(this.timeline?.reason || "This log exceeds the selected baud rate at 1×. Select a higher baud rate.");
      this.connecting = true;
      let port;
      try {
        if (this.closing) { this.notify("Closing the previous connection…"); await this.closing; }
        if (!choose) port = await this.lastPort();
        this.notify(port ? "Connecting…" : "Choose the instrument's serial port…");
        // Only called by the connect buttons; never request permission on page load.
        port ||= await this.serial.requestPort();
        await port.open({ baudRate: baud, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none", bufferSize: 1024 });
        if (!allowedSpeeds(this.timeline, baud).includes(1)) throw new Error("The loaded log changed; choose a supported sensor log and reconnect.");
        this.lastUsed = port;
        try { this.storage?.setItem(PORT_KEY, portId(port)); } catch (_) { /* no storage */ }
        this.port = port; this.writer = port.writable.getWriter(); this.baud = baud; this.speed = 1; this.initialized = false;
        await this.restore(this.position, false, true);
      } catch (e) {
        if (this.connected) this.fail(e);
        else if (port) await port.close().catch(() => {});
        this.notify(e.name === "NotFoundError" ? "Port selection canceled" : "Serial: " + e.message);
      } finally { this.connecting = false; this.notify(); }
    }
    setTimeline(timeline, t = timeline?.start || 0) {
      if (this.disconnecting) return this.disconnectTask.then(() => this.setTimeline(timeline, t));
      this.cancel(); this.wanted = false; this.ready = false;
      this.timeline = timeline; this.position = t; this.index = 0;
      if (this.connected) {
        this.speed = 1;
        if (!allowedSpeeds(timeline, this.baud).includes(1)) {
          return this.disconnect(timeline?.reason || "Log exceeds this baud rate at 1×; serial disconnected.");
        }
        return this.restore(t, false);
      }
      this.notify();
      return Promise.resolve();
    }
    restore(t, resume = this.wanted, initialize = false) {
      if (this.disconnecting || !this.connected) return Promise.resolve();
      initialize ||= !this.initialized;
      const g = this.cancel();
      this.wanted = resume; this.ready = false;
      this.position = Math.max(this.timeline.start, Math.min(this.timeline.end, t));
      const state = stateAt(this.timeline, this.position);
      this.index = state.index;
      this.notify(initialize ? "Initializing instrument…" : "Restoring replay position…");
      return this.enqueue(g, async () => {
        if (!await this.send(command(0), g)) return; // enters simulation mode, clock stopped
        if (initialize) {
          await new Promise(resolve => this.setTimer(resolve, 1000));
          if (g !== this.generation) return;
          this.initialized = true;
        }
        for (const line of state.lines) if (!await this.send(line, g)) return;
        // Freeze the target until Play, including after the bare initialization command.
        if (!await this.send(command(resume ? this.speed : 0), g) || g !== this.generation) return;
        this.ready = true; this.running = resume; this.anchor = this.now();
        this.notify(resume ? "Streaming · allow the instrument to settle" : "Connected · paused · allow the instrument to settle");
        this.schedule(g);
      });
    }
    seek(t) { return this.restore(t); }
    play() {
      if (!this.connected) return Promise.resolve();
      return this.control(this.speed, true);
    }
    pause(reason = "Connected · paused") {
      if (!this.connected) return Promise.resolve();
      return this.control(this.speed, false, reason);
    }
    setSpeed(speed) {
      if (!allowedSpeeds(this.timeline, this.baud).includes(speed)) throw new Error("Playback speed exceeds the serial/target limit.");
      return this.control(speed, this.wanted);
    }
    control(speed, resume, message) {
      if (this.disconnecting) return Promise.resolve();
      // A control arriving during initialization/seek must still finish the state restoration.
      if (!this.ready) { this.speed = speed; return this.restore(this.position, resume); }
      const g = this.cancel();
      this.speed = speed; this.wanted = resume;
      this.notify(resume ? "Starting serial playback…" : (message || "Connected · paused"));
      return this.enqueue(g, async () => {
        if (!await this.send(command(resume ? speed : 0), g) || g !== this.generation) return;
        this.running = resume; this.anchor = this.now();
        this.notify(message || (resume ? "Streaming · allow the instrument to settle" : "Connected · paused"));
        this.schedule(g);
      });
    }
    schedule(g) {
      if (!this.running || g !== this.generation) return;
      const next = this.timeline.events[this.index];
      if (!next) {
        // Rejected sensor samples can leave a timed gap after the last outgoing sentence.
        const complete = () => {
          this.timer = null;
          if (!this.running || g !== this.generation) return;
          this.position = this.timeline.end;
          this.pause("Connected · replay complete");
        };
        const delay = (this.timeline.end - this.time()) * 1000 / this.speed;
        if (delay > 0) this.timer = this.setTimer(complete, delay);
        else complete();
        return;
      }
      const delay = (next.t - this.time()) * 1000 / this.speed;
      this.timer = this.setTimer(() => {
        this.timer = null;
        this.enqueue(g, () => this.pump(g));
      }, Math.max(0, delay));
    }
    async pump(g) {
      if (!this.running || g !== this.generation) return;
      const e = this.timeline.events[this.index];
      const due = this.anchor + (e.t - this.position) * 1000 / this.speed;
      if (this.now() - due > this.lagMs) { this.lag(e.t); return; }
      // Freeze the shared clock even if writer.ready/write is still blocked. The serialized pause
      // command follows the active write once it drains (or the bounded timeout closes the port).
      const watchdog = this.setTimer(() => {
        if (g === this.generation && this.running) this.lag(e.t);
      }, Math.max(0, this.lagMs - (this.now() - due)));
      try {
        // Consume only once handed to the writer. Backpressure canceled before handoff must retry;
        // a seek has already replaced the cursor, and a handed-off sample must never be duplicated.
        if (await this.send(e.line, g, () => this.index++) && g === this.generation) this.schedule(g);
      } finally { this.clearTimer(watchdog); }
    }
    lag(t) {
      this.cancel(); this.position = t;
      this.pause(`Serial lag exceeded ${this.lagMs} ms · paused without dropping samples. Resume or reduce speed.`);
    }
    fail(e) {
      // Invalidate synchronously so the page pauses before any other animation or stale write.
      this.cancel(); this.wanted = this.ready = false;
      const writer = this.writer, port = this.port;
      this.writer = this.port = null;
      this.notify("Serial stopped: " + e.message);
      this.closeResources(writer, port, true);
    }
    closeResources(writer, port, abort = false) {
      const closing = (async () => {
        try {
          if (abort && writer) await this.bounded(Promise.resolve().then(() => writer.abort()), this.writeTimeoutMs, "Serial abort timed out").catch(() => {});
        }
        finally {
          try { writer?.releaseLock(); } catch (_) { /* device removed */ }
          try { if (port) await this.bounded(port.close(), this.writeTimeoutMs, "Serial close timed out"); } catch (_) { /* device removed or driver stalled */ }
        }
      })();
      this.closing = closing;
      closing.finally(() => { if (this.closing === closing) this.closing = null; });
      return closing;
    }
    disconnect(message = "Disconnected · restart the instrument to leave simulation mode") {
      if (this.disconnecting) return this.disconnectTask;
      if (!this.connected) return Promise.resolve();
      this.disconnecting = true;
      const g = this.cancel(); this.wanted = this.ready = false;
      const writer = this.writer, port = this.port;
      this.notify("Disconnecting…");
      this.disconnectTask = (async () => {
        await this.enqueue(g, () => this.send(command(0), g));
        if (g !== this.generation) return; // an error already owns cleanup
        this.writer = this.port = null;
        this.notify(message);
        await this.closeResources(writer, port);
      })().finally(() => { this.disconnecting = false; this.notify(); });
      return this.disconnectTask;
    }
  }
  const api = { SerialReplay, allowedSpeeds, stateAt, command };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.AirMassSerial = api;
})(typeof globalThis !== "undefined" ? globalThis : this);
