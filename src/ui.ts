import { APP_NAME } from './constants';
import { formatMetric } from './telemetry';
import type { TelemetryFrame } from './types';

export interface UIEvents {
  onPlayPause: () => void;
  onReset: () => void;
  onSpeedChange: (speed: number) => void;
  onSeek: (time: number) => void;
  onFile: (file: File) => void;
}

export interface UIHandle {
  root: HTMLElement;
  rgbContainer: HTMLDivElement;
  depthContainer: HTMLDivElement;
  setLoading(message: string): void;
  setReady(): void;
  setError(message: string): void;
  setMotionInfo(name: string, frames: number, duration: number, fps: number, warnings: string[]): void;
  setTime(time: number, duration: number): void;
  setPlaying(playing: boolean): void;
  setTelemetry(frame: TelemetryFrame): void;
}

export function buildUI(events: UIEvents): UIHandle {
  let playing = true;

  const root = document.createElement('div');
  root.className = 'shell fk-shell';
  root.innerHTML = `
    <header class="topbar">
      <div class="brand-block">
        <div class="brand-mark">ST</div>
        <div>
          <h1>${APP_NAME}</h1>
          <p>MuJoCo FK playback - Sonic dynamic motion - Unitree G1</p>
        </div>
      </div>
      <div class="status-pill" data-status>Booting MuJoCo</div>
    </header>

    <aside class="left-rail">
      <section class="panel transport-panel">
        <div class="panel-title">Playback</div>
        <div class="button-grid">
          <button class="primary-action" data-play>Pause</button>
          <button data-reset>Reset</button>
        </div>
        <label class="control-line">
          <span>Speed</span>
          <input data-speed type="range" min="0.1" max="2.5" step="0.1" value="1">
          <strong data-speed-value>1.0x</strong>
        </label>
        <label class="control-line">
          <span>Time</span>
          <input data-time type="range" min="0" max="1" step="0.01" value="0">
          <strong data-time-value>0.00s</strong>
        </label>
      </section>

      <section class="panel upload-panel">
        <div class="panel-title">Motion</div>
        <input data-file type="file" accept=".json,.npz,application/json" hidden>
        <button class="wide-button" data-upload>Upload JSON / NPZ</button>
        <div class="motion-card">
          <div data-motion-name class="motion-name">Loading bundled squat_001</div>
          <div data-motion-meta class="motion-meta">-- frames · -- s · -- fps</div>
          <div data-motion-warnings class="motion-warnings"></div>
        </div>
      </section>
    </aside>

    <aside class="right-rail">
      <section class="panel metrics-panel">
        <div class="panel-title">Telemetry</div>
        <div class="metric-grid">
          <div><span>FPS</span><strong data-fps>--</strong></div>
          <div><span>Time</span><strong data-runtime>--</strong></div>
          <div><span>root z</span><strong data-zsim>--</strong></div>
          <div><span>FK err</span><strong data-rms>0.000</strong></div>
        </div>
      </section>

      <section class="panel sensor-panel">
        <div class="panel-title">Head Cameras</div>
        <div class="sensor-label">RGB</div>
        <div class="sensor-window" data-rgb></div>
        <div class="sensor-label">Depth</div>
        <div class="sensor-window" data-depth></div>
      </section>
    </aside>
  `;

  const status = must(root, '[data-status]');
  const playButton = must<HTMLButtonElement>(root, '[data-play]');
  const resetButton = must<HTMLButtonElement>(root, '[data-reset]');
  const speed = must<HTMLInputElement>(root, '[data-speed]');
  const speedValue = must(root, '[data-speed-value]');
  const time = must<HTMLInputElement>(root, '[data-time]');
  const timeValue = must(root, '[data-time-value]');
  const fileInput = must<HTMLInputElement>(root, '[data-file]');
  const upload = must<HTMLButtonElement>(root, '[data-upload]');

  playButton.onclick = () => {
    playing = !playing;
    events.onPlayPause();
    setPlaying(playing);
  };
  resetButton.onclick = events.onReset;
  speed.oninput = () => {
    const value = Number(speed.value);
    speedValue.textContent = `${value.toFixed(1)}x`;
    events.onSpeedChange(value);
  };
  time.oninput = () => events.onSeek(Number(time.value));
  upload.onclick = () => fileInput.click();
  fileInput.onchange = () => {
    const file = fileInput.files?.[0];
    if (file) events.onFile(file);
    fileInput.value = '';
  };

  function setPlaying(value: boolean) {
    playing = value;
    playButton.textContent = value ? 'Pause' : 'Play';
    playButton.classList.toggle('is-paused', !value);
  }

  return {
    root,
    rgbContainer: must<HTMLDivElement>(root, '[data-rgb]'),
    depthContainer: must<HTMLDivElement>(root, '[data-depth]'),
    setLoading(message: string) {
      status.textContent = message;
      status.className = 'status-pill loading';
    },
    setReady() {
      status.textContent = 'FK Live';
      status.className = 'status-pill ready';
    },
    setError(message: string) {
      status.textContent = message;
      status.className = 'status-pill error';
    },
    setMotionInfo(name: string, frames: number, duration: number, fps: number, warnings: string[]) {
      must(root, '[data-motion-name]').textContent = name;
      must(root, '[data-motion-meta]').textContent = `${frames} frames - ${duration.toFixed(2)}s - ${fps.toFixed(1)} fps`;
      must(root, '[data-motion-warnings]').textContent = warnings.join(' ');
      time.max = String(Math.max(duration, 0.01));
      time.step = String(Math.max(duration / 1000, 0.001));
    },
    setTime(current: number) {
      if (document.activeElement !== time) time.value = String(current);
      timeValue.textContent = `${current.toFixed(2)}s`;
    },
    setPlaying,
    setTelemetry(frame: TelemetryFrame) {
      must(root, '[data-fps]').textContent = formatMetric(frame.fps, 0);
      must(root, '[data-runtime]').textContent = `${frame.time.toFixed(2)}s`;
      must(root, '[data-zsim]').textContent = formatMetric(frame.rootHeightActual, 2);
      must(root, '[data-rms]').textContent = formatMetric(frame.trackingRms, 3);
    },
  };
}

function must<T extends Element = HTMLElement>(root: ParentNode, selector: string): T {
  const el = root.querySelector<T>(selector);
  if (!el) throw new Error(`Missing UI element: ${selector}`);
  return el;
}
