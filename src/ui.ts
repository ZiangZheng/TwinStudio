import { APP_NAME, DEFAULT_CONTROLLER } from './constants';
import { formatMetric } from './telemetry';
import type { ControllerOptions, PlaybackMode, TelemetryFrame } from './types';

export interface UIEvents {
  onPlayPause: () => void;
  onReset: () => void;
  onModeChange: (mode: PlaybackMode) => void;
  onSpeedChange: (speed: number) => void;
  onSeek: (time: number) => void;
  onFile: (file: File) => void;
  onControllerChange: (options: ControllerOptions) => void;
  onGhostChange: (visible: boolean) => void;
}

export interface UIHandle {
  root: HTMLElement;
  rgbContainer: HTMLDivElement;
  depthContainer: HTMLDivElement;
  trackingCanvas: HTMLCanvasElement;
  heightCanvas: HTMLCanvasElement;
  effortCanvas: HTMLCanvasElement;
  setLoading(message: string): void;
  setReady(): void;
  setError(message: string): void;
  setMotionInfo(name: string, frames: number, duration: number, fps: number, warnings: string[]): void;
  setTime(time: number, duration: number): void;
  setPlaying(playing: boolean): void;
  setMode(mode: PlaybackMode): void;
  setTelemetry(frame: TelemetryFrame): void;
}

export function buildUI(events: UIEvents): UIHandle {
  let playing = true;
  let controller: ControllerOptions = { ...DEFAULT_CONTROLLER };

  const root = document.createElement('div');
  root.className = 'shell';
  root.innerHTML = `
    <header class="topbar">
      <div class="brand-block">
        <div class="brand-mark">ST</div>
        <div>
          <h1>${APP_NAME}</h1>
          <p>MuJoCo WASM · Sonic sim2sim · Unitree G1</p>
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
        <div class="segmented" role="group" aria-label="Playback mode">
          <button class="selected" data-mode="kinematic">Reference</button>
          <button data-mode="sim2sim">Sim2Sim</button>
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
        <div class="panel-title">Reference</div>
        <input data-file type="file" accept=".json,.npz,application/json" hidden>
        <button class="wide-button" data-upload>Upload JSON / NPZ</button>
        <div class="motion-card">
          <div data-motion-name class="motion-name">Loading bundled squat_001</div>
          <div data-motion-meta class="motion-meta">-- frames · -- s · -- fps</div>
          <div data-motion-warnings class="motion-warnings"></div>
        </div>
      </section>

      <section class="panel options-panel">
        <div class="panel-title">Options</div>
        <label class="control-line">
          <span>Kp</span>
          <input data-kp type="range" min="0" max="500" step="5" value="${DEFAULT_CONTROLLER.kp}">
          <strong data-kp-value>${DEFAULT_CONTROLLER.kp}</strong>
        </label>
        <label class="control-line">
          <span>Kd</span>
          <input data-kd type="range" min="0" max="60" step="1" value="${DEFAULT_CONTROLLER.kd}">
          <strong data-kd-value>${DEFAULT_CONTROLLER.kd}</strong>
        </label>
        <label class="control-line">
          <span>Torque</span>
          <input data-torque type="range" min="0.1" max="2" step="0.05" value="${DEFAULT_CONTROLLER.torqueScale}">
          <strong data-torque-value>${DEFAULT_CONTROLLER.torqueScale.toFixed(2)}</strong>
        </label>
        <label class="toggle-line">
          <span>Reference ghost</span>
          <input data-ghost type="checkbox" checked>
        </label>
      </section>
    </aside>

    <aside class="right-rail">
      <section class="panel metrics-panel">
        <div class="panel-title">Telemetry</div>
        <div class="metric-grid">
          <div><span>FPS</span><strong data-fps>--</strong></div>
          <div><span>RMS</span><strong data-rms>--</strong></div>
          <div><span>z ref</span><strong data-zref>--</strong></div>
          <div><span>z sim</span><strong data-zsim>--</strong></div>
          <div><span>mean tau</span><strong data-mean-torque>--</strong></div>
          <div><span>max tau</span><strong data-max-torque>--</strong></div>
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

    <section class="bottom-dock">
      <div class="plot-panel"><canvas data-tracking-plot></canvas></div>
      <div class="plot-panel"><canvas data-height-plot></canvas></div>
      <div class="plot-panel"><canvas data-effort-plot></canvas></div>
    </section>
  `;

  const status = must(root, '[data-status]');
  const playButton = must<HTMLButtonElement>(root, '[data-play]');
  const resetButton = must<HTMLButtonElement>(root, '[data-reset]');
  const modeButtons = Array.from(root.querySelectorAll<HTMLButtonElement>('[data-mode]'));
  const speed = must<HTMLInputElement>(root, '[data-speed]');
  const speedValue = must(root, '[data-speed-value]');
  const time = must<HTMLInputElement>(root, '[data-time]');
  const timeValue = must(root, '[data-time-value]');
  const fileInput = must<HTMLInputElement>(root, '[data-file]');
  const upload = must<HTMLButtonElement>(root, '[data-upload]');
  const kp = must<HTMLInputElement>(root, '[data-kp]');
  const kd = must<HTMLInputElement>(root, '[data-kd]');
  const torque = must<HTMLInputElement>(root, '[data-torque]');
  const ghost = must<HTMLInputElement>(root, '[data-ghost]');

  playButton.onclick = () => {
    playing = !playing;
    events.onPlayPause();
    setPlaying(playing);
  };
  resetButton.onclick = events.onReset;
  modeButtons.forEach((button) => {
    button.onclick = () => events.onModeChange(button.dataset.mode as PlaybackMode);
  });
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

  const updateController = () => {
    controller = {
      kp: Number(kp.value),
      kd: Number(kd.value),
      torqueScale: Number(torque.value),
    };
    must(root, '[data-kp-value]').textContent = String(controller.kp);
    must(root, '[data-kd-value]').textContent = String(controller.kd);
    must(root, '[data-torque-value]').textContent = controller.torqueScale.toFixed(2);
    events.onControllerChange(controller);
  };
  kp.oninput = updateController;
  kd.oninput = updateController;
  torque.oninput = updateController;
  ghost.onchange = () => events.onGhostChange(ghost.checked);

  function setPlaying(value: boolean) {
    playing = value;
    playButton.textContent = value ? 'Pause' : 'Play';
    playButton.classList.toggle('is-paused', !value);
  }

  return {
    root,
    rgbContainer: must<HTMLDivElement>(root, '[data-rgb]'),
    depthContainer: must<HTMLDivElement>(root, '[data-depth]'),
    trackingCanvas: must<HTMLCanvasElement>(root, '[data-tracking-plot]'),
    heightCanvas: must<HTMLCanvasElement>(root, '[data-height-plot]'),
    effortCanvas: must<HTMLCanvasElement>(root, '[data-effort-plot]'),
    setLoading(message: string) {
      status.textContent = message;
      status.className = 'status-pill loading';
    },
    setReady() {
      status.textContent = 'Live';
      status.className = 'status-pill ready';
    },
    setError(message: string) {
      status.textContent = message;
      status.className = 'status-pill error';
    },
    setMotionInfo(name: string, frames: number, duration: number, fps: number, warnings: string[]) {
      must(root, '[data-motion-name]').textContent = name;
      must(root, '[data-motion-meta]').textContent = `${frames} frames · ${duration.toFixed(2)}s · ${fps.toFixed(1)} fps`;
      must(root, '[data-motion-warnings]').textContent = warnings.join(' ');
      time.max = String(Math.max(duration, 0.01));
      time.step = String(Math.max(duration / 1000, 0.001));
    },
    setTime(current: number, duration: number) {
      if (document.activeElement !== time) time.value = String(Math.min(current, duration));
      timeValue.textContent = `${current.toFixed(2)}s`;
    },
    setPlaying,
    setMode(mode: PlaybackMode) {
      modeButtons.forEach((button) => {
        button.classList.toggle('selected', button.dataset.mode === mode);
      });
    },
    setTelemetry(frame: TelemetryFrame) {
      must(root, '[data-fps]').textContent = formatMetric(frame.fps, 0);
      must(root, '[data-rms]').textContent = formatMetric(frame.trackingRms, 3);
      must(root, '[data-zref]').textContent = formatMetric(frame.rootHeightRef, 2);
      must(root, '[data-zsim]').textContent = formatMetric(frame.rootHeightActual, 2);
      must(root, '[data-mean-torque]').textContent = formatMetric(frame.meanTorque, 1);
      must(root, '[data-max-torque]').textContent = formatMetric(frame.maxTorque, 1);
    },
  };
}

function must<T extends Element = HTMLElement>(root: ParentNode, selector: string): T {
  const el = root.querySelector<T>(selector);
  if (!el) throw new Error(`Missing UI element: ${selector}`);
  return el;
}
