import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import loadMujoco from '@mujoco/mujoco';
import wasmUrl from '@mujoco/mujoco/mujoco.wasm?url';
import { CameraWindow } from './cameras';
import { DEFAULT_CONTROLLER, DEFAULT_MOTION_URL, INITIAL_STAND_QPOS } from './constants';
import { applyPDControl, setStateFromReference } from './controller';
import { loadMotionFromFile, loadMotionFromURL, sampleMotion } from './motion';
import { getBodyWorldTransform, loadMuJoCoWorld, setupMujocoVFS, updateVisualTransforms, type MuJoCoWorld } from './mujocoWorld';
import { RealTimePlot } from './plots';
import { buildUI } from './ui';
import { computeTrackingRms, EMPTY_TELEMETRY } from './telemetry';
import type { ControlStats, ControllerOptions, MotionClip, PlaybackMode, TelemetryFrame } from './types';

export async function runApp(container: HTMLElement): Promise<void> {
  container.innerHTML = '<div id="viewport"></div>';
  const viewport = container.querySelector<HTMLDivElement>('#viewport');
  if (!viewport) throw new Error('Viewport mount failed.');

  let world: MuJoCoWorld | null = null;
  let mujoco: Awaited<ReturnType<typeof loadMujoco>> | null = null;
  let motion: MotionClip | null = null;
  let mode: PlaybackMode = 'kinematic';
  let playing = true;
  let speed = 1;
  let currentTime = 0;
  let simTime = 0;
  let showGhost = true;
  let controller: ControllerOptions = { ...DEFAULT_CONTROLLER };
  let lastFrame = performance.now();
  let smoothedFps = 0;
  let latestControl: ControlStats = { meanAbsTorque: 0, maxAbsTorque: 0 };
  let plotAccumulator = 0;
  const stageOffset = new THREE.Vector3();
  const cameraTarget = new THREE.Vector3(0, 0.85, 0);

  const ui = buildUI({
    onPlayPause: () => {
      playing = !playing;
    },
    onReset: () => {
      resetTo(0);
      trackingPlot.clear();
      heightPlot.clear();
      effortPlot.clear();
    },
    onModeChange: (nextMode) => {
      mode = nextMode;
      ui.setMode(mode);
      if (world) world.reference.root.visible = mode === 'sim2sim' && showGhost;
      resetTo(currentTime);
    },
    onSpeedChange: (value) => {
      speed = value;
    },
    onSeek: (value) => {
      resetTo(value);
    },
    onFile: async (file) => {
      try {
        ui.setLoading(`Loading ${file.name}`);
        const nextMotion = await loadMotionFromFile(file);
        installMotion(nextMotion);
        ui.setReady();
      } catch (error) {
        ui.setError(error instanceof Error ? error.message : String(error));
      }
    },
    onControllerChange: (next) => {
      controller = next;
    },
    onGhostChange: (visible) => {
      showGhost = visible;
      if (world) world.reference.root.visible = mode === 'sim2sim' && showGhost;
    },
  });
  container.appendChild(ui.root);

  const trackingPlot = new RealTimePlot(ui.trackingCanvas, 'Tracking RMS', ['qpos rms'], ['#55d6ff']);
  const heightPlot = new RealTimePlot(ui.heightCanvas, 'Root Height', ['reference', 'actual'], ['#6ee7b7', '#f9a86c']);
  const effortPlot = new RealTimePlot(ui.effortCanvas, 'Control Effort', ['mean |tau|', 'max |tau|'], ['#b78cff', '#ff5c8a']);
  ui.setTelemetry(EMPTY_TELEMETRY);

  ui.setLoading('Loading MuJoCo WASM');
  mujoco = await loadMujoco({ locateFile: (path: string) => (path === 'mujoco.wasm' ? wasmUrl : path) });
  ui.setLoading('Preparing G1 model');
  await setupMujocoVFS(mujoco);
  world = loadMuJoCoWorld(mujoco);
  setInitialStandState(world, mujoco);

  const scene = createScene();
  const stageRoot = new THREE.Group();
  stageRoot.name = 'Centered Robot Stage';
  stageRoot.add(world.actual.root);
  stageRoot.add(world.reference.root);
  scene.add(stageRoot);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight, false);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  viewport.appendChild(renderer.domElement);

  const camera = new THREE.PerspectiveCamera(38, window.innerWidth / window.innerHeight, 0.02, 120);
  camera.position.set(2.35, 1.55, 2.35);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;
  controls.target.copy(cameraTarget);

  const rgbWindow = new CameraWindow(ui.rgbContainer, false);
  const depthWindow = new CameraWindow(ui.depthContainer, true);

  installMotion(await loadMotionFromURL(DEFAULT_MOTION_URL));
  ui.setReady();
  requestAnimationFrame(animate);

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight, false);
  });

  function installMotion(nextMotion: MotionClip): void {
    if (!world) return;
    const warnings = [...nextMotion.warnings];
    const firstQpos = nextMotion.qpos[0];
    const firstQvel = nextMotion.qvel[0];
    if (firstQpos.length !== world.model.nq) warnings.push(`qpos has ${firstQpos.length} values, model expects ${world.model.nq}.`);
    if (firstQvel.length !== world.model.nv) warnings.push(`qvel has ${firstQvel.length} values, model expects ${world.model.nv}.`);
    motion = { ...nextMotion, warnings };
    ui.setMotionInfo(motion.sourceName, motion.qpos.length, motion.duration, motion.fps, warnings);
    resetTo(0);
    trackingPlot.clear();
    heightPlot.clear();
    effortPlot.clear();
  }

  function resetTo(time: number): void {
    if (!world || !mujoco || !motion) return;
    currentTime = wrapTime(time, motion.duration);
    simTime = currentTime;
    latestControl = { meanAbsTorque: 0, maxAbsTorque: 0 };
    const sample = sampleMotion(motion, currentTime);
    if (mode === 'sim2sim') {
      setStateFromReference(world.model, world.data, buildStandQposAtReferenceRoot(sample.qpos), new Float32Array(world.model.nv));
    } else {
      setStateFromReference(world.model, world.data, sample.qpos, sample.qvel);
    }
    setStateFromReference(world.model, world.referenceData, sample.qpos, sample.qvel);
    mujoco.mj_forward(world.model, world.data);
    mujoco.mj_forward(world.model, world.referenceData);
    updateVisualTransforms(world.model, world.data, world.actual.bodies);
    updateVisualTransforms(world.model, world.referenceData, world.reference.bodies);
    updateStageCenter();
    world.reference.root.visible = mode === 'sim2sim' && showGhost;
    ui.setTime(currentTime, motion.duration);
  }

  function setInitialStandState(nextWorld: MuJoCoWorld, nextMujoco: Awaited<ReturnType<typeof loadMujoco>>): void {
    const qpos = new Float32Array(INITIAL_STAND_QPOS);
    const qvel = new Float32Array(nextWorld.model.nv);
    setStateFromReference(nextWorld.model, nextWorld.data, qpos, qvel);
    setStateFromReference(nextWorld.model, nextWorld.referenceData, qpos, qvel);
    nextMujoco.mj_forward(nextWorld.model, nextWorld.data);
    nextMujoco.mj_forward(nextWorld.model, nextWorld.referenceData);
    updateVisualTransforms(nextWorld.model, nextWorld.data, nextWorld.actual.bodies);
    updateVisualTransforms(nextWorld.model, nextWorld.referenceData, nextWorld.reference.bodies);
  }

  function animate(now: number): void {
    requestAnimationFrame(animate);
    if (!world || !mujoco) return;

    const dt = Math.min(Math.max((now - lastFrame) / 1000, 0), 0.1);
    lastFrame = now;
    const instantFps = dt > 0 ? 1 / dt : 0;
    smoothedFps = smoothedFps === 0 ? instantFps : smoothedFps * 0.92 + instantFps * 0.08;

    if (motion && playing) stepMotion(dt);

    updateVisualTransforms(world.model, world.data, world.actual.bodies);
    updateVisualTransforms(world.model, world.referenceData, world.reference.bodies);
    updateStageCenter();
    const head = getBodyWorldTransform(world.data, world.headBodyId);
    head.position.add(stageOffset);
    rgbWindow.updatePose(head.position, head.quaternion);
    depthWindow.updatePose(head.position, head.quaternion);

    controls.update();
    renderer.render(scene, camera);
    rgbWindow.render(scene);
    depthWindow.render(scene);
  }

  function stepMotion(dt: number): void {
    if (!world || !mujoco || !motion) return;

    if (mode === 'kinematic') {
      currentTime = wrapTime(currentTime + dt * speed, motion.duration);
      simTime = currentTime;
      const sample = sampleMotion(motion, currentTime);
      setStateFromReference(world.model, world.data, sample.qpos, sample.qvel);
      setStateFromReference(world.model, world.referenceData, sample.qpos, sample.qvel);
      mujoco.mj_forward(world.model, world.data);
      mujoco.mj_forward(world.model, world.referenceData);
      latestControl = { meanAbsTorque: 0, maxAbsTorque: 0 };
    } else {
      const step = world.model.opt.timestep;
      const target = simTime + dt * speed;
      let steps = 0;
      while (simTime < target && steps < 80) {
        const sample = sampleMotion(motion, simTime);
        setStateFromReference(world.model, world.referenceData, sample.qpos, sample.qvel);
        mujoco.mj_forward(world.model, world.referenceData);
        latestControl = applyPDControl(mujoco, world.model, world.data, sample.qpos, sample.qvel, controller);
        mujoco.mj_step(world.model, world.data);
        stabilizeFloatingBase(world.model, world.data, sample.qpos, sample.qvel);
        mujoco.mj_forward(world.model, world.data);
        simTime += step;
        steps++;
      }
      currentTime = simTime;
      if (currentTime >= motion.duration) resetTo(0);
    }

    ui.setTime(currentTime, motion.duration);
    pushTelemetry(dt);
  }

  function pushTelemetry(dt: number): void {
    if (!world || !motion) return;
    plotAccumulator += dt;
    const ref = sampleMotion(motion, currentTime);
    const telemetry: TelemetryFrame = {
      time: currentTime,
      fps: smoothedFps,
      trackingRms: computeTrackingRms(world.model, world.data.qpos, ref.qpos),
      rootHeightRef: ref.qpos[2] ?? 0,
      rootHeightActual: world.data.qpos[2] ?? 0,
      meanTorque: latestControl.meanAbsTorque,
      maxTorque: latestControl.maxAbsTorque,
    };
    ui.setTelemetry(telemetry);
    if (plotAccumulator < 0.08) return;
    plotAccumulator = 0;
    const label = telemetry.time.toFixed(1);
    trackingPlot.push([telemetry.trackingRms], label);
    heightPlot.push([telemetry.rootHeightRef, telemetry.rootHeightActual], label);
    effortPlot.push([telemetry.meanTorque, telemetry.maxTorque], label);
  }

  function updateStageCenter(): void {
    if (!world) return;
    const pelvis = getBodyWorldTransform(world.data, world.pelvisBodyId).position;
    stageOffset.set(-pelvis.x, 0, -pelvis.z);
    stageRoot.position.copy(stageOffset);
    cameraTarget.set(0, Math.max(0.72, Math.min(1.35, pelvis.y + 0.05)), 0);
    controls.target.lerp(cameraTarget, 0.12);
  }
}

function buildStandQposAtReferenceRoot(referenceQpos: Float32Array): Float32Array {
  const qpos = new Float32Array(INITIAL_STAND_QPOS);
  for (let i = 0; i < Math.min(7, referenceQpos.length); i++) qpos[i] = referenceQpos[i];
  return qpos;
}

function stabilizeFloatingBase(model: any, data: any, qposRef: Float32Array, qvelRef: Float32Array): void {
  for (let i = 0; i < Math.min(7, model.nq, qposRef.length); i++) data.qpos[i] = qposRef[i];
  for (let i = 0; i < Math.min(6, model.nv, qvelRef.length); i++) data.qvel[i] = qvelRef[i];
}

function createScene(): THREE.Scene {
  const scene = new THREE.Scene();
  scene.background = makeSkyTexture();
  scene.fog = new THREE.Fog(new THREE.Color(0x9fb5c6), 18, 80);

  scene.add(new THREE.HemisphereLight(0xbfe8ff, 0x423d35, 0.74));
  const key = new THREE.DirectionalLight(0xfff2d4, 2.5);
  key.position.set(6, 9, 4);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.near = 0.5;
  key.shadow.camera.far = 45;
  key.shadow.camera.left = -10;
  key.shadow.camera.right = 10;
  key.shadow.camera.top = 10;
  key.shadow.camera.bottom = -10;
  scene.add(key);

  const rim = new THREE.DirectionalLight(0x63d7ff, 1.2);
  rim.position.set(-4, 4, -5);
  scene.add(rim);
  return scene;
}

function makeSkyTexture(): THREE.CanvasTexture {
  const canvas = document.createElement('canvas');
  canvas.width = 32;
  canvas.height = 512;
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('Canvas is unavailable.');
  const gradient = ctx.createLinearGradient(0, 512, 0, 0);
  gradient.addColorStop(0, '#d7d2c8');
  gradient.addColorStop(0.32, '#a9c1cf');
  gradient.addColorStop(0.62, '#657f98');
  gradient.addColorStop(1, '#1b2d42');
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function wrapTime(time: number, duration: number): number {
  if (duration <= 0) return 0;
  return ((time % duration) + duration) % duration;
}
