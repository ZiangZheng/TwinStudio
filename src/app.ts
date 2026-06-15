import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import loadMujoco from 'mujoco-js/dist/mujoco_wasm.js';
import { CameraWindow } from './cameras';
import { DEFAULT_CONTROLLER, DEFAULT_MOTION_URL, INITIAL_STAND_QPOS } from './constants';
import { applyPDControl } from './controller';
import { loadMotionFromFile, loadMotionFromURL, sampleMotion } from './motion';
import { getBodyWorldTransform, loadPhpFkWorld, setupPhpMujocoVFS, updateVisualTransforms, type PhpFkWorld } from './phpFkWorld';
import { buildUI } from './ui';
import { computeTrackingRms, EMPTY_TELEMETRY } from './telemetry';
import type { ControlStats, ControllerOptions, MotionClip, PlaybackMode, TelemetryFrame } from './types';

export async function runApp(container: HTMLElement): Promise<void> {
  container.innerHTML = '<div id="viewport"></div>';
  const viewport = container.querySelector<HTMLDivElement>('#viewport');
  if (!viewport) throw new Error('Viewport mount failed.');

  let world: PhpFkWorld | null = null;
  let mujoco: any = null;
  let motion: MotionClip | null = null;
  let playing = true;
  let playbackMode: PlaybackMode = readInitialPlaybackMode();
  let speed = 1;
  let currentTime = 0;
  let showReference = playbackMode === 'sim2sim';
  let lastFrame = performance.now();
  let smoothedFps = 0;
  let controllerOptions: ControllerOptions = { ...DEFAULT_CONTROLLER };
  let lastControlStats: ControlStats = { meanAbsTorque: 0, maxAbsTorque: 0 };
  const stageOffset = new THREE.Vector3();
  const cameraTarget = new THREE.Vector3(0, 0.9, 0);

  const ui = buildUI({
    onPlayPause: () => {
      playing = !playing;
    },
    onReset: () => resetTo(0),
    onModeChange: (mode) => {
      playbackMode = mode;
      showReference = mode === 'sim2sim';
      ui.setReferenceVisible(showReference);
      resetTo(currentTime);
    },
    onSpeedChange: (value) => {
      speed = value;
    },
    onSeek: (value) => resetTo(value),
    onReferenceChange: (visible) => {
      showReference = visible;
      updateReferenceVisibility();
    },
    onKpChange: (kp) => {
      controllerOptions = { ...controllerOptions, kp };
    },
    onKdChange: (kd) => {
      controllerOptions = { ...controllerOptions, kd };
    },
    onTorqueScaleChange: (torqueScale) => {
      controllerOptions = { ...controllerOptions, torqueScale };
    },
    onFile: async (file) => {
      try {
        ui.setLoading(`Loading ${file.name}`);
        installMotion(await loadMotionFromFile(file));
        ui.setReady();
      } catch (error) {
        ui.setError(error instanceof Error ? error.message : String(error));
      }
    },
  });
  ui.setMode(playbackMode);
  ui.setReferenceVisible(showReference);
  container.appendChild(ui.root);
  ui.setTelemetry(EMPTY_TELEMETRY);

  ui.setLoading('Loading MuJoCo WASM');
  mujoco = await loadMujoco();
  ui.setLoading('Preparing PHP FK pipeline');
  await setupPhpMujocoVFS(mujoco);
  world = loadPhpFkWorld(mujoco);
  setStateFromQpos(world.model, world.data, new Float32Array(INITIAL_STAND_QPOS), new Float32Array(world.model.nv));
  mujoco.mj_forward(world.model, world.data);

  const scene = createScene();
  const stageRoot = new THREE.Group();
  stageRoot.name = 'Centered FK Stage';
  stageRoot.add(world.actual.root);
  stageRoot.add(world.reference.root);
  scene.add(stageRoot);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight, false);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
  viewport.appendChild(renderer.domElement);

  const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 0.02, 160);
  camera.position.set(2.7, 1.65, 2.5);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
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
    if (nextMotion.referenceQpos && nextMotion.referenceQpos[0]?.length !== world.model.nq) {
      warnings.push(`reference qpos has ${nextMotion.referenceQpos[0].length} values, model expects ${world.model.nq}.`);
    }
    motion = { ...nextMotion, warnings };
    ui.setMotionInfo(motion.sourceName, motion.qpos.length, motion.duration, motion.fps, warnings);
    resetTo(0);
  }

  function resetTo(time: number): void {
    if (!world || !mujoco || !motion) return;
    currentTime = wrapTime(time, motion.duration);
    lastControlStats = { meanAbsTorque: 0, maxAbsTorque: 0 };
    const sample = sampleMotion(motion, currentTime);
    setStateFromQpos(world.model, world.data, sample.qpos, sample.qvel);
    setReferenceFromSample(sample);
    mujoco.mj_forward(world.model, world.data);
    mujoco.mj_forward(world.model, world.referenceData);
    updateVisualTransforms(world.model, world.data, world.actual.bodies);
    updateVisualTransforms(world.model, world.referenceData, world.reference.bodies);
    updateReferenceVisibility();
    updateStageCenter();
    ui.setTime(currentTime, motion.duration);
    pushTelemetry();
  }

  function animate(now: number): void {
    requestAnimationFrame(animate);
    if (!world || !mujoco) return;

    const dt = Math.min(Math.max((now - lastFrame) / 1000, 0), 0.1);
    lastFrame = now;
    const instantFps = dt > 0 ? 1 / dt : 0;
    smoothedFps = smoothedFps === 0 ? instantFps : smoothedFps * 0.92 + instantFps * 0.08;

    if (motion && playing) stepPlayback(dt);

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

  function stepPlayback(dt: number): void {
    if (playbackMode === 'sim2sim') stepSim2Sim(dt);
    else stepKinematic(dt);
  }

  function stepKinematic(dt: number): void {
    if (!world || !mujoco || !motion) return;
    currentTime = wrapTime(currentTime + dt * speed, motion.duration);
    const sample = sampleMotion(motion, currentTime);
    setStateFromQpos(world.model, world.data, sample.qpos, sample.qvel);
    setReferenceFromSample(sample);
    mujoco.mj_forward(world.model, world.data);
    mujoco.mj_forward(world.model, world.referenceData);
    ui.setTime(currentTime, motion.duration);
    pushTelemetry();
  }

  function stepSim2Sim(dt: number): void {
    if (!world || !mujoco || !motion) return;
    const step = world.model.opt.timestep || 1 / 500;
    const targetTime = currentTime + dt * speed;
    let steps = 0;
    let stats: ControlStats = lastControlStats;

    while (currentTime < targetTime && steps < 80) {
      if (currentTime >= motion.duration) {
        resetTo(0);
        return;
      }
      const sample = sampleMotion(motion, currentTime);
      stats = applyPDControl(mujoco, world.model, world.data, sample.qpos, sample.qvel, controllerOptions);
      mujoco.mj_step(world.model, world.data);
      currentTime += step;
      steps++;
    }

    currentTime = wrapTime(currentTime, motion.duration);
    lastControlStats = stats;
    const sample = sampleMotion(motion, currentTime);
    setReferenceFromSample(sample);
    mujoco.mj_forward(world.model, world.referenceData);
    ui.setTime(currentTime, motion.duration);
    pushTelemetry();
  }

  function pushTelemetry(): void {
    if (!world || !motion) return;
    const sample = sampleMotion(motion, currentTime);
    const telemetry: TelemetryFrame = {
      time: currentTime,
      fps: smoothedFps,
      trackingRms: sample.referenceQpos ? computeTrackingRms(world.model, world.data.qpos, sample.referenceQpos) : 0,
      rootHeightRef: sample.referenceQpos?.[2] ?? world.data.qpos[2] ?? 0,
      rootHeightActual: world.data.qpos[2] ?? 0,
      meanTorque: playbackMode === 'sim2sim' ? lastControlStats.meanAbsTorque : 0,
      maxTorque: playbackMode === 'sim2sim' ? lastControlStats.maxAbsTorque : 0,
    };
    ui.setTelemetry(telemetry);
  }

  function updateStageCenter(): void {
    if (!world) return;
    const pelvis = getBodyWorldTransform(world.data, world.pelvisBodyId).position;
    stageOffset.set(-pelvis.x, 0, -pelvis.z);
    stageRoot.position.copy(stageOffset);
    cameraTarget.set(0, Math.max(0.72, Math.min(1.35, pelvis.y + 0.06)), 0);
    controls.target.lerp(cameraTarget, 0.12);
  }

  function setReferenceFromSample(sample: ReturnType<typeof sampleMotion>): void {
    if (!world) return;
    if (sample.referenceQpos) {
      setStateFromQpos(world.model, world.referenceData, sample.referenceQpos, sample.referenceQvel);
    }
  }

  function updateReferenceVisibility(): void {
    if (!world || !motion) return;
    world.reference.root.visible = showReference && !!motion.referenceQpos;
  }
}

function setStateFromQpos(model: any, data: any, qpos: Float32Array, qvel?: Float32Array): void {
  for (let i = 0; i < model.nq; i++) data.qpos[i] = qpos[i] ?? data.qpos[i];
  for (let i = 0; i < model.nv; i++) data.qvel[i] = qvel?.[i] ?? 0;
  for (let i = 0; i < model.nu; i++) data.ctrl[i] = 0;
}

function createScene(): THREE.Scene {
  const scene = new THREE.Scene();
  scene.background = makeSkyTexture();
  scene.fog = new THREE.Fog(new THREE.Color(0x9fb5c6), 20, 90);

  scene.add(new THREE.HemisphereLight(0xbfe8ff, 0x423d35, 0.78));
  const key = new THREE.DirectionalLight(0xfff2d4, 2.6);
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

  const rim = new THREE.DirectionalLight(0x63d7ff, 1.1);
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

function readInitialPlaybackMode(): PlaybackMode {
  const mode = new URLSearchParams(window.location.search).get('mode');
  return mode === 'sim2sim' ? 'sim2sim' : 'kinematic';
}
