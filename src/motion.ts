import { unzipSync } from 'fflate';
import {
  G1_MUJOCO_JOINT_NAMES,
  INITIAL_STAND_QPOS,
  SONIC_ISAACLAB_TO_MUJOCO,
} from './constants';
import type { MotionClip, MotionSample } from './types';

type NpyValue = {
  dtype: string;
  shape: number[];
  data: Float32Array | Float64Array | Int32Array | Uint32Array;
};

type RawMotion = Record<string, unknown>;

export async function loadMotionFromURL(url: string): Promise<MotionClip> {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to load motion: ${response.status} ${response.statusText}`);
  }
  const json = (await response.json()) as RawMotion;
  return normalizeMotion(json, url.split('/').pop() || 'remote motion');
}

export async function loadMotionFromFile(file: File): Promise<MotionClip> {
  const lower = file.name.toLowerCase();
  if (lower.endsWith('.npz')) {
    const buffer = await file.arrayBuffer();
    return loadMotionFromNPZ(new Uint8Array(buffer), file.name);
  }

  const text = await file.text();
  const json = JSON.parse(text) as RawMotion;
  return normalizeMotion(json, file.name);
}

function loadMotionFromNPZ(bytes: Uint8Array, sourceName: string): MotionClip {
  const files = unzipSync(bytes);
  const arrays = new Map<string, NpyValue>();
  for (const [path, data] of Object.entries(files)) {
    if (!path.endsWith('.npy')) continue;
    arrays.set(path.replace(/\.npy$/, '').split('/').pop() || path, parseNpy(data));
  }

  const raw: RawMotion = {};
  const warnings: string[] = [];
  const copy = (from: string, to = from) => {
    const value = arrays.get(from);
    if (value) raw[to] = toMatrixOrVector(value);
  };

  copy('qpos');
  copy('qvel');
  copy('times');
  copy('root_pos');
  copy('root_quat');
  copy('body_pos_w');
  copy('body_quat_w');
  copy('joint_names');
  copy('joint_pos');
  copy('dof_pos', 'joint_pos');
  copy('joint_vel');
  copy('dof_vel', 'joint_vel');
  copy('fps');
  copy('dt');

  return normalizeMotion(raw, sourceName, warnings);
}

function normalizeMotion(raw: RawMotion, sourceName: string, inheritedWarnings: string[] = []): MotionClip {
  const warnings = [...inheritedWarnings];
  const hasDynamicQpos = !!raw.qpos;
  expandJointReference(raw, warnings, hasDynamicQpos);
  const qposRows = asMatrix(raw.qpos, 'qpos');
  const qvelRows = asMatrix(raw.qvel, 'qvel');
  if (qposRows.length === 0) throw new Error('Motion has no frames.');
  if (qvelRows.length !== qposRows.length) {
    throw new Error(`qvel frame count (${qvelRows.length}) does not match qpos (${qposRows.length}).`);
  }
  const reference = buildReferenceTrack(raw, warnings, qposRows, qvelRows);

  const fps = readFps(raw) || 30;
  const times = raw.times ? new Float32Array(asVector(raw.times, 'times')) : makeTimes(qposRows.length, fps);
  const duration = typeof raw.duration === 'number' ? raw.duration : times[times.length - 1] || 0;
  const jointNames = Array.isArray(raw.joint_names) ? raw.joint_names.map(String) : [];

  return {
    sourceName,
    fps,
    duration,
    jointNames,
    times,
    qpos: qposRows.map((row) => new Float32Array(row)),
    qvel: qvelRows.map((row) => new Float32Array(row)),
    referenceQpos: reference?.qpos.map((row) => new Float32Array(row)),
    referenceQvel: reference?.qvel.map((row) => new Float32Array(row)),
    warnings,
  };
}

function buildReferenceTrack(
  raw: RawMotion,
  warnings: string[],
  dynamicQpos: number[][],
  dynamicQvel: number[][],
): { qpos: number[][]; qvel: number[][] } | null {
  if (dynamicQpos.length > 0) {
    warnings.push('Built reference track from dynamic qpos/qvel.');
    return {
      qpos: dynamicQpos.map((row) => [...row]),
      qvel: dynamicQvel.map((row) => [...row]),
    };
  }

  const jointPos = raw.joint_pos as number[][] | undefined;
  if (!jointPos) return null;

  const rows = reorderJointRows(raw, jointPos, warnings);
  const rootPos = readRootRows(raw, 'root_pos', 'body_pos_w', 3);
  const rootQuat = readRootRows(raw, 'root_quat', 'body_quat_w', 4);
  const qpos = rows.map((row, frame) => [
    ...(rootPos?.[frame] || INITIAL_STAND_QPOS.slice(0, 3)),
    ...(rootQuat?.[frame] || INITIAL_STAND_QPOS.slice(3, 7)),
    ...row,
  ]);

  const jointVel = raw.joint_vel as number[][] | undefined;
  const qvel = jointVel
    ? reorderJointRows(raw, jointVel, warnings).map((row) => [0, 0, 0, 0, 0, 0, ...row])
    : estimateQvel(qpos, readFps(raw) || 30);

  warnings.push('Built reference track from joint_pos/dof_pos.');
  return { qpos, qvel };
}

function expandJointReference(raw: RawMotion, warnings: string[], hasDynamicQpos: boolean): void {
  if (!raw.root_pos && raw.body_pos_w) {
    raw.root_pos = asMatrix(raw.body_pos_w, 'body_pos_w').map((row) => row.slice(0, 3));
  }
  if (!raw.root_quat && raw.body_quat_w) {
    raw.root_quat = asMatrix(raw.body_quat_w, 'body_quat_w').map((row) => row.slice(0, 4));
  }

  if (!raw.qpos) {
    const jointPos = raw.joint_pos as number[][] | undefined;
    if (!jointPos) throw new Error('Motion needs qpos, joint_pos, or dof_pos.');

    const rows = reorderJointRows(raw, jointPos, warnings);
    const rootPos = raw.root_pos as number[][] | undefined;
    const rootQuat = raw.root_quat as number[][] | undefined;
    raw.qpos = rows.map((row, frame) => [
      ...(rootPos?.[frame] || INITIAL_STAND_QPOS.slice(0, 3)),
      ...(rootQuat?.[frame] || INITIAL_STAND_QPOS.slice(3, 7)),
      ...row,
    ]);
    warnings.push('Built dynamic qpos from joint_pos/dof_pos.');
  }

  if (!raw.qvel) {
    const jointVel = raw.joint_vel as number[][] | undefined;
    const qpos = raw.qpos as number[][];
    if (hasDynamicQpos) {
      raw.qvel = estimateQvel(qpos, readFps(raw) || 30);
      warnings.push('Estimated dynamic qvel by finite differences.');
    } else if (jointVel) {
      const rows = reorderJointRows(raw, jointVel, warnings);
      raw.qvel = rows.map((row) => [0, 0, 0, 0, 0, 0, ...row]);
    } else {
      raw.qvel = estimateQvel(qpos, readFps(raw) || 30);
      warnings.push('Estimated qvel by finite differences.');
    }
  }
}

function reorderJointRows(raw: RawMotion, rows: number[][], warnings: string[]): number[][] {
  const names = raw.joint_names;
  if (Array.isArray(names) && names.length === G1_MUJOCO_JOINT_NAMES.length) {
    const nameToSource = new Map(names.map((name, index) => [String(name), index]));
    const missing = G1_MUJOCO_JOINT_NAMES.filter((name) => !nameToSource.has(name));
    if (missing.length === 0) {
      return rows.map((row) => G1_MUJOCO_JOINT_NAMES.map((name) => row[nameToSource.get(name) ?? 0] ?? 0));
    }
    warnings.push(`joint_names missing ${missing.length} MuJoCo joints; falling back to Sonic IsaacLab order.`);
  }
  if (shouldTreatAsMujocoOrder(raw)) return rows;
  warnings.push('Reordered joint rows from Sonic IsaacLab order to MuJoCo order.');
  return rows.map((row) => SONIC_ISAACLAB_TO_MUJOCO.map((sourceIndex) => row[sourceIndex] ?? 0));
}

function readRootRows(raw: RawMotion, primary: string, fallback: string, width: number): number[][] | null {
  const value = raw[primary] ?? raw[fallback];
  if (!value) return null;
  return asMatrix(value, primary).map((row) => row.slice(0, width));
}

function shouldTreatAsMujocoOrder(raw: RawMotion): boolean {
  const names = raw.joint_names;
  if (!Array.isArray(names) || names.length !== G1_MUJOCO_JOINT_NAMES.length) return false;
  return names.every((name, index) => String(name) === G1_MUJOCO_JOINT_NAMES[index]);
}

export function sampleMotion(motion: MotionClip, time: number): MotionSample {
  const n = motion.times.length;
  if (n === 0) {
    return { qpos: new Float32Array(), qvel: new Float32Array(), idx: 0, alpha: 0 };
  }
  if (n === 1 || time <= motion.times[0]) {
    return cloneSample(motion, 0, 0);
  }
  if (time >= motion.times[n - 1]) {
    return cloneSample(motion, n - 1, 0);
  }

  let lo = 0;
  let hi = n - 1;
  while (lo + 1 < hi) {
    const mid = (lo + hi) >> 1;
    if (motion.times[mid] <= time) lo = mid;
    else hi = mid;
  }

  const idx = lo;
  const t0 = motion.times[idx];
  const t1 = motion.times[idx + 1];
  const alpha = t1 > t0 ? (time - t0) / (t1 - t0) : 0;
  const qpos = lerpQpos(motion.qpos[idx], motion.qpos[idx + 1], alpha);
  const qvel = lerpArray(motion.qvel[idx], motion.qvel[idx + 1], alpha);
  const referenceQpos = motion.referenceQpos ? lerpQpos(motion.referenceQpos[idx], motion.referenceQpos[idx + 1], alpha) : undefined;
  const referenceQvel = motion.referenceQvel ? lerpArray(motion.referenceQvel[idx], motion.referenceQvel[idx + 1], alpha) : undefined;
  return { qpos, qvel, referenceQpos, referenceQvel, idx, alpha };
}

function cloneSample(motion: MotionClip, idx: number, alpha: number): MotionSample {
  return {
    qpos: new Float32Array(motion.qpos[idx]),
    qvel: new Float32Array(motion.qvel[idx]),
    referenceQpos: motion.referenceQpos?.[idx] ? new Float32Array(motion.referenceQpos[idx]) : undefined,
    referenceQvel: motion.referenceQvel?.[idx] ? new Float32Array(motion.referenceQvel[idx]) : undefined,
    idx,
    alpha,
  };
}

function lerpQpos(a: Float32Array, b: Float32Array, alpha: number): Float32Array {
  const out = lerpArray(a, b, alpha);
  if (a.length >= 7) slerp(a, b, out, 3, alpha);
  return out;
}

function lerpArray(a: Float32Array, b: Float32Array, alpha: number): Float32Array {
  const out = new Float32Array(a.length);
  const count = Math.min(a.length, b.length);
  for (let i = 0; i < count; i++) out[i] = a[i] + alpha * (b[i] - a[i]);
  return out;
}

function slerp(a: Float32Array, b: Float32Array, out: Float32Array, offset: number, t: number) {
  let dot =
    a[offset] * b[offset] +
    a[offset + 1] * b[offset + 1] +
    a[offset + 2] * b[offset + 2] +
    a[offset + 3] * b[offset + 3];
  let qb0 = b[offset];
  let qb1 = b[offset + 1];
  let qb2 = b[offset + 2];
  let qb3 = b[offset + 3];
  if (dot < 0) {
    dot = -dot;
    qb0 = -qb0;
    qb1 = -qb1;
    qb2 = -qb2;
    qb3 = -qb3;
  }
  if (dot > 0.9995) {
    const invLen =
      1 /
      Math.hypot(
        out[offset],
        out[offset + 1],
        out[offset + 2],
        out[offset + 3],
      );
    out[offset] *= invLen;
    out[offset + 1] *= invLen;
    out[offset + 2] *= invLen;
    out[offset + 3] *= invLen;
    return;
  }
  const theta0 = Math.acos(Math.max(-1, Math.min(1, dot)));
  const theta = theta0 * t;
  const sinTheta0 = Math.sin(theta0);
  const s0 = Math.cos(theta) - (dot * Math.sin(theta)) / sinTheta0;
  const s1 = Math.sin(theta) / sinTheta0;
  out[offset] = a[offset] * s0 + qb0 * s1;
  out[offset + 1] = a[offset + 1] * s0 + qb1 * s1;
  out[offset + 2] = a[offset + 2] * s0 + qb2 * s1;
  out[offset + 3] = a[offset + 3] * s0 + qb3 * s1;
}

function parseNpy(bytes: Uint8Array): NpyValue {
  const magic = String.fromCharCode(...bytes.slice(0, 6));
  if (magic !== '\x93NUMPY') throw new Error('Invalid NPY file inside NPZ.');
  const major = bytes[6];
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const headerLen = major === 1 ? view.getUint16(8, true) : view.getUint32(8, true);
  const headerStart = major === 1 ? 10 : 12;
  const header = new TextDecoder().decode(bytes.slice(headerStart, headerStart + headerLen));
  const dtype = /'descr':\s*'([^']+)'/.exec(header)?.[1] || '';
  const fortran = /'fortran_order':\s*(True|False)/.exec(header)?.[1] === 'True';
  if (fortran) throw new Error('Fortran-order NPY arrays are not supported yet.');
  const shapeText = /'shape':\s*\(([^)]*)\)/.exec(header)?.[1] || '';
  const shape = shapeText
    .split(',')
    .map((part) => part.trim())
    .filter(Boolean)
    .map(Number);
  const dataStart = headerStart + headerLen;
  const buffer = bytes.buffer.slice(bytes.byteOffset + dataStart, bytes.byteOffset + bytes.byteLength);

  if (dtype === '<f4' || dtype === '|f4') return { dtype, shape, data: new Float32Array(buffer) };
  if (dtype === '<f8') return { dtype, shape, data: new Float64Array(buffer) };
  if (dtype === '<i4') return { dtype, shape, data: new Int32Array(buffer) };
  if (dtype === '<u4') return { dtype, shape, data: new Uint32Array(buffer) };
  throw new Error(`Unsupported NPY dtype: ${dtype}`);
}

function toMatrixOrVector(value: NpyValue): number[] | number[][] {
  const data = Array.from(value.data, Number);
  if (value.shape.length <= 1) return data;
  const cols = value.shape.slice(1).reduce((acc, item) => acc * item, 1);
  const rows: number[][] = [];
  for (let r = 0; r < value.shape[0]; r++) {
    rows.push(data.slice(r * cols, (r + 1) * cols));
  }
  return rows;
}

function asMatrix(value: unknown, name: string): number[][] {
  if (!Array.isArray(value)) throw new Error(`${name} must be an array.`);
  if (value.length === 0) return [];
  if (!Array.isArray(value[0])) throw new Error(`${name} must be a 2D array.`);
  return (value as unknown[][]).map((row, idx) => {
    if (!Array.isArray(row)) throw new Error(`${name}[${idx}] must be an array.`);
    return row.map(Number);
  });
}

function asVector(value: unknown, name: string): number[] {
  if (!Array.isArray(value)) throw new Error(`${name} must be an array.`);
  return value.map(Number);
}

function readFps(raw: RawMotion): number | null {
  if (typeof raw.fps === 'number') return raw.fps;
  if (Array.isArray(raw.fps) && raw.fps.length > 0) return Number(raw.fps[0]);
  if (typeof raw.dt === 'number' && raw.dt > 0) return 1 / raw.dt;
  if (Array.isArray(raw.dt) && raw.dt.length > 0 && Number(raw.dt[0]) > 0) return 1 / Number(raw.dt[0]);
  return null;
}

function makeTimes(count: number, fps: number): Float32Array {
  const times = new Float32Array(count);
  for (let i = 0; i < count; i++) times[i] = i / fps;
  return times;
}

function estimateQvel(qpos: number[][], fps: number): number[][] {
  return qpos.map((row, idx) => {
    const prev = qpos[Math.max(0, idx - 1)];
    const next = qpos[Math.min(qpos.length - 1, idx + 1)];
    const dt = idx === 0 || idx === qpos.length - 1 ? 1 / fps : 2 / fps;
    const freeRoot = 6;
    const out = new Array(Math.max(row.length - 1, freeRoot)).fill(0);
    for (let i = 7; i < row.length; i++) {
      out[i - 1] = (next[i] - prev[i]) / dt;
    }
    return out;
  });
}
