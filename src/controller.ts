import type loadMujoco from '@mujoco/mujoco';
import type { ControlStats, ControllerOptions } from './types';

type MujocoModule = Awaited<ReturnType<typeof loadMujoco>>;

const jointToActuator = new WeakMap<object, Map<number, number>>();

export function applyPDControl(
  mujoco: MujocoModule,
  model: any,
  data: any,
  qposRef: Float32Array,
  qvelRef: Float32Array,
  options: ControllerOptions,
): ControlStats {
  let map = jointToActuator.get(model);
  if (!map) {
    map = buildJointToActuatorMap(mujoco, model);
    jointToActuator.set(model, map);
  }

  for (let i = 0; i < model.nu; i++) data.ctrl[i] = 0;
  for (let i = 0; i < model.nv; i++) data.qfrc_applied[i] = 0;

  let torqueSum = 0;
  let torqueMax = 0;
  let count = 0;

  for (let j = 0; j < model.njnt; j++) {
    const jointType = model.jnt_type[j];
    if (jointType === mujoco.mjtJoint.mjJNT_FREE.value) continue;
    if (jointType !== mujoco.mjtJoint.mjJNT_HINGE.value && jointType !== mujoco.mjtJoint.mjJNT_SLIDE.value) {
      continue;
    }

    const dofAdr = model.jnt_dofadr[j];
    const qposAdr = model.jnt_qposadr[j];
    if (qposAdr >= qposRef.length || dofAdr >= qvelRef.length) continue;

    const errPos = qposRef[qposAdr] - data.qpos[qposAdr];
    const errVel = qvelRef[dofAdr] - data.qvel[dofAdr];
    let torque = (options.kp * errPos + options.kd * errVel) * options.torqueScale;
    const actIdx = map.get(j);

    if (actIdx !== undefined) {
      torque = clampActuator(model, actIdx, torque);
      data.ctrl[actIdx] = torque;
    } else {
      data.qfrc_applied[dofAdr] = torque;
    }

    const abs = Math.abs(torque);
    torqueSum += abs;
    torqueMax = Math.max(torqueMax, abs);
    count++;
  }

  return {
    meanAbsTorque: count > 0 ? torqueSum / count : 0,
    maxAbsTorque: torqueMax,
  };
}

export function setStateFromReference(model: any, data: any, qposRef: Float32Array, qvelRef?: Float32Array): void {
  for (let i = 0; i < model.nq; i++) data.qpos[i] = qposRef[i] ?? data.qpos[i];
  for (let i = 0; i < model.nv; i++) data.qvel[i] = qvelRef?.[i] ?? 0;
  for (let i = 0; i < model.nu; i++) data.ctrl[i] = 0;
}

function buildJointToActuatorMap(mujoco: MujocoModule, model: any): Map<number, number> {
  const map = new Map<number, number>();
  for (let i = 0; i < model.nu; i++) {
    if (model.actuator_trntype[i] === mujoco.mjtTrn.mjTRN_JOINT.value) {
      map.set(model.actuator_trnid[i * 2], i);
    }
  }
  return map;
}

function clampActuator(model: any, actuatorIndex: number, torque: number): number {
  if (!model.actuator_ctrllimited?.[actuatorIndex]) return torque;
  const lo = model.actuator_ctrlrange[actuatorIndex * 2];
  const hi = model.actuator_ctrlrange[actuatorIndex * 2 + 1];
  return Math.max(lo, Math.min(hi, torque));
}
