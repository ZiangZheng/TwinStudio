export type PlaybackMode = 'kinematic' | 'sim2sim';

export interface MotionClip {
  sourceName: string;
  fps: number;
  duration: number;
  jointNames: string[];
  times: Float32Array;
  qpos: Float32Array[];
  qvel: Float32Array[];
  warnings: string[];
}

export interface MotionSample {
  qpos: Float32Array;
  qvel: Float32Array;
  idx: number;
  alpha: number;
}

export interface ControllerOptions {
  kp: number;
  kd: number;
  torqueScale: number;
}

export interface ControlStats {
  meanAbsTorque: number;
  maxAbsTorque: number;
}

export interface TelemetryFrame {
  time: number;
  fps: number;
  trackingRms: number;
  rootHeightRef: number;
  rootHeightActual: number;
  meanTorque: number;
  maxTorque: number;
}
