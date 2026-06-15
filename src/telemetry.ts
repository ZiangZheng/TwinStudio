import type { TelemetryFrame } from './types';

export function computeTrackingRms(model: any, actualQpos: Float64Array, refQpos: Float32Array): number {
  let sum = 0;
  let count = 0;
  for (let j = 0; j < model.njnt; j++) {
    const qposAdr = model.jnt_qposadr[j];
    if (qposAdr < 7 || qposAdr >= refQpos.length) continue;
    const err = actualQpos[qposAdr] - refQpos[qposAdr];
    sum += err * err;
    count++;
  }
  return count > 0 ? Math.sqrt(sum / count) : 0;
}

export function formatMetric(value: number, digits = 2): string {
  if (!Number.isFinite(value)) return '--';
  return value.toFixed(digits);
}

export const EMPTY_TELEMETRY: TelemetryFrame = {
  time: 0,
  fps: 0,
  trackingRms: 0,
  rootHeightRef: 0,
  rootHeightActual: 0,
  meanTorque: 0,
  maxTorque: 0,
};
