export class PolicyController {
  constructor(mujoco: any, config?: { modelPath?: string; depthModelPath?: string | null; controlDt?: number });
  controlDt: number;
  isReady: boolean;
  autoForward: boolean;
  highSpeedMode: boolean;
  init(model: any): Promise<void>;
  rebuild(model: any): Promise<void>;
  reset(): void;
  requestAction(model: any, data: any): Promise<void>;
  applyControl(model: any, data: any): void;
  setDepthImage(depthData: Float32Array | null, width: number, height: number): void;
}
