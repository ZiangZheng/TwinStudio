import type { ControllerOptions } from './types';

export const APP_NAME = 'SonicTwin Studio';
export const MUJOCO_SCENE_PATH = '/workspace/scene.xml';
export const DEFAULT_XML_URL = './assets/scenes/g1_with_terrain.xml';
export const DEFAULT_ROBOT_XML_URL = './assets/g1/g1_29dof_rev_1_0.xml';
export const DEFAULT_MOTION_URL = './motions/squat_001.json?v=e94d5b8';

export const DEFAULT_CONTROLLER: ControllerOptions = {
  kp: 150,
  kd: 8,
  torqueScale: 1,
};

export const DEFAULT_ROOT_QPOS = [0, 0, 0.8, 1, 0, 0, 0] as const;

export const SONIC_DEFAULT_ANGLES_MUJOCO = [
  -0.312, 0, 0, 0.669, -0.363, 0,
  -0.312, 0, 0, 0.669, -0.363, 0,
  0, 0, 0,
  0.2, 0.2, 0, 0.6, 0, 0, 0,
  0.2, -0.2, 0, 0.6, 0, 0, 0,
] as const;

export const INITIAL_STAND_QPOS = [...DEFAULT_ROOT_QPOS, ...SONIC_DEFAULT_ANGLES_MUJOCO] as const;

export const SONIC_ISAACLAB_TO_MUJOCO = [
  0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20,
  22, 24, 26, 28,
] as const;

export const G1_MUJOCO_JOINT_NAMES = [
  'left_hip_pitch_joint',
  'left_hip_roll_joint',
  'left_hip_yaw_joint',
  'left_knee_joint',
  'left_ankle_pitch_joint',
  'left_ankle_roll_joint',
  'right_hip_pitch_joint',
  'right_hip_roll_joint',
  'right_hip_yaw_joint',
  'right_knee_joint',
  'right_ankle_pitch_joint',
  'right_ankle_roll_joint',
  'waist_yaw_joint',
  'waist_roll_joint',
  'waist_pitch_joint',
  'left_shoulder_pitch_joint',
  'left_shoulder_roll_joint',
  'left_shoulder_yaw_joint',
  'left_elbow_joint',
  'left_wrist_roll_joint',
  'left_wrist_pitch_joint',
  'left_wrist_yaw_joint',
  'right_shoulder_pitch_joint',
  'right_shoulder_roll_joint',
  'right_shoulder_yaw_joint',
  'right_elbow_joint',
  'right_wrist_roll_joint',
  'right_wrist_pitch_joint',
  'right_wrist_yaw_joint',
] as const;
