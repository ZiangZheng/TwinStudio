# SonicTwin Studio

Browser-native Sonic sim2sim workspace powered by MuJoCo WASM.

## Intent

SonicTwin Studio loads a Unitree G1 MuJoCo model in the browser, plays motion references, and can run a lightweight sim2sim loop with a PD tracker. It supports bundled reference playback and user uploads in JSON or common NPZ layouts.

## Run

```bash
npm install
npm run dev
```

Open the Vite URL printed in the terminal.

## Motion Uploads

Supported JSON keys:

- `fps`, `duration`, `times`
- `qpos`, `qvel`
- `root_pos`, `root_quat`
- `joint_names`

Supported NPZ keys are intentionally flexible:

- Full state: `qpos`, optional `qvel`, optional `times` or `fps`
- Sonic-style reference: `joint_pos` or `dof_pos`, optional `joint_vel` or `dof_vel`, optional `root_pos`, optional `root_quat`

When only joint positions are present, the loader builds a floating-base `qpos` using root fields if available, otherwise a standing default root.

## Structure

```text
public/assets/g1/   Unitree G1 MuJoCo XML and meshes
public/motions/     Bundled reference clips
src/app.ts          Runtime orchestration
src/motion.ts       JSON and NPZ motion loaders
src/mujocoWorld.ts  MuJoCo VFS, model loading, Three.js visuals
src/controller.ts   PD tracker
src/ui.ts           DOM controls and HUD
src/plots.ts        Chart.js telemetry plots
src/cameras.ts      RGB/depth first-person render windows
```
