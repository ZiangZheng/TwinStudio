# SonicTwin Studio

Browser-native Sonic FK playback workspace powered by the `mujoco-js` WASM pipeline.

## Intent

SonicTwin Studio loads a Unitree G1 MuJoCo model in the browser, plays Sonic dynamic motion by writing qpos/qvel directly into MuJoCo, and renders the result through the same style of Three.js visual tree used by the PHP MuJoCo WASM demo. The page is a pure FK viewer: no reference ghost, no PD tracker, and no sim2sim control loop.

## Run

```bash
npm install
npm run dev
```

Open the Vite URL printed in the terminal.

The robot is initialized with Sonic's G1 standing root state and MuJoCo-order
default joint angles before any motion clip is installed. Uploaded joint-only
motions inherit this root state unless they provide `root_pos/root_quat` or
`body_pos_w/body_quat_w`.

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
src/app.ts          Runtime orchestration and FK playback loop
src/motion.ts       JSON and NPZ motion loaders
src/phpFkWorld.ts   mujoco-js VFS, model loading, Three.js visuals
src/ui.ts           DOM controls and HUD
src/cameras.ts      RGB/depth first-person render windows
```

## GitHub Pages

The repository includes `.github/workflows/pages.yml`. After pushing to a GitHub
repository, enable Pages with **Source: GitHub Actions** in the repo settings.
The workflow builds `dist/` on every push to `main` and deploys it as a Pages
artifact.
