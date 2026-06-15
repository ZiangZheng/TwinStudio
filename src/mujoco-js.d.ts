declare module 'mujoco-js/dist/mujoco_wasm.js' {
  const loadMujoco: () => Promise<any>;
  export default loadMujoco;
}
