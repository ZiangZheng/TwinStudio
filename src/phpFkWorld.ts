import * as THREE from 'three';
import { DEFAULT_ROBOT_XML_URL, DEFAULT_XML_URL, MUJOCO_SCENE_PATH } from './constants';

export interface PhpFkWorld {
  model: any;
  data: any;
  referenceData: any;
  actual: VisualTree;
  reference: VisualTree;
  headBodyId: number;
  pelvisBodyId: number;
}

export interface VisualTree {
  root: THREE.Group;
  bodies: Record<number, THREE.Group>;
}

const decoder = new TextDecoder('utf-8');

export async function setupPhpMujocoVFS(mujoco: any): Promise<void> {
  mkdirSafe(mujoco, '/workspace');
  mkdirSafe(mujoco, '/workspace/meshes');
  mkdirSafe(mujoco, '/workspace/meshes/g1');

  const sceneText = await fetchText(DEFAULT_XML_URL);
  mujoco.FS.writeFile(MUJOCO_SCENE_PATH, sceneText);

  let robotText = await fetchText(DEFAULT_ROBOT_XML_URL);
  robotText = robotText.replace(/meshdir="\.\.\/meshes\/g1\/"/g, 'meshdir="meshes/g1/"');
  mujoco.FS.writeFile('/workspace/g1_29dof_rev_1_0.xml', robotText);

  const meshFiles = getMeshFiles(robotText);
  await Promise.all(
    meshFiles.map(async (file) => {
      const response = await fetchWithRetry(`./assets/g1/meshes/g1/${file}`);
      mujoco.FS.writeFile(`/workspace/meshes/g1/${file}`, new Uint8Array(await response.arrayBuffer()));
    }),
  );
}

export function loadPhpFkWorld(mujoco: any): PhpFkWorld {
  const model = mujoco.MjModel.loadFromXML(MUJOCO_SCENE_PATH);
  const data = new mujoco.MjData(model);
  const referenceData = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  mujoco.mj_forward(model, referenceData);

  const actual = createVisualTree(mujoco, model, { includeGround: true });
  const reference = createVisualTree(mujoco, model, { ghost: true, includeGround: false });
  reference.root.name = 'Reference Ghost';
  reference.root.visible = false;
  updateVisualTransforms(model, data, actual.bodies);
  updateVisualTransforms(model, referenceData, reference.bodies);

  const pelvisBodyId = findBodyId(model, 'pelvis');
  const torsoBodyId = findBodyId(model, 'torso_link', pelvisBodyId);
  const headBodyId = findBodyId(model, 'head_link', torsoBodyId);

  return { model, data, referenceData, actual, reference, headBodyId, pelvisBodyId };
}

export function updateVisualTransforms(model: any, data: any, bodies: Record<number, THREE.Group>): void {
  for (let b = 0; b < model.nbody; b++) {
    const body = bodies[b];
    if (!body) continue;
    readPosition(data.xpos, b, body.position);
    readQuaternion(data.xquat, b, body.quaternion);
    body.updateWorldMatrix(false, false);
  }
}

export function getBodyWorldTransform(
  data: any,
  bodyId: number,
): { position: THREE.Vector3; quaternion: THREE.Quaternion } {
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  readPosition(data.xpos, bodyId, position);
  readQuaternion(data.xquat, bodyId, quaternion);
  return { position, quaternion };
}

function createVisualTree(mujoco: any, model: any, options: { ghost?: boolean; includeGround?: boolean }): VisualTree {
  const names = new Uint8Array(model.names);
  const root = new THREE.Group();
  root.name = options.ghost ? 'Reference Visual' : 'PHP FK Visual Root';
  const bodies: Record<number, THREE.Group> = {};
  const geometryCache = new Map<number, THREE.BufferGeometry>();

  for (let g = 0; g < model.ngeom; g++) {
    if (!(model.geom_group[g] < 3)) continue;

    const bodyId = model.geom_bodyid[g];
    if (!bodies[bodyId]) bodies[bodyId] = makeBodyGroup(model, names, bodyId);

    const type = model.geom_type[g];
    if (type === mujoco.mjtGeom.mjGEOM_PLANE.value && !options.includeGround) continue;
    const geometry = buildGeometryForGeom(mujoco, model, g, geometryCache);
    const mesh = new THREE.Mesh(geometry, buildMaterial(model, g, !!options.ghost));
    mesh.name = readName(names, model.name_geomadr[g]) || `geom_${g}`;
    mesh.userData.geomId = g;
    mesh.userData.bodyId = bodyId;
    mesh.castShadow = !options.ghost && type !== mujoco.mjtGeom.mjGEOM_PLANE.value;
    mesh.receiveShadow = !options.ghost;

    if (type === mujoco.mjtGeom.mjGEOM_PLANE.value) {
      mesh.rotation.x = -Math.PI / 2;
      mesh.scale.set(96, 96, 1);
    } else {
      readPosition(model.geom_pos, g, mesh.position);
      readQuaternion(model.geom_quat, g, mesh.quaternion);
    }

    bodies[bodyId].add(mesh);
  }

  for (let b = 0; b < model.nbody; b++) {
    if (!bodies[b]) bodies[b] = makeBodyGroup(model, names, b);
    if (b === 0 || !bodies[0]) root.add(bodies[b]);
    else bodies[0].add(bodies[b]);
  }

  return { root, bodies };
}

function buildGeometryForGeom(
  mujoco: any,
  model: any,
  geomId: number,
  meshCache: Map<number, THREE.BufferGeometry>,
): THREE.BufferGeometry {
  const type = model.geom_type[geomId];
  const sx = model.geom_size[geomId * 3 + 0];
  const sy = model.geom_size[geomId * 3 + 1];
  const sz = model.geom_size[geomId * 3 + 2];

  if (type === mujoco.mjtGeom.mjGEOM_SPHERE.value) return new THREE.SphereGeometry(sx, 24, 16);
  if (type === mujoco.mjtGeom.mjGEOM_CAPSULE.value) return new THREE.CapsuleGeometry(sx, sy * 2, 10, 20);
  if (type === mujoco.mjtGeom.mjGEOM_CYLINDER.value) return new THREE.CylinderGeometry(sx, sx, sy * 2, 24);
  if (type === mujoco.mjtGeom.mjGEOM_BOX.value) return new THREE.BoxGeometry(sx * 2, sz * 2, sy * 2);
  if (type === mujoco.mjtGeom.mjGEOM_PLANE.value) return new THREE.PlaneGeometry(1, 1);
  if (type !== mujoco.mjtGeom.mjGEOM_MESH.value) return new THREE.SphereGeometry(Math.max(0.01, sx), 12, 8);

  const meshId = model.geom_dataid[geomId];
  const cached = meshCache.get(meshId);
  if (cached) return cached;

  const geometry = buildMeshGeometry(model, meshId);
  meshCache.set(meshId, geometry);
  return geometry;
}

function buildMeshGeometry(model: any, meshId: number): THREE.BufferGeometry {
  const geometry = new THREE.BufferGeometry();

  const vertStart = model.mesh_vertadr[meshId] * 3;
  const vertEnd = (model.mesh_vertadr[meshId] + model.mesh_vertnum[meshId]) * 3;
  const vertices = new Float32Array(model.mesh_vert.subarray(vertStart, vertEnd));
  swizzleVec3Buffer(vertices);

  const normalStart = model.mesh_normaladr[meshId] * 3;
  const normalEnd = (model.mesh_normaladr[meshId] + model.mesh_normalnum[meshId]) * 3;
  const normals = new Float32Array(model.mesh_normal.subarray(normalStart, normalEnd));
  if (normals.length === vertices.length) swizzleVec3Buffer(normals);

  const faceStart = model.mesh_faceadr[meshId] * 3;
  const faceEnd = (model.mesh_faceadr[meshId] + model.mesh_facenum[meshId]) * 3;
  const faces = new Uint32Array(model.mesh_face.subarray(faceStart, faceEnd));

  geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
  if (normals.length === vertices.length) {
    geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
  }
  geometry.setIndex(Array.from(faces));
  geometry.computeVertexNormals();
  return geometry;
}

function buildMaterial(model: any, geomId: number, ghost: boolean): THREE.Material {
  if (ghost) {
    return new THREE.MeshBasicMaterial({
      color: 0x4de8ff,
      transparent: true,
      opacity: 0.22,
      depthTest: false,
      depthWrite: false,
      wireframe: true,
    });
  }

  const r = model.geom_rgba[geomId * 4 + 0];
  const g = model.geom_rgba[geomId * 4 + 1];
  const b = model.geom_rgba[geomId * 4 + 2];
  const a = model.geom_rgba[geomId * 4 + 3];
  return new THREE.MeshPhysicalMaterial({
    color: new THREE.Color(r, g, b),
    transparent: a < 1,
    opacity: a,
    roughness: 0.56,
    metalness: 0.08,
  });
}

function makeBodyGroup(model: any, names: Uint8Array, bodyId: number): THREE.Group {
  const group = new THREE.Group();
  group.name = readName(names, model.name_bodyadr[bodyId]) || `body_${bodyId}`;
  group.userData.bodyId = bodyId;
  return group;
}

function readPosition(buffer: Float32Array | Float64Array, index: number, target: THREE.Vector3): THREE.Vector3 {
  return target.set(buffer[index * 3 + 0], buffer[index * 3 + 2], -buffer[index * 3 + 1]);
}

function readQuaternion(buffer: Float32Array | Float64Array, index: number, target: THREE.Quaternion): THREE.Quaternion {
  return target.set(
    -buffer[index * 4 + 1],
    -buffer[index * 4 + 3],
    buffer[index * 4 + 2],
    -buffer[index * 4 + 0],
  );
}

function swizzleVec3Buffer(buffer: Float32Array): void {
  for (let i = 0; i < buffer.length; i += 3) {
    const y = buffer[i + 1];
    buffer[i + 1] = buffer[i + 2];
    buffer[i + 2] = -y;
  }
}

function findBodyId(model: any, bodyName: string, fallback = 0): number {
  const names = new Uint8Array(model.names);
  for (let i = 0; i < model.nbody; i++) {
    if (readName(names, model.name_bodyadr[i]) === bodyName) return i;
  }
  return fallback;
}

function readName(namesArray: Uint8Array, address: number): string {
  if (address < 0) return '';
  let end = address;
  while (end < namesArray.length && namesArray[end] !== 0) end++;
  return decoder.decode(namesArray.subarray(address, end));
}

function getMeshFiles(robotXml: string): string[] {
  const doc = new DOMParser().parseFromString(robotXml, 'text/xml');
  return Array.from(doc.querySelectorAll('mesh'))
    .map((el) => el.getAttribute('file'))
    .filter((file): file is string => Boolean(file));
}

async function fetchText(url: string): Promise<string> {
  const response = await fetchWithRetry(url);
  return response.text();
}

async function fetchWithRetry(url: string, attempts = 4): Promise<Response> {
  let lastError = '';
  for (let attempt = 0; attempt < attempts; attempt++) {
    const retryUrl = attempt === 0 ? url : withCacheBust(url, attempt);
    try {
      const response = await fetch(retryUrl, { cache: attempt === 0 ? 'default' : 'reload' });
      if (response.ok) return response;
      lastError = `${response.status} ${response.statusText}`;
    } catch (error) {
      lastError = error instanceof Error ? error.message : String(error);
    }
    await delay(250 * (attempt + 1));
  }
  throw new Error(`Failed to fetch ${url}: ${lastError}`);
}

function withCacheBust(url: string, attempt: number): string {
  const separator = url.includes('?') ? '&' : '?';
  return `${url}${separator}retry=${attempt}-${Date.now()}`;
}

function delay(ms: number): Promise<void> {
  return new Promise((resolve) => window.setTimeout(resolve, ms));
}

function mkdirSafe(mujoco: any, path: string): void {
  try {
    mujoco.FS.mkdir(path);
  } catch {
    // Emscripten FS throws when the directory already exists.
  }
}
