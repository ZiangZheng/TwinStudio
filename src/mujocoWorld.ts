import * as THREE from 'three';
import type loadMujoco from '@mujoco/mujoco';
import { DEFAULT_ROBOT_XML_URL, DEFAULT_XML_URL, MUJOCO_SCENE_PATH } from './constants';

type MujocoModule = Awaited<ReturnType<typeof loadMujoco>>;

export interface VisualTree {
  root: THREE.Group;
  bodies: Record<number, THREE.Group>;
}

export interface MuJoCoWorld {
  model: any;
  data: any;
  referenceData: any;
  actual: VisualTree;
  reference: VisualTree;
  headBodyId: number;
  pelvisBodyId: number;
}

interface VisualOptions {
  ghost?: boolean;
  includeGround?: boolean;
}

const decoder = new TextDecoder('utf-8');

export async function setupMujocoVFS(mujoco: MujocoModule): Promise<void> {
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
      const response = await fetch(`./assets/g1/meshes/g1/${file}`);
      if (!response.ok) throw new Error(`Failed to load mesh ${file}`);
      mujoco.FS.writeFile(`/workspace/meshes/g1/${file}`, new Uint8Array(await response.arrayBuffer()));
    }),
  );
}

export function loadMuJoCoWorld(mujoco: MujocoModule): MuJoCoWorld {
  const model = mujoco.MjModel.from_xml_path(MUJOCO_SCENE_PATH);
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

  const pelvisBodyId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, 'pelvis');
  let headBodyId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, 'head_link');
  if (headBodyId < 0) headBodyId = pelvisBodyId;

  return { model, data, referenceData, actual, reference, headBodyId, pelvisBodyId };
}

export function updateVisualTransforms(model: any, data: any, bodies: Record<number, THREE.Group>): void {
  for (let b = 1; b < model.nbody; b++) {
    const body = bodies[b];
    if (!body) continue;
    readPosition(data.xpos, b, body.position);
    readQuaternion(data.xquat, b, body.quaternion);
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

function createVisualTree(mujoco: MujocoModule, model: any, options: VisualOptions): VisualTree {
  const names = new Uint8Array(model.names);
  const root = new THREE.Group();
  root.name = options.ghost ? 'Reference Visual' : 'Robot Visual';
  const bodies: Record<number, THREE.Group> = {};
  const geometryCache = new Map<number, THREE.BufferGeometry>();

  for (let g = 0; g < model.ngeom; g++) {
    if (model.geom_group[g] >= 3) continue;

    const bodyId = model.geom_bodyid[g];
    if (!bodies[bodyId]) bodies[bodyId] = makeBodyGroup(model, names, bodyId);

    const type = model.geom_type[g];
    if (type === mujoco.mjtGeom.mjGEOM_PLANE.value && !options.includeGround) continue;

    const geometry = buildGeometryForGeom(mujoco, model, g, geometryCache);
    const mesh = new THREE.Mesh(geometry, buildMaterial(model, g, options.ghost));
    mesh.name = readName(names, model.name_geomadr[g]) || `geom_${g}`;
    mesh.userData.geomId = g;
    mesh.userData.bodyId = bodyId;
    mesh.castShadow = !options.ghost && type !== mujoco.mjtGeom.mjGEOM_PLANE.value;
    mesh.receiveShadow = !options.ghost;

    if (type === mujoco.mjtGeom.mjGEOM_PLANE.value) {
      mesh.rotation.x = -Math.PI / 2;
      mesh.scale.set(80, 80, 1);
    } else {
      readPosition(model.geom_pos, g, mesh.position);
      readQuaternion(model.geom_quat, g, mesh.quaternion);
    }

    bodies[bodyId].add(mesh);
  }

  for (let b = 0; b < model.nbody; b++) {
    if (!bodies[b]) bodies[b] = makeBodyGroup(model, names, b);
    if (b === 0) root.add(bodies[b]);
    else bodies[0].add(bodies[b]);
  }

  if (options.includeGround) {
    const grid = new THREE.GridHelper(48, 96, 0x56616f, 0x27313d);
    grid.position.y = 0.002;
    root.add(grid);
  }

  return { root, bodies };
}

function buildGeometryForGeom(
  mujoco: MujocoModule,
  model: any,
  geomId: number,
  meshCache: Map<number, THREE.BufferGeometry>,
): THREE.BufferGeometry {
  const type = model.geom_type[geomId];
  const sx = model.geom_size[geomId * 3 + 0];
  const sy = model.geom_size[geomId * 3 + 1];
  const sz = model.geom_size[geomId * 3 + 2];

  switch (type) {
    case mujoco.mjtGeom.mjGEOM_SPHERE.value:
      return new THREE.SphereGeometry(sx, 24, 16);
    case mujoco.mjtGeom.mjGEOM_CAPSULE.value:
      return new THREE.CapsuleGeometry(sx, sy * 2, 10, 20);
    case mujoco.mjtGeom.mjGEOM_CYLINDER.value:
      return new THREE.CylinderGeometry(sx, sx, sy * 2, 24);
    case mujoco.mjtGeom.mjGEOM_BOX.value:
      return new THREE.BoxGeometry(sx * 2, sz * 2, sy * 2);
    case mujoco.mjtGeom.mjGEOM_PLANE.value:
      return new THREE.PlaneGeometry(1, 1);
    case mujoco.mjtGeom.mjGEOM_MESH.value: {
      const meshId = model.geom_dataid[geomId];
      const cached = meshCache.get(meshId);
      if (cached) return cached;
      const geometry = buildMeshGeometry(model, meshId);
      meshCache.set(meshId, geometry);
      return geometry;
    }
    default:
      return new THREE.SphereGeometry(Math.max(0.01, sx), 12, 8);
  }
}

function buildMaterial(model: any, geomId: number, ghost = false): THREE.Material {
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
    roughness: 0.62,
    metalness: 0.08,
    transparent: a < 1,
    opacity: a,
  });
}

function buildMeshGeometry(model: any, meshId: number): THREE.BufferGeometry {
  const geometry = new THREE.BufferGeometry();
  const vertStart = model.mesh_vertadr[meshId] * 3;
  const vertEnd = (model.mesh_vertadr[meshId] + model.mesh_vertnum[meshId]) * 3;
  const vertices = new Float32Array(model.mesh_vert.subarray(vertStart, vertEnd));
  swizzleVec3Buffer(vertices);

  const faceStart = model.mesh_faceadr[meshId] * 3;
  const faceEnd = (model.mesh_faceadr[meshId] + model.mesh_facenum[meshId]) * 3;
  const faces = new Uint32Array(model.mesh_face.subarray(faceStart, faceEnd));

  geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
  geometry.setIndex(Array.from(faces));
  geometry.computeVertexNormals();
  return geometry;
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
  const response = await fetch(url);
  if (!response.ok) throw new Error(`Failed to fetch ${url}: ${response.statusText}`);
  return response.text();
}

function mkdirSafe(mujoco: MujocoModule, path: string): void {
  try {
    mujoco.FS.mkdir(path);
  } catch {
    // Emscripten FS throws when the directory already exists.
  }
}
