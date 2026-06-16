import * as THREE from 'three';

export class CameraWindow {
  private readonly container: HTMLElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly depthMaterial: THREE.ShaderMaterial;
  private readonly resizeObserver: ResizeObserver;
  private readonly anchorOffset = new THREE.Vector3(0.24, 0.28, 0);
  private readonly lookAtOffset = new THREE.Vector3(1.45, 0.12, 0);
  private readonly localUp = new THREE.Vector3(0, 1, 0);
  private readonly tmpOffset = new THREE.Vector3();
  private readonly tmpTarget = new THREE.Vector3();
  private readonly tmpUp = new THREE.Vector3();

  constructor(container: HTMLElement, private readonly isDepth: boolean) {
    this.container = container;
    const canvas = document.createElement('canvas');
    canvas.className = 'sensor-canvas';
    container.appendChild(canvas);

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this.camera = new THREE.PerspectiveCamera(58, 1, 0.06, 18);
    this.depthMaterial = new THREE.ShaderMaterial({
      uniforms: {
        near: { value: this.camera.near },
        far: { value: this.camera.far },
      },
      vertexShader: `
        varying float vDepth;
        void main() {
          vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
          vDepth = -mvPosition.z;
          gl_Position = projectionMatrix * mvPosition;
        }
      `,
      fragmentShader: `
        varying float vDepth;
        uniform float near;
        uniform float far;
        void main() {
          float normalized = clamp((vDepth - near) / (far - near), 0.0, 1.0);
          gl_FragColor = vec4(vec3(1.0 - normalized), 1.0);
        }
      `,
      side: THREE.DoubleSide,
    });

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(container);
    this.resize();
  }

  updatePose(anchorPos: THREE.Vector3, anchorQuat: THREE.Quaternion): void {
    this.tmpOffset.copy(this.anchorOffset).applyQuaternion(anchorQuat);
    this.camera.position.copy(anchorPos).add(this.tmpOffset);

    this.tmpTarget.copy(this.lookAtOffset).applyQuaternion(anchorQuat).add(anchorPos);
    this.tmpUp.copy(this.localUp).applyQuaternion(anchorQuat).normalize();
    this.camera.up.copy(this.tmpUp);
    this.camera.lookAt(this.tmpTarget);
  }

  render(scene: THREE.Scene): void {
    if (!this.isDepth) {
      this.renderer.render(scene, this.camera);
      return;
    }

    const background = scene.background;
    const override = scene.overrideMaterial;
    scene.background = new THREE.Color(0x05080d);
    scene.overrideMaterial = this.depthMaterial;
    this.depthMaterial.uniforms.near.value = this.camera.near;
    this.depthMaterial.uniforms.far.value = this.camera.far;
    this.renderer.render(scene, this.camera);
    scene.overrideMaterial = override;
    scene.background = background;
  }

  dispose(): void {
    this.resizeObserver.disconnect();
    this.renderer.dispose();
    this.depthMaterial.dispose();
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }
}
