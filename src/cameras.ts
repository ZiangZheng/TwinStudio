import * as THREE from 'three';

export class CameraWindow {
  private readonly container: HTMLElement;
  private readonly renderer: THREE.WebGLRenderer;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly depthMaterial: THREE.ShaderMaterial;
  private readonly captureTarget: THREE.WebGLRenderTarget | null;
  private readonly capturePixels: Float32Array | null;
  private readonly captureFrame: Float32Array | null;
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
    this.captureTarget = isDepth
      ? new THREE.WebGLRenderTarget(106, 60, {
          type: THREE.FloatType,
          format: THREE.RGBAFormat,
          minFilter: THREE.NearestFilter,
          magFilter: THREE.NearestFilter,
          depthBuffer: true,
          stencilBuffer: false,
        })
      : null;
    this.capturePixels = isDepth ? new Float32Array(106 * 60 * 4) : null;
    this.captureFrame = isDepth ? new Float32Array(106 * 60) : null;

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

  captureDepthFrame(scene: THREE.Scene): { data: Float32Array; width: number; height: number } | null {
    if (!this.isDepth || !this.captureTarget || !this.capturePixels || !this.captureFrame) {
      return null;
    }

    const background = scene.background;
    const override = scene.overrideMaterial;
    const previousTarget = this.renderer.getRenderTarget();
    scene.background = new THREE.Color(0x000000);
    scene.overrideMaterial = this.depthMaterial;
    this.depthMaterial.uniforms.near.value = this.camera.near;
    this.depthMaterial.uniforms.far.value = this.camera.far;

    try {
      this.renderer.setRenderTarget(this.captureTarget);
      this.renderer.clear();
      this.renderer.render(scene, this.camera);
      this.renderer.readRenderTargetPixels(
        this.captureTarget,
        0,
        0,
        this.captureTarget.width,
        this.captureTarget.height,
        this.capturePixels,
      );
    } catch (error) {
      return null;
    } finally {
      this.renderer.setRenderTarget(previousTarget);
      scene.overrideMaterial = override;
      scene.background = background;
    }

    const near = this.camera.near;
    const range = this.camera.far - near;
    for (let i = 0; i < this.captureFrame.length; i++) {
      const normalizedInverseDepth = this.capturePixels[i * 4] ?? 0;
      this.captureFrame[i] = near + (1 - normalizedInverseDepth) * range;
    }
    return {
      data: this.captureFrame,
      width: this.captureTarget.width,
      height: this.captureTarget.height,
    };
  }

  dispose(): void {
    this.resizeObserver.disconnect();
    this.renderer.dispose();
    this.depthMaterial.dispose();
    this.captureTarget?.dispose();
  }

  private resize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }
}
