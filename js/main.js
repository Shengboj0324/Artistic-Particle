/**
 * Particle Flow - Advanced Hand Gesture Art
 * Main Application Entry Point
 *
 * Features:
 * - 75,000+ GPU-accelerated particles with custom GLSL shaders
 * - Advanced curl noise physics for organic fluid motion
 * - Real-time hand tracking via MediaPipe (client-side ML)
 * - Post-processing: HDR bloom, depth-based effects
 * - 6 visual presets with unique physics parameters
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { HandTracker3D } from './handTracker3D.js';
import { HandDataProcessor } from './handDataProcessor.js';
import { PhysicsEngine } from './physicsEngine.js';

// Visual presets with enhanced physics parameters
const PRESETS = {
    nebula: {
        colors: [0x6366f1, 0x8b5cf6, 0xa855f7, 0xd946ef, 0x22d3ee],
        particleSize: 3.5,
        bloom: { strength: 1.4, radius: 0.7, threshold: 0.15 },
        physics: { damping: 0.985, noiseScale: 1.2, handForce: 2.8, turbulence: 0.15 }
    },
    aurora: {
        colors: [0x00ff88, 0x00d4ff, 0x22d3ee, 0x10b981, 0x6366f1],
        particleSize: 3.0,
        bloom: { strength: 1.2, radius: 0.6, threshold: 0.2 },
        physics: { damping: 0.98, noiseScale: 1.5, handForce: 2.2, turbulence: 0.2 }
    },
    fireflies: {
        colors: [0xfbbf24, 0xf59e0b, 0xfcd34d, 0x84cc16, 0xa3e635],
        particleSize: 4.5,
        bloom: { strength: 1.8, radius: 0.9, threshold: 0.08 },
        physics: { damping: 0.96, noiseScale: 2.0, handForce: 1.8, turbulence: 0.25 }
    },
    ocean: {
        colors: [0x0077b6, 0x0096c7, 0x00b4d8, 0x48cae4, 0x90e0ef],
        particleSize: 2.8,
        bloom: { strength: 0.9, radius: 0.5, threshold: 0.3 },
        physics: { damping: 0.992, noiseScale: 0.8, handForce: 3.2, turbulence: 0.1 }
    },
    galaxy: {
        colors: [0xffffff, 0xfef3c7, 0xa855f7, 0x6366f1, 0x3b82f6],
        particleSize: 2.5,
        bloom: { strength: 1.5, radius: 0.8, threshold: 0.1 },
        physics: { damping: 0.995, noiseScale: 0.5, handForce: 2.5, turbulence: 0.08 }
    },
    ember: {
        colors: [0xef4444, 0xf97316, 0xfbbf24, 0xdc2626, 0xb91c1c],
        particleSize: 3.5,
        bloom: { strength: 1.6, radius: 0.7, threshold: 0.12 },
        physics: { damping: 0.975, noiseScale: 1.8, handForce: 2.5, turbulence: 0.22 }
    }
};

class ParticleFlowApp {
    constructor() {
        // Three.js core
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.composer = null;
        this.controls = null;
        this.bloomPass = null;

        // Particle system
        this.particles = null;
        this.particleCount = 75000;
        this.positions = null;
        this.velocities = null;
        this.colors = null;
        this.sizes = null;
        this.trails = null;
        this.particleSpeeds = null; // For velocity-based coloring

        // Physics engine
        this.physics = null;

        // Hand tracking
        this.handTracker = null;
        this.handProcessor = null;
        this.handData = null;
        this.handMeshes = [];

        // Settings
        this.preset = 'nebula';
        this.mode = 'flow';
        this.intensity = 1.0;
        this.showHands = true;
        this.showTrails = true;
        this.autoRotate = false;
        this.trailLength = 0.5;

        // State
        this.isCameraActive = false;
        this.isDebugMode = false;
        this.isRunning = false;

        // Performance tracking
        this.time = 0;
        this.lastFrameTime = 0;
        this.fps = 60;
        this.frameCount = 0;
        this.lastFpsUpdate = 0;

        this.init();
    }

    async init() {
        try {
            this.updateLoadingStatus('Initializing 3D engine...');
            await this.initThreeJS();
            this.updateLoadingStatus('Initializing physics engine...');
            this.initPhysics();
            this.updateLoadingStatus('Creating particle system...');
            this.initParticles();
            this.updateLoadingStatus('Setting up post-processing...');
            this.initPostProcessing();
            this.updateLoadingStatus('Initializing hand tracking...');
            await this.initHandTracking();
            this.updateLoadingStatus('Configuring controls...');
            this.initUI();
            this.updateLoadingStatus('Ready!');
            this.hideLoadingScreen();
            this.startAnimationLoop();
        } catch (error) {
            console.error('Initialization failed:', error);
            this.updateLoadingStatus('Error: ' + error.message);
        }
    }

    updateLoadingStatus(msg) {
        const el = document.getElementById('loading-status');
        if (el) el.textContent = msg;
    }

    hideLoadingScreen() {
        setTimeout(() => {
            const s = document.getElementById('loading-screen');
            if (s) s.classList.add('hidden');
        }, 300);
    }

    initPhysics() {
        this.physics = new PhysicsEngine(this.particleCount);
        // Apply preset physics settings
        const preset = PRESETS[this.preset];
        if (preset?.physics) {
            this.physics.turbulenceStrength = preset.physics.turbulence || 0.15;
            this.physics.turbulenceScale = preset.physics.noiseScale || 1.0;
        }
    }

    async initThreeJS() {
        const container = document.getElementById('canvas-container');
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x000000);
        this.scene.fog = new THREE.FogExp2(0x000000, 0.15);
        this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
        this.camera.position.set(0, 0, 4);
        this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.2;
        container.appendChild(this.renderer.domElement);
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true; this.controls.dampingFactor = 0.05;
        this.controls.enablePan = false; this.controls.minDistance = 2; this.controls.maxDistance = 10;
        window.addEventListener('resize', () => this.handleResize());
    }

    initParticles() {
        const count = this.particleCount;
        this.positions = new Float32Array(count * 3);
        this.velocities = new Float32Array(count * 3);
        this.colors = new Float32Array(count * 3);
        this.sizes = new Float32Array(count);
        this.trails = new Float32Array(count);
        this.particleSpeeds = new Float32Array(count);

        // Initialize particles in a spherical distribution
        for (let i = 0; i < count; i++) {
            const i3 = i * 3;
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.acos(2 * Math.random() - 1);
            const r = Math.pow(Math.random(), 0.35) * 2.8; // Slightly larger, more uniform

            this.positions[i3] = r * Math.sin(phi) * Math.cos(theta);
            this.positions[i3 + 1] = r * Math.sin(phi) * Math.sin(theta);
            this.positions[i3 + 2] = r * Math.cos(phi);

            // Small initial velocities for organic motion
            this.velocities[i3] = (Math.random() - 0.5) * 0.01;
            this.velocities[i3 + 1] = (Math.random() - 0.5) * 0.01;
            this.velocities[i3 + 2] = (Math.random() - 0.5) * 0.01;

            // Varied sizes for depth perception
            this.sizes[i] = Math.random() * 1.5 + 0.5;
            this.trails[i] = Math.random();
            this.particleSpeeds[i] = 0;
        }

        this.updateColors();

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));
        geometry.setAttribute('size', new THREE.BufferAttribute(this.sizes, 1));

        // Enhanced GLSL shaders for beautiful particle rendering
        const vertexShader = `
            attribute float size;
            varying vec3 vColor;
            varying float vAlpha;
            varying float vDepth;
            uniform float uPixelRatio;
            uniform float uBaseSize;
            uniform float uTime;

            void main() {
                vColor = color;
                vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                float depth = -mvPosition.z;
                vDepth = depth;

                // Size attenuation with depth
                float sizeAtten = 350.0 / max(depth, 0.1);
                float pulse = 1.0 + 0.1 * sin(uTime * 2.0 + position.x * 3.0);
                gl_PointSize = clamp(size * uBaseSize * uPixelRatio * sizeAtten * pulse, 1.0, 100.0);

                // Alpha based on depth for atmospheric perspective
                vAlpha = smoothstep(12.0, 1.5, depth) * 0.95;

                gl_Position = projectionMatrix * mvPosition;
            }
        `;

        const fragmentShader = `
            varying vec3 vColor;
            varying float vAlpha;
            varying float vDepth;

            void main() {
                vec2 center = gl_PointCoord - vec2(0.5);
                float dist = length(center);

                // Discard pixels outside circle
                if (dist > 0.5) discard;

                // Soft circular gradient with glow
                float coreFalloff = 1.0 - smoothstep(0.0, 0.25, dist);
                float glowFalloff = 1.0 - smoothstep(0.0, 0.5, dist);

                // Core is brighter, glow extends outward
                float alpha = (coreFalloff * 0.8 + glowFalloff * 0.4) * vAlpha;

                // Color enhancement - brighter core
                vec3 finalColor = vColor * (1.0 + coreFalloff * 0.6);

                // Subtle depth-based color shift (atmospheric)
                float depthFactor = smoothstep(1.0, 8.0, vDepth);
                finalColor = mix(finalColor, finalColor * 0.7, depthFactor * 0.3);

                gl_FragColor = vec4(finalColor, alpha);
            }
        `;

        const material = new THREE.ShaderMaterial({
            uniforms: {
                uTime: { value: 0 },
                uPixelRatio: { value: Math.min(window.devicePixelRatio, 2) },
                uBaseSize: { value: PRESETS[this.preset].particleSize }
            },
            vertexShader,
            fragmentShader,
            transparent: true,
            depthWrite: false,
            blending: THREE.AdditiveBlending,
            vertexColors: true
        });

        this.particles = new THREE.Points(geometry, material);
        this.scene.add(this.particles);
    }

    initPostProcessing() {
        const preset = PRESETS[this.preset];
        this.composer = new EffectComposer(this.renderer);
        this.composer.addPass(new RenderPass(this.scene, this.camera));
        this.bloomPass = new UnrealBloomPass(new THREE.Vector2(window.innerWidth, window.innerHeight), preset.bloom.strength, preset.bloom.radius, preset.bloom.threshold);
        this.composer.addPass(this.bloomPass);
    }

    async initHandTracking() {
        this.handProcessor = new HandDataProcessor();
        this.handTracker = new HandTracker3D({ maxHands: 2, modelComplexity: 1, minDetectionConfidence: 0.7, minTrackingConfidence: 0.6 });
        this.handTracker.onResults = (results) => this.handleHandResults(results);
        await this.handTracker.initialize();
        this.createHandMeshes();
    }

    createHandMeshes() {
        const jointGeo = new THREE.SphereGeometry(0.015, 16, 16);
        const jointMat = new THREE.MeshBasicMaterial({ color: 0x22d3ee, transparent: true, opacity: 0.8 });
        const lineMat = new THREE.LineBasicMaterial({ color: 0x6366f1, transparent: true, opacity: 0.6 });
        const connections = [[0,1],[1,2],[2,3],[3,4],[0,5],[5,6],[6,7],[7,8],[0,9],[9,10],[10,11],[11,12],[0,13],[13,14],[14,15],[15,16],[0,17],[17,18],[18,19],[19,20],[5,9],[9,13],[13,17]];
        for (let h = 0; h < 2; h++) {
            const group = new THREE.Group(); group.visible = false;
            const joints = []; for (let j = 0; j < 21; j++) { const joint = new THREE.Mesh(jointGeo, jointMat.clone()); joints.push(joint); group.add(joint); }
            const lines = []; connections.forEach(() => { const lineGeo = new THREE.BufferGeometry(); lineGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3)); const line = new THREE.Line(lineGeo, lineMat.clone()); lines.push(line); group.add(line); });
            this.handMeshes.push({ group, joints, lines, connections }); this.scene.add(group);
        }
    }

    handleHandResults(results) {
        this.handData = this.handProcessor.process(results);
        if (this.showHands) this.updateHandMeshes();
        this.updateHandUI();
        if (this.isDebugMode && this.handTracker) {
            const debugCanvas = document.getElementById('debug-canvas');
            this.handTracker.drawDebug(debugCanvas, results);
        }
    }

    updateHandMeshes() {
        this.handMeshes.forEach(h => h.group.visible = false);
        if (!this.handData?.hands?.length) return;
        this.handData.hands.forEach((hand, idx) => {
            if (idx >= 2) return;
            const mesh = this.handMeshes[idx]; mesh.group.visible = true;
            hand.landmarks.forEach((lm, j) => { mesh.joints[j].position.set(lm.x, lm.y, lm.z); mesh.joints[j].scale.setScalar([4,8,12,16,20].includes(j) ? 1.5 : 1.0); });
            mesh.connections.forEach((conn, i) => {
                const p1 = hand.landmarks[conn[0]], p2 = hand.landmarks[conn[1]], pos = mesh.lines[i].geometry.attributes.position.array;
                pos[0] = p1.x; pos[1] = p1.y; pos[2] = p1.z; pos[3] = p2.x; pos[4] = p2.y; pos[5] = p2.z;
                mesh.lines[i].geometry.attributes.position.needsUpdate = true;
            });
        });
    }

    updateHandUI() {
        const leftBadge = document.getElementById('left-hand-badge'), rightBadge = document.getElementById('right-hand-badge'), handCount = document.getElementById('hand-count');
        let leftHand = null, rightHand = null;
        if (this.handData?.hands) this.handData.hands.forEach(h => { if (h.isLeft) leftHand = h; else rightHand = h; });
        if (leftBadge) { leftBadge.classList.toggle('active', !!leftHand); const s = leftBadge.querySelector('.hand-status'); if (s) s.textContent = leftHand ? ((leftHand.metrics?.openness * 100 || 0).toFixed(0) + '%') : '—'; }
        if (rightBadge) { rightBadge.classList.toggle('active', !!rightHand); const s = rightBadge.querySelector('.hand-status'); if (s) s.textContent = rightHand ? ((rightHand.metrics?.openness * 100 || 0).toFixed(0) + '%') : '—'; }
        if (handCount) handCount.textContent = this.handData?.handCount || 0;
    }

    updateColors() {
        const colorHexes = PRESETS[this.preset].colors;
        const colors = colorHexes.map(hex => new THREE.Color(hex));

        for (let i = 0; i < this.particleCount; i++) {
            const i3 = i * 3;
            const t = this.trails[i];
            const speed = this.particleSpeeds ? this.particleSpeeds[i] : 0;

            // Base color from trail position
            const idx = Math.floor(t * (colors.length - 1));
            const nextIdx = Math.min(idx + 1, colors.length - 1);
            const blend = (t * (colors.length - 1)) - idx;

            const c1 = colors[idx], c2 = colors[nextIdx];
            let r = c1.r + (c2.r - c1.r) * blend;
            let g = c1.g + (c2.g - c1.g) * blend;
            let b = c1.b + (c2.b - c1.b) * blend;

            // Velocity-based color boost (faster = brighter/whiter)
            const speedBoost = Math.min(speed * 0.5, 0.4);
            r = Math.min(r + speedBoost, 1.0);
            g = Math.min(g + speedBoost, 1.0);
            b = Math.min(b + speedBoost, 1.0);

            this.colors[i3] = r;
            this.colors[i3 + 1] = g;
            this.colors[i3 + 2] = b;
        }

        if (this.particles?.geometry?.attributes?.color) {
            this.particles.geometry.attributes.color.needsUpdate = true;
        }
    }

    updatePhysics(dt) {
        if (!this.physics) return;

        const preset = PRESETS[this.preset];
        const count = this.particleCount;

        // Update physics engine time
        this.physics.update(dt);

        // Update turbulence settings from preset
        this.physics.turbulenceStrength = preset.physics.turbulence || 0.15;
        this.physics.turbulenceScale = preset.physics.noiseScale || 1.0;

        // Update each particle using the physics engine
        let maxSpeed = 0;
        for (let i = 0; i < count; i++) {
            const i3 = i * 3;
            const speed = this.physics.updateParticle(
                i3,
                this.positions,
                this.velocities,
                dt,
                this.handData,
                this.mode,
                this.intensity,
                preset
            );

            // Track speed for color modulation
            if (this.particleSpeeds) {
                this.particleSpeeds[i] = speed;
            }
            if (speed > maxSpeed) maxSpeed = speed;
        }

        // Update colors based on velocity (every few frames for performance)
        if (this.frameCount % 3 === 0 && maxSpeed > 0.1) {
            this.updateColors();
        }

        // Mark position buffer for GPU update
        if (this.particles?.geometry?.attributes?.position) {
            this.particles.geometry.attributes.position.needsUpdate = true;
        }
    }

    startAnimationLoop() { this.isRunning = true; this.lastFrameTime = performance.now(); this.animate(); }

    animate() {
        if (!this.isRunning) return;
        requestAnimationFrame(() => this.animate());
        const now = performance.now(), dt = Math.min((now - this.lastFrameTime) / 1000, 0.05);
        this.lastFrameTime = now; this.time += dt;
        this.frameCount++;
        if (now - this.lastFpsUpdate >= 500) {
            this.fps = Math.round((this.frameCount * 1000) / (now - this.lastFpsUpdate)); this.frameCount = 0; this.lastFpsUpdate = now;
            const fpsEl = document.getElementById('fps-value'); if (fpsEl) fpsEl.textContent = this.fps;
            const particleEl = document.getElementById('particle-count'); if (particleEl) particleEl.textContent = (this.particleCount / 1000).toFixed(0) + 'K';
        }
        if (this.particles?.material?.uniforms) this.particles.material.uniforms.uTime.value = this.time;
        this.updatePhysics(dt);
        this.controls.autoRotate = this.autoRotate; this.controls.update();
        this.composer.render();
    }

    handleResize() {
        const w = window.innerWidth, h = window.innerHeight;
        this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h); this.composer.setSize(w, h);
        if (this.particles?.material?.uniforms?.uPixelRatio) this.particles.material.uniforms.uPixelRatio.value = Math.min(window.devicePixelRatio, 2);
    }

    async toggleCamera() { if (this.isCameraActive) await this.stopCamera(); else await this.startCamera(); return this.isCameraActive; }

    async startCamera() {
        const video = document.getElementById('camera-feed');
        const stream = await navigator.mediaDevices.getUserMedia({ video: { width: { ideal: 1280 }, height: { ideal: 720 }, facingMode: 'user' } });
        video.srcObject = stream; await video.play(); await this.handTracker.start(video); this.isCameraActive = true;
    }

    async stopCamera() {
        const video = document.getElementById('camera-feed');
        if (video?.srcObject) { video.srcObject.getTracks().forEach(t => t.stop()); video.srcObject = null; }
        this.handTracker?.stop(); this.handProcessor?.reset(); this.handData = null;
        this.handMeshes.forEach(h => h.group.visible = false); this.isCameraActive = false;
    }

    toggleDebug() { this.isDebugMode = !this.isDebugMode; const c = document.getElementById('debug-canvas'); if (c) c.classList.toggle('visible', this.isDebugMode); return this.isDebugMode; }

    setPreset(name) {
        if (!PRESETS[name]) return;
        this.preset = name;
        this.updateColors();

        const preset = PRESETS[name];

        // Update bloom settings
        if (this.bloomPass) {
            this.bloomPass.strength = preset.bloom.strength;
            this.bloomPass.radius = preset.bloom.radius;
            this.bloomPass.threshold = preset.bloom.threshold;
        }

        // Update particle size
        if (this.particles?.material?.uniforms?.uBaseSize) {
            this.particles.material.uniforms.uBaseSize.value = preset.particleSize;
        }

        // Update physics engine settings
        if (this.physics && preset.physics) {
            this.physics.turbulenceStrength = preset.physics.turbulence || 0.15;
            this.physics.turbulenceScale = preset.physics.noiseScale || 1.0;
            this.physics.globalDamping = preset.physics.damping || 0.98;
        }
    }

    setMode(mode) { this.mode = mode; }
    setIntensity(v) { this.intensity = v / 100; }
    setShowHands(show) { this.showHands = show; if (!show) this.handMeshes.forEach(h => h.group.visible = false); }
    setAutoRotate(r) { this.autoRotate = r; }

    setParticleCount(count) {
        if (count === this.particleCount) return;
        this.particleCount = count;

        // Clean up old particles
        if (this.particles) {
            this.scene.remove(this.particles);
            this.particles.geometry.dispose();
            this.particles.material.dispose();
        }

        // Reinitialize physics engine with new particle count
        this.physics = new PhysicsEngine(count);
        const preset = PRESETS[this.preset];
        if (preset?.physics) {
            this.physics.turbulenceStrength = preset.physics.turbulence || 0.15;
            this.physics.turbulenceScale = preset.physics.noiseScale || 1.0;
            this.physics.globalDamping = preset.physics.damping || 0.98;
        }

        // Create new particles
        this.initParticles();
    }

    initUI() {
        // Camera toggle button
        const cameraBtn = document.getElementById('toggle-camera');
        if (cameraBtn) {
            cameraBtn.addEventListener('click', async () => {
                const active = await this.toggleCamera();
                cameraBtn.classList.toggle('active', active);
                const span = cameraBtn.querySelector('span');
                if (span) span.textContent = active ? 'Stop Camera' : 'Start Camera';
            });
        }

        // Debug toggle button
        const debugBtn = document.getElementById('toggle-debug');
        if (debugBtn) {
            debugBtn.addEventListener('click', () => {
                const active = this.toggleDebug();
                debugBtn.classList.toggle('active', active);
            });
        }

        // Panel collapse toggle
        const panelToggle = document.getElementById('toggle-panel');
        if (panelToggle) {
            panelToggle.addEventListener('click', () => {
                document.getElementById('control-panel')?.classList.toggle('collapsed');
            });
        }

        // Preset buttons
        document.querySelectorAll('.preset-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                document.querySelectorAll('.preset-btn').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.setPreset(btn.dataset.preset);
            });
        });

        // Mode chips
        document.querySelectorAll('.mode-chip').forEach(btn => {
            btn.addEventListener('click', () => {
                document.querySelectorAll('.mode-chip').forEach(b => b.classList.remove('active'));
                btn.classList.add('active');
                this.setMode(btn.dataset.mode);
            });
        });

        // Density slider
        const densitySlider = document.getElementById('density-slider');
        const densityValue = document.getElementById('density-value');
        if (densitySlider) {
            densitySlider.addEventListener('input', (e) => {
                const count = parseInt(e.target.value);
                if (densityValue) densityValue.textContent = (count / 1000).toFixed(0) + 'K';
                this.setParticleCount(count);
            });
        }

        // Intensity slider
        const intensitySlider = document.getElementById('intensity-slider');
        const intensityValue = document.getElementById('intensity-value');
        if (intensitySlider) {
            intensitySlider.addEventListener('input', (e) => {
                if (intensityValue) intensityValue.textContent = e.target.value + '%';
                this.setIntensity(parseInt(e.target.value));
            });
        }

        // Trail slider
        const trailSlider = document.getElementById('trail-slider');
        const trailValue = document.getElementById('trail-value');
        if (trailSlider) {
            trailSlider.addEventListener('input', (e) => {
                const v = parseInt(e.target.value);
                if (trailValue) trailValue.textContent = v < 33 ? 'Short' : v < 66 ? 'Medium' : 'Long';
                this.trailLength = v / 100;
            });
        }

        // Toggle checkboxes
        const showHandsCheckbox = document.getElementById('show-hands');
        if (showHandsCheckbox) {
            showHandsCheckbox.addEventListener('change', (e) => this.setShowHands(e.target.checked));
        }

        const showTrailsCheckbox = document.getElementById('show-trails');
        if (showTrailsCheckbox) {
            showTrailsCheckbox.addEventListener('change', (e) => {
                // Trail visibility is controlled by the particle material blending
                // For now, we just store the preference
                this.showTrails = e.target.checked;
            });
        }

        const autoRotateCheckbox = document.getElementById('auto-rotate');
        if (autoRotateCheckbox) {
            autoRotateCheckbox.addEventListener('change', (e) => this.setAutoRotate(e.target.checked));
        }

        // Dismiss instructions
        const dismissBtn = document.getElementById('dismiss-instructions');
        if (dismissBtn) {
            dismissBtn.addEventListener('click', () => {
                document.getElementById('instructions-overlay')?.classList.add('hidden');
            });
        }
    }
}

document.addEventListener('DOMContentLoaded', () => { window.app = new ParticleFlowApp(); });
document.addEventListener('visibilitychange', () => { if (window.app) { if (document.hidden) window.app.isRunning = false; else { window.app.isRunning = true; window.app.lastFrameTime = performance.now(); window.app.animate(); } } });
export { ParticleFlowApp };

