/**
 * Physics Engine for Particle System
 *
 * Particles are FREE-FLOWING. They are attracted to hands like iron filings to magnets.
 * No pre-assignment. Pure physics-based attraction creates natural hand shapes.
 */

export class PhysicsEngine {
    constructor(particleCount) {
        this.particleCount = particleCount;
        this.time = 0;

        // Noise parameters
        this.noiseOctaves = 3;
        this.noiseLacunarity = 2.0;
        this.noisePersistence = 0.5;

        // Physics constants
        this.globalDamping = 0.985;
        this.maxSpeed = 3.0;
        this.boundaryRadius = 4.0;
        this.boundaryStiffness = 0.2;

        // Ambient turbulence (when no hand)
        this.turbulenceStrength = 0.12;
        this.turbulenceScale = 0.6;

        // HAND ATTRACTION - particles flow toward hand naturally
        this.attractionRadius = 2.5;       // How far attraction reaches
        this.attractionStrength = 4.0;     // Pull strength toward landmarks
        this.clusterRadius = 0.06;         // How close particles cluster
        this.surfaceRepulsion = 2.0;       // Prevents particles from overlapping

        // Precomputed noise
        this.gradients = this.generateGradients();
        this.permutation = this.generatePermutation();
    }
    
    generateGradients() {
        const grads = [];
        for (let i = 0; i < 256; i++) {
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.acos(2 * Math.random() - 1);
            grads.push({
                x: Math.sin(phi) * Math.cos(theta),
                y: Math.sin(phi) * Math.sin(theta),
                z: Math.cos(phi)
            });
        }
        return grads;
    }
    
    generatePermutation() {
        const p = [];
        for (let i = 0; i < 256; i++) p[i] = i;
        for (let i = 255; i > 0; i--) {
            const j = Math.floor(Math.random() * (i + 1));
            [p[i], p[j]] = [p[j], p[i]];
        }
        return [...p, ...p]; // Double for wrapping
    }
    
    // Smooth interpolation function
    fade(t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }
    
    lerp(a, b, t) {
        return a + t * (b - a);
    }
    
    // 3D Perlin noise
    noise3D(x, y, z) {
        const X = Math.floor(x) & 255;
        const Y = Math.floor(y) & 255;
        const Z = Math.floor(z) & 255;
        
        x -= Math.floor(x);
        y -= Math.floor(y);
        z -= Math.floor(z);
        
        const u = this.fade(x);
        const v = this.fade(y);
        const w = this.fade(z);
        
        const p = this.permutation;
        const g = this.gradients;
        
        const A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z;
        const B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;
        
        const dot = (grad, x, y, z) => grad.x * x + grad.y * y + grad.z * z;
        
        return this.lerp(
            this.lerp(
                this.lerp(dot(g[p[AA] & 255], x, y, z), dot(g[p[BA] & 255], x - 1, y, z), u),
                this.lerp(dot(g[p[AB] & 255], x, y - 1, z), dot(g[p[BB] & 255], x - 1, y - 1, z), u),
                v
            ),
            this.lerp(
                this.lerp(dot(g[p[AA + 1] & 255], x, y, z - 1), dot(g[p[BA + 1] & 255], x - 1, y, z - 1), u),
                this.lerp(dot(g[p[AB + 1] & 255], x, y - 1, z - 1), dot(g[p[BB + 1] & 255], x - 1, y - 1, z - 1), u),
                v
            ),
            w
        );
    }
    
    // Fractal Brownian Motion noise
    fbm(x, y, z) {
        let value = 0;
        let amplitude = 1;
        let frequency = 1;
        let maxValue = 0;
        
        for (let i = 0; i < this.noiseOctaves; i++) {
            value += amplitude * this.noise3D(x * frequency, y * frequency, z * frequency);
            maxValue += amplitude;
            amplitude *= this.noisePersistence;
            frequency *= this.noiseLacunarity;
        }
        
        return value / maxValue;
    }
    
    // Curl noise for divergence-free turbulence (incompressible flow)
    // Uses the curl of a potential field to create smooth, swirling motion
    curlNoise(x, y, z, t) {
        const eps = 0.001; // Slightly larger epsilon for numerical stability
        const scale = this.turbulenceScale;
        const evolution = t * this.turbulenceEvolution;

        // We need three independent noise fields to compute curl properly
        // Curl = ∇ × F where F = (Fx, Fy, Fz) are three noise functions

        // Sample noise for computing partial derivatives
        // For curl, we need: ∂Fz/∂y - ∂Fy/∂z, ∂Fx/∂z - ∂Fz/∂x, ∂Fy/∂x - ∂Fx/∂y

        const sx = x * scale, sy = y * scale, sz = z * scale + evolution;

        // Noise field 1 (used for Fx component)
        const n1_py = this.fbm(sx, sy + eps, sz);
        const n1_my = this.fbm(sx, sy - eps, sz);
        const n1_pz = this.fbm(sx, sy, sz + eps);
        const n1_mz = this.fbm(sx, sy, sz - eps);

        // Noise field 2 (used for Fy component) - offset to make independent
        const n2_px = this.fbm(sx + eps + 100, sy + 100, sz + 100);
        const n2_mx = this.fbm(sx - eps + 100, sy + 100, sz + 100);
        const n2_pz = this.fbm(sx + 100, sy + 100, sz + eps + 100);
        const n2_mz = this.fbm(sx + 100, sy + 100, sz - eps + 100);

        // Noise field 3 (used for Fz component) - offset to make independent
        const n3_px = this.fbm(sx + eps + 200, sy + 200, sz + 200);
        const n3_mx = this.fbm(sx - eps + 200, sy + 200, sz + 200);
        const n3_py = this.fbm(sx + 200, sy + eps + 200, sz + 200);
        const n3_my = this.fbm(sx + 200, sy - eps + 200, sz + 200);

        // Compute partial derivatives
        const dFz_dy = (n3_py - n3_my) / (2 * eps);
        const dFy_dz = (n2_pz - n2_mz) / (2 * eps);
        const dFx_dz = (n1_pz - n1_mz) / (2 * eps);
        const dFz_dx = (n3_px - n3_mx) / (2 * eps);
        const dFy_dx = (n2_px - n2_mx) / (2 * eps);
        const dFx_dy = (n1_py - n1_my) / (2 * eps);

        // Curl = (∂Fz/∂y - ∂Fy/∂z, ∂Fx/∂z - ∂Fz/∂x, ∂Fy/∂x - ∂Fx/∂y)
        const curlX = dFz_dy - dFy_dz;
        const curlY = dFx_dz - dFz_dx;
        const curlZ = dFy_dx - dFx_dy;

        return { x: curlX, y: curlY, z: curlZ };
    }

    /**
     * Calculate attraction force from ALL hand landmarks to this particle.
     * Every particle is FREE - attracted to the nearest/strongest landmark.
     * This creates natural clustering around hand shape.
     */
    calculateHandAttraction(px, py, pz, handData, intensity) {
        if (!handData?.hands?.length) return { x: 0, y: 0, z: 0, nearHand: false };

        let fx = 0, fy = 0, fz = 0;
        let minDist = Infinity;
        let nearHand = false;

        for (const hand of handData.hands) {
            if (!hand.landmarks) continue;

            // Each landmark acts as an attractor
            for (let i = 0; i < hand.landmarks.length; i++) {
                const lm = hand.landmarks[i];
                const dx = lm.x - px;
                const dy = lm.y - py;
                const dz = lm.z - pz;
                const distSq = dx * dx + dy * dy + dz * dz;
                const dist = Math.sqrt(distSq);

                if (dist < minDist) minDist = dist;

                // Only attract within radius
                if (dist > this.attractionRadius || dist < 0.001) continue;

                nearHand = true;

                // Attraction strength: inverse square with soft falloff
                // Stronger when closer, but with repulsion at very close range
                let strength;
                if (dist < this.clusterRadius) {
                    // Very close: gentle repulsion to prevent collapse
                    strength = -this.surfaceRepulsion * (1 - dist / this.clusterRadius);
                } else if (dist < this.clusterRadius * 3) {
                    // Sweet spot: particles cluster here
                    strength = this.attractionStrength * 0.5;
                } else {
                    // Far: strong attraction, falls off with distance
                    const t = 1 - (dist - this.clusterRadius * 3) / (this.attractionRadius - this.clusterRadius * 3);
                    strength = this.attractionStrength * t * t * intensity;
                }

                // Accumulate force toward this landmark
                const nx = dx / dist;
                const ny = dy / dist;
                const nz = dz / dist;
                fx += nx * strength;
                fy += ny * strength;
                fz += nz * strength;
            }
        }

        return { x: fx, y: fy, z: fz, nearHand, minDist };
    }

    /**
     * Calculate mode-specific effects (flow, vortex, etc.)
     */
    calculateModeEffect(px, py, pz, vx, vy, vz, handData, mode, intensity) {
        if (!handData?.hands?.length) return { x: 0, y: 0, z: 0 };

        let fx = 0, fy = 0, fz = 0;

        for (const hand of handData.hands) {
            if (!hand.landmarks) continue;

            // Use palm center for mode effects
            const palm = hand.palm?.center || hand.landmarks[0];
            const dx = palm.x - px;
            const dy = palm.y - py;
            const dz = palm.z - pz;
            const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);

            if (dist > this.attractionRadius * 1.5 || dist < 0.01) continue;

            const falloff = Math.pow(1 - dist / (this.attractionRadius * 1.5), 2);
            const str = intensity * falloff;

            switch (mode) {
                case 'flow':
                    // Swirl around palm
                    fx += (-dy * 0.5 + (hand.palm?.normal?.x || 0) * 0.3) * str;
                    fy += (dx * 0.5 + (hand.palm?.normal?.y || 0) * 0.3) * str;
                    fz += (hand.palm?.normal?.z || 0) * 0.3 * str;
                    break;

                case 'attract':
                    // Extra pull (already handled by main attraction)
                    fx += dx / dist * str * 0.5;
                    fy += dy / dist * str * 0.5;
                    fz += dz / dist * str * 0.5;
                    break;

                case 'repel':
                    // Push outward from palm
                    fx -= dx / dist * str * 2;
                    fy -= dy / dist * str * 2;
                    fz -= dz / dist * str * 2;
                    break;

                case 'vortex':
                    // Spiral tornado effect
                    const angle = Math.atan2(dy, dx) + this.time * 2;
                    const r = Math.sqrt(dx * dx + dy * dy);
                    fx += (-Math.sin(angle) * r * 0.5 - dx * 0.1) * str;
                    fy += (Math.cos(angle) * r * 0.5 - dy * 0.1) * str;
                    fz += (0.3 - dz * 0.2) * str;
                    break;

                case 'sculpt':
                    // Form tight surface around hand
                    const targetDist = 0.15;
                    if (dist < targetDist) {
                        fx -= dx / dist * str * 3;
                        fy -= dy / dist * str * 3;
                        fz -= dz / dist * str * 3;
                    } else if (dist < targetDist * 2) {
                        fx += dx / dist * str;
                        fy += dy / dist * str;
                        fz += dz / dist * str;
                    }
                    break;
            }
        }

        return { x: fx, y: fy, z: fz };
    }

    /**
     * Main particle update - pure physics, no pre-assignment
     */
    updateParticle(i3, positions, velocities, dt, handData, mode, intensity, preset) {
        let px = positions[i3], py = positions[i3 + 1], pz = positions[i3 + 2];
        let vx = velocities[i3], vy = velocities[i3 + 1], vz = velocities[i3 + 2];

        // 1. Hand attraction - particles flow toward hand naturally
        const attraction = this.calculateHandAttraction(px, py, pz, handData, intensity);
        const handForce = (preset?.physics?.handForce || 1) * intensity;
        vx += attraction.x * handForce * dt * 50;
        vy += attraction.y * handForce * dt * 50;
        vz += attraction.z * handForce * dt * 50;

        // 2. Mode-specific effects
        const modeEffect = this.calculateModeEffect(px, py, pz, vx, vy, vz, handData, mode, intensity);
        vx += modeEffect.x * dt * 30;
        vy += modeEffect.y * dt * 30;
        vz += modeEffect.z * dt * 30;

        // 3. Ambient turbulence (reduced when near hand for cleaner shape)
        const turbReduction = attraction.nearHand ? 0.2 : 1.0;
        const curl = this.curlNoise(px, py, pz, this.time);
        const turbStr = this.turbulenceStrength * (preset?.physics?.noiseScale || 1) * turbReduction;
        vx += curl.x * turbStr * dt * 60;
        vy += curl.y * turbStr * dt * 60;
        vz += curl.z * turbStr * dt * 60;

        // 4. Soft boundary
        const dist = Math.sqrt(px * px + py * py + pz * pz);
        if (dist > this.boundaryRadius) {
            const push = (dist - this.boundaryRadius) * this.boundaryStiffness;
            vx -= (px / dist) * push * dt * 60;
            vy -= (py / dist) * push * dt * 60;
            vz -= (pz / dist) * push * dt * 60;
        }

        // 5. Damping (less when near hand for responsiveness)
        const damping = attraction.nearHand ? 0.96 : (preset?.physics?.damping || this.globalDamping);
        vx *= damping;
        vy *= damping;
        vz *= damping;

        // 6. Speed limit
        const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        if (speed > this.maxSpeed) {
            const s = this.maxSpeed / speed;
            vx *= s; vy *= s; vz *= s;
        }

        // 7. Update position
        positions[i3] = px + vx * dt * 60;
        positions[i3 + 1] = py + vy * dt * 60;
        positions[i3 + 2] = pz + vz * dt * 60;
        velocities[i3] = vx;
        velocities[i3 + 1] = vy;
        velocities[i3 + 2] = vz;

        return speed;
    }

    update(dt) {
        this.time += dt;
    }
}
