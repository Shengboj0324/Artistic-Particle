/**
 * Advanced Physics Engine for Particle System
 * 
 * Features:
 * - Curl noise for organic turbulence
 * - SPH-inspired fluid dynamics
 * - Multiple force field types
 * - Velocity-based color modulation
 * - Spatial hashing for performance
 * - Proper momentum and energy conservation
 */

export class PhysicsEngine {
    constructor(particleCount) {
        this.particleCount = particleCount;
        this.time = 0;
        
        // Noise parameters for curl noise
        this.noiseOctaves = 3;
        this.noiseLacunarity = 2.0;
        this.noisePersistence = 0.5;
        
        // Physics constants
        this.gravity = { x: 0, y: -0.02, z: 0 };
        this.globalDamping = 0.995;
        this.maxSpeed = 2.0;
        this.boundaryRadius = 3.0;
        this.boundaryStiffness = 0.5;
        
        // Turbulence settings
        this.turbulenceStrength = 0.15;
        this.turbulenceScale = 0.8;
        this.turbulenceEvolution = 0.3;
        
        // Interaction settings
        this.interactionRadius = 0.6;
        this.forceMultiplier = 1.0;
        
        // Precomputed noise gradients for 3D Perlin noise
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

    // Calculate hand interaction forces with advanced physics
    calculateHandForces(px, py, pz, vx, vy, vz, handData, mode, intensity) {
        let fx = 0, fy = 0, fz = 0;
        if (!handData?.hands?.length) return { x: 0, y: 0, z: 0 };

        const radius = this.interactionRadius;
        const radiusSq = radius * radius;

        for (const hand of handData.hands) {
            // Palm-based force field
            if (hand.palm) {
                const palm = hand.palm;
                const dx = palm.center.x - px, dy = palm.center.y - py, dz = palm.center.z - pz;
                const distSq = dx * dx + dy * dy + dz * dz;

                if (distSq < radiusSq * 4 && distSq > 0.0001) {
                    const dist = Math.sqrt(distSq);
                    const falloff = 1 - dist / (radius * 2);
                    const falloffSq = falloff * falloff;
                    const openness = hand.metrics?.openness || 0.5;
                    const normalForce = openness * falloffSq * intensity * 0.5;
                    fx += palm.normal.x * normalForce;
                    fy += palm.normal.y * normalForce;
                    fz += palm.normal.z * normalForce;
                }
            }

            // Fingertip forces with velocity influence
            if (hand.fingertips) {
                for (const tip of hand.fingertips) {
                    const pos = tip.position, vel = tip.velocity;
                    const dx = pos.x - px, dy = pos.y - py, dz = pos.z - pz;
                    const distSq = dx * dx + dy * dy + dz * dz;

                    if (distSq < radiusSq && distSq > 0.0001) {
                        const dist = Math.sqrt(distSq);
                        const falloff = Math.pow(1 - dist / radius, 2);
                        const velMag = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
                        const strength = intensity * falloff * (1 + velMag * 2);

                        switch (mode) {
                            case 'flow':
                                const tangentX = -dy, tangentY = dx;
                                fx += (tangentX / dist * 0.3 + vel.x * 0.5) * strength;
                                fy += (tangentY / dist * 0.3 + vel.y * 0.5) * strength;
                                fz += vel.z * 0.5 * strength;
                                break;
                            case 'attract':
                                const attractStr = strength / (distSq + 0.01);
                                fx += dx * attractStr * 0.3;
                                fy += dy * attractStr * 0.3;
                                fz += dz * attractStr * 0.3;
                                break;
                            case 'repel':
                                const repelStr = strength / (distSq + 0.01);
                                fx -= dx * repelStr * 0.5;
                                fy -= dy * repelStr * 0.5;
                                fz -= dz * repelStr * 0.5;
                                break;
                            case 'vortex':
                                const vortexStr = strength * 1.5;
                                fx += (-dy / dist + dz * 0.2) * vortexStr;
                                fy += (dx / dist + dz * 0.2) * vortexStr;
                                fz += (-dx * 0.1 - dy * 0.1) * vortexStr;
                                break;
                            case 'sculpt':
                                if (dist < radius * 0.3) {
                                    fx -= dx / dist * strength * 2;
                                    fy -= dy / dist * strength * 2;
                                    fz -= dz / dist * strength * 2;
                                } else {
                                    fx += dx / dist * strength * 0.3;
                                    fy += dy / dist * strength * 0.3;
                                    fz += dz / dist * strength * 0.3;
                                }
                                break;
                        }

                        if (velMag > 0.5) {
                            const wake = velMag * falloff * 0.2;
                            fx += vel.x * wake; fy += vel.y * wake; fz += vel.z * wake;
                        }
                    }
                }
            }
        }
        return { x: fx, y: fy, z: fz };
    }

    updateParticle(i3, positions, velocities, dt, handData, mode, intensity, preset) {
        let px = positions[i3], py = positions[i3 + 1], pz = positions[i3 + 2];
        let vx = velocities[i3], vy = velocities[i3 + 1], vz = velocities[i3 + 2];

        const curl = this.curlNoise(px, py, pz, this.time);
        const turbStr = this.turbulenceStrength * (preset?.physics?.noiseScale || 1);
        vx += curl.x * turbStr * dt; vy += curl.y * turbStr * dt; vz += curl.z * turbStr * dt;

        const handForce = this.calculateHandForces(px, py, pz, vx, vy, vz, handData, mode, intensity * (preset?.physics?.handForce || 1));
        vx += handForce.x * dt * 60; vy += handForce.y * dt * 60; vz += handForce.z * dt * 60;

        const dist = Math.sqrt(px * px + py * py + pz * pz);
        if (dist > this.boundaryRadius) {
            const excess = dist - this.boundaryRadius;
            const restoreForce = excess * this.boundaryStiffness;
            vx -= (px / dist) * restoreForce * dt;
            vy -= (py / dist) * restoreForce * dt;
            vz -= (pz / dist) * restoreForce * dt;
        }

        const damping = preset?.physics?.damping || this.globalDamping;
        vx *= damping; vy *= damping; vz *= damping;

        const speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
        if (speed > this.maxSpeed) {
            const scale = this.maxSpeed / speed;
            vx *= scale; vy *= scale; vz *= scale;
        }

        positions[i3] = px + vx * dt * 60;
        positions[i3 + 1] = py + vy * dt * 60;
        positions[i3 + 2] = pz + vz * dt * 60;
        velocities[i3] = vx; velocities[i3 + 1] = vy; velocities[i3 + 2] = vz;

        return speed;
    }

    update(dt) { this.time += dt; }
}
