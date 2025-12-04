/**
 * Hand Data Processor - Continuous 3D hand data processing
 */
export class HandDataProcessor {
    constructor() {
        this.LANDMARKS = {
            WRIST: 0, THUMB_CMC: 1, THUMB_MCP: 2, THUMB_IP: 3, THUMB_TIP: 4,
            INDEX_MCP: 5, INDEX_PIP: 6, INDEX_DIP: 7, INDEX_TIP: 8,
            MIDDLE_MCP: 9, MIDDLE_PIP: 10, MIDDLE_DIP: 11, MIDDLE_TIP: 12,
            RING_MCP: 13, RING_PIP: 14, RING_DIP: 15, RING_TIP: 16,
            PINKY_MCP: 17, PINKY_PIP: 18, PINKY_DIP: 19, PINKY_TIP: 20
        };
        this.FINGERTIPS = [4, 8, 12, 16, 20];
        this.history = new Map();
        this.historyMaxFrames = 5;
    }

    process(results) {
        if (!results?.multiHandLandmarks?.length) {
            return { hands: [], handCount: 0, totalLandmarks: 0, maxVelocity: 0, timestamp: performance.now() };
        }
        const timestamp = results.timestamp || performance.now();
        const hands = [];
        let maxVelocity = 0;

        for (let i = 0; i < results.multiHandLandmarks.length; i++) {
            const rawLandmarks = results.multiHandLandmarks[i];
            const handedness = results.multiHandedness?.[i];
            const handId = handedness?.label || `hand_${i}`;
            const landmarks3D = this.convertTo3D(rawLandmarks);
            const dynamics = this.calculateDynamics(landmarks3D, handId, timestamp);
            const metrics = this.calculateHandMetrics(landmarks3D);
            const palm = this.calculatePalmData(landmarks3D);
            if (dynamics.maxVelocity > maxVelocity) maxVelocity = dynamics.maxVelocity;

            hands.push({
                id: handId, isLeft: handId === 'Left', confidence: handedness?.score || 0,
                landmarks: landmarks3D, velocities: dynamics.velocities, accelerations: dynamics.accelerations,
                avgVelocity: dynamics.avgVelocity, maxVelocity: dynamics.maxVelocity,
                fingertips: this.extractFingertips(landmarks3D, dynamics.velocities), palm, metrics
            });
        }
        return { hands, handCount: hands.length, totalLandmarks: hands.length * 21, maxVelocity, timestamp };
    }

    convertTo3D(landmarks) {
        return landmarks.map(lm => ({ x: (1 - lm.x) * 2 - 1, y: -(lm.y * 2 - 1), z: -lm.z * 2 }));
    }

    calculateDynamics(landmarks, handId, timestamp) {
        const zeroVel = landmarks.map(() => ({ x: 0, y: 0, z: 0 }));
        const history = this.history.get(handId);
        if (!history?.length) {
            this.history.set(handId, [{ landmarks, timestamp, velocities: zeroVel }]);
            return { velocities: zeroVel, accelerations: zeroVel, avgVelocity: 0, maxVelocity: 0 };
        }
        const prevFrame = history[history.length - 1];
        const dt = (timestamp - prevFrame.timestamp) / 1000;
        if (dt <= 0.001) return { velocities: prevFrame.velocities, accelerations: zeroVel, avgVelocity: 0, maxVelocity: 0 };

        const velocities = landmarks.map((lm, i) => {
            const prev = prevFrame.landmarks[i];
            return { x: (lm.x - prev.x) / dt, y: (lm.y - prev.y) / dt, z: (lm.z - prev.z) / dt };
        });
        const accelerations = velocities.map((vel, i) => {
            const prevVel = prevFrame.velocities[i];
            return { x: (vel.x - prevVel.x) / dt, y: (vel.y - prevVel.y) / dt, z: (vel.z - prevVel.z) / dt };
        });
        let totalVelocity = 0, maxVelocity = 0;
        velocities.forEach(vel => {
            const mag = Math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
            totalVelocity += mag;
            if (mag > maxVelocity) maxVelocity = mag;
        });
        history.push({ landmarks, timestamp, velocities });
        while (history.length > this.historyMaxFrames) history.shift();
        return { velocities, accelerations, avgVelocity: totalVelocity / velocities.length, maxVelocity };
    }

    calculateHandMetrics(landmarks) {
        let fingerSpread = 0;
        for (let i = 0; i < this.FINGERTIPS.length - 1; i++) {
            fingerSpread += this.distance3D(landmarks[this.FINGERTIPS[i]], landmarks[this.FINGERTIPS[i + 1]]);
        }
        fingerSpread /= (this.FINGERTIPS.length - 1);
        const palmCenter = this.calculatePalmCenter(landmarks);
        let openness = 0;
        this.FINGERTIPS.forEach(tipIdx => { openness += this.distance3D(landmarks[tipIdx], palmCenter); });
        openness /= this.FINGERTIPS.length;
        const fingerCurls = this.calculateFingerCurls(landmarks);
        const pinchDistance = this.distance3D(landmarks[this.LANDMARKS.THUMB_TIP], landmarks[this.LANDMARKS.INDEX_TIP]);
        const indexMCP = landmarks[this.LANDMARKS.INDEX_MCP], pinkyMCP = landmarks[this.LANDMARKS.PINKY_MCP];
        const handRoll = Math.atan2(pinkyMCP.y - indexMCP.y, pinkyMCP.x - indexMCP.x);
        return { fingerSpread, openness, fingerCurls, pinchDistance, handRoll, avgCurl: fingerCurls.reduce((a, b) => a + b, 0) / fingerCurls.length };
    }

    calculatePalmData(landmarks) {
        const center = this.calculatePalmCenter(landmarks);
        const wrist = landmarks[this.LANDMARKS.WRIST], indexMCP = landmarks[this.LANDMARKS.INDEX_MCP], pinkyMCP = landmarks[this.LANDMARKS.PINKY_MCP];
        const v1 = { x: indexMCP.x - wrist.x, y: indexMCP.y - wrist.y, z: indexMCP.z - wrist.z };
        const v2 = { x: pinkyMCP.x - wrist.x, y: pinkyMCP.y - wrist.y, z: pinkyMCP.z - wrist.z };
        const normal = this.normalize3D({ x: v1.y * v2.z - v1.z * v2.y, y: v1.z * v2.x - v1.x * v2.z, z: v1.x * v2.y - v1.y * v2.x });
        return { center, normal };
    }

    calculatePalmCenter(landmarks) {
        let x = 0, y = 0, z = 0;
        [0, 5, 9, 13, 17].forEach(idx => { x += landmarks[idx].x; y += landmarks[idx].y; z += landmarks[idx].z; });
        return { x: x / 5, y: y / 5, z: z / 5 };
    }

    calculateFingerCurls(landmarks) {
        return [[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16],[17,18,19,20]].map(f => {
            const actualDist = this.distance3D(landmarks[f[0]], landmarks[f[3]]);
            let maxDist = 0;
            for (let i = 0; i < f.length - 1; i++) maxDist += this.distance3D(landmarks[f[i]], landmarks[f[i + 1]]);
            return maxDist > 0 ? 1 - (actualDist / maxDist) : 0;
        });
    }

    extractFingertips(landmarks, velocities) {
        return this.FINGERTIPS.map((idx, i) => ({ position: landmarks[idx], velocity: velocities[idx], fingerIndex: i, name: ['thumb','index','middle','ring','pinky'][i] }));
    }

    distance3D(p1, p2) { const dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z; return Math.sqrt(dx*dx + dy*dy + dz*dz); }
    normalize3D(v) { const len = Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z); return len === 0 ? {x:0,y:0,z:1} : {x:v.x/len,y:v.y/len,z:v.z/len}; }
    reset() { this.history.clear(); }
}

