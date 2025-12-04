/**
 * Hand Tracker 3D Module
 * High-precision 3D hand landmark detection using MediaPipe
 *
 * Features:
 * - 21 3D landmarks per hand with sub-pixel accuracy
 * - Temporal smoothing with configurable parameters
 * - Adaptive smoothing based on movement speed
 * - Full 3D coordinate extraction (x, y, z)
 */

export class HandTracker3D {
    constructor(options = {}) {
        // Configuration with validation
        this.options = {
            maxHands: Math.min(Math.max(options.maxHands || 2, 1), 4),
            modelComplexity: options.modelComplexity || 1, // 0, 1
            minDetectionConfidence: Math.min(Math.max(options.minDetectionConfidence || 0.7, 0.1), 1.0),
            minTrackingConfidence: Math.min(Math.max(options.minTrackingConfidence || 0.7, 0.1), 1.0)
        };

        // MediaPipe instances
        this.hands = null;
        this.camera = null;

        // DOM elements
        this.videoElement = document.getElementById('camera-feed');

        // Callbacks
        this.resultsCallback = null;

        // State
        this.isRunning = false;
        this.isInitialized = false;

        // Smoothing system - adaptive based on velocity
        this.smoothing = {
            factor: 0.4,           // Base smoothing (0 = no smoothing, 1 = max smoothing)
            minFactor: 0.2,        // Minimum smoothing for fast movements
            maxFactor: 0.7,        // Maximum smoothing for slow movements
            velocityThreshold: 0.1 // Velocity threshold for adaptive smoothing
        };

        // Previous frame data for smoothing
        this.previousData = new Map(); // handId -> landmarks

        // Initialize MediaPipe
        this.initMediaPipe();
    }

    /**
     * Initialize MediaPipe Hands with error handling
     */
    initMediaPipe() {
        try {
            // Create Hands instance with CDN file locator
            this.hands = new Hands({
                locateFile: (file) => {
                    return `https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4.1675469240/${file}`;
                }
            });

            // Configure hand detection options
            this.hands.setOptions({
                maxNumHands: this.options.maxHands,
                modelComplexity: this.options.modelComplexity,
                minDetectionConfidence: this.options.minDetectionConfidence,
                minTrackingConfidence: this.options.minTrackingConfidence
            });

            // Set up results callback
            this.hands.onResults((results) => this.processResults(results));

            this.isInitialized = true;
            console.log('✓ MediaPipe Hands initialized');

        } catch (error) {
            console.error('Failed to initialize MediaPipe:', error);
            throw new Error('MediaPipe initialization failed: ' + error.message);
        }
    }

    /**
     * Process raw MediaPipe results
     * Applies smoothing and prepares data for downstream processing
     */
    processResults(results) {
        if (!this.resultsCallback) return;

        // Add timestamp to results
        results.timestamp = performance.now();

        // Apply temporal smoothing to landmarks
        if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
            results.multiHandLandmarks = results.multiHandLandmarks.map((landmarks, handIndex) => {
                const handId = this.getHandId(results.multiHandedness, handIndex);
                return this.smoothLandmarks(landmarks, handId);
            });
        } else {
            // Clear previous data when no hands detected
            this.previousData.clear();
        }

        // Invoke callback with processed results
        this.resultsCallback(results);
    }

    /**
     * Get unique hand identifier based on handedness
     */
    getHandId(multiHandedness, index) {
        if (multiHandedness && multiHandedness[index]) {
            return multiHandedness[index].label; // 'Left' or 'Right'
        }
        return `hand_${index}`;
    }

    /**
     * Apply adaptive temporal smoothing to landmarks
     * Smoothing factor adjusts based on movement velocity
     */
    smoothLandmarks(landmarks, handId) {
        const previous = this.previousData.get(handId);

        if (!previous) {
            // First frame - store and return as-is
            this.previousData.set(handId, this.cloneLandmarks(landmarks));
            return landmarks;
        }

        // Calculate average velocity across all landmarks
        let totalVelocity = 0;
        for (let i = 0; i < landmarks.length; i++) {
            const dx = landmarks[i].x - previous[i].x;
            const dy = landmarks[i].y - previous[i].y;
            const dz = landmarks[i].z - previous[i].z;
            totalVelocity += Math.sqrt(dx * dx + dy * dy + dz * dz);
        }
        const avgVelocity = totalVelocity / landmarks.length;

        // Calculate adaptive smoothing factor
        // Higher velocity = less smoothing (more responsive)
        // Lower velocity = more smoothing (more stable)
        let adaptiveFactor;
        if (avgVelocity > this.smoothing.velocityThreshold) {
            adaptiveFactor = this.smoothing.minFactor;
        } else {
            const t = avgVelocity / this.smoothing.velocityThreshold;
            adaptiveFactor = this.smoothing.maxFactor -
                (this.smoothing.maxFactor - this.smoothing.minFactor) * t;
        }

        // Apply exponential smoothing
        const smoothed = landmarks.map((landmark, i) => {
            const prev = previous[i];
            return {
                x: prev.x + (landmark.x - prev.x) * (1 - adaptiveFactor),
                y: prev.y + (landmark.y - prev.y) * (1 - adaptiveFactor),
                z: prev.z + (landmark.z - prev.z) * (1 - adaptiveFactor)
            };
        });

        // Store for next frame
        this.previousData.set(handId, this.cloneLandmarks(smoothed));

        return smoothed;
    }

    /**
     * Deep clone landmarks array
     */
    cloneLandmarks(landmarks) {
        return landmarks.map(lm => ({ x: lm.x, y: lm.y, z: lm.z }));
    }

    /**
     * Register results callback - can be set as property or called as method
     */
    set onResults(callback) {
        if (typeof callback === 'function') {
            this.resultsCallback = callback;
        }
    }

    get onResults() {
        return this.resultsCallback;
    }

    /**
     * Initialize the hand tracker (returns a promise for async initialization)
     */
    async initialize() {
        // MediaPipe is already initialized in constructor
        // This method exists for compatibility with async initialization patterns
        return Promise.resolve();
    }

    /**
     * Start camera and hand tracking
     * @param {HTMLVideoElement} videoElement - Optional video element (if already set up externally)
     */
    async start(videoElement = null) {
        if (this.isRunning) {
            console.warn('Hand tracker already running');
            return;
        }

        if (!this.isInitialized) {
            throw new Error('Hand tracker not initialized');
        }

        // Use provided video element or fall back to default
        if (videoElement) {
            this.videoElement = videoElement;
        }

        try {
            // If video already has a stream, use it; otherwise request camera
            if (!this.videoElement.srcObject) {
                const stream = await navigator.mediaDevices.getUserMedia({
                    video: {
                        width: { ideal: 1280, min: 640 },
                        height: { ideal: 720, min: 480 },
                        facingMode: 'user',
                        frameRate: { ideal: 30, min: 15 }
                    },
                    audio: false
                });
                this.videoElement.srcObject = stream;
            }

            // Wait for video to be ready if not already playing
            if (this.videoElement.readyState < 2) {
                await new Promise((resolve, reject) => {
                    this.videoElement.onloadedmetadata = () => {
                        this.videoElement.play()
                            .then(resolve)
                            .catch(reject);
                    };
                    this.videoElement.onerror = reject;
                });
            }

            // Initialize MediaPipe camera utility
            this.camera = new Camera(this.videoElement, {
                onFrame: async () => {
                    if (this.isRunning && this.hands) {
                        try {
                            await this.hands.send({ image: this.videoElement });
                        } catch (error) {
                            console.error('Error sending frame to MediaPipe:', error);
                        }
                    }
                },
                width: this.videoElement.videoWidth || 1280,
                height: this.videoElement.videoHeight || 720
            });

            await this.camera.start();
            this.isRunning = true;

            console.log('📷 Camera started:', {
                width: this.videoElement.videoWidth,
                height: this.videoElement.videoHeight
            });

        } catch (error) {
            console.error('Failed to start camera:', error);

            // Provide user-friendly error messages
            if (error.name === 'NotAllowedError') {
                throw new Error('Camera access denied. Please allow camera access and try again.');
            } else if (error.name === 'NotFoundError') {
                throw new Error('No camera found. Please connect a camera and try again.');
            } else if (error.name === 'NotReadableError') {
                throw new Error('Camera is in use by another application.');
            }

            throw error;
        }
    }

    /**
     * Stop camera and hand tracking
     */
    stop() {
        if (!this.isRunning) return;

        // Stop MediaPipe camera
        if (this.camera) {
            try {
                this.camera.stop();
            } catch (e) {
                console.warn('Error stopping camera:', e);
            }
            this.camera = null;
        }

        // Stop video stream
        if (this.videoElement && this.videoElement.srcObject) {
            const tracks = this.videoElement.srcObject.getTracks();
            tracks.forEach(track => track.stop());
            this.videoElement.srcObject = null;
        }

        // Clear state
        this.isRunning = false;
        this.previousData.clear();

        console.log('📷 Camera stopped');
    }

    /**
     * Draw debug visualization on canvas
     */
    drawDebug(canvas, results) {
        if (!canvas || !results) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const width = canvas.width;
        const height = canvas.height;

        // Clear canvas
        ctx.clearRect(0, 0, width, height);

        // Draw video frame (mirrored for natural interaction)
        if (this.videoElement && this.videoElement.readyState >= 2) {
            ctx.save();
            ctx.scale(-1, 1);
            ctx.drawImage(this.videoElement, -width, 0, width, height);
            ctx.restore();
        }

        // Draw hand landmarks if available
        if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
            for (let handIndex = 0; handIndex < results.multiHandLandmarks.length; handIndex++) {
                const landmarks = results.multiHandLandmarks[handIndex];
                const handedness = results.multiHandedness?.[handIndex]?.label || 'Unknown';
                const isLeft = handedness === 'Left';

                // Color based on handedness
                const color = isLeft ? '#00d4ff' : '#ff6b6b';
                const connectionColor = isLeft ? 'rgba(0, 212, 255, 0.5)' : 'rgba(255, 107, 107, 0.5)';

                // Draw connections using MediaPipe utility
                if (typeof drawConnectors === 'function' && typeof HAND_CONNECTIONS !== 'undefined') {
                    drawConnectors(ctx, landmarks, HAND_CONNECTIONS, {
                        color: connectionColor,
                        lineWidth: 2
                    });
                }

                // Draw landmarks using MediaPipe utility
                if (typeof drawLandmarks === 'function') {
                    drawLandmarks(ctx, landmarks, {
                        color: color,
                        lineWidth: 1,
                        radius: 4
                    });
                }

                // Draw hand label
                if (landmarks.length > 0) {
                    const wrist = landmarks[0];
                    ctx.save();
                    ctx.scale(-1, 1);
                    ctx.fillStyle = color;
                    ctx.font = 'bold 14px sans-serif';
                    ctx.fillText(handedness, -(wrist.x * width) - 30, wrist.y * height - 10);
                    ctx.restore();
                }
            }
        }

        // Draw tracking info
        ctx.fillStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.font = '12px monospace';
        ctx.fillText(`Hands: ${results.multiHandLandmarks?.length || 0}`, 10, 20);
        ctx.fillText(`Confidence: ${(results.multiHandedness?.[0]?.score * 100 || 0).toFixed(1)}%`, 10, 36);
    }
}
