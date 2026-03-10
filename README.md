# FMCW Radar Target Tracking & Ghost Mitigation
### An End-to-End Simulation: From LFM Chirps to Kalman State Estimation

## Overview
This project implements a complete End-to-End Radar Signal Processing Pipeline in MATLAB. It bridges the gap between raw RF physics and high-level autonomous "vision." By simulating the physics of 77GHz FMCW chirps, the system employs a Kalman Filter to track both Position and Velocity in real-time. 

The simulation generates 77GHz radar signals, processes them to identify targets amidst noise and "ghost" reflections, and utilizes a 2nd-order Kalman Filter to provide smooth, sub-resolution trajectory estimation.

The system is specifically designed to handle common radar challenges:
* **Quantization Noise:** Overcoming the 1.5m range resolution limit using recursive estimation.
* **Ghost Mitigation:** Identifying and ignoring static reflections (ghosts) using predictive **Gating Logic**.
* **State Learning:** Monitoring the filter as it "converges" to the true velocity of a high-speed target.

---

## The Pipeline Architecture

### 1. Signal Generation (The Physics)
The system models a 77GHz LFM (Linear Frequency Modulation) chirp. By mixing the transmitted signal with the reflected return (heterodyning), we derive the **Beat Frequency ($f_b$)**.



### 2. Range Extraction (The DSP)
Raw signals are digitized and processed through a Fast Fourier Transform (FFT). Due to the $1.5\text{m}$ range resolution limit ($\Delta R = c/2B$), raw measurements appear as discrete "staircase" steps.

### 3. Intelligent Tracking (The Kalman Filter)
The core "brain" of the project uses a 2nd-order Discrete Kalman Filter. It maintains a state vector $x = [p, v]^T$ and uses a constant velocity transition matrix.

Smart Gating Logic: The tracker predicts the target's next location. If multiple detections appear (e.g., a real target and a static ghost), the filter associates the data point closest to its prediction, successfully ignoring environmental noise.

## Software Requirements
I ran this simulation in **MATLAB R2025b** with the following toolboxes installed:

* **Radar Toolbox:** Required for FMCW waveform generation and radar transceiver objects.
* **Signal Processing Toolbox:** Required for the `findpeaks` and `fft` functions.

> Note: If you do not have the Radar Toolbox, the core signal processing and Kalman filtering logic can still be executed using base MATLAB and the Signal Processing Toolbox.
## Live System Results
Below is the real-time output of the system tracking a target moving at **1500 m/s**:



https://github.com/user-attachments/assets/11d30f3a-1b6f-4472-bae8-81dacf3a57d8



### Analysis of the Dashboard:
* Signal Domain: Shows raw FFT peaks. Note the static "Ghost" peak at 50m which is successfully ignored.
* Spatial Domain: The Yellow Dashed Line (Kalman) provides a smooth path, cutting through the noisy Red Xs(Raw Data).
* State Estimation: The bottom plot shows the filter "learning" and locking onto the target's true velocity despite starting from a default guess.
---
