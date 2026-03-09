# FMCW Radar Target Tracking & Ghost Mitigation
### An End-to-End Simulation: From LFM Chirps to Kalman State Estimation

## Overview
This project implements a full-stack FMCW (Frequency-Modulated Continuous Wave) Radar processing pipeline in MATLAB. It bridges the gap between raw RF physics and high-level target tracking. 

The simulation generates 77GHz radar signals, processes them to identify targets amidst noise and "ghost" reflections, and utilizes a 2nd-order Kalman Filter to provide smooth, sub-resolution trajectory estimation.

## Key Features
- **Radar Physics Simulation:** Generation of Linear Frequency Modulated (LFM) chirps and beat signal mixing.
- **Digital Signal Processing:** Real-time FFT-based range extraction and peak detection.
- **Ghost Mitigation:** Gating logic to distinguish moving targets from static environment reflections (simulated "ghosts").
- **State Estimation:** Discrete-time Kalman Filter (Constant Velocity model) to handle measurement quantization and noise.
- **Hardware-Aware Design:** Parameters optimized for 77GHz automotive/industrial radar standards.

---

## The Pipeline Architecture

### 1. Signal Generation (The Physics)
The system simulates a 77GHz carrier with a 100MHz bandwidth. The beat frequency $f_b$ is derived by mixing the transmitted and received signals:
$$R = \frac{f_b \cdot c \cdot T_{sweep}}{2 \cdot B}$$



### 2. Range Extraction (The DSP)
Raw signals are digitized and processed through a Fast Fourier Transform (FFT). Due to the $1.5\text{m}$ range resolution limit ($\Delta R = c/2B$), raw measurements appear as discrete "staircase" steps.

### 3. Target Tracking (The Estimation)
A Kalman Filter is used to maintain a state vector $x = [p, v]^T$. It utilizes a transition matrix $A$ to predict motion between chirps:
$$A = \begin{bmatrix} 1 & \Delta t \\ 0 & 1 \end{bmatrix}$$
The filter effectively "smooths" the quantized measurements and provides a continuous track of the target's true path.

---

## Results

Below is the visualization of a target moving at high velocity.

<img width="1055" height="671" alt="image" src="https://github.com/user-attachments/assets/a3e7483b-bbbe-4cac-8c3f-fb7ba1a4ba67" />


**Analysis:**
- **Green Line:** True target trajectory.
- **Red Xs:** Raw radar measurements showing **Range Bin Quantization**.
- **Blue Dashed Line:** Kalman Filter output, successfully filtering noise and providing a predictive estimate of the object's position.

---
