# GPS Sensor Calculation Pipeline

This document explains the noise injection and frame projection pipeline utilized by the `CarlaGPS` sensor component to simulate an imperfect GNSS receiver.

## Overview

The simulator uses a ground-truth GNSS sensor from CARLA and processes its signals to insert real-world noise models (Gauss-Markov errors and white noise) before serving it to the autopilot framework. Furthermore, it strictly handles geodetic coordinate transformations between CARLA's underlying map and the user's defined global coordinate origin.

## Transformation & Noise Flow

```mermaid
flowchart TD
    subgraph "CARLA Ground Truth"
        GT_GNSS[Raw CARLA GNSS\n(Lat, Lon, Alt)\nRelative to Map GeoRef]
        GT_Vel[CARLA World Velocity\n(X=East, Y=South, Z=Up)]
    end

    subgraph "Geodetic Reprojection"
        GT_ENU[Convert GNSS to Local ENU\n(East, North, Up)\nOrigin: Map GeoRef]
        GT_Proj[Reproject Local ENU\nOrigin: Global Coordinates Origin\nOutputs: GT Lat, GT Lon, GT Alt]
        
        GT_GNSS --> GT_ENU --> GT_Proj
    end

    subgraph "Noise Model"
        RW[Random Walk Generation\n(Wiener Process Step)]
        GM_Bias[Gauss-Markov Bias Integration\nbias = bias + rw - bias * dt / tau]
        WhiteNoise[Position White Noise\n(Discrete StdDev)]
        
        RW --> GM_Bias
    end

    subgraph "Noisy Position Calculation"
        Noisy_ENU[Add White Noise & GM Bias\nto Ground Truth ENU]
        Noisy_Proj[Reproject Noisy ENU\nOrigin: Global Coordinates Origin\nOutputs: Noisy Lat, Lon, Alt]
        
        GT_ENU --> Noisy_ENU
        GM_Bias --> Noisy_ENU
        WhiteNoise --> Noisy_ENU
        Noisy_ENU --> Noisy_Proj
    end

    subgraph "Noisy Velocity Calculation"
        Vel_Map[Map Velocity Axes\nNorth = -Y\nEast = X\nDown = -Z]
        Vel_Noise[Velocity White Noise\n(Discrete StdDev)]
        Vel_Out[Outputs: Noisy Vel North, East, Down]
        
        GT_Vel --> Vel_Map
        Vel_Map --> Vel_Out
        Vel_Noise --> Vel_Out
    end
    
    GT_Proj -.-> Final_Output[(Publish GpsState)]
    Noisy_Proj --> Final_Output
    Vel_Out --> Final_Output
```

## Step-by-Step Details

1. **GNSS Retrieval**: We extract the Latitude, Longitude, and Altitude from CARLA's raw GNSS sensor. This position is relative to CARLA's OpenDRIVE map GeoReference.
2. **ENU Conversion**: We convert the raw map geodetic coordinates to Cartesian `ENU` (East, North, Up) coordinates, taking the map's GeoReference as the origin.
3. **Ground Truth Projection**: We reproject these exact `ENU` Cartesian coordinates outwards using the **user's specified global origin** (using standard `ecef_to_geodetic` Bowring's method). This provides the ground-truth lat/lon/alt for reference.
4. **Noise Injection**:
   - We generate a discrete random walk noise step `rw`.
   - We integrate the `rw` step with the previous bias using a Gauss-Markov equation to simulate satellite clock drifts, ionospheric errors, etc.
   - We calculate standard discrete white noise position offsets.
5. **Noisy Geodetic Projection**: The Gauss-Markov bias and white noise are added to the ground-truth ENU position. This modified ENU position is then reprojected again from the user's global origin to produce the final noisy Latitude, Longitude, and Altitude.
6. **Velocity Noise**: CARLA provides ground truth velocity in its native World Frame (where X points East, Y points South, and Z points Up). The components are remapped and negated appropriately (`North = -Y`, `East = X`, `Down = -Z`), and additive white noise is injected independently into each axis.

---

## Mathematical Derivations

This section details the exact equations used at each stage of the pipeline.

### 1. WGS-84 Ellipsoid Constants

All geodetic conversions use the WGS-84 reference ellipsoid, defined by the following constants:

| Symbol | Name | Value |
|---|---|---|
| $a$ | Semi-major axis | $6{,}378{,}137.0\ \text{m}$ |
| $b$ | Semi-minor axis | $6{,}356{,}752.31424518\ \text{m}$ |
| $e^2$ | First eccentricity squared | $0.00669437999014$ |
| $e'^2$ | Second eccentricity squared | $0.00673949674228$ |

Note that $e^2 = 1 - b^2/a^2$ and $e'^2 = a^2/b^2 - 1$.

---

### 2. Geodetic → ECEF (`geodetic_to_ecef`)

Given a geodetic position $(\varphi, \lambda, h)$ (latitude, longitude, altitude), the ECEF Cartesian coordinates $(X, Y, Z)$ are:

$$N(\varphi) = \frac{a}{\sqrt{1 - e^2 \sin^2\varphi}}$$

$$X = (N + h)\cos\varphi\cos\lambda$$
$$Y = (N + h)\cos\varphi\sin\lambda$$
$$Z = \left[N(1 - e^2) + h\right]\sin\varphi$$

where $N(\varphi)$ is the **radius of curvature in the prime vertical** (the distance from the surface to the Z-axis along the ellipsoid normal).

---

### 3. ECEF → Geodetic (Bowring's Method, `ecef_to_geodetic`)

The inverse problem — recovering $(\varphi, \lambda, h)$ from $(X, Y, Z)$ — uses Bowring's iterative approximation. A single iteration is sufficient for sub-millimetre accuracy.

**Auxiliary parametric latitude $\theta$:**

$$p = \sqrt{X^2 + Y^2}, \qquad \theta = \arctan\!\left(\frac{Z \cdot a}{p \cdot b}\right)$$

**Geodetic latitude $\varphi$ (Bowring's formula):**

$$\varphi = \arctan\!\left(\frac{Z + e'^2 \cdot b \cdot \sin^3\theta}{p - e^2 \cdot a \cdot \cos^3\theta}\right)$$

**Longitude:**

$$\lambda = \arctan2(Y,\ X)$$

**Altitude:**

$$h = p\cos\varphi + Z\sin\varphi - a\sqrt{1 - e^2\sin^2\varphi}$$

The polar singularity case ($p < 10^{-10}\ \text{m}$) is handled separately: $\varphi = \pm\frac{\pi}{2}$ and $h = |Z| - b$.

---

### 4. ECEF ↔ ENU Rotation (`ecef_to_enu` / `enu_to_ecef`)

ENU (East-North-Up) is a local tangent-plane frame anchored at an origin $(\varphi_0, \lambda_0, h_0)$. The ECEF offset $\Delta\mathbf{r} = \mathbf{r} - \mathbf{r}_0$ is rotated into ENU by the orthogonal matrix $\mathbf{R}$:

$$\begin{pmatrix} E \\ N \\ U \end{pmatrix} = \mathbf{R} \begin{pmatrix} \Delta X \\ \Delta Y \\ \Delta Z \end{pmatrix}$$

$$\mathbf{R} = \begin{pmatrix}
-\sin\lambda_0 & \cos\lambda_0 & 0 \\
-\sin\varphi_0\cos\lambda_0 & -\sin\varphi_0\sin\lambda_0 & \cos\varphi_0 \\
\cos\varphi_0\cos\lambda_0 & \cos\varphi_0\sin\lambda_0 & \sin\varphi_0
\end{pmatrix}$$

The inverse (ENU → ECEF offset) uses $\mathbf{R}^{-1} = \mathbf{R}^\top$ (since $\mathbf{R}$ is orthogonal):

$$\begin{pmatrix} \Delta X \\ \Delta Y \\ \Delta Z \end{pmatrix} = \mathbf{R}^\top \begin{pmatrix} E \\ N \\ U \end{pmatrix}$$

$$\Delta X = -\sin\lambda_0\, E - \sin\varphi_0\cos\lambda_0\, N + \cos\varphi_0\cos\lambda_0\, U$$
$$\Delta Y = \phantom{-}\cos\lambda_0\, E - \sin\varphi_0\sin\lambda_0\, N + \cos\varphi_0\sin\lambda_0\, U$$
$$\Delta Z = \phantom{-\sin\lambda_0\, E{} - {}} \cos\varphi_0\, N + \sin\varphi_0\, U$$

---

### 5. Geodetic Re-projection (`reprojection`)

The full ENU → geodetic pipeline is a composition of the above steps:

$$(\varphi, \lambda, h) = \text{ECEF2Geo}\!\left(\, \mathbf{r}_0 + \mathbf{R}^\top \mathbf{e}_{\text{ENU}} \,\right)$$

where $\mathbf{r}_0$ is the ECEF position of the chosen origin and $\mathbf{e}_{\text{ENU}} = [E,\ N,\ U]^\top$ is the displacement in metres. This is used twice:
- Once with the **map GeoRef origin** to decode CARLA's raw GNSS output into true ENU offsets.
- Once with the **user's global origin** to produce the final output lat/lon/alt (both GT and noisy).

---

### 6. Noise Model

#### 6.1 Random Walk (Wiener Process Step)

At each time step $\Delta t$, a zero-mean Gaussian increment is drawn for each ENU axis:

$$\text{rw}_i = \sigma_{\text{rw},i} \cdot \sqrt{\Delta t} \cdot \mathcal{N}(0,1), \qquad i \in \{E, N, U\}$$

where $\sigma_{\text{rw},E} = \sigma_{\text{rw},N} = \texttt{gps\_xy\_random\_walk}$ and $\sigma_{\text{rw},U} = \texttt{gps\_z\_random\_walk}$ (units: $\text{m}/\sqrt{\text{s}}$). The $\sqrt{\Delta t}$ factor converts the continuous-time power spectral density into a discrete step.

#### 6.2 Gauss-Markov Bias Integration

The bias vector $\mathbf{b}_k$ models slowly drifting errors (satellite clock, ionosphere, troposphere). It evolves as a first-order Gauss-Markov process:

$$b_{k+1} = b_k + \text{rw} - b_k \cdot \frac{\Delta t}{\tau}$$

or equivalently:

$$b_{k+1} = b_k \!\left(1 - \frac{\Delta t}{\tau}\right) + \text{rw}$$

where $\tau = \texttt{gps\_correlation\_time}$ (seconds) is the **correlation time** (time constant of the exponential decay). As $\Delta t / \tau \to 0$, the bias approaches a pure random walk; as $\Delta t / \tau \to 1$, it decays quickly toward zero. This is a discrete Euler approximation of the continuous SDE:

$$\dot{b}(t) = -\frac{1}{\tau}b(t) + w(t)$$

where $w(t)$ is a white-noise driving process.

#### 6.3 Position White Noise

Independent discrete white noise is added to each ENU axis:

$$\eta_i = \sigma_{\text{pos},i} \cdot \mathcal{N}(0,1)$$

where $\sigma_{\text{pos},E} = \sigma_{\text{pos},N} = \texttt{gps\_xy\_noise\_density}$ and $\sigma_{\text{pos},U} = \texttt{gps\_z\_noise\_density}$ (units: metres, already a discrete standard deviation — **no** $\sqrt{\Delta t}$ scaling).

#### 6.4 Noisy ENU Position

The final noisy ENU vector passed to re-projection is:

$$\mathbf{e}_{\text{noisy}} = \mathbf{e}_{\text{GT}} + \boldsymbol{\eta} + \mathbf{b}$$

$$\begin{pmatrix} E_{\text{noisy}} \\ N_{\text{noisy}} \\ U_{\text{noisy}} \end{pmatrix} = \begin{pmatrix} E_{\text{GT}} \\ N_{\text{GT}} \\ U_{\text{GT}} \end{pmatrix} + \begin{pmatrix} \eta_E \\ \eta_N \\ \eta_U \end{pmatrix} + \begin{pmatrix} b_E \\ b_N \\ b_U \end{pmatrix}$$

---

### 7. Velocity Frame Remapping and Noise

CARLA's World Frame convention is:

| CARLA Axis | Physical Direction |
|---|---|
| $V_x$ | East |
| $V_y$ | South (negative North) |
| $V_z$ | Up (negative Down) |

Remapping to the standard NED output frame:

$$V_{\text{North}} = -V_y + \sigma_{vxy} \cdot \mathcal{N}(0,1)$$
$$V_{\text{East}} = V_x + \sigma_{vxy} \cdot \mathcal{N}(0,1)$$
$$V_{\text{Down}} = -V_z + \sigma_{vz} \cdot \mathcal{N}(0,1)$$

where $\sigma_{vxy} = \texttt{gps\_vxy\_noise\_density}$ and $\sigma_{vz} = \texttt{gps\_vz\_noise\_density}$ are discrete velocity standard deviations (m/s). Ground speed is computed from the noiseless CARLA velocity components:

$$s = \sqrt{V_x^2 + V_y^2}$$
