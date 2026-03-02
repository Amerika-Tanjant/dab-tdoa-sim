# DAB SoOP TDOA Simulation & Dataset Generator (MATLAB)

This repository generates **simulation-based, labeled datasets** for **SoOP (Signals of Opportunity) TDOA localization** in the **DAB-related VHF band (≈200–230 MHz)**.  
The code is **config-driven**, modular, and uses a **scenario plug-in** design: each scenario is defined in JSON and executed through a common pipeline that produces a consistent dataset schema (HDF5 + JSON metadata).

---

## 1) Global Parameters & Variables

All global parameters are loaded from:

- `configs/master_config.json`

### 1.1 Core constants, sampling, RF, and geometry

| Group | Variable | Meaning | Default Value | Source |
|---|---|---:|---:|---|
| Physics | `c_mps` | Speed of light (m/s) | 299792458.0 | `master_config.json` |
| Sampling | `fs_hz` | Sampling rate (Hz) (DAB-friendly choice) | **2.048e6** | `master_config.json` |
| Time | `frame_period_s` | Epoch time step (s) (DAB Mode-I scale) | **0.096** | `master_config.json` |
| RF | `fc_list_hz` | TX center frequencies (Hz) | [206e6, 214e6, 222e6, 230e6] | `master_config.json` |
| RF | `usrp_bw_hz` | USRP front-end bandwidth (Hz) | 56e6 | `master_config.json` |
| Geometry | `roi_center_llh` | ROI center (lat, lon, h) | (40.782686, 29.461213, 0) | `master_config.json` |
| Geometry | `roi_radius_m` | ROI radius for “nearby area” deployments | 5000 | `master_config.json` |
| TX layout | `tx_enu_m` | TX positions in local ENU plane (E,N) | (±900, ±800) m | `master_config.json` |
| TX layout | `tx_alt_m` | TX antenna altitude (m) | 10 | `master_config.json` |

> **DAB-specific note:** The dataset generator is “timing/geometry focused”. The key DAB-related numerical choices are:
> - `fs_hz = 2.048 MHz` (commonly aligned with DAB time bases / practical DSP workflows)
> - `frame_period_s = 0.096 s` (DAB Mode-I frame duration scale)

---

### 1.2 Measurement impairments (realism knobs)

Defaults come from `master_config.json` and can be overridden per scenario run via each scenario’s `sweeps`.

| Variable | Meaning | Default | Notes |
|---|---|---:|---|
| `sigma_toa_s` | Gaussian TOA timing noise (s) | 5e-9 | swept in scenarios |
| `quantize_to_samples` | Quantize TOA to sampling grid | true | `round(t*fs)/fs` |
| `p_nlos` | Probability of NLOS/multipath bias | 0.02 | swept in S03/S04 |
| `nlos_bias_mean_s` | Mean positive bias (s) | 5e-9 | swept in S03/S04 |
| `nlos_bias_std_s` | Bias std (s) | 3e-9 | swept in S03/S04 |
| `rx_offset_diff_s` | Constant inter-RX clock offset (UAV − Ref) (s) | 0.0 | swept in S02–S04 |

---

## 2) Repository Layout (Where Everything Lives)

### 2.1 Configuration

| Path | Contents |
|---|---|
| `configs/master_config.json` | Global parameters (fs, frame_period, fc list, default TX layout, default impairments, output settings) |
| `configs/scenarios/*.json` | Scenario definitions + sweep parameters |
| `configs/sweeps/*.json` | Optional helper sweep lists (distances, offsets, altitudes, speeds) |

### 2.2 MATLAB code (modular pipeline)

| Path | Role |
|---|---|
| `src/run_all.m` | Runs all scenario configs under `configs/scenarios/` |
| `src/run_one.m` | Runs one scenario config (cartesian sweeps) + writes dataset |
| `src/simulate_run.m` | Core simulation: TOA/TDOA/DD/Doppler/WLS |
| `src/scenario_factory.m` | Maps `scenario_type` → scenario generator |

**MATLAB packages (`+` folders):**

| Path | Module |
|---|---|
| `src/+scenarios/` | Scenario generators (plug-ins) |
| `src/+geometry/` | ENU ring points, LLH<->ENU, distances |
| `src/+motion/` | Trajectories and velocity estimation |
| `src/+measurements/` | TOA/TDOA/double-difference/Doppler |
| `src/+solvers/` | WLS position solver (hyperbolic model) |
| `src/+dataset/` | Dataset writer (HDF5) |
| `src/+utils/` | JSON helpers, sweeps, hashing, RNG seeding |

### 2.3 Output dataset

Each run produces:

| Path | Description |
|---|---|
| `outputs/dataset/<scenario_id>/index.csv` | One-line index per run (timestamp, run_id, seed, quick summary) |
| `outputs/dataset/<scenario_id>/run_<run_id>/meta.json` | Full run metadata (master + scenario + sweep params + summary) |
| `outputs/dataset/<scenario_id>/run_<run_id>/data.h5` | Dataset payload (HDF5 schema below) |

---

## 3) Dataset Format (HDF5 Schema)

Each `data.h5` uses a consistent structure:

### Inputs
| Dataset | Shape | Meaning |
|---|---:|---|
| `/inputs/tx_pos_enu` | (Ntx × 3) | TX positions in ENU (m) |
| `/inputs/ref_pos_enu` | (1 × 3) | Reference receiver ENU (m), if present |
| `/inputs/uav_traj_enu` | (K × 3) | UAV receiver trajectory ENU (m) |

### Measurements
| Dataset | Shape | Meaning |
|---|---:|---|
| `/measurements/toa_uav_s` | (K × Ntx) | UAV TOA measurements (s) |
| `/measurements/toa_ref_s` | (K × Ntx) | Ref TOA measurements (s), if present |
| `/measurements/dt_s` | (K × Ntx) | Receiver-centric TDOA Δt (s), if present |
| `/measurements/ddt_s` | (K × (Ntx−1)) | Double-difference δt (s), if present |
| `/measurements/dd_dist_m` | (K × (Ntx−1)) | Double-difference distance δd = c·δt (m), if present |

### Labels
| Dataset | Shape | Meaning |
|---|---:|---|
| `/labels/pos_true_enu` | (K × 3) | Ground-truth UAV position ENU (m) |
| `/labels/pos_est_enu` | (K × 3) | Estimated position ENU (m), if solved |

### Aux
| Dataset | Shape | Meaning |
|---|---:|---|
| `/aux/vr_mps` | (K × Ntx) | Radial velocity per TX (m/s) |
| `/aux/fd_hz` | (K × Ntx) | Doppler shift per TX (Hz) |
| `/aux/weights` | (K × (Ntx−1)) | WLS weights, if used |

---

## 4) Core Equations Used

### 4.1 TOA (Time of Arrival)
For TX at \(s\) and RX at \(r\):

\[
d=\|r-s\|,\qquad t_{\text{true}}=\frac{d}{c}
\]

Measurement model (noise + optional NLOS bias + optional quantization):

\[
\hat t = t_{\text{true}} + n + b_{\text{NLOS}}
\]

Quantization to sampling grid (`quantize_to_samples=true`):

\[
\hat t \leftarrow \frac{\text{round}(\hat t f_s)}{f_s}
\]

### 4.2 Receiver-centric TDOA (two receivers)
For transmitter \(k\):

\[
\Delta \hat t_k = \hat t_{uav,k} - \hat t_{ref,k}
\]

A constant inter-RX clock offset can be injected as:

\[
\hat t_{uav,k} \leftarrow \hat t_{uav,k} + (b_{uav}-b_{ref})
\]

### 4.3 Double-difference (TX1 as reference)
For \(k=2,3,4\):

\[
\delta \hat t_k = \Delta \hat t_k - \Delta \hat t_1
\]
\[
\delta \hat d_k = c\cdot \delta \hat t_k
\]

### 4.4 Doppler (radial component)
Let UAV velocity be \(v(t)\). For TX \(k\) at position \(s_k\):

\[
u_k(t)=\frac{s_k-r_u(t)}{\|s_k-r_u(t)\|},\qquad v_{r,k}(t)=v(t)^\top u_k(t)
\]
\[
f_{D,k}(t)=\frac{v_{r,k}(t)}{c}f_{c,k}
\]

### 4.5 WLS positioning (hyperbolic model)
Define:

\[
f_k(x)=\|x-s_k\|-\|x-s_1\|
\]

Solve:

\[
\hat x=\arg\min_x \sum_{k=2}^{N_{tx}} w_k\,(f_k(x)-z_k)^2
\]

(Implementation uses a Gauss–Newton style solver with a small regularization when needed.)

---

## 5) Scenarios (What They Do + What They Sweep)

Scenario configs are in `configs/scenarios/`. Outputs are written to `outputs/dataset/<scenario_id>/`.

---

### Scenario S01 — `S01_1TX_1RX_static`
**Goal:** Baseline TOA generation and sensitivity to timing noise / sampling quantization.

| Item | Value |
|---|---|
| TX count | 1 (TX1) |
| RX count | 1 (static) |
| Motion | none (K=1) |
| Clock offset | not applicable |

**Equations used:** TOA only (Section 4.1)

**Swept variables (from `S01_1TX_1RX_static.json`):**

| Variable | Values |
|---|---|
| `seeds` | [1, 2, 3, 4, 5] |
| `sigma_toa_s` | [1e-9, 5e-9, 20e-9] |
| `quantize_to_samples` | [true] |

**Saved outputs (per run):**
- `meta.json`, `data.h5`, plus `index.csv` entry

**Key HDF5 fields:**
- `/measurements/toa_uav_s`
- `/labels/pos_true_enu`

---

### Scenario S02 — `S02_1TX_2RX_offset_static`
**Goal:** Receiver-centric TDOA and the effect of a constant inter-RX clock offset.

| Item | Value |
|---|---|
| TX count | 1 (TX1) |
| RX count | 2 (Ref + UAV) |
| Motion | none (K=1) |
| Clock offset | injected as `rx_offset_diff_s` |

**Equations used:** TOA + Δt (Sections 4.1–4.2)

**Swept variables (from `S02_1TX_2RX_offset_static.json`):**

| Variable | Values |
|---|---|
| `seeds` | [1, 2, 3] |
| `rx_offset_diff_s` | [0, 100e-9, 500e-9, 1e-6, 5e-6] |
| `sigma_toa_s` | [1e-9, 5e-9, 20e-9] |
| `quantize_to_samples` | [true] |

**Key HDF5 fields:**
- `/measurements/toa_ref_s`
- `/measurements/toa_uav_s`
- `/measurements/dt_s`

---

### Scenario S03 — `S03_4TX_2RX_static_dd_wls`
**Goal:** Double-difference to suppress constant inter-RX offset + WLS positioning + robustness under NLOS bias/noise.

| Item | Value |
|---|---|
| TX count | 4 (TX1–TX4) |
| RX count | 2 (Ref + UAV) |
| Motion | none (K=1) |
| Clock offset | swept but mitigated via double-difference |

**Equations used:** TOA + Δt + δt + WLS (Sections 4.1–4.5)

**Swept variables (from `S03_4TX_2RX_static_dd_wls.json`):**

| Variable | Values |
|---|---|
| `seeds` | [1, 2] |
| `rx_offset_diff_s` | [0, 100e-9, 1e-6, 5e-6] |
| `sigma_toa_s` | [5e-9, 20e-9] |
| `p_nlos` | [0.0, 0.02] |
| `nlos_bias_mean_s` | [0.0, 5e-9] |
| `nlos_bias_std_s` | [0.0, 3e-9] |
| `quantize_to_samples` | [true] |

**Key HDF5 fields:**
- `/measurements/dt_s` (K×4)
- `/measurements/ddt_s` (K×3)
- `/measurements/dd_dist_m` (K×3)
- `/labels/pos_est_enu` (if solved)

---

### Scenario S04 — `S04_4TX_2RX_dynamic_linear_altitude`
**Goal:** Main time-series dataset: moving UAV (linear trajectory), altitude sweep, clock offset sweep, NLOS/noise sweeps, Doppler and per-epoch WLS position estimation.

| Item | Value |
|---|---|
| TX count | 4 (TX1–TX4) |
| RX count | 2 (Ref + UAV) |
| Motion | linear trajectory (K>1) |
| Time step | `dt_s = frame_period_s` |
| Altitude | swept (constant per run, K samples) |
| Doppler | computed per epoch |
| Clock offset | swept (`rx_offset_diff_s`) |

**Equations used:** TOA + Δt + δt + Doppler + WLS (Sections 4.1–4.5)

**Swept variables (from `S04_4TX_2RX_dynamic_linear_altitude.json`):**

| Variable | Values |
|---|---|
| `seeds` | [1, 2] |
| `rx_offset_diff_s` | [0, 100e-9, 1e-6, 5e-6] |
| `sigma_toa_s` | [5e-9, 20e-9] |
| `p_nlos` | [0.0, 0.02] |
| `nlos_bias_mean_s` | [0.0, 5e-9] |
| `nlos_bias_std_s` | [0.0, 3e-9] |
| `quantize_to_samples` | [true] |
| `speeds_mps` (traj config) | [5, 10, 20] |
| `altitudes_m` (traj config) | [30, 50, 100, 200, 300] |

**Key HDF5 fields:**
- `/inputs/uav_traj_enu` (K×3)
- `/aux/vr_mps` and `/aux/fd_hz` (K×4)
- `/measurements/ddt_s` and `/measurements/dd_dist_m` (K×3)
- `/labels/pos_est_enu` (K×3)

---

## 6) How to Run

From MATLAB:

```matlab
cd <repo>/src
run_all
```

Datasets will appear under:

- `outputs/dataset/<scenario_id>/run_<run_id>/`

---

## 7) Notes / Next Extensions

This repo currently focuses on **geometry-driven TOA/TDOA** plus configurable impairments. DAB-specific realism can be extended by adding:

- Null symbol + PRS correlation profiles as additional dataset features
- Time-varying clock drift models
- More UAV motion profiles (circle, waypoint, climb/descent \(h(t)\))
- Explicit multi-band channelization constraints (closer to USRP capture limitations)
