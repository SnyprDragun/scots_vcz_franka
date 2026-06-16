# SCOTS vs FaSTrack — Sanity Check and Final Paper Tables

---

## Sanity Check

### SCOTS (`reach_example`) — all clean

| Check | Expected | Actual | Status |
|---|---|---|---|
| State grid dims | 38×71×54 = 145,692 | 145,692 | ✅ |
| Input grid | 11³ = 1,331 | 1,331 | ✅ |
| Tightening: δ = ū·τ/2 + η | 0.05×0.15 + 0.01 = 0.0175 m | — | ✅ |
| λ+δ | 0.05 + 0.0175 = 0.0675 m ≈ 0.07 m | — | ✅ |
| Tightened target x | [0.47+0.07, 0.69−0.07] = [0.54, 0.62] | [0.54, 0.62] | ✅ |
| Target cells | 9×9×11 = 891 | — | ✅ |
| Winning domain | 112,677 / 145,692 = 77.3% | 77.3% | ✅ |
| Abstraction time | — | 24.65 s | ✅ |
| Synthesis time | — | 16.69 s | ✅ |
| **Total offline time** | — | **41.33 s** | ✅ |

### FaSTrack — two important flags

| Check | Value | Status |
|---|---|---|
| up_max matches SCOTS i_ub | 0.05 m/s = 0.05 m/s | ✅ |
| Analytical TEB = up_max²/(2(ut_max−d_nom)) | 0.05²/(2×4.5) = **0.000278 m** | ✅ |
| Numerical TEB from code | 0.004383 m | ⚠️ grid artifact — do NOT use |
| Why numerical ≠ analytical | dx_r = 0.008 m >> TEB_true = 0.000278 m (ratio: 29×) | ⚠️ noted |
| compMethod fix applied | `'zero'` → `'maxVWithV0'` | ✅ critical fix |
| Perturbed: d_pert ≥ ut_max | 7.0 ≥ 5.0 → diverges | ✅ |
| Synthesis time (3 axes) | 5.79 s | ✅ |

**The numerical TEB (0.004383 m) must not appear in the paper.** The FaSTrack grid (101×101, dx_r = 0.008 m) is 29× coarser than the true TEB, so the numerical value is pure GLF dissipation artifact. The analytical TEB (0.000278 m) is derived in closed form from the game dynamics and verified by the trajectory derivation:

- Adversary applies u_p = −up_max, d = +d_nom starting from (r=0, v=0)
- v̇ = −(ut_max − d_nom) = −4.5 m/s²
- r(t) = −2.25t² + 0.05t, maximised at t* = 0.05/4.5 = 0.0111 s
- r(t*) = **0.000278 m** ✓

---

## Paper Table 1 — Comparison (final)

| Method | Offline synthesis time | Success — nominal | Success — 1 kg payload at wrist |
|---|---|---|---|
| Ours (SCOTS + impedance) | **41.33 s** | YES | YES |
| FaSTrack | **5.79 s** | YES | NO |

**Footnotes:**

- SCOTS offline time = abstraction (24.65 s) + controller synthesis (16.69 s). Run on Apple M-series (macOS).
- FaSTrack offline time = HJ VI solve for 3 independent 2D subsystems. Run on Windows 11 / MATLAB R2023b.
- FaSTrack failure (nominal → perturbed): 1 kg payload at Franka wrist raises effective EE disturbance from d_nom = 0.5 m/s² to d_pert ≈ 7.0 m/s², which exceeds the tracker's control authority (ut_max = 5.0 m/s²). The HJ value function diverges — no valid TEB exists — and the precomputed controller no longer guarantees bounded tracking error. Our impedance controller is passivity-based and adapts without model knowledge.
- Synthesis times measured on different hardware; they are order-of-magnitude indicative rather than a precise benchmark. Both are well under one hour, consistent with practical offline deployment.

---

## Paper Table 2 — Implementation specs and parameters

| Parameter | SCOTS + impedance | FaSTrack |
|---|---|---|
| **Planning model** | 3D single integrator ξ̇ = u | 3D single integrator ξ̇ = u_p |
| **Tracking model** | None (model-free) | Cartesian double integrator ẋ=v, v̇=u_t+d |
| **Low-level controller** | Passivity-based impedance | Optimal tracker: τ = J^T · (m_eff · u_t) |
| **Planning state space** | x∈[0.28,0.65], y∈[-0.35,0.35], z∈[0.02,0.55] m | Same (3D Cartesian EE) |
| **Velocity bound** | ū = 0.05 m/s | up_max = 0.05 m/s |
| **Offline tool** | SCOTS (C++, symbolic control) | helperOC + ToolboxLS (MATLAB, HJ reachability) |
| **Synthesis space** | Full 3D state space | 3 × independent 2D relative-state spaces |
| **Grid size** | 38×71×54 = 145,692 cells | 3 × 101×101 = 30,603 cells |
| **Grid resolution** | η = 0.01 m (all axes) | dx_r = 0.008 m, dx_v = 0.006 m/s |
| **Sampling / timestep** | τ = 0.3 s | τ_out = 0.02 s (CFL-limited sub-steps internally) |
| **Disturbance bound (nominal)** | Not required | d_nom = 0.5 m/s² |
| **Disturbance bound (1 kg payload)** | Not required | d_pert = 7.0 m/s² > ut_max → no valid TEB |
| **Original goal G** | x∈[0.47,0.69], y∈[-0.11,0.11], z∈[0.26,0.50] m | Same |
| **Specification tightening** | λ+δ = 0.0675 m per side | TEB = 0.000278 m per side |
| **Tightened target (planner)** | x∈[0.54,0.62], y∈[-0.04,0.04], z∈[0.33,0.43] m | G shrunk by 0.000278 m per side |
| **Controller output** | Static lookup table (112,677 winning states) | ∇V lookup table (3 × 101×101 grids) |
| **Offline synthesis time** | **41.33 s** | **5.79 s** |
