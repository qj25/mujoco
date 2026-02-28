# MuJoCo Passive Plugin Pseudocode

**Conventions:** b = number of rod/plugin bodies. Pseudocode uses `FOR each body b`, semicolons, and consistent indentation (2 spaces). All tables share the same step/statement style.

---

## Table 1: Passive Plugin Call Flow per Step

```
mj_step(m, d);
  → mj_forward(m, d);
    → mj_fwdVelocity(m, d);
      → mj_passive(m, d);
        FOR i = 0 TO nplugin-1:
          IF plugin[i].capabilityflags & mjPLUGIN_PASSIVE:
            plugin[i].compute(m, d, i, mjPLUGIN_PASSIVE);
```

---

## Table 2: Plugin Compute Pseudocode

### Wire (mujoco.elasticity.wire)

| Step | Pseudocode |
|------|------------|
| 1 | `updateVars(d); updateBishopFrame(d);` |
| 2 | `IF fullDyn: FOR bwi: updateTheta(θ_loc,bwi);` |
| 3 | `ELIF pqsActive: detectInteractions; updateTheta/splitTheta per segment;` |
| 4 | `ELSE: updateTheta(θ_loc,nv); splitTheta(0,nv);` |
| 5 | `updateMatFrame();` |
| 6 | `FOR each body b:` |
| 7 | `  compute curvature ω, material frame;` |
| 8 | `  compute bend/twist torques (dE/dθ, dE/dγ);` |
| 9 | `  mj_applyFT(m,d, 0, lfrc, xpos, body, qfrc_passive);` |

Simplified for pqs:
| Step | Pseudocode |
|------|------------|
| 1 | `updateVars(d); updateBishopFrame(d);` |
| 3 | `detectInteractions;` |
| 4 | `updateTheta(θ_loc,nv); splitTheta(0,nv);` |
| 5 | `updateMatFrame();` |
| 6 | `FOR each body b:` |
| 7 | `  compute curvature ω, material frame;` |
| 8 | `  compute bend/twist torques (dE/dθ, dE/dγ);` |
| 9 | `  mj_applyFT(m,d, 0, lfrc, xpos, body, qfrc_passive);` |

### WireQST (mujoco.elasticity.wire_qst)

| Step | Pseudocode |
|------|------------|
| 1 | `updateVars(d); UpdateBishopFrame(d); θ_n = get_thetan(d); updateTheta(θ_n);` |
| 2 | `populate distmat (distance matrix);` |
| 3 | `FOR each body b:` |
| 3 | `  compute curvature k, kb; nabkb, nabpsi;` |
| 4 | `  force[j] += -α∇kb·kb/lbar + βΔθ·∇ψ/Lbar;` |
| 5 | `  torqvec = distmat × force; torq = rot(torqvec, quat_inv);` |
| 6 | `add torques directly to d->qfrc_passive;` |

### Cable (mujoco.elasticity.cable)

| Step | Pseudocode |
|------|------------|
| 1 | `FOR each body b:` |
| 2 | `  IF stiffness[b]==0: continue` |
| 3 | `  QuatDiff(quat, body_quat, joint_quat);` |
| 4 | `  LocalStress(stress, stiffness, quat, omega0) → elastic torque;` |
| 5 | `  lfrc += stress (from prev/next neighbors);` |
| 6 | `  mj_applyFT(m,d, 0, lfrc, xpos, body, qfrc_passive);` |

---

## Summary: Key Helper Functions

### QuatDiff

Computes the quaternion difference between two frames in joint coordinates.

- **Inputs**: `body_quat` (body orientation), `joint_quat` (joint orientation)
- **Output**: `quat` = relative orientation between the two frames
- **pullback=false**: `quat = body_quat * joint_quat` — orientation in local/body frame
- **pullback=true**: Same product, then negated — orientation pulled back into the neighboring body’s frame
- **Purpose**: Gives the relative rotation between adjacent bodies for computing elastic deformation (deviation from reference curvature).

### LocalStress

Computes the local elastic stress (restoring torque) from material properties and orientation.

- **Inputs**: `stiffness` (twist G, bend Iy·E, Iz·E, length), `quat` (orientation from QuatDiff), `omega0` (reference curvature)
- **Output**: `stress[3]` — elastic torque in local coordinates
- **Steps**: (1) Convert `quat` to curvature `omega` via `quat2Vel`; (2) Compute `stress = -stiffness[i]·(omega[i] - omega0[i]) / length` for each axis; (3) Optionally pull-back into the other body frame
- **Purpose**: Produces the elastic restoring torque that resists deviation from the reference curvature.

### mj_applyFT vs Direct Addition to qfrc_passive

| Approach | When to use | What it does |
|----------|-------------|--------------|
| **Direct addition** | Forces/torques already in joint space | Add `qfrc[dof_addr] += torque` for each joint DOF. Requires knowing the mapping from bodies to DOFs. |
| **mj_applyFT** | Forces/torques in Cartesian space (F, τ at a 3D point) | Computes `qfrc += J'·[F; τ]` where J is the Jacobian from joint velocities to linear/angular velocity at the point. Maps a Cartesian wrench at a body point to generalized forces. |

**Why mj_applyFT**: Elasticity models (e.g. Cable) produce forces and torques in Cartesian or body-local space. To affect the simulation, these must be converted to joint-space generalized forces. `mj_applyFT` performs this conversion via the transpose Jacobian. Direct addition would only apply if the model already produced `qfrc` values.

---

## Xfrc (Python stiffness → `data.xfrc_applied` → MuJoCo `qfrc`)

The xfrc approach implements rod stiffness in Python and applies Cartesian forces through `data.xfrc_applied`; MuJoCo then converts these to generalized forces during the forward pass.

### Python side (per step, before `mj_step`)

| Step | Pseudocode |
|------|------------|
| 1 | `updateVars(d); UpdateBishopFrame(d); θ_n = get_thetan(d); updateTheta(θ_n);` |
| 6 | `FOR each body b:` |
| 3 | `  compute curvature k, kb; nabkb, nabpsi;` |
| 6 | `  add forces directly to d->xfrc_applied;` |

### MuJoCo side (during mj_forward → mj_fwdAcceleration)

| Step | Pseudocode |
|------|------------|
| 1 | `qfrc_smooth = qfrc_passive - qfrc_bias + qfrc_applied + qfrc_actuator;` |
| 2 | `mj_xfrcAccumulate(m, d, qfrc_smooth);` |
| 3 | `FOR each body b:` |
| 4 | `  IF d->xfrc_applied != 0:` |
| 5 | `    F = xfrc_applied[:3]; τ = xfrc_applied[3:];` |
| 6 | `    mj_applyFT(m, d, F, τ, xpos, body, qfrc_total);` |

---

## Table 3: mj_applyFT Calls per Step

| Method | mj_applyFT calls | Notes |
|--------|------------------|-------|
| **Wire** | **B** | One per body. |
| **Cable** | **B** | One per body. |
| **WireQST** | **0** | Torques added directly to `qfrc_passive` by DOF index; no Jacobian. |
| **Xfrc** | **B** | One per body inside `mj_xfrcAccumulate`. |

---

## Computational Load (applyFT and Jacobian)

- **Cost per mj_applyFT:** Each call computes (or uses) the Jacobian of body point velocity w.r.t. joint velocities, then performs `qfrc += J'·[F; τ]`. Cost is on the order of one Jacobian evaluation plus O(nv) for the transpose-vector product.
- **Wire, Cable, Xfrc:** All do **B** such calls per step → applyFT-related cost is **O(B · (Jacobian + nv))** per step. For a rod with B bodies, Wire and Cable each do B calls in the plugin; Xfrc does B' calls in `mj_xfrcAccumulate` during forward.
- **WireQST:** **0** mj_applyFT calls → no Jacobian cost for applying torques; only direct indexing into `qfrc_passive`. So among the four, WireQST has the lowest applyFT-related load; Wire, Cable, and Xfrc are comparable (all linear in number of rod bodies), with the actual constant depending on how the Jacobian is computed/cached in MuJoCo.