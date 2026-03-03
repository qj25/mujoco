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

---

## Detailed Pseudocode: WireQST Torque Computation (lines 322-333)

### WireQST Torque Calculation from Forces

| Step | Pseudocode |
|------|------------|
| 1 | `torqvec = zeros(nv+2, 3); torqvec_indiv = zeros(nv+2, 3);` |
| 2 | `FOR i = 0 TO nv+1:` |
| 3 | `  FOR j = 0 TO nv+1:` |
| 4 | `    torqvec_indiv[j] = cross(distmat[i][j], nodes[i].force);` |
| 5 | `  END FOR` |
| 6 | `  torqvec += torqvec_indiv;` |
| 7 | `END FOR` |
| 8 | `torqvec /= 2.0;` |
| 9 | `FOR i = 0 TO nv+1:` |
| 10 | `  quat_inv = inverseQuat(nodes[i].quat);` |
| 11 | `  nodes[i].torq = rotVecQuat(torqvec[i], quat_inv);` |
| 12 | `END FOR` |

**Notes:**
- `distmat[i][j]` is the vector from node `i` to node `j` (distance matrix)
- The cross product `distmat[i][j] × nodes[i].force` computes the torque contribution from force at node `i` acting at distance vector to node `j`
- The division by 2.0 accounts for double-counting in the pairwise computation
- The final rotation transforms the torque vector from world frame to the local body frame using the inverse quaternion

---

## Detailed Pseudocode: mj_applyFT Function

### mj_applyFT Implementation

| Step | Pseudocode |
|------|------------|
| 1 | `mj_applyFT(m, d, force[3], torque[3], point[3], body, qfrc_target):` |
| 2 | `  nv = m->nv;` |
| 3 | `  IF force != NULL: allocate jacp[3*nv];` |
| 4 | `  IF torque != NULL: allocate jacr[3*nv];` |
| 5 | `  allocate qforce[nv];` |
| 6 | `  IF body < 0 OR body >= m->nbody: ERROR;` |
| 7 | `  IF mj_isSparse(m):` |
| 8 | `    chain = allocate(nv);` |
| 9 | `    NV = mj_bodyChain(m, body, chain);` |
| 10 | `    mj_jacSparse(m, d, jacp, jacr, point, body, NV, chain);` |
| 11 | `    IF force != NULL:` |
| 12 | `      qforce = jacp' * force;  // transpose-vector product` |
| 13 | `      FOR i = 0 TO NV-1:` |
| 14 | `        qfrc_target[chain[i]] += qforce[i];` |
| 15 | `      END FOR` |
| 16 | `    END IF` |
| 17 | `    IF torque != NULL:` |
| 18 | `      qforce = jacr' * torque;  // transpose-vector product` |
| 19 | `      FOR i = 0 TO NV-1:` |
| 20 | `        qfrc_target[chain[i]] += qforce[i];` |
| 21 | `      END FOR` |
| 22 | `    END IF` |
| 23 | `  ELSE:  // dense case` |
| 24 | `    mj_jac(m, d, jacp, jacr, point, body);` |
| 25 | `    IF force != NULL:` |
| 26 | `      qforce = jacp' * force;  // transpose-vector product` |
| 27 | `      qfrc_target += qforce;  // add to all nv elements` |
| 28 | `    END IF` |
| 29 | `    IF torque != NULL:` |
| 30 | `      qforce = jacr' * torque;  // transpose-vector product` |
| 31 | `      qfrc_target += qforce;  // add to all nv elements` |
| 32 | `    END IF` |
| 33 | `  END IF` |
| 34 | `  freeStack(d);` |
| 35 | `END` |

**Key Components:**
- **jacp**: Position Jacobian (3×nv) — maps joint velocities to linear velocity at `point`
- **jacr**: Rotation Jacobian (3×nv) — maps joint velocities to angular velocity of `body`
- **Sparse case**: Only computes Jacobian for DOFs in the kinematic chain from root to `body` (NV ≤ nv)
- **Dense case**: Computes full Jacobian for all nv DOFs
- **Transpose-vector product**: `J'·f` converts Cartesian force/torque to generalized forces

---

## Explanation: mj_applyFT Internal Mechanics and Computational Differences

### What Happens When mj_applyFT is Called

When `mj_applyFT` is invoked, it performs the following operations:

1. **Jacobian Computation**: 
   - If `force != NULL`, computes the **position Jacobian** `jacp` (3×nv) that relates joint velocities `q̇` to the linear velocity `v` of the point: `v = jacp · q̇`
   - If `torque != NULL`, computes the **rotation Jacobian** `jacr` (3×nv) that relates joint velocities `q̇` to the angular velocity `ω` of the body: `ω = jacr · q̇`

2. **Force/Torque Mapping**:
   - Uses the **transpose Jacobian** to convert Cartesian forces/torques to generalized forces: `qfrc = J'·[F; τ]`
   - This is the principle of virtual work: the work done by Cartesian forces must equal the work done by generalized forces

3. **Accumulation**:
   - Adds the computed generalized forces to `qfrc_target` (typically `qfrc_passive` or `qfrc_applied`)

### Why Force vs Torque Have Different Computational Costs

The computational difference between applying forces and torques in `mj_applyFT` stems from the different Jacobians required:

| Aspect | Force (via jacp) | Torque (via jacr) |
|--------|------------------|-------------------|
| **Jacobian Type** | Position Jacobian | Rotation Jacobian |
| **Physical Meaning** | Maps joint velocities → linear velocity at point | Maps joint velocities → angular velocity of body |
| **Point Dependency** | **Yes** — depends on `point` location | **No** — independent of `point` (only depends on `body`) |
| **Computation Cost** | Higher — must account for lever arm effects | Lower — no lever arm computation needed |

**Key Differences:**

1. **Position Jacobian (jacp) for Forces**:
   - Must compute how the point's position changes with each joint DOF
   - Requires evaluating the kinematic chain from root to the body, then computing the lever arm from body center to the point
   - The point location affects the Jacobian: forces applied at different points produce different generalized forces
   - Cost: O(NV) for sparse, O(nv) for dense, where NV is the chain length

2. **Rotation Jacobian (jacr) for Torques**:
   - Only depends on the body's orientation, not the point location
   - Computes how the body's angular velocity relates to joint velocities
   - The point parameter is ignored for torque application (torques are body-centric)
   - Cost: O(NV) for sparse, O(nv) for dense, but typically faster than jacp due to simpler geometric operations

**Practical Implications:**

- **Force-only calls**: Require computing `jacp`, which involves more geometric operations (lever arm calculations)
- **Torque-only calls**: Require computing `jacr`, which is typically faster as it doesn't depend on point location
- **Both force and torque**: Computes both Jacobians, but the total cost is roughly the sum of individual costs
- **Sparse vs Dense**: Sparse mode only computes Jacobians for DOFs in the kinematic chain (NV ≤ nv), significantly reducing cost for deep kinematic trees

**In the Context of Elasticity Plugins:**

- **Wire/Cable**: Apply torques only → use `jacr` → faster than if they applied forces
- **Xfrc**: Can apply both forces and torques → may compute both `jacp` and `jacr` if both are non-zero
- **WireQST**: Bypasses `mj_applyFT` entirely → avoids all Jacobian computation → fastest approach


**Notes**
our FT method indeed saves computational time - instead of doing applyFT
	- add in pseudocode to thesis
	- suggest reason for difference 
		-- direct has expensive applyFT calls in which jac is computed, whereas adapted uses the force-lever simplification.
		-- direct, native, and jpqDER have same number of applyFT (jacSparse) calls but jacp is more than jacr (around 3x more)

- see what applyFT is doing for just torque, then see if simplification is possible? 