# Cable Plugin: Physics and Mathematics

This document describes the physics and mathematics of the cable elasticity model implemented in `plugin/elasticity/cable.cc`. The model discretizes an **inextensible, shearless, elastic rod** with twist and bending stiffness and applies restoring torques at each body.

---

## 1. Continuum model and relation to known rod theories

The cable plugin implements a **discrete Kirchhoff (special Cosserat) rod** with the following assumptions:

- **Inextensible and unshearable**: arc length is preserved; cross sections remain normal to the centerline (no shear strain).
- **Linear elastic constitutive law**: internal moment is proportional to the difference between current and reference curvature/twist.
- **No stretch energy**: only bending and twist moments are considered; no axial force from stretch.

In the continuum, the **Kirchhoff rod** (or special Cosserat rod) has an internal moment \(\mathbf{M}(s)\) related to the **curvature-twist vector** \(\boldsymbol{\kappa}(s)\) (in the material frame) by
\[
\mathbf{M} = \mathbf{K}\,(\boldsymbol{\kappa} - \boldsymbol{\kappa}_0),
\]
where \(\mathbf{K}\) is the stiffness matrix (diagonal for circular cross sections) and \(\boldsymbol{\kappa}_0\) is the reference (stress-free) curvature. The balance of moments is \(\mathbf{M}' + \boldsymbol{\tau} = \mathbf{0}\) (in the absence of external torques), where \(\boldsymbol{\tau}\) is the distributed external torque. The cable plugin discretizes this in space and applies the resulting torques as passive forces in MuJoCo.

**References:** Antman (2005), *Nonlinear Problems of Elasticity*; Bergou et al. (2008), *Discrete elastic rods* (SIGGRAPH); or any "Kirchhoff rod" / "special Cosserat rod" formulation.

---

## 2. Discrete setup

- **Bodies:** \(n\) bodies indexed \(b = 0, \ldots, n-1\), with global body indices \(i = i_0 + b\).
- **Segments:** Segment \(b\) connects body \(b-1\) to body \(b\) (for \(b \geq 1\)). So segment \(b\) is "in front of" body \(b\).
- **Orientation:** Each body has a world orientation (quaternion); the "joint" stores the reference orientation (e.g. from `qpos`). The **relative rotation** of body \(b\) with respect to its reference is used as the discrete curvature for segment \(b\).

**Notation:**

- \(\mathbf{q}_i\): world orientation (quaternion) of body \(i\).
- \(\mathbf{q}_{j,i}\): reference (joint) quaternion for body \(i\) (from `qpos`).
- \(\mathbf{q}_{\mathrm{diff}}\) or \(\mathrm{QuatDiff}\): relative quaternion from reference to current orientation (see below).
- \(\boldsymbol{\omega}\): 3D curvature-twist vector (axis–angle representation of that relative rotation, with "time" \(= 1\)).
- \(\boldsymbol{\omega}_0\): reference curvature for the segment (stored in `omega0`).
- \(L_b\): length of segment \(b\), i.e. distance between body \(b-1\) and body \(b\) (stored as `stiffness[4*b+3]`).

---

## 3. Quaternion difference and curvature

**QuatDiff** (relative orientation in "joint" frame):

\[
\mathbf{q}_{\mathrm{diff}} = \mathbf{q}_{\mathrm{body}} * \mathbf{q}_{\mathrm{joint}}.
\]

So the body orientation is expressed relative to the joint (reference) frame. If `pullback == true`, the code uses the conjugate so that the result is pulled back into the other body's frame when needed.

**Curvature from quaternion** (`mju_quat2Vel`): The relative quaternion is converted to an axis–angle vector \(\boldsymbol{\omega}\) with "time step" \(\Delta t = 1\). So \(\boldsymbol{\omega}\) is the rotation axis scaled by the angle (in radians):

\[
\mathbf{q} = (\cos\frac{\theta}{2}, \, \mathbf{n}\sin\frac{\theta}{2}), \quad
\boldsymbol{\omega} = \frac{2\,\mathrm{atan2}(\|\mathbf{n}\sin\frac{\theta}{2}\|,\, \cos\frac{\theta}{2})}{\Delta t}\,\frac{(\mathbf{n}\sin\frac{\theta}{2})}{\|\mathbf{n}\sin\frac{\theta}{2}\|} \quad (\Delta t = 1).
\]

So \(\boldsymbol{\omega}\) has magnitude equal to the rotation angle (rad) and direction along the rotation axis. This is used as the **discrete curvature–twist vector** for the segment (angle between two consecutive body frames, with no division by arc length in the quat2Vel call; the division by length is in the stress formula).

---

## 4. Stiffness parameters (per segment \(b\))

From the first body's geometry and plugin parameters:

- **Twist stiffness:** \(G\) (shear modulus, plugin attribute `twist` [Pa]), and polar second moment \(J\):
  \[
  K_0 = J\,G.
  \]
- **Bending stiffness:** Young's modulus \(E\) (plugin attribute `bend` [Pa]), and second moments of area \(I_y\), \(I_z\):
  \[
  K_1 = I_y\,E, \qquad K_2 = I_z\,E.
  \]
- **Length:** \(L_b = \|\mathbf{x}_i - \mathbf{x}_{i-1}\|\) at initialization (stored in `stiffness[4*b+3]`).

For a **cylinder/capsule** (radius \(r\)):

\[
J = \frac{\pi r^4}{2}, \qquad I_y = I_z = \frac{\pi r^4}{4}.
\]

For a **box** (half-widths \(h, w\) in the two cross-section dimensions), \(J\) uses the rectangular torsion formula; \(I_y\), \(I_z\) are the usual area moments of the rectangle.

So the diagonal stiffness vector used in the code is \(\mathbf{K} = (K_0, K_1, K_2)\) and the segment length is \(L_b\).

---

## 5. Local stress (internal moment)

**LocalStress** computes the elastic torque (stress) in the **local** (body) frame for one segment. Let \(\boldsymbol{\omega}\) be the curvature from the relative quaternion (via `mju_quat2Vel(quat, 1)`), and \(\boldsymbol{\omega}_0\) the reference curvature for that segment.

**Formula:**

\[
\boldsymbol{\sigma} = -\frac{1}{L_b}\,
\begin{bmatrix}
K_0\,(\omega_1 - \omega_{0,1}) \\
K_1\,(\omega_2 - \omega_{0,2}) \\
K_2\,(\omega_3 - \omega_{0,3})
\end{bmatrix}
= -\frac{1}{L_b}\,\mathbf{K}\odot(\boldsymbol{\omega} - \boldsymbol{\omega}_0),
\]

where \(\odot\) denotes element-wise product. So the internal moment (torque) in the local frame is

\[
\mathbf{M} = -\frac{\mathbf{K}}{L_b}\,(\boldsymbol{\omega} - \boldsymbol{\omega}_0).
\]

If `pullback == true`, \(\boldsymbol{\sigma}\) is rotated by the inverse of the relative quaternion so that the moment is expressed in the "other" body's frame for the balance of torques.

---

## 6. Balance of torques at each body

Body \(b\) (index \(i\)) receives:

1. **From segment \(b\) (between \(b-1\) and \(b\)):** the moment \(\mathbf{M}_b\) from the segment that ends at body \(b\). It is pulled back into body \(b\)'s frame and added: \(\mathbf{f}_{\mathrm{local}} \mathrel{+}= \boldsymbol{\sigma}_b\).
2. **From segment \(b+1\) (between \(b\) and \(b+1\)):** the moment from the segment that starts at body \(b\) is computed in body \(b+1\)'s local frame; the reaction on body \(b\) is \(-\boldsymbol{\sigma}_{b+1}\) (no pullback in the same way; the sign is \(-1\)). So \(\mathbf{f}_{\mathrm{local}} \mathrel{+}= -\boldsymbol{\sigma}_{b+1}\).

So the **net elastic torque** on body \(b\) in its local frame is

\[
\boldsymbol{\tau}_{\mathrm{local}}^{(b)} = \boldsymbol{\sigma}_b - \boldsymbol{\sigma}_{b+1}.
\]

(With the convention that \(\boldsymbol{\sigma}_0 = \mathbf{0}\) for the first body and \(\boldsymbol{\sigma}_{n} = \mathbf{0}\) for the last.) This is the discrete form of \(\mathbf{M}' = -\boldsymbol{\tau}_{\mathrm{ext}}\).

Then this torque is rotated to the world frame and applied at the body center of mass with zero force:

\[
\boldsymbol{\tau}_{\mathrm{world}}^{(b)} = \mathbf{R}(\mathbf{q}_i)\,\boldsymbol{\tau}_{\mathrm{local}}^{(b)},
\]

and `mj_applyFT(m, d, 0, xfrc, xpos, i, d->qfrc_passive)` is called with force \(\mathbf{0}\) and torque \(\boldsymbol{\tau}_{\mathrm{world}}^{(b)}\) at `xpos`, which adds \(\mathbf{J}^\top [\mathbf{0}; \boldsymbol{\tau}_{\mathrm{world}}]\) to `qfrc_passive`.

---

## 7. Reference curvature \(\boldsymbol{\omega}_0\)

- If **flat** is true: \(\boldsymbol{\omega}_0 = \mathbf{0}\) (straight, stress-free reference).
- Otherwise: at initialization, after running kinematics with `qpos0`,
  \[
  \boldsymbol{\omega}_0^{(b)} = \mathrm{quat2Vel}\bigl(\mathrm{neg}(\mathbf{q}_{j,i}) * \mathbf{q}_{\mathrm{body},i},\, 1\bigr),
  \]
  i.e. the axis–angle of the rotation from the joint reference quaternion to the body quaternion at rest. So the reference curvature is the **initial** relative orientation of each body w.r.t. its joint.

---

## 8. Pseudocode summary

```
CONSTRUCTOR:
  For b = 0 .. n-1:
    prev[b] = (b == 0 ? 0 : -1),  next[b] = (b == n-1 ? 0 : +1)
    If prev[b] and flat != "true":
      omega0[3*b:3*b+3] = subQuat(body_quat[i], qpos[joint_quat_adr])
    Else:
      omega0[3*b:3*b+3] = 0
    Compute J, Iy, Iz from geom (cylinder/box)
    stiffness[4*b+0] = J*G,  stiffness[4*b+1] = Iy*E,  stiffness[4*b+2] = Iz*E
    stiffness[4*b+3] = distance(xpos[i], xpos[i+prev[b]])  // L_b

QuatDiff(quat_out, body_quat, joint_quat, pullback):
  If !pullback:  quat_out = body_quat * joint_quat
  Else:          quat_out = neg(body_quat * joint_quat)

LocalStress(stress, stiffness, quat, omega0, pullback):
  omega = quat2Vel(quat, 1)
  tmp[0] = -stiffness[0]*(omega[0]-omega0[0])/stiffness[3]
  tmp[1] = -stiffness[1]*(omega[1]-omega0[1])/stiffness[3]
  tmp[2] = -stiffness[2]*(omega[2]-omega0[2])/stiffness[3]
  If pullback:  stress = R(neg(quat)) * tmp
  Else:         stress = tmp

Compute (per step):
  For b = 0 .. n-1:
    If all stiffness[b*4+0..2] == 0: continue
    lfrc = 0
    If prev[b]:
      QuatDiff(quat, body_quat[i], qpos[joint_i], pullback=true)
      LocalStress(stress_b, stiffness[4*b], quat, omega0[3*b], pullback=true)
      lfrc += stress_b
    If next[b]:
      QuatDiff(quat, body_quat[in], qpos[joint_in], pullback=false)
      LocalStress(stress_bn, stiffness[4*bn], quat, omega0[3*bn], pullback=false)
      lfrc -= stress_bn
    xfrc = R(xquat[i]) * lfrc
    mj_applyFT(m, d, 0, xfrc, xpos[i], i, d->qfrc_passive)
```

---

## 9. Energy (for reference)

In the continuum, the elastic energy density (per arc length) for a Kirchhoff rod is

\[
\frac{1}{2}\,(\boldsymbol{\kappa} - \boldsymbol{\kappa}_0)^\top \mathbf{K}\,(\boldsymbol{\kappa} - \boldsymbol{\kappa}_0).
\]

The discrete analogue for segment \(b\) would be

\[
E_b = \frac{L_b}{2}\,(\boldsymbol{\omega} - \boldsymbol{\omega}_0)^\top \mathbf{K}\,(\boldsymbol{\omega} - \boldsymbol{\omega}_0)\,/\,L_b
= \frac{1}{2}\,(\boldsymbol{\omega} - \boldsymbol{\omega}_0)^\top \mathbf{K}\,(\boldsymbol{\omega} - \boldsymbol{\omega}_0).
\]

The torques applied by the plugin are consistent with the derivative of this energy with respect to the body orientations (with the sign convention and frame conventions used in the code).

---

## 10. Relation to existing models

| Model | Relation |
|-------|----------|
| **Kirchhoff rod** | Same constitutive law \(\mathbf{M} = \mathbf{K}(\boldsymbol{\kappa} - \boldsymbol{\kappa}_0)\) and moment balance; cable is a spatial discretization with one segment per body. |
| **Discrete elastic rods (Bergou et al.)** | Same idea (discrete curvature, bending/twist stiffness). DER uses Bishop frame and curvature binormals; cable uses a simpler discrete curvature from quaternion difference. |
| **Cosserat rod** | Special case (inextensible, unshearable) with linear material law; cable does not solve the full Cosserat equations (e.g. no axial force). |

**Identifying the model:** Search for "Kirchhoff rod," "special Cosserat rod," "discrete elastic rod," or "bend and twist stiffness" plus "inextensible" to find the same class of models.
