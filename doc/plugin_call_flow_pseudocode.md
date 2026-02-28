# MuJoCo Plugin Call Flow Pseudocode

This document describes the pseudocode for how plugins are called during each MuJoCo simulation step.

## Overview

MuJoCo plugins can be called at different stages of the simulation pipeline depending on their capability flags:
- **mjPLUGIN_PASSIVE**: Called during passive force computation (e.g., Wire, Cable plugins)
- **mjPLUGIN_ACTUATOR**: Called during actuator force computation
- **mjPLUGIN_SENSOR**: Called during sensor computation (at POS, VEL, or ACC stages)

## Main Simulation Step: mj_step()

```
FUNCTION mj_step(m, d):
    // Common to all integrators
    mj_checkPos(m, d)
    mj_checkVel(m, d)
    mj_forward(m, d)          // ← Plugins called here
    mj_checkAcc(m, d)
    
    // Compare forward and inverse solutions if enabled
    IF mjENABLED(mjENBL_FWDINV):
        mj_compareFwdInv(m, d)
    
    // Use selected integrator
    SWITCH integrator:
        CASE mjINT_EULER:
            mj_Euler(m, d)
        CASE mjINT_RK4:
            mj_RungeKutta(m, d, 4)
        CASE mjINT_IMPLICIT:
        CASE mjINT_IMPLICITFAST:
            mj_implicit(m, d)
```

## Forward Dynamics: mj_forward()

```
FUNCTION mj_forward(m, d):
    mj_forwardSkip(m, d, mjSTAGE_NONE, 0)
```

## Forward Dynamics (Skip): mj_forwardSkip()

```
FUNCTION mj_forwardSkip(m, d, skipstage, skipsensor):
    // Position-dependent computations
    IF skipstage < mjSTAGE_POS:
        mj_fwdPosition(m, d)
        
        IF NOT skipsensor:
            mj_sensorPos(m, d)        // ← Sensor plugins called here (POS stage)
        
        IF NOT energyPosSensor(m):
            IF mjENABLED(mjENBL_ENERGY):
                mj_energyPos(m, d)
            ELSE:
                d->energy[0] = d->energy[1] = 0
    
    // Velocity-dependent computations
    IF skipstage < mjSTAGE_VEL:
        mj_fwdVelocity(m, d)           // ← Passive plugins called here
        IF NOT skipsensor:
            mj_sensorVel(m, d)         // ← Sensor plugins called here (VEL stage)
        
        IF mjENABLED(mjENBL_ENERGY) AND NOT energyVelSensor(m):
            mj_energyVel(m, d)
    
    // Acceleration-dependent computations
    IF mjcb_control AND NOT mjDISABLED(mjDSBL_ACTUATION):
        mjcb_control(m, d)
    
    mj_fwdActuation(m, d)              // ← Actuator plugins called here
    mj_fwdAcceleration(m, d)
    mj_fwdConstraint(m, d)
    
    IF NOT skipsensor:
        mj_sensorAcc(m, d)             // ← Sensor plugins called here (ACC stage)
```

## Forward Velocity: mj_fwdVelocity()

```
FUNCTION mj_fwdVelocity(m, d):
    // Compute flexedge velocity
    IF mj_isSparse(m):
        mju_mulMatVecSparse(...)
    ELSE:
        mju_mulMatVec(...)
    
    // Compute tendon velocity
    IF mj_isSparse(m):
        mju_mulMatVecSparse(...)
    ELSE:
        mju_mulMatVec(...)
    
    // Compute actuator velocity
    IF NOT mjDISABLED(mjDSBL_ACTUATION):
        mju_mulMatVecSparse(...)
    
    // Compute COM-based velocities, passive forces, constraint references
    mj_comVel(m, d)
    mj_passive(m, d)                   // ← Passive plugins called here
    mj_referenceConstraint(m, d)
    
    // Compute qfrc_bias with abbreviated RNE
    mj_rne(m, d, 0, d->qfrc_bias)
    
    // Add bias force due to tendon armature
    mj_tendonBias(m, d, d->qfrc_bias)
```

## Passive Forces: mj_passive()

```
FUNCTION mj_passive(m, d):
    // Clear all passive force vectors
    mju_zero(d->qfrc_spring, nv)
    mju_zero(d->qfrc_damper, nv)
    mju_zero(d->qfrc_gravcomp, nv)
    mju_zero(d->qfrc_fluid, nv)
    mju_zero(d->qfrc_passive, nv)
    
    // Return early if passive forces disabled
    IF mjDISABLED(mjDSBL_PASSIVE):
        RETURN
    
    // Compute springs and dampers
    mj_springdamper(m, d)
    
    // Compute gravity compensation
    has_gravcomp = mj_gravcomp(m, d)
    
    // Compute fluid forces
    has_fluid = mj_fluid(m, d)
    
    // Add passive forces into qfrc_passive
    mju_add(d->qfrc_passive, d->qfrc_spring, d->qfrc_damper, nv)
    IF has_fluid:
        mju_addTo(d->qfrc_passive, d->qfrc_fluid, nv)
    IF has_gravcomp:
        // Add gravcomp forces (unless added via actuators)
        FOR each joint i:
            IF NOT m->jnt_actgravcomp[i]:
                // Add gravcomp force for this joint
    
    // User callback: add custom passive forces
    IF mjcb_passive:
        mjcb_passive(m, d)
    
    // PLUGIN: add custom passive forces
    IF m->nplugin > 0:
        nslot = mjp_pluginCount()
        FOR i = 0 TO m->nplugin - 1:
            slot = m->plugin[i]
            plugin = mjp_getPluginAtSlotUnsafe(slot, nslot)
            IF plugin == NULL:
                mjERROR("invalid plugin slot")
            IF plugin->capabilityflags & mjPLUGIN_PASSIVE:
                IF plugin->compute == NULL:
                    mjERROR("compute is null")
                // ← CALL PLUGIN Compute() FUNCTION HERE
                plugin->compute(m, d, i, mjPLUGIN_PASSIVE)
                // For Wire/Cable plugins, this calls Wire::Compute() or Cable::Compute()
```

## Forward Actuation: mj_fwdActuation()

```
FUNCTION mj_fwdActuation(m, d):
    // ... actuator force computation ...
    
    // PLUGIN: handle actuator plugins
    IF m->nplugin > 0:
        nslot = mjp_pluginCount()
        FOR i = 0 TO m->nplugin - 1:
            slot = m->plugin[i]
            plugin = mjp_getPluginAtSlotUnsafe(slot, nslot)
            IF plugin == NULL:
                mjERROR("invalid plugin slot")
            IF plugin->capabilityflags & mjPLUGIN_ACTUATOR:
                IF plugin->compute == NULL:
                    mjERROR("compute is null")
                // ← CALL PLUGIN Compute() FUNCTION HERE
                plugin->compute(m, d, i, mjPLUGIN_ACTUATOR)
```

## Sensor Position: mj_sensorPos()

```
FUNCTION mj_sensorPos(m, d):
    // ... compute position-dependent sensors ...
    
    // User sensors
    IF nusersensor > 0 AND mjcb_sensor:
        mjcb_sensor(m, d, mjSTAGE_POS)
    
    // PLUGIN: compute plugin sensor values
    IF m->nplugin > 0:
        nslot = mjp_pluginCount()
        FOR i = 0 TO m->nplugin - 1:
            slot = m->plugin[i]
            plugin = mjp_getPluginAtSlotUnsafe(slot, nslot)
            IF plugin == NULL:
                mjERROR("invalid plugin slot")
            IF (plugin->capabilityflags & mjPLUGIN_SENSOR) AND 
               (plugin->needstage == mjSTAGE_POS OR plugin->needstage == mjSTAGE_NONE):
                IF plugin->compute == NULL:
                    mjERROR("compute is null")
                // ← CALL PLUGIN Compute() FUNCTION HERE
                plugin->compute(m, d, i, mjPLUGIN_SENSOR)
```

## Sensor Velocity: mj_sensorVel()

```
FUNCTION mj_sensorVel(m, d):
    // ... compute velocity-dependent sensors ...
    
    // User sensors
    IF nusersensor > 0 AND mjcb_sensor:
        mjcb_sensor(m, d, mjSTAGE_VEL)
    
    // PLUGIN: trigger computation of plugins
    IF m->nplugin > 0:
        nslot = mjp_pluginCount()
        FOR i = 0 TO m->nplugin - 1:
            slot = m->plugin[i]
            plugin = mjp_getPluginAtSlotUnsafe(slot, nslot)
            IF plugin == NULL:
                mjERROR("invalid plugin slot")
            IF (plugin->capabilityflags & mjPLUGIN_SENSOR) AND 
               plugin->needstage == mjSTAGE_VEL:
                IF plugin->compute == NULL:
                    mjERROR("compute is null")
                // Compute subtree_linvel, subtree_angmom if needed
                IF subtreeVel == 0:
                    mj_subtreeVel(m, d)
                    subtreeVel = 1
                // ← CALL PLUGIN Compute() FUNCTION HERE
                plugin->compute(m, d, i, mjPLUGIN_SENSOR)
```

## Sensor Acceleration: mj_sensorAcc()

```
FUNCTION mj_sensorAcc(m, d):
    // ... compute acceleration-dependent sensors ...
    
    // User sensors
    IF nusersensor > 0 AND mjcb_sensor:
        mjcb_sensor(m, d, mjSTAGE_ACC)
    
    // PLUGIN: trigger computation of plugins
    IF m->nplugin > 0:
        nslot = mjp_pluginCount()
        FOR i = 0 TO m->nplugin - 1:
            slot = m->plugin[i]
            plugin = mjp_getPluginAtSlotUnsafe(slot, nslot)
            IF plugin == NULL:
                mjERROR("invalid plugin slot")
            IF (plugin->capabilityflags & mjPLUGIN_SENSOR) AND 
               plugin->needstage == mjSTAGE_ACC:
                IF plugin->compute == NULL:
                    mjERROR("compute is null")
                // Compute cacc, cfrc_int, cfrc_ext if needed
                IF rnePost == 0:
                    mj_rnePostConstraint(m, d)
                    rnePost = 1
                // ← CALL PLUGIN Compute() FUNCTION HERE
                plugin->compute(m, d, i, mjPLUGIN_SENSOR)
```

## Two-Phase Step Functions

MuJoCo also provides two-phase step functions for finer control:

### mj_step1() - Before user input

```
FUNCTION mj_step1(m, d):
    mj_checkPos(m, d)
    mj_checkVel(m, d)
    mj_fwdPosition(m, d)
    mj_sensorPos(m, d)                 // ← Sensor plugins (POS)
    // ... energy computation ...
    mj_fwdVelocity(m, d)               // ← Passive plugins called here
    mj_sensorVel(m, d)                 // ← Sensor plugins (VEL)
    // ... energy computation ...
    IF mjcb_control:
        mjcb_control(m, d)
```

### mj_step2() - After user input

```
FUNCTION mj_step2(m, d):
    mj_fwdActuation(m, d)              // ← Actuator plugins called here
    mj_fwdAcceleration(m, d)
    mj_fwdConstraint(m, d)
    mj_sensorAcc(m, d)                 // ← Sensor plugins (ACC)
    mj_checkAcc(m, d)
    
    // Compare forward and inverse solutions if enabled
    IF mjENABLED(mjENBL_FWDINV):
        mj_compareFwdInv(m, d)
    
    // Integrate
    IF integrator == mjINT_IMPLICIT OR integrator == mjINT_IMPLICITFAST:
        mj_implicit(m, d)
    ELSE:
        mj_Euler(m, d)
```

## Summary: Plugin Call Order During mj_step()

1. **Position Stage**: Sensor plugins with `needstage == mjSTAGE_POS` or `mjSTAGE_NONE`
2. **Velocity Stage**: 
   - Passive plugins (mjPLUGIN_PASSIVE) ← **Wire/Cable plugins called here**
   - Sensor plugins with `needstage == mjSTAGE_VEL`
3. **Actuation Stage**: Actuator plugins (mjPLUGIN_ACTUATOR)
4. **Acceleration Stage**: Sensor plugins with `needstage == mjSTAGE_ACC`

## Key Files

- `src/engine/engine_forward.c`: Main step functions (mj_step, mj_step1, mj_step2, mj_forward)
- `src/engine/engine_passive.c`: Passive force computation (mj_passive) - **Wire/Cable plugins called here**
- `src/engine/engine_sensor.c`: Sensor computation (mj_sensorPos, mj_sensorVel, mj_sensorAcc)
- `plugin/elasticity/wire.cc`: Wire plugin implementation (Wire::Compute)
- `plugin/elasticity/cable.cc`: Cable plugin implementation (Cable::Compute)

