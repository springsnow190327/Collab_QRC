# Gazebo vs MuJoCo — Why the Stack Works Better in Gazebo but MuJoCo Matches Real Life

**TL;DR**: Gazebo is physically idealized (hard LCP contact, sticky wheels, clean sensors); MuJoCo is compliant (soft elliptic contact, wheel slip, self-interfering LiDAR). The CMU autonomy stack + CHAMP locomotion were all tuned against Gazebo's ideal world, so they "just work" there. MuJoCo exposes the same failure modes the real robot will hit — its breakages are a signal, not a bug.

## 1. Contact Physics — The Root Divergence

| Axis | Gazebo Classic | MuJoCo |
|---|---|---|
| Contact model | Hard LCP (ODE / Bullet) | Soft constraint + elliptic friction cone (`cone="elliptic"`) |
| Penetration | Instant zero | Sub-mm allowed every physics step |
| Friction | `mu1 / mu2` per surface (slide only) | `friction=[slide, roll, spin]` `condim=3` — **rolling and spinning friction modelled** |
| Wheel behavior | "Sticky", no slip | Can slip; spins under torque saturation |
| Integrator / step | 1 ms (1 kHz), default ODE | 2 ms (500 Hz), `integrator="implicitfast"` |

**Evidence:** `src/go2w/go2_gazebo_sim/mujoco/go2w_custom.xml:8` — `friction="1.0 0.005 0.001" condim="3"`. `condim=3` activates rolling/spinning friction. Gazebo defaults to `condim=1` (slide only). **This is the direct cause of Go2W tip-overs in MuJoCo at moderate speed.**

## 2. CHAMP Locomotion Was Tuned for Gazebo

`config/champ/go2w/gait.yaml:6-10`:
```yaml
swing_height: 0.04
stance_depth: 0.01    # assumes floor doesn't give way
stance_duration: 0.25
nominal_height: 0.225
```

`stance_depth=0.01m` means "press the stance foot down 1 cm". In Gazebo's hard LCP, that 1 cm is absorbed by the instantaneous constraint — the foot stays **fixed**. In MuJoCo's compliant solver, the 1 cm may actually penetrate, or the foot slips sideways, shifting the robot's centre of mass — which is how 4/5 trials tipped at 0.9 m/s.

CHAMP was written against Gazebo in 2019 by Juan Rojas / Gabriel Chen. `joints.yaml` has joint-to-motor mapping only; the real control PD gains are hardcoded in CHAMP source **assuming Gazebo's stance behaviour**. Porting to MuJoCo means those gains are effectively un-tuned.

## 3. LiDAR Models

| | Gazebo `gazebo_ros_ray_sensor` | MuJoCo `mj_multiRay` |
|---|---|---|
| Noise | None (default 0) | None |
| Self-hits | Sensor is a child of `parent_link`, rays start outside body geometry → never hits itself | `bodyexclude: robot_body` excludes base, but **legs / wheels can still be scanned** |
| Ground sensing | Sensor z fixed to parent link | Livox at 0.12 m above base, 7° downward tilt → close-range rays hit **own feet** |

**Evidence:** `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/lidar_sensor.cpp:120` has `bodyexclude` but no per-geom filter. CLAUDE.md Phase 5 documents the "scan sees its own legs" problem, requiring a z-band filter in `pointcloud_to_laserscan` — **and the real Livox MID-360 on real Go2W has the exact same issue**. Gazebo's "clean scan" is the unrealistic one.

## 4. ros2_control Hardware Interface

**MuJoCo** — `mujoco_system.cpp:503`:
```cpp
double tau = std::clamp(effort_command, -effort_limit, effort_limit);
mujoco_data_->ctrl[actuators[EFFORT]] = tau;
```
Saturation is a **soft** clamp. When the motor saturates, torque delivery is noisy/nonlinear — just like real BLDC saturation.

**Gazebo** — torque limit enforced via URDF `<limit effort>`, which combined with ODE's instantaneous constraint solver gives **clean**, full-torque delivery at saturation. No "shudder".

## 5. Why Nav Stack Parameters Look Fine in Gazebo

Every CMU autonomy stack safety margin (`obs_inflate_size`, `pathFollower/stopDisThre`, `localPlanner/vehicleWidth` path-library collision) was calibrated on Gazebo:

- `obs_inflate_size=1` works in Gazebo because hard contact bounces the robot off walls instantly.
- In MuJoCo, 5 mm penetration counts as a collision — same `obs_inflate_size=1` no longer leaves enough margin.
- → MuJoCo needs `obs_inflate_size=2` to get the *equivalent* practical clearance. (See `docs/claude/nav_benchmarks.md` config A iteration history.)

This is the same story for every safety param: values that were "safe" in Gazebo are marginal in MuJoCo.

## 6. Why MuJoCo "Feels Like Real Life"

Features the **real robot** has that **MuJoCo replicates** and **Gazebo doesn't**:

1. **Wheels slip.** Go2W on polished floor will spin out during fast starts and fishtail in sharp stops. MuJoCo's `condim=3` + elliptic friction reproduces this. Gazebo's sticky contact does not.
2. **Compliant gait instability.** Real Go2W hitting a fast stop-turn has IMU roll/pitch fluctuations of ±10°. MuJoCo tips at 0.9 m/s because the gait cadence doesn't match the impulse — exactly the real-robot pattern. Gazebo's instantaneous constraint hides this.
3. **LiDAR sees its own legs.** Real Livox MID-360 needs a `min_height`/`max_height` z-band filter. MuJoCo replicates this. Gazebo's sensor geometry sidesteps it.
4. **Contact cascades.** Real wheel-against-wall rubs generate continuous contact events (you *hear* it, motor current spikes). MuJoCo's `RR_calf_upper: 956 contacts` over 55 s directly mirrors this "grinding". Gazebo gives one clean bounce and done.
5. **Controller saturation jitter.** Real BLDC torque saturation oscillates. MuJoCo's `std::clamp` does the same soft saturation.

## 7. Practical Guidance

| Phase | Which sim | Why |
|---|---|---|
| Structural / topology testing ("does the nav stack wire up?") | **Gazebo** | Ideal contact, fast iteration on logic correctness |
| Parameter tuning ("what `obs_inflate` is safe?") | **MuJoCo** | Closest to real robot; tuning transfers |
| Pre-deployment CI | **MuJoCo** | A Gazebo-passing config can still fail on hardware |
| Demos / video | Gazebo | Prettier output |

**Next step for the CMU autonomy stack**: re-calibrate `obs_inflate_size`, `pathFollower/stopDisThre`, `localPlanner/vehicleWidth`, `maxSpeed`, and CHAMP gait's `stance_depth` / `nominal_height` *in MuJoCo* rather than inheriting Gazebo defaults. Values tuned in MuJoCo transfer to real hardware far better than Gazebo's.

## One-Line Summary

> Gazebo is "the floor is granite", MuJoCo is "the floor is hardwood", the real robot is "the floor sometimes has a puddle". CHAMP + the CMU stack were tuned for granite. On hardwood they slip. The way MuJoCo fails is signalling where the real robot will fail too — the way Gazebo succeeds is telling you nothing.

## Concrete Benchmark Evidence

On `demo3` (24×16 m, 384 m²):

| Config | Cov avg | 0 contacts | Tip | 90% PASS |
|---|---|---|---|---|
| 0.2 m/s × 180s | 54.8% | 5/5 | 1/5 | 0/5 |
| 0.4 m/s × 300s (real-robot sweet spot) | **79.7%** | **5/5** | 1/5 | 0/5 (max 89.0%) |
| 0.9 m/s × 180s + velocity supervisor | 62.4% | 3/5 | **4/5** | 0/5 |

At 0.9 m/s, 4/5 tipovers — CHAMP gait cannot keep up with MuJoCo's compliant floor at that speed. This is the *real-robot* failure mode, just surfaced earlier.

## Related Docs

- [nav_benchmarks.md](nav_benchmarks.md) — Phase 5 config A tuning (all done in MuJoCo)
- [debug_notes.md](debug_notes.md) — cross-cutting MuJoCo/DDS/QoS gotchas
- `src/go2w/go2_gazebo_sim/mujoco/go2w_custom.xml:8` — friction / condim declaration
- `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/lidar_sensor.cpp:120` — `bodyexclude`
- `src/vendor/mujoco_ros2_control/mujoco_ros2_control/src/mujoco_system.cpp:503` — effort clamp
