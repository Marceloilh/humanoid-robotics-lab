# humanoid-robotics-lab

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/24.04/)
[![C++](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/20)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://docs.python.org/3.12/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-CPU%20physics-000000)](https://mujoco.readthedocs.io/)
[![License](https://img.shields.io/badge/License-Apache%202.0-D22128)](LICENSE)

**Engineering logbook for a structured transition into humanoid robotics software** —
ROS 2, modern C++, physics simulation and real-time control, built from first
principles over 36 weeks.

Every phase ships running code. Every performance claim ships a measured number.

---

## Table of contents

- [1. What this repository is](#1-what-this-repository-is)
- [2. The system under study](#2-the-system-under-study)
- [3. Engineering constraints](#3-engineering-constraints)
- [4. Design decisions](#4-design-decisions)
- [5. Roadmap](#5-roadmap)
- [6. Portfolio projects](#6-portfolio-projects)
- [7. Repository layout](#7-repository-layout)
- [8. Toolchain and reproduction](#8-toolchain-and-reproduction)
- [9. Engineering standards](#9-engineering-standards)
- [10. On real-time, honestly](#10-on-real-time-honestly)
- [11. References](#11-references)

---

## 1. What this repository is

A public, incremental build-up of the software stack that drives a legged
humanoid: from the joint control loop running at kilohertz on a real-time
kernel, up to the behaviour layer that decides where the robot should go.

**What it is:** a working record. Source, robot descriptions, simulation
models, controller configs, measurements and the reasoning behind each
architectural choice.

**What it is not:** a tutorial mirror, and not a claim of production
experience. Phase status below is stated honestly — `done`, `in progress` or
`planned`. Anything marked `planned` is design intent, not shipped code.
If you are evaluating this repository, [section 4](#4-design-decisions) and
[section 10](#10-on-real-time-honestly) are where the engineering judgement
lives; the rest is scaffolding.

---

## 2. The system under study

A humanoid control stack is not one program — it is a set of nested loops with
wildly different deadlines. The single most consequential design decision in
the whole system is **where each boundary falls**, because that determines the
language, the memory discipline and the failure mode of everything inside it.

```
        rate        layer                        language   timing class
   ┌──────────────────────────────────────────────────────────────────────┐
   │   ~1 Hz     │  Task / behaviour           │  Python  │  no deadline  │
   │             │  mission logic, supervision │          │               │
   ├─────────────┼─────────────────────────────┼──────────┼───────────────┤
   │  10–50 Hz   │  Planning & kinematics      │   C++    │  soft RT      │
   │             │  footstep plan, IK, MPC     │          │  (miss = jerk)│
   ├─────────────┼─────────────────────────────┼──────────┼───────────────┤
   │ 200–1000 Hz │  Whole-body control         │   C++    │  firm RT      │
   │             │  balance, impedance, QP     │          │  (miss = fall)│
   ├─────────────┼─────────────────────────────┼──────────┼───────────────┤
   │  1–8 kHz    │  Joint loop + fieldbus      │   C++    │  hard RT      │
   │             │  torque control, EtherCAT   │  no heap │  (miss = trip)│
   └──────────────────────────────────────────────────────────────────────┘
             ▲                                                    │
             │  encoder / IMU / F-T feedback     torque setpoints │
             └────────────────────────────────────────────────────┘
```

**The rule this repository follows:** C++ from sensor to actuator, Python from
supervision up. Python never closes a joint loop — the GIL and the garbage
collector introduce pauses that are not bounded, and an unbounded pause in a
1 kHz torque loop is a robot on the floor.

The corollary is that the interesting engineering is not "which language is
better" but **the seam**: how a non-deterministic supervisor hands intent to a
deterministic controller without ever blocking it. In ROS 2 that seam is
built from lock-free queues, `TRANSIENT_LOCAL` state topics and actions —
never from a synchronous service call inside a control callback.

---

## 3. Engineering constraints

These are measured properties of the development machine, not aspirations.
They are listed because they *determined* the choices in section 4 — a stack
designed against imaginary hardware is a stack that has never run.

| Constraint | Measured value | Consequence |
|---|---|---|
| GPU | Intel HD Graphics 620 (integrated, no RT cores) | Isaac Sim is out. RTX with RT cores is a hard requirement, not a recommendation. |
| RAM | 12 GB | Isaac's 32 GB floor is unreachable; large-batch RL must be staged or moved to cloud. |
| Kernel | WSL2, virtualised Linux 6.6 | Latency measurement is meaningless here — see [section 10](#10-on-real-time-honestly). |
| Host disk | Single 445 GB volume, `ext4.vhdx` on `C:` | Datasets, rosbags and RL checkpoints are excluded from the repo and stored off-volume. |
| Fieldbus hardware | none yet | EtherCAT work is protocol-level and simulated until an IgH/SOEM-capable NIC is available. |

The physics simulator therefore runs on CPU, the RL work is sized to fit, and
real-time claims are deferred to hardware that can actually substantiate them.

---

## 4. Design decisions

Each row is a decision, the alternative that was rejected, and the reason.
Decisions are revisited when the constraint that produced them changes.

| # | Decision | Rejected alternative | Rationale |
|---|---|---|---|
| 1 | **ROS 2 Jazzy Jalisco** | Kilted / Rolling | Jazzy is LTS through May 2029. A non-LTS distro forces a migration mid-plan and its packages disappear from `apt` before the plan ends. |
| 2 | **MuJoCo as the working simulator** | Isaac Sim / Isaac Lab | MuJoCo's soft-constraint solver handles contact-rich dynamics well on CPU. Isaac needs RT cores and 32 GB (see section 3). Isaac stays a cloud option, not a local dependency. |
| 3 | **Learn ROS 2 in Python first, port to C++ after** | C++ from day one | Learning both at once makes every error ambiguous — you cannot tell a robotics misconception from a syntax mistake. Separating the two failure modes is what makes the sequence tractable. |
| 4 | **C++20**, `-Wall -Wextra -Wpedantic` | C++14/17 defaults | Concepts and `std::span` remove whole categories of template error and raw-pointer arithmetic from control code. Jazzy's own ABI is built against a modern toolchain. |
| 5 | **clangd + `compile_commands.json`**, MS IntelliSense disabled | vscode-cpptools engine | clangd reads the exact colcon compile database, so ROS 2 headers resolve correctly and `clang-tidy` runs inline. Two IntelliSense engines on the same buffer fight and both lose. |
| 6 | **Interfaces (`.msg`/`.srv`/`.action`) in a dedicated package** | Interfaces beside the node | Lets the C++ node and the Python supervisor evolve independently. Coupling interfaces to an implementation package makes every consumer rebuild on every internal change. |
| 7 | **Explicit QoS per topic** | Default QoS everywhere | Sensor streams: `SensorDataQoS` (best-effort, `KEEP_LAST(1)`) — a stale IMU sample is worse than a dropped one. Commands: `RELIABLE`. Latched state: `TRANSIENT_LOCAL`. Mismatched QoS is the single most common cause of "the topic publishes but nothing arrives". |
| 8 | **`ros2_control` as the controller framework** | Bespoke control node | It is the most-requested and least-understood item in humanoid job descriptions, and its `hardware_interface` boundary is exactly the sim-to-real seam this plan needs. |
| 9 | **Real-time validation off WSL2** | `cyclictest` inside WSL2 | A virtualised kernel cannot give a meaningful worst-case latency. PREEMPT_RT numbers will come from bare metal (Raspberry Pi or dual boot) or not at all. |
| 10 | **Units encoded in identifiers** (`timeout_ms`, `angle_rad`, `torque_nm`) | Bare names + comments | Roughly half of all robotics bugs are a unit or a reference-frame confusion. The type system will not catch it; the name will. |

---

## 5. Roadmap

Ten linear phases, 36 weeks at 10–12 h/week. Phases deliberately overlap —
language study and applied work run in parallel — but **no phase begins before
its prerequisite has shipped**.

| # | Phase | Weeks | Status |
|---|---|---|---|
| 0 | Environment: WSL2, ROS 2 Jazzy, colcon, clangd, RViz2 | — | ✅ done |
| 1 | Python gaps: classes, packages, exceptions, tests | 1–3 | 🔄 in progress |
| 2 | ROS 2 concepts in Python: nodes, topics, services, params, actions, QoS | 3–8 | ⬜ planned |
| 3 | C++ from zero: compilation model, types, pointers, classes, STL | 5–14 | ⬜ planned |
| 4 | ROS 2 in C++: `rclcpp`, executors, composition | 13–18 | ⬜ planned |
| 5 | Intermediate C++: ownership, RAII, smart pointers, templates | 17–24 | ⬜ planned |
| 6 | Simulation: MuJoCo, MJCF modelling, model validation | 19–26 | ⬜ planned |
| 7 | Advanced C++: concurrency, lock-free, allocation-free hot paths | 25–30 | ⬜ planned |
| 8 | `ros2_control`: hardware interfaces and custom controllers | 27–32 | ⬜ planned |
| 9 | Reinforcement learning and sim-to-real | 31–34 | ⬜ planned |
| 10 | Real-time kernels and EtherCAT | 33–36 | ⬜ planned |

**Verification checkpoints.** A phase is not complete because its weeks
elapsed — it is complete when the checkpoint question can be answered without
looking anything up. Selected examples:

| Week | The question |
|---|---|
| 8 | Can I create a ROS 2 Python package from scratch — node, parameter, launch file — without copying from another project? |
| 12 | Can I explain pass-by-value vs by-reference vs by-pointer, and pick the right one with a reason? |
| 24 | Can I choose between `unique_ptr`, `shared_ptr` and a raw observing reference, and justify it technically? |
| 30 | Can I justify every `memory_order` I wrote, instead of defaulting to `seq_cst`? |
| 36 | Do I have a *measured* jitter number, and three documented public projects with video? |

---

## 6. Portfolio projects

Three deliverables, each proving a distinct competency cluster.

### P1 — 3-DoF arm with inverse kinematics (week 18) · `05_portfolio/p1_braco_ik`
Own URDF/xacro, `robot_state_publisher`, a C++ IK node built on Eigen, a
`MoveToPose` action server, RViz2 visualisation, GoogleTest coverage, and a
Python supervision node.
*Demonstrates:* modern C++, end-to-end ROS 2, URDF, TF2, Eigen, kinematics,
hybrid C++/Python architecture.

### P2 — Full `ros2_control` stack in simulation (week 32) · `05_portfolio/p2_ros2_control`
Custom `hardware_interface`, a custom impedance controller, Gazebo
integration, gains via YAML, a single launch entry point, GitHub Actions CI,
and integration tests with `launch_testing`.
*Demonstrates:* `ros2_control` internals, pluginlib, logical real-time
discipline, software engineering practice.

### P3 — RL locomotion (week 36) · `05_portfolio/p3_locomocao_rl`
A MuJoCo Menagerie model, a Gymnasium environment, PPO training with domain
randomisation, an exported policy, and a C++ node running inference.
*Demonstrates:* simulation, sim-to-real, RL, Python↔C++ integration.

---

## 7. Repository layout

```
humanoid-robotics-lab/
├── 00_admin/              progress log, source prompts, reference notes
├── 01_trilha_b_cpp/       C++ track — notes and compiled deliverables
├── 02_trilha_a_ros2/      ROS 2 track
│   ├── ros2_ws/src/       colcon workspace (the only tracked part)
│   └── urdf_referencia/   reference robot descriptions
├── 03_trilha_c_sim/       simulation track — MJCF models (renders ignored)
├── 04_trilha_d_rt/        real-time & EtherCAT — summarised measurements
├── 05_portfolio/          the three projects above
├── .clangd                clang-tidy / diagnostics policy
├── .gitattributes         LF normalisation + language stats
└── .gitignore             build output, datasets, media, personal material
```

Not tracked, by design: `build/`, `install/`, `log/` (regenerated by colcon and
full of absolute paths), datasets and rosbags, model weights, rendered video,
and career material. See [`.gitignore`](.gitignore) for the full policy.

---

## 8. Toolchain and reproduction

| Component | Version |
|---|---|
| OS | Ubuntu 24.04.3 LTS (WSL2) |
| ROS 2 | Jazzy Jalisco |
| Compiler | GCC 13.3.0 |
| CMake | 3.28.3 |
| Build tool | colcon |
| Python | 3.12.3 |
| Language standard | C++20 |

```bash
# Build the ROS 2 workspace
cd 02_trilha_a_ros2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

```bash
# Run the tests
cd 02_trilha_a_ros2/ros2_ws
colcon test --event-handlers console_direct+ && colcon test-result --verbose
```

`--symlink-install` is not cosmetic: it makes Python nodes and config files
editable without a rebuild, which is the difference between a two-second and a
ninety-second iteration loop.

---

## 9. Engineering standards

Rules applied to everything committed here.

**C++**
- `const` by default. A parameter that does not change is `const&`; a method
  that does not mutate is `const`.
- Ownership is explicit: `unique_ptr` for sole ownership, a raw reference to
  observe, `shared_ptr` only when ownership is genuinely shared.
- No allocation, no locks, no I/O in a control `update()`. Pre-allocate,
  `mlockall`, and hand data across the boundary lock-free.
- Compile with `-Wall -Wextra -Wpedantic`; warnings are defects.

**Python**
- Type hints everywhere — they train the static-typing habit C++ will demand.
- One module, one responsibility. An 800-line file is a decomposition failure.
- Never `pip install` into the system interpreter; Ubuntu 24.04 blocks it on
  purpose. Use a venv.

**ROS 2**
- One node, one responsibility. A node that does everything cannot be tested.
- Every magic constant becomes a parameter, with a descriptor and a valid range.
- Never block a callback. Long work goes to an action or a separate thread.
- Record `ros2 bag` during tests. Debugging a robot without a recorded log is
  guessing.

**Measurement**
- Log the random seed and every version. A non-reproducible experiment is not
  a result.
- Validate a simulation model before trusting it: energy conservation,
  pendulum period, free-fall time.

---

## 10. On real-time, honestly

A large share of "real-time robotics" claims do not survive contact with a
latency histogram. This section states what can and cannot be demonstrated
here.

**What "real-time" means.** Not "fast" — *bounded*. A 1 kHz joint loop has a
1000 µs budget. What matters is not the mean iteration time but the worst
case over hours of running under load. A controller averaging 200 µs with a
3 ms outlier once a minute is a broken controller.

**Where the jitter comes from.** Page faults on first touch (fixed with
`mlockall(MCL_CURRENT | MCL_FUTURE)`), heap allocation in the hot path (fixed
by pre-allocating), priority inversion (fixed with priority inheritance
mutexes), CPU frequency scaling and SMIs (fixed by pinning, isolating cores
and disabling deep C-states). A stock kernel adds its own scheduling latency
on top, which is what PREEMPT_RT exists to bound.

**Why no numbers here yet.** WSL2 runs a virtualised kernel under a Windows
hypervisor. `cyclictest` inside it measures the hypervisor's scheduling
behaviour, not the robot's. Publishing those numbers would be misleading, so
they are not published. The real-time phase moves to bare metal — a
PREEMPT_RT Raspberry Pi or a dual boot — and the histogram goes in
`04_trilha_d_rt/medicoes/` when it exists.

**EtherCAT.** The fieldbus is where the timing budget is actually spent.
Distributed Clocks synchronise slaves to sub-microsecond, but only if the
master's cycle is itself jitter-bounded — which puts the whole burden back on
the kernel above. Until fieldbus hardware is available, this track stays at
the protocol level with SOEM and `ethercat_driver_ros2`.

---

## 11. References

**ROS 2** — [docs.ros.org/en/jazzy](https://docs.ros.org/en/jazzy/) ·
[control.ros.org](https://control.ros.org/) ·
[ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)

**C++** — [learncpp.com](https://www.learncpp.com/) ·
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) ·
Meyers, *Effective Modern C++* · Williams, *C++ Concurrency in Action* (2nd ed.)

**Simulation** — [mujoco.readthedocs.io](https://mujoco.readthedocs.io/) ·
[mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)

**Real-time & fieldbus** — [Linux Foundation RT wiki](https://wiki.linuxfoundation.org/realtime/start) ·
[SOEM](https://github.com/OpenEtherCATsociety/SOEM) ·
[ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2)

**Theory** — Lynch & Park, *Modern Robotics* · Siciliano & Khatib,
*Springer Handbook of Robotics* · Sutton & Barto, *Reinforcement Learning*

---

## License

[Apache License 2.0](LICENSE) — the same license used by ROS 2 itself, so code
here can be contributed upstream without a relicensing step.

---

**Marcelo Silva de Almeida** — moving from data engineering into robotics
software. Built in the open, phase by phase.
