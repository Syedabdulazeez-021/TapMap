# TapMap — Complete Project Explanation
### Everything you need to answer any question in the review
**Group 11 · CSE-AI Batch A · Mid Review 9 September 2026**

---

## 1. THE PROBLEM, IN PLAIN LANGUAGE

### What is the defect?

When a tile is laid, adhesive bonds it to the surface underneath. When concrete is poured, it bonds around steel reinforcement bars. Over time that bond can fail — water gets in, the adhesive degrades, or rebar rusts and expands — and a **thin air gap** forms between the surface layer and the base underneath.

This is called **debonding** (in tiles) or **delamination** (in concrete).

The critical property: **you cannot see it.** The surface looks completely intact. There is no crack, no stain, no visible sign. The tile is sitting there held by nothing but friction and its neighbours.

### Why it matters

- **Façade cladding** — tiles and stone on building exteriors debond and eventually fall. In Indian cities this kills and injures pedestrians every year.
- **Bridge decks and slabs** — rebar corrodes, expands 2–6× in volume, and cracks the concrete cover from the inside. Delamination is the earliest detectable symptom of internal corrosion damage.
- **Floors** — hollow tiles crack under load, pop up, and turn a spot repair into re-laying an entire floor.

### How it is detected today

The standard field method is **percussion sounding**, formalised in ASTM D4580. An inspector walks the surface striking it with a small hammer or steel rod, and listens.

That is it. A trained human ear.

### Why tapping works — the physics

When you strike a surface, you inject a short burst of energy. What happens to that energy depends on what is underneath.

**Bonded tile:** the energy passes straight into the substrate and dissipates. The tile barely moves. You get a **sharp, high-frequency, fast-decaying "tick"** — roughly 2 kHz, dying away in a few milliseconds.

**Debonded tile:** the air gap means the tile is effectively a small drum head. It flexes, vibrates at its own natural frequency, and rings. You get a **dull, low-frequency, slow-decaying "thud"** — roughly 500 Hz, ringing for tens of milliseconds.

Two things change: the **frequency content** shifts downward, and the **decay time** gets longer. Both are measurable.

### Why the human method is not good enough

| Problem | Consequence |
|---|---|
| Subjective | Depends entirely on one inspector's ear and experience |
| No record | No coordinates, no map, no evidence anyone can re-check |
| Slow and tiring | Fatigue degrades accuracy across a large area |
| Not repeatable | Two inspectors produce two different results |

### Why the expensive alternatives are not the answer

Ground-penetrating radar, infrared thermography, ultrasonic array scanners — all work, all cost several lakh rupees, all need a trained operator, and all are still hand-held. A municipal body or a housing society will never own one.

### The gap TapMap fills

Nobody has cheaply **robotised** percussion sounding with spatial mapping. That is the entire project in one sentence:

> **Inspectors find hollow tiles by tapping and listening. TapMap does the tapping, the listening, and the mapping — autonomously — and hands you a defect map instead of a guess.**

---

## 2. ONE COMPLETE SCAN, TRACED END TO END

This is the most useful thing to have straight in your head. If you can narrate this, you can answer almost anything.

### Phase 1 — EXPLORE (the robot finds its own workspace)

You place the robot in a corner of the room. You do not type in any dimensions.

1. Odometry is zeroed. The `map` frame is anchored at this exact spot.
2. The robot drives forward. The ToF sensor points forward, measuring distance continuously.
3. When the ToF reads under 15 cm, a wall is there. The robot stops and records the distance travelled as **length L**.
4. It rotates exactly 90°, closed-loop on the fused gyro heading.
5. Drives again until the next wall. Records **width W**.
6. Returns to origin.

The robot now knows it is working in an L × W rectangle.

*Why one forward sensor is enough:* the robot itself acts as the pan mechanism. Instead of rotating a sensor, we rotate the whole robot. This is exactly why we have only one ToF and still get 2D information.

### Phase 2 — CALIBRATE (learn this surface)

The robot performs 10 taps on an area assumed good. From these it learns the baseline acoustic signature of *this particular* surface — ceramic tile, concrete, marble, plaster all sound different, and even the same tile type differs by thickness and adhesive.

DL-2 (surface classifier) identifies the surface type and loads matching classifier weights.

*Why this matters:* without calibration, a model trained on one tile type fails on another. This step is what makes TapMap deployable beyond a single demo panel.

### Phase 3 — SCAN (the main loop)

The coverage planner generates a **boustrophedon** path — the lawnmower zig-zag — over the discovered rectangle at a fixed spacing (5 cm in our tests, giving ~187 tap points over 0.8 × 0.5 m).

For every point on that grid:

1. **DRIVE** — the line tracker steers the robot to the waypoint using fused odometry. Speed ~0.10 m/s.
2. **STOP** — full stop, zero velocity commanded.
3. **SETTLE** — wait 400 ms. This matters: after stopping, the chassis is still micro-vibrating. Tapping too early contaminates the recording with the robot's own ringing.
4. **STRIKE** — the ESP32 energises the solenoid for exactly 30 ms. The rubber-tipped plunger hits the surface.
5. **LISTEN** — at the moment of the trigger, a 100 ms audio capture window opens on the I²S microphone via DMA. Only 100 ms. Only during the strike.
6. **CLASSIFY** — the window is transformed into features and passed to the classifier, which returns `bonded` or `debonded` plus a confidence.
7. **MAP** — the mapper looks up where the *striker tip* was at that instant (TF at `tapper_link`, not robot centre) and writes the result into that grid cell.
8. **NEXT** — planner advances to the next waypoint.

Repeat 187 times.

### Phase 4 — RETURN

After the final tap, the robot drives back to (0, 0) — its start corner — and measures how far off it lands. In our last full run: **1.47 cm** over 187 taps.

*Why this is clever:* it is a free, self-administered accuracy audit. If the robot returns within 1–2 cm of its start mark, odometry held for the entire scan, which means every coordinate on the map is trustworthy. If it returned 15 cm off, the map would be worthless and you would know immediately.

### Phase 5 — REFINE AND REPORT

DL-3 (region refiner) passes over the accumulated grid, removes isolated single-cell false positives, and merges scattered hits into contiguous defect regions with area estimates.

The report generator renders a PDF: heat map image, tap-by-tap table with coordinates and confidences, spectrograms of flagged cells, total defect area, scan duration, and the return-home error stated as an accuracy metric.

---

## 3. EVERY COMPONENT, AND WHY IT IS THERE

For each: what it does, and **what breaks if you remove it**.

### Raspberry Pi 4 — the brain
Runs the complete ROS 2 Jazzy stack: coverage planner, localisation, classifier, mapper, report generator.
**Remove it:** no autonomy. The robot becomes a remote-controlled tapper needing a laptop tethered to it.

### ESP32-WROOM-32 — the reflexes
Runs micro-ROS. Handles the 100 Hz motor PID loop, encoder pulse counting in hardware, solenoid gating, and I²S audio capture.
**Remove it:** the Pi runs Linux, which is not a real-time OS. The kernel will preempt a process at arbitrary moments. A 100 ms audio window gated to a 30 ms strike needs microsecond-level timing consistency — Linux cannot guarantee that, and the ESP32 can.

**This split is the architecture. Say it exactly like this:** *the Pi thinks, the ESP32 reflexes.*

### N20 12 V geared motors WITH ENCODERS ×2 — drive and localisation
100:1 gearbox, 7 PPR magnetic encoder = 700 ticks per wheel revolution = **0.19 mm of travel per tick**.
**Remove the encoders:** the project collapses. Without tick counting the robot has no idea where it is, cannot stamp taps to coordinates, and cannot build a map. It becomes a robot that taps randomly and reports nothing. This is why the lab's plain BO motors could not be used.

### 43 mm wheels + rear ball caster — differential drive geometry
Two driven wheels on the centre axis, one free caster at the back.
**Use 4-wheel skid steer instead:** turning drags wheels sideways. That slip corrupts the encoder distance readings, and the defect map visibly smears. Differential drive rotates about its own centre with no slip — this is why robot vacuums and virtually every indoor mapping robot use this layout.

### 12 V push-pull solenoid (JF-0530B) — the tapper
30 ms electrical pulse, ~5 N force, rubber-tipped striker.
**Use a servo hammer instead:** a servo cannot guarantee constant impact energy. Varying strike energy means varying acoustics, which means the classifier is comparing apples to oranges between cells. The solenoid delivers **repeatable impulse energy** — that repeatability is what makes the acoustic problem learnable at all.

### IRLZ44N MOSFET + 1N5822 diode + 1000 µF capacitor — the drive stage
- **MOSFET:** the ESP32's 3.3 V pin can source a few milliamps. The solenoid draws about 1 A. The MOSFET is the switch that lets a tiny signal control a large current.
- **Flyback diode:** when the coil's current is switched off, the collapsing magnetic field generates a large reverse voltage spike. The diode gives it a path to dissipate. **Without it, the ESP32 dies.** Not optional.
- **Capacitor:** supplies the 1 A strike locally so the 5 V rail does not dip and reboot the Pi mid-scan.

### INMP441 I²S MEMS microphone — the ear
Digital MEMS microphone, 61 dB SNR, output is I²S digital.
**Use an analog electret instead:** the motor driver switches PWM at kilohertz rates, radiating electrical noise. An analog mic's signal wire picks that up before it is digitised. The INMP441 digitises *at the sensor*, so PWM noise never enters the signal path. This single choice is the difference between clean and unusable data.

### MPU6050 IMU — heading
6-axis accelerometer and gyroscope, mounted at the chassis rotation centre on foam tape.
**Remove it:** encoders are excellent at measuring distance and poor at measuring heading (a tiny wheel-diameter mismatch compounds into a large angular error). The gyro is the opposite. Fusing them in the EKF is what keeps scan rows straight and makes 180° turns land accurately.

### VL53L0X ToF laser ranger — boundary reference
Laser time-of-flight, 3–120 cm, ±3 %.
**Two jobs:** discovering the workspace in Phase 1, and correcting lateral odometry drift during long scans.
**Why not HC-SR04 ultrasonic:** it emits a 40 kHz acoustic chirp — right next to our microphone. Also cm-resolution instead of mm.

### L298N motor driver
Takes logic-level direction and PWM signals, switches motor current.
**Why not DRV8833 (our original choice):** the battery is 12 V. DRV8833's absolute maximum is 10.8 V — it would be destroyed. L298N is 12 V rated. Its ~2 V internal drop actually lands the 12 V motors at a safe ~10 V.

### XL4015 5 A buck converter
Steps 12 V down to a stable 5 V / 3 A for the Pi.
**Why not LM2596:** practical output around 2 A. A Pi 4 under load can pull close to 3 A. Undervoltage causes throttling and random reboots.
**Critical:** it ships at an arbitrary output voltage. Set it to 5.0 V with a multimeter with nothing connected, *before* the Pi is ever attached.

### SSD1306 OLED — on-robot status
Shows IP address, scan percentage, battery, return error.
**Why it earns its place:** the robot remains legible when no laptop is attached. Judges see live state on the machine itself.

### E-stop and rocker switch — safety
The E-stop is wired into the **actuator power branch only**: it kills motors and solenoid instantly while the logic stays alive to report state. That is the correct safety pattern, and reviewers ask about it.

---

## 4. THE ROS 2 ARCHITECTURE

### Why ROS 2 at all

ROS 2 is not a library we import for convenience. It is the **nervous system**. The project is a set of independent programs (nodes) that must exchange data reliably at different rates, with different reliability guarantees, across two physical computers. That is precisely the problem ROS 2 exists to solve.

Remove ROS 2 and you have: motors that do not know where to go, a solenoid that does not know when to fire, a microphone recording sounds nobody interprets, and no map. A dead body with no nervous system.

### The computation graph

**On the ESP32 — one node, many interfaces:**

```
Node: tapmap_firmware  (micro-ROS client)
  publishes : /wheel_ticks, /imu/data_raw, /tof/range,
              /audio/tap_window, /battery
  subscribes: /cmd_vel, /oled/text
  serves    : /tapper/fire
```

*Why one node and not several:* the ESP32 has ~320 KB of RAM. Each additional micro-ROS node adds DDS overhead. The professional pattern on a memory-constrained MCU is a single driver node exposing all hardware. **Node count is not a quality metric — the reason for the split is.**

**On the Pi — the processing nodes:**

| Node | Responsibility |
|---|---|
| `micro_ros_agent` | Bridges the ESP32 into the ROS 2 graph over USB serial |
| `base_controller` *(lifecycle)* | Wheel ticks → differential-drive kinematics → `/odom` + TF broadcast |
| `imu_filter` | Raw IMU → filtered orientation |
| `ekf_localization` | Fuses wheel odometry with IMU yaw → `/odometry/filtered` |
| `boundary_explorer` | Hosts `/discover_area` action — Phase 1 workspace discovery |
| `coverage_planner` *(lifecycle)* | Hosts `/scan_area` action — grid generation, mission sequencing, return-home |
| `tap_scheduler` | Settle timing, fires the tap, retries on low SNR |
| `surface_classifier` | DL-2 — identifies surface type during calibration |
| `audio_classifier` *(lifecycle)* | DL-1 — features → classification → `/tap/classification` |
| `defect_mapper` | Builds `/defect_grid` (OccupancyGrid) and `/defect_markers` |
| `defect_refiner` | DL-3 — cleans and merges defect regions |
| `report_generator` | Renders the PDF inspection report |
| `system_monitor` | Battery, heartbeats, safe-stop on fault |

### Topics — and the QoS reasoning

| Topic | Type | Rate |
|---|---|---|
| `/wheel_ticks` | custom WheelTicks | 50 Hz |
| `/imu/data_raw` | sensor_msgs/Imu | 100 Hz |
| `/tof/range` | sensor_msgs/Range | 20 Hz |
| `/odom` → `/odometry/filtered` | nav_msgs/Odometry | 50 Hz |
| `/cmd_vel` | geometry_msgs/Twist | 20 Hz |
| `/audio/tap_window` | Float32MultiArray | per tap |
| `/tap/classification` | custom TapResult | per tap |
| `/defect_grid` | nav_msgs/OccupancyGrid | 1 Hz |
| `/defect_markers` | visualization_msgs/MarkerArray | per tap |

**QoS design — this is a strong point to raise:**
Sensor streams use **BEST_EFFORT**. Audio windows and classification results use **RELIABLE / KEEP_ALL**.
*Reasoning:* losing one IMU packet out of a hundred per second changes nothing — the next one arrives in 10 ms. Losing one tap classification leaves a permanent hole in the map. Different data, different guarantees. Applying the same QoS to everything would be lazy engineering.

### Services

| Service | Purpose |
|---|---|
| `/tapper/fire` | One gated tap: energise 30 ms, capture 100 ms, return status |
| `/calibrate_surface` | 10-tap baseline routine for a new surface |
| `/generate_report` | Render and save the PDF |
| `/reset_odom` | Zero the pose at the start corner |
| `/estop` | Software emergency stop (hardware E-stop also exists) |

### Actions — and why actions, not services

| Action | Goal / Feedback / Result |
|---|---|
| `/discover_area` | max dimensions → progress → discovered L, W |
| `/scan_area` | width, height, spacing, return_home → % complete, taps done, defects found, current pose → total taps, defect cells, **return_error** |

**Why an action and not a service:** a scan takes minutes. Services are request-response and block. Actions stream **feedback** while running and are **cancellable**. A judge pressing cancel mid-scan and watching the robot halt gracefully is a deliberate demo beat — and it is only possible because this is an action.

### The TF tree

```
map ──(static, anchored at scan start)──► odom ──(EKF)──► base_link
                                                            ├── imu_link
                                                            ├── tof_link
                                                            ├── mic_link
                                                            └── tapper_link   ← taps stamped HERE
```

**The `tapper_link` detail is worth raising unprompted.** The striker sits about 6 cm ahead of the robot's centre. If we stamped taps at `base_link`, every single point on the map would be shifted 6 cm from where the strike actually landed. Using TF to resolve the striker's pose at the moment of impact is what makes the map geometrically correct.

### Why NOT Nav2 — a deliberate omission

Nav2 is the standard ROS 2 navigation stack. We do not use it, and that is a decision, not an oversight.

Nav2 assumes an obstacle-rich environment: costmaps, global and local planners, recovery behaviours, dynamic obstacle avoidance. Our robot operates on a **flat, empty, known rectangle**. None of that machinery applies. Bolting it on would add configuration complexity, CPU load, and failure modes in exchange for nothing.

Instead we built a purpose-fit coverage planner: boustrophedon grid generation, a line tracker, and closed-loop turns on fused yaw. It matches the problem exactly.

**Nav2 is on the future-work list** for cluttered real-world sites where obstacle avoidance actually matters.

*Being able to defend an omission scores higher than using a stack decoratively.*

---

## 5. THE HEAT MAP — CELL BY CELL

This is the project's core output, so understand it thoroughly.

### Step 1 — The grid exists before any tap

A `nav_msgs/OccupancyGrid` is created over the scan area. Resolution 5 cm per cell. Origin at the robot's start corner. Every cell initialised to **−1**, meaning *unknown*, which renders grey in RViz.

### Step 2 — Every tap gets a position

When the classifier publishes a result, `defect_mapper` performs a **TF lookup at `tapper_link`** — the striker tip's pose in the map frame at that moment. Not the robot's centre. That 6 cm offset would otherwise shift the whole map.

### Step 3 — The cell is written with confidence weighting

```
bonded   → cell = 0
debonded → cell = round(100 × confidence)
```

So a debonded tap at 0.95 confidence writes 95; a marginal one at 0.62 writes 62. **The map encodes certainty, not a binary yes/no.** If the same cell is tapped more than once, values combine as a running average.

### Step 4 — Two visual layers

- **`/defect_grid`** → rendered as the Map display in RViz. This is the heat map itself.
- **`/defect_markers`** → a coloured cylinder at each individual tap point. Green = bonded, red = debonded. This shows exactly where each strike landed, which the grid alone does not.

### Step 5 — DL-3 refinement

After the scan, the region refiner smooths the grid. Real debonding is spatially contiguous — a single red cell surrounded by eight green cells is almost certainly noise. The refiner removes isolated false positives and merges scattered hits into coherent regions with area estimates.

### Step 6 — Export

The report generator renders the final map into a PDF alongside the tap table, spectrograms of flagged cells, total defect area, and the return-home error.

### The key contrast to draw

Many field maps (gas concentration, temperature, signal strength) are **interpolated** — sparse samples, with everything between them mathematically estimated.

**Our map is directly measured.** Every coloured cell was physically struck at that coordinate. Nothing is guessed, diffused, or interpolated between samples. That is a materially stronger claim and it is easier to defend.

---

## 6. WHERE DEEP LEARNING SITS — AND WHY

Three models. Each solves a problem that hand-written rules genuinely cannot.

### DL-1 — Tap Acoustic Classifier (CORE)

- **Input:** 100 ms window at 16 kHz → MFCC features (13 coefficients × ~9 frames)
- **Model:** small CNN, under 100k parameters, ~2 ms inference on the Pi
- **Output:** bonded / debonded + confidence

**Why learning is genuinely the right tool here:** the difference between the two classes is not a single threshold. It lives in the *joint pattern* of spectral centroid, decay envelope, harmonic structure, and resonance peaks — and that pattern shifts with tile type, thickness, grout, and mounting. Hand-tuning rules for that generalises badly.

**We have direct evidence of this.** Our first rule-based classifier used spectral centroid with a 1200 Hz threshold. On clean signals it separated perfectly (hollow 661 Hz, solid 2197 Hz). Add a realistic noise floor and the hollow case jumped to 1542 Hz — above the threshold — and it misclassified everything. Wideband noise dragged the centroid up. We fixed it by switching to a **band-energy ratio** (150–1000 Hz vs 1000–4000 Hz), which is noise-robust because it ignores the out-of-band energy entirely. Separation became 0.008 vs 0.997.

That experience is exactly the argument for learned features: a single hand-picked statistic is brittle, and finding a robust one took analysis and iteration. A CNN learns the discriminative structure from data.

**Fallback:** the band-ratio rule classifier ships alongside the CNN. It runs if the model fails to load, and it serves as the **ablation baseline** for the paper — "CNN vs rule-based, X% vs Y%."

### DL-2 — Surface Type Classifier (SUPPORT)

- **Input:** the 10 calibration taps
- **Model:** lightweight classifier on MFCC statistics
- **Output:** ceramic / concrete / marble / plaster

**Why:** a model trained on ceramic tile fails on concrete. DL-2 identifies the surface during the 30-second calibration and loads matching DL-1 weights. **This is the mechanism that makes TapMap deployable beyond one demo panel**, and it is the direct answer to "does this generalise?"

### DL-3 — Defect Region Refiner (SUPPORT)

- **Input:** the raw defect occupancy grid
- **Model:** compact U-Net-lite over the grid
- **Output:** contiguous defect regions with area estimates

**Why:** real debonding is spatially contiguous. Isolated single-cell hits are noise. This turns a speckled grid into a professional-looking defect map and produces the area figures an inspection report needs.

### Why navigation is NOT learned — anticipate this question

A learned navigation controller on a known, flat, rectangular workspace would add: a training data requirement, a training pipeline, unpredictable failure modes, and inference latency in the control loop. It would add **zero** accuracy over a classical line tracker and PID.

Learning is applied where rules genuinely fail — **perception**. Motion control is a solved classical problem here, and using DL there would be engineering theatre.

Say this confidently. It pre-empts the weakest question anyone could ask.

---

## 7. WHAT IS BUILT VS WHAT IS NEXT

### Working today — demonstrable live

- **Raspberry Pi 4** flashed with Ubuntu 24.04 Server 64-bit, **ROS 2 Jazzy** installed, headless over SSH
- **Complete ROS 2 workspace** (`tapmap_ws`) building and running **on the Pi's onboard computer**
- **Custom interfaces:** `TapResult.msg`, `ScanArea.action`
- **Five nodes running:** sim_firmware, coverage_planner (action server), audio_classifier, defect_mapper, static TF publisher
- **Full autonomous scan verified:** 187 tap points over 0.8 × 0.5 m at 5 cm spacing
- **Correct defect localisation:** 16 defect cells found, spatially matching a hollow region the robot was never told about
- **Return-to-origin accuracy: 1.47 cm** after 187 taps
- **Live heat map in RViz 2** — green markers on bonded taps, red patch over the debonded region
- **Hardware architecture finalised and costed** at approximately ₹3,000
- **Procurement complete** — Pi 4, ESP32, MPU6050, VL53L0X, OLED, L298N, chassis from the lab; motors, solenoid, INMP441, buck converter, wiring purchased and received

### Immediately next

- Set XL4015 to 5.0 V, mount motors and wheels, wire the power tree
- micro-ROS firmware on the ESP32: encoder ticks, motor PID, `/cmd_vel`
- `base_controller` and EKF fusion on real hardware
- **Gate:** 2 m out-and-back within 2 cm across ten runs

### Then

Solenoid drive stage → gated I²S capture → record 600+ labelled taps → train DL-1 → boundary discovery → full autonomous scan on the real demo panel → DL-2, DL-3, PDF report → hardening and demo rehearsal.

### Future work (deliberately short)

Nav2 for cluttered sites · void **depth** estimation from resonance frequency (f = C/2d) · non-rectangular workspaces · façade-climbing variant · multi-robot tiling of large decks · cloud dashboard and BIM overlay.

---

## 8. THE NUMBERS YOU QUOTE

| Figure | Meaning |
|---|---|
| **187** | Tap points in the last full scan (0.8 × 0.5 m at 5 cm spacing) |
| **16** | Defect cells correctly identified |
| **1.47 cm** | Return-to-origin error after 187 taps — the odometry self-audit |
| **0.008 vs 0.997** | Band-energy ratio for bonded vs debonded — the class separation |
| **0.19 mm** | Odometry resolution per encoder tick |
| **~2 ms** | CNN inference time per tap on the Pi |
| **100 ms** | Audio capture window, gated to the strike |
| **30 ms** | Solenoid pulse duration |
| **400 ms** | Settle time before each tap |
| **~₹3,000** | Total build cost |
| **Several lakh** | Cost of commercial acoustic NDT instruments |

---

## 9. QUESTION AND ANSWER DRILL

**Q: Is this a robotics project or just IoT?**
It plans coverage, localises itself through sensor fusion, executes closed-loop motion, and produces a spatial occupancy grid. Sense → plan → act → localise → map is the complete robotics loop. An IoT device would be a sensor on a fixed stand with a human moving it.

**Q: Where exactly is ROS 2 implemented?**
Every subsystem is a ROS 2 node. The ESP32 itself runs micro-ROS and appears in the graph. The deliverable — the defect heat map — is a `nav_msgs/OccupancyGrid`, a standard ROS 2 type, rendered live in RViz 2. Remove ROS 2 and nothing coordinates.

**Q: Why do you need both a Pi and an ESP32?**
The Pi runs Linux, which is not real-time. Gating a 100 ms audio window to a 30 ms strike with under 2 ms of jitter requires deterministic timing the kernel cannot guarantee. The ESP32 provides that. Conversely the ESP32 cannot run ROS 2 or a CNN in 320 KB of RAM. Pi thinks, ESP32 reflexes.

**Q: Ambient noise in a hall will ruin the classification.**
Capture is a 100 ms window gated to the strike — we only listen during the tap, when the transient dominates ambient by a wide margin. We also train with hall noise mixed into the dataset, and re-tap when SNR falls below a floor.

**Q: How do you know the map coordinates are correct?**
The robot returns to its start point and reports its own position error. Our last run landed 1.47 cm from origin after 187 taps. If odometry had drifted, that number would expose it immediately.

**Q: Why not use a camera?**
The defect is subsurface. There is nothing to see — that is the entire premise. A camera would be useful for surface rust staining and spalling, which is on the future-work list, but it cannot detect a void under an intact tile.

**Q: Does this detect corrosion?**
Not directly, and we are careful about that claim. It detects **delamination**, which is the primary structural *consequence* of rebar corrosion — rust expands 2–6× and cracks the concrete cover from within. This is exactly why highway agencies use tap sounding as the standard field proxy for internal corrosion damage. Direct electrochemical corrosion detection (half-cell potential) is future work.

**Q: Why not Nav2?**
Nav2 assumes obstacle-rich costmap navigation. On a flat empty rectangle it is unjustifiable overhead with no benefit. We built a coverage planner that matches the problem. Nav2 becomes relevant for cluttered real sites, which is on the roadmap.

**Q: Why is navigation not deep learning?**
On a known rectangular grid a learned controller adds training burden, unpredictable failure modes and inference latency for zero accuracy gain. We apply learning where rules genuinely fail — acoustic classification.

**Q: What if the room is not rectangular?**
Version 1 assumes a convex rectangular workspace, with a max-dimension safety parameter so a missed wall reading cannot send the robot across the building. Perimeter tracing for concave rooms is future work.

**Q: Why differential drive instead of four-wheel?**
Skid steering drags wheels sideways through every turn. That slip corrupts encoder-based odometry and visibly smears the defect map. Two driven wheels plus a caster rotate about the robot's own centre with no slip.

**Q: Why not use the lab's existing motors?**
They have no encoders. Encoders are the localisation. Without tick counting there is no position, no coordinates, no map.

**Q: How accurate is the classifier?**
Currently we have verified perfect separation in simulation with a noise-robust band-energy feature: 0.008 for bonded, 0.997 for debonded. Real-hardware accuracy figures come after the dataset is recorded — target is ≥90% cross-validated, with a live blind test where a teammate relocates the hollow tile.

**Q: What is your dataset?**
We are recording it ourselves — 300+ labelled taps per class on our demo panel, with variations in position and ambient noise. No public labelled tap-acoustic dataset of this kind exists at scale, so the dataset is itself a contribution.

**Q: How is this different from a phone app that listens to taps?**
A phone cannot self-localise, cannot systematically cover an area, and cannot produce a spatial map. It also cannot deliver a repeatable strike. The autonomy and the mapping are the contribution.

**Q: Solenoid wear over thousands of taps?**
Rubber-tipped striker, current-limited drive, 30 ms pulses with a large duty-cycle margin. Coil heating is negligible at this duty cycle.

**Q: What happens if Wi-Fi drops mid-scan?**
Nothing. The Pi is onboard and runs the entire mission. Wi-Fi carries only the optional RViz view. Close the laptop and the scan continues.

**Q: What is your emergency stop?**
A latching switch wired into the actuator power branch. It kills motors and solenoid instantly while logic power stays on so the robot can still report its state. Plus a 200 ms firmware deadman that stops motors if velocity commands cease.

**Q: What is the cost, and how does it compare?**
About ₹3,000 total. Commercial acoustic NDT instruments run to several lakh. That is the difference between a lab instrument and something a municipality could actually deploy at scale.

**Q: What is the startup or commercial angle?**
Façade safety audits and post-construction quality assurance, sold as inspection-as-a-service: hardware plus per-scan report generation. Every façade-collapse news cycle creates demand for documented cladding audits.

**Q: Is this publishable?**
Yes — three contributions: an integrated low-cost percussive scan-and-map method, a labelled bonded/debonded tap-acoustic dataset, and an open ROS 2 reference architecture for audio-based mobile NDT.

**Q: What is the biggest risk in your project?**
Odometry drift. Everything downstream depends on knowing the robot's position. That is why the 2 m out-and-back test within 2 cm is a hard gate before we proceed to the tapper — a map built on 10 cm of drift is worthless no matter how good the classifier is.

**Q: What have you actually built versus planned?**
The complete ROS 2 stack runs today on the robot's onboard Pi, executes a full autonomous scan, correctly localises a defect it was not told about, and returns to origin within 1.47 cm. Hardware is procured and assembly is underway. Software is deliberately ahead of hardware — when the parts are mounted we plug them into a stack that already works.

---

## THE ONE-LINE PITCH

> **Inspectors find hidden hollow tiles by tapping and listening. TapMap is a ₹3,000 robot that does the tapping, the listening, and the mapping — autonomously — and hands you a defect heat map instead of a guess.**
