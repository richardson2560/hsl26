# HSL26 — Final Engineering Architecture (Independent Audit and Delivery Structure)

**Role:** development team lead / independent technical auditor.
**Audited inputs:** `Регламент_HSL26_-_v06092026.pdf` (official rulebook, 6 pp.), `DESIGN_OVERVIEW.md` (qualification-stage system, fixed sensor), `audit1.md` (reviewer proposal: evolutionary ASG + spectral signatures + coevolution), `audit2.md` (reviewer proposal: rejection of pure OT/PH-CPG/MPPI, A*+DWA+CBF hierarchy), `DESIGN_OVERVIEW_FINAL.md` (previous synthesis, rev. 2.0), 
**Method:** no reviewer claim is accepted on authority alone. Every equation cited below was re-derived or dimensionally cross-checked by me before being accepted; every requirement was verified against the literal text of the PDF. Discrepancies found between reviewers are resolved explicitly in §0 and §10.

---

## 0. Audit verdict (what is accepted, corrected, or discarded)

### 0.1 What both reviewers got right and is kept

- **Separating "what I want to achieve" (tactics) from "what command I can execute" (control+safety)** is the correct idea and is exactly the option/policy separation from options theory (Sutton, Precup & Singh 1999). Kept as the backbone.
- **Discarding neural networks** is reasonable given the available time, the absence of labeled data for the real opponent, and the 100%-on-CPU execution requirement (R11). This is not a limitation of the problem; it is a correct risk-management decision.
- **audit2.md is right to discard** Wasserstein-2 optimal transport and the 32D Port-Hamiltonian generator as the control core: both are designed for continuous densities or for arms/legs with rich null spaces; a 2D differential base does not have that problem, and forcing those tools adds cost without demonstrable benefit. **Independently verified**: the control space of a differential robot is $\mathbb R^2$ ($v,\omega$), and primitives (arcs, trapezoidal profiles) solve it in closed form; there is no null subspace that would justify lifting to $\mathbb R^{32}$.

### 0.2 Real mathematical errors found in the proposals (verified by me, not just cited)

| # | Original claim | My own verification | Correction |
|---|---|---|---|
| E1 | audit1.md: distance CBF with sign `−∇h·ṗ+γh≥0` | With $h(p)=\lVert p-o\rVert^2-R^2$ (safe if $h\ge0$) and $\dot p=v\,e(\theta)$: $\dot h=2(p-o)^Te(\theta)v$. The Nagumo/CBF invariance condition is $\dot h\ge-\gamma h$, i.e. $\dot h+\gamma h\ge0$. With the proposed negative sign, moving away from the obstacle ($\dot h>0$) could *violate* the constraint for large $\gamma h$ — the opposite of the intent. | `2(p−o)ᵀe(θ)v + γh ≥ 0` (positive sign). Confirmed against Ames, Xu, Grizzle & Tabuada, *CBF-QP*, IEEE TAC / arXiv:1609.06408. |
| E2 | audit1.md: reinforcing Dirichlet-Multinomial "collapses the transition to deterministic" | Counterexample I constructed: a real Bernoulli(0.5,0.5) transition. With $A=\sum\alpha_j\approx200$, $\mathrm{Var}(p_j)=\bar p_j(1-\bar p_j)/(A+1)\approx0.25/201\approx0.00124$, std. dev. $\approx0.035$: the **estimate** of $p$ converges, but the entropy of the **physical transition** remains at $\log 2$. Reducing epistemic variance of the parameter does not reduce the aleatoric randomness of the event. | Explicitly distinguish parameter uncertainty (reducible with data) from outcome uncertainty (not reducible without changing the physics/opponent). Do not use "variance collapse" as an argument for tactical determinism. |
| E3 | audit1.md: first non-zero eigenvalue of the Laplacian called interchangeably $\lambda_1$ and used as $\lambda_2-\lambda_1$ | For the symmetric normalized Laplacian $L_{sym}=I-D^{-1/2}WD^{-1/2}$ of a connected graph, $\lambda_1=0$ (constant eigenvector) and $\lambda_2$ is the Fiedler value. Under that convention, $\lambda_2-\lambda_1\equiv\lambda_2$: the "spectral gap" in the sketch is a tautology, not a second independent signal. | Fix indices from $\lambda_1=0$; use $\lambda_2,\lambda_3,\dim\ker(L)$ as a non-redundant signature, not $\lambda_2-\lambda_1$. |
| E4 | audit1.md: spectral signature uniquely identifies chokepoint/corner/zone | A graph's spectrum is not injective with respect to geometry (co-spectral non-isomorphic graphs exist; long chains without narrowing also decrease $\lambda_2$). | Spectral signature as an auxiliary *feature* concatenated to explicit geometric characteristics (minimum width, number of exits — §5.4), never as a unique identifier or a substitute for collision checking. |
| E5 | audit2.md: capture-approach law with sign `v=k(0.40−d)` (advance when `d` is large) | If the goal is to reduce `d` toward `d*` and `d>d*`, the correct proportional law for $\dot e=-k e$ with $e=d-d^*$ is $v\propto k(d-d^*)$ (advance if `d` is large relative to `d*`), not $k(d_0-d)$, which reverses the direction of advance for `d>d0`. | $v_{nom}=\mathrm{clip}(k_d(d-d_*),0,v_{approach})\max(0,\cos\beta)$, with bearing-based orientation. Verified: for a static rival and $\beta=0$, $\dot e=-k_d e$ gives correct exponential convergence. |
| E6 | audit2.md: A* with negative cost or a "reward" edge near the opponent | Dijkstra/A* with negative-length edges does not guarantee termination or optimality (breaks the non-decreasing property of the priority queue); it can generate negative-cost cycles. | Edge costs strictly $\ge0$ (Eq. 19 below); the "press/intercept" objective is encoded in the tactical layer (utility $U_G,U_E$), not as a negative graph cost. |
| E7 | audit2.md: single interception formula $t=d/(v_G-v_E\cdot u)$ presented as a general solution | That formula is only valid for collinear motion at constant speed in free space; in a maze with walls and acceleration it is nothing more than a degenerate special case. | General quadratic equation (Eq. 22–23 below) solved as the minimum valid positive root, and for the maze it is replaced by topological route times $\hat T_G,\hat T_E$ (Eq. 24), not the collinear formula. |
| E8 | audit2.md: `mvsim.World(...).captured()` and similar methods cited as a real API | These do not exist in the public MVSim documentation consulted (mvsimulator.readthedocs.io); it is pseudocode, not a verified interface. | Define a custom adapter (`SimulationAdapter`) with its own methods (`reset/step/observe/ground_truth_for_referee`) implemented against the real APIs of the installed version. |

### 0.3 Own mathematical confirmations (derivations I verified from scratch)

- **Capture predicate (Eq. 15)** literally reproduces the rulebook, p. 6: *"расстояние между СК роботов менее 0.45 м... ось Х стража не более чем на 45 град... между роботами отсутствуют препятствия"* → $d<0.45\ \land\ e_G^Tr\ge d\cos(\pi/4)\ \land\ \mathrm{LOS}$. It matches exactly; there is no room for interpretation here.
- **Braking distance with latency (Eq. 27–28)**: derived from elementary kinematics — distance covered during latency $\tau$ plus braking distance at constant deceleration $b_{min}$: $v\tau+v^2/(2b_{min})\le d-m$. Solving the quadratic for $v$ gives exactly Eq. 28. It is the standard "stopping sight distance" formula from vehicle engineering, correctly adapted.
- **Quadratic interception equation (Eq. 22)**: from $\lVert r+v_Et\rVert=s_Gt$, squaring gives $(\lVert v_E\rVert^2-s_G^2)t^2+2r^Tv_Et+\lVert r\rVert^2=0$. Verified term by term.
- **Dirichlet posterior variance (Eq. 34)**: for $p_i\sim\mathrm{Dir}(\alpha)$, $\mathrm{Var}(p_i)=\alpha_i(\alpha_0-\alpha_i)/(\alpha_0^2(\alpha_0+1))$ is the standard formula; substituting $\bar p_i=\alpha_i/\alpha_0$ gives $\bar p_i(1-\bar p_i)/(\alpha_0+1)$, which is exactly Eq. 34. Confirmed against the same Dirichlet-Multinomial model that appears literally in §20.3 (lines 7816–7838 of the monograph), with the same $\alpha_0=0.10$.
- **Kemeny–Snell lumpability (§20.6, Theorem 20.6.1)**: it exists in the monograph and is correct *under its hypotheses* (equality of the transition kernel **and** the per-class expected reward/duration). audit1.md's error was omitting the duration condition: two transitions with the same destination probability but $\tau=1$s vs $\tau=10$s are not interchangeable under discounting. I verified this with the continuous-time discounted Bellman operator $K_o(s,C)=\mathbb E[e^{-\beta\tau}\mathbf 1\{s'\in C\}]$: if $\tau$ differs, $K_o$ differs even when $P(s'\in C)$ matches.

---

## 1. Rulebook → verifiable engineering requirements

Extracted and verified line by line against the PDF (p. = document page):

| ID | Normative fact (source) | Mandatory technical consequence |
|---|---|---|
| R1 | TurtleBot2 + Livox MID-360 (p. 4) | Differential kinematic model; single authorized sensor |
| R2 | Modules ≈1×1 m, maze (p. 4) | Do not assume a 1 m clear passage; measure real walls/posts |
| R3 | Maze configuration known on Day 1, fixed during competition (p. 4) | Reusable structural map **if** it is clarified that saving it is allowed; `online_only` mode as fallback |
| R4 | Obstacles in unknown but equal order for all teams; width/length ≥0.1 m, height ≥0.15 m (p. 4) | Minimum-block detection with real LiDAR, not simulated |
| R5 | Opaque, non-specular surfaces (p. 4) | LiDAR return is not guaranteed from every angle; validate with hardware |
| R6 | 2 symmetric 10-minute stages, roles swapped (p. 5) | The same binary must operate both roles without recompiling |
| R7 | 4-minute `freeze` inside the start zone before active (p. 5) | `FREEZE` state with motion authority inhibited at the supervisor level, not the tactics level |
| R8 | Preparation only via SSH from the organizer's PC (p. 5) | Startup with no mandatory GUI, no external network |
| R9 | Forbidden to hardcode in software any information tied to particulars unknown beforehand (p. 5) | No hardcoded obstacle/opponent positions; reset of stage-specific memory pending clarification |
| R10 | Code editing only before crossing the line and with organizer agreement (p. 5) | Binary and config frozen by hash before each stage |
| R11 | No extra sensors/network/compute; no altering appearance (p. 5) | 100% onboard execution, no offload |
| R12 | Stage ends by arrival, capture, or timeout; an uncaptured explorer wins the stage (p. 5) | Explicit terminal with 3 outcomes; do not optimize only for "arrive fast" |
| R13 | Points announced on Day 1; wins by sum of stages (p. 5–6) | `score_profile` decoupled and versionable, not hardcoded |
| R14 | Arrival = contour contact with the zone (p. 6) | Footprint-based goal geometry, not center/waypoint |
| R15 | Capture = distance between frames <0.45 m ∧ guardian's X axis ≤45° from the line ∧ no obstacle between robots (p. 6) | Exact predicate Eq. (15); see §0.3 |
| R16 | Terminal indication recommended (p. 6) | SSH-readable telemetry with no visualizer |
| R17 | No penalty for collision with static obstacles; restart if a critical shift occurs (p. 6) | Avoiding collision remains a reliability requirement, not a scoring one |
| R18 | Access blocked 30 min before; robots powered off and handed over (p. 3) | Reproducible cold-start mandatory |
| R19 | Full restart once, by agreement of both captains or the organizer; stopping a stage cedes the maximum score to the opponent (p. 5) | "Restart" is not a normal autonomous recovery mechanism |

**Open questions that affect design and do not block development** (moving forward with the provisional decision indicated, to be confirmed before freezing the competition build):

1. Is it allowed to keep the structural map built during preparation? → Implement **two profiles** (`approved_map`, `online_only`); decide which to use on competition day.
2. Does memory/map persist between stage 1 and stage 2 of the same match? → By default, **reset** opponent track and detection map per stage.
3. How is the target zone and initial pose communicated (coordinates, visual marker)? → With no official input, no claim of "autonomous arrival at an identified zone" is made; the metadata-entry mechanism still needs to be defined.
4. Do the 4 minutes of preparation count within the 10, or are they additional? → Parameterize `stage_total=600s, freeze=240s, active=360s` as a literal reading of the text, adjustable via configuration.

---

## 2. Why this architecture and not another (engineering contrast against families other teams are likely to use)

| Solution family | What it gains | Why it loses on this specific problem | Verdict |
|---|---|---|---|
| **Standard Nav2 (DWB/TEB) + reactive pursuit** | Fast to set up, many teams know it | No concept of "role", "intentional opponent", or "capture by relative geometry"; it is point-to-point navigation, not a two-player game | Insufficient alone; *regulated pursuit* is reused as the control layer, not as the tactical brain |
| **Mapless reactive pursuit** (chase last seen position) | Trivial to implement | Loses the rival around a corner, does not anticipate portals, does not distinguish "capturing" from "approaching" | Serves as a low-priority *fallback*, not the main policy |
| **End-to-end deep RL** | Expressive, fashionable | No real-opponent data, no competitive GPU, no time to validate sim-to-real for a black-box policy; high risk of failure on the real robot on competition day | Explicitly excluded for this delivery (matches the team's decision to remove NNs) |
| **Heavy MPC/MPPI in the control loop** | Good trajectory quality with rich costs | CPU with no GPU (Intel NUC): $K$ stochastic rollouts at 20 Hz compete for cycles with the opponent EKF and the LiDAR detector; no measured budget guarantees it | *Future* candidate, not discarded by dogma but by an unmeasured budget; tested after the MVP if time allows |
| **Optimal transport (Wasserstein-2) / 32D Port-Hamiltonian as the core** | Elegant for swarms or manipulators with null spaces | The maze's free space is strongly non-convex (OT geodesics would cross walls); the robot is a single rigid body in $SE(2)$, not a density or an articulated arm | Discarded with justification (§0.1), not reusable without an iterative projection that cancels its analytic advantage |
| **Pure learned ASG/SMDP (no base FSM)** | Discovers unanticipated tactics | No coverage guarantees in the available time; risk of state aliasing (partial observability + opponent) if deployed without a base policy | Implemented as an **optional improvement** on top of a deterministic FSM that can actually be certified before competing |
| **This architecture: option FSM/ASG + topological A* + regulated control + independent physical supervisor** | Debuggable, runs on CPU, each layer is tested separately, degrades gracefully if a module fails | Requires discipline in inter-layer contracts; not the most "sophisticated" on paper | **Chosen**: maximizes `validated benefit / implementation-and-test cost` with the available time |

The decision criterion is not "what is more elegant in theory" but what can be **certified with unit and integration tests before the access lockout (R18)**. A controller with beautiful math that is never validated on hardware is worse than a simple FSM that actually was tested.

---

## 3. Layer and block architecture

```
LiDAR + IMU + wheel odom
        │
        ▼
┌───────────────────────┐      ┌────────────────────────────┐
│ LAYER 0 · ACQUISITION  │      │ LAYER 5 · STAGE MANAGER     │
│ deskew, TF, own state  │◄────►│ freeze / active / timeout   │
│ (EgoState)             │      │ score_profile, match_state  │
└──────────┬─────────────┘      └───────────────┬────────────┘
           │                                     │ authorizes
           ▼                                     ▼
┌───────────────────────┐  fast     ┌──────────────────────────┐
│ LAYER 1 · WORLD        │─────────►│ LAYER 6 · SUPERVISOR      │
│ structural map,        │ local    │ sole writer of             │
│ collision map,         │ obstacles│ cmd_vel_final              │
│ topological graph      │          │ (admission, braking,      │
└──────────┬─────────────┘          │  expiration, watchdog)    │
           │                        └─────────────▲─────────────┘
           ▼                                       │ nominal
┌───────────────────────┐                          │
│ LAYER 2 · OPPONENT     │                          │
│ PERCEPTION             │                          │
│ segmentation,          │                          │
│ GPIS registration,     │                          │
│ EKF, belief            │                          │
│ (OpponentTrack)        │                          │
└──────────┬─────────────┘                          │
           ▼                                        │
┌───────────────────────┐                           │
│ LAYER 3 · TACTICS      │                           │
│ per-role option        │                           │
│ FSM/ASG (OptionGoal)   │                           │
└──────────┬─────────────┘                           │
           ▼                                         │
┌───────────────────────┐                            │
│ LAYER 4 · PLANNING &   │                            │
│ LOCAL CONTROL          │                            │
│ topological A*, DWA/RPP│────────────────────────────┘
│ (MotionCandidate)      │
└─────────────────────────┘

         ┌──────────────────────────────┐
         │ LAYER 7 · OFFLINE LEARNING    │  (outside the critical loop,
         │ Dirichlet, per-option Q,      │   never writes to the driver
         │ bounded evolution, shadow-mode│   directly)
         └──────────────────────────────┘
```

### 3.1 Authority table (who can do what, and what is explicitly forbidden)

| Block (layer) | Responsibility | Explicitly NOT authorized to |
|---|---|---|
| 0. Acquisition/estimation | Time, TF, `EgoState`, sensor health | Declare a pose valid just because a TF exists |
| 1. World | Occupancy, collision map, topology | Erase obstacles "to make capture easier" |
| 2. Opponent perception | `OpponentTrack`, belief under occlusion | Publish opponent yaw when it is not observable (§6.3) |
| 3. Tactics (options) | Propose an effect: intercept, break LOS, advance | Publish directly to the base driver |
| 4. Planning/control | Feasible path + `MotionCandidate` | Declare a path safe just because A* returned it |
| 5. Stage manager | `freeze`/`active`/timeout, `score_profile` | Infer stage start from the first point cloud |
| 6. Supervisor | Final admission, braking, watchdog — **sole producer of `cmd_vel_final`** | Promise invulnerability against an arbitrary opponent |
| 7. Offline learning | Tune parameters/ASG *offline* | Change physical radius, minimum braking, or time sources; write during competition |

---

## 4. Data structures and inter-layer contracts

Every critical inter-layer message carries, at minimum: `seq`, `observation_stamp`, `publication_stamp`, `valid_until`, `frame_id`, `source_id`, `map_version`/`localization_epoch`, `status`, and explicit units. An isolated `confidence: float` field is **not** an acceptable contract — without a measurement timestamp, velocity, and yaw validity, an interception cannot be certified.

| Structure | Essential fields | Producer | Consumer |
|---|---|---|---|
| `EgoState` | SE(2) pose, twist, covariance, epoch, health, slip flag | Layer 0 | All |
| `WorldSnapshot` | grid version, transform snapshot, portals, obstacles, zones, provenance | Layer 1 | Layer 3, 4 |
| `OpponentTrack` | `id`, $(x,y,v_x,v_y)$, 4×4 covariance matrix, `yaw_valid: bool`, `t_last_meas`, FSM state (`SEARCHING/TRACKED/COASTING/LOST`) | Layer 2 | Layer 3, 6 |
| `OptionGoal` | `option_id`, role, sub-goal, optional heading, tolerances, `deadline`, preconditions | Layer 3 | Layer 4 |
| `OptionFeedback` | `id`, progress, status/cause, proposed vs. applied command | Layer 4 | Layer 3, 7 |
| `MotionCandidate` | $v,\omega$, state origin time, horizon, max age, mode | Layer 4 | Layer 6 |
| `SafetyStatus` | veto reason, free distance, measured latency, admissible velocity, health | Layer 6 | Layer 3, log |
| `MatchState` | stage, role, `freeze`, `deadline`, `score_profile`, motion authorization | Layer 5 | Layer 3, 6 |

**Golden concurrency rule:** one producer per physical channel (e.g., if odometry fusion is used, the driver does not simultaneously publish the same `odom→base_link`); immutable snapshots read by tactics; any change of `map_version`/`localization_epoch` invalidates paths and in-flight options — an old path is never "patched" with a new transform.

---

## 5. End-to-end data flow (a typical cycle)

1. **Acquisition (Layer 0):** LiDAR + IMU + wheel odometry → own-motion *deskew* (equation in §6.1) → `EgoState` published with epoch and health.
2. **World (Layer 1), in parallel, two paths with different priority:**
   - *Fast obstacle path*: subtracts against the structural map with an uncertainty-dependent threshold → `WorldSnapshot.local_obstacles`, goes directly to the Supervisor (Layer 6) without waiting for identification.
   - *Topology path*: only when the map changes, recomputes the portal/width/exit graph.
3. **Opponent perception (Layer 2):** from candidates not vetoed as background, DBSCAN → shape/volume filter → registration against the opponent's GPIS prior → constant-velocity EKF with Mahalanobis *gating* → `OpponentTrack` (with `yaw_valid=false` if the shape is axially symmetric). See §6.3-bis for occlusion handling.
4. **Tactics (Layer 3):** at 2–5 Hz or on a critical event, generates candidate options per role (Guardian: `SEARCH_PORTAL, INTERCEPT_PORTAL, PRESSURE_ROUTE, APPROACH_CAPTURE, RECOVER_VIEW, FALLBACK_DEFEND_BASE`; Explorer: `ADVANCE_BASE, BREAK_LOS, TAKE_ALTERNATE_PORTAL, KEEP_ESCAPE_ROUTE, OBSERVE_SAFE`), evaluates feasibility + utility ($U_G$/$U_E$, Appendix A), applies hysteresis, and emits `OptionGoal`.
5. **Planning + local control (Layer 4):** A* over the topological graph with costs $\ge0$ (Eq. 19) yields a feasible path; the regulated pursuit controller (Eq. 25) or the DWA-style arc evaluator (Eq. 26) turns it into a `MotionCandidate` at 20 Hz, already respecting the nominal braking limit.
6. **Supervisor (Layer 6), 30–50 Hz, independent of the latency of the layers above:** if `freeze`/`match.finished` → stop; if data is stale/inconsistent → bounded stop; otherwise, applies the real braking envelope (Eq. 27–28) to the `MotionCandidate` and publishes `cmd_vel_final` with a short expiration. **This is the only path that reaches the base driver.**
7. **Stage manager (Layer 5):** produces `CAPTURE_ESTIMATE`, `REACHED_BASE`, `TIMEOUT`, `ABORT` events per the §6 predicate (official rule), independent of the organizer's real referee — the system **estimates**, it does not certify.
8. **Offline learning (Layer 7):** consumes logs from complete matches (not individual frames), never writes directly to Layer 3/4/6 during competition; only produces frozen artifacts (`artifacts/policies/`) that are loaded as configuration before `freeze`.

---

## 6. Core mathematics per block (with the derivation that validates each formula)

### 6.1 Kinematics and acquisition

Nominal differential model:
$$\dot x=v\cos\theta,\quad \dot y=v\sin\theta,\quad \dot\theta=\omega,\qquad v=\tfrac{r_w}{2}(\dot\phi_R+\dot\phi_L),\ \omega=\tfrac{r_w}{b}(\dot\phi_R-\dot\phi_L).$$

Exact constant-velocity integration over $\Delta t$ (avoids Euler discretization, which accumulates curvature error):
$$
q_{k+1}=\begin{cases}
\left(x+\tfrac v\omega[\sin(\theta+\omega\Delta t)-\sin\theta],\ y-\tfrac v\omega[\cos(\theta+\omega\Delta t)-\cos\theta],\ \theta+\omega\Delta t\right), & |\omega|>\epsilon\\
(x+v\Delta t\cos\theta,\ y+v\Delta t\sin\theta,\ \theta), & |\omega|\le\epsilon
\end{cases}
$$

Point-cloud *deskew* (own-motion correction during the sweep, essential because the MID-360 is not an instantaneous scan):
$${}^{L_r}\bar p_i=[{}^OT_B(t_r){}^BT_L]^{-1}\,{}^OT_B(t_i){}^BT_L\,{}^{L_i}\bar p_i.$$
Sanity check: at rest this is the identity; with a fixed wall and the robot turning, corrected points must coincide again with the same wall. **Important limit I confirmed**: this *deskew* corrects the robot's own motion, **not** the opponent's — if the rival moves fast during the accumulation window, a residual *smear* bounded by $v_{E,max}\cdot T$ persists, so long online accumulation windows must not be used (unlike the offline prior of the robot's own shape in the qualification stage).

### 6.2 Maps and topology

Log-odds occupancy with clipping:
$$l_{c,k}=\mathrm{clip}\big(l_{c,k-1}+\log\tfrac{p(c\mid z_k)}{1-p(c\mid z_k)}-l_{c,0},\ l_{min},l_{max}\big).$$

Planning-radius inflation, summing each error source **exactly once**:
$$r_{plan}=r+\epsilon_p+\epsilon_m+\epsilon_t+\epsilon_{grid}.$$

Topological graph: skeleton/medial axis of the thresholded free space → nodes at intersections/width changes/target zones → edges valid only if the full footprint can traverse them (a swept check, not just *k*-NN between centers, which could cross a wall) → components/articulation points/bridges via DFS in $O(|V|+|E|)$. A "geometric chokepoint" (narrow width) and a "topological chokepoint" (vulnerable connectivity, graph bridge) **are not the same thing**: a narrow corridor may have an alternate route; a wide portal may be the only link. Both are stored as separate *features*.

### 6.3 Opponent perception (reuses and adapts the qualification-stage pipeline)

From `DESIGN_OVERVIEW.md`: the qualification system already solves, with a **fixed** sensor, the shape/pose separation via an implicit Hermite-GPIS-W prior (Gaussian Process Implicit Surface with a compact-support Wendland kernel) built offline, and online 3-DOF Gauss-Newton registration ($x,y,\psi$) against that prior, followed by EKF+FSM (`SEARCHING/ACTIVE_TRACKING/COASTING`). **This is reused as the base of the opponent tracker**, with mandatory changes:

- The sensor is now **mobile**: the fixed-background map (OBB shells) cannot veto in sensor coordinates without composing with `EgoState`; and the fixed ground alignment must be recomputed from extrinsics and own attitude at every instant, not as a universal fixed transform.
- Registration cost with dimensionally consistent uncertainty:
$$E(\xi)=\sum_j\rho\!\left(\frac{f(z_j)}{\sigma_{f,j}}\right),\qquad \sigma_{f,j}^2=\sigma_{GP,j}^2+\nabla f(z_j)^T\Sigma_{z,j}\nabla f(z_j)+\sigma_{model}^2,$$
  where $\Sigma_{z,j}\nabla f$ transports the metric noise into the implicit-field units through the gradient — directly adding variances in different units without this transport is a dimensional error.
- **Yaw is not always observable**: for an approximately axially symmetric shape, the yaw column of the registration Jacobian is nearly zero and $J^TWJ$ is ill-conditioned. The system **must** publish `yaw_valid=false` instead of an artificially small covariance. Recommended minimal opponent state: $s_E=[x_E,y_E,v_{x,E},v_{y,E}]^T$ (position + velocity, orientation not mandatory).
- Constant-velocity filter with continuous white-acceleration noise:
$$F=\begin{pmatrix}I_2&\Delta t\,I_2\\0&I_2\end{pmatrix},\quad Q=q_a\begin{pmatrix}\Delta t^3I_2/3&\Delta t^2I_2/2\\\Delta t^2I_2/2&\Delta t\,I_2\end{pmatrix},$$
  with the standard update $\nu=z-H\hat s^-$, $S=HP^-H^T+R$, $K=P^-H^TS^{-1}$, and **gating** $\nu^TS^{-1}\nu\le\chi^2_{2,1-\alpha}$ applied to **every** measurement (an excellent registration score does not exempt a measurement from the kinematic check).

### 6.3-bis Occlusion handling and anti-flanking behavior (topological belief tracker)

**Why this matters:** losing sight of the opponent around a corner is the classic ambush/flanking scenario. If the robot simply "forgets" the opponent once LOS is lost, or predicts it in a straight line through the wall, the opponent can exploit that blind spot to circle through an alternate corridor and win the stage. This subsection replaces the single-line occlusion-belief sketch of the original design (the pairwise Bayes-filter formula below §6.3) with a fully specified three-phase mechanism, and defines the corresponding anti-flanking tactics referenced in §6.5 and §5.

**Two memory windows must not be confused.** (1) *LiDAR accumulation memory* (short, physical, $\sim0.1$–$0.2$ s): as already noted in §6.1, this window must stay short, or the opponent's shape smears into an elongated blob and GPIS-W registration diverges. (2) *Tactical belief memory* (long, probabilistic, $\sim5$–$30$ s): this lives in the **topological graph**, not in the point-cloud buffer, and is **not** cleared when LOS is lost. Conflating the two — e.g., trying to extend the short LiDAR window to "remember longer" — is the wrong mechanism and was explicitly avoided.

**Phase 1 — Confined coasting ($0<\Delta t\le0.5$ s).** At the instant LOS is lost ($t_{loss}$), the tracker transitions `TRACKED → COASTING`. Constant-velocity extrapolation in the free Cartesian plane ($p(t)=p_0+v\Delta t$) is **forbidden**, since it can place the estimate inside a wall. Instead, the EKF's pre-loss velocity estimate $\vec v_E$ is projected onto the centerline of the corridor the opponent just entered; for the first $0.5$ s the filter has high confidence the opponent is within the first $\sim30$ cm of that corridor.

**Phase 2 — Diffusion over the geodesic reachable set ($0.5\text{ s}<\Delta t\le5.0\text{ s}$).** The opponent is physically bounded by $v_{E,max}\approx0.65$ m/s and $a_{E,max}\approx0.8$ m/s². The set of positions where the opponent can physically be is the **geodesic reachable set**:
$$\mathcal R_E(\Delta t)=\Big\{x\in\mathcal W_{free}\ \Big|\ D_{geo}(x,p_{loss})\le v_{E,max}\Delta t+\tfrac12 a_{E,max}\Delta t^2\Big\},$$
where $D_{geo}$ is the shortest-path (geodesic) distance in the free-space graph, not the Euclidean distance — this is what keeps belief mass confined to actual corridors instead of leaking through walls, and is the direct generalization of the pairwise belief-propagation sketch $b^-_{k+1}(j)=\sum_iP_{ij}b_k(i)$ already present in §6.3, now made concrete over the topological graph's portal nodes. At a branch point, probability mass splits across reachable branches (e.g., a 50/50 split at a T-junction if both arms are equally consistent with $\mathcal R_E(\Delta t)$ and prior behavior).

**Phase 3 — Bayesian update from negative information.** Not seeing the opponent where it should be visible is as informative as seeing it. For a region $j$ swept by the current LiDAR field of view, with a null observation $z=\varnothing$:
$$b^+(j)=\frac{P(z=\varnothing\mid x\in j)\,b^-(j)}{\sum_kP(z=\varnothing\mid x\in k)\,b^-(k)}.$$
If corridor $j$ is inside the LiDAR's field of view and is empty, $P(z=\varnothing\mid x\in j)\approx0.05\Rightarrow b^+(j)\to0$; if corridor $k$ is occluded by a wall, $P(z=\varnothing\mid x\in k)\approx1.00\Rightarrow b^+(k)\to1.00$ after renormalization. This is the same "no-detection likelihood $\approx1-P_D(j)$" principle already stated in §6.3 for a single region, generalized here to redistribute mass across the whole reachable set rather than leaving it static. After $\Delta t>15$ s with no re-detection, the state degrades to `LOST` and belief mass is treated as uniform over $\mathcal R_E$ for planning purposes (no false confidence in a stale estimate).

State machine summary: `TRACKED → COASTING (≤0.5 s, corridor-confined) → OCCLUDED_BELIEF (diffusion + negative information over the topological graph) → LOST (>15 s)`, with `TRACKED` re-entered instantly on any gated re-detection (§6.3). Implementation lives in `hsl_core/perception/topological_belief.py` (see repository structure document, §1) as a pure-Python module consuming the topological graph from §6.2 and the `visible_polygons` swept region from the current LiDAR frame; it has no ROS dependency, consistent with the core/adapter separation of the whole codebase.

### 6.4 Capture and arrival geometry (official predicates, not approximations)

$$C(q_G,q_E,\mathcal O)=[d<0.45]\ \land\ [e_G^Tr\ge d\cos(\pi/4)]\ \land\ [\mathrm{LOS}(p_G,p_E,\mathcal O)],\qquad r=p_E-p_G,\ d=\lVert r\rVert.$$

Conditional certification under bounded error $\epsilon_r$ (radial) and $\epsilon_\theta$ (own angular):
$$\hat d+\epsilon_r<0.45,\qquad |\hat\beta|+\epsilon_\theta+\arcsin(\epsilon_r/\hat d)\le\pi/4.$$

**Geometric compatibility between "no collision" and "capture certifiable"**: with inflated radii $r_G,r_E$, physical margin $m$, and error $\epsilon_r$, a valid operating point $d_*$ exists only if
$$r_G+r_E+m+\epsilon_r<d_*<0.45-\epsilon_r \iff r_G+r_E+m+2\epsilon_r<0.45.$$
This is a **design** constraint, not merely a software one: if the inflated footprint is too large, capture becomes geometrically impossible to certify without colliding, and no algorithm fixes that — the real footprint must be measured and the margin calibrated, not "shrinking the opponent out of the map" to force the event.

Arrival (R14): event on the **first contour contact** from outside the zone (full footprint, not the center reaching a waypoint); the estimated simulator/referee must check the swept path between simulation steps, not just the final state, so a brief contact is not missed.

### 6.5 Global planning and per-role tactics

Non-negative edge cost (mandatory, see error E6):
$$c_{ij}=\ell_{ij}\big[1+\lambda_o\phi_o(j)+\lambda_r\phi_r(j)\big],\quad \lambda_o,\lambda_r\ge0.$$
With $c_{ij}\ge\ell_{ij}$ and the triangle inequality, $h(i)=\lVert p_i-p_g\rVert$ is an admissible and consistent heuristic for A* — this is what guarantees optimality *on that graph and those costs*, not in the full game.

Interception in free space (general form, not the degenerate collinear case of E7):
$$(\lVert v_E\rVert^2-s_G^2)t^2+2r^Tv_Et+\lVert r\rVert^2=0,$$
minimum positive real root; for the maze this is replaced by topological route times $\hat T_G(c)=T_{route,G}(c)+T_{alignment}(c)$, $\hat T_E^{(h)}(c)=T_{route,E}^{(h)}(c)$ evaluated per portal hypothesis $c$ and opponent-behavior hypothesis $h$.

Option-selection utility (deterministic initial policy, no training):
$$U_E(o)=-w_c\hat P_{cap}(o)+w_g\Delta D_{base}(o)/D_*+w_l\hat P_{breakLOS}(o)+w_x\min(n_{exits}(o),3)/3-w_t\tau_o/T_*,$$
$$U_G(o)=w_c\hat P_{cap}(o)+w_i\sum_hb_h\,\mathrm{clip}\!\Big(\tfrac{\hat T_E^{(h)}(c_o)-\hat T_G(c_o)}{T_*},-1,1\Big)+w_v\hat P_{observe}(o)-w_t\tau_o/T_*.$$
$w_c$ dominates by design: a small route saving must never outweigh a large threat of capturing/being captured. If the probabilistic estimate is not yet calibrated, an ordinal risk *ranking* is used instead of presenting the numbers as reliable probabilities.

**Anti-flanking tactics for the Guardian role** (using the belief mechanism of §6.3-bis): when the Explorer turns a corner, the Guardian is forbidden to blindly chase along the same line. Three options are evaluated against the topological graph:

- **Tactic A — `RECOVER_VIEW` (tangential corner clearing / corner peek):** if the Guardian is close ($<1.5$ m), the planner does not route to the center of the opponent's corridor; it routes to the corridor's outer vertex, orienting the LiDAR axis to maximize the visible-area derivative $\mathrm d\mathcal A_{vis}/\mathrm dt$, exposing the field of view at the shortest possible distance before the opponent completes its acceleration.
- **Tactic B — `INTERCEPT_PORTAL` (bottleneck interception, anti-flanking):** if the corner leads into a longer loop that could bring the opponent back to the Guardian's base from behind, the Guardian does **not** chase through the corner. It consults the global topological graph; if the alternate loop must converge on a chokepoint/portal before the base, the Guardian takes the shorter geodesic route to that portal instead of the opponent's route. Since $D_G<D_E$ at equal maximum speed, $T_G=D_G/v_{max}<T_E=D_E/v_{max}$: the Guardian reaches the portal first, stops perpendicular to it, and waits facing the corridor — turning the flanking attempt into an automatic capture opportunity.
- **Tactic C — `FALLBACK_DEFEND_BASE`:** if uncertainty grows because the maze has multiple symmetric cycles and $\mathcal R_E(\Delta t)$ touches two possible approaches to the base, the Guardian cancels the chase and positions at its own start-zone threshold at $\approx0.50$ m with a full-sweep sensor. Since the Explorer must physically touch the base contour to win (R14), it is forced to enter the Guardian's final visible zone regardless of which corridor it used, neutralizing any advantage gained by cornering.

These three tactics are added to the Guardian option set already listed in §5 (`SEARCH_PORTAL, INTERCEPT_PORTAL, PRESSURE_ROUTE, APPROACH_CAPTURE, RECOVER_VIEW, FALLBACK_DEFEND_BASE`); no new layer or contract is required — `INTERCEPT_PORTAL` and `RECOVER_VIEW` were already present, `FALLBACK_DEFEND_BASE` is the one addition, selected through the same $U_G$ utility with the topological-belief threat estimate feeding $\hat T_E^{(h)}$.

### 6.6 Local control and physical safety (hard authority boundary)

Regulated pursuit: $\kappa=2y_L/L^2$, $\omega=v\kappa$, $v\le\min(v_{max},\ \omega_{max}/(|\kappa|+\epsilon),\ v_{brake})$.

Window of reachable arcs over $\Delta t$ (for DWA-style evaluation when regulated pursuit is not enough, e.g. near the opponent):
$$\mathcal U_k=\{(v,\omega): |v-v_k|\le a_{max}\Delta t,\ |\omega-\omega_k|\le\alpha_{max}\Delta t\}\cap\mathcal U_{hardware}.$$

Braking distance with total latency $\tau$ (sensor+queue+processing+transport+mechanical response) and **measured** minimum deceleration $b_{min}$:
$$v\tau+\frac{v^2}{2b_{min}}\le d-m \;\Rightarrow\; v_{brake}=\max\Big\{0,\ -b_{min}\tau+\sqrt{b_{min}^2\tau^2+2b_{min}\max(0,d-m)}\Big\}.$$
Against an opponent that may keep advancing during the robot's own braking, a more conservative bound adds $v_E(\tau+v/b_{min})$: **stopping does not universally guarantee no contact** if the other body keeps moving — this is reported as risk reduction within a validated envelope, not an absolute guarantee.

Distance CBF to a fixed obstacle, **corrected sign** (E1): $h(p)=\lVert p-o\rVert^2-R^2$,
$$\dot h=2(p-o)^Te(\theta)v,\qquad 2(p-o)^Te(\theta)v+\gamma h\ge0.$$
Under $h(0)\ge0$ and the continuous inequality, $h(t)\ge e^{-\gamma t}h(0)\ge0$: this is the **conditional** proof of safe-set invariance; it does not by itself cover discrete commands, saturation, moving obstacles, or an infeasible QP. This is why the CBF-QP is left as a post-MVP extension and **not** a requirement for the first delivery — the primary safety path is the explicit braking envelope above, simpler to certify with hardware tests.

**Supervisor evaluation order** (sole writer of `cmd_vel_final`): physical/health stop → `freeze`/match state → temporal validity of sensors/estimation → hardware limits → swept collision + braking → final command. The *watchdog* covers both node death and full-process death (driver timeout + independent supervisor process).

---

## 7. Statistical/evolutionary learning layer (optional, offline, no NN)

This instantiates the correct ideas from audit1.md (Dirichlet-Multinomial, pruning, state fusion, bounded evolution) already corrected in §0.2–0.3:

- **Transition model**: $p_{i,:}^o\mid\mathcal D\sim\mathrm{Dir}(\alpha_1+n_1,\dots,\alpha_K+n_K)$, mean $\mathbb E[p_j]=(\alpha_j+n_j)/A$, variance $\bar p_j(1-\bar p_j)/(A+1)$. Learning $p$ **does not** by itself optimize a policy; a value/reward over durative options is needed:
$$Q(s,o)\leftarrow Q(s,o)+\alpha_{lr}\Big[R_o+\gamma^n\max_{o'}Q(s',o')-Q(s,o)\Big],\quad R_o=\sum_{j=0}^{n-1}\gamma^j r_{t+j}.$$
- **Count every outcome**, including aborts and supervisor interventions — reinforcing only successes is survivorship bias.
- **State fusion (lumpability)** only if, for every relevant option, **both** the aggregated transition kernel **and** the per-class expected reward/duration coincide (Theorem 20.6.1, not just equality of $P$ — see the correction in §0.3).
- **Spectral signatures**: an experiment *after* the MVP, never a substitute for collision checking or a unique zone identifier (E3, E4). Concatenated as an auxiliary *feature* to already-explicit geometric characteristics of the topological graph, and validated on held-out data; removed if it does not help.
- **Bounded evolution**: tune 5–10 tactical parameters (interception horizon, hysteresis, dead-end penalty, etc.), never footprint, minimum braking, or physical margins. Evaluate against a fixed bank of opponents (not just the latest champion, to avoid rock-paper-scissors strategic cycles).
- **Deployment rule**: during competition, ASG nodes and policy realizers are **frozen**; learning runs offline and is only promoted to competition if it beats the base FSM on held-out tests with unseen maps/opponents (`policy=shadow` before `policy=learned`).

---

## 8. Simulation and sim-to-real validity

Two levels of fidelity, deliberately **not** just one:

| Level | What it's for | What it does NOT validate |
|---|---|---|
| Lightweight kinematic tactical simulator (NumPy) | Thousands of episodes for rules/options/learning | Real Livox returns, real GPIS, friction |
| MVSim + ROS + sensors | Interfaces, both vehicles, dynamics, collisions | Exact equivalence with the MID-360's non-repetitive scan pattern |
| Replay of real bags + robot | Perception, synchronization, physical behavior | Universal generalization |

A custom adapter (`SimulationAdapter.reset/step/observe/ground_truth_for_referee`) translates to the real APIs of the installed MVSim version — methods such as `.captured()` are not assumed since they are undocumented (E8). The policy/estimator only ever see `observe()`; ground truth is reserved for the referee and metrics — **forbidden** for tactics to subscribe to the opponent's true simulated pose without going through the detection/occlusion model, or training will learn an advantage that does not exist in real competition.

Minimum scenarios to cover: straight corridor, T-junction, crossroads, cycle, dead end, narrow corridor, two routes to base, minimum obstacle (0.1×0.1×0.15 m), thin wall, posts, obstacles adjacent to the robot, base near a corner — varying start pose, role, noise, frame drops, and time offset. If the policy is trained with perfect detection during occlusion, it will learn to exploit an advantage it will not have on competition day.

---

## 9. Hardware constraints and computational budget

All values are **initial targets to be measured**, not certified performance:

| Function | Candidate frequency | Degradation if not met |
|---|---|---|
| Own state | ≥30–50 Hz ideal | Stop if there is no admissible continuity/age |
| Opponent detector | 5–10 Hz | Process only the latest frame, cap candidates |
| Fast obstacles | Every valid cloud | Brake when the observation expires |
| Local control | 20 Hz | Reduce horizon, always keeping the braking check |
| Supervisor | 30–50 Hz + base timeout | Independent of tactics/learning load |
| Global path (A*) | 1–2 Hz or event-driven | Keep the previous path if still valid |
| Tactics | 2–5 Hz + critical events | Fall back to the base policy on compute timeout |
| Learning | Offline/*shadow* | **Disabled** in the competition binary |

Hard constraints from R11 (no extra compute/sensor, only what is provided — likely a NUC or equivalent with no dedicated GPU): this rules out, by budget (not theoretical impossibility), MPPI with hundreds of high-frequency rollouts and any neural-network inference in the critical loop. Measured with end-to-end `p50/p95/p99` latency, not just the average — a WCET is not "the maximum observed in a short test." Avoid BLAS/OpenMP thread over-subscription relative to the NUC's real core count. Compute priority: control/supervisor above logging and detailed recording.

Recommended initial test speed 0.15–0.25 m/s, increasing only after validating detection and braking on real hardware — **do not** start from a historical spec-sheet figure (e.g. "0.7 m/s") as the competition speed without having measured $b_{min}$ on real battery, load, and floor.

---

## 10. Acceptance-gate plan (mandatory implementation order)

| Gate | Content | Exit criterion | If it fails |
|---|---|---|---|
| G0 | Inventory, topics, TF, time, command, shutdown | Reproducible SSH start/stop | Autonomous navigation is not started |
| G1 | Footprint, minimum objects (0.1×0.1×0.15 m), braking (Eq. 27–28) | All test obstacles detected at a compatible distance | Reduce speed or fix the fast path |
| G2 | Odom/map, path, pursuit, added obstacle | Contact-free paths; recovery tested | Keep only base navigation |
| G3 | Opponent tracking (static/moving, corner) | Error and interval (18) compatible | Capture only in a low-speed regime |
| G4 | Both roles, FSM, belief under occlusion, referee | Complete match with correct `freeze`+timeout+roles | Trim to reliable options |
| G5 | Tactical improvement (ASG/parameters), `shadow mode` | Reproducible improvement without degrading navigation | The base FSM is deployed |
| G6 | Final release: hash, SSH startup, reset, logging | Cold start + two complete stages | *Rollback* to the previous version |

**Critical path: G0→G1→G2→G3/G4→G6. G5 never blocks delivery.** Suggested code implementation order: `rules.py` (capture/arrival predicates), `types.py` (contracts), driver adapter, supervisor, navigation, opponent tracker, options, evaluation. Reserve at least 25–30% of total time for integration and regressions; with less than 3 days available, spectral signatures, evolution, and ASG are excluded from scope.

---

## 11. Residual risks the architecture reduces but does not eliminate

| Risk | Mitigation in this architecture | What remains unguaranteed |
|---|---|---|
| Ambiguous localization in repeated symmetric corridors (R2) | Keep multiple hypotheses, reduce speed, seek a discriminating observation | No global guarantee without distinguishable evidence in the environment |
| A stationary opponent absorbed as part of the background | Separate map layers (structural vs. collision vs. detection), temporal memory | Identification may remain ambiguous |
| Stopping in front of an aggressive opponent | Short-horizon prediction, evasion available, kinematic limits | Stopping does not guarantee no contact if the opponent keeps advancing |
| Saturated CPU on the NUC | Latest-sample queues, per-cycle candidate limits, fixed priorities | Requires thermal/load testing on the final hardware |
| Geometrically infeasible capture due to a miscalibrated footprint | Explicit check of interval (18) before competing | If the interval is empty, the estimate must be improved or it must be accepted that capture cannot be certified |
| Overfitting of offline learning | Baseline FSM, held-out maps/opponents, automatic rollback | The sim-to-real gap never fully disappears |

**Delivery decision:** deploy the simplest version that passes G0–G4 and G6. Add the learning layer (Layer 7) only if it passes G5 on held-out tests.
