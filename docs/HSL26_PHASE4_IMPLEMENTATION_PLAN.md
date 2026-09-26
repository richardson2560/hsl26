# HSL26 — Phase 4 implementation plan and SIL environment

**Revision:** 1.1 · **Date:** 2026-09-25 · **Status:** P4.1–P4.5 SIL implemented; external acceptance blocked
**Authority:** `HSL26_FINAL_ARCHITECTURE.md`, `HSL26_TECHNICAL_SPECIFICATION.md`,
and `HSL26_PHASE4_OPPONENT_PERCEPTION.md`.

## 1. Scope and entry decision

Phase 4 is prepared as a software-in-the-loop (SIL) workstream because no
physical robot, complete verified opponent model, registered scans, or
hardware calibration is available in this checkout. The environment therefore
supports deterministic tracker, registration, belief and rejection tests only.
It cannot close the real-cloud GPIS subgate or G3 physical acceptance.

The P3 handoff is accepted as the input contract:

- `map_version`, `topology_version`, `localization_epoch`, observation stamp
  and sensor `frame_id` remain attached to every synthetic sequence;
- synthetic detections are labelled `SYNTHETIC_DETECTION`;
- referee truth is confined to scenario generation and assertions;
- topology is version-bound and may provide conservative route context, never
  an observation of the opponent.

The reference fixture is
[`opponent_perception.json`](../sim/kinematic/scenarios/opponent_perception.json).
Its dimensions and seed are development data, not competition geometry,
calibration, or physical evidence.

## 2. Environment and dependency decision

The existing `hsl_core` package is sufficient for the first P4 increments.
Python 3.10 or newer, NumPy, SciPy and pytest are required, matching the P2/P3
baselines. **No additional library is required** for the planned SIL
environment. In particular:

- Open3D is optional future tooling for registered point-cloud inspection; it
  is not required for pure-core or synthetic-detection tests.
- PyYAML, ROS 2, colcon and MVSim are not required for this stage.
- A future real-cloud adapter may add a dependency only after the sensor/model
  profile is approved and recorded in its own manifest.

Recommended Windows commands from the repository root:

```powershell
$env:PYTHONPATH = "hsl_core"
python -m pytest hsl_core\tests sim\kinematic -q
python -m compileall -q hsl_core\hsl_core sim
```

The P4 target command, once tests are added, is:

```powershell
$env:PYTHONPATH = "hsl_core"
python -m pytest hsl_core\tests\test_p4_*.py sim\kinematic\test_p4*.py -q
```

## 3. Work-package sequence

| Package | Implementation order | Required output | SIL exit evidence |
|---|---:|---|---|
| P4.1 | 1 | `implicit_surface.py`, `train_gpis_prior.py`, model arrays and manifest | deterministic kernel/derivative checks, conditioning, support rejection, disjoint validation views |
| P4.2 | 2 | `segmenter.py`, `registration.py`, validation tool | origin error by held-out view/range, support fraction, false-match and yaw observability tables |
| P4.3 | 3 | `ekf_opponent.py`, tracker node and codec | accepted/rejected gates, Joseph covariance PSD, independent measurement/prediction/publication stamps |
| P4.4 | 4 | `topological_belief.py`, sequence generator | stationary, occluded, long-edge, junction and reappearance traces with unknown mass |
| P4.5 | 5 | `runtime_codec.py`, `bounded_worker.py`, versioned ROS IDL and profile evaluator | round-trip tests, timeout/stale discard, synthetic bypass versus real-cloud path declaration |

Each package must preserve the existing pure-core/adapter split. No P4 module
may publish a motor command or replace the independent safety return path.

## 4. HGW theoretical constraints used by P4.1/P4.2

The authoritative mathematical source is
[`main_hgw.tex`](main_hgw.tex), especially Chapter 2 sections
“The Compactly Supported Wendland C2 Kernel”, “Regularity Order Required by
Differential Observations”, “Hermite Observation Model” and “Closed-Form
Posterior Inference”, plus Chapter 3 “Hermite Covariance Blocks and Symmetry”.
The following implementation rules are extracted from that source and from
the architecture contract:

1. Let the latent field be \(f:\mathbb{R}^3\rightarrow\mathbb{R}\), with
   value observations \(f(x_i)\), directional/normal derivative observations
   \(\nabla f(x_i)\cdot n_i\), and signed off-surface anchors. A value-only
   zero-mean fit is not sufficient because it can collapse to an uninformative
   zero field.
2. For a compactly supported kernel \(k\), the Hermite Gram matrix is formed
   by applying the observation operators to both arguments:
   \(K_{ab}=\mathcal{L}_a^x\mathcal{L}_b^{x'}k(x,x')\). The value-value,
   value-derivative and derivative-derivative blocks must use consistent
   units and orientation.
3. The posterior mean at a query \(z\) is
   \(m(z)=k_{\mathcal{L}}(z)^T(K+\Sigma)^{-1}y\), with regularization/noise
   \(\Sigma\). Predictive variance is retained where registration weighting
   needs it.
4. Compact support means unsupported queries have zero covariance with the
   active set; a zero prior mean outside support is **not** a surface match.
   Registration must report support fraction and reject weak support.
5. Derivative data require the kernel regularity declared by the selected
   Wendland family. Kernel version, support radius, normalization, coordinate
   units, transform \(T_B^M\), hashes and validation split are mandatory model
   metadata.

Online registration must optimize bounded planar translation and yaw in the
documented model/base frames. Residual weighting follows the architecture:
\[
 \sigma_{f,j}^{2}=\sigma_{GP,j}^{2}+
 \nabla f(z_j)^T\Sigma_{z,j}\nabla f(z_j)+\sigma_{model}^{2}.
\]
Axisymmetric or insufficient views publish `yaw_valid=false`; they must not
invent heading from velocity.

## 5. Truth isolation and scenario design

The fixture contains stationary, moving, occluded, blind-sector and
reappearance labels. The referee may use `opponent_truth` only to score a
test. The tracker input must contain observations, timestamps, covariance,
frame and source/fidelity metadata, but not the hidden pose or target ID.

Required negative cases are executable acceptance criteria:

- unsupported compact-kernel queries are rejected rather than treated as
  zero-level matches;
- a gated outlier does not update `last_measurement_s`;
- an axisymmetric view has invalid yaw but may retain valid position;
- negative observations update only freshly covered regions;
- an occlusion preserves unknown/reachable mass;
- bounded-speed propagation cannot reach a far edge before its travel time;
- stale map/topology/localization versions are discarded.

## 6. Evidence, limitations and G3 boundary

Each work package stores machine-readable results under
`artifacts/reports/phase4/` and model outputs under
`artifacts/models/opponent_gpis/`. Reports must include command, profile,
seed, source/model/topology hashes, expected result, observed result and
`PASS`, `FAIL`, `BLOCKED` or `NOT_RUN`.

The implemented software-in-the-loop work packages are test-verified, but this
does not establish:
complete robot geometry, \(T_B^M\) calibration, real-cloud origin accuracy,
Livox/MVSim fidelity, ROS transport behavior, CPU WCET, or physical safety.
G3 remains blocked until held-out registered/real views and the declared
sensor conditions are available.

## 7. P4.5 runtime profile boundary

The revision-2 `OpponentTrack` and `OpponentBelief` records carry
`ContractHeader` metadata, map/topology/localization versions, observation,
state and publication times, expiry, validity and sensor fidelity. Belief
cells preserve edge interval and mass; unknown mass is never dropped by the
codec. Codecs reject unknown schemas, malformed timestamps, invalid covariance,
noncanonical invalid yaw and non-normalized belief.

The single-flight worker exposes a caller-bounded deadline and discards work
after map/topology/localization or explicit generation changes. Python threads
cannot be forcibly stopped: it never spawns replacements while a timed-out
task is still running. Production hard CPU isolation therefore needs a
supervised process boundary; this SIL contract makes no WCET claim.

The profile evaluator accepts referee labels only in evaluation data and emits
descriptive error, miss, false-positive, covariance-coverage and latency
metrics. It does not import labels into the tracker and does not infer
acceptance thresholds. Real bag replay, ROS graph/QoS traces, MVSim sensor
fidelity and paired real-cloud/synthetic measurements stay `BLOCKED` until
their runtime and data prerequisites are supplied.
