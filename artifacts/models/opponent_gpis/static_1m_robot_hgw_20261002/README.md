# HGW reduced robot model candidates

This directory contains one KISS HGW candidate per sampled surface resolution:

| Directory | Surface samples | Hermite observations |
|---|---:|---:|
| `surface_points_100/` | 100 | 200 |
| `surface_points_150/` | 150 | 300 |
| `surface_points_200/` | 200 | 400 |
| `surface_points_250/` | 250 | 500 |

“Surface samples” counts distinct 3D return locations selected from the input
cloud. Each location contributes two scalar Hermite observations at that same
location: one value constraint (`f=0`) and one normal-derivative constraint
(`∇f·n=1`). Thus the 100-sample model correctly contains 200 observation rows;
it does not contain 200 distinct surface locations. The resolution index and
suite generator verify both the unique-location count and operator counts.

Each candidate has its own `model.npz`, `manifest.json`, and
`training_report.json`. `resolution_candidates.json` indexes their model IDs,
artifact hashes, and source-data provenance.

The sole point source is `data/static_1m_robot.npz`, array `points_base_link`.
The adjacent `data/static_1m_robot.json` is extraction metadata and provenance;
it is not a second point cloud. The models use Open3D-estimated normals and
surface-only Hermite constraints (`f=0`, `∇f·n=1`), with no off-surface anchors.

Rebuild the set from the repository root:

```powershell
$env:PYTHONPATH = "hsl_core"
python tools\create_gpis_resolution_suite.py data\static_1m_robot.npz artifacts\models\opponent_gpis\static_1m_robot_hgw_20261002
```

These are resolution candidates, not independently validated recognition
models. The index records hashes so each candidate can be checked against its
source; identification accuracy and the minimum usable resolution still need
real test data.
