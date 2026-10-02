# R3-A — admisión de observaciones normalizadas

R3-A no abre Livox, Kobuki ni MVSim y no publica comandos. Verifica el núcleo
que admite una nube sólo cuando conserva frame, epochs, calibración, tiempo por
punto y frescura; cualquier inconsistencia se rechaza antes de producir
geometría local o cobertura libre.

## Ejecutar en el stage de compilación aislado

Desde la raíz del repositorio, en PowerShell:

```powershell
$RunId = "r3a_$(Get-Date -Format 'yyyyMMddTHHmmssZ')"
$Report = "artifacts/reports/r3/$RunId"
New-Item -ItemType Directory -Force $Report | Out-Null
$R3TestImage = 'hsl26:r3-observation-test'
docker build --pull=false --no-cache --progress=plain --target hsl-build `
  --build-arg BASE_IMAGE=nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619 `
  -f docker/Dockerfile.acceptance -t $R3TestImage . 2>&1 |
  Tee-Object "$Report/r3a-test-image-build.txt"
docker image inspect $R3TestImage | Out-File "$Report/r3a-test-image-inspect.json" -Encoding utf8
docker run --rm --network none --entrypoint /bin/bash $R3TestImage -lc "source /opt/ros/humble/setup.bash && PYTHONPATH=/workspace_hsl26/hsl_core pytest -q /workspace_hsl26/hsl_core/tests/test_observation_gate.py /workspace_hsl26/hsl_core/tests/test_deskew.py /workspace_hsl26/hsl_core/tests/test_mapping.py" 2>&1 |
  Tee-Object "$Report/r3a-observation-core-tests.txt"
```

El resultado esperado es `51 passed`. Un rechazo por frame, epoch, calibración,
stale timestamp o punto posterior al timestamp de observación es un resultado
correcto de seguridad.

## Límites explícitos de R3-A

Todavía no hay adaptador ROS de Livox/IMU/odom ni se publican `EgoState` o
`LocalObstacleSnapshot`. Antes de R3-B hay que capturar y versionar: campos y
unidades de la nube normalizada (incluido timestamp por punto), QoS ofrecido,
frame de cada sensor, transformada extrínseca con `calibration_id`, covarianzas
de odometría/IMU, comportamiento de no-retorno y zonas ciegas. Sin esa
evidencia, cualquier nodo ROS debe rechazar la entrada o declarar cobertura
desconocida; no puede inventar espacio libre.
