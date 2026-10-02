# R2 — Supervisor y watchdog con parada cerrada por defecto

R2 implementa los dos procesos ROS de seguridad y sus salidas de parada. No
instala un perfil de límites/calibración aceptado y, por diseño, **no puede
autorizar movimiento**. Es una prueba del límite de proceso, del latido y de
la prioridad de parada; no una autorización para banco físico o competición.

## 1. Construir el candidato

Desde la raíz del repositorio, en PowerShell:

```powershell
$RunId = Get-Date -Format "yyyyMMddTHHmmssZ"
$Report = "artifacts/reports/r2/r2_$RunId"
New-Item -ItemType Directory -Force $Report | Out-Null
$R2Image = "hsl26:r2-safety"
docker build --pull=false --no-cache --progress=plain `
  --build-arg BASE_IMAGE=nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619 `
  -f docker/Dockerfile.acceptance -t $R2Image . 2>&1 |
  Tee-Object "$Report/r2-image-build.txt"
docker image inspect $R2Image | Out-File "$Report/r2-image-inspect.json" -Encoding utf8
```

La reconstrucción invalida la reutilización de la evidencia R1 como imagen de
release; conserva sus informes, pero vuelve a ejecutar R1 antes de cualquier
aceptación conjunta.

## 2. Pruebas unitarias sin ROS ni red

La imagen de aceptación contiene sólo los artefactos instalados, no los
ficheros `test/`. Construya el stage de compilación por separado para ejecutar
las pruebas exactamente contra las fuentes incluidas en el candidato:

```powershell
docker build --pull=false --no-cache --progress=plain --target hsl-build `
  --build-arg BASE_IMAGE=nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619 `
  -f docker/Dockerfile.acceptance -t hsl26:r2-safety-test . 2>&1 |
  Tee-Object "$Report/r2-test-image-build.txt"
docker run --rm --network none --entrypoint /bin/bash hsl26:r2-safety-test -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && PYTHONPATH=/workspace_hsl26/src/hsl_safety pytest -q /workspace_hsl26/src/hsl_safety/test" 2>&1 |
  Tee-Object "$Report/r2-unit-tests.txt"
```

## 3. Arranque aislado y comprobación de parada

```powershell
$Container = "hsl26-r2-safety"
docker rm -f $Container 2>$null
docker run -d --rm --name $Container --network none $R2Image /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 launch hsl_safety safety.launch.py"
Start-Sleep -Seconds 3
docker logs $Container 2>&1 | Tee-Object "$Report/r2-launch.txt"
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 topic echo /hsl/cmd_vel_stop --once && ros2 topic echo /hsl/cmd_vel_final --once && ros2 topic echo /hsl/watchdog_health --once" 2>&1 |
  Tee-Object "$Report/r2-zero-and-health.txt"
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 service call /hsl/rearm_safety hsl_interfaces/srv/RearmSafety \"{request_id: r2-test, expected_stage_id: r2-disarmed, expected_config_hash: UNCONFIGURED_NO_MOTION, recovery_policy_id: none, authorization_ref: none}\"" 2>&1 |
  Tee-Object "$Report/r2-rearm-denied.txt"
```

Se espera `linear.x: 0.0`, `angular.z: 0.0`, `stop_asserted: true` y un
`accepted: false` explícito. Cualquier comando no nulo, una parada no asertada
o un rearme aceptado es un **FAIL R2**.

## 4. Cerrar el proceso

```powershell
docker logs $Container 2>&1 | Tee-Object "$Report/r2-container-final.txt"
docker stop $Container
```

No conectes `/commands/velocity`, hardware, Livox ni MVSim en esta prueba. La
integración con mux y la evidencia de detención física requieren R3/R4 y una
calibración aprobada.
