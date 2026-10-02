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
docker image inspect $R2Image --format '{{.Id}} entrypoint={{json .Config.Entrypoint}}' |
  Tee-Object "$Report/r2-image-entrypoint.txt"
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
docker run --rm --network none --entrypoint /bin/bash hsl26:r2-safety-test -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && PYTHONPATH=/workspace_hsl26:/workspace_hsl26/src/hsl_safety pytest -q /workspace_hsl26/src/hsl_safety/test" 2>&1 |
  Tee-Object "$Report/r2-unit-tests.txt"
```

`test_p17_mux.py` comprueba el verificador estático de autoridad como parte
del contrato del mux. Por eso el stage `hsl-build` copia expresamente
`tools/check_ros_graph_authority.py`; `/workspace_hsl26` debe estar en
`PYTHONPATH` junto al paquete fuente. Si se usó una imagen construida antes de
este cambio, hay que repetir el `docker build --target hsl-build`: modificar
sólo el comando `docker run` no añade el módulo a una imagen ya creada.

## 3. Arranque aislado y comprobación de parada

```powershell
$Container = "hsl26-r2-safety"
docker rm -f $Container 2>$null
# No use --rm here: a launch failure must leave its log inspectable.
docker run -d --name $Container --network none --entrypoint /bin/bash $R2Image -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 launch hsl_safety safety.launch.py"
Start-Sleep -Seconds 3
docker logs $Container 2>&1 | Tee-Object "$Report/r2-launch.txt"
docker inspect $Container --format '{{.State.Status}} exit={{.State.ExitCode}}' |
  Tee-Object "$Report/r2-container-state.txt"
if ((docker inspect $Container --format '{{.State.Running}}') -ne 'true') {
  throw 'R2 launch failed: revise r2-launch.txt before running any docker exec check.'
}
```

Sólo si `r2-container-state.txt` dice `running exit=0`, ejecute las
comprobaciones siguientes. No ejecute todavía la sección 4.

```powershell
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 topic echo /hsl/cmd_vel_stop --once && ros2 topic echo /hsl/cmd_vel_final --once && ros2 topic echo /hsl/watchdog_health --once" 2>&1 |
  Tee-Object "$Report/r2-zero-and-health.txt"
# R2 rejects every request; `{}` avoids PowerShell/Docker YAML quoting changes.
docker exec $Container /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 service call /hsl/rearm_safety hsl_interfaces/srv/RearmSafety {}' 2>&1 |
  Tee-Object "$Report/r2-rearm-denied.txt"
```

Se espera `linear.x: 0.0`, `angular.z: 0.0`, `stop_asserted: true` y un
`accepted: false` explícito. Cualquier comando no nulo, una parada no asertada
o un rearme aceptado es un **FAIL R2**.

## 3B. R2-B — prioridad watchdog sobre el mux, sin hardware

R2-B se ejecuta en un contenedor nuevo y aislado. Arranca el mux y los dos
procesos R2, pero **no** un driver Kobuki ni un dispositivo. El probe es un
ejecutable de prueba que publica `linear.x=0.2` sólo en la entrada lógica
`/hsl/cmd_vel_final`; debe terminar con `PASS` porque la entrada de stop del
watchdog tiene mayor prioridad y `/commands/velocity` debe permanecer en cero.
No ejecute este probe fuera de este contenedor de prueba.

```powershell
$RunId = "r2b_$(Get-Date -Format 'yyyyMMddTHHmmssZ')"
$Report = "artifacts/reports/r2/$RunId"
New-Item -ItemType Directory -Force $Report | Out-Null
$R2Image = 'hsl26:r2-safety-r2b'
docker build --pull=false --no-cache --progress=plain `
  --build-arg BASE_IMAGE=nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619 `
  -f docker/Dockerfile.acceptance -t $R2Image . 2>&1 |
  Tee-Object "$Report/r2b-image-build.txt"
docker image inspect $R2Image | Out-File "$Report/r2b-image-inspect.json" -Encoding utf8

$Container = 'hsl26-r2b-mux'
$MuxConfig = '/workspace_kobuki/install/cmd_vel_mux/share/cmd_vel_mux/config/cmd_vel_mux_params.yaml'
docker rm -f $Container 2>$null
docker run -d --name $Container --network none --entrypoint /bin/bash $R2Image -lc 'exec sleep infinity' |
  Tee-Object "$Report/r2b-container-id.txt"
docker exec -d $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && exec ros2 run cmd_vel_mux cmd_vel_mux_node --ros-args --params-file $MuxConfig"
docker exec -d $Container /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && exec ros2 launch hsl_safety safety.launch.py'
Start-Sleep -Seconds 3
docker exec $Container /bin/bash -lc "pgrep -af '[c]md_vel_mux_node|[s]afety_supervisor|[s]afety_watchdog'" |
  Tee-Object "$Report/r2b-processes.txt"
docker exec -e HSL26_TEST_ONLY=1 $Container /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_safety r2_stop_priority_probe --allow-test-nonzero --linear-x 0.2 --duration-seconds 3 --minimum-samples 5' 2>&1 |
  Tee-Object "$Report/r2b-stop-priority.txt"
if ($LASTEXITCODE -ne 0) { throw 'R2-B FAIL: el stop del watchdog no domino la salida del mux.' }
docker exec $Container /bin/bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic info --verbose /commands/velocity' 2>&1 |
  Tee-Object "$Report/r2b-physical-writer.txt"
```

El resultado aprobado contiene `PASS: ... physical samples remained zero` y
un solo publicador de `/commands/velocity`, `/cmd_vel_mux`. El probe es un
publicador temporal no autorizado de una **entrada** del mux, no del tópico
físico; no sustituye R1 ni se conserva en un launch de producción.

## 3C. R2-C — caída del supervisor y expiración del latido

Ejecute esta prueba sólo después de R2-B, con el mismo `$Container` aún vivo.
Mata el supervisor, espera más de su lease de `0.15 s`, y vuelve a inyectar
la entrada lógica de prueba. El watchdog debe quedar latched, conservar su
stop y el probe debe seguir observando únicamente ceros en la salida del mux.

```powershell
docker exec $Container /bin/bash -lc "pkill -f '[s]afety_supervisor'"
Start-Sleep -Milliseconds 500
docker exec $Container /bin/bash -lc "pgrep -af '[s]afety_supervisor|[s]afety_watchdog|[c]md_vel_mux_node'" |
  Tee-Object "$Report/r2c-processes-after-supervisor-kill.txt"
docker exec -e HSL26_TEST_ONLY=1 $Container /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_safety r2_stop_priority_probe --allow-test-nonzero --linear-x 0.2 --duration-seconds 3 --minimum-samples 5' 2>&1 |
  Tee-Object "$Report/r2c-stop-after-supervisor-kill.txt"
if ($LASTEXITCODE -ne 0) { throw 'R2-C FAIL: la salida del mux no permaneció en cero tras matar el supervisor.' }
$Health = docker exec $Container /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 topic echo /hsl/watchdog_health --once' 2>&1
$Health | Tee-Object "$Report/r2c-watchdog-health-after-supervisor-kill.txt"
if ($Health -notmatch 'healthy: false' -or $Health -notmatch 'stop_asserted: true' -or $Health -notmatch 'fault_reason: STALE_HEARTBEAT') {
  throw 'R2-C FAIL: el watchdog no quedó en parada latched por heartbeat vencido.'
}
```

Un `PASS` de R2-C no prueba la caída del propio watchdog ni una detención
física; esas pruebas exigen downstream/hardware calibrado y quedan fuera de
este contenedor.

## 3D. R2-D — diagnóstico de caída del watchdog (resultado esperado: bloqueo)

Este caso se ejecuta sólo en el contenedor aislado R2-B/R2-C, sin driver ni
dispositivo. Después de matar el watchdog, el probe lógico debe detectar una
salida no nula del mux y terminar con error: ese resultado **no es un PASS de
seguridad**, sino la evidencia de que la garantía de parada ante pérdida del
watchdog debe proporcionarla un interlock/downstream independiente. No ejecute
este paso con hardware, MVSim conectado a actuadores ni ningún driver activo.

```powershell
docker exec $Container /bin/bash -lc "pkill -f '[s]afety_watchdog'"
Start-Sleep -Milliseconds 500
docker exec $Container /bin/bash -lc "pgrep -af '[s]afety_watchdog|[c]md_vel_mux_node'" |
  Tee-Object "$Report/r2d-processes-after-watchdog-kill.txt"
docker exec -e HSL26_TEST_ONLY=1 $Container /bin/bash -lc 'source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_safety r2_stop_priority_probe --allow-test-nonzero --linear-x 0.2 --duration-seconds 3 --minimum-samples 5' 2>&1 |
  Tee-Object "$Report/r2d-watchdog-loss-diagnostic.txt"
$WatchdogLossExit = $LASTEXITCODE
if ($WatchdogLossExit -eq 0) {
  throw 'R2-D inconcluso: el probe no detectó la ausencia del watchdog.'
}
```

Se espera el texto `FAIL: watchdog stop did not dominate mux output`. Registre
el resultado como limitación abierta, no como autorización para operar. La
aceptación de R2 requiere después una prueba del interlock real y del timeout
del controlador con su configuración y latencias medidas.

## 4. Cerrar el proceso

```powershell
docker logs $Container 2>&1 | Tee-Object "$Report/$RunId-container-final.txt"
docker rm -f $Container
```

No conectes `/commands/velocity`, hardware, Livox ni MVSim en esta prueba. La
integración con mux y la evidencia de detención física requieren R3/R4 y una
calibración aprobada.
