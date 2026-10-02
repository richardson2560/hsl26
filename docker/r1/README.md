# R1 — inventario y autoridad del grafo ROS en vivo

R1 inspecciona un grafo ROS que ya está en ejecución. No inicia drivers Kobuki
o Livox, no publica comandos y no autoriza movimiento. El único proceso de la
prueba positiva es el `cmd_vel_mux` heredado, sin entrada ni base conectada.

Ejecute desde la raíz del repositorio en PowerShell. Cree un identificador de
evidencia nuevo; R1 no debe sobrescribir los reportes R0.

```powershell
$RunId = "r1_$(Get-Date -Format 'yyyyMMddTHHmmssZ')"
$Report = "artifacts/reports/r1/$RunId"
$R1Image = 'hsl26:r1-graph'
New-Item -ItemType Directory -Force $Report | Out-Null
git rev-parse HEAD | Out-File "$Report/git-head.txt" -Encoding utf8
git status --short | Out-File "$Report/git-status.txt" -Encoding utf8
Get-FileHash ros_ws/src/hsl_bringup/config/ros_graph_contract.json -Algorithm SHA256 |
  Format-List | Out-File "$Report/ros-graph-contract-sha256.txt" -Encoding utf8
```

## 1. Construir el candidato R1

La imagen R0 ya congelada sigue siendo la evidencia de R0. Esta construcción
incluye el código R1 nuevo y por tanto es un candidato separado; no sustituye
la etiqueta ni el digest que fueron revisados para R0.

El repositorio contiene un `.dockerignore` que excluye `__pycache__` y
`.pytest_cache` generados. Si Docker Desktop informa `Access is denied` al
cargar el contexto, confirme que ese fichero está presente en la raíz y repita
el mismo comando; no elimine ni cambie permisos de una caché que pueda estar
abierta por Python o el IDE.

```powershell
docker build --no-cache --pull=false --progress=plain `
  --build-arg BASE_IMAGE=nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619 `
  -f docker/Dockerfile.acceptance -t $R1Image . 2>&1 |
  Tee-Object "$Report/r1-image-build.txt"
docker image inspect $R1Image |
  Out-File "$Report/r1-image-inspect.json" -Encoding utf8
```

El build puede reutilizar las capas apt controladas de Docker. No añada
`--privileged`, `--device`, montajes del checkout ni `--network host` a los
comandos de ejecución siguientes.

`--no-cache` es obligatorio para R1: la evidencia debe mostrar que el paso
`COPY ros_ws/src` y el `colcon build` se ejecutaron para este checkout. Si esos
pasos aparecen como `CACHED`, la imagen puede contener una instalación anterior
y no incluir `hsl_graph_verifier`.

## 2. Localizar y arrancar únicamente el mux en un contenedor aislado

Primero confirme el ejecutable y su fichero de parámetros instalado. No
adivine el nombre del fichero si este comando no devuelve exactamente uno.

```powershell
docker run --rm --network none --entrypoint /bin/bash $R1Image -lc `
  "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && ros2 pkg executables cmd_vel_mux && find /workspace_kobuki/install -path '*cmd_vel_mux*' -type f -name '*.yaml' -print" 2>&1 |
  Tee-Object "$Report/cmd-vel-mux-discovery.txt"
```

Sustituya el valor de `$MuxConfig` por el único YAML de parámetros que indique
la salida `output: "/commands/velocity"` (normalmente está bajo
`/workspace_kobuki/install/cmd_vel_mux/share/cmd_vel_mux/config/`).
La salida registrada en R1 debe además contener el ejecutable
`cmd_vel_mux_node`; úselo tal cual, no `cmd_vel_mux`.

```powershell
$MuxConfig = '/workspace_kobuki/install/cmd_vel_mux/share/cmd_vel_mux/config/cmd_vel_mux_params.yaml'
$MuxExecutable = 'cmd_vel_mux_node'
$Container = 'hsl26-r1-mux'
docker rm -f $Container 2>$null
docker run -d --name $Container --network none --entrypoint /bin/bash $R1Image -lc 'exec sleep infinity' |
  Tee-Object "$Report/r1-container-id.txt"
docker exec -d $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && exec ros2 run cmd_vel_mux $MuxExecutable --ros-args --params-file $MuxConfig"
Start-Sleep -Seconds 2
docker exec $Container /bin/bash -lc "pgrep -af '[c]md_vel_mux_node'" |
  Tee-Object "$Report/cmd-vel-mux-process.txt"
```

## 3. Verificar autoridad y capturar el inventario vivo

El verificador termina con cero solamente si hay exactamente un publicador de
`/commands/velocity` y se llama `/cmd_vel_mux`. También almacena los endpoints
y sus QoS de todos los tópicos declarados, sin modificar ninguno.

```powershell
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_bringup hsl_graph_verifier --output /tmp/r1-live-graph.json" 2>&1 |
  Tee-Object "$Report/r1-live-verifier.txt"
docker cp "${Container}:/tmp/r1-live-graph.json" "$Report/r1-live-graph.json"
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && ros2 topic info --verbose /commands/velocity" 2>&1 |
  Tee-Object "$Report/commands-velocity-verbose.txt"
```

## 4. Pruebas negativas y reinicio de fuente

R1 no acepta un segundo publicador físico. Las dos pruebas negativas usan
`r1_graph_fixture`, que sólo publica un `Twist` cero y nunca arranca un driver.
El fallo del verificador es el resultado esperado; el script PowerShell lo
convierte en evidencia `PASS` sólo cuando dicho fallo sucede.

```powershell
docker exec -d $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && exec ros2 run hsl_bringup r1_graph_fixture --scenario unauthorized_command --duration-seconds 30"
Start-Sleep -Seconds 2
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_bringup hsl_graph_verifier --output /tmp/r1-unauthorized-writer.json" 2>&1 |
  Tee-Object "$Report/r1-unauthorized-writer.txt"
$UnauthorizedWriterExit = $LASTEXITCODE
docker cp "${Container}:/tmp/r1-unauthorized-writer.json" "$Report/r1-unauthorized-writer.json"
if ($UnauthorizedWriterExit -eq 0) { throw 'R1-C FAIL: un segundo escritor físico no fue rechazado.' }
docker exec $Container /bin/bash -lc "pkill -f '[r]1_graph_fixture'" 2>$null

docker exec -d $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && exec ros2 run hsl_bringup r1_graph_fixture --scenario qos_incompatible --duration-seconds 30"
Start-Sleep -Seconds 2
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_bringup hsl_graph_verifier --additional-topic /r1/qos_incompatible --output /tmp/r1-qos-incompatible.json" 2>&1 |
  Tee-Object "$Report/r1-qos-incompatible.txt"
$QosFixtureExit = $LASTEXITCODE
docker cp "${Container}:/tmp/r1-qos-incompatible.json" "$Report/r1-qos-incompatible.json"
if ($QosFixtureExit -eq 0) { throw 'R1-C FAIL: la QoS incompatible no fue rechazada.' }
docker exec $Container /bin/bash -lc "pkill -f '[r]1_graph_fixture'" 2>$null
```

## 5. Estructura de TF estático

El verificador se suscribe pasivamente a `/tf_static` con QoS `reliable` y
`transient_local`. La prueba positiva exige las aristas de calibración
`base_link→lidar_link` y `base_link→imu_link`; la negativa introduce una arista
duplicada. Los fixtures no emiten `/tf` dinámico ni comandos.

```powershell
docker exec -d $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && exec ros2 run hsl_bringup r1_graph_fixture --scenario tf_valid --duration-seconds 30"
Start-Sleep -Seconds 2
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_bringup hsl_graph_verifier --check-tf-static --settle-seconds 2 --output /tmp/r1-tf-valid.json" 2>&1 |
  Tee-Object "$Report/r1-tf-valid.txt"
if ($LASTEXITCODE -ne 0) { throw 'R1-D FAIL: las aristas TF estáticas válidas no fueron aceptadas.' }
docker cp "${Container}:/tmp/r1-tf-valid.json" "$Report/r1-tf-valid.json"
docker exec $Container /bin/bash -lc "pkill -f '[r]1_graph_fixture'" 2>$null

docker exec -d $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_hsl26/install/setup.bash && exec ros2 run hsl_bringup r1_graph_fixture --scenario tf_duplicate --duration-seconds 30"
Start-Sleep -Seconds 2
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_bringup hsl_graph_verifier --check-tf-static --settle-seconds 2 --output /tmp/r1-tf-duplicate.json" 2>&1 |
  Tee-Object "$Report/r1-tf-duplicate.txt"
$TfDuplicateExit = $LASTEXITCODE
docker cp "${Container}:/tmp/r1-tf-duplicate.json" "$Report/r1-tf-duplicate.json"
if ($TfDuplicateExit -eq 0) { throw 'R1-D FAIL: una arista TF duplicada no fue rechazada.' }
docker exec $Container /bin/bash -lc "pkill -f '[r]1_graph_fixture'" 2>$null
```

Para el reinicio del mux, capture una segunda instantánea con un PID nuevo y
compare que la autoridad sigue siendo exclusiva. Pare y arranque de nuevo el
proceso sólo en el contenedor de prueba:

```powershell
docker exec $Container /bin/bash -lc "pkill -f '[c]md_vel_mux'"
Start-Sleep -Seconds 1
docker exec -d $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && exec ros2 run cmd_vel_mux $MuxExecutable --ros-args --params-file $MuxConfig"
Start-Sleep -Seconds 2
docker exec $Container /bin/bash -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && ros2 run hsl_bringup hsl_graph_verifier --output /tmp/r1-after-restart.json" 2>&1 |
  Tee-Object "$Report/r1-after-restart-verifier.txt"
docker cp "${Container}:/tmp/r1-after-restart.json" "$Report/r1-after-restart.json"
docker rm -f $Container
```

Estas fixtures validan el mecanismo estructural, no atribuyen una arista a un
nodo de producción: los mensajes TF no transportan ese nombre de nodo. Antes
de promover un grafo con localización real, su launch y el contrato firmado
deben identificar al broadcaster propietario de cada arista. R2, R7, hardware
y movimiento permanecen expresamente fuera de este procedimiento.
