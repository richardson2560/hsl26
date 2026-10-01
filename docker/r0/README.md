# R0 — ejecución en un host con Docker

Esta carpeta contiene el procedimiento reproducible para **capturar** el entorno. No sustituye la revisión ni autoriza G0. Ejecute todos los comandos desde la raíz del repositorio en PowerShell. Guarde las salidas en `artifacts/reports/r0/<run-id>/`; no sobrescriba una ejecución anterior.

## 1. Capturar la imagen heredada

```powershell
$RunId = "r0_$(Get-Date -Format yyyyMMddTHHmmssZ)"
$Report = "artifacts/reports/r0/$RunId"
$BaseImage = "nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619"
New-Item -ItemType Directory -Force $Report | Out-Null
docker version | Tee-Object "$Report/docker-version.txt"
docker buildx version | Tee-Object "$Report/buildx-version.txt"
docker pull $BaseImage
python tools/r0_capture_environment.py --base-image $BaseImage --output "$Report/base-image-capture.json"
Get-Content "$Report/base-image-capture.json"
```

El `docker pull` muestra el progreso de una descarga inicial; puede tardar, pero no debe parecer bloqueado. El capturador muestra cada sonda y limita cada una a 90 segundos, registrando un timeout como fallo de la sonda en vez de abandonar el informe. Envíe el JSON completo; no copie únicamente la etiqueta. Si se necesita resolver otra imagen o comprobar el digest de plataforma, ejecute y guarde también:

```powershell
docker buildx imagetools inspect $BaseImage | Tee-Object "$Report/base-image-imagetools.txt"
```

## 2. Capturar el bundle apt y construir el resolver bootstrap

La caché inicial de la imagen puede contener candidatos apt caducados. El siguiente script ejecuta `apt-get update`, resuelve el candidato disponible **en esa misma operación**, registra la resolución y descarga sus `.deb` y dependencias. Es un **bootstrap controlado**, no una imagen aceptable. No fije ni sustituya una versión manualmente.

```powershell
$AptBundle = Join-Path $PWD.Path "docker/r0/apt-wheel-resolver"
if (Test-Path -LiteralPath $AptBundle) {
    if ((Get-ChildItem -LiteralPath $AptBundle -Force | Measure-Object).Count -gt 0) {
        throw "El bundle apt ya contiene archivos; revísalo o crea un run-id nuevo antes de continuar."
    }
}
New-Item -ItemType Directory -Force $AptBundle | Out-Null

docker run --rm --network bridge -v "$($PWD.Path):/workspace" -w /workspace --entrypoint /bin/bash $BaseImage -lc "bash docker/r0/download_apt_bootstrap.sh /workspace/docker/r0/apt-wheel-resolver" 2>&1 | Tee-Object "$Report/apt-bootstrap-download.txt"
Get-Content "$AptBundle/apt-resolution.txt" | Tee-Object "$Report/apt-bootstrap-resolution.txt"
Get-Content "$AptBundle/apt-bundle-packages.tsv" | Tee-Object "$Report/apt-bootstrap-packages.tsv"
Get-ChildItem $AptBundle -Filter *.deb -File | Get-FileHash -Algorithm SHA256 | Sort-Object Path | Format-Table -AutoSize | Out-File "$Report/apt-bootstrap-sha256.txt"

docker build --network none --no-cache --progress=plain --build-arg BASE_IMAGE=$BaseImage -f docker/r0/Dockerfile.wheel-resolver -t hsl26:r0-wheel-resolver . 2>&1 | Tee-Object "$Report/wheel-resolver-build.txt"
python tools/r0_capture_environment.py --base-image hsl26:r0-wheel-resolver --output "$Report/wheel-resolver-capture.json"
```

El build del resolver debe terminar sin red y sin advertencia `InvalidDefaultArgInFrom`; su captura debe mostrar `pip` con código 0. Si `dpkg --install` solicita una dependencia ausente, no use `apt -f install`: envíe el log para completar el bundle de forma explícita.

## 3. Descargar el wheelhouse de la plataforma Linux objetivo

El wheelhouse se resuelve dentro del resolver Linux, no con Python de Windows. Esto evita fijar ruedas incompatibles por error. El siguiente comando necesita red **solo** para esta descarga controlada:

```powershell
$Wheelhouse = Join-Path $PWD.Path "docker/r0/wheelhouse"
if (Test-Path -LiteralPath $Wheelhouse) {
    if ((Get-ChildItem -LiteralPath $Wheelhouse -Force | Measure-Object).Count -gt 0) {
        throw "El wheelhouse ya contiene archivos; revísalo o crea un run-id nuevo antes de continuar."
    }
}
New-Item -ItemType Directory -Force $Wheelhouse | Out-Null
docker run --rm --network bridge -v "$($PWD.Path):/workspace" -w /workspace/docker/r0 --entrypoint /bin/bash hsl26:r0-wheel-resolver -lc "python3 -m pip download --disable-pip-version-check --progress-bar off --retries 5 --timeout 300 --only-binary=:all: --dest wheelhouse -r requirements.in" 2>&1 | Tee-Object "$Report/wheelhouse-download.txt"
python docker/r0/generate_requirements_lock.py --wheelhouse docker/r0/wheelhouse --output docker/r0/requirements.lock
python docker/r0/verify_requirements_lock.py --wheelhouse docker/r0/wheelhouse --lock docker/r0/requirements.lock
Get-FileHash docker/r0/requirements.lock -Algorithm SHA256 | Tee-Object "$Report/requirements-lock-sha256.txt"
Get-ChildItem docker/r0/wheelhouse -File | Get-FileHash -Algorithm SHA256 | Sort-Object Path | Format-Table -AutoSize | Out-File "$Report/wheelhouse-sha256.txt"
```

Si no hay wheel binario para la combinación de Python/arquitectura, deténgase: no cambie a una compilación implícita. Envíe el error para decidir si se añade una dependencia de compilación o se separa el componente opcional.

El comando de descarga es la única operación PyPI autorizada para R0.3. Usa cinco reintentos y 300 segundos por lectura para enlaces lentos. Si no encuentra una rueda binaria, se detiene sin compilar desde fuente ni instalar paquetes. Si falla tras agotar los reintentos, conserve el log y no genere un lock desde un wheelhouse parcial.

### Corrección de compatibilidad `ament_python`

Los paquetes ROS 2 Humble de este repositorio se construyen con `colcon build --symlink-install`. Esa ruta invoca `setup.py develop --editable`; `setuptools 80+` retiró dicho argumento. Por ello `requirements.in` fija `setuptools==79.0.1`. Si ya se descargó el wheelhouse anterior con `setuptools-84.0.0`, sustituya **solo** esa rueda y regenere el lock antes de reconstruir la imagen:

```powershell
$OldSetuptools = Join-Path $PWD.Path "docker/r0/wheelhouse/setuptools-84.0.0-py3-none-any.whl"
if (-not (Test-Path -LiteralPath $OldSetuptools)) { throw "No se encontró la rueda anterior esperada: $OldSetuptools" }
Remove-Item -LiteralPath $OldSetuptools
docker run --rm --network bridge -v "$($PWD.Path):/workspace" -w /workspace/docker/r0 --entrypoint /bin/bash hsl26:r0-wheel-resolver -lc "python3 -m pip download --disable-pip-version-check --progress-bar off --retries 5 --timeout 300 --only-binary=:all: --dest wheelhouse setuptools==79.0.1" 2>&1 | Tee-Object "$Report/setuptools-79-download.txt"
python docker/r0/generate_requirements_lock.py --wheelhouse docker/r0/wheelhouse --output docker/r0/requirements.lock
python docker/r0/verify_requirements_lock.py --wheelhouse docker/r0/wheelhouse --lock docker/r0/requirements.lock
Get-FileHash docker/r0/requirements.lock -Algorithm SHA256 | Tee-Object "$Report/requirements-lock-sha256.txt"
```

No continúe si el verificador informa dos ruedas para una misma distribución, si falta `setuptools-79.0.1`, o si la instalación offline de la sección siguiente no termina satisfactoriamente.

El resumen final de BuildKit (`failed to solve ... exit code: 1`) no identifica la causa. Para este incidente, la línea determinante de `development-build.txt` es `hsl_perception ... error: option --editable not recognized`. El Dockerfile contiene una precondición que, mientras el lock aún tenga `setuptools>=80`, falla antes del build ROS con una instrucción explícita; no es un resultado válido de R0-B.

### Corrección de compatibilidad `pytest` / `launch_testing`

El primer build con `setuptools 79.0.1` completó los nueve paquetes HSL26, pero el test ROS no llegó a ejecutar `test_contract_schema.py`: `pytest 9.1.1` rechaza el hook de colección que implementa `launch_testing` de ROS 2 Humble. `requirements.in` fija por ello `pytest==7.4.4`. Sustituya exclusivamente la rueda anterior, regenere el lock y vuelva a validar offline:

```powershell
$OldPytest = Join-Path $PWD.Path "docker/r0/wheelhouse/pytest-9.1.1-py3-none-any.whl"
if (-not (Test-Path -LiteralPath $OldPytest)) { throw "No se encontró la rueda anterior esperada: $OldPytest" }
Remove-Item -LiteralPath $OldPytest
docker run --rm --network bridge -v "$($PWD.Path):/workspace" -w /workspace/docker/r0 --entrypoint /bin/bash hsl26:r0-wheel-resolver -lc "python3 -m pip download --disable-pip-version-check --progress-bar off --retries 5 --timeout 300 --only-binary=:all: --dest wheelhouse pytest==7.4.4" 2>&1 | Tee-Object "$Report/pytest-7-download.txt"
python docker/r0/generate_requirements_lock.py --wheelhouse docker/r0/wheelhouse --output docker/r0/requirements.lock
python docker/r0/verify_requirements_lock.py --wheelhouse docker/r0/wheelhouse --lock docker/r0/requirements.lock
docker run --rm --network none -v "$($PWD.Path):/workspace:ro" -w /workspace/docker/r0 --entrypoint /bin/bash hsl26:r0-wheel-resolver -lc "python3 -m pip install --no-index --require-hashes --find-links wheelhouse -r requirements.lock && python3 -m pip check" 2>&1 | Tee-Object "$Report/wheelhouse-offline-install.txt"
```

El Dockerfile rechaza también un lock con `pytest>=8` antes de compilar ROS. Después de esta corrección se deben repetir el build y `colcon test`; el resumen anterior `2 tests, 1 error, 1 failure` no revela aún un fallo funcional del contrato.

## 4. Comprobar que la instalación puede hacerse sin red

```powershell
docker run --rm --network none -v "$($PWD.Path):/workspace:ro" -w /workspace/docker/r0 --entrypoint /bin/bash hsl26:r0-wheel-resolver -lc "python3 -m pip install --no-index --require-hashes --find-links wheelhouse -r requirements.lock && python3 -m pip check" 2>&1 | Tee-Object "$Report/wheelhouse-offline-install.txt"
```

Este comando no prueba todavía la imagen HSL26 ni inicia ningún driver. Su éxito solo demuestra que las dependencias Python descargadas están completas y verificadas para esa base.

## 5. Build de desarrollo existente y evidencia ROS (R0-B)

```powershell
$BaseImage = "nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619"
docker build --pull --progress=plain --build-arg BASE_IMAGE=$BaseImage -f docker/Dockerfile -t hsl26:development . 2>&1 | Tee-Object "$Report/development-build.txt"
docker run --rm --network none --entrypoint /bin/bash hsl26:development -lc "source /opt/ros/humble/setup.bash && source /workspace_kobuki/install/setup.bash && source /workspace_hsl26/install/setup.bash && colcon test --base-paths /workspace_hsl26/src --event-handlers console_direct+ && colcon test-result --verbose" 2>&1 | Tee-Object "$Report/ros-test.txt"
docker image inspect hsl26:development | Out-File "$Report/development-image-inspect.json" -Encoding utf8
```

No conecte `--privileged`, `--device`, dispositivos serie/USB, red host ni mounts de código durante R0-D. El build actual puede requerir red para apt/PyPI y se clasifica solo como desarrollo: conserve el log y no lo use como evidencia de imagen de aceptación. La ejecución de los tests debe usar red `none`.

## 6. Candidato acceptance y R0-D

`docker/Dockerfile.acceptance` es distinto del perfil development: compila ambos
workspaces sin `--symlink-install`; el stage final solo recibe instalaciones,
el wheel de `hsl_core` y el wheelhouse R0. No arranca drivers ni nodos por su
entrypoint. Construya y pruebe el candidato así:

```powershell
$BaseImage = "nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619"
docker build --pull --progress=plain --build-arg BASE_IMAGE=$BaseImage -f docker/Dockerfile.acceptance -t hsl26:acceptance . 2>&1 | Tee-Object "$Report/acceptance-build.txt"
docker run --rm --network none --entrypoint /usr/local/bin/hsl26-acceptance-smoke hsl26:acceptance 2>&1 | Tee-Object "$Report/acceptance-runtime-smoke.txt"
docker image inspect hsl26:acceptance | Out-File "$Report/acceptance-image-inspect.json" -Encoding utf8
```

El smoke debe imprimir `acceptance environment OK` y no mostrar procesos de
drivers, MVSim o movimiento. Si el build final informa una biblioteca runtime
faltante, conserve el log: no añada apt desde red al stage final.

## 7. Entrega para revisión

## 8. R0-E — descubrimiento seguro de MVSim y drivers

Ejecute primero este inventario sin red, dispositivos ni mounts. No inicia ROS,
MVSim ni drivers; solo consulta ejecutables, paquetes y bibliotecas ya presentes
en `hsl26:acceptance`:

```powershell
docker run --rm --network none --entrypoint /bin/bash hsl26:acceptance -lc "set -e; echo '== executables =='; command -v mvsim || true; command -v livox_ros2_driver_node || true; command -v kobuki_node || true; echo '== packages =='; dpkg-query -W 2>/dev/null | grep -Ei 'mvsim|livox|kobuki' || true; echo '== libraries =='; find /opt /usr/local -type f \( -iname '*mvsim*' -o -iname '*livox*' -o -iname '*kobuki*' \) -print 2>/dev/null | sort || true" 2>&1 | Tee-Object "$Report/r0e-image-inventory.txt"

$SourceRoots = @('README.md', 'docs', 'ros_ws', 'sim', 'hsl_core', 'kobuki/workspace/src', 'docker')
Get-ChildItem -Path $SourceRoots -Recurse -File -ErrorAction SilentlyContinue | Where-Object { $_.Extension -notin '.pyc', '.so', '.a' } | Select-String -Pattern 'mvsim|livox|kobuki' -CaseSensitive:$false | ForEach-Object { "{0}:{1}:{2}" -f $_.Path, $_.LineNumber, $_.Line.Trim() } | Tee-Object "$Report/r0e-repository-references.txt"
```

Interpretación:

- Si aparece `mvsim`, envíe ambos informes; se preparará un smoke de versión y mundo mínimo sin GUI.
- Si no aparece, no instale un paquete por intuición. Indique la fuente autorizada (URL/commit de repositorio, paquete/versionado o imagen) para congelarla y añadirla al perfil acceptance.
- La presencia de Livox o Kobuki no autoriza abrir `/dev`, USB, serie ni red: R0-E solo registra versiones, ejecutables y dependencias.

## 9. R0-E — perfil MVSim exclusivo de pruebas

MVSim se instala **dentro** de `hsl26:simulation`, derivada de acceptance. No se instala en Windows ni se conecta por red al host. `hsl26:acceptance` y cualquier perfil de competición quedan sin MVSim. La descarga controlada requiere red una sola vez:

```powershell
$MvsimBundle = Join-Path $PWD.Path 'docker/r0/apt-mvsim'
if (Test-Path -LiteralPath $MvsimBundle) { if ((Get-ChildItem -LiteralPath $MvsimBundle -Force | Measure-Object).Count -gt 0) { throw "El bundle MVSim ya contiene archivos; use un run-id nuevo o revíselo." } }
New-Item -ItemType Directory -Force $MvsimBundle | Out-Null
docker run --rm --network bridge -v "$($PWD.Path):/workspace" -w /workspace --entrypoint /bin/bash hsl26:acceptance -lc "bash docker/r0/download_mvsim_apt_bundle.sh docker/r0/apt-mvsim" 2>&1 | Tee-Object "$Report/mvsim-apt-download.txt"
Get-Content docker/r0/apt-mvsim/packages.tsv | Tee-Object "$Report/mvsim-apt-packages.tsv"
Get-Content docker/r0/apt-mvsim/sha256sums.txt | Tee-Object "$Report/mvsim-apt-sha256.txt"
```

Solo si el bundle termina correctamente, construir y comprobar el perfil sin red:

```powershell
$AcceptanceImage = 'hsl26:acceptance'
$ExpectedAcceptanceId = 'sha256:3c8a8251f65805c486023c2fafdd7276668e523d6981cb8bd535ac5645aa231b'
$ActualAcceptanceId = (docker image inspect $AcceptanceImage | ConvertFrom-Json)[0].Id
if ($ActualAcceptanceId -ne $ExpectedAcceptanceId) { throw "La etiqueta local $AcceptanceImage no corresponde al digest R0-D esperado. Esperado: $ExpectedAcceptanceId; actual: $ActualAcceptanceId" }
docker build --pull=false --network none --progress=plain --build-arg ACCEPTANCE_IMAGE=$AcceptanceImage --build-arg REQUIRE_MVSIM_CACHE=0 -f docker/Dockerfile.simulation -t hsl26:simulation . 2>&1 | Tee-Object "$Report/mvsim-simulation-bootstrap-build.txt"
docker run --rm --network none --entrypoint /bin/bash hsl26:simulation -lc "source /opt/ros/humble/setup.bash && mvsim --version && ros2 pkg prefix mvsim && ros2 pkg executables mvsim" 2>&1 | Tee-Object "$Report/mvsim-version.txt"

# El demo oficial carga recursos remotos de forma diferida. Precárguelos una vez
# con red controlada, detenga el proceso cuando el mundo haya arrancado y congele
# el cache antes de reconstruir la imagen sin red.
$MvsimCache = Join-Path $PWD.Path 'docker/r0/mvsim-cache'
New-Item -ItemType Directory -Force $MvsimCache | Out-Null
docker run --rm --network bridge -e HSL26_SIMULATION_ENABLED=1 -e HOME=/cache -e ROS_LOG_DIR=/tmp/ros-log -v "${MvsimCache}:/cache" --entrypoint /usr/local/bin/hsl26-run-mvsim hsl26:simulation 2>&1 | Tee-Object "$Report/mvsim-resource-prefetch.txt"
Get-ChildItem -Path (Join-Path $MvsimCache '.cache/mvsim-storage') -Recurse -File | Get-FileHash -Algorithm SHA256 | Sort-Object Path | Format-Table -AutoSize | Out-File "$Report/mvsim-resource-sha256.txt"

docker build --pull=false --network none --progress=plain --build-arg ACCEPTANCE_IMAGE=$AcceptanceImage --build-arg REQUIRE_MVSIM_CACHE=1 -f docker/Dockerfile.simulation -t hsl26:simulation . 2>&1 | Tee-Object "$Report/mvsim-simulation-build.txt"
docker run --rm --network none -e HSL26_SIMULATION_ENABLED=1 --entrypoint /usr/local/bin/hsl26-run-mvsim hsl26:simulation 2>&1 | Tee-Object "$Report/mvsim-headless-smoke.txt"
```

El prefetch es reanudable: no borre el cache existente. Espere hasta que dejen de aparecer líneas `Downloading remote resources`; si el proceso continúa vivo durante al menos 30 segundos sin errores, el mundo ya está en ejecución y puede detenerlo con `Ctrl+C`. Si aparece otro recurso remoto faltante, ejecute el mismo prefetch otra vez: MVSim conservará los recursos ya descargados y continuará con el siguiente. Solo reconstruya la imagen con `REQUIRE_MVSIM_CACHE=1` cuando el prefetch haya arrancado sin error. El último smoke debe arrancar sin intentos de `wget`; deténgalo tras confirmar el arranque y conserve el log. GUI es solo una prueba independiente de desarrollo: requiere `HSL26_MVSIM_GUI=1`, `HSL26_SIMULATION_DEVELOPMENT_GUI_ACK=1`, un servidor gráfico Linux/WSLg/X11 y una política de display explícita; no se habilita ni documenta para el perfil de competición.

## 10. R0-F — SBOM y rebuild independiente

Genere un SBOM SPDX del perfil de simulación con la capacidad SBOM de Docker. Si el subcomando no está disponible, conserve el error y no sustituya el SBOM por una lista manual:

```powershell
# Tras reiniciar PowerShell, restaure las variables de la sesión antes de usar
# Docker Scout. Algunas versiones de Scout no crean el directorio temporal por
# digest en Windows; se crea explícitamente sin borrar su cache.
$Report = 'artifacts/reports/r0/r0_20261001T151427Z'
if (-not (Test-Path -LiteralPath $Report)) { throw "No existe el directorio de evidencia: $Report" }
$SimulationId = ((docker image inspect hsl26:simulation | ConvertFrom-Json)[0].Id -replace '^sha256:', '')
$ScoutImageTemp = Join-Path $env:TEMP "docker-scout/sha256/$SimulationId"
New-Item -ItemType Directory -Force $ScoutImageTemp | Out-Null
docker scout sbom --format spdx --output "$Report/simulation.spdx.json" local://hsl26:simulation
docker build --no-cache --pull=false --network none --progress=plain --build-arg ACCEPTANCE_IMAGE=$AcceptanceImage --build-arg REQUIRE_MVSIM_CACHE=1 -f docker/Dockerfile.simulation -t hsl26:simulation-r0f-rebuild . 2>&1 | Tee-Object "$Report/mvsim-simulation-rebuild.txt"
docker image inspect hsl26:simulation hsl26:simulation-r0f-rebuild | Out-File "$Report/mvsim-rebuild-image-inspect.json" -Encoding utf8
docker run --rm --network none -e HSL26_SIMULATION_ENABLED=1 --entrypoint /usr/local/bin/hsl26-run-mvsim hsl26:simulation-r0f-rebuild 2>&1 | Tee-Object "$Report/mvsim-rebuild-headless-smoke.txt"
```

Los dos digests de imagen pueden diferir por metadatos de build. R0-F exige que ambos builds carguen el mundo sin red y que el bundle apt, `requirements.lock` y `mvsim-resource-sha256.txt` sean idénticos; cualquier diferencia debe explicarse antes de cerrar R0.

Envíe el directorio completo de `$Report`, `docker/r0/requirements.lock` y el listado de hashes del wheelhouse. Indique además la plataforma que se usará en integración y en el NUC. Con esos datos se sustituirá la etiqueta mutable por el digest real y se preparará el Dockerfile `integration/acceptance` sin alterar las garantías de R1–R8.
