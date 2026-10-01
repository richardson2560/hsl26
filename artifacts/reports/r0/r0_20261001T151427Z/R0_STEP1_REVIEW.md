# Revisión R0, paso 1

**Estado:** `R0-A PASS`; no constituye aceptación R0 ni G0.

## Evidencia recibida

- Docker Desktop 4.92.0, Engine 29.8.0, contexto `desktop-linux`.
- Cliente Windows/amd64; servidor Docker Linux/amd64.
- Buildx `v0.37.1`.
- Imagen base resuelta por `docker buildx imagetools inspect`:
  `docker.io/nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619`.
- Captura local por el mismo digest: Linux/amd64, Ubuntu 22.04.5 y Python 3.10.12.
- ROS Humble está presente; `ros2 doctor --report` terminó con código 0 bajo red deshabilitada. Las advertencias de PackageReport/RosdistroReport se conservan como diagnóstico del modo aislado.

## Diagnóstico del capturador original

La primera versión capturaba la salida de `docker pull`, por lo que una descarga
inicial larga no daba progreso visible. Además, las sondas interiores no tenían
un tiempo máximo. No hay evidencia de que Docker o la imagen hayan fallado.

`tools/r0_capture_environment.py` se actualizó para mostrar el pull, limitar
cada sonda a 90 segundos por defecto y escribir el informe aunque una sonda
agote el tiempo. Un timeout queda registrado con `exit_code: 124`. La sonda de
colcon se corrigió: `colcon --version` no es un comando válido en esta versión;
la nueva sonda consulta el metadato instalado de `colcon-core`.

La captura también confirma que `python3 -m pip` falla porque la imagen base no
incluye pip. Esto no invalida R0-A: el Dockerfile de desarrollo instala pip en
una capa posterior. Sí bloquea R0.3 hasta que R0.2 registre la fuente y versión
apt de `python3-pip` y cree un resolver bootstrap explícitamente no aceptable.

## Estado de puerta

| Caso | Estado | Motivo |
| --- | --- | --- |
| R0-A | `PASS` | Digest remoto y local coinciden; plataforma Linux/amd64 y SO quedan capturados. |
| R0.2 | `PASS` | Bundle controlado capturado: `python3-pip=22.0.2+dfsg-1ubuntu0.7` y `python3-wheel=0.37.1-2ubuntu0.22.04.1`, con hashes. El resolver se construyó sin red y sin advertencias, como `hsl26@sha256:cb130f05ecd386619337f41dab05cb376253b01713ed425b1ec91f0daac702b5`; la captura confirma `pip`, ROS y colcon con código 0. |
| R0.3 | `PASS` | El lock final usa `setuptools 79.0.1` y `pytest 7.4.4`; el wheelhouse conserva 64 ruedas y verifica 64/64. Su SHA-256 es `a84132086fc027b569bc005d19c97d6c3a0cdfebcd2ea9586fc7747e3006a6fb`. La instalación con `--network none --no-index --require-hashes` terminó y `pip check` no reportó requisitos rotos. |
| R0-B | `PASS` | Con `setuptools 79.0.1`, el Dockerfile instala `hsl_core-0.1.0` de forma importable y `colcon build --symlink-install` termina los nueve paquetes HSL26. Las advertencias `tests_require` no son fallos de build. |
| R0-C | `PASS` | Con `pytest 7.4.4`, `hsl_interfaces/contract_schema_test` pasa y `colcon test-result --verbose` informa `Summary: 9 tests, 0 errors, 0 failures, 0 skipped`. Se ejecutó con `--network none`, sin mounts ni dispositivos. |
| R0-D | `PASS` | `hsl26:acceptance` se reconstruyó con el smoke corregido como `hsl26@sha256:3c8a8251f65805c486023c2fafdd7276668e523d6981cb8bd535ac5645aa231b` para Linux/amd64. El smoke, sin red, mounts ni dispositivos, cargó los tres entornos ROS e importó `hsl_core`/`rclpy`; no detectó procesos Kobuki, Livox, MVSim ni `cmd_vel_mux`. El digest anterior `1acd…` queda supersedido por esta reconstrucción. |
| R0-E | `PASS` | `ros-humble-mvsim=1.4.0-1jammy.20260908.093724` y sus dependencias se descargaron como bundle hash-verificado. El cache de recursos del mundo se congeló; `hsl26:simulation` se construyó sin red como `hsl26@sha256:bf6664fa055484a269d87e90e59967e29a5c886c66c2ba066e0dba75efb255c8`. El smoke sin red cargó `demo_warehouse.world.xml` hasta `World file load done.` sin `wget`. Kobuki/Livox no se iniciaron ni se abrieron dispositivos. Se observaron avisos de simulación más lenta que tiempo real; son baseline de rendimiento, no aprobación temporal. |
| R0-F | `PASS` | SBOM SPDX 2.3 generado (`2492` paquetes; SHA-256 `8d6f443f3ebb0192efe832596c7aeb6ff81cda9c82d1ba644cf99ef70bdb8a50`). El rebuild sin caché, a `hsl26@sha256:dae951d77402ef4487f39b054dcf2e08e10c63a7bcf88d264ad431a8d3de3089`, cargó el mundo sin red. Aunque el digest difiere del original `bf6664…` por metadatos de BuildKit, ambos inventarios runtime son idénticos, SHA-256 `1d60f17b5a8f25da97f5a7e9885295f0653b5b14690d3f81d4893e39b2a102ef`. |

## Conclusión técnica

R0-A–R0-F están en `PASS`. La baseline técnica queda formada por acceptance
`hsl26@sha256:3c8a8251f65805c486023c2fafdd7276668e523d6981cb8bd535ac5645aa231b`,
simulation `hsl26@sha256:bf6664fa055484a269d87e90e59967e29a5c886c66c2ba066e0dba75efb255c8`,
el lock Python `a84132086fc027b569bc005d19c97d6c3a0cdfebcd2ea9586fc7747e3006a6fb`,
el cache MVSim hash-verificado y el SBOM SPDX. Antes de declarar R0 global `PASS`,
un revisor distinto del autor debe comprobar esta evidencia y firmar la puerta.

## Avance autorizado

El procedimiento de captura apt y construcción sin red de
`hsl26:r0-wheel-resolver` está en `docker/r0/README.md`, sección 2. La captura
`wheel-resolver-capture.json` confirma `pip.exit_code == 0`; queda autorizada la
descarga del wheelhouse de la sección 3.

La primera construcción funcional emitió una advertencia de linter
`InvalidDefaultArgInFrom`: el argumento `BASE_IMAGE` no tenía valor por defecto
aunque la llamada lo suministraba. Se corrigió el Dockerfile con el digest R0-A
como default, sin impedir su sobrescritura explícita. Debe reconstruirse una vez
para dejar evidencia sin esa advertencia.

## Incidencia R0.3 — descarga PyPI

La primera descarga del wheelhouse agotó el timeout de lectura de pip durante
`numpy-2.2.6` desde `files.pythonhosted.org`; no es un fallo de resolución ni
un artefacto válido. El procedimiento usa ahora `--retries 5 --timeout 300` y
sin barra de progreso. Solo se continúa si la descarga concluye y el verificador
del lock confirma todas las ruedas.

La reanudación posterior completó la descarga, incluyendo `open3d-0.20.0` para
CPython 3.10/Linux. Esto fija la resolución para este perfil; no acredita aún
que pip pueda instalarla sin red ni que el workspace ROS compile.
