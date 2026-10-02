# HSL26 — Plan de implementación ROS 2 y MVSim

**Propósito:** hacer ejecutable la arquitectura de `HSL26_FINAL_ARCHITECTURE.md` §§4–5, 10–11 y `HSL26_TECHNICAL_SPECIFICATION.md` §§3–6, 13–14. **Estado inicial:** ROS 2 tiene IDL y adaptadores parciales, pero la mayoría de nodos son esqueletos; `sim/mvsim/adapter/simulation_adapter.py` carece de lógica y los launch de perfiles no inician la cadena.

## 1. Contrato extremo a extremo

```text
MVSim/driver → sensores propios → ego/obstáculos/mapa/topología/track
→ estado de etapa + opciones → acción ExecuteOption → ruta y MotionCandidate
→ SafetySupervisor → /hsl/cmd_vel_final → cmd_vel_mux → planta
                 ↑                 ↓
       watchdog independiente ← heartbeat/health
```

Cada flecha conserva `ContractHeader` revisión 2: fuente/sesión/secuencia, etapa, epochs, frame, observación/estado/publicación/expiración, versiones y validez. El tiempo ROS rige observaciones y etapa; el tiempo steady local rige watchdog y leases. Saltos de `/clock` invalidan estado temporal. La interfaz `ExecuteOption.action` y los mensajes existentes son el contrato único; cambios de campos requieren versión y migración explícitas.

## 2. Paquetes de trabajo

| ID | Implementación y archivos principales | Prueba de salida |
| --- | --- | --- |
| R0 — entorno | Congelar y medir el entorno de integración antes de modificar nodos: imagen base por digest, paquetes apt, artefactos Python, ROS Humble, fuentes/commit de drivers y MVSim. Separar la imagen de desarrollo de una imagen de aceptación instalable sin red; la segunda no usa `pip -e` ni `--symlink-install`. | R0.1–R0.8 completos; build limpio y prueba desde imagen congelada, inventario SBOM/hash y evidencia reproducible. Hasta entonces, estado `BLOCKED_NOT_RUN`, no G0. |
| R1 — inventario de grafo | Documentar tópicos, tipos, QoS, TF owner, rates y namespace por robot; implementar verificador de grafo **en vivo**, además de `tools/check_ros_graph_authority.py` sobre JSON. | Un único escritor de `/commands/velocity`, cero escritores ajenos y TF sin duplicados; prueba de QoS incompatible y reinicio de fuente. |
| R2 — seguridad ejecutable | Completar `hsl_safety/supervisor_node.py` y `watchdog.py`: subscriptions, cachés validadas, timers en procesos separados, heartbeat posterior a publicación, rearm autorizado, salida cero antes de inputs y en excepción. Conectar `cmd_vel_mux` heredado y timeout del driver. | I01–I05: arranque en cero, parada prioritaria, kill de planner/supervisor/watchdog/mux, lease vencido y recuperación medida. |
| R3 — observación/estado | Implementar normalización Livox/IMU/odom, deskew, nube→obstáculos/cobertura, fusión de odometría, mapa y topología en `hsl_perception`/`hsl_world`. Usar adaptadores puros y validar extrínsecos. | Rechazo de datos viejos, no retornos y zonas ciegas; epochs y transformaciones; comparación con bags y fixtures T07/T20/T22/T23. |
| R4 — oponente | Conectar segmentación/registro GPIS, EKF, asociación y creencia topológica con worker acotado. Publicar origen, covarianza, yaw válido y edad; registrar fallos sin inventar detección. | I10/G3 con rival parado/móvil/oculto, falsos positivos, error de origen, calibración e inferencia dentro del presupuesto CPU. |
| R5 — decisión/match | Implementar `stage_manager_node.py`, `tactics_node.py` y cliente de acciones. Zonas con procedencia; `INIT/FREEZE/ACTIVE/TERMINAL`; reintentos idempotentes y cancelación. Cargar duración autorizada de 10 minutos y preparación de 4 minutos desde perfil versionado; el cero durante freeze es la política conservadora HSL26. Separar reset técnico del único reinicio oficial permitido por ambos capitanes u organizadores. | I04/I11–I13: freeze, metadata ausente, carrera cancel/reemplazo, intervalos de evento y reset sin comandos viejos; dos etapas con cambio de rol y sin reinicio oficial inventado. |
| R6 — navegación | Crear el `option_executor_node.py` requerido por la especificación; conectar A*, realización de rutas, control local, `MotionCandidate` y `ExecutionState`. La navegación jamás escribe el comando final. | Cambio de mapa/lease cancela ruta; sin corner cutting, obstáculo dinámico, control de giro y replanificación I09/T21/T29. |
| R7 — bringup | Sustituir avisos en `hsl26.launch.py`, `sim_mvsim.launch.py`, `real_hardware.launch.py` y `replay_bag.launch.py` por composición real, validación de modo/rol/robot y start disarmed. | `ros2 launch ... mode:=sim/real/replay` crea exactamente los procesos previstos; bag/replay sin dispositivo ni comando físico. |
| R8 — adaptador MVSim | Implementar puertos separados de observación, actuador admitido, control de episodio y verdad árbitro. Adaptar mensajes y `/clock` de la versión fijada; configurar dos robots y mundos versionados. | Match determinista con dos namespaces; políticas sin `/referee/*` ni odom verdadero rival; grafo inspeccionado en vivo I14. |

R2 y R3 pueden desarrollarse antes de R8. El primer movimiento simulado no autoriza movimiento real. R4 puede empezar con bags/sensores sintéticos mientras se caracteriza MVSim.

### R3-A — puerta de observación normalizada

**Estado: `PASS` técnico de núcleo; `IN_PROGRESS` para runtime ROS.**
`hsl_core.perception.observation_gate` admite una nube sólo si su frame,
clock/localization epochs, `calibration_id`, stamps por punto y edad coinciden
con el contexto receptor. Después aplica deskew y `LocalObstacleBuilder`; un
dato incompleto, futuro, stale o incoherente es rechazo, nunca cobertura libre.
Las pruebas de puerta, deskew y mapping se ejecutan en R3-A; los comandos y
la evidencia Docker están en `docker/r3/README.md`. R3-B permanece bloqueado
hasta inventariar el contrato real Livox/IMU/odom y una calibración aceptada.

## 2.1 R0 — línea base reproducible y segura

**Objetivo de R0:** producir una base de ejecución que otra persona pueda reconstruir, inspeccionar y arrancar sin interpretar etiquetas mutables ni descargar dependencias durante la validación final. R0 no implementa seguridad, grafo, MVSim ni movimiento. Su salida solo habilita la ejecución fiable de R1–R8.

**Estado R0 a 2026-10-01:** se resolvió la imagen heredada para Linux/amd64 por el digest `nickodema/kobuki@sha256:2034fe21fbe8aae8677191967ec9c4fc168a1337ada7bb1913a642b92f2b8619` (R0-A `PASS`). La base Ubuntu 22.04.5 no contenía pip; se registraron sus fuentes apt, se descargó un bundle hash-verificado de `python3-pip=22.0.2+dfsg-1ubuntu0.7` y `python3-wheel=0.37.1-2ubuntu0.22.04.1`, y se construyó sin red el resolver bootstrap local `hsl26@sha256:cb130f05ecd386619337f41dab05cb376253b01713ed425b1ec91f0daac702b5` (R0.2 `PASS`). R0.3 `PASS`: el wheelhouse Linux/CPython 3.10 conserva 64 ruedas, usa `setuptools 79.0.1` y `pytest 7.4.4`, verifica 64/64 y su lock final tiene SHA-256 `a84132086fc027b569bc005d19c97d6c3a0cdfebcd2ea9586fc7747e3006a6fb`; su instalación sin red aprobó `pip check`. R0-B y R0-C son `PASS`: `hsl_core-0.1.0` se instala/importa, los nueve paquetes HSL26 compilan y `colcon test-result --verbose` informa `9 tests, 0 errors, 0 failures, 0 skipped` con red deshabilitada. R0-D es `PASS`: la imagen separada reconstruida `hsl26@sha256:3c8a8251f65805c486023c2fafdd7276668e523d6981cb8bd535ac5645aa231b` se validó sin red, mounts ni dispositivos, cargó sus entornos y no inició procesos de drivers, MVSim o movimiento. R0-E está en implementación con el perfil simulation aislado; falta validar su bundle y smoke. R0-F (SBOM y rebuild independiente) sigue pendiente; por ello R0 global permanece abierto y no modifica el estado de pruebas SIL.

### R0.1 — preservar el punto de partida y separar perfiles

1. Registrar el `HEAD` de Git, `git status --porcelain`, sistema anfitrión, versión de Docker/BuildKit, arquitectura, fecha UTC y el comando exacto de cada ejecución.
2. Conservar la imagen actual únicamente como perfil `development`; no se etiqueta ni se emplea como `release`, `competition` o evidencia G0.
3. Definir perfiles inequívocos:
   - `development`: puede montar el checkout y usar enlaces simbólicos para iteración; sus resultados no son evidencia de release.
   - `integration`: imagen autocontenida para `colcon build`, pruebas y lanzamientos MVSim; todas las entradas están identificadas.
   - `acceptance`: derivada del artefacto `integration` identificado por digest; se ejecuta sin red y sin montar código, y es la única candidata para I01–I14.
4. Prohibir que un cambio de perfil, argumento de build, arquitectura o fuente de paquetes reutilice el mismo identificador de evidencia.

### R0.2 — congelar la procedencia de la imagen y del sistema

1. Resolver la imagen heredada a una referencia `repository@sha256:<64-hex>` para la arquitectura objetivo. Registrar tanto el digest de índice como el digest de manifiesto de plataforma; una etiqueta por sí sola falla R0.
2. Registrar el digest/versión de Dockerfile heredado, el sistema operativo de la imagen, `dpkg-query -W`, fuentes apt, claves, arquitectura y el resultado de `ros2 doctor --report`.
3. Fijar cada paquete apt por versión y origen de repositorio o usar un snapshot fechado verificable. Si la base ya contiene una dependencia (Livox-SDK2, Kobuki, ecl o ROS), inventariarla y comprobarla; no reinstalarla implícitamente.
4. Registrar commit, remoto y licencia de `kobuki/workspace/src`, `livox_ros_driver2` y MVSim. Un árbol copiado localmente sin commit verificable se declara `UNVERIFIED_LOCAL_SOURCE`, no “upstream pinned”.
5. Rechazar imágenes multi-arquitectura cuando la plataforma seleccionada no está declarada. El NUC objetivo y la CI deben construir para la misma plataforma o llevar evidencias separadas.

### R0.3 — dependencias Python y artefactos sin red

1. Sustituir rangos abiertos de dependencias de la imagen por un lock de resolución que contenga nombre, versión exacta, URL/origen, SHA-256 de cada distribución y marcador de plataforma/Python. `hsl_core/pyproject.toml` puede conservar rangos de compatibilidad para desarrollo; el lock de imagen es la autoridad de aceptación.
2. Descargar en una operación controlada un wheelhouse para la plataforma objetivo, generar su manifiesto con hashes y guardarlo como entrada de build. La fase `acceptance` instala exclusivamente desde ese wheelhouse con `--no-index --require-hashes`.
3. Instalar `hsl_core` como wheel versionado en `integration`/`acceptance`; prohibir `pip install -e` allí. La imagen de desarrollo puede usar editable de forma explícita y aislada.
4. Tratar `open3d` como dependencia opcional de visualización o fijarla a un wheel disponible para la plataforma; no permitir que su ausencia bloquee tests de contratos/seguridad que no la importan.

### R0.4 — construcción ROS reproducible

1. Construir primero el workspace heredado y después `ros_ws`, siempre desde árboles copiados con hashes. Usar instalaciones normales para `integration`/`acceptance`; `--symlink-install` queda limitado a desarrollo.
2. Limpiar `build/`, `install/` y `log/` dentro del contenedor antes de cada build de evidencia. Ejecutar `colcon build --event-handlers console_direct+` con el mismo conjunto de paquetes y argumentos registrados.
3. Verificar generación de todas las interfaces de `hsl_interfaces` y que cada paquete declarado por `colcon list` se resuelve sin dependencia ausente. `colcon test` y `colcon test-result --verbose` deben ejecutarse después del build, no inferirse de imports Python del anfitrión.
4. Publicar un SBOM CycloneDX o SPDX, hashes de los workspaces instalados y `pip check`. El SBOM identifica componentes; no sustituye pruebas de comportamiento.

### R0.5 — MVSim y drivers: caracterizar antes de integrar

1. No añadir MVSim al Dockerfile por nombre supuesto. Primero capturar cómo se instala (paquete, fuente o contenedor), versión/commit, bibliotecas enlazadas, comandos disponibles y licencia.
2. Ejecutar un smoke sin robot: `mvsim --version` o el comando real documentado, carga de un mundo mínimo sin GUI y registro de salida. Si no hay API ROS 2 compatible, R8 deberá usar un adaptador de proyecto y R0 lo anota como restricción, no como fallo silencioso.
3. Verificar que el driver Livox y Kobuki se compilan contra las bibliotecas esperadas y que sus dispositivos no se abren durante build, test ni modo `replay`.

### R0.6 — controles de seguridad de la imagen

1. La imagen de aceptación arranca desarmada y no inicia automáticamente drivers, `cmd_vel_mux` ni ningún nodo de movimiento; R2/R7 demostrarán el encadenamiento posterior.
2. La ejecución de validación usa red deshabilitada después de construir la imagen, sin `--privileged`, sin dispositivos y sin montaje del checkout. El acceso a hardware solo aparece en el perfil real posterior, mediante una configuración explícita.
3. Secretos, credenciales de registro y configuraciones de red no se copian a capas ni se escriben en SBOM, logs o manifiestos.
4. Escanear vulnerabilidades es diagnóstico versionado: toda excepción requiere responsable, CVE, alcance, mitigación, fecha de expiración y decisión explícita; un escaneo limpio no certifica seguridad funcional.

### R0.7 — matriz de aceptación y evidencia

| Caso | Procedimiento mínimo | Resultado exigido | Estado inicial |
| --- | --- | --- | --- |
| R0-A | Resolver y registrar imagen por digest y plataforma | Referencias inmutables y verificables | `BLOCKED_NOT_RUN` |
| R0-B | Construir `integration` desde checkout limpio | `colcon build` y rosidl correctos | `BLOCKED_NOT_RUN` |
| R0-C | Ejecutar tests ROS dentro de la imagen | `colcon test` + `test-result --verbose` sin fallos | `BLOCKED_NOT_RUN` |
| R0-D | Arrancar `acceptance` con red deshabilitada y sin mounts | Shell/entorno ROS cargan; no hay descargas ni procesos de movimiento | `BLOCKED_NOT_RUN` |
| R0-E | Inspeccionar MVSim y drivers | versión/API/fuentes registradas; sin apertura de dispositivo | `BLOCKED_NOT_RUN` |
| R0-F | Rebuild independiente con los mismos insumos | mismos hashes de paquetes/instalación, o diferencias explicadas | `BLOCKED_NOT_RUN` |

Cada caso genera `artifacts/reports/r0/<run-id>/` con comando, salida completa, UTC, plataforma, digest de imagen, hashes de contexto/lockfiles, SBOM, resultados `colcon`, y revisor. Un `BLOCKED` debe indicar el insumo faltante; un `FAIL` conserva los logs y no se convierte en `PASS` mediante reintento sin diagnóstico.

La automatización inicial queda versionada en `tools/r0_capture_environment.py` y `docker/r0/`: el primero captura la imagen local sin red ni drivers; `requirements.in` se resuelve dentro de dicha imagen y los scripts generan/verifican un lock exacto contra el wheelhouse. El procedimiento PowerShell, incluidos comandos de captura, descarga controlada, instalación sin red y build de desarrollo, está en `docker/r0/README.md`. Los artefactos producidos se revisan antes de añadirlos como inputs de una imagen `integration`/`acceptance`.

### R0.8 — decisión de puerta y traspaso

**Cierre técnico a 2026-10-02:** R0-A–R0-F tienen evidencia `PASS`. La simulación
MVSim se ejecuta headless y sin red desde un cache de recursos hash-verificado; el
rebuild sin caché también carga el mundo y su inventario runtime coincide con el
original (SHA-256 `1d60f17b5a8f25da97f5a7e9885295f0653b5b14690d3f81d4893e39b2a102ef`).
Los digests de imágenes difieren solo por metadatos de BuildKit y quedan
registrados en la evidencia R0. La puerta global sigue pendiente de revisión por
una persona distinta del autor, como exige esta sección.

R0 pasa a `PASS` únicamente si R0-A–R0-F están aprobados por un revisor distinto del autor y la imagen `acceptance` se identifica por digest. La ausencia de Docker, de acceso al registro, de digest base, de wheelhouse o de una fuente MVSim verificable deja R0 en `BLOCKED_NOT_RUN`; no autoriza sustituirlos con números supuestos. Al pasar R0, se congelan el digest de imagen, locks, SBOM, commit y configuración de build como baseline de R1. Cualquier cambio en ellos invalida R0-B–R0-F y la evidencia ROS/MVSim que dependa de esa imagen.

## 2.2 R1 — inventario y autoridad del grafo

**Cierre técnico a 2026-10-02: `PASS`.** El run
`artifacts/reports/r1/r1_20261002T052131Z/` validó el contrato SHA-256
`7a9da872a358d0e22d26169880e81f3a2cda05591d623db15751b5870c55b2d1` contra
la imagen `hsl26@sha256:aa6852929a8f26eafec66f47991139466de550c614398447bb54711939f9011f`.
El mux fue el único escritor físico antes y después de reiniciarse; los
fixtures de segundo escritor, QoS incompatible y TF duplicado fueron
rechazados; las aristas TF estáticas válidas aprobaron. R1 no activa el robot
ni implementa la seguridad de R2, y no acredita aún el grafo de dos robots de
R8. La decisión formal de puerta sigue pendiente de revisión independiente.

### R1.1 — congelar el registro de grafo

1. `ros_ws/src/hsl_bringup/config/ros_graph_contract.json` es el inventario
   legible por máquina de los tópicos canónicos: nombre robot-relativo, tipo,
   productor semántico único, QoS y tasa objetivo. La tabla §5 de
   `HSL26_TECHNICAL_SPECIFICATION.md` continúa siendo normativa si hay una
   discrepancia; cualquier cambio se hace en ambos documentos y aumenta la
   revisión del contrato.
2. Registrar las aristas `map→odom→base_link→{lidar_link,imu_link}` y su
   propietario semántico único. Un frame duplicado, un padre múltiple o un
   broadcaster no declarado es `FAIL`; no se corrige con remapping.
3. Para dos robots, prefijar todos los nombres canónicos bajo `/guardian` o
   `/explorer`; nunca compartir `/commands/velocity`, `/tf` ni un frame hijo.
   La variante de un solo robot conserva los nombres raíz de la tabla.
4. El único escritor físico es `/cmd_vel_mux` sobre
   `/commands/velocity`. Supervisor, watchdog y teleoperación sólo escriben
   sus entradas del mux. R1 no autoriza que ningún fixture publique comandos
   de movimiento no nulos.

### R1.2 — verificador vivo y pruebas negativas

1. `hsl_graph_verifier` usa la API de grafo de `rclpy`, no texto parseado de
   `ros2 node list`. Captura publicadores, suscriptores, tipos y QoS y escribe
   un JSON de evidencia. Falla si el tópico físico no tiene exactamente el
   mux declarado como único publicador o si detecta una pareja QoS incompatible.
2. Mantener `tools/check_ros_graph_authority.py` como control estático de
   fixtures JSON; no sustituye al verificador vivo.
3. La prueba positiva inicial levanta sólo `cmd_vel_mux` en un contenedor
   `--network none`, sin base ni sensores. La prueba negativa añade un
   publicador de fixture, espera fallo del verificador y conserva ese log. No
   se usa un driver o robot físico para provocar el fallo.
4. Reiniciar el único proceso fuente, esperar descubrimiento DDS y repetir la
   captura. Debe conservarse exactamente un escritor físico y no debe aparecer
   otro por el reinicio. El verificador observa `/tf_static` con QoS durable y
   rechaza aristas duplicadas, padres múltiples o aristas de calibración
   ausentes. Como TF no transporta la identidad del nodo emisor, la atribución
   de propietario se revisa contra el launch y el contrato firmado; no se
   infiere del nombre del frame.

### R1.3 — criterios de salida

| Caso | Evidencia requerida | Resultado para aprobar |
| --- | --- | --- |
| R1-A | hash del contrato, tipos/tópicos/QoS/TF y namespaces revisados | registro coincide con §5 y no contiene alias de autoridad |
| R1-B | captura viva positiva con mux aislado | sólo `/cmd_vel_mux` publica `/commands/velocity` |
| R1-C | fixture de segundo publicador y fixture QoS incompatible | ambos son detectados y el verificador termina en `FAIL` esperado |
| R1-D | captura antes/después de reiniciar fuente y reporte TF | no hay escritor residual ni arista TF duplicada |

Los comandos de ejecución y la forma de conservar los reportes están en
`docker/r1/README.md`. R1-A–R1-D tienen evidencia técnica en
`artifacts/reports/r1/r1_20261002T052131Z/`; la revisión independiente sigue
pendiente. R2 puede desarrollar y probar su límite de parada sin habilitar
movimiento; R7 y cualquier nodo que permita movimiento continúan fuera de
alcance.

## 2.3 R2 — límite supervisor/watchdog, cerrado por defecto

**Estado de implementación: `IN_PROGRESS`; R2-A (arranque desarmado) tiene
`PASS` técnico, sin autorización de movimiento.**

R2 materializa el límite de proceso de seguridad sin reutilizar como
calibración los valores orientativos de `hardware.yaml`. El perfil
`r2_no_motion.yaml` declara explícitamente `UNCONFIGURED_NO_MOTION`: el
supervisor publica sólo `Twist()` nulo en `/hsl/cmd_vel_final`; el watchdog,
en otro proceso, publica periódicamente `Twist()` nulo en
`/hsl/cmd_vel_stop`. El watchdog arranca desarmado, supervisa secuencia,
identidad, tiempo steady y lease del latido, y niega todo rearme mientras no
exista evidencia de velocidad física y recuperación calibrada.

La orden de publicación del supervisor es comando final, estado y sólo
entonces latido. Si falla esa secuencia, no se publica latido y el watchdog
conserva la parada. `WatchdogHealth` usa su IDL R2 real (`healthy` y
`stop_asserted`); los alias históricos no son autoridad válida.

Los comandos reproducibles, criterios PASS/FAIL y dónde guardar la evidencia
están en `docker/r2/README.md`. El cierre de R2 requiere, además de esas
pruebas de software, I01–I05 contra mux y downstream reales, medición de
latencias/timeout y revisión independiente. R2 por sí solo no acredita base
física, MVSim, ni competición.

La evidencia R2-A está en `artifacts/reports/r2/r2_20261002T104405Z/`: imagen
`hsl26@sha256:750ec9f4f99de9b74007285f3cbdc42204830238bb12ddabfc12404c819567df`,
39 pruebas unitarias aprobadas, ambos procesos vivos con `--network none`,
dos capturas `Twist` nulas, `WatchdogHealth.stop_asserted=true` y rechazo
explícito de `RearmSafety`. Esta evidencia no cubre el mux ni una planta y no
se puede extrapolar a I01–I05.

R2-B tiene `PASS` técnico en
`artifacts/reports/r2/r2b_20261002T140144Z/`: la imagen
`hsl26@sha256:6a16bc85fee5950f5f864e3f6534a8b97073c32fb79f4fbbd57bd3540d0f7a68`
ejecutó 61 muestras de `/commands/velocity` en cero frente a una inyección
lógica de `0.2 m/s`, y el único escritor físico fue `/cmd_vel_mux`. El probe
exige tanto `--allow-test-nonzero` como `HSL26_TEST_ONLY=1`; no forma parte de
un launch de producción.

R2-C tiene `PASS` técnico en el mismo run: tras terminar el supervisor, el
watchdog y mux continuaron vivos, 61 muestras físicas lógicas siguieron en
cero y `WatchdogHealth` pasó a `healthy=false`, `stop_asserted=true`,
`STALE_HEARTBEAT`. R2-D debe tratar la caída del propio watchdog como una
limitación de arquitectura a cerrar con downstream/interlock independiente;
en R2-D, al perder el watchdog, el mux dejó pasar `0.2 m/s` en su salida
lógica. Por tanto, R2 no puede cerrar I01–I05 ni autorizar hardware hasta que
un controlador/downstream independiente haya sido diseñado, integrado y
medido; un mux ROS no puede demostrar por sí solo esa detención física.

## 3. Caracterización obligatoria de MVSim y MID-360

Antes de diseñar el puente, registrar commit/paquete/API instalados, tipos y frames emitidos, frecuencia, timestamps, latencia, QoS, número/distribución de rayos, alcance, ángulos verticales, zonas ciegas, intensidad y comportamiento ante obstáculos bajos. Comparar con documentación y, cuando existan, bags reales MID-360. Repetir pruebas con el robot en giro y con un rival parcialmente oculto. La salida se clasifica:

- `FULL_3D_CANDIDATE`: nube y tiempos suficientes para ensayar GPIS, aún pendiente validación de fidelidad.
- `PLANAR_OR_GENERIC_3D`: útil para grafo/control y algunas regresiones; reconocimiento GPIS no acreditado.
- `INSUFFICIENT`: usar MVSim solo para dinámica/grafo ROS; validar GPIS con bags y robot.

No modificar el modelo de GPIS para compensar silenciosamente una simulación deficiente. El modelo y las tácticas tienen artefactos/versiones separados.

## 4. Dos robots y aislamiento

Para Guardian y Explorer, declarar prefijos de nodos, frame IDs, remappings, reloj compartido y fuentes de odometría independientes. El árbitro recibe verdad de ambas plantas; cada política recibe solo su observación, `MatchState` autorizado y creencia estimada del rival. Un test debe intentar suscribirse desde la política a verdad del árbitro/otro robot y fallar por grafo y por configuración. Etiquetar cualquier perfil con mapa previo aprobado; `oracle` se reserva para ablación y nunca para entrenamiento/promoción.

## 5. Verificación escalonada

1. **Estática/build:** IDL y paquetes compilan; tests puros y rosidl; análisis de imports y dependencias de seguridad; configuración rechaza frames, hashes y modos inválidos.
2. **Grafo sin movimiento:** lanzar procesos, inspeccionar productores/suscriptores/QoS/TF; sin sensores o etapa, comandos cero; inyectar relojes y mensajes viejos.
3. **Loop MVSim mínimo:** un robot, ruta corta, obstáculo, stop, reinicio y traza comando nominal→admitido→mux→planta.
4. **Dos robots:** roles intercambiados, conflicto de acciones, fin de etapa y aislamiento de verdad; comparar estados/decisiones con una fixture cinemática equivalente dentro de tolerancias definidas.
5. **Fidelidad/performance:** I06–I10, latencia p50/p95/p99/peor observada, dropped samples, uso CPU/memoria/temperatura y tiempo de parada; no llamar WCET a un máximo observado.
6. **Hardware restringido:** calibración de huella, frenado, cobertura y timeout bajo condiciones declaradas; arranque por SSH desde el PC autorizado, sin hardware adicional ni edición fuera de la ventana permitida; G0/G1/G2/G3 solo se aceptan con sus propias evidencias físicas.

## 6. Criterio de finalización

Esta línea termina cuando un `colcon build/test` limpio y un lanzamiento MVSim de dos robots producen trazas completas sin verdad cruzada, con seguridad/mux y fallos de proceso validados; los límites de sensor se declaran. El modo real exige además aceptación física G0–G3 y autoridad del organizador. Si la caracterización no demuestra nube 3D adecuada, MVSim queda aprobado solo para los aspectos que sí mide y GPIS conserva una puerta de bags/hardware pendiente.
