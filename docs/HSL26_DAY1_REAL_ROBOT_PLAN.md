# HSL26 — Plan de trabajo con robot real desde el día 1

**Fecha:** 2026-10-02  
**Estado de partida:** robot Turtlebot 2 + Kobuki + Livox MID-360 disponible,
laberinto pequeño, estático y simétrico conocido durante la preparación. Este
documento organiza pruebas reales **sin MVSim**, pero no transforma pruebas de
software o simulación en aceptación física.

## 1. Decisión operativa

Se puede empezar hoy con el robot real en dos carriles distintos:

| Carril | Se puede iniciar | No autoriza |
| --- | --- | --- |
| Inventario y percepción pasiva | Inmediatamente, con los drivers oficiales y sin publicar movimiento | Que HSL26 mueva la base |
| Movimiento restringido | Sólo tras R2-H, R7-H y G0 físico aprobados abajo | Navegación autónoma, captura o competición |
| Detección de obstáculos/cobertura | Tras inventario de LiDAR; primero detenido y luego en movimiento restringido | Detección fiable del rival |
| Detección/seguimiento del rival | Tras R4-H y evidencia con el otro robot real | Decisión táctica o persecución autónoma |

**Conclusión:** R2, tal como está, no habilita movimiento. Es un perfil
intencionadamente cerrado (`UNCONFIGURED_NO_MOTION`) que publica sólo ceros y
deniega `RearmSafety`. El primer movimiento no depende de terminar R3--R8,
pero sí de convertir R2 en una cadena de parada física demostrada y de crear
el bringup real mínimo de R7. MVSim puede quedar fuera de las pruebas HIL
iniciales; no se pueden omitir los gates físicos.

## 2. Estado comprobado del checkout

| Elemento | Estado | Consecuencia |
| --- | --- | --- |
| Entorno R0 y verificador R1 | Evidencia técnica existente; R1 comprobó el mux como único escritor de `/commands/velocity` en contenedor aislado | Es una buena base estructural, no una prueba de la base física |
| R2 supervisor/watchdog | `IN_PROGRESS`, perfil no-motion | Mantener desarmado en hardware hasta implementar el perfil calibrado y sus fallos reales |
| `execution_profiles.yaml` | `real.enabled: false`, `allow_motion: false`, gate `BLOCKED_FOR_HARDWARE` | Ningún cambio manual de esos booleanos es una autorización |
| `real_hardware.launch.py` y `hsl26.launch.py` | Scaffolds: no arrancan drivers, mux ni cadena HSL26 | Deben sustituirse antes de usar `mode:=real` |
| Drivers heredados | Fuentes de Kobuki, `cmd_vel_mux` y Livox disponibles en `kobuki/` | Sirven para inventario y como componentes del futuro launch real |
| Límites de `hardware.yaml` | Valores provisionales | No usarlos como calibración de velocidad, giro o frenado |
| Lógica P3--P6 | Mayormente SIL/kinemática; no aceptación ROS/física | No desplegar navegación, táctica ni aprendizaje en el robot aún |

El árbol de trabajo está modificado. Cada sesión física debe registrar `git
rev-parse HEAD`, `git status --short`, hashes de configuración y el ID de la
imagen para no mezclar evidencia de checkouts distintos.

## 3. Reglas y parámetros de competición que mandan el diseño

Las reglas recibidas el día 1 tienen prioridad sobre las hipótesis anteriores.
El reglamento general y las reglas adicionales establecen:

1. El laberinto es estático durante el evento y se conoce el primer día, pero
   sus obstáculos no deben codificarse en el software si representan rasgos
   previamente desconocidos. Se permite una representación generada por
   observación y una configuración de ensayo versionada; se prohíben
   coordenadas/atajos hard-coded derivados del laberinto.
2. Cada enfrentamiento tiene dos etapas simétricas de 10 minutos, con 4
   minutos iniciales de `FREEZE`: no se cruza la línea de salida. Al terminar
   `FREEZE`, el lanzamiento puede ser manual o automático.
3. Una etapa termina al alcanzar el área inicial del guardian, al ser capturado
   el explorer o por tiempo. Sólo hay un reinicio completo, solicitado por
   ambos capitanes o por organización. Parar por decisión del capitán concede
   al rival su máximo puntaje.
4. La captura oficial exige simultáneamente distancia entre centros menor que
   0.45 m, eje X del guardian con desviación de como máximo 45 grados respecto
   al explorer y línea de visión sin obstáculo. Una estimación local puede
   frenar el robot, pero no declarar marcador oficial.
5. Las reglas adicionales dicen que un equipo conserva su color/rol durante
   un partido y que cualquiera de las dos posiciones de inicio puede tocarle.
   La política debe ser invariante al espejo y debe inferir/recibir la posición
   inicial; no puede fijar un único lado del laberinto.
6. Las fórmulas adicionales usan 360 s: para explorer no capturado,
   `EXP = 360 + 0.3*t_mark`; para guardian que captura,
   `GRD = 468 - 0.3*t_catch`. No se deduce todavía el significado preciso de
   `t_mark` ni si reemplazan o complementan los 10 minutos del reglamento.

### Inconsistencia que debe cerrarse con organización

[`score_profile.yaml`](../ros_ws/src/hsl_bringup/config/score_profile.yaml)
declara correctamente `official_values_loaded: false`, aunque contiene
`stage_total_s: 600`, `freeze_s: 240`, `active_s: 360`. El reglamento sí
confirma 10 min + 4 min de freeze; las fórmulas aportadas contienen 360 s.
Antes de implementar puntaje, pedir al organizador por escrito:

- definición y reloj origen de `t_mark` y `t_catch`;
- si 360 s es duración de puntuación, duración de etapa, o una constante de
  una fórmula que convive con la etapa de 600 s;
- definición geométrica/pose de la zona inicial objetivo y quién emite el
  evento oficial;
- si la asignación de color fija también fija explorer/guardian, y cómo se
  anuncia la posición inicial;
- mecanismo permitido de lanzamiento manual tras `FREEZE`.

Hasta recibirlo, mantener el perfil como no oficial y no publicar resultados
numéricos de puntuación.

## 4. Secuencia física de día 1 (sin movimiento)

### H0 — preparación y conexión

**Objetivo:** alcanzar el NUC usando el PC autorizado y comprobar inventario,
sin ejecutar HSL26 ni publicar `Twist`.

1. Registrar número del robot, operador, batería/estado, espacio disponible y
   autorización del organizador. Cargar Kobuki con su cargador y Rombica con
   su fuente; no intercambiar sus alimentaciones.
2. Encender en orden: Kobuki, Rombica, NUC. Esperar el arranque y entrar por
   SSH usando exclusivamente el host/credencial que entregue organización.
3. Localizar el checkout oficial preinstalado en `/home/tb*`; no sustituirlo
   con este checkout de Windows ni ejecutar Docker con `--privileged` o
   `--net=host` por costumbre.
4. Documentar versión de ROS, imagen, interfaces de red, ruta de workspace y
   dispositivos visibles. Confirmar que NUC y MID-360 están conectados al
   MikroTik como indica la documentación oficial.

**Salida H0:** `artifacts/reports/phase2/<run-id>/hardware_manifest.md` con
inventario y conectividad. Fallar la conexión deja los siguientes pasos
`BLOCKED`, no justifica cambiar IPs o el JSON de Livox a ciegas.

### H1 — observación pasiva de drivers

**Objetivo:** fijar la interfaz verdadera del robot antes de escribir
adaptadores. La base permanece apagada, elevada o asegurada según indique el
organizador; no existe teleop ni publicador de comandos.

1. En el contenedor oficial, descubrir los launch y ejecutables instalados
   con `ros2 pkg executables`/`ros2 pkg prefix`; usar los launch oficiales
   documentados (`tb kobuki.launch.py` y `tb mid360.launch.py`) sólo si esa
   instalación los confirma.
2. Arrancar primero Livox y registrar `ros2 topic list -t`, `ros2 topic info
   --verbose`, `ros2 node info`, `ros2 run tf2_tools view_frames` y frecuencias
   de nube/IMU. Confirmar en particular si la nube es `PointCloud2` o
   `livox_ros_driver2/msg/CustomMsg`, el frame y el origen de `offset_time`.
3. Arrancar el driver Kobuki sin fuente de `cmd_vel`; registrar odometría,
   IMU si existe, estado de batería, TF y el timeout documentado/observable.
4. Grabar bags separados: inmóvil, obstáculos bajos/altos, paredes y vistas
   desde ambas orientaciones del laberinto. Etiquetar cada bag con posición,
   orientación, batería, fecha y commit; no llamar a estos datos verdad de
   árbitro.

**Salida H1:** `P2.1_hardware_manifest.json`, bags inmutables y un mapa de
tópicos/QoS/frames. Esto permite desarrollar R3 en replay sin mover el robot.

### H2 — topología y cobertura sin autonomía

**Objetivo:** usar el laberinto pequeño para caracterizar el sensor, no para
meter un mapa oculto en la política.

Medir, en ambos lados simétricos: alcance de pared, huecos por postes y
cables, retorno de obstáculos de 0.1 x 0.1 x 0.15 m, áreas ciegas frontal y
trasera, auto-oclusiones y estabilidad de la nube en giro. Construir una
plantilla de mapa sólo a partir de scans/bags y conservar la procedencia. La
simetría exige pruebas espejo: el mismo caso se ejecuta con yaw/posición
invertidos y debe producir decisión equivalente tras transformar el frame.

**Salida H2:** informe de cobertura y conjunto de bags de calibración. El
detector rápido de obstáculos de R3 puede probarse offline con ellos.

## 5. Gates de R2 a movimiento, detección y autonomía

Los sufijos `-H` son gates físicos propuestos; no equivalen a marcar un R como
terminado sólo por pruebas de software.

| Gate | Implementación necesaria | Prueba permitida al aprobar | Bloquea todavía |
| --- | --- | --- | --- |
| R2 actual | Supervisor + watchdog `UNCONFIGURED_NO_MOTION` | Ceros, health, rearme denegado en contenedor aislado | Todo movimiento y hardware conectado a R2 |
| R2-H / G0 físico | Perfil con hash y límites aprobados; supervisor/watchdog en procesos separados; mux real; timeout Kobuki medido; registro de parada | Pulsos manuales mínimos, rectos, en zona despejada y con operador | Detección en marcha, giros, planner, autonomía |
| R7-H mínimo | `real_hardware.launch.py` compone drivers, mux, R2-H, diagnóstico y graph verifier; arranque siempre desarmado | Repetir R2-H desde un único launch y fallos kill/lease/mux | Rutas y decisiones HSL26 |
| R3-H | Adaptadores Livox/odom, TF dueño único, ego/obstáculos/cobertura con edad, frame y zonas desconocidas | Escaneo estático; luego movimiento restringido para validar stale/dropout y deskew | Persecución/seguimiento rival |
| G1 | Huella, frenado, respuesta, límites de giro y cobertura medidos para la envolvente exacta | Maniobras manuales/restringidas que no excedan la envolvente | Movimiento no cubierto, reversa y giro independiente sin evidencia |
| R4-H / G3 | Segmentación, registro, EKF y creencia validados contra otro robot parado, móvil y oculto | Pruebas de detección/track, sin usar el track como permiso de movimiento | Captura y táctica autónoma |
| R6-H | Executor de opciones, planificador y controlador producen sólo `MotionCandidate`; safety conserva veto | Ruta corta autónoma en laberinto, con stop/fallos inyectados | Partido completo |
| R5-H | Estado de etapa, freeze, roles, zonas con procedencia, eventos y reset autorizados | Inicio manual/automático tras `FREEZE`; pruebas de dos posiciones | Puntuación oficial sin confirmación de reglas |
| R8-H / G4 real | Dos namespaces/robots, aislamiento de verdad, QoS/TF y pruebas de fallo de extremo a extremo | Ensayo de duelo controlado | Liberación de competición |

R8/MVSim sigue siendo útil para integración y regresiones, pero no es una
precondición lógica para H0--H2, R2-H, R7-H o R3-H. Toda prueba real conserva
su propia evidencia; una pasada en MVSim no aprueba el gate físico y viceversa.

## 6. Trabajo de implementación, en orden

### A. Cerrar R2-H antes de cualquier pulso no nulo

1. Inventariar la configuración realmente instalada de `cmd_vel_mux` y el
   input real del driver Kobuki. Mantener como contrato una sola salida física:
   `/commands/velocity` publicada exclusivamente por `/cmd_vel_mux`.
2. Crear un perfil de seguridad físico separado de `r2_no_motion.yaml`. Debe
   contener hash de configuración, `LimitsProfile` medido, stage/robot ID,
   leases y política de rearme; no copiar los números provisionales de
   `hardware.yaml`.
3. Conectar entradas al mux con prioridades vigentes: stop watchdog 200,
   teleop de emergencia 100, supervisor 50. Teleop sólo es entrada temporal
   de ensayo, nunca publicador directo de salida física.
4. Implementar/corroborar I01--I05 sobre la base: arranque en cero, stop
   prioritario, pérdida de supervisor, pérdida de watchdog, muerte del mux y
   silencio total. Medir por separado instante de pedido de cero, recepción
   por el driver y reposo físico por odometría/vídeo/medición.
5. Definir un mecanismo de paro físico supervisado, operador responsable,
   pasillo despejado y criterio de aborto. Si un fault no detiene la base,
   gate `FAIL`, perfil desarmado y no se reintenta elevando velocidad.

### B. Implementar R7-H mínimo y reproducible

`real_hardware.launch.py` debe recibir `robot_id`, `role`, `profile_path`,
`use_sim_time:=false` y un argumento explícito de armamento que por defecto
sea falso. Debe lanzar/validar en este orden: drivers → TF estático declarado
→ mux → watchdog → supervisor → graph verifier/diagnóstico. Debe abortar o
permanecer desarmado si falta un driver, hay TF duplicado, hash inválido,
watchdog no sano o más de un escritor físico. Ningún nodo de decisión,
navegación o teleop puede saltar ese camino.

La evidencia R7-H debe incluir launch exacto, hashes, inventario DDS, QoS,
TF, estado de batería y los resultados antes/después de cada fallo inducido.

### C. R3-H: percepción utilizable

1. Fijar decoder según el tipo real Livox y su reloj; no asumir `PointCloud2`.
2. Fijar `base_link -> livox` mediante calibración, no sólo el transform fijo
de ejemplo del driver heredado.
3. Publicar `EgoState` y `LocalObstacleSnapshot` con frame, edad, epoch,
   validez frontal y celdas `UNKNOWN`; una ausencia de retorno nunca despeja
   espacio.
4. Ejecutar los bags H1/H2 y luego una prueba de recta/giro restringida contra
   obstáculos bajos. Ante nube vieja, TF faltante, hueco de cobertura o salto
   de odometría, el supervisor recibe una condición de stop.

### D. R4-H, R5-H y R6-H: pasar de detectar a competir

- R4-H usa datos reales de rival parado, cruzando, ocluido y junto a pared.
  Registrar falsos positivos/negativos, latencia y covarianza. El evento de
  captura queda como estimación de seguridad; el marcador oficial es externo.
- R5-H implementa `INIT -> FREEZE -> ACTIVE -> TERMINAL`; durante `FREEZE`
  manda cero incluso si el launch fue manual. Role/color/posición inicial son
  configuración anunciada, no inferencias de un único lado del laberinto.
- R6-H implementa rutas espejo, replanning y control que sólo publica
  `MotionCandidate`. Pruebas: salida corta, stop por obstáculo/desconocido,
  cancelación, perdida de TF y cambio de rol. El supervisor sigue siendo el
  único que publica al mux.

## 7. Matriz de ensayos reales

| ID | Inicio | Ensayo | PASS mínimo | Abort/FAIL |
| --- | --- | --- | --- | --- |
| H0-01 | Hoy | SSH, red e inventario | NUC/Livox/Kobuki identificados | IP/credenciales inferidas o cambiadas sin autorización |
| H1-01 | Hoy | LiDAR inmóvil y bags | tipo, QoS, frame y frecuencia registrados | nube sin frame/timestamp se usa como válida |
| H2-01 | Hoy | Laberinto y obstáculo bajo | cobertura/zonas ciegas medidas en ambos espejos | mapa hard-coded como observación |
| G0-01 | Tras R2-H | Arranque real desarmado | ceros y un solo writer físico | cualquier Twist no nulo antes de armar |
| G0-02 | Tras G0-01 | Stop, kill y timeout | parada física trazada para cada fallo | sólo se observa cero ROS, sin medir reposo |
| G1-01 | Tras G0 | Pulsos rectos de baja energía | velocidad/distancia/frenado dentro del perfil | giro/reversa no aprobados |
| G1-02 | Tras G1-01 | Giro y obstáculo bajo | límite específico y stop seguro | reutilizar la calibración recta |
| R3-01 | Tras G1 | Odom + nube en marcha | edad/TF/coverage correctos o stop | aceptar datos viejos |
| R4-01 | Tras R3 | Rival real con oclusión | track con incertidumbre y fallo declarado | usar verdad del árbitro o pose rival |
| R6-01 | Tras R3/G1 | Ruta corta autónoma | candidate→safety→mux→reposo trazable | comando directo o salida del área |
| G4-01 | Tras R4/R5/R6/R8-H | Duelo simétrico | ambos lados, freeze y roles aislados | puntaje oficial supuesto |

## 8. Entregables y criterio diario

Crear un directorio por sesión:

```text
artifacts/reports/phase2/<UTC-run-id>/
  manifest.json                 # commit, imagen, robot, batería, operador
  ros_graph.txt                 # tópicos, tipos, QoS y TF
  command_trace.csv             # sólo desde G0-02
  raw/                          # bags, vídeos y lecturas no modificadas
  result.md                     # PASS / FAIL / BLOCKED / NOT_RUN y siguiente gate
```

La regla diaria es simple: se puede avanzar a la siguiente fila sólo si la
anterior tiene evidencia `PASS` para el mismo robot, imagen, configuración y
envolvente. Si una prueba falla, se conserva el log, se deja el perfil real
desarmado y se vuelve al último gate aprobado.

## 9. Próximas acciones concretas

1. Ejecutar H0 y H1 hoy usando el stack oficial del NUC, sin copiar todavía
   HSL26 al robot.
2. Crear manifiesto de hardware y dos bags espejo del laberinto/obstáculos.
3. Solicitar por escrito las cinco aclaraciones de reglas de la sección 3.
4. En paralelo, implementar R2-H y R7-H en una rama/revisión identificable;
   no modificar el perfil real para “probar rápido”.
5. Solicitar un slot y autorización explícita antes de G0-02; el primer pulso
   debe ser el experimento de parada, no navegación ni detección del rival.
