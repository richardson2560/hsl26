# HSL26 — Plan de auditoría de SIL, arquitectura y reglamento

**Pregunta central:** ¿el simulador cinemático verifica el sistema que se pretende desplegar y las reglas aplicables, o solamente pasa pruebas dentro de su propio modelo? El informe P5.6 declara un perfil sintético limitado y G4 bloqueado; 634 pruebas aprobadas de núcleo/SIL no equivalen a esta auditoría.

## 1. Fuente normativa y decisiones pendientes

El orden es reglamento oficial y aclaraciones autorizadas → `HSL26_FINAL_ARCHITECTURE.md` rev. 2.2 → `HSL26_TECHNICAL_SPECIFICATION.md` rev. 2.2 → contratos de código → fixtures. Se revisó directamente la transcripción suministrada `Регламент HSL26 - v06092026.md` (seis páginas marcadas; SHA-256 `b5a2eea6e58a164583cd39466f89d50cbfaf609b9e2013c5ee324cded4608ac4`). La arquitectura cita un PDF con sufijo `(1)`; el PDF original no está actualmente en `docs`. La primera tarea documental es conservar la transcripción y cotejarla con el PDF autorizado cuando esté disponible. Cualquier divergencia produce un registro con texto/página, decisión y test cambiado.

La transcripción confirma dos etapas simétricas de 10 minutos, cuatro minutos de preparación desde el inicio de cada etapa sin cruzar la línea de salida, intercambio de roles, distancia de captura estrictamente menor de 0.45 m, ángulo Guardian no mayor de 45°, LOS libre y llegada por contacto de contornos. La política HSL26 de comando cero durante preparación es más restrictiva que la regla de no cruzar la línea. Los perfiles SIL actuales de 30 segundos y freeze de 0.15 segundos son fixtures aceleradas para desarrollo, no ensayos del formato completo. Permanecen sin valor numérico oficial los puntos; además requieren decisión autorizada la identidad/geometría de zonas, retención entre etapas, señal técnica del reloj y desempate de eventos simultáneos.

### Reglas comprobadas en la transcripción y consecuencias de prueba

| Página | Texto confirmado | Implicación para SIL y release |
| --- | --- | --- |
| 3 | Acceso a robots/polígono termina 30 minutos antes; durante competición solo se permite arrancar desde PC del organizador | Ensayar arranque sin dependencia de GUI/red externa y entrega de imagen congelada |
| 4 | TurtleBot2 con MID-360 principal; módulos ~1 m; obstáculos estáticos desconocidos, mínimos 0.1 × 0.1 × 0.15 m; geometría del laberinto se conoce el primer día | Medir cobertura de obstáculos bajos y distinguir mapa autorizado de información de prueba desconocida |
| 5 | Eliminatoria por pares, dos etapas de 10 minutos, roles intercambiados; cuatro minutos iniciales sin cruzar línea | Ejecutar dos etapas completas con roles cambiados; verificar línea de salida y el cero conservador de HSL26 |
| 5 | Un reinicio completo solo por ambos capitanes o organizadores; detener etapa por capitán da máxima puntuación rival | El reset interno no se convierte en reinicio oficial; una parada de seguridad no se etiqueta concesión |
| 5 | Prohibido fijar datos de detalles antes desconocidos y añadir sensores/equipo de red/cómputo o modificar robot | Auditar procedencia de mapas/zonas/modelos y configuración de hardware final |
| 5–6 | Explorer gana etapa si no es capturado; gana el match quien suma más puntos; puntos detallados se anuncian el primer día | `official_score=null` hasta recibir perfil autorizado; surrogate de victoria etiquetado |
| 6 | Llegada al primer contacto entre contorno del robot y contorno de la zona inicial Guardian | Detector de evento continuo y rechazo de posición inicialmente dentro, salvo decisión oficial |
| 6 | Captura con distancia entre marcos `<0.45 m`, X Guardian a `≤45°` y sin obstáculos entre robots | Probar umbrales estricto/inclusivo y simultaneidad de las tres condiciones |
| 6 | Colisiones estáticas sin penalidad; desplazamiento crítico puede causar reinicio; jueces/organizador resuelven disputas | Seguridad sigue evitando colisiones por fiabilidad; árbitro SIL no inventa decisión oficial |

## 2. Matriz de conformidad a construir

Cada fila tendrá `regla/contrato`, `implementación`, `prueba positiva`, `prueba negativa`, `perfil`, `traza`, `estado`, `revisor`. Ninguna fila será `PASS` por existencia de código o por una prueba ajena al requisito.

| Área | Contrato a contrastar | Código y prueba objetivo |
| --- | --- | --- |
| Reglas | Captura: origen a origen `d<0.45 m`, ángulo Guardian ≤45°, LOS verdadero simultáneos; llegada: primer contacto de contornos desde fuera; timeout Explorer vencedor si no capturado | `hsl_core/rules.py`, `sim/kinematic/referee.py`, `match.py`; T01–T05, T14, I12; casos de entrada y salida dentro de un tick, tangencia, simultaneidad y obstáculo entre robots |
| Etapa | FREEZE/ACTIVE/TERMINAL, reloj oficial vs estimado, reset, holds, puntuación no inventada | `hsl_core/match.py`, `sim/kinematic/match.py`; T30, I04/I11–I13; dos etapas con cambio real de roles y política de memoria declarada |
| Verdad y observación | Planta/referee separados; política solo observación propia, mapa conocido autorizado y creencia estimada; fuentes/versiones intactas | `scenario.py`, `sensors.py`, `autonomous.py`, `benchmark.py`; pruebas de capacidad, imports y trazas de cada dato; I14 |
| Mundo y rutas | Mapa desconocido/ocupado/libre, cobertura, topología extraída de observación; A* con coste/versión; obstáculos dinámicos | `mapping.py`, `topology.py`, `astar.py`, `sim/kinematic/perception.py`; T10–T17, T29, I09; separar `observed_map`, `approved_map`, `oracle` |
| Rival | Medida con covarianza, edad y marco; EKF/asociación/oclusión; origen y yaw válidos; GPIS solo cuando hay nube apropiada | `ekf_opponent.py`, `topological_belief.py`, `implicit_surface.py`; T08/T09/T18/T19/T25/T26, I10; falsos positivos y cobertura |
| Táctica | Propuestas factibles desde observación, orden prioridad→urgencia→utilidad, autoridad de opción, cancel/efecto/timeout y ambos roles | `tactics/fsm.py`, `options.py`, `autonomous.py`; T21/T28, I11/I13; traza de features y por qué se eligió cada opción |
| Movimiento/seguridad | Ruta realizable, `MotionCandidate` con lease, supervisor en cada tick, curvatura y barrido, huella física y timeout | `control/*`, `autonomous.py`, `benchmark.py`; T06/T12/T13/T24, I01–I08 según perfil; ningún bypass a planta |

## 3. Perfiles y alcance de sus conclusiones

| Perfil | Entradas permitidas | Conclusión posible |
| --- | --- | --- |
| `kinematic/observed_map` | Rayos 2D y odometría sintética; detección rival sintética declarada | Reglas, topología, táctica, control y seguridad bajo supuestos de sensores; no GPIS 3D |
| `kinematic/approved_map` | Mapa estructural con procedencia explícita | Táctica con mapa previo autorizado; no prueba descubrimiento |
| `kinematic/oracle` | Verdad de mapa/pose solo en ablación | Cota diagnóstica; excluido de dataset elegible |
| `mvsim` | Tópicos ROS y sensor caracterizado en plan de integración | Dinámica, ROS y percepción hasta la fidelidad medida |
| `replay/real` | Bags reales o sensores físicos | Fidelidad/seguridad bajo los escenarios y condiciones observados |

Auditar que `benchmark.py` no marque como elegibles episodios de fixture; mantener estratos separados por sensor, mapa, dinámica, versión GPIS y permiso de zona. Si una comparación necesita el mismo escenario en dos perfiles, comparar decisiones intermedias y divergencias con tolerancias declaradas, nunca exigir trayectorias idénticas cuando la física difiere.

## 4. Pruebas adversariales nuevas

1. Ocultar por completo el rival y variar dropout/blind sector: la creencia crece en incertidumbre y no aparece una captura certificada.
2. Presentar un retorno de pared como rival, mover un obstáculo al corredor y retrasar scans: sin falso origen del robot ni ruta que atraviese ocupación.
3. Cambiar mapa/epoch/lease durante una opción; retrasar plan viejo y cancelar a la vez: cero comando admitido de la instancia revocada.
4. Generar entrada/salida de zona y umbral de captura dentro de un paso; barrer ángulo y LOS con cambio de estado en el mismo intervalo: evento más temprano o ambiguo, sin decidir por orden de callbacks.
5. Alterar radio de huella, latencia, slip y velocidad del rival dentro de un banco declarado: reportar éxito y fallos por estrato; cualquier violación de seguridad invalida ese candidato.
6. Intentar inyectar pose verdadera, zona no aprobada o dato del segundo robot en `PolicyInput`; rechazo verificable y registro de procedencia.
7. Ejecutar el horizonte completo autorizado y dos etapas consecutivas sin editar política entre ellas; medir preparación, tiempo activo, rendimiento y política de memoria. Mantener el ensayo corto para regresión, con nombre distinto.

## 5. Baseline G4 y resultado de auditoría

El baseline se congela con hash de código, perfil, mapas, sensores, calibraciones sintéticas, propuestas/guardas y política de ambos roles. La matriz G4 requiere Guardian y Explorer con acciones no triviales, fallback cuando falta zona, efectos y cancelación, freeze/timeout, recuperación y trazas observación→opción→comando→supervisor→planta. El árbitro ideal y la salida estimada se comparan pero permanecen separados. Un revisor independiente decide `PASS` por perfil solo tras revisar I11–I14 y los casos aplicables T01–T30.

Entregables: matriz regla↔código↔test, registro de divergencias y correcciones, benchmarks de duración real, baseline reproducible, informe de límites de fidelidad y acta G4. Un G4 cinemático aceptado habilita la recolección en ese perfil; no cierra G0–G3 ROS/físicos ni G6.
