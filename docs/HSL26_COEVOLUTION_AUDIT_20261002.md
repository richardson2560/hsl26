# HSL26 — Auditoría de la estrategia de coevolución y del dictamen externo

**Fecha:** 2026-10-02 · **Base Git:** `87a22494fd05e5733019d29a485e2cf731f25d15` más cambios locales previos. **Alcance:** revisión de documentación/código, pruebas focalizadas y diagnóstico cinemático de desarrollo. No aceptación ROS, física, HGW real ni G4/G5/G6.

## 1. Dictamen

La recomendación de priorizar integración, sensibilidad táctica y presupuesto es correcta. No se justifica cancelar definitivamente la coevolución ni imponer dos modelos HGW. La ruta de cierre es: baseline conforme → alternativas tácticas realizables → diagnóstico de genes → búsqueda pequeña frente a rivales congelados → coevolución alternada únicamente con señal y presupuesto → evaluación independiente y transferencia.

Un único prior HGW con distintos presupuestos de registro es el diseño inicial adecuado si ambas plataformas comparten geometría. Su precisión y coste deben demostrarse; ni la superioridad milimétrica ni el colapso inevitable en movimiento están establecidos. Las propuestas de relajar seguridad, rotar durante freeze y codificar el mapa conocido requieren contraste con contratos y reglamento, no adopción automática.

El [plan revisado](HSL26_COEVOLUTION_PLAN.md) desarrolla estas decisiones con entregables, dependencias y reglas de parada. La [entrada rev. 1.1](../artifacts/reports/coevolution_audit_20261002/plan_rev1_1_input.md) preserva las modificaciones locales del usuario: las puertas del backend ligero quedan archivadas como propuesta diferida, no como requisito de cierre.

## 2. Contraste de afirmaciones

| Afirmación del auditor | Evidencia contrastada | Dictamen y efecto sobre el plan |
| --- | --- | --- |
| El lazo autónomo básico no existe | [P5.6](../artifacts/reports/phase5/P5.6_autonomous_integration_report.json), `autonomous.py`, `match.py` y pruebas | Parcialmente desactualizado. Hay selector/autoridad/routing/control/supervisor y duelo G/E opcional; falta conformidad completa y aceptación G4. Completar y auditar lo existente |
| G4 solo requiere conectar tópicos ROS | [SIL conformance](HSL26_SIL_CONFORMANCE_PLAN.md) §5 y [integración](HSL26_ROS_MVSIM_INTEGRATION_PLAN.md) R1–R8 | Insuficiente. G4 cinemático y runtime ROS tienen evidencias distintas; hacen falta lifecycle, roles, terminales, truth isolation, fallos y puertas anteriores aplicables |
| La urgencia anula todos los pesos y hace inviable evolucionar | [Selector](../hsl_core/hsl_core/tactics/fsm.py) y generación de propuestas [autónoma](../sim/kinematic/autonomous.py) | La utilidad sí ordena alternativas de igual prioridad/urgencia. Restricción dominante local: Guardian tracked y Explorer entregan una propuesta; Explorer preselecciona ruta con heurística fija. Guardian sin track sí tiene alternativas |
| Deltas cero prueban gradiente global cero | [P6.3](../artifacts/reports/phase6/P6.3_runtime_and_learning_audit.json) y `evolution.py` | No. Evidencia de plateau muestral y paisaje discreto. Hay también exclusiones por contacto/seguridad; no tratarlas como empates. Separar cableado, alternativas, perturbación y objetivo |
| Un segundo simulador resolverá el aprendizaje | Plan rev. 1.1 frente al coste real y ausencia de señal Explorer | No demostrado. Diferirlo; primero corregir elección y perfilado en referencia. Mantener posibilidad futura con amortización y comparación diferencial |
| Basta vectorizar raycasting para obtener 15x | [Raycaster](../sim/kinematic/raycaster.py) ya tiene `cast_many`; P6.3 documenta comparación scalar/batched | Propuesta parcialmente realizada, factor no demostrado. Diagnóstico nuevo señala muchos árboles de rutas y clearance de arcos; optimizar hotspots medidos |
| Un grid de unas 50 partidas tarda ≤1 h | Runtime P6.3 y número de cruces baseline/candidato | Sin fundamento general. Contar matches por rol/mapa/rival, duración hasta terminal y coste de validación. Tres mapas × nueve configuraciones no son nueve episodios; evaluación pareada añade referencias |
| Se puede evaluar supervivencia o llegada con el benchmark actual | [Benchmark](../sim/kinematic/benchmark.py) solo admite CAPTURE/TIMEOUT y freeze 0.15 s | Sobrevivir al timeout completo es outcome válido; truncar no equivale. Falta ARRIVAL autorizado. El objetivo no corresponde todavía a competición completa |
| Ambos roles necesitan dos HGW de resolución diferente | [Plan HGW](HSL26_HGW_ROBOT_MODEL_PLAN.md), `registration.py` e `implicit_surface.py` | No es requisito. Un modelo compartido, mismo registrador y trabajo/cadencia acotados. Segundo modelo añade calibración, covarianza, hashes y validación de conmutación; exigir necesidad medida |
| HGW consume >40% y clustering <2–3%; modelos a 12–18/70–90 ms | Informes P4 sin benchmark Livox/NUC real | Cifras no respaldadas en este checkout. No convertirlas en gates ni decisiones de despliegue. Medir pipeline completo bajo carga sostenida |
| NUC BOXNUC7I7BNH usa i7-7500U/15 W | [TPS Intel](https://www.intel.com/content/dam/support/us/en/documents/mini-pcs/nuc-kits/NUC7i5BN_NUC7i7BN_TechProdSpec.pdf) y [ficha i7-7567U](https://www.intel.com/content/www/us/en/products/sku/97541/intel-core-i77567u-processor-4m-cache-up-to-4-00-ghz/specifications.html) | Identificación incorrecta: NUC7i7BN usa i7-7567U, 28 W; confirmar robot real. Frecuencia/temperatura/CPU del solver no se deducen de una ficha |
| HGW garantiza origen milimétrico | [P4.2](../artifacts/reports/phase4/P4.2_registration_report.json) bloquea error del origen real, geometría y calibración | No demostrado. Ajuste de superficie reduce sesgo solo con modelo/TF/soporte adecuados; medir vistas parciales y covarianza. Yaw rival no es necesario para captura |
| El SIL usa centroide superficial puro | [Extractor](../sim/kinematic/perception.py) agrega radio a cada rango antes de promediar | Inexacto. Tiene corrección geométrica sintética, aún sesgada y no validada para montaje físico. No sustituye HGW ni certifica captura |
| EKF entrega yaw para warm start | `OpponentFilter` SIL de cuatro estados y `yaw_valid=false` en `autonomous.py` | Inexacto. Predice posición/velocidad; yaw requiere hipótesis de registro previa y tratamiento de no observabilidad |
| Deskew y paredes planas resuelven sustracción en movimiento | [Plan HGW](HSL26_HGW_ROBOT_MODEL_PLAN.md) §2.6 y contratos de P2 | Ayudan; no eliminan incertidumbre ego/mapa ni movimiento del rival. Deskew ego no compensa automáticamente al blanco. No existe evidencia de veto infalible |
| Guardian debe usar HGW quieto y nunca en marcha; Explorer puede apagarlo siempre | No hay aceptación comparativa de esos modos | Hipótesis de scheduling/precisión, no conclusión física. Variar trabajo por residual, incertidumbre y rol; mantener un pipeline común y medir decisiones perdidas |
| Margen 0.20 + radio 0.15 deja robot inmóvil a clearance 0.35 | [P56Profile](../sim/kinematic/autonomous.py) valida margen ≥radio; `SafetySupervisor` resta margen una sola vez | Doble conteo. El margen 0.20 incluye radio. Con fixture a 0.20 m/s, frenado nominal ≈0.0385 m; 0.15 m disponible no implica cero. Lateral/frontal/arco son cantidades distintas |
| Reducir margen total a 0.06–0.08 es suficiente y seguro | Constructor rechaza esos valores; físico YAML radio 0.22 frente a SIL 0.15 | Rechazado como cambio directo. Calibrar footprint/uncertainty y velocidad dentro del supervisor. No mutar seguridad ni adoptar velocidad física desde fixture |
| Contacto estático sin penalización justifica rozar/empujar | Reglamento §5.3 también admite reinicio por desplazamiento crítico y adjudicación del organizador | Regla deportiva no garantiza seguridad ni movilidad. Contacto intencional fuera del alcance; reportar conflicto entre capacidad física y margen, no retirar guardas |
| Rotar durante freeze siempre está permitido y mejora SLAM | Regla prohíbe cruzar línea; A01 adopta cero movimiento; LiDAR azimut 360° | No prohibición textual absoluta de rotación, pero sí cambio del contrato actual. La huella completa debe quedar dentro; rotación no revela detrás de paredes. Acumulación pasiva primero |
| Mapa fijo vuelve irrelevante toda generalización y autoriza preprogramarlo | Reglamento §§5.1/5.2: mapa fijo y restricción de información previamente desconocida | Interpretación incompleta. Mantener robustez mínima y origen autorizado de mapa/zonas; adaptación al mapa observado/aprobado separada de claims entre mapas |
| Deben eliminarse espectros de grafos | Arquitectura §6 los declara opcionales; no se llaman en política autónoma examinada | No son el hotspot demostrado. Mantener fuera del camino crítico y diferir experimentos espectrales; no atribuirles starvation sin medida |

## 3. Verificación ejecutada en esta revisión

### 3.1 Regresión focalizada

Comando en la raíz del repositorio:

```powershell
python -m pytest hsl_core/tests/test_p53_tactics.py hsl_core/tests/test_p63_evolution.py hsl_core/tests/test_p42_registration.py hsl_core/tests/test_safety.py sim/kinematic/test_p56_perception.py sim/kinematic/test_p56_autonomous.py sim/kinematic/test_p63_benchmark.py -q
```

**Resultado:** [190 passed en 6.57 s](../artifacts/reports/coevolution_audit_20261002/focused_tests.txt); un aviso del cache de pytest en Windows. La regresión cubre los contratos relevantes; no acredita ningún gate ni exactitud física. No se modificó código del controlador, reglas, modelos ni límites.

### 3.2 Elección y perfilado de un fixture de entrenamiento

El script reproducible [review_checks.py](../artifacts/reports/coevolution_audit_20261002/review_checks.py) ejecuta únicamente `maze_multiring_7x7_train_a`, baseline y seed 20260931. No utiliza validation ni held-out. Instrumenta el selector en ese proceso y restaura el método original; no edita su implementación. Resultados y hashes: [review_checks.json](../artifacts/reports/coevolution_audit_20261002/review_checks.json).

| Medida | Resultado | Interpretación |
| --- | --- | --- |
| Calls de selector por rol | 986 G y 986 E | Observaciones correlacionadas de un solo episodio, no muestras estadísticas independientes |
| G con ≥2 alternativas en misma cohorte prioridad/urgencia | 543 (55.1%) | Elección por utilidad posible en búsqueda; no prueba de mejora por mutación |
| G con una alternativa elegible | 440 | Consistente con persecución de propuesta única; el resto incluye estados sin alternativa |
| E con ≥2 alternativas en cohorte | 0 | Confirma falta de elección de pesos en ese fixture |
| E con una / cero alternativas elegibles | 715 / 271 | Distinguir propuesta única de fallback/inadmisibilidad |
| Terminal instrumentado | CAPTURE a 49.15 s | Reproduce el caso histórico, pero hay contacto registrado para ambos roles: no baseline aceptable |
| Tres runs sin profiler de cap 8 s | 2.940, 1.935, 1.818 s pared; 7.85 s activos | Microdiagnóstico local con warm-up/carga variable; no benchmark sostenido del horizonte completo |
| Márgenes 0.06 / 0.08 / 0.20 | rechazado / rechazado / válido en schema | Evidencia directa del contrato de margen que incluye radio; no calibración física |

[Perfil cProfile](../artifacts/reports/coevolution_audit_20261002/profile_cumulative.txt): 24,619 consultas de ruta y 16,813 árboles Dijkstra en 986 pasos. Tiempos inclusivos aproximados: `_path_to_node` 25.010 s, `dijkstra_tree` 22.975 s, `_curved_path_clearance` 14.335 s, `cast_many` 4.309 s. Se solapan, y cProfile modifica sustancialmente los tiempos: **no sumar ni comparar con runs sin profiler para afirmar speedup**.

La LRU actual retiene dos fuentes; al probar fuentes distintas para muchos destinos puede recomputar árboles. El perfil sustenta investigar agrupación/reutilización por fuente y snapshot; no demuestra por sí solo que ampliar cache preserve rutas o elimine todo el coste. Mantener regresiones de recorte desde pose, OPEN/UNKNOWN, topología nueva y determinismo. Seguridad de arco es otro coste medido; simplificarla sin comparación sería cambiar el experimento.

### 3.3 Límites de la auditoría

No se accedió a los bags de la prueba previa ni a su código externo `TurtleBot Tracker`/`DESIGN_OVERVIEW.md`: la conversación del auditor no aporta esos archivos y no se encontraron como fuentes disponibles en este checkout. Su éxito previo se trata como antecedente por verificar. No hay modelo HGW de producción aceptado localmente ni evidencia de pose milimétrica del ensamblaje final.

Se revisó la transcripción del reglamento suministrada, no se afirmó equivalencia con PDF original ni ausencia de aclaraciones posteriores. El mapa oficial, puntos, señal técnica, memoria y autorización de entradas continúan en el circuito del plan maestro.

El archivo `simulation.spdx.json` y los outputs R0 ya modificados por el usuario se preservaron. La snapshot del plan previo queda en artifacts; esta auditoría no marca pasos R0, G4, G5 o G6 como completados.

## 4. Cambios documentales y criterio de cierre

El plan rev. 2.0 sustituye la obligación de un segundo simulador por perfilado/optimización condicional; distingue schema actual de extensiones tácticas; exige alternativas realizables antes de evolución; propone búsqueda por rol y rondas alternadas; conserva rivales de referencia, safety exclusion y held-out; fija presupuesto, censura, estadística y cutoff; incorpora un HGW compartido con registro acotado y transferencia medida.

Se sincronizan los planes maestro, HGW y P6, y la descripción SIL P5.6 del README para evitar instrucciones contradictorias. Arquitectura/TS mantienen sus contratos: no se relaja freeze ni seguridad mediante una edición de este plan.

Resultado de la revisión: **plan accionable y diagnóstico reproducible; entrenamiento elegible y despliegue pendientes**. La siguiente entrega técnica es resolver propuestas únicas/terminales y conformidad, con tiempo reservado para integración/percepción y evaluación. Un experimento cerrado puede conservar baseline por falta de ventaja; eso no convierte un baseline de fixture en baseline G4 aceptado.

## 5. Resolución de la segunda revisión externa — plan rev. 2.1

**Fuente:** texto aportado por el usuario el 2026-10-02, adjunto `6dc40f64-57ab-424d-a53f-39449d644ede/Pasted text.txt`. Es una revisión externa del documento; el autor no está identificado y declara que no volvió a ejecutar las 190 pruebas. SHA-256 del adjunto: `26f9f2f3dceedd3d9b0a219b25d630a8af3404611e451f59505549c01152a0f7`. No se registra como segundo sign-off de aceptación ni se exige una nueva gobernanza de dos auditores.

| Observación | Resolución | Referencia del plan |
| --- | --- | --- |
| Falta algoritmo para construir hasta tres alternativas | Aceptada. Snapshot y catálogo autorizado, rutas OPEN, presupuestos explícitos de catálogo/rutas, recorrido intercalado por rama, orden determinista independiente del genoma, deduplicación de plan/efecto y cap con omisiones registradas. La heurística anterior no preselecciona ganadores. Rutas distintas al mismo target requieren contrato adicional y se difieren | §3.2 |
| `choice_rate` confunde disponibilidad y oportunidades | Aceptada. Contadores y métricas separados para ACTIVE, propuestas admisibles, cohortes de alternativas distintas y casos vacíos; denominadores cero dan null. Los conteos históricos solo prueban cardinalidad por llamada, sin acreditar deduplicación ni fase activa | §3.1 y §11 |
| 40 Hz táctico y 50 Hz seguridad podrían ser procesos distintos | Se acepta detectar la discrepancia; no esa explicación sin evidencia. YAML llama al parámetro `supervisor_rate_hz`; arquitectura §14 y TS §5 atribuyen 50 Hz al supervisor de seguridad/publicación final. Resolver ownership/timer/perfil antes del runtime; prioridad 50 del mux no es frecuencia | §9.2 |
| No hay dos sign-offs identificados | Correcto como límite de trazabilidad, sin implicar un requisito nuevo. Este anexo registra fuente, alcance, dictamen y resolución; no certifica independencia institucional ni gates | Este anexo |
| Frecuencias 40/50 Hz deberían permitir simulación mucho más rápida | Sí como distinción de relojes, no como throughput garantizado. Añadida definición de periodo virtual, RTF, evaluaciones por segundo real, speedup entre backends y costes globales; el runner existente ya avanza sin esperar al reloj real | §9.3 |

Comprobación de código en esta revisión: `TwoRobotMatch.step()` avanza `start_ns + dt_ns`; el bucle del benchmark no incluye `sleep`. La política se invoca cada paso virtual de control de 50 ms por defecto; no hay un timer SIL independiente a 40/50 Hz. `autonomous.py` entrega al supervisor tiempos steady sintéticos 0/1 ns, lo que **no mide el coste real del callback** ni acredita deadlines físicos. Estos hechos justifican separar presupuesto de CPU del experimento y latencia modelada del agente.

El algoritmo propuesto sigue pendiente de implementación. Un límite de tres propuestas puede sesgar el espacio de tácticas; independencia del genoma no equivale a ausencia de sesgo. El plan exige registrar truncamiento, diversidad y sensibilidad al catálogo en training antes de seleccionar candidatos.

En esta revisión documental se contrastaron código y datos existentes, unidades/aritmética, enlaces locales y diff. No se repitieron los episodios perfilados ni las 190 pruebas: continúan siendo evidencia de la revisión anterior, no ejecuciones nuevas. Se preservan los cambios concurrentes de integración/R1, runtime y configuración.
