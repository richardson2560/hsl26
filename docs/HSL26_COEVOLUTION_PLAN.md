# HSL26 — Plan de coevolución Guardian/Explorer y selección competitiva

**Revisión:** 2.1 · **Fecha:** 2026-10-02 · **Estado:** plan de ejecución revisado; no acredita implementación, aprendizaje ni cierre de gates.

**Objetivo:** obtener una mejora táctica reproducible de ambos roles dentro del presupuesto disponible y entregar una política congelada, compatible con el sistema integrado. Si la evidencia no respalda un candidato, cerrar el experimento con sus datos y conservar el baseline aceptado. G5 es opcional; disponer de una población entrenada no sustituye G4 ni G6.

Esta revisión contrasta la conversación del auditor con código, informes y reglamento. La [auditoría trazable](HSL26_COEVOLUTION_AUDIT_20261002.md) distingue hechos, hipótesis y recomendaciones. La [revisión 1.1 de entrada](../artifacts/reports/coevolution_audit_20261002/plan_rev1_1_input.md) conserva también las modificaciones locales anteriores a esta auditoría. Sus propuestas de backend ligero y puertas L0–L5 quedan **diferidas y sustituidas por la secuencia de este documento**. La arquitectura §§7, 9–12, la especificación §§8, 11–15 y las puertas del [plan maestro](HSL26_INTEGRATION_AND_COMPETITION_MASTER_PLAN.md) mantienen su autoridad.

La revisión 2.1 incorpora la segunda revisión externa: concreta construcción/deduplicación de alternativas, separa disponibilidad de propuestas de oportunidades de elección y distingue frecuencias virtuales, rendimiento de lote y deadlines físicos. La observación sobre 40/50 Hz se registra como discrepancia de configuración/objetivo, sin inventar dos supervisores. Su resolución se documenta en §5 de la auditoría; no constituye un segundo sign-off de aceptación.

## 1. Decisiones de alcance para terminar

| Decisión | Trabajo inmediato | Trabajo condicionado o diferido |
| --- | --- | --- |
| Simulación | Reutilizar `sim/kinematic/`; perfilar, corregir elección táctica y preservar contratos | No construir `sim/kinematic_light/` en el camino crítico. Reabrir únicamente con señal demostrada, coste/beneficio medido y tiempo sobrante después de reservar integración y evaluación |
| Búsqueda | Sensibilidad → búsqueda pequeña contra rivales congelados → coevolución alternada si aporta valor | Sin población inicial 16×16, torneo exhaustivo, crossover, gramática de opciones ni búsqueda ciega de generaciones |
| HGW | **Un único prior geométrico aceptado y un registrador**, compartidos entre roles si el ensamblaje es idéntico; presupuesto de ejecución adaptable | Sin obligación de entrenar `M_fast` y `M_full`. Un segundo modelo solo ante fallo medido que no resuelvan ROI, muestreo, warm start o implementación |
| Seguridad | Medir footprint, frenado, cobertura, latencia y motivos de limitación; resolver navegación lenta dentro del contrato | No evolucionar márgenes, límites físicos, reglas ni autoridad; no reducir el margen total por debajo del radio |
| Preparación | Cero movimiento, adquisición pasiva de nube, mapa y localización de lo visible | Rotación en freeze solo tras modificar y verificar expresamente la política A01 y el confinamiento de toda la huella; fuera del MVP |
| Entrega | Baseline integrado, datos inmutables, candidato o rechazo razonado, transferencia y rollback | Espectros de grafos opcionales, aprendizaje online, shaping adicional y generalización masiva fuera del cierre mínimo |

No se elimina la coevolución por una imposibilidad teórica: se condiciona a que existan elecciones controlables y tiempo para evaluar. Tampoco se desplaza P4 a P6: el modelo HGW se construye y acepta en su [plan específico](HSL26_HGW_ROBOT_MODEL_PLAN.md); el aprendizaje táctico fija su hash y caracteriza la transferencia.

## 2. Estado real y dependencias

| Elemento revisado | Evidencia actual | Consecuencia |
| --- | --- | --- |
| P5.6 | `autonomous.py` integra selector, autoridad, routing, control y seguridad; el duelo opcional incluye ambos roles y EKF sintético | Integración parcial existente, no un lazo completamente ausente ni baseline aceptado |
| P6.3 | `TacticalGenome`, mutación proyectada, comparación por rol contra baseline y trainer de fixtures | No hay dos poblaciones independientes ni archivo de rivales implementados en ese trainer |
| Señal | Deltas cero en muestras registradas; también candidatos excluidos por seguridad/completitud | Separar empate, exclusión y falta de potencia. Ninguno prueba inutilidad global de la coevolución |
| Runtime | P6.3 registró aproximadamente 2.3x–2.8x en determinadas ejecuciones; raycaster vectorizado y cache de árboles de rutas ya existen | Medir nuevamente; no prometer otra aceleración de 15x por añadir NumPy |
| Terminales del benchmark | Solo `CAPTURE` y `TIMEOUT`; objetivo sintético sin autoridad de llegada | No sirve todavía para optimizar ni acreditar victoria por llegada a base |
| Fidelidad | Rayos 2D, mapa estático de fixture, odometría sintética; no HGW real | No confundir ese perfil con mapa observado físicamente, detector Livox ni rendimiento embarcado |
| Gates | Informes P5.6 y cierre P6.4/P6.5: G4 `BLOCKED_NOT_RUN`, G5 `NOT_RUN`, G6 `BLOCKED` | Los diagnósticos pre-G4 son `development-only`; datos elegibles y promoción esperan aceptación por perfil |
| ROS/MVSim | Adaptador MVSim sin lógica; nodos operativos incompletos; R0 aporta entorno y smoke, no autonomía | Cerrar R1–R8 aplicables y evidencia de grafo/actuación según plan de integración; conectar tópicos por sí solo no cierra G4 |

Los informes históricos se conservan. Toda aceptación posterior requiere reporte nuevo, matriz de casos aplicables y revisión independiente. Antes de recolectar episodios elegibles: baseline determinista G/E, perfil de sensores/mapa/reglas completo, ausencia de truth leakage, cancelación/reemplazo, fallback, terminales y seguridad auditados. Un G4 cinemático habilita datos de ese perfil, no movimiento físico ni G6.

## 3. Resolver la señal antes de optimizar

### 3.1 Diagnóstico causal

La prioridad/urgencia limita legítimamente la utilidad; no demuestra que todo gen sea irrelevante. Hay una restricción anterior:

- Guardian con track utilizable entra en `_guardian_pursuit_proposal()` y devuelve **una sola** propuesta `PRESSURE_ROUTE`.
- Explorer entrega una propuesta. Bajo amenaza elige previamente una ruta por LOS y `distance_from_rival - 0.25 * path.cost`; esa elección no depende del genoma.
- Sin rival, Guardian sí genera propuestas de búsqueda para varios portales/frontiers. Allí puede existir elección por utilidad.
- Histéresis/dwell pueden afectar transiciones, incluso con una propuesta nueva; no confundir pesos sin alternativas con todos los parámetros inactivos.

La [comprobación de esta auditoría](../artifacts/reports/coevolution_audit_20261002/review_checks.json), en un único fixture de training, contó 543/986 llamadas Guardian con al menos dos alternativas en la cohorte ganadora y 0/986 Explorer. Guardian tuvo una sola alternativa elegible en 440 llamadas. Son conteos de llamadas, no 986 oportunidades reales de elección: incluyen estados sin propuestas y no registran por separado fase ACTIVE ni deduplicación conductual. Respaldan el diagnóstico local; no estiman frecuencias para todos los mapas.

Registrar por contexto: fase, readiness y autorización, propuestas totales y factibles, cohorte de mayor prioridad/urgencia, features por target, utilidad, distancia al empate, decisión y motivo, continuidad de opción, ruta y comandos. Contar las siguientes cantidades por rol, escenario y modo de percepción:

- `N_active`: invocaciones del selector en ACTIVE; distinguir dentro de ellas falta de readiness/lease y otros guards globales. Contar FREEZE/INIT/TERMINAL aparte.
- `N_available`: invocaciones ACTIVE con al menos una propuesta factible distinta de HOLD_SAFE después de todos los gates.
- `N_choice`: invocaciones ACTIVE con al menos dos alternativas **conductualmente distintas** dentro de la cohorte ganadora de prioridad/urgencia. Los IDs distintos no bastan.
- `N_forced = N_available - N_choice` y `N_empty = N_active - N_available`: casos sin elección por utilidad y casos sin propuesta admisible. Registrar HOLD_SAFE/fallback y su razón por separado; un HOLD por fase no es una oportunidad fallida de táctica activa.

| Métrica propuesta | Fórmula | Interpretación |
| --- | --- | --- |
| `proposal_availability_rate` | `N_available / N_active` | Disponibilidad de acción admisible |
| `choice_coverage_rate` | `N_choice / N_active` | Cobertura de oportunidades en las decisiones activas; sustituye el nombre ambiguo `choice_rate` |
| `conditional_choice_rate` | `N_choice / N_available` | Capacidad de elegir entre alternativas cuando existe alguna propuesta admisible |
| `no_proposal_rate` | `N_empty / N_active` | Falta de propuesta admisible, desglosada por causa |

Registrar también ciclos ACTIVE que retornan antes del selector (inputs ausentes/incoherentes, fallos), fuera de `N_active`: estas métricas describen invocaciones del selector y no deben ocultar indisponibilidad anterior de la cadena.

Guardar numeradores y denominadores; si el denominador es cero, resultado `null`, nunca cero ni NaN. Para comparaciones pareadas de mutación, medir decisiones distintas sobre snapshots comunes con igual estado de selector y oportunidad real de elección; contabilizar aparte cambios debidos a histéresis/dwell. En episodios cerrados, registrar también primera divergencia de decisión, target/ruta/comando y outcome: después de divergir, los snapshots ya no son idénticos. `mutation_effect_rate` exige declarar gen perturbado y denominador; un hash diferente no es efecto conductual.

Los conteos históricos permiten afirmar 0/715 oportunidades **por cardinalidad** de Explorer entre llamadas con propuestas admisibles, y 271/986 llamadas sin propuesta; no convertir esos cocientes en las métricas ACTIVE/deduplicadas sin recoger los campos que faltan. En Guardian, 543/983 es el cociente por cardinalidad condicionado a disponibilidad; la diversidad de rutas y el efecto de genes todavía se deben medir.

### 3.2 Cambio mínimo de propuestas

Conservar los guards duros y el orden lexicográfico. Separar **admisibilidad** de **preferencia**. Emitir un conjunto pequeño de targets/rutas realizables de opciones ya soportadas, en vez de ocultar la selección en la heurística anterior.

- Explorer: hasta tres rutas de escape factibles y distintas, con urgencia derivada del **mismo estado de amenaza**. Entre alternativas de igual clase/prioridad/urgencia, la utilidad del genoma decide progreso, oclusión, salidas y riesgo. No forzar igualdad entre clases con distinta prioridad ni permitir avanzar a base contra un guard de amenaza.
- Guardian: exponer alternativas de búsqueda o presión realmente ejecutables. Comparar portales/intercepciones/defensa solo cuando tengan realizador y evidencia de guards. La existencia de un `OptionKind` en el enum no demuestra su integración.
- Priorizar este conjunto mínimo. Si añadir opciones semánticas exige implementaciones nuevas, diferirlas. No emitir propuestas ficticias solo para aumentar `choice_rate`.
- Mantener keys estables por tipo/target y lifecycle coherente; reevaluar una propuesta no debe crear cancelaciones/reenvíos innecesarios. Revisar generaciones de lease y comandos obsoletos.

**Generador de alternativas propuesto para el MVP — pendiente de implementación:**

1. Fijar un snapshot coherente de pose, track, topología, mapa y fase. Exigir fuente autorizada, antigüedad/epochs/versiones válidas. Construir/reutilizar árboles de rutas solo por aristas OPEN y recortar desde la pose según el contrato actual; no admitir UNKNOWN como escape.
2. Enumerar targets existentes y realizables del catálogo del perfil: portales/frontiers conocidos y no completados; objetivo de base solo si está autorizado y sus guards permiten avanzar. Un horizonte de ruta fijo y límites de nodos/aristas del grafo acotan el catálogo. Versionar esos límites antes del piloto; no dependen del genoma, de la distancia al rival como score ni de información oculta. Un catálogo vacío produce fallback explícito.
3. Agrupar el catálogo por primera salida/corredor dirigido del plan recortado desde la pose. Recorrer grupos de forma intercalada para no gastar todo el presupuesto en muchos targets de la misma rama. Dentro de cada grupo usar orden estable por hash de identidad geométrica y seed de catálogo congelada, ajena a la seed de mutación; registrar su derivación. Revisar rutas en ese orden hasta un máximo `B_paths` (piloto propuesto: 12). Otras operaciones tienen límites explícitos de tamaño/trabajo: limitar propuestas a tres no basta para acotar una búsqueda sobre un grafo ilimitado.
4. Para cada ruta revisada, verificar realizabilidad, recorte, precondiciones y evidencia de guards sobre el snapshot. Obtener LOS/escape y clase de opción de esa ruta, conservando el orden normativo. Para las opciones de escape ante la misma amenaza, compartir urgencia basada en el mismo track/instante, sin puntuar de nuevo cada target mediante `distance_from_rival - 0.25 * path.cost`. Calcular features de cada alternativa superviviente, no solo de un ganador geométrico previo.
5. Deduplicar por recorrido dirigido canónico y efecto de ejecución. Normalizar puntos/segmentos redundantes y distinguir rutas paralelas por sus edge IDs; endpoints diferentes que producen el mismo plan y efecto no cuentan dos veces. No declarar equivalentes dos rutas solo por compartir su primer tramo: pueden divergir en el siguiente cruce o producir efectos distintos. Registrar firma de ruta, target/efecto y motivo de cada deduplicación; confirmar diversidad mediante el realizador.
6. De las supervivientes, admitir como máximo `K_proposals=3` en el piloto. El cap se reparte primero entre ramas distintas en el orden estable anterior y luego entre rutas distintas de las ramas ya representadas. Prioridad/guards pueden excluir legítimamente opciones; **la utilidad, los pesos y el score geométrico anterior no pueden recortar el conjunto antes del selector**. Cualquier cap pierde alternativas y puede sesgar la búsqueda: registrar revisadas, rechazadas y omitidas, comparar caps/seed de catálogo en training y no afirmar que se conserva todo el ranking de la enumeración exhaustiva.
7. Entregar al selector propuestas y planes con identidad consistente. La estructura actual guarda paths por `target_node_id` y la key por tipo/target: el primer MVP representa una ruta canónica por target distinto. Dos rutas al mismo target requieren ampliar ese contrato con identidad de ruta y verificar que el ejecutor no replantee ambas hacia el mismo camino; quedan fuera del MVP. Revalidar que el plan ejecutado conserva la alternativa elegida tras cambios de snapshot, con cancelación/replanificación cuando corresponda.

Determinismo significa misma salida con el mismo snapshot, catálogo/config/seed y estado de ejecución; no obliga a igual salida después de observaciones diferentes. El generador propone, el selector expresa preferencia y el realizador/supervisor pueden rechazar: no fabricar `feasible=true` sin comprobar las precondiciones. Guardian reutiliza este esquema solo para targets/opciones de su rol ya realizables.

Pruebas necesarias: un cambio de pesos cambia una selección conocida entre alternativas admisibles **manteniendo idéntico el conjunto generado sobre un snapshot fijo**; guards/urgencia siguen prevaleciendo; target rechazado no llega al ejecutor; dwell no impide preempción obligatoria; ninguna alternativa usa estado verdadero o atraviesa un muro. Añadir casos de catálogo vacío/una/múltiples ramas, rutas equivalentes y paralelas, cap alcanzado, seed estable, nodo renumerado con identidad geométrica equivalente y plan realizado distinto del propuesto. La renumeración no debe sesgar el catálogo hacia IDs accidentalmente bajos.

**Salida:** al menos un caso reproducible y no trivial de cambio de decisión por rol para los genes que se pretenden buscar; reporte de frecuencia en episodios representativos. Genes sin efecto quedan congelados. Si no hay realizadores/elección suficiente, cerrar el diagnóstico y usar calibración táctica registrada; no iniciar evolución de pesos.

### 3.3 Espacio de búsqueda implementado y extensiones

El `TacticalGenome` v1 contiene pesos G/E normalizados y con signo restringido, más **histéresis y dwell compartidos**. No incluye horizonte, umbral de amenaza, lookahead ni velocidad.

| Parámetro | Tratamiento en este cierre |
| --- | --- |
| Pesos G/E existentes | Mutables solo si tienen efecto medido; proyección actual y masa de coeficientes ≤1 |
| Histéresis/dwell v1 | Búsqueda acotada a límites operativos preinscritos; los máximos de schema (1 y 60 s) no son recomendaciones |
| `opponent_threat_distance_m` | Posible extensión táctica pequeña, si la sensibilidad demuestra utilidad. Exige schema, carga/exportación, hash y pruebas nuevos; fijo mientras no exista esa extensión |
| Lookahead y umbral de giro suave | Calibración de navegación separada, congelada antes de comparar tácticas; no son genes actuales |
| Velocidad, frenado, radio, márgenes, cobertura, timeout de seguridad | Perfil físico/sintético fijo por experimento. No genes, ni recompensa intercambiable por riesgo |
| Horizonte, capture/LOS/llegada, sensor/verdad, prioridad dura | Contratos del entorno y runtime; no se mutan |

Conservar un artefacto que empaqueta ambos roles. `theta_G`/`theta_E` designan sus bloques, no dos formatos ya existentes. Para optimizar por rol, mutar solo su bloque y congelar el otro. No atribuir mejoras a un rol cuando cambian simultáneamente histéresis/dwell comunes: tratarlos como ablation separada o versionar parámetros por rol con migración explícita.

## 4. Reglas y objetivo competitivo

La [transcripción suministrada](Регламент%20HSL26%20-%20v06092026.md) confirma dos etapas, intercambio de roles, suma de puntos, captura con `d < 0.45 m`, eje X de Guardian a ≤45° y LOS sin obstáculos; llegada por contacto del contorno del Explorer con el contorno de la zona. Timeout sin captura favorece Explorer.

El perfil vigente de arquitectura R7 interpreta **600 s totales, 240 s de preparación y 360 s activos**; confirmar con organizador cualquier ambigüedad. Un fixture de 600 s con freeze de 0.15 s no reproduce ese formato. Reloj, simultaneidad de eventos, zonas y memoria entre etapas se fijan antes de evaluar.

El mapa se anuncia el día 1 y permanece fijo, pero §5.2 prohíbe fijar en el software información ligada a particularidades previamente desconocidas. **Mapa fijo no equivale a permiso para codificarlo**. Registrar procedencia y autorización de `approved_map`, zonas y actualizaciones; `observed_map` se construye de observaciones propias. Sin zona aceptada no se inventa la base rival ni un terminal de llegada.

### 4.1 Métricas y coste de censura

Hasta conocer puntos oficiales, `official_score=null`. Separar outcome, objetivo surrogate versionado y diagnósticos:

1. Métrica primaria pre-oficial: victoria por rol y diferencia pareada frente a baseline, con reporte separado de captura, llegada y timeout. No llamar a su promedio “puntos oficiales”.
2. Tiempos a captura/llegada, progreso, LOS, replanificaciones, CPU, intervenciones y diversidad son diagnósticos. Un episodio truncado antes del horizonte no es una victoria por supervivencia.
3. Para screening corto, usar casos que fuerzan elecciones y ventanas locales predefinidas. No promover un candidato por resultados censurados. Finalistas se ejecutan al horizonte completo del perfil aceptado.
4. El benchmark actual usa `s * exp(-0.01 * t)`, que favorece unas duraciones aun sin tabla oficial. Mantenerlo como perfil histórico; para nuevo objetivo binario sin descuento, versionar perfil y configurar beta=0 donde corresponda. El trainer actual fija beta=0.01: necesita cambio explícito, no una opción CLI supuesta.
5. No cambiar objetivo después de mirar candidatos para rescatar una mejora. Al llegar la tabla oficial, fijar nuevo score profile y reevaluar políticas congeladas; conservar resultados anteriores bajo su identidad.

Para aprendizaje por opciones, puede mantenerse descuento temporal conforme a P6.2; no inferir que ese descuento es el objetivo deportivo. Shaping queda fuera del MVP. Si se añade, requiere contrato separado y validación: en un modelo Markov/SMDP compatible, la forma potencial `exp(-beta*tau)*Phi(s') - Phi(s)`, con tratamiento terminal correcto, puede preservar el objetivo base; no garantiza ranking de genomas en observación parcial, self-play cambiante o representación agregada incorrecta. Véase [Ng, Harada y Russell](https://ai.stanford.edu/~ang/papers/shaping-icml99.pdf).

### 4.2 Completar el árbitro aplicable

Antes de optimizar “llegar a base”, conectar zona autorizada, evento de contacto de contornos y clasificación de llegada al benchmark integrado. El fixture no autoriza llegada oficial por alcanzar un nodo. Ejercitar llegada antes/después de captura, evento entre ticks, timeout, LOS obstruida, eventos ambiguos y consistencia de los dos managers. El árbitro recibe truth de simulación y etiqueta `SIM_TRUTH`; la política nunca lo consume.

Si solo se dispone del perfil actual CAPTURE/TIMEOUT, limitar el experimento a persecución/supervivencia y declarar ese límite. No seleccionar así una política de infiltración completa para G5.

## 5. Una sola geometría HGW con cómputo acotado

### 5.1 Artefacto y precisión

Construir offline **un prior Hermite-GPIS-W** del ensamblaje visible real, con soporte, normales, anclas si aplican y `T_B_M` calibrado. Compartirlo si ambos robots tienen el mismo ensamblaje; diferencias físicas verificadas requieren variantes por geometría, no por rol o por fase.

Online se registra una nube parcial contra ese prior; no se reentrena/reconstruye la superficie en cada frame. El centroide de un cluster no es `base_link`. Un prior correcto puede reducir sesgo de vista parcial, pero **no garantiza error milimétrico**: geometría, extrínsecos, soporte, ruido, oclusión y movimiento dominan su aceptación. Validar error del origen y falsos matches en vistas/bags disjuntos, especialmente cerca de captura.

El extractor SIL actual agrega el radio a cada retorno antes de promediar; no es un centroide crudo ni un reemplazo aceptado para el robot físico. Un fallback puede entregar posición aproximada con sesgo/covarianza y edad declarados; nunca certificar captura por una estimación superficial no calibrada.

### 5.2 Un pipeline y un worker por robot

`nube con tiempos/TF → deskew ego → veto estático con incertidumbre → candidatos → registro HGW acotado → asociación/EKF → track/creencia`.

- La rama de obstáculos de seguridad conserva retornos físicos completos; quitar fondo del detector semántico no elimina paredes del supervisor.
- Deskew compensa movimiento del observador, no automáticamente el del rival. Acotar ventana/velocidad del oponente y declarar error residual; ego quieto no elimina la distorsión del blanco.
- Mapas ortogonales ayudan al veto, pero no borran drift ni accesorios móviles. Usar tolerancia basada en incertidumbre de pose/mapa/rango y no descartar al rival junto a una pared por ensancharla a ciegas.
- Prior, coeficientes y estructuras se cargan una vez. ROI por track con expansión por incertidumbre; límite de puntos/clusters, muestreo reproducible y diversidad de superficie. No depender solo de DBSCAN sin coste acotado.
- Seguimiento con predicción de posición; yaw solo se usa como evidencia si es observable. El EKF SIL tiene cuatro estados y `yaw_valid=false`: no proporciona un yaw fiable para warm start. Registrar hipótesis de yaw previas o una búsqueda acotada con covarianza/rechazo apropiados.
- Adquisición/reenganche usa más inicializaciones/iteraciones **del mismo modelo**; seguimiento usa menos trabajo cuando residual/soporte/asociación lo permiten. No prescribir 1–2 iteraciones sin medir convergencia.
- Una cola latest-only y un trabajo en curso por robot; timestamps conservados. Descartar resultados viejos tras pérdida, cambio de epoch/modelo/rol o reset. Iteraciones acotadas no garantizan deadline: comprobar tiempo monotónico entre lotes e implementar aislamiento/cancelación cuando una operación no sea interrumpible.
- No ejecutar entrenamiento, BLAS sin límite de hilos ni ocho ajustes paralelos junto al control. Medir bajo carga conjunta driver/mapa/track/planner/supervisor y carga térmica sostenida.

El código actual de `HermiteGPIS.evaluate()` recorre observaciones y usa un solve de varianza por punto; el kernel de soporte compacto **no convierte esa implementación densa en evaluación O(1)**. Medir antes de optimizar. Batchear consultas, resolver triangularmente, cachear o indexar soporte son candidatos de optimización con comparación numérica de media/gradiente/varianza; no omitir incertidumbre para obtener una latencia atractiva.

### 5.3 Frecuencia y degradación por rol

Guardian exige precisión del origen al aproximarse y para interceptar; no requiere yaw de Explorer para el predicado de captura. Explorer puede aceptar un track menos preciso a distancia, pero el error importa cerca, en cruces y bajo oclusión. Compartir detector/modelo permite reducir cadencia o trabajo en Explorer según error y deadline; **no desactivar HGW por rol sin verificar que conserva decisiones**.

Freeze inmóvil permite acumulación pasiva y adquisición, si hay visibilidad. No presupone ver al rival ni que este avance antes de finalizar preparación. El MID-360 cubre 360° de azimut; rotar no revela el espacio detrás de paredes y añade deskew/confinamiento. Una rutina activa se justificaría solo por ganancia medida y autorización de política.

Las cifras del auditor “12–18 ms”, “70–90 ms”, “<3% CPU” y tamaños 40/200 son hipótesis, no benchmarks del ensamblaje. Fijar presupuesto desde deadline de control, antigüedad admisible y error de seguimiento, con p50/p95/p99, máximos observados y tasa de timeout. Informar CPU en segundos de CPU/segundo de pared y concurrencia, no porcentajes ambiguos.

La identidad `BOXNUC7I7BNH → i7-7500U/15 W` del auditor es incorrecta: el [TPS Intel NUC7i7BN](https://www.intel.com/content/dam/support/us/en/documents/mini-pcs/nuc-kits/NUC7i5BN_NUC7i7BN_TechProdSpec.pdf) identifica i7-7567U y 28 W. Confirmar inventario real, RAM y límites térmicos antes de fijar presupuesto; la ficha no prueba rendimiento del software.

## 6. Seguridad y competitividad sin cambiar el contrato

“No hay penalización por contacto estático” es una regla de puntuación, no prueba de que rozar sea inocuo. Puede bloquear la base, dañar el montaje o desplazar elementos críticos que provocan reinicio. No incorporar contacto intencional al aprendizaje de este cierre.

En `P56Profile`, `clearance_margin_m=0.20` **incluye** el radio sintético 0.15; la validación rechaza márgenes menores al radio. No sumar otra vez el radio al aplicar esa misma distancia ni reducir el margen total a 0.06–0.08. El YAML físico declara radio 0.22, mientras el SIL usa 0.15: esa diferencia debe resolverse antes de transferir y evaluar viabilidad de captura.

Registrar la convención de clearance para avance/arco/rotación (centro, borde, corredor, incertidumbre y cobertura). Con margen total 0.20 y clearance frontal 0.35, quedan 0.15 para frenado: con v=0.20, tau=0.075, b=0.85 y aceleración de retardo cero, `d_stop≈0.0385 m`. No se deduce velocidad cero de sumar radio y margen otra vez. Tampoco una separación lateral equivale a clearance frontal.

Optimizar competitividad mediante ruta y velocidad regulada admisible; diagnosticar `BRAKING_INFEASIBLE`, cobertura, arco, ALIGN, latencia y watchdog por separado. Calibrar cambios de footprint/margen/límites con evidencia P1/P2 y versión propia **antes** de congelar el experimento. No ajustarlos buscando reward.

Comprobar intervalo de captura sin contacto: suma de envolventes físicas, error del origen relativo y sobreavance durante latencia deben caber antes de 0.45 m. Dos radios 0.22 dejan solo 0.01 m nominales; el disco conservador puede diferir de la huella real. Medir geometría y control de standoff antes de prometer captura. El árbitro simulado puede detectar captura mientras la trayectoria ya produjo contacto: es fallo a analizar, no éxito aceptable.

## 7. Bancos y diversidad con tiempo limitado

Mantener training, validation y held-out congelados antes de selección; usar los bancos existentes como desarrollo cuando su procedencia no habilite más. Variar aquello que cambia las decisiones: cruces con dos escapes factibles, occlusión, callejones, defensa/intercepción viable, rival quieto, rutas bloqueadas, pérdida/reenganche y meta desconocida. No ampliar automáticamente a decenas de familias.

Dos objetivos separados:

- **Robustez pre-evento:** unas pocas familias distintas, configuraciones de rival y perturbaciones basadas en medición; validar transferencia de táctica.
- **Adaptación al evento:** solo con entradas autorizadas. Mapa fijo observado/aprobado y escenarios de starts/rivales/sensores permitidos. En un único mapa no afirmar generalización entre mapas; agrupar por escenario/condición independiente y conservar validación no usada.

Un seed distinto con sensor sin ruido y política determinista puede dar la misma trayectoria. No contar esas repeticiones como evidencia independiente. Aleatoriedad de mutación no es aleatoriedad de entorno. Registrar exactamente qué randomiza cada seed y conservar common random numbers dentro de cada comparación pareada.

Banco de rivales mínimo: baseline fijo y, si existen ejecutores verificados, uno o dos comportamientos distintos por rol. Nombres “rush”, “patrulla” o “intercepción” no crean implementaciones. Reutilizar realizadores comprobados; no construir seis agentes completos antes de probar sensibilidad.

## 8. Búsqueda pequeña y coevolución alternada

### 8.1 Primer piloto

Congelar entorno, score profile, genes efectivos y rivales. Proponer baseline más **hasta cuatro candidatos nuevos por rol**, centrados en genes sensibles; probar uno a la vez o diseños conjuntos pequeños, no producto cartesiano arbitrario. Un LHS/random design no obliga a desarrollar CMA-ES/BayesOpt.

Evaluar `G_i × E_base` y `G_base × E_j`, más banco congelado cuando el presupuesto lo admita. Compartir baseline vs baseline solo con igual perfil, mundo, seed, identidades y hashes. No sumar retornos de la misma partida cero-suma: se anulan. La comparación role-swapped implementada ya evita esa cancelación, aunque no equivale a un partido físico de dos etapas.

El trainer existente usa una población de genomas conjuntos, defaults de tres generaciones, seis individuos, sigma=0.08, freeze de 0.15 s y beta=0.01. No implementa este piloto por rol ni la liga. Adaptarlo incrementalmente o hacer un runner explícito; no presentar un nuevo CLI o formatos propuestos como disponibles.

### 8.2 Alternar solo después de señal

1. Optimizar Guardian con Explorer/banco **congelados** durante toda la ronda; congelar finalista G.
2. Optimizar Explorer contra ese G y el banco; congelar finalista E.
3. Comparar ambos contra baseline y versiones anteriores mediante matriz cruzada. Repetir como máximo una segunda ronda si queda presupuesto y mejora el rendimiento contra referencias.
4. Archivo inicial: baseline y hasta dos adversarios históricos por rol, retenidos por diferencia de conducta y capacidad de exponer fallos, no por antigüedad sola.
5. Actualizar archivo al terminar ronda, nunca cambiar la distribución de rivales a mitad de la evaluación de candidatos.

El archivo mitiga olvido/ciclos, pero no prueba convergencia; la literatura documenta también pérdida de estrategias a largo plazo ([Andersen, Stanley y Miikkulainen](https://www.cs.utexas.edu/ftp/techreports/tr02-32.pdf)). Medir ventaja frente a archivo y baseline, no únicamente self-play actual. El campeón contra un rival no domina necesariamente todos los demás.

No se necesita resolver un equilibrio de Nash para este cierre. Retener diversidad de conductas, evitar que la media oculte un fallo severo de un rol y elegir una política determinista por rol para despliegue.

## 9. Coste real, optimización y reglas de parada

### 9.1 Presupuesto antes de lanzar

Fijar `B_total` en horas disponibles, responsables y cutoff del evento; no asumir un plazo que no está comunicado. Reservar primero integración, HGW/calibración según perfil y transferencia/release. Propuesta de reparto adaptable:

| Trabajo | Fracción inicial del tiempo de ingeniería restante |
| --- | --- |
| Integración/conformidad/baseline G4 | 40% |
| Percepción, calibración y viabilidad de captura/navegación | 20% |
| Diagnóstico y búsqueda táctica | 20% máximo |
| Validación/transferencia/imagen/ensayo/rollback | 20% reservado |

Si hay dependencias sin resolver, reducir búsqueda, no la reserva de evaluación. CPU y horas humanas se presupuestan por separado. Primera entrega de diagnóstico/perfilado: máximo dos horas de ingeniería como timebox propuesto; si no identifica acción concreta, documentar bloqueo y devolver esfuerzo a integración.

Medir episodios completos sin profiler/telemetría detallada y un episodio instrumentado con overhead declarado. Reportar segundos simulados/pared, episodios/hora y coste de baseline, candidato y validación. La igualdad de terminales no acredita aceleración ni equivalencia de decisiones.

Para el runner role-swapped, con C candidatos nuevos y K combinaciones independientes de escenario/rival/seed:

`N_matches = K * (1 + 2*C)`

si todos usan el mismo baseline y se reutiliza su referencia dentro de ese lote. Con poblaciones por rol de C_G/C_E candidatos nuevos:

`N_matches = K * (1 + C_G + C_E)`

bajo ese mismo supuesto; rivales adicionales requieren referencias adicionales. Contar ejecuciones del simulador, no filas `EpisodeResult`. Ejemplo: cuatro por rol y seis combinaciones requieren 54 matches, antes de validación, archivo o transferencia. “50 simulaciones/una hora” no es una garantía.

Estimar `T_lote ≈ sum(costes medidos por estrato) / speedup_workers_medido + overhead`; reservar 25% de contingencia de CPU. No dividir por número de workers sin benchmark. A 2.5x, un timeout de 600 s de fixture ronda cuatro minutos por match: 54 matches serían unas 3.6 horas en un worker, por extrapolación, no medida de este lote. Presupuesto insuficiente implica reducir candidatos/rondas, no simular victorias truncadas.

Parar una ronda si: no hay efecto de genes tras diagnóstico; todos los candidatos son excluidos; no hay diversidad conductual nueva; no mejora frente a referencias congeladas; coste proyectado supera cuota; o llega cutoff. Guardar motivo y todos los resultados.

### 9.2 Optimización del simulador existente

Primero perfilar política, routing, rayos, percepción residual, seguridad de arcos, referee y logging. Eliminar recomputación demostrada, batchear y limitar telemetría de entrenamiento conservando manifests. Mantener traza completa reproducible para diagnóstico. No crear otro simulador para eludir ese trabajo.

El [perfil diagnóstico del 2026-10-02](../artifacts/reports/coevolution_audit_20261002/profile_cumulative.txt), con overhead de cProfile, contó 24,619 llamadas a `_path_to_node` y 16,813 a `dijkstra_tree` en 986 pasos. Tiempo acumulado: rutas ≈25.01 s, árboles ≈22.98 s, clearance de arco ≈14.34 s y raycaster ≈4.31 s; son tiempos inclusivos **solapados**, no porcentajes sumables ni throughput. Priorizar reutilización de árboles por fuente/snapshot y agrupación de destinos; investigar si la LRU de dos fuentes pierde trabajo al ensayar distintos starts. Verificar recorte desde pose, rutas OPEN, rechazo UNKNOWN, cambios de topología y determinismo antes de ampliar/cachear o simplificar. Después medir los barridos de arco; no retirarlos para acelerar.

Táctica 2–5 Hz y control 20 Hz pueden separarse como **cambio posterior medido**: conservar opción/ruta y reevaluar con TTL/invalidation por track, mapa, goal, fase, lease, progreso o guard. El supervisor, cobertura fresca y eventos terminales no se espacian para ahorrar CPU. Las frecuencias se interpretan según §9.3, sin esperas de tiempo real en lotes cinemáticos.

Hay una discrepancia concreta: `config/tactics.yaml` de bringup declara planificación 5 Hz, control 20 Hz y `supervisor_rate_hz=40`, mientras arquitectura §14 y TS §5 fijan **50 Hz como objetivo inicial del supervisor de seguridad/publicación final**. No hay evidencia para llamar al primero “supervisor táctico” y al segundo “supervisor de seguridad” como si fueran procesos diferentes. Antes del runtime integrado, enlazar cada parámetro al timer/owner efectivo y resolver el perfil: usar 50 Hz como objetivo de diseño vigente o justificar/versionar otro valor con presupuesto de respuesta y evidencia de gates. No cambiar aquí el YAML ni afirmar que una tasa objetivo sea una garantía de tiempo real. La prioridad 50 del mux en `hardware.yaml` es un entero de arbitraje, no Hz.

Toda optimización conserva perfil o declara uno nuevo y ejecuta comparación diferencial de opciones, rutas, comandos, terminales y contacto. Rayos sectorizados, menos observaciones o planta a otro paso cambian fidelidad y exigen validación aparte. Evitar inversiones de ranking cerca de captura/occlusión. Un único caso idéntico no prueba equivalencia global.

Reabrir backend ligero solo si la referencia optimizada sigue fuera de presupuesto, la señal está demostrada, el ahorro esperado amortiza construcción **y** validación antes del cutoff, y existe tiempo para confirmar finalistas en referencia. En ese caso: paquete aislado, sin ROS/dispositivos/truth de política, `promotion_eligible=false`, fidelidad explícita y contraste de ranking/terminales. Las antiguas metas 10x/20x y L0–L5 no son compromisos actuales.

### 9.3 Frecuencia simulada, reloj de pared y latencia

**Un ciclo a 50 Hz simulados no limita el lote a 50 ejecuciones por segundo real.** Significa una evaluación cada 0.020 s del mundo virtual; a 40 Hz, cada 0.025 s. Un runner offline avanza su reloj tras calcular el paso, sin `sleep()`, rate limiter ni sincronización deliberada con tiempo real. Mantener resolución/frecuencia virtual preserva la semántica temporal; la aceleración procede de resolver esos pasos más deprisa, no de omitirlos.

| Magnitud | Unidad y uso |
| --- | --- |
| `dt_sim`, periodos de planta/sensor/control/seguridad/táctica | Segundos simulados; definen orden y resolución del mundo |
| `T_sim`, `T_wall` | Duración virtual realmente ejecutada y duración monotónica de pared del episodio; distinguir preparación/activo/inicialización |
| `RTF = T_sim / T_wall` | Factor respecto a tiempo real de un episodio, mayor o menor que 1; no es speedup frente a otro backend |
| `updates_per_wall_s` | Número real de ejecuciones de un subsistema dividido por pared; para un ciclo periódico sin drops, aproximadamente `f_sim * RTF` |
| `backend_speedup` | `T_wall_reference / T_wall_candidate` para igual workload/semántica y host; diferente de RTF |
| Latencia de sensor/acción modelada | Segundos simulados entre adquisición, entrega, decisión y efecto; afecta al agente y sus leases |
| Coste CPU/deadline operativo | Pared/steady del proceso real; mide si el software puede desplegarse bajo carga física |

Ejemplo aritmético, **no benchmark**: 600 s simulados con un ciclo de 50 Hz requieren unas 30,000 evaluaciones por robot si no hay terminal temprano. Con coste total medio de 0.2 ms de pared por avance global de 20 ms simulados, el episodio tarda unos 6 s y RTF≈100; el ciclo ejecuta unas 5,000 evaluaciones por segundo real por robot. Con 2 ms por ese mismo avance global, tarda unos 60 s y RTF≈10. El coste global incluye ambos robots, sensores, seguridad, árbitro y registro: no multiplicar una latencia aislada de función como si fuera todo el episodio. Startup, variación de carga y terminales modifican esas cifras.

El runner actual ya es no sincronizado: `benchmark.py` llama repetidamente a `TwoRobotMatch.step(dt)` y `match.py` incrementa timestamps por `dt`; no espera ese intervalo en pared. Con `control_period_s=0.05`, la política y su evaluación integrada de seguridad se invocan a 20 Hz **virtuales** por robot; no reproduce un proceso independiente de supervisor a 40/50 Hz. El RTF observado de aproximadamente 2.3x–2.8x proviene del trabajo por tick, no de una espera intencional. Eliminar `sleep` no ofrece un ahorro nuevo en ese camino.

Si se implementan varias frecuencias, usar deadlines absolutos en tiempo virtual y avanzar al siguiente evento relevante (o subpaso físico acotado), integrando planta y comprobando eventos entre ellos. Orden causal y desempate son versionados. No redondear 40 Hz a ticks de 20 ms: 25 ms no es múltiplo de 20 ms; un calendario de eventos debe preservar ambos periodos. Una actualización no renueva timestamps de observación vieja. Mantener el árbitro continuo/subdivisión y las comprobaciones de contacto aplicables; cumplir 50 Hz no impide por sí solo saltarse una captura entre ticks.

Separar dos perfiles de tiempo:

1. **Lote de investigación:** estados, edad de medidas y leases usan tiempo virtual; retrasos, jitter, pérdida, colas y sobrepasos de cómputo se inyectan explícitamente en ese dominio con procedencia. Si se modela duración del cálculo, el resultado se entrega después de la latencia virtual declarada, nunca en el pasado; la planta continúa durante esa demora. El tiempo de pared mide el coste del experimento. Perfilado/velocidad del host no deben modificar inadvertidamente la táctica.
2. **Conformidad de runtime/robot:** watchdog, tiempos de proceso y deadlines steady se miden realmente bajo carga, con sensores/driver/grafo y reloj definidos. Un RTF alto o un `PASS` de timing sintético no acredita esos tiempos. ROS/MVSim no acelera automáticamente todos los timers/colas por publicar `/clock` más rápido: documentar qué utiliza ROS time y qué steady, y probar esa configuración por separado.

En la política SIL actual, `SafetySupervisor.evaluate()` recibe `received_steady_ns=0` y `now_steady_ns=1`: comprueba un coste sintético de 1 ns, **no mide la duración real del callback**. Los tests pueden verificar rechazos por overrun con tiempos inyectados; para demostrar deadlines de despliegue hace falta el wrapper operativo y medida independiente. Conservar esta limitación en los manifests; no escalar ni desactivar watchdog físico para que un experimento parezca rápido.

Un backend ligero podría alcanzar mucho mayor RTF si reduce el coste global conservando lo necesario para selección; no existe garantía de un factor concreto. Replanificar menos, simplificar sensores o retirar capas cambia trabajo **y posiblemente semántica**, por lo que requiere contraste de rankings/terminales. Ejecutar sin esperar es una condición de lote que la referencia ya cumple; no justifica por sí misma construir otro simulador.

## 10. Estadística, selección y transferencia

Antes del piloto, registrar métrica primaria, unidad independiente, rivales, splits, perfil, seed derivation, tamaño de evaluación, confianza, mínimos de mejora, tolerancia de regresión por rol y criterios de seguridad. Entrenamiento puede usar screening adaptativo; confirmación/held-out usa plan fijado y no inspecciones repetidas hasta obtener significación.

El evaluador actual requiere ambos roles por pair key, aplica intervalo t a deltas balanceados y excluye candidatos con contacto/violaciones, demasiados overrides o más overrides que baseline. No implementa intervalos por familia ni un evaluador independiente por rol. Cambiar estos contratos requiere versión y pruebas, no solo editar el plan. No retirar una exclusión para rescatar un candidato.

Con mapas/condiciones repetidos, agrupar la incertidumbre por unidad independiente; miles de ticks o seeds sin efecto no aumentan n. Si hay muy pocos clusters, declarar incertidumbre insuficiente: un bootstrap no fabrica evidencia. Para tasa de victoria usar comparación pareada compatible con los outcomes; diferencias por mapa/rol y peores estratos se reportan además de la media. Un IC [0,0] en fixtures idénticas describe esas muestras, no equivalencia universal.

Seleccionar finalistas en training, escoger artefacto en validation y congelar antes del held-out único. Ante fracaso, baseline o evaluación nueva con hold-out nuevo y procedencia; no ajustar y reutilizar el anterior. Mínimos de mejora se fijan según efecto competitivo útil y precisión piloto, no un número de partidas arbitrario.

Ruta de aceptación:

| Paso | Evidencia | Decisión |
| --- | --- | --- |
| Diagnóstico pre-G4 | Sensibilidad, coste, causales; desarrollo sintético | Corregir/fijar genes; no claims empíricos elegibles |
| Training con G4 por perfil | Datos/bancos aceptados, búsqueda frente a referencias | Finalista G/E o baseline |
| Validation → held-out único | Candidato congelado, comparaciones pareadas, incertidumbre y límites por rol | Solicitar revisión G5 o rechazar |
| Replay/3D/MVSim/ROS | Track/origen, latencia, discrepancias de decisiones y seguridad de grafo | Transferencia del perfil correspondiente; sensor genérico no acredita MID-360 |
| Robot/imagen final | Calibración física, dos etapas autorizadas, ambos roles, cold start sin red y rollback | G0–G4/G6 físicos según matriz; aprendizaje online desactivado |

Si el candidato se degrada con mayor error de percepción, reentrenar bajo un perfil medido nuevo y reiniciar selección. No retocar política en hardware y conservar el mismo hash/aceptación.

## 11. Datos y entregables de aprendizaje

Guardar transiciones de opción con rol, etapa, estado/features y schemas, propuesta/cohorte/target, inicio/fin/duración, outcome, cancelación, intervención de seguridad y censura; reward de perfil, terminal, rival, map/seed bank, sensor/dinámica, hashes de código/config/política/HGW y epochs. No usar truth como feature: el evaluador puede conservarla en registro separado para medir error y resultado.

Esquema de reporte mínimo propuesto, **pendiente de implementación**:

`run_id, commit, backend, evaluation_profile_id, eligibility, split_manifest_sha256, role_policy_ids, opponent_policy_ids, score_profile_id, official_score, seed_derivation, simulated_s, wall_s, rtf, timing_fidelity_profile, modeled_latency_profile_id, terminal_kind, completed, censored, exclusion_reason, safety_counts, N_active, N_available, N_choice, proposal_availability_rate, choice_coverage_rate, conditional_choice_rate, no_proposal_rate, mutation_effect_rate`.

Añadir periodos virtuales por subsistema y counts reales, campos de truncamiento/seed del catálogo y firma de plan/efecto. No relabelar retrospectivamente los conteos históricos como métricas de un schema nuevo.

Preservar episodios fallidos/excluidos. Excluir del fitness con estado/razón finitos, no fitness infinito ni silencio. Los bancos de rivales necesitan identidad en pair key/provenance; el esquema actual no basta para comparar indiscriminadamente adversarios distintos.

P6.1/P6.2 siguen siendo útiles: logs inmutables → estimación Dirichlet/transiciones → valor de opciones con duración y terminales. No introducir fusión de estados sin comprobar compatibilidad de acciones, reward y kernel descontado. Tablas de valor son inicialmente análisis offline/shadow; conectarlas al selector exige interfaz, schema, ablation y nueva evaluación. Una exportación vacía o las pruebas de algoritmo no constituyen aprendizaje empírico.

Salida por rol: baseline/candidato con pesos, parámetros permitidos, manifiesto y compatibilidad; reporte de entrenamiento/validación y matriz de rivales. Artefactos de política siguen `manifest.json`, `parameters.json` y `.npz` no ejecutable cuando aplique; `online_adaptation=false`. El runtime no importa trainer y carga únicamente política aceptada o baseline firmado.

## 12. Orden de trabajo y cierre verificable

Estos paquetes ordenan el trabajo; **no crean nuevos gates normativos**.

| Orden | Entrega concreta | Condición de salida |
| --- | --- | --- |
| 1 — inventario y plan congelado | Estado G4/R0–R8/P4, reglas/zonas/tiempos, presupuesto, baseline hashes y splits | Dependencias y cutoff visibles; sin estado PASS inventado |
| 2 — diagnóstico acotado | Conteos de elección, perturbaciones controladas y perfilado con coste | Genes efectivos identificados; acción de corrección o stop documentado |
| 3 — elección y semántica | Alternativas realizables pequeñas, lifecycle y terminales del perfil | Casos de sensibilidad por rol, invariantes/truth negativos y matriz G4 revisada |
| 4 — datos/búsqueda | Logs aceptados tras G4, piloto por rol, archivo mínimo; segunda ronda opcional | Candidato reproducible o baseline retenido con causas |
| 5 — evaluación congelada | Validation, held-out, incertidumbre, regresiones, transferencia | Decisión G5 revisada o falta de evidencia explícita |
| 6 — integración/release | Grafo, percepción y calibración aplicables; imagen, ensayo y rollback | Puertas del plan maestro y G6 según entorno |

Lista de cierre:

Comprobaciones disponibles ahora (desde la raíz; resultados de desarrollo):

```powershell
python -m pytest hsl_core/tests/test_p53_tactics.py hsl_core/tests/test_p63_evolution.py hsl_core/tests/test_p42_registration.py hsl_core/tests/test_safety.py sim/kinematic/test_p56_perception.py sim/kinematic/test_p56_autonomous.py sim/kinematic/test_p63_benchmark.py -q
python artifacts/reports/coevolution_audit_20261002/review_checks.py
```

El segundo comando reproduce solo el diagnóstico de training y actualiza sus outputs locales; no ejecuta el nuevo piloto ni cambia gates. El trainer existente exige `--development-fixtures`; no usarlo como recolector elegible mientras no se implementen y acepten las dependencias descritas. No repetir poblaciones largas para diagnosticar una propuesta única.

Para cerrar la ejecución del plan:

- [ ] Baseline integrado y G4 aceptado por perfil; los bloqueos físicos permanecen explícitos.
- [ ] Generador acotado, determinista y sin preselección por utilidad; sensibilidad por rol, deduplicación y genes inactivos documentados.
- [ ] Terminales, reloj, mapa/zona y score corresponden al objetivo evaluado.
- [ ] Un modelo HGW calibrado o limitación/degradación declarada; coste bajo carga y error del origen medidos para el perfil físico.
- [ ] Presupuesto humano/CPU, cutoff y reserva de validación respetados.
- [ ] Periodos virtuales, RTF y latencias/deadlines separados; discrepancia 40/50 Hz resuelta para el runtime aplicable.
- [ ] Dataset, fallos/censuras, curva de búsqueda y torneo contra rivales conservados.
- [ ] Selección sin fuga de splits y evidencia por rol; held-out una sola vez si procede.
- [ ] Política congelada compatible y decisión de promoción/retención revisada.
- [ ] Transferencia, imagen y ensayo físico/autorizado acreditados aparte.

**Cierre de investigación** puede ser “no se detectó mejora; baseline retenido” con datos útiles y causas precisas. **Cierre competitivo** requiere el baseline/candidato aceptado y la integración/release correspondientes. No declarar ninguno solo por terminar este documento.
