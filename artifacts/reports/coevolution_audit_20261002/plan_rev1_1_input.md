# HSL26 — Plan de coevolución Guardian/Explorer y selección competitiva

**Revisión:** 1.1 · **Fecha:** 2026-10-02 · **Estado:** plan de desarrollo; no implica implementación ni cierre de gates.

**Objetivo:** aprender tácticas para ambos roles mediante rondas de persecución/evasión reproducibles y seleccionar una política desplegable por rol. La búsqueda modifica solamente parámetros tácticos versionados y acotados de opciones existentes. Reglas, seguridad, geometría, autoridad ROS y acceso a verdad permanecen fijos. La arquitectura §§11–12 y la especificación §15 son el contrato.

**Decisión de arquitectura para desarrollo:** conservar `sim/kinematic/` como simulador de referencia y crear, como línea aislada y explícitamente no promotable, un backend táctico ligero en `sim/kinematic_light/`. No se simplifica ni se sustituye el sistema existente. El backend nuevo acelera experimentos de selección táctica; no acredita seguridad, G4/G5/G6, percepción 3D, rendimiento competitivo ni preparación de hardware.

## 1. Punto de partida y condición de entrada

Ya existen `learning/transitions.py`, `dirichlet.py`, `option_value.py`, `evolution.py`, `tools/train_evolution.py` y un runner pareado en `sim/kinematic/benchmark.py`. Los experimentos P6.3 retuvieron el baseline: en las fixtures recientes las diferencias balanceadas de los mutantes fueron cero, no hubo finalista respaldado ni evaluación held-out. El [informe de runtime P6.3](../artifacts/reports/phase6/P6.3_runtime_and_learning_audit.json) mide alrededor de 2.3x–2.8x tiempo simulado en los ensayos descritos, pero esos números dependen del fixture, telemetría y host; no identifican por sí mismos qué función domina el coste. Un episodio de captura puede gastar muchos ticks haciendo evaluación repetida, y un timeout largo puede resultar más caro todavía.

La preparación del backend ligero puede empezar antes de G4 como trabajo de infraestructura y diagnóstico, siempre etiquetando sus episodios como desarrollo sintético. La recolección de datos elegibles, la promoción y cualquier afirmación de mejora requieren primero un baseline G4 aceptado para el perfil aplicable y bancos independientes con procedencia. GPIS/MVSim son puertas de transferencia y reconocimiento, no un sustituto de la velocidad del simulador cinemático.

## 2. Representación de los dos actores

Mantener dos genomas con identificadores propios, `theta_G` y `theta_E`, aunque compartan ejecutable, interfaces y reglas. Cada uno selecciona entre opciones realizables del rol y contiene pesos normalizados, horizontes tácticos permitidos, hysteresis y dwell. La versión del genoma, feature schema y registro de opciones fijan qué parámetros son mutables. No introducir nuevas opciones por mutación de números; una nueva primitiva exige diseño, realizador y auditoría aparte.

El episodio enfrenta `G_i` contra `E_j` bajo la misma configuración de mundo y semilla, con observaciones independientes y árbitro externo. También ejecutar referencia `G_i` contra `E_base` y `G_base` contra `E_j` para medir contribución por rol. Un resultado del par no demuestra cuál miembro mejoró. La transcripción del reglamento confirma dos etapas simétricas de 10 minutos, cuatro minutos iniciales sin cruzar la línea de salida, roles intercambiados y ganador por suma de puntos. Por tanto, la evaluación final empareja las dos etapas; la puntuación oficial solo se calcula cuando sus valores se anuncien y autoricen. Hasta entonces, `official_score=null` y un surrogate win/loss etiquetado; timeout sin captura favorece Explorer, sin transformar tiempo transcurrido en derrota implícita.

## 3. Diseño experimental antes de la búsqueda

1. **Bancos:** separar por unidad independiente de mapa/familia geométrica, condiciones del rival y semillas. Congelar entrenamiento, validación y held-out antes de mirar resultados. Incluir corredores, ciclos, salas, callejones, zonas no resueltas, obstáculos, occlusión y perfiles de sensor con y sin error. Evitar variantes casi idénticas en distintos splits.
2. **Baseline y oponentes:** conservar políticas deterministas G/E, agentes simples de búsqueda/escape y snapshots históricos. Crear un salón de políticas anteriores y banco fijo para impedir que dos poblaciones coadaptadas exploten un defecto mutuo.
3. **Instrumentación:** guardar features, orden prioridad/urgencia/utilidad, decisiones, targets, trayectorias, eventos, cancelaciones, intervención de seguridad, CPU y duración. Calcular diversidad de acciones, proporción de opciones factibles, distancia entre trayectorias, terminales y sensibilidad por parámetro.
4. **Diagnóstico de señal:** probar mutaciones controladas a cada parámetro y ejemplos que fuercen empates de prioridad/urgencia; comprobar que cambian decisiones cuando corresponde. Si todas las políticas empatan, revisar propuestas, guards, horizonte, escenario y surrogate. No alterar la puntuación para fabricar un gradiente.
5. **Presupuesto y horizonte:** medir throughput por episodio, distribución de tiempos de evento y tiempo de reloj. Usar episodios cortos para depuración y episodios del horizonte autorizado para evaluación; no inferir velocidad de simulación a partir de un smoke test.

## 4. Ciclo de coevolución

En la generación `t`, muestrear `G_t × E_t` con un subconjunto de parejas balanceado y reproducible, más cruces contra baseline, banco fijo y salón histórico. Usar semillas pareadas donde el escenario lo permita. El muestreo de rivales debe cubrir fuertes y débiles sin hacer depender toda la aptitud de una pareja actual. Guardar la matriz de emparejamientos y resultados, no solo la media de cada genoma.

Para cada rol, primero descartar fallos duros: comando fuera del contrato, colisión/violación de seguridad por encima del criterio preinscrito, truth leakage, versión incompatible o episodio incompleto sin clasificación. Los overrides de seguridad se reportan y se acotan por criterio; no se cambian por más reward. Entre supervivientes, ordenar por resultado por rol, robustez en familias de escenarios y peor cola; incluir incertidumbre agrupada por mapa/semilla. Retener siempre el baseline como referencia, élites por rol y diversidad suficiente para evitar convergencia prematura. Mutar con proyección a los límites publicados y semilla propia; cualquier crossover debe especificarse y probarse antes de usarlo.

Entrenar `G` y `E` juntos **no** significa usar la suma de retornos de una sola partida cero-suma como aptitud: esa suma se anula por construcción. Calcular ventajas frente a referencias fijadas y por rol, más un torneo cruzado. Revisar que cambios de Guardian no degradan la política Explorer que se enviaría con el mismo artefacto, si se empaquetan ambos roles juntos.

## 5. Selección, transferencia y promoción

| Fase | Evaluación | Decisión |
| --- | --- | --- |
| Entrenamiento cinemático | Muchas parejas de mapas/rivales/semillas, observación realista declarada | Elegir finalistas G y E y registrar curva de aprendizaje, no promover |
| Validación cinemática | Familias separadas, baseline y salón histórico, dos etapas virtuales por pareja | Seleccionar una pareja o políticas por rol; congelar parámetros e hipótesis |
| Held-out cinemático | Ejecutar una sola vez tras preregistrar tamaño, métricas e intervalos | Rechazar o solicitar revisión G5; no retocar y volver a mirar el mismo conjunto |
| MVSim/ROS | Ejecutar políticas congeladas con dinámica y LiDAR caracterizados, ambos roles y seguridad real del grafo | Medir caída de rendimiento, percepción GPIS, latencia y discrepancias; cualquier ajuste crea nuevo candidato |
| Bags/hardware | GPIS/origen/yaw y seguridad física bajo condiciones medidas; ensayo de dos etapas autorizadas | Shadow antes de control físico; G0–G4/G6 por separado; retorno al baseline si falla |

La promoción preinscribe métrica primaria para cada rol y para la suma de etapas, nivel de confianza, tamaño muestral derivado de varianza piloto, mínimos de mejora, máximos de regresión y veto de seguridad. Reportar intervalos por cluster de mapa/semilla y percentiles, no solo win rate agregado. Si el baseline ya domina o la evidencia es insuficiente, conservarlo; G5 puede quedar `FAIL`/`NOT_RUN` sin impedir evaluar una liberación del baseline que sí tenga G4/G6.

## 6. Datos y artefactos

Cada transición de opción guarda rol, etapa, estado/feature hash, opción/instancia, tiempos y duración, outcome/censura, reward descontado, evento oficial si existe, seguridad, política, modelo GPIS, fidelidad sensor/mapa/dinámica, rival y semillas. No descartar fallos, cancelaciones ni censurados; no bootstrap de terminales. El dataset es inmutable y sus splits tienen manifests con hashes. Para inferencia, exportar `manifest.json`, `parameters.json` y arrays `.npz` sin pickle con shapes/dtypes y compatibilidad de schema; `online_adaptation=false`. El runtime solo carga una política aceptada o el baseline firmado y no importa el entrenador.

## 7. Criterio de finalización

La línea entrega una matriz reproducible de coevolución, políticas G/E seleccionadas con evaluación held-out una sola vez, comparación pareada con baseline, reporte de transferencia MVSim/GPIS y decisión G5 revisada. La participación exige después imagen final, calibración y G6. La meta de ganar orienta el diseño de escenarios, rivales y selección; el plan demuestra ventajas medibles y límites, no promete un resultado deportivo.

## 8. Decisión de diseño: simulador completo intacto y backend de investigación ligero

### 8.1 Principio y límites de uso

La simulación completa y la ligera responden preguntas distintas y deben producir resultados identificables por perfil:

| Backend | Pregunta que puede responder | Usos permitidos | Conclusiones prohibidas |
| --- | --- | --- | --- |
| `sim/kinematic/` (referencia) | ¿Se conserva el comportamiento integrado del perfil cinemático, incluidos sus sensores, política, arbitraje y guardas implementados? | Regresión del sistema, conformance del perfil, evaluación cruzada y referencia de costes. | Fidelidad de LiDAR físico, garantía de seguridad física o aceptación de hardware por sí sola. |
| `sim/kinematic_light/` (nuevo, aislado) | ¿Qué tácticas y genomas parecen prometedores cuando se exploran más emparejamientos bajo una abstracción controlada? | Cribado temprano, diagnóstico de sensibilidad/diversidad y exploración de población. | Promoción, evidencia G4/G5/G6, seguridad, percepción GPIS, resultado oficial o rendimiento real. |
| MVSim/ROS, replay y hardware | ¿Se transfiere el candidato congelado bajo sus respectivas condiciones medidas? | Validación de transferencia y puertas de integración/seguridad que correspondan. | Ningún perfil sustituye la evidencia requerida por otro entorno. |

El backend ligero no es un “modo rápido” oculto del simulador actual, ni una variante de producción, ni una autoridad alternativa. Se implementa en un paquete/ruta de ejecución independiente, con configuración y `evaluation_profile_id` propios. No se cambia el comportamiento por defecto de `sim/kinematic/`, las APIs públicas, las reglas, los filtros de seguridad ni los límites de actuación para conseguir velocidad. La eliminación de un cálculo de seguridad del backend ligero solo indica que dicho perfil no modela esa capa; nunca autoriza a relajarla en producción.

### 8.2 Límites de arquitectura e interfaces

1. **Frontera explícita de política.** La política recibe una instantánea tipada de observaciones/creencias y devuelve una intención táctica o una selección de opción existente. No recibe el estado verdadero de la planta, el mapa oculto, el estado del árbitro ni referencias compartidas a esos objetos. El árbitro y la planta permanecen en el lado del entorno.
2. **Reutilizar contratos puros, no duplicar el runtime.** Reutilizar genoma, `Role`, esquema de features, límites de mutación, evaluación estadística y serialización compartidos cuando puedan importarse sin arrastrar el runtime completo. No importar la política autónoma de `sim/kinematic/` dentro del backend nuevo para evitar que el “light” ejecute inadvertidamente la cadena costosa completa.
3. **Mismo artefacto táctico, adaptadores por perfil.** El backend rápido y el completo deben cargar el mismo genoma versionado y los mismos pesos permitidos. Si una abstracción exige distinto cálculo de features, la transformación se declara en el manifiesto y se prueba contra observaciones pareadas; nunca se sobrescribe el artefacto del candidato.
4. **Seguridad de experimento, no seguridad de robot.** El entorno ligero mantiene como mínimo límites finitos de velocidad/aceleración configurados, colisión geométrica de footprint, límites del episodio y validación de comandos para impedir que trayectorias imposibles o fallos numéricos generen fitness espurio. Estas comprobaciones son guardas de integridad del experimento y no se presentan como la cadena de admisión, watchdog, cobertura ni seguridad del sistema completo.
5. **Sin salida física ni verdad oportunista.** El backend no publica a ROS ni a dispositivos. Una política solo consume el registro de observación entregado en ese tick; no puede consultar directamente raycaster, fixture, planta o árbitro. Cualquier perfil oracle existente se mantiene separado y excluido de fitness elegible.
6. **Artefactos sin ambigüedad.** Cada resultado incluye backend y versión, hash de mapa/escenario, configuración, seed, política, perfil de observación, reloj simulado, tasa de decisión, causa terminal y bandera `promotion_eligible=false` para el backend ligero.

La ruta `sim/kinematic_light/` es una ubicación de implementación propuesta. Antes de crearla se comprueba que no exista un paquete equivalente; si el inventario revela uno, se conserva su identidad y se documenta la separación funcional sin duplicar carpetas.

### 8.3 Alcance de fidelidad táctica suficiente

La fidelidad se juzga por si el entorno conserva las relaciones causa-efecto que necesita la táctica, no por semejanza visual o física exhaustiva. La primera versión ligera deberá preservar:

- observación parcial con oclusión y antigüedad de datos; la política no ve la pose verdadera del rival;
- geometría navegable, obstáculos y footprint finito: no se puede atravesar una pared ni capturar a través de una obstrucción por simplificación accidental;
- continuidad temporal y orden causal: percepción, decisión, acción, movimiento y evaluación terminal ocurren en orden reproducible;
- límites de velocidad y aceleración del perfil experimental, distinción Guardian/Explorer y predicates terminales idénticos a los autorizados para ese perfil;
- incertidumbre configurable de sensores/retardo y diversidad de escenarios suficiente para que la selección no dependa de una única semilla o topología;
- distinción entre colisión, timeout, fallo de episodio, intervención de guarda y terminal competitivo. Los errores técnicos no se recompensan como victoria.

Se pueden reducir la resolución/rango de rayos, los detalles dinámicos, la frecuencia de decisión táctica o la telemetría por tick, siempre que cada reducción quede en el manifiesto y que la prueba diferencial identifique qué decisiones altera. No se debe eliminar el problema de percepción parcial usando verdad del mapa o del rival. Se comienza con abstracciones 2D, observaciones sectorizadas y control cinemático; mayor detalle se añade solo si una diferencia observada en el backend completo demuestra que es necesario para la táctica estudiada.

## 9. Plan reactivo de ejecución y ahorro de cómputo

### 9.1 Separar los relojes

El backend conserva un paso físico simulado pequeño y determinista para integrar movimiento y detectar eventos de contacto, pero no ejecuta una planificación táctica completa en cada subpaso. Los relojes son virtuales: no hay `sleep()` ni espera de tiempo real en entrenamiento.

| Función | Cadencia inicial propuesta | Regla de actualización |
| --- | --- | --- |
| Integración de planta y detección de colisión/evento | Paso interno fijo configurable, inicialmente 50 ms para comparación con la referencia | En cada subpaso; subdividir o resolver analíticamente el cruce de un evento si el paso podría saltárselo. |
| Observación sintética | 5–10 Hz | Generar un nuevo registro solo en los instantes de sensor programados; entre observaciones se conserva la última medida con timestamp/covarianza. |
| Selección táctica | 2–5 Hz, es decir, período nominal 200–500 ms | Reevaluar en tick programado o por invalidación/evento material, no por cada integración de planta. |
| Replanificación de ruta completa | Event-driven, con límite superior configurable de 1–2 Hz durante el piloto | Solo cuando se invalida la ruta, cambia la versión de topología/objetivo o la decisión local demuestra que la ruta dejó de ser útil. |
| Registro de telemetría | Muestreo configurable durante entrenamiento; completo en diagnóstico | No construir objetos/logs detallados en cada subpaso del camino de producción del benchmark salvo en runs instrumentados. |

Las frecuencias son valores de partida para experimentos, no requisitos temporales del robot. El período de control del perfil de producción no se modifica aquí. Si la comparación requiere otra cadencia, se ejecuta un barrido preinscrito y se informa el efecto en decisiones y métricas.

### 9.2 Reutilización e invalidación del plan

Mantener en memoria el objetivo, la opción activa, ruta, snapshot de topología, timestamp y motivo de la última evaluación. Reutilizar la decisión mientras no haya expirado y sus precondiciones sigan siendo válidas. Una nueva planificación táctica/ruta se activa ante uno o más de estos sucesos observables:

- opción terminada, rechazada, caducada o incapaz de progresar durante un timeout acotado;
- nuevo objetivo, cambio de fase autorizado, fin del dwell mínimo o expiración del TTL del plan;
- observación que mueve al rival entre bandas tácticas/belief states, o cuya antigüedad supera el límite configurado;
- cambio versionado de topología/mapa, ruta obstruida, o desviación que hace inalcanzable el próximo waypoint bajo límites del perfil;
- margen geométrico o de colisión cruza un umbral experimental; esta detección local invalida el plan, pero no pretende ser el supervisor de seguridad normativo;
- cambio de feature normalizada por encima de un umbral con histéresis, definido en unidades de feature y medido en una prueba de sensibilidad.

La expiración temporal es un máximo de antigüedad, no una obligación de recalcular en cada tick. Los umbrales evitan la oscilación; no retrasan los eventos terminales ni colisiones. El simulador de referencia conserva su ciclo actual hasta que cualquier optimización separada haya sido implementada, probada y aprobada en su propio cambio.

### 9.3 Perfilado antes de optimizar

La hipótesis principal es que la política, la generación/evaluación de candidatos, los rayos, la seguridad geométrica y la telemetría por tick pueden dominar sobre la integración cinemática; todavía no se atribuye la causa a una función concreta. La primera entrega mide, por episodio y por tick, tiempo inclusivo/exclusivo de: sensor/raycast, preparación de features, selector de opción, routing, guardas, dinámica, referee y serialización/telemetría. Registrar además llamadas, candidatos evaluados, rutas cacheadas/recalculadas, ticks por episodio, ticks por reevaluación táctica y proporción de eventos.

El benchmark compara en el mismo host, Python, fixtures, seeds y política:

1. perfil sin profiler/telemetría detallada para medir el coste real de lote;
2. perfil instrumentado para localizar hotspots, cuantificando y declarando overhead;
3. modo de decisión cada tick y modo reactivo a igual paso físico;
4. referencia completa y ligera con resultados pareados y hash de configuración.

Hacer al menos 30 repeticiones por caso de microbenchmark y suficientes episodios completos para reportar mediana, p95 e intervalo de confianza de throughput; fijar y registrar host, versión, carga y warm-up. No comparar ejecuciones de hosts diferentes como evidencia de aceleración. Reportar `simulated_seconds / wall_seconds`, episodios/hora, ticks/segundo y latencias de fases; no llamar “x veces más rápido” a una medida sin especificar la razón.

**Objetivo provisional de capacidad:** primero medir el costo con workload representativo y derivar el lote de entrenamiento admisible del presupuesto real de CPU. Como umbral inicial del backend ligero se propone al menos 5x de episodios completos por segundo frente al backend de referencia sin telemetría detallada y al menos 10x tiempo simulado/tiempo de pared en el conjunto de fixtures, con 20x como objetivo de ingeniería. Estos umbrales se ratifican o revisan en un reporte L0 antes de usarlos como gate; no equivalen a una garantía de throughput de producción y no justifican alterar el paso temporal para lograr una cifra.

## 10. Orden de implementación, gates y criterios de salida

Las etiquetas L0–L5 son gates de esta línea experimental, no sustituyen G0–G6. Se puede avanzar en paralelo con los contratos puros P6.1/P6.2, pero no se puede usar el perfil ligero para cerrar G4 o autorizar G5.

| Gate / paquete | Trabajo | Pruebas y artefactos | Salida / bloqueo |
| --- | --- | --- | --- |
| L0 — línea base y presupuesto | Fijar commit, host y configuración; reproducir fixtures representativos; perfilado por subsistema y overhead; medir sensibilidad temporal y costo de telemetría. | Reporte de microbench/episodio, comando y hashes, p50/p95/IC, baseline rendimiento y lista ordenada de hotspots. | Elegir si el primer ahorro debe ser el backend ligero, el muestreo o el registro. Bloquea claims de aceleración si no es reproducible. |
| L1 — contratos aislados | Crear el backend separado, escenario/plant sintéticos, observación tipada y árbitro desacoplado; telemetría con `promotion_eligible=false`; sin importación accidental del runtime completo ni acceso de política a truth. | Tests de determinismo, aislamiento de imports, fallo/censura, truth-leak y ausencia de salida ROS/dispositivos. | Todo episodio y artefacto identifica el perfil ligero. |
| L2 — semántica y eventos | Integrar paso de planta, sensor programado, decisión táctica periódica/event-driven, TTL, invalidadores, colisión swept/evento terminal. | Tests de orden causal, observación caducada, obstáculo/goal change, no atravesar pared, eventos entre ticks y comparación de terminales. | Ningún evento obligatorio se pierde por saltar decisiones tácticas. |
| L3 — diferencial de política | Evaluar los mismos genomas/configuraciones en casos pareados ligero y referencia, primero con decisión cada tick y luego con cadencia reactiva; identificar desacuerdos de features, opción, ruta y terminal. | Matriz de acuerdo por decisión/episodio y explicación de cada desacuerdo; perfiles de 200, 333 y 500 ms o cadencia equivalente preinscrita. | El desacuerdo se cuantifica y se considera tolerable para cribado; no se exige identidad de trayectorias. Cambios sistemáticos en el orden de candidatos bloquean su uso como filtro de promoción. |
| L4 — throughput y sensibilidad | Ejecutar benchmark repetido con y sin telemetry, medir beneficio neto y sensibilidad a población/semillas/ruido. | Reporte contra L0 y suite de regresión focalizada; tabla de costo por candidato/generación y diversidad de decisiones. | Cumple o documenta desvío de objetivos de throughput, sin ocultar degradación táctica. |
| L5 — búsqueda experimental | Ejecutar piloto de evolución pequeño; conservar baseline, archivo y bancos independientes. Comparar finalistas solo en referencia y después seguir puertas de validación existentes. | Matriz genoma×rival×mapa×seed, selección, intervalos agrupados y manifiestos; no ejecutar held-out desde el trainer. | Solo produce candidatos a validar. La promoción exige G4 aceptado, plan estadístico preinscrito y revisión de evidencia en el backend/perfil aplicable. |

### 10.1 Prueba de equivalencia útil, no identidad física

No se exige que el backend rápido reproduzca cada pose. Debe conservar utilidad de selección. Para conjuntos pareados con igual genoma, mapa semántico, rival y seed, medir:

- acuerdo en la elección de opción y transición de estado táctico en los instantes de decisión comunes;
- acuerdo en el orden/ranking de candidatos, correlación de métricas primarias y tasa de inversiones de ranking (A parece mejor que B en light y peor en referencia);
- diferencias de tipo terminal, colisiones, duración y eventos de objetivo;
- sensibilidad de las métricas a período de decisión, observación, resolución de rayos, ruido y latencia;
- bootstrap/intervalos agrupados por familia de mapa, no por ticks correlacionados.

Como umbral piloto —sujeto a ratificación en L0/L3— exigir al menos 90% de acuerdo de opción en las decisiones pareadas comparables, ninguna diferencia de terminal causada por una violación de invariantes y ninguna inversión sistemática del ranking que exceda el error de muestreo. Un promedio de acuerdo alto no excusa fallos críticos en oclusión, colisión o fase. Si no se cumple, el backend sigue siendo útil para depuración/ablation pero no para filtrar finalistas.

## 11. Presupuesto inicial de coevolución

No aumentar de inmediato generaciones o población sobre un fitness plano. La siguiente configuración es un piloto sujeto a L0–L4 y al costo medido; es deliberadamente modesta y reversible, no un valor óptimo demostrado:

| Parámetro | Piloto recomendado | Justificación/ajuste |
| --- | --- | --- |
| Población por rol | 8 en smoke de infraestructura; 16 Guardian + 16 Explorer para primer piloto seleccionable | Mantener poblaciones separadas y diversidad por rol; no confundir 16 individuos con 16 episodios de evidencia. |
| Baseline | 1 baseline fijo por rol, fuera de la población evolutiva | Siempre comparar con el mismo hash y no permitir que mutación lo elimine. |
| Archivo de rivales | 4 políticas históricas por rol inicialmente más baseline; crecer solo con diversidad/amenaza medida | Evita adaptación circular a la población contemporánea sin coste de torneo completo. |
| Retención | 2 élites por rol + al menos 4 individuos diversos por distancia de genoma/decisión, si el ranking permite | Evitar reemplazo accidental de baseline y convergencia inmediata; la élite no sustituye validación. |
| Evaluación por candidato y rol | 8–12 emparejamientos pareados como piloto; distribuir entre baseline, archivo y población actual, mínimo 2 familias de mapa y 2 seeds | Ampliar adaptativamente con incertidumbre; unidad estadística es episodio/pareja independiente, no tick. |
| Mutación | Gaussian/proyección existente, `sigma=0.04` como centro de prueba y barrido corto `[0.02, 0.08]` | Es una escala experimental sobre genes normalizados; medir tasa de genomas idénticos, diversidad y cambios de decisión. No asumir que sigma mayor mejora exploración útil. |
| Crossover/genes nuevos | Desactivados en el primer piloto | El crossover no está especificado como parte de la búsqueda existente y nuevos realizadores/opciones quedan fuera del alcance y requieren verificación aparte. |
| Semillas | Derivación determinista por generación, candidato, rol, mapa y rival; common random numbers dentro de comparaciones pareadas | No reusar una seed como si fueran muestras independientes; guardar derivación completa. |
| Paralelismo | Medir 1 worker antes de escalar; workers por CPU disponibles con límites de memoria y sin nondeterminismo compartido | Verificar determinismo de cada episodio independiente y ausencia de contención que empeore throughput. |

Este diseño da al inicio 128–192 episodios de emparejamiento por generación si se evalúan 16 candidatos de un rol con 8–12 pruebas cada uno; el planificador puede compartir una partida entre puntuaciones de ambos roles solo si mantiene la evaluación y los rivales bien identificados. Antes de iniciar una generación completa, estimar su costo con el benchmark L4 y registrar duración prevista. Si excede el presupuesto, reducir emparejamientos de entrenamiento con muestreo estratificado; no reducir número de familias en validación ni reutilizar held-out. Si las diferencias de aptitud son menores que el error de muestreo, ampliar muestras según un cálculo de potencia/precisión basado en piloto, no aumentar mutación a ciegas.

El término `sigma=0.04` no prescribe cómo se deben redistribuir pesos ni reemplaza los límites de `mutate_genome`; usar la proyección y sign constraints existentes. La mutación no toca selección de reglas, condiciones de seguridad, probabilidades de sensor privilegiadas, geometría de captura o canales de autoridad. `minimum_dwell` e histéresis evolucionan solo dentro de los límites registrados y se evalúan por separado para detectar que no se conviertan en una demora sistemática de respuesta.

## 12. Recompensa, señal de selección y coadaptación

Una búsqueda rápida con mala señal solo encuentra antes una política sobreajustada. Antes de L5, ejecutar pruebas unitarias de sensibilidad por gen y pruebas de comportamiento en las que el ranking de utilidad de dos opciones sea conocido. Registrar por rol:

- tasa de mutantes que cambian una decisión, proporción de decisiones en que cambian y qué gen las causó;
- cantidad de opciones realmente elegibles, distribución de duración, repetición/oscillación de opciones y tasa de intervención/terminación;
- outcomes por tipo, tiempos a evento, progreso táctico observable y errores/censuras;
- brecha entre resultado frente a baseline/archivo y frente a oponente coevolutivo.

El informe P6.3 documenta que el retorno terminal win/loss produce deltas cero cuando ambos candidatos conservan el mismo terminal y tiempo. Por tanto, antes de escalar población se debe: (a) verificar que genoma → pesos → selector está cableado y produce cambios de decisión cuando corresponde; (b) crear escenarios donde los features registrados discriminan alternativas y verificar que el baseline no resuelve todo de inmediato; (c) revisar muestreo y rivals; (d) calcular sensibilidad/error con emparejamientos pareados. Un simulador más ligero no resuelve por sí solo un plateau de reward.

Mantener la métrica primaria preregistrada y distinta de métricas diagnósticas. Si se propone reward intermedio/potential shaping, especificarlo en un ADR/versión de perfil separado antes de entrenar, demostrar que no filtra verdad ni altera el objetivo competitivo elegido, y validar que el orden de políticas coincide con la métrica primaria en casos de control. No sumar a posteriori métricas convenientes para rescatar candidatos. Cada agente evolutivo se enfrenta a baseline, archivo y oponentes actuales; informar tanto rendimiento absoluto como robustez por rol para reducir explotación mutua.

## 13. Transferencia de candidatos y uso de los backends

La ruta de cada genoma es:

1. **Light discovery:** entrenamiento/cribado con semillas de entrenamiento y manifests. Los resultados son diagnósticos, evidencia `development-only`, no promoción.
2. **Reference confirmation:** reevaluar finalistas en el simulador cinemático existente, con el runner, política, perfil y eventos completos que correspondan; comparar con baseline en parejas y roles intercambiados. No seleccionar exclusivamente por el score del backend ligero.
3. **Validation:** congelar los finalistas antes de usar el banco de validación; escoger solo sobre validación. Fijar métrica, incertidumbre, límites de regresión y tamaño muestral antes de mirar estos datos.
4. **Held-out:** uso único posterior a preregistro, reservado al proceso de evaluación, fuera del trainer y con revisión independiente. Un fracaso no autoriza retocar y reusar el mismo held-out.
5. **MVSim/ROS/hardware:** cargar genomas congelados sin adaptación online; ejecutar transferencia y supervisión de seguridad conforme a los gates del plan maestro. Cambios producen un nuevo candidato y nuevos hashes.

La discrepancia entre backend ligero y referencia se analiza como señal sobre fidelidad, no se promedia para ocultar fallos. El candidato puede avanzar pese a que la mayoría de las trayectorias difieran si el ranking y los criterios preregistrados se conservan; queda bloqueado si las diferencias cambian quién gana, permiten atravesar obstáculos, dependen de observaciones privilegiadas o cambian las conclusiones sobre seguridad.

## 14. Datos, reportes y configuración del backend ligero

El manifiesto de episodio del backend ligero añade, como mínimo:

```json
{
  "backend": "kinematic_light",
  "backend_version": "semver-or-commit",
  "promotion_eligible": false,
  "evaluation_profile_id": "explicit-profile-id",
  "decision_period_s": 0.25,
  "observation_period_s": 0.1,
  "plant_step_s": 0.05,
  "scenario_sha256": "sha256",
  "policy_sha256": "sha256",
  "seed": 0,
  "truth_access": "referee-and-plant-only"
}
```

Los valores de cadencia del ejemplo son ilustrativos y deben coincidir con la configuración efectiva. Validar tipos/rangos al cargar; fallar explícitamente ante parámetros ausentes/inconsistentes. El manifiesto real contiene además commit, versión del runtime, seed derivation, configuración de sensor/dinámica, población, rivales, tiempos de pared/simulación, terminal, contadores de colisión, hash de perfil/feature y evidencia de fuente de escenarios. Los episodios fallidos, censurados, abortados y excluidos se conservan con motivo; no se borran para mejorar el fitness aparente.

Reducir coste de telemetría desactivando los arrays por tick en el camino normal de entrenamiento, no eliminando el resultado mínimo reproducible ni sus hashes. Ofrecer modo de traza completa para depurar un episodio reproducido desde manifiesto y seed. La telemetría resumida deberá bastar para reconstruir conteos, duración, objetivo, opción, colisión y motivo de exclusión.

## 15. Riesgos, fallos previsibles y mitigación

| Riesgo | Señal de detección | Mitigación / decisión |
| --- | --- | --- |
| “Light” conserva los costes principales | Perfil L0/L4 muestra raycasting o selector como hotspots aun con menos ticks tácticos | Optimizar solo el hotspot medido; considerar sensores sectorizados/malla espacial/eventos, conservando comparación diferencial. |
| Replanificación demasiado lenta | Aumentan colisiones, opciones inválidas, tiempo de recuperación o inversiones de ranking al crecer período | Reducir período o añadir invalidadores específicos. Seguridad/eventos terminales no se espacian para ahorrar CPU. |
| Replanificación todavía por tick | Ticks tácticos/replan no disminuyen y el costo se mantiene proporcional a cada subpaso | Revisar puntos de entrada y caches; testear que sin eventos el plan se reutiliza hasta TTL. |
| Fuga de estado verdadero | Política depende de objetos de escenario/árbitro, o un cambio solo visible en truth altera salida antes de observación | Separar procesos/records, forbidden imports, pruebas metamórficas y feeds de observación inmutables. |
| Atajo artificial por geografía/sensor | Candidato mejora solo en una familia o perfil sin oclusión/ruido | Estratificar por familia y condiciones; usar archivo de rivales y reporte de sensibilidad. |
| Modelo de recompensa plano | Mayoría de genomas indistinguibles, terminales idénticos | Diagnosticar cableado/escenarios y ampliar muestras; aprobar una métrica/shaping versionada antes de usarla. |
| Sobreajuste entre poblaciones | Alta ventaja self-play, regresión contra baseline/archivo | Mantener rivales congelados y validación por familias; no promover por self-play aislado. |
| Confundir fitness cero con seguridad | Mutante excluido o sin episodio completo tratado como puntaje neutro | Estado explícito `excluded`/`incomplete`, política de elegibilidad preregistrada, baseline conservado. |
| “Speedup” por cambiar semántica | Diferencia proviene de horizonte, paso, terminal o menor costo de observación | Parear perfiles y reportar throughput junto con acuerdo de decisión/terminal; no afirmar equivalencia. |

## 16. Lista de salida de este plan

Este plan se considera ejecutado solo cuando los artefactos siguientes existen y han sido revisados; añadir una implementación no equivale a cerrar sus gates:

- [ ] reporte L0 reproducible y presupuesto real de episodios/generación;
- [ ] backend aislado identificado como `development-only`, sin cambios colaterales en `sim/kinematic/`;
- [ ] observaciones parciales, verdad segregada, semántica terminal y clocks/eventos probados;
- [ ] perfil reactivo con TTL, invalidadores, periodicidad máxima y registro de motivos de replan;
- [ ] comparación de acuerdo/ranking con el simulador de referencia y análisis de desacuerdos;
- [ ] medición de throughput y precisión de intervalos bajo configuración/host registrados;
- [ ] sensibilidad de genoma/recompensa no plana o decisión documentada de bloquear evolución;
- [ ] población, rivales, métricas y presupuestos congelados antes del piloto;
- [ ] resultados de pilotos conservados con manifests, incluidos fallos y exclusiones;
- [ ] confirmación en referencia, validación independiente y held-out bajo las puertas originales antes de cualquier promoción.

El criterio rector es acelerar la exploración sin debilitar la evidencia: se protege el sistema completo, se evita recalcular táctica cuando no ha cambiado información relevante, y se conservan la semántica de seguridad y los gates de transferencia para el candidato que aspire a salir del entorno de investigación.
