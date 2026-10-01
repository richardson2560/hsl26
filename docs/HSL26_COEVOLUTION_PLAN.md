# HSL26 — Plan de coevolución Guardian/Explorer y selección competitiva

**Objetivo:** aprender tácticas para ambos roles mediante rondas de persecución/evasión reproducibles y seleccionar una política desplegable por rol. La búsqueda modifica solamente parámetros tácticos versionados y acotados de opciones existentes. Reglas, seguridad, geometría, autoridad ROS y acceso a verdad permanecen fijos. La arquitectura §§11–12 y la especificación §15 son el contrato.

## 1. Punto de partida y condición de entrada

Ya existen `learning/transitions.py`, `dirichlet.py`, `option_value.py`, `evolution.py`, `tools/train_evolution.py` y un runner pareado en `sim/kinematic/benchmark.py`. Los experimentos P6.3 retenieron el baseline: en las fixtures recientes las diferencias balanceadas de los mutantes fueron cero, no hubo finalista respaldado ni evaluación held-out. Esto exige diagnosticar sensibilidad de política y recompensa antes de aumentar población o recursos.

La preparación de código puede empezar ahora. Episodios para entrenamiento/promoción requieren primero un baseline G4 aceptado y bancos independientes con procedencia. GPIS/MVSim son puertas de transferencia y reconocimiento, no un sustituto de la velocidad del simulador cinemático.

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
