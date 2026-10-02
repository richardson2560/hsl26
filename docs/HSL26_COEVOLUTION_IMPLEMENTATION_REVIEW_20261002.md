# HSL26 — Auditoría del primer paso y guía de implementación de coevolución

**Fecha:** 2026-10-02. **Plan contrastado:** [revisión 2.1](HSL26_COEVOLUTION_PLAN.md). **Base examinada:** Git `8d1c6a96d1517bd953cb6d16da1dfe4d653d69e3` más cambios locales, incluidos los de `autonomous.py` y `train_evolution.py`. **Estado auditado:** primer intento NO ACEPTABLE como diagnóstico de señal. **Avance actual:** paquetes A y B implementados y verificados en desarrollo (§§11–12); C, búsqueda elegible y promoción pendientes.

Esta revisión responde al dictamen adjunto y al intento de instrumentación. Complementa la [auditoría anterior](HSL26_COEVOLUTION_AUDIT_20261002.md); no cambia las puertas del [plan maestro](HSL26_INTEGRATION_AND_COMPETITION_MASTER_PLAN.md) ni acredita G4/G5/G6. §§1–3 conservan los hallazgos históricos, con sus artefactos originales. §§10–12 registran el dictamen posterior y las implementaciones de A y B; la agregación C y la liga todavía no están implementadas. Esta entrega B preserva los cambios existentes de Docker/R2, ROS, HGW y modelos.

## 1. Dictamen sustentado por código y ejecución

El plan 2.1 tiene una secuencia razonable. El primer intento implementa campos y metadata, pero no el diagnóstico que exige el plan. Hay defectos que alteran numeradores, elegibilidad y transporte de datos; además falta resolver elección táctica, lifecycle y evaluación por rol antes de hablar de coevolución.

| Hallazgo | Severidad y origen | Evidencia | Corrección requerida |
| --- | --- | --- | --- |
| `n_available` es el número de propuestas, no un indicador | Crítico, instrumentación nueva | `_selection_trace_state_for`: cuatro propuestas dan `(active, available, choice, forced, empty)=(1,4,1,3,0)` | Separar cardinalidades de indicadores binarios |
| El conteo precede a los gates reales | Crítico, instrumentación nueva | Usa `p.feasible`; no usa `SelectionResult.alternatives`. Readiness falsa, lease inválida, safety stop, falta de autorización o guard ausente pueden dar disponibilidad 1 con cero opciones aplicables | Medir después de `select()`, desde sus evaluaciones finales |
| Se deduplica por ID de instancia y sobre todas las propuestas | Crítico, instrumentación nueva | Dos propuestas con mismo tipo/target e IDs distintos cuentan dos; diferentes prioridades o urgencias dan elección 1 aunque la cohorte ganadora tenga un miembro | Cohorte prioridad/urgencia y firma de plan/efecto |
| Movimiento y observación pierden los campos nuevos | Crítico, instrumentación nueva | `_execute_candidate` y `_execute_observation_option` crean trazas con los defaults | Un único constructor de trazas que transporte el contexto de ciclo |
| Retornos anteriores al selector heredan el ciclo previo | Grave, instrumentación nueva | Tras un ciclo válido, `missing_stage_pose_or_topology` registra `n_active=1` sin haber invocado el selector | Reiniciar contexto al entrar en cada llamada y distinguir salida previa al selector |
| Inicio de dwell se recalcula como `now_ns` | Grave, preexistente; bloquea búsqueda | `goal.deadline_ns - (match.stage_ends_at_ns - now_ns)`; los goals usan deadline igual al fin de etapa | Guardar instante real de inicio por instancia de opción |
| No hay transporte/agregación al reporte | Grave, entrega incompleta | `SILEpisodeSample` lleva poses/comandos/opciones; no lleva los contadores. El trainer no agrega selección/cohorte | Agregador por episodio/rol/escenario/percepción y reporte versionado |
| `step` del reporte describe un diagnóstico no realizado | Trazabilidad insuficiente | Se etiqueta `signal_diagnostics_and_registration`, pero el artefacto registra búsqueda de fixtures | Etiquetar la operación real y registrar evidencia de cada paso |
| Explorer entrega una sola propuesta; Guardian tracked también | Dependencia táctica preexistente | `_explorer_proposals`, `_guardian_pursuit_proposal` | Generación acotada de alternativas realizables antes de buscar pesos |
| Trainer conjunto sin liga ni evaluación independiente por rol | Dependencia metodológica preexistente | Mutación de ambos bloques y switching común; comparación contra baseline fijo | Piloto por rol, rivales congelados y después rondas alternadas |

**No iniciar una población grande para resolver estos defectos.** Primero cerrar la instrumentación y reproducir sensibilidad. El `traceability` actual es útil como metadata de desarrollo, pero no permite declarar el paso completo.

## 2. Contraste del dictamen adjunto

Son correctos los diagnósticos sobre la partición, los IDs, la omisión de cohorte, la pérdida de campos y la ausencia de agregación. Hay que corregir también estas limitaciones de la propuesta adjunta:

- Su reemplazo de `_selection_trace_state_for` **no filtra la cohorte**, aunque el comentario dice que lo hace. Sigue contando todas las propuestas factibles.
- Sigue usando `proposal.feasible` y no la aplicabilidad tras registry, guards y gates globales. Corregir solo `n_available=int(len(feasible)>0)` conserva falsos positivos.
- `(kind, target_node_id)` no demuestra equivalencia conductual. Puede colapsar rutas distintas al mismo target o contar targets distintos que terminan realizando la misma conducta. Para OBSERVE_SAFE no existe necesariamente una ruta; hay que definir firma de efecto.
- Sus números de prioridad son ilustrativos y no coinciden con el selector actual: escape/LOS de Explorer tienen prioridad 0; avance base/ruta conocida, 2. La urgencia más alta gana dentro de la menor prioridad. La implementación debe consultar evaluaciones reales, no copiar una tabla aproximada.
- El test sugerido usa `make_valid_test_policy_input`, que no existe aquí. Se puede construir el caso con los builders existentes, y convertirlo después en tests mantenibles.
- Las 49 pruebas aprobadas fueron regresiones válidas. La conclusión correcta es **cobertura insuficiente para aceptar la nueva telemetría**, no que todas esas pruebas sean inútiles.
- El artefacto citado ejecutó una generación y población de dos, con horizonte de 8 s y aproximadamente 31.86 s de pared. Fue búsqueda de desarrollo, no entrenamiento completo ni búsqueda elegible. El problema es presentarla como evidencia del diagnóstico; no hay base para afirmar que consumió una población larga.

El reporte histórico declara correctamente `eligible_empirical_training=false`, `promotion_eligible=false` y gates bloqueados. Conservarlo con su identidad original. Un informe nuevo puede declarar que no acredita el paso; no reescribirlo retrospectivamente con contadores que nunca recogió.

## 3. Evidencia nueva y límites

Reproducción: [script](../artifacts/reports/coevolution_step1_audit_20261002/reproduce.py) y [datos](../artifacts/reports/coevolution_step1_audit_20261002/findings.json). El script usa builders de tests para snapshots adversarios y wrappers temporales restaurados con `finally`. Observa resultados sin modificar ranking, comandos, límites ni archivos de producción. Registra commit y SHA-256 de fuentes y verifica que no cambiaron durante la ejecución.

| Caso reproducido | Observado | Esperado |
| --- | --- | --- |
| Cuatro propuestas aplicables | `1+3+0=4` para choice/forced/empty con active 1 | Partición igual a 1 |
| Dos propuestas de distinta prioridad | `n_choice=1`; cohorte final con un miembro | `n_choice=0` |
| Dos escapes de diferente urgencia | `n_choice=1`; cohorte final con un miembro | `n_choice=0` |
| Readiness/lease/safety/autorización bloquean | `n_available=1`; cero alternativas aplicables | `n_available=0`, `n_empty=1` en la llamada ACTIVE |
| Guard requerido ausente | Disponibilidad 1 y rechazo `guard_not_satisfied` | Disponibilidad 0 |
| Movimiento nominal ACTIVE | `phase=""`, `system_ready=false`, contadores cero | Contexto ACTIVE y una invocación registrada |
| OBSERVE_SAFE autorizado | Campos nuevos a defaults | Una invocación ACTIVE, con disponibilidad de observación |
| Siguiente entrada sin topología | `n_active=1`; selector no llamado | `n_active=0`, salida previa al selector registrada aparte |
| Dwell con start antiguo real | Cambia a `search_portal-2` por mejora de utilidad | Correcto |
| Mismo snapshot con start recalculado al presente | Retiene `search_portal-1` por `minimum_dwell` | Evidencia del sesgo de un reloj de dwell reiniciado |

El ensayo de 8 s en `maze_multiring_7x7_train_a` se combina en el informe con tres casos pequeños de regresión. En ese conjunto el selector se invoca en ACTIVE 159 veces por rol, pero las trazas suman `n_active=3` Guardian y `1` Explorer. Las llamadas con inicio de opción vigente son 155 G y 156 E; **todas reciben edad cero**. Estas cifras prueban fallos de cableado y lifecycle. No son estimaciones poblacionales ni las tasas §3.1 definitivas: las cohortes se cuentan por cardinalidad, sin certificar todavía deduplicación de plan/efecto.

Se ejecutó la regresión existente: **149 passed, 1 warning en 92.99 s**, registrada en [regression_result.txt](../artifacts/reports/coevolution_step1_audit_20261002/regression_result.txt). El warning es de cache de pytest en Windows. No se ejecutaron búsquedas, validation, held-out, ROS, hardware ni aceptación HGW. Un timeout del perfil corto de 8 s no se interpreta como victoria de supervivencia en competición.

## 4. Contrato correcto de instrumentación

### 4.1 Fuente de verdad: evaluación final del selector

Conservar `TacticalSelector.select()` como autoridad de aplicabilidad y ranking. `SelectionResult.alternatives` ya contiene aplicabilidad final, razón de rechazo, prioridad, urgencia y utilidad, incluidos bloqueos globales. No llamar a `_evaluate()` otra vez ni duplicar la lógica de guards en `autonomous.py`: se arriesga una divergencia entre selector y auditor.

Por ciclo:

1. Reiniciar contexto al entrar en `__call__`, antes de los retornos por tiempo/inputs/coherencia. Capturar fase conocida y estado de inputs; readiness no calculada no equivale a readiness falsa comprobada. Registrar `selector_invoked=false` inicialmente.
2. Construir snapshot y propuestas; conservar sus keys, features, planes y evidencia. `proposal_total` debe declarar si incluye HOLD_SAFE; el código actual sí lo incluye.
3. Invocar `select()` una sola vez. Registrar invocación y resultado. Contar las aplicables **después** de sus gates, excluyendo HOLD_SAFE.
4. Encontrar la prioridad mínima entre aplicables. En ese subconjunto encontrar la urgencia máxima, con `None` equivalente a 0. Ese es el conjunto cuyo ranking puede cambiar por utilidad.
5. Asociar sus keys a las propuestas y planes originales. Deduplicar conducta dentro de esa cohorte, con las reglas §4.2.
6. Construir indicadores. Transportarlos sin cambios a cualquier salida posterior: hold, fallo de autoridad, movimiento, observación, efecto satisfecho o parada del supervisor.

Contrato binario para las invocaciones con evaluación válida:

```python
n_active = int(selector_invoked and phase == "ACTIVE")
n_available = int(n_active == 1 and applicable_non_hold_count > 0)
n_choice = int(n_active == 1 and distinct_winning_behaviors >= 2)
n_forced = n_available - n_choice
n_empty = n_active - n_available
```

Invariantes comprobadas con validación explícita, además de tests:

```text
0 <= n_choice <= n_available <= n_active <= 1
n_choice + n_forced + n_empty == n_active
sum(N_choice, N_forced, N_empty) == N_active
```

No usar `max(0, ...)` para ocultar una partición inválida. Un `assert` aislado no es validación de reporte robusta, porque puede desactivarse con Python optimizado. Si el selector lanza una excepción, registrar invocación fallida y razón; marcar incompleta la evidencia y no presentar sus tasas como diagnóstico aceptado. No convertir la excepción en una evaluación ficticia válida.

Propuesta de nombres: `proposal_feasible_raw` para `p.feasible`, `proposal_applicable` para el resultado final, `winning_cohort_size` antes de deduplicar, `distinct_winning_behaviors` después. Si se conservan `proposal_feasible`/`proposal_choice`, definir su significado en un schema nuevo y comprobar migración. `proposal_choice` puede ser 0, 1, 2…; una cohorte unitaria tiene cardinalidad 1 aunque `n_choice=0`.

`N_choice` mide oportunidad estructural dentro de la cohorte. Histéresis/dwell pueden retener la opción actual; registrar ese motivo y el efecto de perturbaciones aparte. Una propuesta elegible posteriormente rechazada por autoridad/supervisor conserva el diagnóstico de selección y añade su rechazo de ejecución. Mezclar ambos niveles ocultaría si falló la decisión, el realizador o la seguridad.

### 4.2 Firma de conducta y trazas

Crear una función probada para identidad de conducta realizada, no usar `option_instance_id`, `stable_key` o un hash del genoma como sustitutos. Para movimiento, representar recorrido dirigido completo recortado desde pose, aristas y geometría canónica, junto con clase/semántica terminal del efecto y tolerancias relevantes. Normalizar redundancias geométricas con convención versionada. No fusionar rutas paralelas ni rutas que comparten un prefijo y divergen después. Para observación/parada, definir efecto sin ruta.

Los IDs de arista pueden distinguir recorridos dentro del snapshot; el orden del catálogo frente a renumeración requiere además identidad geométrica estable. No usar stamps, contador de instancia o versiones del mapa como prueba de diversidad conductual entre dos propuestas del mismo snapshot. Registrar esas versiones como procedencia. Un plan que no se puede asociar o validar no acredita diversidad; emitir razón y diagnóstico incompleto.

Mantener el MVP con una ruta canónica por target distinto, según §3.2 del plan. Actualmente `paths[target_node_id]` y la key tipo/target no representan dos rutas al mismo target. Ampliar ese contrato exigiría identidad de ruta en propuesta, selección, lifecycle y ejecutor; se difiere. La equivalencia por tipo/target solo puede ser un diagnóstico provisional limitado, declarado como tal.

Recomendación de estructura: un dataclass de contexto de selección, y un helper `_record_trace(...)` que use argumentos nombrados y añada ese contexto a la traza. Evitar repetir 21 argumentos posicionales en cada salida. Mantener compatibilidad de consumidores cuando se versione el schema.

### 4.3 Agregación y disponibilidad anterior al selector

Agregar por `run/episode/role/scenario/perception_mode/evaluation_profile`. Guardar numeradores y denominadores, no promediar tasas de episodios con distinta exposición:

```python
availability = N_available / N_active if N_active else None
coverage = N_choice / N_active if N_active else None
conditional = N_choice / N_available if N_available else None
no_proposal = N_empty / N_active if N_active else None
```

Contar aparte FREEZE/INIT/TERMINAL, HOLD por fase, ciclos ACTIVE que salen antes del selector, fallos de selector, causas de no readiness y rechazos de autoridad/supervisor. No inferir esos ciclos desde comandos cero: OBSERVE_SAFE puede ser una opción admisible sin movimiento.

En `benchmark.py` la agregación puede operar en cada llamada de política y mantener solo totales durante screening. Para diagnóstico, almacenar filas por decisión. No sumar repetidamente `last_trace` si un tick no llama a la política: usar un ID/secuencia de ciclo y comprobar unicidad. La verdad del árbitro permanece en registro separado; no añadirla a features.

## 5. Paquetes concretos de implementación

La tabla especifica los entregables. `selection_diagnostics.py` y sus pruebas del paquete A están implementados (§11); `ActiveOption` y sus pruebas de lifecycle B también (§12). Los módulos nuevos de C–I siguen siendo propuestas. Cada paquete debe cerrarse con su evidencia antes de depender de él. Son unidades de trabajo, no gates adicionales.

| Orden | Archivos y puntos a modificar | Entrega y criterio de salida |
| --- | --- | --- |
| A — instrumentación fiel | `sim/kinematic/autonomous.py`: `PolicyCycleTrace`, entrada de `__call__`, cálculo después de `select`, `_execute_candidate`, `_execute_observation_option`, `_trace_stop`; módulo propuesto `sim/kinematic/selection_diagnostics.py`; tests en `sim/kinematic/test_p56_autonomous.py` y propuesto `test_selection_diagnostics.py` | Once snapshots adversarios y todas las rutas de salida cumplen transporte/partición; cohorte real y deduplicación probada; no cambia ninguna decisión/comando |
| B — lifecycle y dwell | `autonomous.py`: `_active`, `_start_motion_option`, selección/continuidad, cancelación/revoke/efecto; `hsl_core/hsl_core/tactics/fsm.py` solo si hacen falta metadatos públicos; tests autónomos y `hsl_core/tests/test_p53_tactics.py` | Start real conservado durante replans/renovaciones; reset en nueva instancia; preempción por prioridad/urgencia intacta; cambio por utilidad tras dwell real |
| C — diagnóstico independiente | `sim/kinematic/benchmark.py`: agregación en runner y reporte de episodio; nuevo `tools/diagnose_coevolution.py`; `tools/benchmark_p63.py` solo si comparte exportador; `sim/kinematic/test_p63_benchmark.py`, `test_p63_tools.py` | CLI de desarrollo sin evolución y training-only por defecto; totales/ratios válidos, trazas auditables, hashes y coste; cero validation/held-out implícitos |
| D — alternativas ejecutables | `autonomous.py`: `_proposals`, `_explorer_proposals`, `_guardian_pursuit_proposal`, `_path_to_node`, `_make_proposal`, asociación de planes; módulo propuesto `sim/kinematic/tactical_candidates.py`; fixtures y tests autónomos | Catálogo independiente del genoma, `B_paths=12`, `K=3` como piloto versionado; varias alternativas reales en ambos roles cuando el escenario lo permite; cada elegida realiza la ruta propuesta |
| E — perfil y conformidad | `sim/kinematic/benchmark.py`, `sim/kinematic/match.py`, `sim/kinematic/referee.py`, `hsl_core/hsl_core/match.py` solo donde falte la integración; pruebas de match/referee/benchmark; `phase6_learning_release.json` tras aceptación verificable | Perfil de terminales/zonas/tiempos/sensores definido; baseline sin contacto/fallos bajo criterios aplicables; matriz SIL y revisión G4 del perfil |
| F — sensibilidad y genes | nuevo `tools/diagnose_coevolution.py`; `hsl_core/hsl_core/learning/evolution.py`: mutación restringida; `hsl_core/tests/test_p63_evolution.py`; manifests del experimento | Snapshot común con estado de selector igual, conjunto generado idéntico y primer cambio de conducta por rol; máscara de genes efectivos, switching evaluado aparte |
| G — piloto por rol | `tools/train_evolution.py`: modo/operación/reportes; `evolution.py`: assessment por rol y pareado ampliado; `benchmark.py`: rivales explícitos; `tools/p63_policy.py` y tests de serialización | Baseline + hasta cuatro candidatos por rol frente a rivales congelados; solo bloque objetivo cambia; otra política, entorno y seeds pareados intactos; coste y exclusiones reportados |
| H — coevolución alternada | módulo propuesto `hsl_core/hsl_core/learning/opponent_archive.py`, `tools/train_evolution.py`, tipos de episodio/provenance y tests | Una ronda G→E, archivo baseline + hasta dos históricos por rol, matriz cruzada; archivo inmutable durante cada ronda; segunda ronda solo con ventaja y presupuesto |
| I — evaluación y release | assessments/reportes, `tools/p63_policy.py`, `tools/release_freeze.py`, `tools/rehearse_phase6_sil.py`, cargador de runtime y planes de aceptación correspondientes | Finalista training→validation→freeze→held-out una vez; paquete compatible, hashes, baseline/rollback y decisión G5/G6 del entorno correspondiente |

### 5.1 Cambios de lifecycle que no se deben omitir

Agregar `started_at_ns` al estado activo, idealmente en un dataclass `ActiveOption` en lugar de extender una tupla sin nombres. Fijarlo al admitir/iniciar una instancia. No reiniciarlo en `begin_replan`, renovación de lease, reevaluación o actualización de path de la misma instancia. Al cancelar, terminar, revocar o sustituir una opción, limpiar ese estado. Reset de epoch/etapa exige invalidación coherente.

El selector puro ya valida start y aplica prioridad/urgencia antes de dwell. La reproducción indica fallo del caller; no justifica relajar esas reglas en `fsm.py`. En particular, no basta con poner dwell=0 para hacer que una población parezca sensible: corrige el reloj y evalúa switching por separado.

### 5.2 Construcción de alternativas

Implementar §3.2 del plan, con las restricciones de fuente observada/autorizada, freshness, OPEN y realizador. Enumerar catálogo acotado de targets válidos, agrupar por rama dirigida, recorrer intercaladamente con orden geométrico/seed congelada, validar hasta `B_paths`, deduplicar y admitir hasta K. Registrar revisados, rechazados, omitidos, ramas y seed de catálogo. Añadir límites del tamaño del grafo/catálogo: K=3 no acota por sí solo un escaneo ilimitado.

Explorer bajo amenaza no debe ejecutar el `max(hidden_routes or candidate_routes, score)` previo y entregar su ganador. Emitir alternativas supervivientes con clase/guards apropiados y urgencia derivada del mismo track/instante. Los pesos solo deciden dentro de la cohorte real. No igualar arbitrariamente prioridades ni admitir avance a base contra guards de amenaza.

Guardian sin track ya tiene búsqueda, pero emite un conjunto potencialmente grande. Acotarlo con diversidad y registrar pérdidas; con track, exponer presión realizable hacia targets válidos. No crear INTERCEPT/DEFEND solo porque existen en el enum: sus guards, timing y realizador tienen que funcionar. El piloto puede limitarse a opciones ya soportadas.

Comprobar que `paths[selected.target_node_id]` corresponde a la alternativa elegida y que replanning conserva su intención o invalida explícitamente la propuesta. Tres keys que el ejecutor convierte en un mismo camino no producen elección útil.

### 5.3 Sensibilidad antes de fitness

Para cada gen permitido, construir perturbaciones válidas del genoma y comparar snapshots comunes, con start/histéresis/dwell, rival y estado iguales. Registrar features de cada alternativa, contribuciones de utilidad, margen entre primera/segunda, elección, ruta y efecto. La mutación no puede cambiar el catálogo previo al ranking.

La firma de genoma distinta no prueba sensibilidad. Dos features Guardian (`capture_opportunity`, `portal_time_advantage`) permanecen cero en los generadores inspeccionados; `observation_gain` suele ser común a las alternativas del mismo snapshot. Determinar señal mediante perturbaciones y diversidad de features, sin asumir que añadir rutas activa automáticamente todos los genes. La proyección/normalización y la histéresis pueden acoplar pesos: registrar también los cambios efectivos de los otros coeficientes.

Una mutación restringida debe preservar exactamente el bloque rival, switching congelado y genes fijados. Si se proyecta solo el subconjunto mutable, respetar masa residual `1 - sum(abs(pesos_congelados))`. La mutación actual recorre ambos roles y ambos parámetros comunes; requiere una API explícita de máscara/rol o composición validada. No atribuir una mejora a Guardian si al mismo tiempo se alteró dwell de Explorer.

En rollout cerrado, registrar primera divergencia de decisión, ruta y comando, además de outcome. Tras la divergencia ya no hay snapshots comunes. `mutation_effect_rate` necesita declarar gen perturbado y denominador de oportunidades evaluadas; no contar ticks posteriores como réplicas independientes.

## 6. Evaluación por rol y liga: contratos necesarios

La comparación role-swapped actual es útil: `G_i × E_base` y `G_base × E_j`, con referencia baseline×baseline. Evita sumar los dos retornos cero-suma de una única partida, pero aún no es liga ni dos etapas físicas de competición.

`assess_paired_candidate()` exige ambos roles y usa deltas balanceados; no se debe alimentar con una fila falsa para el rol congelado. Añadir assessment independiente por rol, con evaluación primaria de ese rol y comprobación de seguridad de **ambos robots en todo el match**. Guardar la partida completa aunque se extraiga un resultado focal para fitness.

`EpisodeResult.pair_key` contiene `opponent_bank_id`, no identidad explícita del adversario congelado. Para archivo/ligas, incorporar al pareado y manifests rival/hash por rol, ronda y snapshot de archivo. No emparejar contra baseline si ese baseline se evaluó frente a otro adversario. La referencia cacheada debe compartir escenario/hash, perfil, rival, seeds, sensores y configuración; una nueva ronda o nuevo adversario exige su referencia propia.

El código actual excluye por fallos del candidato, pero no certifica que el baseline sea aceptado: `_baseline_assessment` fabrica la referencia y `assess_paired_candidate` no comprueba todas las condiciones de seguridad del baseline. Añadir una verificación previa del baseline y del perfil. Si falla, la ronda no tiene referencia aceptable; no presentarlo como baseline válido por construcción.

Secuencia implementable:

1. Congelar entorno, métrica, genes, switching, adversarios y splits; medir el coste y fijar presupuesto/cutoff.
2. Piloto G contra E baseline/banco fijo, sin mutar E; piloto E contra G baseline/banco fijo, sin mutar G. Máximo cuatro nuevos por rol como diseño inicial, sujeto a presupuesto.
3. Con señal y datos elegibles, optimizar G durante una ronda contra E/archivo congelados. Congelar finalista G.
4. Optimizar E contra ese G y referencias congeladas durante toda la ronda. Congelar finalista E.
5. Ejecutar matriz contra baseline y versiones anteriores; actualizar archivo solo al cerrar ronda. Conservar referencias que expongan conducta/fallos distintos. No anunciar convergencia por ganar al rival más reciente.
6. Repetir como máximo una segunda ronda si mejora frente a referencias y cabe en presupuesto. Si no, cerrar con candidato o baseline y causas documentadas.

Con rivales/perfiles iguales y referencia baseline compartible, C_G y C_E candidatos en K combinaciones requieren `K*(1+C_G+C_E)` matches. Los rivales adicionales añaden referencias y evaluaciones propias. Comparaciones por genoma conjunto requieren `K*(1+2*C)`. Estas fórmulas cuentan simulaciones, no filas por rol ni una garantía de horas de ejecución.

Para evaluación final fijar unidad independiente y plan antes de mirar resultados. El intervalo t actual sobre pair keys no resuelve correlación entre seeds del mismo mapa ni repetición determinista sin ruido. Reportar estratos/roles y peor caso; definir incertidumbre compatible con agrupación y outcomes. Pocas unidades independientes significan evidencia insuficiente, no permiso para tratar ticks como muestras.

## 7. Perfil competitivo, datos y entrega

El benchmark admite CAPTURE/TIMEOUT y tiene freeze de 0.15 s. El núcleo de match ya incluye ARRIVAL, pero falta su uso en el benchmark con zona autorizada y evento de contornos. No simular llegada oficial por alcanzar `synthetic_goal_node_id`. Integrar captura/llegada/timeout y carreras entre eventos según matriz del perfil; el árbitro puede usar SIM_TRUTH, la política no.

El horizonte de screening corto es útil para cableado/decisiones. No promueve por supervivencia truncada. La confirmación usa el horizonte completo del perfil aceptado. El trainer fija beta=0.01; cualquier objetivo binario sin descuento requiere configuración y score profile nuevo. `official_score` permanece null mientras no haya tabla autorizada. No cambiar objetivo o límites tras observar candidatos para rescatar una mejora.

Guardar manifests separados de entorno/perfil, splits, políticas, adversarios, catálogo y datos. Incluir commit **y fuentes locales reales**, hashes de configuración, `fsm.py`, `options.py`, utilidad, routing, safety, match/referee, sensor/percepción y dinámica, además de genes/trainer. La lista actual `_source_hashes()` omite dependencias críticas del ranking; el hash del genoma solo identifica parámetros, no su conducta bajo otro ejecutor.

Cada reporte de diagnóstico/búsqueda debe incluir los campos §11 del plan y además `N_forced`, `N_empty`, salidas previas al selector, invalidaciones de diagnóstico y motivos. Registrar `null` para métricas no medidas, no un cero fabricado. Declarar backend/percepción/fidelidad temporal; RTF no acredita deadlines físicos. La política SIL usa tiempos steady sintéticos en seguridad y no mide latencia real del callback.

Mantener compatibilidad con `tools/p63_policy.py`: ese formato es de desarrollo no promocionable. El release requiere `manifest.json`/`parameters.json` y artefactos no ejecutables cuando apliquen, compatibilidad de schemas, carga validada, adaptación online desactivada y baseline de rollback. `release_freeze.py` no convierte automáticamente una política de fixture en política aceptada.

HGW, ROS y calibración siguen sus planes propios. No es necesario completar hardware para ejecutar los diagnósticos sintéticos A–D/F, pero sí aceptar el perfil G4 antes de datos elegibles y demostrar transferencia para G6 del entorno final. R2 aislado comprueba parada/watchdog y no aporta autonomía ni aceptación G4. No acoplar la reparación táctica a cambios de margen, radio, velocidad, prioridad o permisos de movimiento.

## 8. Pruebas de aceptación mínimas por paquete

| Paquete | Pruebas necesarias para cerrar el paquete (A/B cubiertas en §§11–12) |
| --- | --- |
| A | ACTIVE con cero/una/cuatro opciones; HOLD excluido; rechazo de guard/registry/schema/ready/lease/safety/autorización; prioridades/urgencias distintas; mismo plan con IDs distintos; prefijo común con divergencia; rutas paralelas; denominador cero→null; todos los caminos de traza; fallo temprano tras ciclo válido; contador agregado una vez por invocación |
| B | Start preservado tras replans; start nuevo tras sustitución; dwell expira y permite mejora; histéresis retiene dentro de delta; prioridad/urgencia preemptan antes del dwell; cancelación/revoke/epoch/etapa y candidatos obsoletos |
| C | Solo training por defecto; misma suma por filas y agregado; reportes incompletos rechazados; esquema/serialización; JSON sin NaN; no ejecutar mutación ni leer validation/held-out para diagnosticar |
| D/F | Catálogo vacío/una/múltiples ramas; presupuestos alcanzados y omisiones; determinismo/renumeración; OPEN vs UNKNOWN/BLOCKED; guard y truth negativos; conjunto idéntico bajo pesos distintos; selección y camino ejecutado distintos en un caso no trivial por rol |
| E | ARRIVAL autorizado si el perfil lo requiere; captura/LOS obstruida; eventos entre ticks/ambiguos; timeout completo; consistencia de managers; baseline sin contactos y contratos SIL aplicables |
| G/H | Mutación no cambia bloque congelado; máscara preserva genes; rival en pair key; cache no reutilizado frente a adversario distinto; archivo fijo en ronda; update solo entre rondas; baseline inválido bloquea lote; seguridad de ambos robots; límites de coste/parada y matriz cruzada |
| I | Candidato congelado antes de held-out; carga y hashes compatibles; rechazo de parámetros corruptos/incompatibles; baseline/rollback; transferencia y gates del perfil documentados |

## 9. Acciones inmediatas y comandos reproducibles

**Siguiente entrega recomendada:** C, apoyado en A y B ya verificados. La aritmética sola no es suficiente. Esos tres paquetes cierran el primer paso y permiten medir el efecto de implementar D. El orden operativo es diagnóstico fiel → alternativas → sensibilidad → G4/datos aceptados → piloto por rol → liga condicionada → evaluación/release. La conformidad puede avanzar en su propio trabajo mientras se diagnostican fixtures, pero ninguna búsqueda de desarrollo acredita las puertas.

Comandos disponibles ahora, desde la raíz:

```powershell
python -m pytest hsl_core/tests/test_p53_tactics.py hsl_core/tests/test_p63_evolution.py hsl_core/tests/test_p52_option_authority.py hsl_core/tests/test_p35_planning_control.py hsl_core/tests/test_safety.py hsl_core/tests/test_p51_match.py sim/kinematic/test_selection_diagnostics.py sim/kinematic/test_active_option_lifecycle.py sim/kinematic/test_p56_autonomous.py sim/kinematic/test_p63_benchmark.py sim/kinematic/test_p63_tools.py -q -p no:cacheprovider --tb=short 2>&1 | Tee-Object artifacts/reports/coevolution_package_b_20261002/tests.txt
if ($LASTEXITCODE -ne 0) { throw 'Regresión B fallida' }
python artifacts/reports/coevolution_package_b_20261002/verify_runtime.py
```

El verificador B valida y registra el hash del log de regresión; genera `runtime_verification.json` con equivalencia frente a A bajo dwell cero, edades reales bajo dwell positivo y controles adversarios (§12). No es el runner C ni una búsqueda. Los scripts históricos de `coevolution_step1_audit_20261002` y `coevolution_package_a_20261002` dependen de las versiones auditadas: no ejecutarlos sobre B ni sobrescribir sus resultados históricos para aparentar continuidad. A y B están implementados; C–I y `diagnose_coevolution.py` siguen pendientes. No hay un comando de coevolución correcto listo para ejecutar.

Cerrar A/B/C únicamente si las trazas nominales y de fallo son coherentes, las identidades se cumplen y la agregación representa invocaciones reales. Cerrar D/F solo con ejemplos de efecto de genes por ambos roles y realizadores verificados. Si no hay efecto suficiente, congelar genes o conservar baseline y documentar causas. Terminar con evidencia de ausencia de mejora es un resultado válido; terminar con metadata sin señal no lo es.

## 10. Detalles del dictamen posterior: aceptados y matizados

**Fuente:** adjunto del usuario `05abcdac-b415-4565-954d-b645c6cef63c/Pasted text.txt`, SHA-256 `04b7dbbad3d18b82cf7088a8c2f6d96d816721083981d8977ede38ea6d1b2a7c`. Se registra como revisión externa aportada, no como aceptación de G4/G5/G6 ni cambio de autoridad normativa. La instrucción del usuario en esta entrega es incorporar lo pertinente y ejecutar **A**.

| Detalle | Resolución |
| --- | --- |
| Dataclass `ActiveOption` explícito | Aceptado como especificación de B, con los campos del ejemplo siguiente. No implementar B dentro de A, que debe conservar decisiones y comandos nominales |
| Inicio inmutable durante replans | Aceptado. Solo nueva instancia obtiene otro inicio; congelar el dataclass exige usar `replace` para actualizar path/generación |
| Firma `(kind, target_node_id, edge_ids)` | Insuficiente como contrato de equivalencia. Añadir geometría recortada, dirección y semántica terminal; quitar IDs de instancia/target del criterio conductual. Los IDs de arista distinguen paralelas dentro del mismo snapshot |
| Omitir deduplicación con cohorte unitaria | Aceptable para evitar comparaciones entre alternativas, pero no para omitir validación de la asociación al plan. Una única propuesta con path inexistente o incoherente también deja evidencia incompleta |
| JSON: cero denominador → `None`, serializar con `allow_nan=False` | Aceptado como requisito de C. En A los contadores desconocidos por fallo/firma inválida también son `None`; no fabricar ceros |
| G choice >0 / E choice=0 antes de D | Útil como diagnóstico del perfil actual en fixtures que fuerzan alternativas de búsqueda. No exigirlo en todos los mapas o contextos ni considerarlo prueba de sensibilidad de genes. El CLI C deberá reportar estratos/contextos, no asumir tasas de un ensayo corto |
| Dwell explica todos los empates históricos | No demostrado. Con dwell positivo y mejora de utilidad, edad cero retiene la opción actual. Prioridad/urgencia pueden preemptar legítimamente. Dwell cero permite cambios por utilidad; falta de propuestas, features y objetivo son causas independientes. No hay evidencia causal suficiente para atribuir todo P6.3 a este fallo |
| Documento aprobado como especificación vinculante | Se conserva como opinión del dictamen externo. No sustituye arquitectura/TS/plan maestro ni aporta sign-off de aceptación del runtime |

Especificación de B:

```python
@dataclass(frozen=True)
class ActiveOption:
    stable_key: str
    option_instance_id: str
    goal: OptionGoal
    path: PlannedPath
    started_at_ns: int
    action_id: str
    lease_generation: int  # Generación vigente; actualizar con la autoridad.
```

`action_id` se fija en admisión. `started_at_ns` no se reescribe por replan, renovación de lease o nueva observación. `path` y generación se actualizan mediante reemplazo del estado conservando el inicio. Terminación, cancelación, revoke o cambio de epoch/etapa limpian el estado según el contrato de autoridad. Esta especificación se implementa y verifica en B (§12).

## 11. Implementación del paquete A

### 11.1 Código y contrato entregados

- [selection_diagnostics.py](../sim/kinematic/selection_diagnostics.py) contiene el contexto tipado, validación explícita de indicadores/cardinalidades y lectura del resultado final del selector. No recalcula guards ni utilidades.
- [autonomous.py](../sim/kinematic/autonomous.py) reinicia el contexto en cada llamada validada; marca invocación antes del selector y lo completa después; centraliza las trazas en `_record_trace()` para movimiento, observación, hold y todos los fallos que retornan por la política.
- La firma compara recorrido dirigido restante, geometría normalizada y efecto/tolerancias. Valida OPEN, versiones, target y correspondencia de la geometría al sufijo del plan. Las aristas históricas conservadas por `_clip_path_to_pose` se excluyen de la firma restante. No replantea ni modifica el plan.
- Readiness no evaluada es `None`; entradas inválidas anteriores al selector registran `selector_invoked=false` y contadores cero. Cada llamada tiene secuencia y stamp para evitar duplicaciones en C.
- Si falta un plan o la firma no se valida, la política mantiene su decisión original, pero registra `diagnostic_valid=false`, razón y contadores de elección desconocidos (`None`). Si el selector falla por `ValueError`, registra invocación incompleta, revoca movimiento y retorna parada. Excepciones inesperadas también registran parada/revoke y se propagan; no se convierten en un resultado táctico válido.
- La traza conserva features por key, evaluaciones finales con utilidades/contribuciones y evidencia de guards, cohorte, firmas, duplicados, candidato elegido, perfil y motivo de decisión. No introduce campos del árbitro como inputs.

**Schema:** `hsl26.policy-cycle.v2`; firma `hsl26.realized-behavior.v1`, tolerancia geométrica 1e-9 m. Los once argumentos de ejecución originales de `PolicyCycleTrace` siguen admitidos; el contexto es keyword-only. `proposal_total` incluye HOLD; `proposal_feasible_raw` cuenta el flag original sin HOLD; `proposal_feasible` cuenta aplicabilidad final sin HOLD; `proposal_choice` cuenta conductas distintas de la cohorte y puede valer 1. Las métricas del intento anterior no se migran ni relabelan.

El contrato de firma es local al snapshot: los IDs de arista preservan la distinción entre corredores paralelos. Una renumeración de aristas en otro snapshot no es una medida de divergencia conductual entre episodios. Para ello C/F deberán fijar snapshot común o usar una identidad geométrica de comparación entre snapshots.

### 11.2 Verificación y límites

Se añaden 32 casos parametrizados/unitarios en [test_selection_diagnostics.py](../sim/kinematic/test_selection_diagnostics.py) y nueve casos nuevos más aserciones nominales en [test_p56_autonomous.py](../sim/kinematic/test_p56_autonomous.py). Cubren partición, guards, cohortes, instancia duplicada, geometría redundante, paralelas, prefijos divergentes/ya recorridos, renumeración de target, tolerancias de efecto, observación sin ruta, firma incompleta, reset temprano y errores de selector/autoridad/candidato. La regresión conjunta termina con **190 passed en 36.45 s**, sin cache de pytest; son 149 casos existentes y 41 nuevos.

La comparación diferencial [verify_runtime.py](../artifacts/reports/coevolution_package_a_20261002/verify_runtime.py) ejecuta los cuatro fixtures training existentes, baseline congelado, seed 20261002 y horizonte 8 s. Contrasta opción, motivo, comando, estado de autoridad, lease, supervisor y outcome con el controlador previo a la instrumentación del commit fijo `8d1c6a96d1517bd953cb6d16da1dfe4d653d69e3`. No compara los campos nuevos con los defaults defectuosos como criterio de equivalencia. Requiere Git con esa revisión disponible y el log de regresión aprobado; registra su hash sin volver a ejecutar pytest.

Resultados y hashes: [runtime_verification.json](../artifacts/reports/coevolution_package_a_20261002/runtime_verification.json). Regresión: [tests.txt](../artifacts/reports/coevolution_package_a_20261002/tests.txt). La verificación diferencial confirma equivalencia de comportamiento en los cuatro casos y ausencia de diagnósticos incompletos nominales. Cada fixture registra 157 invocaciones ACTIVE por rol: Guardian tiene 157 oportunidades estructurales; Explorer, cero. Son datos de cuatro ensayos deterministas cortos, no mejora aprendida ni frecuencia generalizable.

**Salida histórica de A:** instrumentación implementada y verificada en desarrollo. B corrige posteriormente el reloj de dwell (§12). **Pendiente:** C debe transportar/agregar/exportar estos datos con su CLI y tasas, y D/F deben demostrar alternativas/sensibilidad. El trainer conserva su estado de desarrollo y su metadata previa; estas entregas no ejecutan poblaciones ni modifican gates.

## 12. Implementación y aceptación del paquete B

### 12.1 Estado y semántica temporal

[autonomous.py](../sim/kinematic/autonomous.py) sustituye la tupla privada por `ActiveOption` congelado. Valida inicio entero no negativo, instancia/goal coincidentes, generación admitida, opción de movimiento y correspondencia de target/versions/localización entre path y goal. Los campos se fijan tras admisión aceptada; una admisión rechazada no crea estado activo. `_active_action_id()` obtiene la acción del registro, incluso cuando su entrada desaparece del journal acotado de auditoría.

Para la misma instancia `i`, en el mismo clock epoch:

```text
t0(i) = tiempo de admisión aceptada
age(i, t) = t - t0(i) >= 0
t0(i) permanece constante en todos los replans
```

El caller pasa ese `t0` al selector. Un replan compatible actualiza path y generación mediante `replace`; conserva goal admitido, action ID, instance ID e inicio. Se reutiliza el goal inmutable al volver a generar su propuesta. Un cambio de goal bajo la misma identidad se rechaza y revoca; no se renueva su deadline silenciosamente.

Si la opción actual sigue siendo aplicable y una alternativa tiene igual prioridad y urgencia, el cambio por utilidad exige simultáneamente:

```text
U_alternativa > U_actual + hysteresis_delta_u
age >= minimum_dwell_ns
```

La igualdad en utilidad más delta retiene; la igualdad en el límite de dwell permite cambiar. Prioridad superior o urgencia superior dentro de la misma prioridad preemptan antes del dwell. Una opción ya inaplicable no obtiene protección por dwell. Los gates de readiness, lease, fase, autorización y safety siguen precediendo al ranking. No se modifica `fsm.py`, ni las prioridades, pesos, márgenes o permisos del supervisor.

### 12.2 Replan, revocación y confirmación de efectos

`begin_replan` revoca autorización y avanza generación en la autoridad existente. El registro sigue esa generación. Solo se produce un candidato si `mark_executing` retorna realmente EXECUTING con autorización; puede terminar la opción durante la validación sin lanzar excepción. Todos los candidatos de generaciones anteriores o instancias canceladas siguen rechazados.

Se limpia `_active` tras sustitución, cancelación, fallo de plan/candidato, revocación, STOP del supervisor y terminación confirmada. Si la autoridad ya publicó un resultado, se limpia el cache sin cancelar dos veces. Si falla la creación del registro después de una admisión aceptada, se cierra también su lease de planificación.

Antes de confirmar llegada se valida que el path corresponde al goal admitido y que su endpoint coincide con la geometría del nodo target del grafo, con tolerancia de identidad 1e-9 m. La llegada satisface `dist(pose, target) + error_bound <= goal.position_tolerance_m`, usando las coordenadas del nodo admitido, no el endpoint aproximado de la ruta; solo un resultado real SUCCESS de la autoridad añade el nodo a completados. Llegar al endpoint de una ruta ajena, de otra versión o con geometría incoherente no genera éxito. La tolerancia de identidad geométrica no amplía la tolerancia de llegada ni modifica ningún margen de seguridad; una prueba con discrepancia inferior a 1e-9 m verifica el límite exacto.

La revisión encontró y reprodujo ese falso positivo de llegada en tres casos: target distinto, versión distinta y endpoint geométrico distinto. Los tests fallaban con `option_effect_satisfied` antes de añadir la validación. El verificador conserva controles ejecutables contra la fuente A congelada: A confirma esos efectos incorrectamente; B los rechaza y revoca. No se interpreta una traza de diagnóstico inválida como evidencia válida de éxito.

### 12.3 Cambios de contexto y reloj

Un cambio coherente de etapa, localization epoch, map version o topology version detiene el primer ciclo del nuevo contexto, revoca la instancia anterior y limpia historial de nodos, caches de rutas y filtro rival. El siguiente ciclo coherente puede admitir una instancia nueva, con nuevo inicio y generación creciente. La invalidación también funciona sin opción activa, para no conservar nodos completados de otra versión. El latch de seguridad permanece activo durante estas limpiezas.

Esta política y su autoridad pertenecen a **un único clock epoch**. Si cambia, la revocación se fecha con el último stamp conocido de la autoridad anterior, sin comparar, restar ni reutilizar stamps del nuevo epoch. Ese fin es un límite conservador conocido del reloj anterior; no mide duración física entre epochs. La instancia queda detenida, incluso si después reaparece el epoch viejo. Para el nuevo epoch se requiere una política nueva en un contexto de ejecución nuevo; B no introduce un reset interno de tiempos, generaciones ni un rearme automático de seguridad.

El SIL actual conserva el epoch durante el episodio. La integración de un reinicio operativo entre epochs y su aceptación ROS/hardware quedan en sus planes propios. Las garantías de leases aquí verificadas corresponden a la autoridad existente y su vida útil; no se afirma interoperabilidad de candidatos entre autoridades independientes.

### 12.4 Evidencia reproducible y límites

[test_active_option_lifecycle.py](../sim/kinematic/test_active_option_lifecycle.py) añade **40 casos** de integración y validación. Cubren ambos roles, dwell en D−1/D, inicio nuevo, replans 1/5/12 con journal acotado, histéresis exacta, preempción por prioridad/urgencia, dwell cero, opción actual ausente, paradas duras, cambios de contexto/reloj, fallos de admisión/plan, STOP y latch, terminación externa, efectos y candidatos obsoletos. Los controles de utilidad y del límite de llegada usan valores representables exactamente en binario, para distinguir errores lógicos de redondeo en los umbrales.

La regresión conjunta del comando §9 cubre A/B, táctica, autoridad, planificación/control, safety, match, evolución y herramientas existentes: **419 pruebas aprobadas**. Log y duración exacta: [tests.txt](../artifacts/reports/coevolution_package_b_20261002/tests.txt).

[verify_runtime.py](../artifacts/reports/coevolution_package_b_20261002/verify_runtime.py) usa una [copia fija de A](../artifacts/reports/coevolution_package_b_20261002/reference_a_autonomous.py), SHA-256 `0d7b1a748de5839b5c36aa32597bfbc5d5afabc6f6a0b89a07024c5e0d9e37ab`. En los cuatro fixtures training, seed 20261002 y horizonte 8 s, comprueba:

1. Con dwell=0 e histéresis conservada, igualdad de decisiones, motivos, comandos, fases/razones de autoridad, instancias, generaciones, autorización, decisiones del supervisor y outcomes frente a A.
2. Con dwell=250 ms, caller/start/autoridad coincidentes, edades positivas y ningún reinicio dentro de la instancia. En cada fixture la edad máxima observada es 4.25 s Guardian y 7.8 s Explorer; no queda permanentemente en cero.
3. Para ambos roles, alternativas controladas con igual prioridad/urgencia y mejora de utilidad: B cambia al llegar a 250 ms; emular `start=now` conserva edad cero y bloquea. Este control prueba la corrección sin depender de que un rollout nominal contenga espontáneamente la divergencia adecuada.
4. Los tres controles de endpoint reproducen el falso SUCCESS en A y lo rechazan en B. Todos estos wrappers se restauran al acabar; no se usan en producción.

Resultados, hashes reales de fuentes, genomas y log: [runtime_verification.json](../artifacts/reports/coevolution_package_b_20261002/runtime_verification.json). La comparación con dwell cero protege el comportamiento nominal previo; con dwell positivo se verifica la corrección temporal, sin exigir conservar el defecto. Los resultados son DEVELOPMENT_ONLY, no promocionables. Los TIMEOUT de 8 s son outcomes del perfil corto, no victorias oficiales ni evidencia de mejora aprendida. Las mediciones de pared incluyen wrappers y calentamiento; no acreditan latencia física ni coste sostenido.

**Salida de B:** lifecycle e inicio real implementados, confirmación de efectos protegida y regresiones verificadas en desarrollo. **Siguiente paquete:** C, transporte/agregación/exportación del diagnóstico sobre training, con hashes, particiones, tasas y evidencia incompleta explícitos. D/F, conformidad, piloto por rol, liga y release conservan las condiciones de salida originales.
