# HSL26 — Plan maestro de integración y preparación competitiva

**Estado:** plan de ejecución; no es evidencia de aceptación. **Base revisada:** `HSL26_FINAL_ARCHITECTURE.md` rev. 2.2, `HSL26_TECHNICAL_SPECIFICATION.md` rev. 2.2, planes P1–P6, código y reportes existentes hasta 2026-09-30 y la transcripción suministrada `docs/Регламент HSL26 - v06092026.md` (seis páginas marcadas). El texto Markdown confirma las reglas operativas principales; su equivalencia con el PDF original y cualquier aclaración posterior del organizador siguen pendientes de verificación.

## 1. Objetivo y criterio de éxito

Entregar un único sistema de decisión, navegación y seguridad que pueda operar como Guardian o Explorer, con entradas de sensores adaptadas a tres entornos: simulador cinemático, MVSim/ROS 2 y robot físico. Entrenar tácticas de ambos roles mediante partidas entre políticas, seleccionar candidatos reproducibles y desplegar solamente artefactos que superen evaluación independiente y las puertas de seguridad. El rendimiento competitivo se medirá con los resultados autorizados del concurso; ninguna prueba de software garantiza ganar.

La secuencia tiene tres líneas de trabajo con una dependencia de aceptación:

1. [Integración ROS 2 y MVSim](HSL26_ROS_MVSIM_INTEGRATION_PLAN.md): convertir interfaces y lanzadores en procesos ejecutables; cerrar la ruta observación → decisión → seguridad → mux → planta.
2. [Auditoría de conformidad SIL](HSL26_SIL_CONFORMANCE_PLAN.md): demostrar qué partes del simulador siguen la arquitectura y el reglamento, distinguir perfiles con y sin percepción 3D y fijar un baseline G4 válido.
3. [Coevolución y selección](HSL26_COEVOLUTION_PLAN.md): ampliar el entrenamiento de ambos roles, cruzar oponentes y transferir candidatos entre SIL, MVSim y hardware.

El trabajo puro de estas líneas puede avanzar en paralelo. **La recolección de datos elegibles, la promoción G5 y la liberación G6 dependen de un baseline G4 aceptado para el perfil correspondiente.** Las puertas G0–G3 de ROS/hardware siguen siendo independientes de un `PASS` cinemático.

## 2. Estado inicial que no debe reinterpretarse

| Componente | Estado observado | Evidencia y límite |
| --- | --- | --- |
| Núcleo y SIL | Suite de núcleo/cinemático aprobada; integración P5.6 de ambos roles en un perfil sintético limitado | `sim/kinematic/autonomous.py`, `benchmark.py`, reporte P5.6; no acredita arquitectura completa, GPIS real, ROS ni G4 |
| MVSim | `simulation_adapter.py` sin implementación; lanzador solo informa que falta conexión | Sin dinámica/sensores/actuación integrados |
| ROS | IDL y adaptadores parciales; paquetes de percepción, mundo, decisión, navegación y partido son esqueletos; `main()` de supervisor/watchdog lanza excepción | Ningún grafo extremo a extremo ni stop físico validado |
| Aprendizaje | P6.1/P6.2 tienen contratos y algoritmos puros; P6.3 ejecuta búsqueda de fixture y retuvo el baseline | No hubo mejora respaldada, dataset elegible, evaluación held-out ni G5 |
| Release | Dossier y ensayo de dos etapas SIL | G4 `BLOCKED_NOT_RUN`, G5 `NOT_RUN`, G6 `BLOCKED`; imagen final y robot sin validar |

## 3. Decisiones de diseño fijadas

- El reglamento y aclaraciones autorizadas prevalecen sobre arquitectura, especificación técnica y código. Las discrepancias se registran como ADR, con cambio de versión y pruebas.
- Se reutiliza `hsl_core` para reglas, opciones, planificador, control y seguridad. Los nodos ROS convierten mensajes y gestionan tiempo, QoS, leases y procesos; no duplican algoritmos.
- Cada política ve solo sensores, estados estimados y metadatos autorizados de su robot. Planta y árbitro poseen verdad; ninguna política recibe pose verdadera rival, mapa no observado ni resultado oficial anticipado.
- Solo el mux escribe `/commands/velocity`; el supervisor escribe `/hsl/cmd_vel_final` y el watchdog escribe un canal de stop prioritario. Sin salud mutua, datos frescos y lease de etapa, la salida es cero.
- Un perfil de simulación tiene identidad y fidelidad explícitas. Un resultado `kinematic/observed_map` no acredita reconocimiento de nube 3D; una nube MVSim genérica tampoco acredita equivalencia MID-360.
- G5 es opcional. Si ningún candidato supera al baseline aceptado, se conserva ese baseline para G6.

## 4. Puertas y entregables

| Hito | Condición verificable | Artefacto esperado |
| --- | --- | --- |
| M0: reglas congeladas | Transcripción Markdown cotejada con R1–R19; PDF autorizado cotejado cuando esté disponible; entradas desconocidas registradas; fixtures normativos corregidos | matriz R1–R19 con página, decisión, test, hash de fuente y responsable |
| M1: G0 de software/ROS | Imagen reproducible; interfaces generadas; procesos arrancan; grafo de tópicos/TF y autoridad correctos; fallos de proceso producen cero | reportes I01–I05, trazas ROS, hash de imagen/config |
| M2: MVSim caracterizado | Versión/API/sensor/tiempo medidos; dos namespaces; políticas sin verdad; comandos pasan por supervisor | capability report, bags, grafo ROS, escenarios I06–I14 según fidelidad |
| M3: SIL conforme | Matriz arquitectura/reglamento satisfecha en perfil declarado; ejecución de ambos roles, carrera de acciones y eventos continuos; baseline revisado | matriz T01–T30/I11–I14, trazas observación→actuación y aceptación G4 por perfil |
| M4: datos y coevolución | Bancos independientes, episodios completos y censurados, torneo cruzado, selección por validación, held-out una vez | manifests de dataset, poblaciones, matrices de torneos y evaluación G5 |
| M5: release | Política compatible y firmada; dos etapas de duración autorizada, roles intercambiados, arranque sin red, rollback y evidencia física | manifest final, runbook, I16/G6, informe de limitaciones |

Cada reporte debe registrar comando exacto, revisión/hash, imagen, perfil, mapa/oponente/semilla, reloj, condiciones del robot, logs crudos, esperado/observado, estado `PASS`/`FAIL`/`BLOCKED`/`NOT_RUN` y revisor. Una prueba solo cuenta para el entorno donde se ejecutó.

## 5. Riesgos que gobiernan la planificación

1. **Semántica oficial incompleta:** el texto suministrado confirma etapas de 10 minutos con cuatro minutos de preparación sin cruzar la línea, cambio de roles y suma de puntos de las dos etapas. Puntos numéricos, origen preciso de zonas, retención de mapa entre etapas, señal técnica de inicio, resolución de eventos simultáneos y cambios posteriores del organizador no se inventan. El modo sin zona aceptada puede sobrevivir o buscar, pero no afirmar llegada.
2. **Captura físicamente difícil:** la condición `d < 0.45 m` y la separación real de huellas pueden dejar un intervalo de alcance muy estrecho. Medir radios, errores y control de aproximación antes de optimizar la táctica.
3. **Brecha de percepción:** GPIS requiere nube 3D y transformada modelo→base válidas. Registrar dominio de soporte, error de origen, yaw válido, falsos positivos, oclusión y coste CPU; usar degradación etiquetada si no se alcanza el umbral.
4. **Entrenamiento estéril o sobreajustado:** los experimentos guardados reportan diferencias cero y baseline retenido. Medir diversidad de decisiones, duraciones, capturas, llegadas y calidad de emparejamiento antes de aumentar generaciones.
5. **Simulación demasiado ideal:** variar latencia, slip, obstáculos, geometría, LiDAR y rival con rangos basados en medición; conservar estratos separados para diagnósticos y promoción.

## 6. Orden de revisión y control de cambios

La primera revisión aprueba M0 y los contratos comunes. Después, cada entrega incremental produce código, pruebas negativas y evidencia de grafo o traza; el revisor decide la puerta, no el autor del cambio. Antes de entrenar, se congela un baseline determinista para cada rol y un conjunto de bancos. Antes de liberar, se congelan hashes de código, IDL, configuración, calibración, modelo GPIS y políticas; un cambio obliga a repetir la evidencia afectada.
