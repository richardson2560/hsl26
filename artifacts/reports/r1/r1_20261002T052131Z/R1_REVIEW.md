# R1 — revisión de inventario y autoridad del grafo

- **Run ID:** `r1_20261002T052131Z`
- **Checkout:** `a277630a4b74b1d096ddc62cd37ba808d55e288f` (el estado de trabajo
  no estaba limpio y quedó registrado en `git-status.txt`).
- **Imagen candidata:**
  `hsl26@sha256:aa6852929a8f26eafec66f47991139466de550c614398447bb54711939f9011f`
  para `linux/amd64`.
- **Contrato de grafo:** SHA-256
  `7a9da872a358d0e22d26169880e81f3a2cda05591d623db15751b5870c55b2d1`.

| Caso | Veredicto | Evidencia |
| --- | --- | --- |
| R1-A | `PASS` | Inventario JSON versionado, hash y configuración instalada en el package share. |
| R1-B | `PASS` | `r1-live-graph.json` y `commands-velocity-verbose.txt`: sólo `/cmd_vel_mux` publica `/commands/velocity` con `geometry_msgs/msg/Twist`. |
| R1-C | `PASS` | El fixture de escritor adicional fue rechazado; el fixture `best_effort` → `reliable` fue rechazado por QoS. Ambos `FAIL` son resultados negativos esperados. |
| R1-D | `PASS` | TF válido contiene las dos aristas estáticas esperadas; TF duplicado fue rechazado; tras reiniciar el mux, `r1-after-restart.json` volvió a `PASS` con un solo publicador físico. |

## Conclusión técnica

R1 está `PASS` para el alcance declarado: contrato de grafo, autoridad física,
compatibilidad QoS, estructura TF estática y recuperación del proceso mux. Los
fixtures se ejecutaron en un contenedor `--network none`, sin base, sensores ni
comandos no nulos.

No constituye evidencia de R2: no prueba supervisor/watchdog, arranque seguro
de la cadena de movimiento, timeout de driver ni hardware. Tampoco acredita el
grafo de dos robots de R8. La decisión formal de puerta sigue requiriendo la
revisión independiente que establecen los planes maestros.
