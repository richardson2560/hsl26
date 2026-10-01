# Plan para construir el modelo HGW del robot HSL26

**Fecha de auditoría:** 2026-10-01  
**Estado:** plan de trabajo; no se genera ni promueve todavía un modelo de producción  
**Alcance:** modelo geométrico Hermite-GPIS-W del Kobuki/TurtleBot2 usado por ambos participantes de la competencia simétrica  
**Autoridad técnica:** [especificación técnica HSL26](./HSL26_TECHNICAL_SPECIFICATION.md), [arquitectura final HSL26](./HSL26_FINAL_ARCHITECTURE.md) y documentos de P4

## 1. Decisión ejecutiva

El repositorio ya implementa el núcleo matemático HGW y su registro planar en P4.1–P4.2, pero todavía **no contiene un modelo geométrico HGW entrenado con datos del robot real**. El entrenamiento disponible recibe observaciones Hermite preconstruidas en JSON; no importa ni procesa CAD/DAE. Los informes existentes son evidencia de pruebas SIL y declaran explícitamente bloqueada la aceptación con nube real.

Hay dos mallas COLLADA locales —carcasa principal y rueda—, pero no constituyen por sí solas una representación verificada del robot completo. El Xacro agrega geometría de colisión simplificada para base, ruedas y ruedas locas; en el inventario local no se encontró geometría CAD del MID-360, su soporte, cableado u otros elementos montados que formen parte de la superficie visible. Además, no hay una calibración validada del transform entre el marco del modelo y `base_link`.

**Recomendación:** tratar las mallas actuales como fuentes candidatas, no como modelo de producción. Primero verificar la configuración física exacta compartida por ambos robots; después completar y calibrar la geometría visible, implementar el puente CAD/escaneos → observaciones HGW y aceptar el artefacto solo con vistas reales de validación independientes. No inventar geometría faltante ni declarar fidelidad física a partir de una visualización CAD incompleta.

**Actualización de prioridad tras revisar la foto del hardware y conocer los bags de la prueba previa:** para modelar el blanco del concurso, los bags capturados con el ensamblaje real deben ser la primera fuente empírica que se audite. La foto muestra una estructura elevada propia —plataformas, postes, sensor suspendido, equipo y cableado— que no aparece en las dos mallas del Kobuki. El plan no debe esperar pasivamente por CAD completo ni entrenar el HGW final con el Kobuki solo. Las mallas del Kobuki quedan como geometría de referencia/base; los bags pueden aportar la envolvente real observada y la validación Livox, si los campos temporales, TF y movimiento permiten reconstruirla correctamente.

## 2. Hallazgos del estado actual

### 2.1 Geometría y descripción del robot

| Evidencia revisada | Estado observado | Consecuencia |
|---|---|---|
| `kobuki_description/meshes/main_body.dae` | COLLADA 1.4.1, unidades declaradas en metros; describe la carcasa principal. Metadatos de creación de 2013. | Fuente visual útil, pero no prueba de que incluya todos los elementos observables del robot de competencia. |
| `kobuki_description/meshes/wheel.dae` | COLLADA 1.4.1, unidades declaradas en metros; malla de rueda. | Completa parte de la geometría, no el ensamblaje completo. |
| `kobuki_description/urdf/kobuki.urdf.xacro` | Usa las dos mallas como visuales. Colisión de la base cilíndrica, ruedas cilíndricas y ruedas locas simplificadas. Declara dimensiones y offsets del modelo Kobuki. | URDF/Xacro es una referencia de ensamblaje y marcos, no una malla superficial completa para ajustar puntos de una nube. |
| `kobuki_description/package.xml` | Declara licencia BSD y apunta al repositorio `kobuki-base/kobuki_ros`. | Registrar procedencia y conservar atribución/licencia al exportar o transformar geometría. |
| `config/frames.yaml` | Declara `lidar: lidar_link`. | El nombre lógico del frame está configurado, pero no se encontró aquí una descripción Xacro local que modele el MID-360 y su montaje. Hay que reconciliar el árbol de TF real con el modelo. |
| Árbol de archivos CAD/DAE local | Solo aparecen `main_body.dae` y `wheel.dae`; no se encontró STEP/STP, ensamblaje DAE completo ni nube real registrada. La carcasa contiene 4,222 vértices y 8,440 triángulos; la rueda, 2,396 vértices y 4,788 triángulos. | Sí son mallas triangulares muestreables para el Kobuki base/rueda, pero no son una nube de puntos ni el ensamblaje completo. |

Los DAE locales tienen coordenadas de geometría y unidades declaradas en metros. Se contrastaron sus identificadores Git blob contra `kobuki-base/kobuki_ros` commit `e1fbf7bdec199411008bd7c94c6e4b7d82903347`: `main_body.dae` = `9606185ab277645cfa7e0d7fbbffe0ce3978ae66` y `wheel.dae` = `014f5970ddf693c909badfb8779d36f7eafc37a4`. Esto fija una procedencia reproducible para las dos mallas, pero no prueba que representen cada modificación de los robots de competencia.

**Importante:** no usar la lista de vértices como si fueran observaciones uniformes. Al preparar entrenamiento se deben muestrear las caras en proporción a su área, transformar cada instancia (ruedas izquierda/derecha incluidas) según el Xacro y generar normales orientadas. Las mallas de color/material dependen de imágenes relativas que deben acompañar al DAE en su carpeta de recurso. Verificar ejes, escala, offset y envolvente resultantes en RViz/Blender/MeshLab antes del muestreo.

La condición de competencia simétrica permite reutilizar el mismo modelo geométrico para cualquiera de los dos robots **si se verifica que ambos usan la misma revisión, accesorios, ubicación/orientación del LiDAR y configuración física**. Esa simetría no sustituye la verificación dimensional ni la calibración de marcos.

### 2.2 HGW y pipeline implementado

| Componente | Implementación actual | Límite para este trabajo |
|---|---|---|
| `hsl_core/perception/implicit_surface.py` | Kernel Wendland C4 3D `wendland_c4_d3_unit_center_v1`, operadores de valor y derivada direccional, covarianza Hermite, posterior con Cholesky, gradiente/varianza, control de soporte y serialización NPZ sin pickle. | El modelo es ajustado desde observaciones ya preparadas; la extracción de geometría CAD, orientación de normales y creación de anclas con signo no está implementada. |
| `tools/train_gpis_prior.py` | Lee `observations` desde JSON y produce `model.npz` y `manifest.json`. Registra el hash del JSON de entrada. | No acepta directamente DAE/CAD; el transform guardado actualmente es identidad. No basta para establecer `T_B_M` de un robot real. |
| `tools/validate_gpis_prior.py` | Verifica hash, versión de esquema/kernel, arrays no ejecutables, formas, finitud y parte de la estructura rígida del transform. | Es validación de integridad/esquema, no de exactitud geométrica, sesgo del origen, observabilidad ni fidelidad de nube real. |
| `hsl_core/perception/registration.py` | Registro robusto planar acotado de traslación y yaw sobre el campo; puede rechazar soporte insuficiente y soluciones degeneradas. | Sus resultados SIL no demuestran rendimiento con nubes Livox ni con el robot completo. |
| Artefactos | Los informes P4.1, P4.2 y P4.5 existen en `artifacts/reports/phase4/`. No se encontró `artifacts/models/opponent_gpis/model.npz` ni artefacto de producción equivalente. | No hay un modelo físico actualmente listo para integrar. |

### 2.3 Informes e historial contrastados

- [Informe P4.1](../artifacts/reports/phase4/P4.1_gpis_prior_report.json): **PASS SIL**, kernel C4; declara que no se promovió un modelo de producción y que faltan geometría completa, nubes registradas, calibración `T_B_M` y vistas disjuntas de validación.
- [Informe P4.2](../artifacts/reports/phase4/P4.2_registration_report.json): **PASS SIL con bloqueos de nube real**; reporta 6 pruebas focalizadas y 164 pruebas en regresión al momento del informe. Declara bloqueados el error real del origen, tasa de falsos positivos, covarianza calibrada y `T_B_M`.
- [Informe P4.5](../artifacts/reports/phase4/P4.5_runtime_fidelity_report.json): **PASS de contratos SIL**, pero perfil de nube real bloqueado por falta de robot/bolsas registradas; MVSim bloqueado hasta inspeccionar su modelo de sensor.
- [Baseline de P4](../artifacts/reports/phase4/P4_environment_baseline.json): confirma que la simulación cinemática y detecciones sintéticas no constituyen evidencia de fidelidad de nube ni de aceptación física.
- [Auditoría independiente de P6](./HSL26_PHASE6_INDEPENDENT_AUDIT.md): ubica correctamente la producción GPIS desde CAD dentro de P4; P6 solo debe fijar un artefacto aceptado si el perfil de despliegue lo consume.
- El changelog registra la selección normativa de C4 en lugar del kernel C2 didáctico anterior. Sin embargo, el docstring del módulo y la sección 4 de `HSL26_PHASE4_IMPLEMENTATION_PLAN.md` aún mencionan C2. Esta inconsistencia documental debe corregirse antes de usar esos textos como instrucciones de implementación; la especificación técnica y el identificador de kernel vigente son la autoridad.
- La documentación de P4 enlaza `main_hgw.tex`, pero el archivo no aparece en el checkout revisado. La trazabilidad matemática disponible debe apoyarse en la especificación técnica vigente hasta que se aporte esa fuente, si es necesaria para el proyecto.

### 2.4 Recursos externos de geometría

**Kobuki.** El repositorio activo `kobuki-base/kobuki_ros` contiene las mismas mallas que el checkout local, en el commit indicado arriba. Son la mejor fuente descargable localizada para carcasa y ruedas del Kobuki; el Xacro y el `package.xml` del paquete de descripción declaran licencia BSD. No se encontró en ese paquete una malla de ensamblaje que incorpore el MID-360 ni su soporte.

**Livox MID-360.** La página oficial de descargas consultada publica Livox Viewer 2 y su manual, pero no presenta una malla CAD/STEP descargable. Se localizó una opción comunitaria en `jfrascon/robotics_description`, commit `c731dd26920a604caf985f8f0461267c702a7f55`: contiene [OBJ](https://github.com/jfrascon/robotics_description/blob/c731dd26920a604caf985f8f0461267c702a7f55/meshes/sensors/lidars/livox_mid360/livox_mid360.obj), [STL](https://github.com/jfrascon/robotics_description/blob/c731dd26920a604caf985f8f0461267c702a7f55/meshes/sensors/lidars/livox_mid360/livox_mid360.stl), material MTL y una macro Xacro que define montaje, escala y uso de mallas. El `package.xml` declara Apache-2.0; aun así, conservar licencia/procedencia y verificar si los archivos de malla tienen atribuciones adicionales. La macro describe el sensor como aproximación de caja de 65 × 65 × 60 mm, no como CAD oficial de Livox ni como verificación dimensional independiente. Es candidato visual para la escena, sujeto a comparar con el equipo físico.

### 2.5 Encaje y límite de fidelidad de MVSim

La documentación oficial de MVSim permite usar un DAE en `<visual><model_uri>…</model_uri></visual>` y configura por separado escala/offset visual, dinámica diferencial y huella del chasis. Su catálogo de modelos incluye TurtleBot3, no se encontró un modelo Kobuki/TurtleBot2 con el ensamblaje HSL26. Es razonable crear una clase diferencial propia con la malla Kobuki compuesta más el OBJ/STL Mid-360, mientras se declaran explícitamente las cotas y la huella de colisión del Kobuki. La malla visual no define por sí sola la física ni los límites seguros.

MVSim documenta perfiles 3D como Velodyne VLP-16, Ouster OS1 y Hesai Helios; la búsqueda en documentación/código oficial no encontró una implementación Mid-360/Livox. Aunque el sensor `lidar3d` admita rayos verticales personalizados y una malla visual, eso no reproduce el patrón de escaneo no repetitivo, timestamps por punto ni comportamiento de retorno del MID-360. Por tanto:

- **MVSim sirve ya** para movimiento, oclusión geométrica, visualización del robot ensamblado y pruebas de integración con un LiDAR sintético declarado como aproximación.
- **MVSim no valida por sí solo** fidelidad de nube Mid-360 ni aceptación real del GPIS.
- No etiquetar un perfil simulado Velodyne/OS1/Hesai como `SIM_CLOUD` equivalente a Livox sin una comparación cuantitativa específica; registrar el perfil como aproximado.
- Mantener el modelo CAD/visual, huella física y modelo de sensor como tres artefactos/versiones separados.

### 2.6 Bags de la prueba y capacidad del driver

En el checkout **no hay archivos de rosbag ni nubes PCD/PLY/LAS**, por lo que todavía no se han inspeccionado las grabaciones que mencionas. Esta ausencia no significa que los bags de la prueba no existan fuera del repo.

El driver vendorizado en `kobuki/workspace/src/livox_ros_driver2` proporciona evidencia útil para revisar esos bags:

- `launch/mid360.launch.py` selecciona `xfer_format = 0`, que genera `sensor_msgs/PointCloud2` con campos `x`, `y`, `z`, `intensity`, `tag`, `line` y `timestamp`.
- `src/lddc.cpp` escribe por punto `timestamp = pkg.points[i].offset_time`, mientras que `header.stamp` se inicializa con `pkg.base_time`. La interfaz `CustomMsg`, alternativa `xfer_format = 1`, conserva explícitamente `timebase` y `CustomPoint.offset_time` en nanosegundos.
- Por tanto, los bags podrían conservar tiempos por punto suficientes para deskew, pero no se debe asumir que el campo `PointCloud2.timestamp` está en segundos ni que su época/unidad queda clara por el nombre. Hay que cotejar los valores y diferencias con `header.stamp`, `CustomMsg` si está disponible y la implementación/configuración exacta del driver usada durante la prueba.
- El launch local publica además una transformación estática de ejemplo `base_link → livox` (`0 0 0.3`, `PI/2 PI 0`). No es evidencia de la pose real del LiDAR que aparece en la foto ni debe usarse para reconstrucción hasta contrastarla con `/tf_static`, medidas del montaje y calibración.

Los informes de P2 ya incluyen `deskew_cloud(cloud, pose_history, T_B_L, t_ref)` y validaciones de tiempos por punto, pero el reporte P2.2 deja la semántica temporal real del Livox como `BLOCKED`. El deskew existente compensa el movimiento del **sensor/robot observador**; no compensa automáticamente la rotación del **robot oponente**.

## 3. Objetivo del artefacto

El resultado final será un prior geométrico HGW versionado para estimar el origen y, cuando la geometría lo permita, la orientación planar del robot observado a partir de una nube parcial.

El modelo:

1. Representará la superficie **externa y observable** del ensamblaje de competencia, en un marco de modelo explícito.
2. Mantendrá unidades físicas, escala, orientación de normales y una relación calibrada con `base_link`.
3. Usará observaciones Hermite (valores de campo y derivadas direccionales), con anclas firmadas dentro/fuera cuando el método de construcción las requiera.
4. No se interpretará necesariamente como distancia firmada global exacta; el GPIS de soporte compacto solo es válido en el dominio donde hay soporte suficiente.
5. No reemplazará el detector de obstáculos de seguridad: los retornos físicos completos siguen alimentando la ruta de colisión, aunque se rechace un candidato semántico.

La simetría de la competencia reduce la cantidad de modelos geométricos, no la cantidad de evidencia: habrá que demostrar que una única geometría aplica a ambos robots. Si existen diferencias, se definirán variantes explícitas con IDs y hashes propios; no se mezclarán en un solo ajuste.

## 4. Trabajo propuesto y dependencias

### Fase 0 — Congelar alcance y procedencia

**Entradas:** inventario físico de ambos robots, documentación del montaje, permiso/procedencia de CAD.  
**Acciones:**

- Confirmar fabricante/revisión del Kobuki, dimensiones y variante de batería.
- Verificar que ambos participantes montan el mismo MID-360, soporte y accesorios; documentar piezas ausentes o variantes.
- Obtener autorización y procedencia verificable del CAD completo; preferir CAD del ensamblaje de competencia frente a una malla visual antigua.
- Registrar versión de archivos fuente, licencia, unidades declaradas y SHA-256. No incluir datos de red o configuración privada innecesaria en el artefacto geométrico.

**Salida:** manifiesto de fuentes aprobadas y lista de discrepancias.  
**Gate G0:** no entrenar para producción hasta resolver identidad, alcance y procedencia de la geometría.

### Fase 1 — Completar y normalizar geometría

**Entradas:** CAD/STEP/DAE verificado, Xacro/URDF y mediciones del ensamblaje real.  
**Acciones:**

- Importar el ensamblaje completo preservando jerarquía y unidades; revisar manualmente escala y ejes antes de convertir.
- Unificar las piezas que realmente forman la superficie visible: base, ruedas que sobresalgan, LiDAR, soporte, cubiertas y montajes.
- Excluir tornillería interna, superficies ocultas y elementos que no pertenezcan a la configuración compartida; conservar detalles que generen retornos del sensor.
- Corregir mallas degeneradas, caras duplicadas, agujeros no físicos y normales inconsistentes sin suavizar bordes o alterar la envolvente observada.
- Generar una malla derivada para entrenamiento, separada de los originales, con herramienta/versión/parámetros documentados.
- Comparar la envolvente contra cotas oficiales y mediciones del robot; inspeccionar overlays CAD–URDF–escaneo desde varios ángulos.

**Salida:** modelo geométrico de trabajo en unidades SI, reporte de integridad y hash del original y derivados.  
**Gate G1:** tolerancias dimensionales acordadas y verificadas; no se fijan aquí valores numéricos sin el plano, resolución del sensor y requisitos de registro.

### Fase 2 — Definir marcos y calibración

**Entradas:** malla validada, `base_link`, frames del sensor y TF medido.  
**Acciones:**

- Declarar el marco del modelo `M` y el origen de referencia (recomendado: origen mecánico estable relacionado con el eje de ruedas/base del Kobuki).
- Establecer la convención de transform; conservar sin ambigüedad `T_B_M`/`base_from_model` y comprobarla contra el registro ROS del robot.
- Medir/estimar el transform sensor–base (`T_B_L`) con procedimiento y covarianza; validar que concuerda con el TF observado y con montaje físico.
- Calibrar o derivar el transform modelo–base a partir del CAD de ensamblaje, y contrastarlo contra mediciones físicas independientes. No reemplazar un transform desconocido por la matriz identidad.
- Documentar incertidumbre dimensional, de montaje y de calibración que entrará en los pesos del registro.

**Salida:** ficha de marcos y transform rígido con evidencia y covarianza.  
**Gate G2:** convención, orientación, escala y transform validados por revisión independiente.

### Fase 3 — Preparar observaciones HGW desde CAD

**Entradas:** malla completa validada y transform del Gate G2.  
**Acciones:**

- Definir un preprocesador reproducible para leer la malla y generar puntos de superficie con muestreo de densidad controlada.
- Obtener normales orientadas coherentemente; verificar signo y continuidad en bordes, superficies delgadas y componentes desconectados.
- Construir observaciones de valor/derivada y anclas de signo solo mediante una regla geométrica probada para la representación elegida. Evitar asumir que cualquier malla cerrada es orientable o que su interior está bien definido.
- Aplicar límites de número de puntos y memoria antes de formar la matriz Hermite densa; medir condición, duplicados, cobertura y conectividad.
- Preservar el hash de cada fuente, configuración, versión de herramientas y partición de datos. No generar validación usando la misma vista/fragmento que se usó para entrenar.

**Salida:** dataset intermedio versionado de observaciones Hermite y reporte de calidad.  
**Gate G3:** pruebas de unidades, orientación de normales, consistencia de signo, soporte, condición numérica y reproducibilidad.

### Fase 4 — Integrar el preprocesador con P4.1

**Cambios previstos, aún no implementados:**

- Extender el flujo actual basado en JSON con un paso separado CAD/mesh → observaciones; mantener `HermiteGPIS` puro y desacoplado de ROS.
- Exigir parámetros de kernel, unidades, marco, transform, política de ruido y parámetros de muestreo explícitos y tipados.
- Corregir el guardado para recibir y persistir el transform real, no escribir identidad por defecto en un artefacto físico.
- Ampliar el manifiesto con procedencia y hashes de archivos CAD originales y derivados, herramienta/conversión, transform/covarianza, configuración y splits de validación.
- Mantener NPZ con arrays no ejecutables y `allow_pickle=False`. El validador debe comprobar la rigidez física del transform (incluida orientación propia, no reflexión), coherencia de metadatos, hashes y consistencia entre manifiesto y arrays.
- Alinear el docstring C2 y referencias restantes con el kernel C4 vinculante antes de documentar comandos oficiales.

**Salida:** candidato reproducible `model.npz`, `manifest.json`, dataset/config con hash y reporte de entrenamiento.  
**Gate G4:** integridad y reconstrucción determinista del candidato; ningún candidato se declara “real” solo por pasar el validador estructural.

### Fase 5 — Validar con vistas reales independientes

**Entradas:** robot final, MID-360, calibraciones, nubes/rosbags etiquetadas y candidato congelado.  
**Acciones:**

- Adquirir vistas estáticas desde varios azimuts, elevaciones/rangos disponibles y oclusiones representativas, siguiendo un protocolo seguro.
- Separar entrenamiento y prueba por sesión/vista física, no mediante división aleatoria de puntos vecinos de la misma nube.
- Etiquetar origen y orientación desde una referencia independiente con incertidumbre conocida; conservar los datos originales inmutables.
- Evaluar registro en vistas parciales y condiciones de mala observabilidad. El yaw debe ser inválido cuando la geometría visible no lo determina.
- Medir error del origen, tasa de aceptación/rechazo, falsos positivos, fracción de soporte, calibración de covarianza, latencia, memoria y sensibilidad a ruido/calibración.
- Comparar modelo CAD contra vistas reales y contra un baseline sin GPIS; registrar fallos y no solo medias agregadas.
- Preinscribir umbrales por perfil de sensor y tolerancia de aplicación antes de evaluar el conjunto de prueba. No inventar ahora números sin datos/requisitos aprobados.

**Salida:** `validation.json`, tablas por condición, ejemplos de rechazo/fallo y hash del candidato exacto.  
**Gate G5:** cumplir todos los límites cuantitativos preacordados, incluidos límites de falsos positivos y latencia; resultados únicamente SIL/sintéticos no satisfacen este gate.

### Fase 6 — Publicación e integración

**Acciones:**

- Promover solo el modelo que pase G0–G5 a `artifacts/models/opponent_gpis/<model_id>/`.
- Incluir `model.npz`, manifiesto, reporte de validación y referencias/hash de fuentes según política de datos.
- Hacer que el cargador compruebe esquema, kernel, hash, unidades, marco, transform y perfil de fidelidad; fallar explícitamente si el artefacto es incompatible.
- Integrar en P4.2 para el ajuste semántico planar; mantener las salidas completas del sensor en seguridad.
- Ejecutar pruebas unitarias P4.1/P4.2, regresión SIL, validación de esquema, prueba de carga del artefacto y replay real; medir el runtime en la plataforma objetivo.
- Registrar modelo, software, calibración y configuración conjuntamente en el manifiesto de release. Los cambios del CAD o montaje invalidan la aceptación anterior hasta reevaluación.

**Salida:** artefacto inmutable y reporte de liberación con perfiles habilitados/bloqueados.  
**Gate G6:** identidad/hash del modelo reproducible y aceptación acotada al hardware, sensor y condiciones efectivamente probadas.

## 5. Criterios técnicos de aceptación

El candidato no se promoverá si falla cualquiera de estos criterios:

1. **Procedencia:** cada fuente tiene origen, revisión, licencia/permiso y hash registrados.
2. **Geometría:** unidades en metros verificadas; ensamblaje coincide con ambos robots y con las piezas visibles a la altura/ángulos observados por el LiDAR.
3. **Marcos:** `M`, `base_link` y sensor están definidos; transform no es un valor por defecto y su incertidumbre está declarada.
4. **Muestreo/Hermite:** normales coherentes; observaciones repetidas, degeneradas o fuera de rango detectadas; ruido y unidades de valor/derivada consistentes.
5. **Kernel/esquema:** `wendland_c4_d3_unit_center_v1`, schema vigente, condiciones numéricas finitas y artefacto no ejecutable con integridad SHA-256.
6. **Soporte:** queries fuera del soporte o con fracción de soporte insuficiente son rechazadas; cero media fuera de soporte nunca se interpreta como una coincidencia superficial.
7. **Observabilidad:** el estimador puede aceptar posición y marcar yaw inválido; no inventa orientación para vistas simétricas o degeneradas.
8. **Validación independiente:** vistas reales de prueba no participaron en ajuste, selección de hiperparámetros ni transformación manual del modelo.
9. **Exactitud y operación:** errores, falsos positivos, soporte, calibración de incertidumbre y latencia cumplen umbrales preacordados para el perfil declarado.
10. **Seguridad y aislamiento:** GPIS no borra ni filtra retornos usados para colisión; modelo fallido/ausente produce rechazo semántico explícito, no un falso éxito ni movimiento autorizado.

## 6. Datos que deben solicitarse o medirse

- CAD/STEP o ensamblaje DAE de la **configuración de competencia completa**, incluidos MID-360, soporte y accesorios; revisión exacta para ambos robots.
- Plano/cotas oficiales y masas/accesorios relevantes para confirmar dimensiones; fotografías y mediciones controladas del hardware real.
- Transform y convención `base_link` ↔ marco del CAD ↔ `lidar_link`, con método, unidades e incertidumbre.
- Nubes de puntos reales con timestamps, frame, estado del sensor, configuración Livox y sesiones/vistas etiquetadas; referencia independiente del origen y yaw cuando sean observables.
- Requisitos cuantitativos del usuario/competencia para error admisible, falsa detección, carga de CPU/memoria y rangos de operación.
- Política de retención y acceso para CAD, bag files e imágenes, más aprobación de procedencia/licencia para artefactos compartidos.

Si el CAD completo no está disponible, el plan alternativo es reconstrucción geométrica del ensamblaje desde mediciones y escaneos multi-vista registrados, con incertidumbre explícita. Un escaneo parcial sin alineamiento y sin referencia de origen no sustituye esa información.

### 6.1 Recomendación: bags de la prueba como primera fuente de la geometría real

Para la configuración que realmente compite, **sí es preferible empezar por los bags de la prueba previa antes que entrenar el modelo final desde el Kobuki solo**. Ya contienen el sensor real y, según lo indicado, observaciones con el oponente inmóvil y girando. Eso los hace potencialmente la mejor evidencia disponible de la superficie que el Livox realmente devuelve en el ensamblaje elevado. No elimina el uso de CAD: las dos fuentes son complementarias.

Decisión propuesta:

- Usar `main_body.dae`/`wheel.dae` y el Xacro como **prior geométrico de la base Kobuki**, para comprobar dimensiones, marco y coherencia de la zona baja.
- Usar bags para recuperar la **envolvente observada del ensamblaje real** (incluidas plataformas, postes, carcasa del sensor y equipo visible) y para validar ruido, sparsity, reflectividad, oclusiones y pipeline de detección.
- Combinar CAD base y reconstrucción de escaneo solo tras registro en un marco común y comparación contra cotas/foto/mediciones; conservar procedencia separada y no ajustar forzadamente para que el scan coincida con una base incompleta.
- Si el objetivo es estrictamente reconocer lo visible al sensor, entrenar/validar con la superficie observada puede ser más útil que un CAD perfecto pero distinto del montaje. Aun así, un mapa parcial debe llevar dominio/cobertura explícitos: no imputar caras ocultas como si fueran vistas.
- Crear en MVSim un modelo visual HSL26 compuesto de base más geometría del bastidor reconstruida, una vez existan datos; mantener un perfil de sensor sintético Livox aproximado, no declarado equivalente al hardware.

#### Separar fondo y puntos del oponente

Primero estabilizar cada nube respecto al **observador** y al frame del sensor usando su tiempo de adquisición por punto, `T_B_L` medido y una historia de poses sincronizada. Después separar el objetivo:

1. Priorizar una grabación del mismo escenario sin oponente, si existe, para sustraer fondo registrado. Si no existe, combinar ROI espacial aproximado, clustering 3D, consistencia temporal y revisión manual en CloudCompare/RViz; preservar pisos y planos de la plataforma del robot, que son geometría del objetivo y no “suelo” descartable.
2. No usar solo “estacionario en el mundo = pared”: un oponente quieto puede parecer estructura. La máscara debe usar la región aproximada del robot y/o una referencia sin objetivo, y conservar etiquetas/confianza de cada punto.
3. Para la secuencia con objetivo inmóvil, acumular varias vistas únicamente si se movió el observador alrededor del blanco o si se capturaron varias poses relativas. Si sensor y objetivo no cambiaron pose, el bag solo aporta densidad temporal, no cobertura de superficies ocultas.
4. Para la secuencia con objetivo girando, el giro puede revelar lados adicionales, pero no basta con concatenar nubes. Estimar `T_W_O(t)` del robot objetivo a lo largo del tiempo y transformar cada punto a un frame del objetivo en una época de referencia, usando el timestamp individual del punto. Este paso “desgira” el blanco además del deskew del observador. Puede usar encoder/odometría del objetivo si está grabada y sincronizada, un estimador independiente o registro multi-scan robusto; no presuponer que el tracker HGW ya entrenado proporciona esa pose.
5. Si no hay trayectoria angular fiable del objetivo, tratar la secuencia giratoria como vistas independientes etiquetadas por orientación desconocida. Usarlas para detectar geometría candidata y para evaluar robustez, pero no fusionarlas en una malla métrica rígida ni asignar normales/signos a una superficie reconstruida como si el giro fuese conocido.
6. Rechazar o segmentar por separado puntos de movimiento rápido no compensable, oclusiones, suelo, puntos multipath/ruido y componentes desconectados. Reportar qué zonas nunca recibieron retorno.

La implementación P2.2 puede corregir movimiento del observador cuando `point_times_s`, `pose_history` y `T_B_L` están validados. La transformación de puntos de un blanco giratorio requiere una segunda trayectoria (`T_W_O(t)`) y una etapa explícita fuera del deskew existente.

Para una muestra `p_L(t_i)` del sensor, la fusión al frame del objetivo en la referencia `t_r` es:

\[
{}^{O(t_r)}p_i =
\left(T_{W O}(t_r)\right)^{-1}
T_{W B}(t_i)\,T_{B L}\,{}^{L}p_i.
\]

Si el objetivo permanece inmóvil, `T_WO(t_r)` es constante. Si gira, debe conocerse/estimarse en cada instante del punto; interpolar una sola pose por mensaje puede dejar distorsión residual dentro del giro. La covarianza debe incluir errores de tiempo, ego-pose, extrínseco y pose del objetivo.

#### Auditoría mínima de los bags antes de reconstruir

Ejecutar en el entorno ROS 2/Humble fijado, sin alterar los originales:

1. `ros2 bag info <bag>`: formato/storage, tópicos, tipos, duración, conteos y tasa.
2. Registrar versiones/hashes del bag, driver y configuración Livox; extraer esquema de campos de nube, frame ID y timestamps de header y punto.
3. Confirmar presencia y sincronía de `/livox/lidar`, `/livox/imu`, `/tf`, `/tf_static`, `/odom` y cualquier estado/comando/telemetría del robot objetivo. Medir monotonicidad, saltos, jitter, pérdidas y el rango temporal cubierto por IMU/odometría.
4. Visualizar nubes crudas y deskewed por separado; comparar una pared/estructura fija y medir residuales antes/después. Nunca ocultar un resultado pobre fusionando nubes.
5. Marcar para cada bag: `USABLE_FOR_STATIC_VIEW`, `USABLE_FOR_EGO_DESKEW`, `USABLE_FOR_TARGET_DESPIN`, `USABLE_FOR_METRIC_RECONSTRUCTION` o `VALIDATION_ONLY`, con motivos concretos.

Si no hay tiempo por punto, TF/pose sincronizada o calibración del sensor, el bag aún puede servir para segmentación/validación por frames o vistas estáticas, pero no para una reconstrucción métrica multi-scan de confianza. En tal caso, obtener CAD del montaje y/o repetir captura con el logging correcto antes de declarar modelo 3D validado.

#### Herramientas externas apropiadas

- **ROS 2 rosbag2** (en el contenedor Humble fijado): inventario, reproducción y exportación de mensajes manteniendo el bag original inmutable.
- **RViz2**: revisar frame, TF y nubes antes/después del deskew.
- **CloudCompare**: inspección manual, selección/clipping del oponente, registro inicial y comparación visual/métrica; mantener pasos y parámetros anotados para que no sea el único pipeline reproducible.
- **Open3D o PCL**: automatizar filtrado/ROI/clustering, registro multi-vista, estimación de normales y reconstrucción en scripts versionados. Elegir la librería después del formato/tamaño de datos; no instalar ambas sin necesidad.
- **MeshLab o Blender**: revisar la malla final y ensamblar/exportar el visual de MVSim, no sustituir validación numérica ni procedencia.

El dataset de entrenamiento y validación se particiona por **sesión/vista completa**, no por puntos aleatorios de una misma nube. La verdad de pose debe proceder de una referencia independiente y con incertidumbre registrada. Los bags originales se conservan inmutables; la geometría derivada de ellos se etiqueta como **reconstrucción observada**, no CAD oficial ni verdad completa de superficies ocultas.

## 7. Riesgos y mitigaciones

| Riesgo | Efecto | Mitigación |
|---|---|---|
| Entrenar solo con la carcasa base | Sesgo del origen/yaw por el sensor o montaje omitido; soporte equivocado en puntos reales | Incluir ensamblaje visible y validar con escaneos por vistas; no promover malla parcial. |
| Confundir visual con colisión URDF | Cuerpo real difiere de cilindros de colisión simplificados | Usar malla/medición superficial completa para GPIS; conservar geometría conservadora e independiente para seguridad. |
| Transform identidad o ejes invertidos | Registro sistemáticamente desplazado o reflejado | Calibración medida, pruebas de transformación, chequeos de determinante/orientación y revisión independiente. |
| Normales o signos inconsistentes | Matriz Hermite incorrecta y campo deformado | Validación geométrica de normales, anclas y casos sintéticos antes de entrenar desde CAD. |
| Puntos densos/duplicados | Matriz Cholesky mal condicionada y coste O(N³) | Muestreo controlado, límites explícitos, análisis de condición y estrategia de reducción documentada. |
| Overfit a CAD ideal | Baja precisión por reflectividad, oclusión, montaje y ruido reales | Validación held-out con nube real, modelo de ruido, covarianza y rechazos bajo baja observabilidad. |
| Confundir métricas SIL con aceptación | Afirmación de fidelidad no respaldada | Etiquetar cada evidencia por perfil; bloquear promoción hasta G5. |
| Cambios no versionados de robot | Artefacto deja de representar el hardware | Identidad/configuración del robot ligada al hash del modelo y política de revalidación. |

## 8. Referencias del análisis

### Repositorio

- [Arquitectura final HSL26](./HSL26_FINAL_ARCHITECTURE.md), §7 y §11.
- [Especificación técnica HSL26](./HSL26_TECHNICAL_SPECIFICATION.md), §8 y flujo de artefactos de GPIS.
- [Plan de implementación P4](./HSL26_PHASE4_IMPLEMENTATION_PLAN.md), restricciones HGW y gates.
- [Fase 4: percepción del oponente](./HSL26_PHASE4_OPPONENT_PERCEPTION.md), P4.1/P4.2 y G3.
- [Changelog HSL26](./HSL26_SESSION_CHANGELOG.md), implementación P4.1/P4.2 y corrección normativa C4.
- Los informes P4.1, P4.2, P4.5 y el baseline de P4 enlazados en §2.3.
- [Descripción Xacro de Kobuki](../kobuki/workspace/src/kobuki_ros/kobuki_description/urdf/kobuki.urdf.xacro), [malla DAE de la carcasa](../kobuki/workspace/src/kobuki_ros/kobuki_description/meshes/main_body.dae) y [malla DAE de la rueda](../kobuki/workspace/src/kobuki_ros/kobuki_description/meshes/wheel.dae).

### Referencia externa de procedencia

- [Mallas Kobuki en `kobuki-base/kobuki_ros`, commit fijado](https://github.com/kobuki-base/kobuki_ros/tree/e1fbf7bdec199411008bd7c94c6e4b7d82903347/kobuki_description/meshes): coincide con las mallas locales según los Git blob SHA-1 registrados en §2.1. El [Xacro fijado](https://github.com/kobuki-base/kobuki_ros/blob/e1fbf7bdec199411008bd7c94c6e4b7d82903347/kobuki_description/urdf/kobuki.urdf.xacro) da las instancias y offsets; respétalos al componer la escena.
- [Livox MID-360: página oficial de descargas](https://www.livoxtech.com/mid-360/downloads) y [especificaciones oficiales](https://www.livoxtech.com/mid-360/specs): la descarga revisada expone Viewer 2 y manual del Viewer, no un CAD/STEP del sensor. No abrir/desmontar el LiDAR para obtener geometría.
- [Malla MID-360 comunitaria OBJ](https://github.com/jfrascon/robotics_description/blob/c731dd26920a604caf985f8f0461267c702a7f55/meshes/sensors/lidars/livox_mid360/livox_mid360.obj), [STL](https://github.com/jfrascon/robotics_description/blob/c731dd26920a604caf985f8f0461267c702a7f55/meshes/sensors/lidars/livox_mid360/livox_mid360.stl) y [macro Xacro fijada](https://github.com/jfrascon/robotics_description/blob/c731dd26920a604caf985f8f0461267c702a7f55/urdf/sensors/lidars/livox_mid360_macro.xacro). Es una aproximación de procedencia comunitaria, no un modelo CAD oficial Livox.
- [Modelos de MVSim](https://github.com/MRPT/mvsim-models) y [catálogo web](https://mrpt.github.io/mvsim-models/): hay un recurso TurtleBot3, pero no se identificó un ensamblaje Kobuki/TurtleBot2 HSL26.
- [Formato de vehículos/visuales MVSim](https://github.com/MRPT/mvsim/blob/develop/docs/vehicles.rst) y [sensores MVSim](https://github.com/MRPT/mvsim/blob/develop/docs/sensors.rst): documentan DAE como `model_uri`, huella de colisión separada y LiDAR 3D configurable. No documentan Mid-360 nativo.
- [Livox ROS Driver 2 oficial](https://github.com/Livox-SDK/livox_ros_driver2): referencia para obtener la nube MID-360 real en ROS 2 y registrar datos con el driver/configuración correspondiente.

## 9. Orden recomendado de ejecución

1. Solicitar y hacer inventario/hash de los bags previos; están fuera del checkout revisado y aún no se han inspeccionado.
2. Determinar formato/tipo de nube, `timestamp` por punto, referencia temporal, `/tf`, `/tf_static`, `/odom` y si se registró estado del robot objetivo. Separar capacidad de ego-deskew y target-despin.
3. Procesar primero el bag del objetivo inmóvil para segmentación y validar el deskew del observador; analizar el bag del objetivo girando como fuente de nuevas vistas solo si se puede estimar su trayectoria angular.
4. Mantener Kobuki DAE/Xacro y sensor MID-360 candidato como referencia parcial; comparar la envolvente baja y el montaje con las nubes. Pedir CAD del bastidor si existe, pero no bloquear la prueba de viabilidad de bag.
5. Si las grabaciones contienen vistas, tiempos y poses utilizables, reconstruir una geometría observada versionada; en paralelo preparar la escena MVSim con geometría compuesta y perfil de sensor explícitamente aproximado.
6. Si falta cobertura, pose, sincronización o visibilidad, pedir CAD/mediciones del bastidor y repetir capturas controladas; no forzar la geometría a partir de datos insuficientes.
7. Construir observaciones Hermite, actualizar el pipeline/manifiesto, y validar el candidato en sesiones reales separadas.
8. Promover e integrar solo después de cumplir G0–G6 con evidencia cuantitativa.

Hasta inspeccionar los bags, el estado correcto es: **HGW matemático implementado y SIL probado; candidato Kobuki-base parcial disponible; datos reales potencialmente más representativos identificados pero aún no auditados; modelo geométrico del ensamblaje y registro real pendientes de validación**.
