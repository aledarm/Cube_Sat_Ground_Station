# Guía para entender y defender la Ground Station

## 1. Qué es esta aplicación

La Ground Station es la interfaz de supervisión. Recibe estado y permite solicitar referencias, modos de operación y ganancias. No calcula en la GUI la corrección continua que mantiene el cubo estable.

En operación real, el control es onboard en el ESP32. Como todavía no tenemos protocolo ni hardware conectado, lo sustituimos temporalmente por un **módulo de simulación** separado. Que ese módulo corra en el mismo computador no significa que la GUI tenga que controlar el satélite por Wi-Fi.

Una forma de explicarlo en la defensa:

> La estación de tierra supervisa y configura. El controlador embarcado mantiene la actitud de forma autónoma. Para probar la interfaz antes de integrar el equipo, un simulador local sustituye al cubo y responde a los mismos tipos de comandos internos.

## 2. Las carpetas son código real, no un comando

```text
Cube_Sat_Ground_Station/
  main.py                    Abre la aplicación
  gui/                       Ventanas y widgets
    main_window.py           Barra global y coordinación de eventos
    telemetry_tab.py         Datos, gráficas, cubo y cámara
    control_tab.py           Consignas y ganancias
    simulator_tab.py         Escenarios y perturbaciones
    widgets.py               Componentes visuales reutilizables
    style.py                 Estilo compartido
  core/                      Significado de datos y comandos
    models.py                Telemetry, Command, CommandResult
    recording.py             Historial y CSV
  communication/             Intercambio de mensajes
    interface.py             Contrato interno para un backend
    local_backend.py         Conexión local al simulador
    protocol_adapter.py      Lugar reservado para el ICD real
    udp_transport.py         Transporte de bytes, no conectado a la GUI
  simulation/                Cubo virtual y control de referencia
    plant.py                 Dinámica de cuerpo/ruedas y PID
    camera.py                Cámara sintética y centroide
    engine.py                Control virtual onboard
    scenarios.py             Ensayos repetibles
  tests/                     Comprobaciones automáticas
  docs/                      Guías
```

La barra `/` indica una carpeta. `gui/telemetry_tab.py` significa: archivo `telemetry_tab.py` dentro de `gui`. Los archivos `__init__.py` permiten organizar estas carpetas como paquetes importables de Python.

**Por qué lo dividimos:** cambiar un formato de paquete no debería obligarnos a redibujar botones, y cambiar colores no debería modificar un controlador. No es necesario escribir todo en `main.py`.

## 3. Recorrido de un comando

1. La usuaria escribe Roll/Pitch/Yaw y pulsa Send attitude setpoint.
2. La pestaña crea un objeto `Command`: tipo, datos e identificador.
3. `main_window.py` lo entrega a `LocalBackend`.
4. `SimulationEngine` comprueba valores, modo y enclavamiento.
5. Si lo acepta, actualiza la referencia y devuelve un `CommandResult`.
6. El controlador virtual calcula el par en `simulation/`, nunca en un botón.
7. La telemetría vuelve a la GUI y muestra el efecto observado.

Esto separa **solicitado**, **aceptado** y **aplicado**. En este backend local aceptación y aplicación son inmediatas; un enlace real requerirá respuestas del ESP32 y estados pendientes.

## 4. Orientación no es posición orbital

Roll, Pitch y Yaw describen actitud. No estamos mostrando una posición X/Y/Z en metros, ni latitud/longitud, porque no existe esa medición en este prototipo. La vista del cubo gira según la actitud recibida; no es una simulación orbital.

La convención de demostración es `Rz(yaw) Ry(pitch) Rx(roll)`. Debe alinearse con los ejes, signos, unidades y referencias del firmware antes de la integración.

## 5. Qué significa cada consigna

- **Target [deg]**: orientación que queremos mantener.
- **Max rate [deg/s]**: límite de la referencia de velocidad angular del cuerpo durante la maniobra; no es una garantía de límite físico bajo cualquier perturbación.
- **Wheel speed [rpm]**: velocidad relativa de una rueda respecto al cuerpo, en un modo de banco separado.
- **Wheel angle [deg]**: ángulo relativo acumulado módulo 360, incluido como diagnóstico. No indica hacia dónde apunta el cubo.
- **Motor torque [Nm]**: par sobre el rotor. Su reacción sobre el cuerpo tiene sentido opuesto en el modelo.

## 6. Qué se ha supuesto en la simulación

Se usan tres ejes independientes, uno por rueda, porque Alessia confirmó tres ruedas ortogonales. Por cada eje:

```text
J_body * body_acceleration = -motor_torque - damping * body_rate
wheel_relative_acceleration = motor_torque / J_wheel - body_acceleration
```

Las inercias, el rozamiento y los límites de par y rpm son **valores ficticios de demostración**. En ausencia de rozamiento, el modelo conserva por eje el momento angular combinado del cuerpo y el rotor; hay una prueba automática de esa propiedad.

No incluye acoplamiento giroscópico entre ejes, gravedad y fricción detalladas del gimbal, modelo eléctrico del motor ni una estimación inercial validada. La actualización de Euler es una aproximación por ejes independientes, apropiada para una demostración cerca de equilibrio, no para certificar maniobras 3D grandes.

La estimación que utiliza el controlador virtual es idealizada. El ruido configurable se añade a la telemetría: no convierte esta entrega en una prueba real de fusión de sensores.

## 7. Ganancias y lazos

En esta demostración hay una cascada **actitud -> velocidad angular -> par del cuerpo**. El par del rotor tiene el signo opuesto. También hay un PID independiente para el modo de prueba de velocidad de ruedas.

**No es la implementación final de la cascada del equipo**, cuyo documento plantea actitud -> velocidad angular -> velocidad de rueda. Se ha elegido una ley de referencia pequeña para probar los controles de la GUI. Las pestañas identifican expresamente los lazos como SIM.

- Kp: responde al error actual.
- Ki: acumula error; debe limitarse para no crecer al saturar.
- Kd: aporta una respuesta a la variación de la medición; aquí se filtra.

Cada eje tiene sus ganancias. Las unidades dependen del lazo y aparecen en pantalla. No se deben copiar estos valores al firmware. Se exige desarmar antes de aplicarlos para evitar cambios accidentales en marcha en esta primera versión.

## 8. Punto visual y Roll

La imagen sintética contiene un punto blanco. El detector calcula su centro a partir de píxeles brillantes; el controlador no recibe directamente la posición verdadera del objetivo. Utiliza ese centro y una estimación de actitud idealizada.

Un único punto aporta dos restricciones de dirección, no una referencia completa de rotación alrededor de la línea de visión. Por eso la cámara ajusta Pitch/Yaw y Roll se mantiene en una referencia inercial de cero. Las consignas manuales sí permiten controlar los tres ejes.

Para inferir Roll solo con la imagen harían falta referencias visuales adicionales o estructura identificable. Esta entrega no pretende que un punto por sí solo mida los tres ángulos.

## 9. Por qué Emergency Stop está fuera de las pestañas

Es la misma acción global, siempre visible. No hay tres botones con estados distintos. Al pulsar:

1. Se enclava inmediatamente el estado en el backend local.
2. Se desarma y se pide par nulo.
3. Se bloquean los comandos que podrían reactivar el movimiento.
4. Clear E-STOP libera solo el enclavamiento.
5. Arm SIM es una acción adicional explícita.

Un motor sin par no elimina instantáneamente el momento angular: puede seguir girando. En el hardware debe acordarse y verificarse el estado seguro. La GUI y el Wi-Fi no sustituyen una protección física.

## 10. Ensayos y alcance de un resultado

El enunciado original plantea adquisición desde 30 grados en menos de 6 s, mantenimiento dentro de +/-2 grados durante un minuto, recuperación de una perturbación de unos 20 grados en menos de 4 s, seguimiento y ocultación de cámara durante 2 s.

Para adquisición y recuperación, este prototipo define explícitamente como criterio adicional una banda de +/-2 grados y una permanencia de 0.25 s. Es una decisión del demostrador, no un requisito textual nuevo del profesor. Los ensayos automáticos cuantitativos usan por defecto una maniobra de Yaw para corresponder al enunciado original; la GUI permite maniobras manuales de tres ejes.

Seguimiento y ocultación terminan con **REVIEW** porque no hay una rúbrica numérica completa para declarar PASS. Todos los resultados usan el reloj y la verdad del modelo, no certifican tiempos de ejecución del ESP32 ni prestaciones mecánicas.

## 11. Origen de los requisitos y referencias

**Material de la conversación:**
- `AE_ProjetoV2.pdf`, p. 2: Wi-Fi/UDP, posible BLE y autonomía onboard.
- `AE_ProjetoV2.pdf`, p. 3-4: visualización, comandos de motor, consignas, cámara y reparto de tareas.
- `4_-_Cube_Reaction_Wheels_STUDENTS_EN.pdf`, p. 10: cinco pruebas finales.
- Confirmaciones de Alessia: tres ruedas ortogonales, tres pestañas, interfaz en inglés y parada siempre disponible.

**Decisiones de implementación de esta entrega:** estructura de archivos, controles de interfaz, modelo y ganancias SIM, reglas de enclavamiento, formato interno de objetos y criterios complementarios de captura. No son el ICD oficial.

**Documentación primaria para consultar APIs:**
- Qt QShortcut: https://doc.qt.io/archives/qt-5.15/qshortcut.html
- PyQt5: https://pypi.org/project/PyQt5/
- PyQtGraph: https://pyqtgraph.readthedocs.io/en/pyqtgraph-0.13.7/

