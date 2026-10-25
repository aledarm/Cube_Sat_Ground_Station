# Integración futura - contratos y decisiones pendientes

## Estado actual

La interfaz funciona con `LocalBackend`. No hay conexión ESP32 ni paquetes reales. `UdpTransport` permite transportar bytes, pero **no está conectado a los botones**. `ProtocolAdapter` lanza `ProtocolNotConfigured` en lugar de inventar un mensaje o fingir una confirmación.

Los nombres de `Telemetry` y `Command` son un modelo interno de la aplicación. No fijan el formato del paquete de Ricardo.

## Con Ricardo: protocolo

Acordar tipos de mensaje, versión, identificadores, campos obligatorios/opcionales, tamaños, orden de bytes, unidades, frecuencias y puertos. Definir exactamente el CRC, los datos cubiertos y cómo rechazar tramas inválidas. Distinguir recibido, aceptado y aplicado; definir rechazo, caducidad, duplicados, reintentos limitados e idempotencia.

Un `sendto()` exitoso **no** demuestra que el receptor haya aplicado el comando. Un campo ausente debe mostrarse como desconocido, no como cero. El CRC no autentica por sí mismo a quien envía comandos: debe acordarse además cómo limitar quién puede controlar el cubo.

Para diagnosticar edad de muestra, pérdida y latencia, documentar secuencias, marcas de tiempo y dominio de reloj. Sin sincronización no debe inventarse una latencia unidireccional.

## Con Edson: datos y estado seguro

Confirmar qué publica realmente cada controlador: rpm, ángulo relativo o absoluto, par medido/estimado/mandado, corriente, temperatura y fallos. Especificar los identificadores de las tres ruedas y su correspondencia con X/Y/Z.

Confirmar la IMU, el magnetómetro, la estimación de actitud y sus unidades, referencias y validez. Acordar imagen, resolución, formato, ritmo, fragmentación, timestamp y metadatos del punto. No asumir que la imagen completa cabe en un datagrama.

Definir qué significa parada, rearme y pérdida de enlace. La protección local y las limitaciones de par, velocidad, temperatura y batería no pueden depender de que la GUI esté viva.

## Con Ana y Maurício: lazos

Confirmar la cascada real, las variables de cada lazo, ecuaciones, periodo, unidades, signo, límites, anti-windup y política de actualización de ganancias. No transferir las ganancias del ejemplo a hardware sin identificación y ensayos.

Confirmar si las consignas son absolutas respecto a qué referencia o incrementales; si se usan Euler o cuaterniones; y qué eje mantiene la referencia de Roll al seguir un punto.

## Pasos de integración

1. Formalizar el ICD y crear paquetes de prueba con resultados esperados.
2. Implementar y probar `ProtocolAdapter`, incluidas unidades, errores y casos incompletos.
3. Añadir un backend de hardware que implemente `Backend` y mantenga estados de comando pendientes/confirmados.
4. Sustituir en la coordinación las lecturas de estado específicas del simulador por las recibidas del backend real. La pestaña Simulator seguirá usando su propio motor solo cuando esté seleccionada la fuente SIM.
5. Leer telemetría de hardware sin permitir actúacion al principio.
6. Habilitar modos y comandos gradualmente, con límites acordados y protecciones locales.
7. Validar degradación de red, duplicados, mensajes atrasados, reinicios y estado seguro.

La separación reduce cambios en la GUI, pero **la integración no consiste solo en pegar una cadena de texto**: requiere implementar y verificar el contrato real.

## ESP32 + planta simulada

Para ejecutar firmware real en un ESP32 conectado a una planta de PC, se necesita una interfaz de ensayo adicional: sensores simulados hacia el firmware, comandos de actuador hacia la planta, unidades y sincronización. Esa prueba con procesador en el lazo no está implementada en v0.1. No debe confundirse con generar datos aleatorios para comprobar gráficas.
