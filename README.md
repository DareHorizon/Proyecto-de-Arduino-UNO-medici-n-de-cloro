* Proyecto de Arduino UNO medición de cloro en construccion

//////////////////// DISCLAIMER IMPORTANTE \\\\\\\\\\\\\\\\\\\\\\

El proposito de este repositorio es dejar un log de todos los cambios hechos en la logica del codigo
que va a estar funcionando en el Arduino UNO R3, se espera guardar versiones, features extras o ideas
eliminadas.

El proyecto esta listo para ser usado en Wokwi. "sketchv3.ino" es el codigo principal, "diagram.json"
es la estructura fisica del emulador y "libraries.txt" es para los modulos de simulación. Es llegar
copiar y pegar. De momento, todo el ambiente esta siendo probado de manera digital, es decir, en
entornos emulados. Aun no se prueba en una placa real, sin embargo, los tests han probado ser solidos
en las 3 versiones distintas de "sketch.ino".

######################################## Alertas por color #########################################

El módulo RGB tiene dos modalidades, "Normal y Daltonismo", y la lógica de esta es la siguiente.

/// NORMAL:

ROJO = rango de cloro por debajo de 3.0ppm o por encima de 5.0ppm, es decir, TODA ALERTA ES ROJA.

AZUL = cloro dentro del rango normal, es decir, dentro de los 3.0ppm y 5.0ppm.

VERDE = confirmación activa de alerta por usuario interactuando con el joystick, es decir, alguien
tiene que fisicamente confirmar que vió la alerta y apretar un boton para que se active esta luz.

/// DALTONISMO (Deuteranopia y Protanopia):

AMARILLO = rango de cloro por debajo de 3.0ppm o por encima de 5.0ppm, es decir, TODA ALERTA ES
AMARILLA Y PARPADEA.

AZUL = cloro dentro del rango normal, es decir, dentro de los 3.0ppm y 5.0ppm.

BLANCO = confirmación activa de alerta por usuario interactuando con el joystick, es decir, alguien
tiene que fisicamente confirmar que vió la alerta y apretar un boton para que se active esta luz.

########################################### Uso del menú ###########################################

El menú se navega desde un joystick, este menú aparece en una pantalla LCD y es interactuable.

/// Joystick arriba = Subir

/// Joystick abajo = Bajar

///Joystick izquierda = Atras/Salir

/// Joystick derecha = Entrar

/// Joystick click = Seleccionar/Confirmar

Si la pantalla llega a estar apagada con el Arduino encendido, un click del joystick la enciende.
Si no hay actividad, el menu se defaultea a "Lectura en vivo", siempre y cuando no hayan alertas.

(el comportamiento del joystick arriba/abajo puede ser invertido segun donde se aplique el codigo)

############################################  Modalidades  ##########################################

Existen dos modos dentro del arduino, modo producción y modo prueba. 

/// El modo producción es la logica de alertas real del programa, donde la medición se hace cada 1 minuto.

/// El modo prueba usa la misma lógica de alertas, pero la medición es cada 2 segundos.

Para el modo de producción, toda alerta por debajo de 3.0ppm y por encima de 5.0ppm lanza una alerta,
sin embargo, para los valores que rondeen estos umbrales, al menos dos mediciones del mismo tipo tienen
que aparecer para que el buzzer suenen y se tome como alerta real.

Para el modo prueba la logica de alertas es 70% normal, 15% bajo 3.0ppm y 15% por encima de 5.0ppm.

\\\\\\\\\\\\\\ POR RELLENAR \\\\\\\\\\\\\\
\\\\\\\\\\\\\\ POR RELLENAR \\\\\\\\\\\\\\
\\\\\\\\\\\\\\ POR RELLENAR \\\\\\\\\\\\\\

