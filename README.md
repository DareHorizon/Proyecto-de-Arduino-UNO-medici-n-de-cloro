############################  Proyecto de Arduino UNO medición de cloro  ###########################

* DISCLAIMER IMPORTANTE*

El proposito de este repositorio es dejar un log de todos los cambios hechos en la logica del codigo
que va a estar funcionando en el Arduino UNO R3, se espera guardar versiones, features extras o ideas
eliminadas.

El proyecto esta listo para ser usado en Wokwi. "sketchv3.ino" es el codigo principal, "diagram.json"
es la estructura fisica del emulador y "libraries.txt" es para los modulos de simulación. Es llegar
copiar y pegar. De momento, todo el ambiente esta siendo probado de manera digital, es decir, en
entornos emulados. Aun no se prueba en una placa real, sin embargo, los tests han probado ser solidos
en las 3 versiones distintas de "sketch.ino".

###########################################  Uso del menú  ##########################################

El menú se navega desde un joystick, este menú aparece en una pantalla LCD y es interactuable.

Joystick arriba = Subir
Joystick abajo = Bajar
Joystick izquierda = Atras
Joystick derecha = Entrar
Joystick click = Seleccionar/Confirmar

Si la pantalla llega a estar apagada con el Arduino encendido, un click del joystick la enciende.

Si no hay actividad, el menu se defaultea a "Lectura en vivo", siempre y cuando no hayan alertas.
