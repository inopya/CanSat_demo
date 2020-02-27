# CanSat_demo
Programa base para consurso Esero cansat >>  http://esero.es/cansat/

```diff
-REPOSITORIO EN PROCESO......
```


Este repositorio toma el relevo al "Datalogger_en_EEPROM_del_328p" desarrollado para cohetes de agua,
que se puede encontrar en https://github.com/inopya/Datalogger_en_EEPROM_del_328p


Mantiene el mismo concepto de grabación de datos en EEPROM. A priori en EEPROM interna del microcontrolador (está pensado para MCUs ATMega328p). Aun así, si por el volumen de datos recopilados o un tiempo de vuelo demasiado largo, se estimase necesario se podría facilmente cambiar el destino de grabación a una EEPROM externa. 

En este repositorio se encuentran utilidades para simplificar el manejo de dichas EEPROM externas (hasta 64K).

https://github.com/inopya/I2C_EEPROM
  
Otros repositorios relacionados y utiles para este proyecto son:

https://github.com/inopya/APC220_GUI_config   (configuracion de APC220 desde entorno gráafico)

https://github.com/inopya/inopya_Universal_GPS  (libreria de uso muy sencillo para usod e GPS NEO6/7/8

https://github.com/inopya/Temporizador_Virtual  (libreria para simplicifar el uso de tareas temporizadas)


  
Este es un programa base para datalogger y envío en tiempo real de dichos datos a una estación de tierra mientras el cohete está en vuelo.

Recopila datos de presión y temperatura mediante un barómetro, realiza mediciones de ultravioleta con tres sensores, y envía datos a una estación de tierra mediante un Transceptor APC220.
Así mismo utiliza un zumbador y la emmisión de la posición GPS para ser localizado una vez en tierra.

Algunas partes aun están en fase beta.

Este montaje usa varios sensores ultravioleta en bus I2C con el inconveniente de que son identicos y por tanto con la misma dirección. Para poder controlarlos utilizamos un pequeño "truco":
alimentarlos de un pin digital e ir alternando cual está activo al momento de leer informacion desde el bus I2C:

**vease el esquema:**

![](./conexionUV.png)

**Toma de muestras y visualizacion de UVmax desde 2x VELM6070 sobre el mismo bus I2C**

![](./uv_x2_sample.jpg)
