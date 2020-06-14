
/*

  #       _\|/_   A ver..., ¿que tenemos por aqui?
  #       (O-O)        
  # ---oOO-(_)-OOo-----------------------------------------
   
   
  ##########################################################
  # ****************************************************** #
  # *            DOMOTICA PARA PRINCIPIANTES             * #
  # * USO DE VARIOS SENSORES I2C CON IDENTICA DIRECCION  * #
  # *          Autor:  Eulogio López Cayuela             * #
  # *                  https://github.com/inopya         * #
  # *                                                    * #
  # *       Versión v1.0      Fecha: 27/02/2020          * #
  # ****************************************************** #
  ##########################################################
  
*/

/*
  >> DESCRIPCION DEL PROYECTO
  
  Uso de varios sensores identicos sobre bus I2C valiendonos de alimentarlos solo en el momento de leerlos
  de forma que la librearia "piense" que siempre esta leyendo un unico sensor.
  La ventaja de este metodo es poder usar varios sensores identicos sobre el mismo bus,
  por contra tendremos el hecho de que hemos de inicializar el sensor antes de cada ciclo de lectura
  debiso a que pierde su conficuracion al cortarsele la alimentacion despues de su uso.
  En el caso de estos modulos VEML6070, dicha operacion conlleva un retraso de 380µs 
  por cada sensor que necesitemos leer.

  En este caso práctico usamos sensores de radiacion ultravioleta VEML6070 que debido a su bajisimo consumo 
  (un máximo de 250µA) podrian ser alimentados directametne desde los pines que usemos para su control,
  pero se opta como caso general por el uso de un transistor en cada salida que se usa para el control de la alimentación


  >> TIEMPOS DE INTEGRACION DEL SENSOR [para Rset = 300 kΩ,(este modulo)]  [para Rset = 600 kΩ]

  VEML6070_HALF_T, (0 : 0) = ½T          62.5 ms                              125 ms
  VEML6070_1_T,    (0 : 1) = 1T           125 ms                              250 ms
  VEML6070_2_T,    (1 : 0) = 2T           250 ms                              500 ms
  VEML6070_4_T,    (1 : 1) = 4T           500 ms                             1000 ms

 
 >>  CONEXIONES del modulo VEML6070 con ARDUINO
  
      VEML6070   ======>  ARDUINO
        Vcc        ==>     Pin digital y transistor
        GND        ==>     GND
        SCL        ==>     SLC / A5 (Arduino UNO, NANO)
        SDA        ==>     SDA / A4 (Arduino UNO, NANO)
    
/*
  


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION PARA IMPORTACION DE LIBRERIAS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#include <Wire.h>                   // biblioteca para comunicaciones I2C
#include <Adafruit_VEML6070.h>      // control del sensor de radiacion ultravioleta
#include <Temporizador_inopya.h>    // senscillo control de tiempos



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DEFINICION DE PINES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define PIN_TRANSISTOR_1      2
#define PIN_TRANSISTOR_2      3

#define UV_1                  2        
#define UV_2                  3  

#define TIEMPO_MUESTREO       VEML6070_HALF_T   //½T 62.5 ms



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


#define INTERVALO_MUESTRAS    2000    // tiempo en milisegundos entre cada muestra
                                      // Recordad que disponemos de memoria para 204 muestras,
                                      // dependiendo del intervalo, tendremos mayor o menor tiempo de grabacion 

#define PNP_ON                 LOW    // un transistor PNP se activa con voltaje bajo
#define PNP_OFF               HIGH    // un transistor PNP se desactiva con voltaje alto


boolean FLAG_uso_eeprom = false;                    // Preserva la eeprom de usos innecesarios
int puntero_eeprom = 0;   
               
float indiceUv;



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE CREACION DE OBJETOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


/* creación del objeto sensor uv */
Adafruit_VEML6070 sensor_UV6070 = Adafruit_VEML6070();

uint8_t uvSensorList[2] = {UV_1, UV_2};                                 // pines en que se conectan los sensores
uint8_t num_sensores = sizeof(uvSensorList)/sizeof(uvSensorList[0]);    // 'autocalculo' del numero de sensores

/* creación de  objetos Temporizador_inopya */
Temporizador_inopya relojMuestras;



//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//                             FUNCION DE CONFIGURACION
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 


void setup() 
{
  Serial.begin(115200);  // [iniciar el puerto serie para DEBUG]

  /* Asignacion del 'modo salida' a los pines de alimentacion de los sensores */
  for( uint8_t i=0; i<num_sensores; i++ ){
    pinMode(uvSensorList[i], OUTPUT); 
  }

  digitalWrite(uvSensorList[0], PNP_ON);  // activar el primeo de os sensores para que la libreria... 
  sensor_UV6070.begin(TIEMPO_MUESTREO);   // ...lo encuentre y se inicialice correctamente el objeto sensor
  
}




//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//                              BUCLE PRINCIPAL DEL PROGRAMA  
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 


void loop() 
{
  if( relojMuestras.estado()==false ){
    relojMuestras.begin(INTERVALO_MUESTRAS);    // reiniciar el temporizador virtual
    /* Actualizar el indice Uv */
    indiceUv = obtener_UV_max();                // obtener el indice UV máximo de entre todos los sensores
    Serial.print(F("UV max. = ")); Serial.println(indiceUv); Serial.println();
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   OBTENER INDICE UV
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

float obtener_UV_max()
{
  //uint32_t t0=micros();                             // [control de tiempo para DEBUB]
  
  float indice_UV_max = 0.0;                          // reseteamos el indice uv maximo

  /* cortamos la alimentacion a todos los sensores */
  for( uint8_t j=0; j<num_sensores; j++ ){
    digitalWrite(uvSensorList[j], PNP_OFF);           
  }
  delayMicroseconds(20);                              // pausa para estabilizar la alimentacion
  
  for( uint8_t n=0; n<num_sensores; n++ ){

    digitalWrite(uvSensorList[n], PNP_ON);            // alimentamos los sensores secuencialmente
    delayMicroseconds(20);                            // pausa para estabilizar la alimentacion
    sensor_UV6070.begin(TIEMPO_MUESTREO);             // reiniciamos el sensor (si no no funciona correctamente)
    //sensor_UV6070.clearAck();                       // [se realiza automaticamente al inicializar el sensor]
    int lecturaUV_RAW = sensor_UV6070.readUV();       // obtener el valor de luz ultravioleta
    float indice_UV = lecturaUV_RAW/280.0;            // procesamos para obtener indice de radiacion UV
    digitalWrite(uvSensorList[n], PNP_OFF);           // apagar el sensor tras ser utilizado
    //TWCR=0;                                         // [limpiar el buffer del bus i2c (TwoWireClear)  (no es necesario)]  
    if( indice_UV > indice_UV_max ){
      indice_UV_max = indice_UV;
    }

    /* ¡¡¡ OJO, eliminar todos los print del programa final !!!!  */
    Serial.print(F("Sensor UV(")); Serial.print(n);
    Serial.print(F(") RAW:")); Serial.print(lecturaUV_RAW);
    Serial.print(F(" , UV index:")); Serial.println(indice_UV,2);
  }
  
  //uint32_t t1=micros();                             // [control de tiempo para DEBUB]
  //Serial.println(t1-t0);                            // [control de tiempo para DEBUB]
  
  return indice_UV_max;
}


//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
