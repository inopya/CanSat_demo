


/*
#       _\|/_   A ver..., ¿que tenemos por aqui?
#       (O-O)        
# ---oOO-(_)-OOo---------------------------------
 
 
##########################################################
# ****************************************************** #
# *         AERONAUTICA PARA PRINCIPIANTES             * #
# *            Programa base para CanSat               * #
# *                  Autor:  inopya                    * #
# *         (Aunque parece obra de PePe por            * #
# *   la cantidad de recortes a que ha sido sometido)  * #
# *                                                    * #
# *       Versión 0.5       Fecha: 14/07/2020          * #
# ****************************************************** #
##########################################################
*/

#define __VERSION__ "CanSat_Delta 0.5"


/*
      ===== NOTAS DE LA VERSION ===== 
  
  Programa para el Cansat del equipo SpaceSix.
  
  ***   Version minimalista   ***
      
  Esto es una Simplificacion Extrema de CanSat_delta.

  Se corresponde con la version v8 utilizada por los chavales del equipo SpaceSix 
  y es una version minimalista de la optimizacion que en su momento se hizo de dicha version. 
  
  Se han eliminado funcionalidades de un cierto 'caracter estetico':
  El zumbador, la deteccion de descenso, la deteccion de aterrizaje...
  todo ellos para dar aun mas robustez a la la version delta. 
  Sigue realizando todas las funciones vitales, 
  por tanto es apta para su utilizacion en el concurso ESERO/CANSAT, 


   usos de memoria para la v8 original:
   FLASH, 18802 bytes (58%).
   RAM utilizada, 1352 bytes (66%).

   usos de memoria para la v8 OPTIMIZADA (delta):
   FLASH, 18444 bytes (57%).
   RAM utilizada, 902 bytes (44%).
   
   usos de memoria para la v8 (delta_minimalista): 
   FLASH, 17092 bytes (52%).
   RAM utilizada, 875 bytes (42%).
   
*/




// mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        Valores a cambiar o tener en cuenta para un lanzamiento oficial
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define WAIT_SERIAL_AT_STARTUP      25000     // 25 segundos de espera al reinciar para poder recuperar datos 

#define ALTURA_PARA_EMPEZAR_A_MEDIR   200     // A que altura se ha de empezar a tomar muestras al ascender

#define EN_DEPURACION               false     // Si (1 o true),en depuracion, tendremos mensajes por puerto serie
                                              // y (0 o false) es modo normal de operacion para el concurso
                                              
// mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/



#include <Wire.h>                  // Utilidades para comunicaciones I2C
#include <i2c_BMP280.h>            // Biblioteca de funciones para el barometro
#include <EEPROM.h>                // Control y uso de eeprom interna del ATmega328p
#include <Adafruit_VEML6070.h>     // Driver para sensor ultravioleta
#include <SoftwareSerial.h>        // Utilidades apra disponer de puertos serial extra 
#include <Temporizador_inopya.h>   // Biblioteca para gestion de temporizadores de manera sencilla
#include <Universal_GPS_inopya.h>  // Driver "Universal" y simplificado para acceso a GPS Neo6/7/8

#define ANTENA_RX           11     // pines de comunicacion con la antena RF
#define ANTENA_TX           10     // pines de comunicacion con la antena RF
#define GPS_RX               3     // comunicacion con el modulo GPS
#define GPS_TX               4     // comunicacion con el modulo GPS


#define TIEMPO_MUESTREO       VEML6070_HALF_T   // ½T 62.5 ms  tiempo de muestreo para los sensores UV
#define TCAADDR               0x70              // direccion base del multiplexor I2C
#define PUERTO_UV_1           2                 // direccion del sensor UV1
#define PUERTO_UV_2           3                 // direccion del sensor UV2
#define PUERTO_UV_3           4                 // direccion del sensor UV3


#define INTERVALO_MUESTRAS    1000    // tiempo en milisegundos entre cada muestra realizada por el satelite
                                      // Recordad que disponemos de memoria para 204 muestras,
                                      // dependiendo del intervalo, tendremos mayor o menor tiempo de grabacion 



/* creacion de los objetos "Sensor Ultravioleta" */
Adafruit_VEML6070 uv_1 = Adafruit_VEML6070();
Adafruit_VEML6070 uv_2 = Adafruit_VEML6070();
Adafruit_VEML6070 uv_3 = Adafruit_VEML6070();


/* creacion del objeto barometro/altimetro */
BMP280 bmp280;

/* creación de un puerto serie para Emisor de radiofrecuencia */
SoftwareSerial radioLink( ANTENA_TX , ANTENA_RX); 

/* creación de  objetos Temporizador_inopya */
Temporizador_inopya relojMuestras;
Temporizador_inopya relojEscucha;

/* creación de un puerto serie para el GPS */
SoftwareSerial gpsPort(GPS_TX, GPS_RX);  //4 ,3

/* creación del objeto GPS */
Universal_GPS_inopya gps(&gpsPort);

/* creacion de una structura de datos para contener las muestras */
struct CanSatDatos {  
                     int altura; 
                     int temperatura; 
                     uint8_t uv;
                   };

boolean FLAG_uso_eeprom = false;    // Bandera apra preservar la eeprom de usos innecesarios
int puntero_eeprom = 0;             // puntero de escritura de la eeprom

/*
  variables para los datos del experimento. Con todo su rango tal como se envian. 
  Las que se almacenan en memoria se truncan para reducir sus decimales
*/
float altura;                       // contiene la altura tal como se recoge del barometro/altimetro
float temperatura;                  // almacena la temepratura tal como se recoge 
float indiceUv;                     // para contener el valor de uv con todos sus decimales


float altura_suelo;                 // nos marcará una referencia "cota cero" al momento de iniciar el sistema

int indice_muestra = 0;             // indice de muestra que se enviará por radio junto con los datos recabados

/* Funcion de seleccion del sensor UV que va a estar activo */
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  ANTES DEL LANZAMIENTO
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void setup() 
{
  /* inicializamos el puerto serie para el PC  (pruebas) */
  Serial.begin(9600);
  
  /* inicializamos el puerto serie para el emisor RF */
  radioLink.begin(9600);

   /* inicializamos el GPS */
  gps.begin(9600);  
  
  /* inicializamos el barometro */    
  altura_suelo = inicializarBarometro();

  /* inicializamos los sensores Ultravioletas */
  tcaselect(PUERTO_UV_1);
  uv_1.begin(TIEMPO_MUESTREO);
  tcaselect(PUERTO_UV_2);
  uv_2.begin(TIEMPO_MUESTREO);
  tcaselect(PUERTO_UV_3);
  uv_3.begin(TIEMPO_MUESTREO);

  // Atiende las posibles ordenes entrantes por radio o puerto serie durante 25 segundos
  enviar_mensaje(F("Introduzca D si quiere ver los datos del anterior lanzamiento"));
  atenderPeticionesEntrantes(WAIT_SERIAL_AT_STARTUP);

  enviar_mensaje(F("Modo ESPERA y ENVIO continuo de datos"));

  /* Esta parte espera a que el cohete despegue y alcance una altura de más de ALTURA_PARA_EMPEZAR_A_MEDIR (metros) */
  while(true) {
    /* Tomar y enviar por radio una muestra cada 1 segundo */
    if (relojMuestras.estado() == false){
	    indice_muestra++;
      relojMuestras.begin(INTERVALO_MUESTRAS);
      medirAlturaYTemperatura();
      obtener_UV_max();
      envio_datos();
    }
	  /* cada 25 segundos, mientras permanece en espera y/o no se alcance la cota de lanzamiento */
    if(indice_muestra%25==0){ gps.get(); }
	
    /* si se alcance la cota de lanzamiento iniciar modo normal */
    if((altura - altura_suelo) > ALTURA_PARA_EMPEZAR_A_MEDIR){ break; }
  }
  
  enviar_mensaje(F("Lanzamiento detectado"));
  indice_muestra = 0;
  FLAG_uso_eeprom = true;
}


//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  DURANTE EL LANZAMIENTO 
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void loop() 
{ 
  if(relojMuestras.estado()== false){
    indice_muestra++;
    relojMuestras.begin(INTERVALO_MUESTRAS); //Reinicio del reloj de toma de muestras

    medirAlturaYTemperatura();
    obtener_UV_max();
	  if(indice_muestra%5==0){ gps.get(); }  //actualizar valores desde el gps
    
    envio_datos();                         //incluye datos de gps

    /* guardar datos en memoria si aun hay espacio */
    if(FLAG_uso_eeprom==true){ guardarDatosMemoria(); }

    // no necesitamos control de altura maxima, ni de aterrizaje, ni nada por el estilo, 
    // Grabamos en eeprom desde el pistoletazo de salida (ALTURA_PARA_EMPEZAR_A_MEDIR)
    // Enviamos datos desde el inicio del cansat una vez por segundo y gps  cada 5 segundos
  }                                                                                                                                                                                                                    
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
   ###################################################################################################### 
        BLOQUE DE FUNCIONES: LECTURAS DE SENSORES, COMUNICACIONES... 
   ###################################################################################################### 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   BAROMETRO 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

float inicializarBarometro(){
  
  if (bmp280.initialize()){
    if(EN_DEPURACION){ Serial.println(F("Barometro ok"));}
  }
  else{
    if(EN_DEPURACION){ Serial.println(F("Fallo de Barometro, programa detenido"));}
    
    while (true) {  //si no hay barometro el programa queda en bucle infinito
      radioLink.println(F("Fallo de Barometro, programa detenido"));
      delay(10000);
    } 
  }

  /* Configuracion de calibracion */
  bmp280.setPressureOversampleRatio(2);    
  bmp280.setTemperatureOversampleRatio(1);
  bmp280.setFilterRatio(4);                
  bmp280.setStandby(0);                     
  
  /* Medidas bajo demanda */
  bmp280.setEnabled(0);                     //0=Desactivamos el modo automatico. Solo obtendremos respuesta
                                            //del sensor  tras una peticion con 'triggerMeasurement()

  // Si no se toman varias medidas consecutivas no mide bien la altura del suelo
  for(int i =0 ; i < 5 ; i++){
    medirAlturaYTemperatura();
    delay(100);
  }
  
  return  altura;
}

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   ALTIMETRO Y TERMOMETRO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void medirAlturaYTemperatura()
{
  bmp280.triggerMeasurement();    //peticion de nueva medicion
  bmp280.awaitMeasurement();      //esperar a que el sensor termine la medicion
  bmp280.getAltitude(altura);     //asignar el valor leido a la variable que se le indica
  bmp280.getTemperature(temperatura);
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   ULTRAVIOLETAS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void obtener_UV_max()
{
  
  int lectura_UV_max = 0;                  

  tcaselect(PUERTO_UV_1);
  int lecturaUV_RAW_1 = uv_1.readUV();
  if(lecturaUV_RAW_1 > lectura_UV_max){
    lectura_UV_max = lecturaUV_RAW_1;
  }
  tcaselect(PUERTO_UV_2);
  int lecturaUV_RAW_2 = uv_2.readUV();
  if(lecturaUV_RAW_2 > lectura_UV_max){
    lectura_UV_max = lecturaUV_RAW_2;
  }
  tcaselect(PUERTO_UV_3);
  int lecturaUV_RAW_3 = uv_3.readUV();
  if(lecturaUV_RAW_3 > lectura_UV_max){
    lectura_UV_max = lecturaUV_RAW_3;
  }

  indiceUv = lectura_UV_max/280.0;
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   EEPROM
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  SALVAR DATOS EN LA EPPROM
//========================================================

void guardarDatosMemoria(){
  
  /* Salvado de datos (MIENTRAS QUEDE MEMORIA) --  */
    if(FLAG_uso_eeprom==true){
      /* altura con  con un decimal, como un entero */
      int altura_int = int(altura*10);
      /* temepratura con dos decimales, como un entero */
      int temperatura_int = int(temperatura*100);
      /* indice UV con un decimal, como un entero */  
      uint8_t indiceUv_int = indiceUv*10;
  
      /* empaquetado de los datos de interes en un 'struct' */
      CanSatDatos datos_actuales = { altura_int, temperatura_int, indiceUv_int };
      //saveData(puntero_eeprom, datos_actuales);         //salvamos un dato del tipo 'CanSatDatos'
	  EEPROM.put(puntero_eeprom, datos_actuales);         //salvamos un dato del tipo 'CanSatDatos'
      puntero_eeprom+=5;                                //incrementamos el puntero para escritura
      /* Si llenamos la eeprom, dejamos de grabar y desactivamos los permisos de acceso*/
      if(puntero_eeprom > 1019 || puntero_eeprom < 5){ 
        FLAG_uso_eeprom = false; // bloqueo de acceso para evitar sobreescribir
      }
    }
}


//========================================================
//  LISTAR EL CONTENIDO DE LA EPPROM
//========================================================

void listar_datos()
{
  int puntero_lectura = 0;
  int contador_muestra = 0;
  
  Serial.println(F("\nTiempo (s), Altura (m), Temperatura (C) ,  Ultravioletas"));

  radioLink.println(F("\nTiempo (s), Altura (m), Temperatura (C), Ultravioletas"));
  
  while(puntero_lectura < 1020){
    /* recuperar datos de la eeprom */
    CanSatDatos dato_leido;
    EEPROM.get(puntero_lectura, dato_leido);
    
    /* incrementar el puntero de lectura de la eeprom */
    puntero_lectura +=5;
    
    /* incrementar contador de muestras */
    contador_muestra++;
    
    /* tratar los datos recuperados */
    float altura_float = float(dato_leido.altura)/10.0;
    float temperatura_float = float(dato_leido.temperatura)/100.0;
    float indiceUV_float = float(dato_leido.uv)/10.0;
   
    /* mostar datos por puerto serie */
    Serial.print(contador_muestra); Serial.print(F(","));
    Serial.print(altura_float); Serial.print(F(","));
    Serial.print(temperatura_float);Serial.print(F(","));
    Serial.println(indiceUV_float);

    /* mostar datos por puerto radio */
    radioLink.print(contador_muestra); radioLink.print(F(","));
    radioLink.print(altura_float); radioLink.print(F(","));
    radioLink.print(temperatura_float);radioLink.print(F(","));
    radioLink.println(indiceUV_float);
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   PUERTO SERIE y RADIO ENLACE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// FUNCION PARA LECTURA DE CARACTERES POR PUERTO SERIE y RADIO ENLACE
//========================================================

void atenderPeticionesEntrantes(int intervalo_miliseg) 
{
  /* Se queda a la escucha del puerto serie durante un intervalo de tiempo determinado */
  relojEscucha.stop();
  relojEscucha.begin(intervalo_miliseg); 
  
  /* salir de la escucha de comandos si se produce TIMEOUT  del temporizador relojEscucha */  
  while( relojEscucha.estado() == true ) {
	char orden_recibida = ' ';
    /* comprobar si hay comandos en el puerto serie */
    if(Serial.available()){ orden_recibida = Serial.read(); }
    
    /* comprobar si hay comandos en el via radio */
    if(radioLink.available()){ orden_recibida = radioLink.read(); }

    /* comprobar si hay una orden valida */
    if(orden_recibida == 'd' or orden_recibida == 'D'){  listar_datos(); }
  }
}



//========================================================
//  ENVIO DE MENSAJES (PUERTO SERIE y RADIOFRECUENCIA)
//========================================================
void enviar_mensaje(String mensaje){

  radioLink.println(F("Mensaje de Cansat SpaceSix: "));
  radioLink.println(mensaje);
  radioLink.println(F("***************"));

  if(EN_DEPURACION){
    Serial.println(mensaje);
    Serial.println(F("================"));
  }
}


//========================================================
// ENVIO DE DATOS (PUERTO SERIE y RADIOFRECUENCIA)
//========================================================

void envio_datos()
{
  /* Transmitir datos del experimento */
  radioLink.print(indice_muestra); radioLink.print(F(","));
  radioLink.print(altura); radioLink.print(F(","));
  radioLink.print(temperatura); radioLink.print(F(","));
  radioLink.print(indiceUv); radioLink.print(F(","));
  /* Transmitir datos de POSICION GPS */
  radioLink.print(gps.longitud,6); radioLink.print(F(","));
  radioLink.println(gps.latitud,6);
 
  /* mostar datos por serial */ 
  if(EN_DEPURACION){
    Serial.print(indice_muestra); Serial.print(F("*"));  
    Serial.print(altura); Serial.print(F("*"));  
    Serial.print(temperatura); Serial.print(F("*"));  
    Serial.print(indiceUv); Serial.print(F("*"));
    Serial.print(gps.longitud,6); Serial.print(F("*"));  
    Serial.println(gps.latitud,6);
  }
}




//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
