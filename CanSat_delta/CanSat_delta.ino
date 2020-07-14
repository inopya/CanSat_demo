


/*
#       _\|/_   A ver..., ¿que tenemos por aqui?
#       (O-O)        
# ---oOO-(_)-OOo---------------------------------
 
 
##########################################################
# ****************************************************** #
# *         AERONAUTICA PARA PRINCIPIANTES             * #
# *                                                    * #
# *          Autor:  Eulogio López Cayuela             * #
# *                                                    * #
# *       Versión 0.4       Fecha: 12/07/2020          * #
# ****************************************************** #
##########################################################
*/

#define __VERSION__ "CanSat_Delta 0.4"


/*
      ===== NOTAS DE LA VERSION ===== 
  
  Programa para el Cansat del equipo SpaceSix.
      
  Simplificacion de CanSat_Gamma.
  Dispone de todos los elementos integrados, por tanto es apta para participacion en el concurso ESERO/CANSAT, 
  pero eliminadas todas las partes correspondientes a debug y testeo de sistemas durante la produccion, 
  convirtiendose  de esta manera en una version mas robusta y menos ávida de recursos.
  Renombradas algunas variables y añadidos algunos comentarios extra para facilitar el uso de este código.

  Se corresponde con la version v8 utilizada por los chavales del equipo SpaceSix 
  y es una optimizacion de dicha version. 
 
   usos de memoria para la v8 antes de optimizar:
   FLASH, 18802 bytes (58%).
   RAM utilizada, 1352 bytes (66%).

   usos de memoria para la v8 DESPUES DE OPTIMIZAR:
   FLASH, 18444 bytes (57%).
   RAM utilizada, 902 bytes (44%).
   
*/




// mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        Valores a cambiar o tener en cuenta para un lanzamiento oficial
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define WAIT_SERIAL_AT_STARTUP      25000     // 25 segundos de espera al reinciar para poder recuperar datos 

#define MARGEN_COTA_MAX               350     // haber descendido al menos 350 metros desde la cota maxima
#define MARGEN_PROXIMIDAD_SUELO       300     // encontrarnos a 300 metros o menos del suelo una vez en descenso
#define ALTURA_PARA_EMPEZAR_A_MEDIR    30     // A que altura se ha de empezar a tomar muestras al ascender

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


#define PIN_PULSADOR         2     // Sin uso
#define PIN_RESET            5     // habilitar la posibilidad de un reset controlado por software
#define PIN_ALTAVOZ          6     // zumbador asociaso a la baliza
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


/* Variables para el control de altitud y deteccion de descenso y aterrizaje */
float altura_suelo;                 // nos marcará una referencia "cota cero" al momento de iniciar el sistema
float altura_anterior = 10000;      // usada en comparaciones para el control de llegada al suelo, iniciada fuera de rango
uint8_t contador_suelo = 0;         // contador para detectar que hemos aterrizado
float altura_maxima=0;              // control de cota maxima para detectar el descenso

int indice_muestra = 0;             // indice de muestra que se enviará por radio junto con los datos recabados

bool FLAG_reiniciar_lanzamiento = false;   //bandera apra el control de reinicio de lanzamiento desde señal de radio



//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  ANTES DEL LANZAMIENTO
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void setup() 
{
  /* setup de una salida para actuar como reset software */
  digitalWrite(PIN_RESET, HIGH);
  delay(200); 
  pinMode(PIN_RESET, OUTPUT);

  /* setup de la salida para altavoz/zumbador */
  pinMode(PIN_ALTAVOZ, OUTPUT);

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
  uint8_t contador_espera_lanzamiento = 0;
  while(true) {
    // Tomar y enviar por radio una muestra cada 1 segundo
    if (relojMuestras.estado() == false){
      relojMuestras.begin(INTERVALO_MUESTRAS);
      medirAlturaYTemperatura();
      obtener_UV_max();
      envio_datos();
      contador_espera_lanzamiento++;
    }
    if(contador_espera_lanzamiento>=25){
      contador_espera_lanzamiento=0;
      /* cada 25 segundos, mientras permanece en espera y/o no se alcance la cota de lanzamiento */
      comunicar_posicion(); 
    }

    if((altura - altura_suelo) > ALTURA_PARA_EMPEZAR_A_MEDIR){
      enviar_mensaje(F("Lanzamiento detectado"));
      indice_muestra = 0;
      FLAG_uso_eeprom = true;
      break;
    }
  }
}


//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  DURANTE EL LANZAMIENTO 
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void loop() 
{ 
  if(relojMuestras.estado()== false){
    relojMuestras.begin(INTERVALO_MUESTRAS); //Reinicio del reloj de toma de muestras

    medirAlturaYTemperatura();
    obtener_UV_max();
    envio_datos();

    /* guardar datos en memoria si aun hay espacio */
    if(FLAG_uso_eeprom==true){
      guardarDatosMemoria();
    }
    
    /* Actualizamos el dato de altura maxima alcanzada */
    if ( altura>altura_maxima ){
      altura_maxima = altura;
    }

    /* Enviar coordenadas durante la parte final de la caida  para ir facilitando el rescate */
    // ** Nota: Si usamos solo la condicion de proximidad, 
    //          enviamos posicion GPS durante ese tramo del recorrido  tambien durante el ascenso
    //     <---- condicion descenso ------>            <-------- condicion proximidad --------->
    if( altura<(altura_maxima - MARGEN_COTA_MAX) && altura<(altura_suelo + MARGEN_PROXIMIDAD_SUELO) ){
      comunicar_posicion(); 
    }
    
    /* Control para detectar la llegada al suelo */ 
    if(relojEscucha.estado()== false){
      relojEscucha.begin(10000);
      if (abs(altura - altura_anterior)<1){
        if(contador_suelo>2){
            baliza_Rescate();  /* llegada al suelo!!  emitir los datos de GPS como unica tarea */
        }
        contador_suelo++;
      }
      altura_anterior = altura;
    }
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
    if(EN_DEPURACION){ Serial.println(F("Sensor presion ok"));}
  }
  else{
    if(EN_DEPURACION){ Serial.println(F("Fallo de Sensor, programa detenido"));}
    
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
//   ULTRAVIOLETA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  SELECCION DE SENSOR UV QUE DEBE ESTAR ACTIVO
//========================================================
void tcaselect(uint8_t i) 
{
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


//========================================================
//  OBTENER INDICE UV MAXIMO
//========================================================

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
  int contador_serie = 0;
  int contador_radio = 0;
  
  Serial.println(F("\nTiempo (s), Altura (m), Temperatura (C) ,  Ultravioletas"));

  radioLink.println(F("\nTiempo (s), Altura (m), Temperatura (C), Ultravioletas"));
  
  while(puntero_lectura < 1020){
    /* recuperar datos de la eeprom */
    CanSatDatos dato_leido;
    EEPROM.get(puntero_lectura, dato_leido);
    
    /* tratar los datos recuperados */
    float altura_float = float(dato_leido.altura)/10.0;
    float temperatura_float = float(dato_leido.temperatura)/100.0;
    float indiceUV_float = float(dato_leido.uv)/10.0;
   
    /* mostar datos por puerto serie */
    Serial.print(contador_serie++); Serial.print(F(","));
    Serial.print(altura_float); Serial.print(F(","));
    Serial.print(temperatura_float);Serial.print(F(","));
    Serial.println(indiceUV_float);

    /* mostar datos por puerto radio */
    radioLink.print(contador_radio++); radioLink.print(F(","));
    radioLink.print(altura_float); radioLink.print(F(","));
    radioLink.print(temperatura_float);radioLink.print(F(","));
    radioLink.println(indiceUV_float);
    
    /* incrementar el puntero de lectura de la eeprom */
    puntero_lectura +=5;
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   PUERTO SERIE y RADIO ENLACE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// FUNCION PARA LECTURA DE CARACTERES POR PUERTO SERIE y RADIO ENLACE
//========================================================

void atenderPeticionesEntrantes(uint16_t tiempo_escucha) 
{
  relojEscucha.stop();  //paramos el reloj por si tenia tiempo aun de otras tareas
  relojEscucha.begin(tiempo_escucha); //le cargamos el tiempo de escucha (en milisegundos)
    
  /* salir de la escucha de comandos si se produce TIMEOUT  del temporizador relojEscucha */ 
  while( relojEscucha.estado() == true ) {
    char orden_recibida = ' ';
    
    if(Serial.available()){ orden_recibida = Serial.read(); }
    if(radioLink.available()){ orden_recibida = radioLink.read(); }
	  
    if( orden_recibida == 'd' or orden_recibida == 'D' ){ listar_datos(); }
    if( orden_recibida == 'l' or orden_recibida == 'L' ){ FLAG_reiniciar_lanzamiento = true; }
   }
}



//========================================================
//  ENVIO DE MENSAJES (PUERTO SERIE y RADIOFRECUENCIA)
//========================================================
void enviar_mensaje(String mensaje){

  //SeparadorRadio();
  radioLink.println(F("***************"));
  radioLink.print(F("Mensaje de Cansat SpaceSix: "));
  radioLink.println(mensaje);
  //SeparadorRadio();
  radioLink.println(F("***************"));

  if(EN_DEPURACION){
    //SeparadorSerial();
    Serial.println(F("==============="));
    Serial.println(mensaje);
    //SeparadorSerial();
    Serial.println(F("==============="));  
  }
}


//========================================================
// MOSTAR BARRA SEPARADORA ENTRE ALGUNOS MENSAJES
//========================================================
void SeparadorRadio()
{
	for(uint8_t i=0;i<70;i++){
		radioLink.print(F("*"));
	}
	radioLink.println();
}


void SeparadorSerial()
{
	for(uint8_t i=0;i<70;i++){
		Serial.print(F("="));
	}
	Serial.println();
}


//========================================================
// ENVIO DE DATOS (PUERTO SERIE y RADIOFRECUENCIA)
//========================================================

void envio_datos()
{
  indice_muestra++;

  radioLink.print(indice_muestra); radioLink.print(F(",\t\t"));
  radioLink.print(altura); radioLink.print(F(",\t\t"));
  radioLink.print(temperatura); radioLink.print(F(",\t\t"));
  radioLink.println(indiceUv);
 
  /* mostar datos por serial */ 
 if(EN_DEPURACION){
  Serial.print(indice_muestra);
  Serial.print(F("*"));  
  Serial.print(altura);
  Serial.print(F("*"));  
  Serial.print(temperatura);
  Serial.print(F("*"));  
  Serial.println(indiceUv);
 }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   ALTAVOZ:  PARA GENERAR UN SONIDO DURANTE EL LANZAMIENTO  (SIN USO EN ESTA VERSION)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void sonidoLanzamiento(int numRepeticiones){

  int cont = 0;
  while(cont < numRepeticiones){
    /* Generar tono para localizacion */
    tone(PIN_ALTAVOZ, 2100);  //frecuencia que emite un sonido bastante estridente
    delay (450);
    noTone(PIN_ALTAVOZ);
    delay (450);
    cont++;
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   GPS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void comunicar_posicion()
{
   /* obtener datos actualizados del GPS */
  gps.get(); 
  /* Transmitir datos de POSICION GPS */
  radioLink.print(F("Longitud ")); radioLink.print(gps.longitud,6);
  radioLink.print(F(" , "));
  radioLink.print(F("Latitud ")); radioLink.println(gps.latitud,6);
  
  if(EN_DEPURACION){
    Serial.print(F("Longitud ")); Serial.print(gps.longitud,6);
    Serial.print(F(" , "));
    Serial.print(F("Latitud ")); Serial.println(gps.latitud,6);
  }
}

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   LOCALIZACION DURANTE EL RESCATE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void baliza_Rescate()
{
  FLAG_reiniciar_lanzamiento = false;
  while( true ){
    
    /* Generar tono para localizacion */
    for(int i =0; i < 5; i++){
      tone(PIN_ALTAVOZ, 2100);  //frecuencia que emite un sonido bastante estridente
      delay (450);
      noTone(PIN_ALTAVOZ);
    }

    comunicar_posicion(); 
    
    enviar_mensaje(F("Cansat en tierra, presione D para ver los datos o L para nuevo lanzamiento"));
   
    atenderPeticionesEntrantes(2000);
    if(FLAG_reiniciar_lanzamiento == true){
         
      if(EN_DEPURACION){ Serial.println(F("Reiniciando el sistema")); }
      radioLink.println(F("Reiniciando el sistema"));
      
      //Reseteo el Arduino
      digitalWrite(PIN_RESET, LOW); 
    }
  }
}


//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
