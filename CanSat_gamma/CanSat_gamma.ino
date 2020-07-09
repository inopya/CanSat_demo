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
# *       Versión 0.2       Fecha: 30/06/2020          * #
# ****************************************************** #
##########################################################
*/

#define __VERSION__ "CanSat_Gamma"


/*
      ===== NOTAS DE LA VERSION ===== 
  
  Programa para el Cansat del equipo SpaceSix.
      
  Evolucion la version CanSat_beta, que fue el programa  base para el campeonato regional. 
  Superado ese peldaño  y clasificados para la siguiente fase,
  esta es la version base para usar en el campeonato nacional.
  
  - Se ha sustituido la alimentacion en secuencia de los sensores UV 
    por un multiplexor de I2C, facilitando en cierto modo el montaje electrico,
    aunque apurando un poco más el poco espacio disponible (nada es gratis)
  - Montada la eeprom externa para tener mayor margen de maniobra a la hora de tomar datos.

*/


#define DEBUG_MODE   false      // Si(true): estamos en depuracion y se muestra informacion por puerto serie   
#define EEPROM_EXTERNA          // Permitir cargar la libreria de eeprom externa
                                // (Para en un futuro implemetar una version mixta, con EEPROM interna y/o externa)



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCIONPARA IMPORTACION DE LIBRERIAS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#include <Wire.h>                   // Utilidades para comunicaciones I2C
#include <Adafruit_Sensor.h>        // libreria base para sensores Adafruit
#include <i2c_BMP280.h>             // libreria de funciones para el barometro
#include <Adafruit_VEML6070.h>      // libreria para el sensor de radiacion ultravioleta
#include <SoftwareSerial.h>         // para poder implemetar mas puertos Serial
#include <Temporizador_inopya.h>    // gestion de temporiadores de forma sencilla y "amigable"
#include <Universal_GPS_inopya.h>   // Gestion simple de GPS NEO6/7/8
#include <I2C_EEPROM_inopya.h>      // Gestion simple de memorias EEPROM externas


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DEFINICION DE PINES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define PIN_RESET             5 
#define PIN_ALTAVOZ           6  
#define ANTENA_RX            11 
#define ANTENA_TX            10 
#define GPS_RX                3
#define GPS_TX                4


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/



#ifdef EEPROM_EXTERNA
  #include <I2C_EEPROM_inopya.h>  // usar memoria externa
#else
  #include <EEPROM.h>             // usar memoria interna
#endif


#define TIEMPO_MUESTREO       VEML6070_HALF_T    // ½T 62.5 ms
#define TCAADDR               0x70
#define PUERTO_UV_1           2
#define PUERTO_UV_2           3 
#define PUERTO_UV_3           4 



#define INTERVALO_MUESTRAS    1000    // tiempo en milisegundos entre cada muestra
                                      // dependiendo del intervalo, tendremos mayor o menor tiempo de grabacion 
                                      // Recordad que disponemos de memoria para solo 204 muestras si se usa eepron interna.
                                      // 32000 o 64000 en el caso de las eeprom externas de que disponemos
                                      
                                      
#define INTERVALO_RECALIBRACION_ALTURA  60000   //  60000 ms == 1 minuto



/* Definiciones relacionadas con la eeprom externa*/

#define CHIP_ID           0x50      // la direccion de memoria del chip (24LC256) si A0, A1 y A2 estan a GND es 0x50
#define EEPROM_SIZE       32700     // recordar que el chip (24LC256) dispone de 32768 posiciones de memoria (0-32767)
                                    // establece este valos con margen para que se pueda grabar la "ultima muetra"

#define POS_MEM_SESION     100      // primera posicion usada para datos, (se resevan desde la 0 hasta esta)
#define POS_MEM_CONTADOR    10      // puede ser interesante llevar un contador del numero de muestras tomadas
                                    // para facilitar el volcado de datos. Sin uso en principio

#define DATA_SIZE    5              //por si decidimos cambiar la cantidad o tipos de datos almacenados

uint16_t puntero_eeprom = POS_MEM_SESION;       // unsigned int para disponer del rango 0-65535  


/* creacion de un nuevo tipo de datos para contener las muestras */
struct CanSatDatos {  
                     int altura; 
                     int temperatura; 
                     uint8_t uv;
                   };



boolean FLAG_uso_eeprom = false;                // Preserva la eeprom de usos innecesarios

                
/* variables para almaceanr los datos de las mediciones */    
float altura;      
float temperatura;
float indiceUv;


float altura_suelo;
int altura_para_empezar_a_medir = 100;    //A que altura se ha de empezar a tomar muestras
                                          //se modifica por comando via radio

/* para el control de caida y llegada a suelo, iniciadas fuera de rango */
float altura_maxima = 0;         
float altura_anterior = 10000;
uint8_t contador_suelo = 0;


uint32_t tiempo_base;    // para establecer una marca de tiempo sobre la que referenciar la toma de muestras 

bool orden_lanzamiento = false;


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE CREACION DE OBJETOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

/* Sensores Ultravioletas */
Adafruit_VEML6070 uv_1 = Adafruit_VEML6070();
Adafruit_VEML6070 uv_2 = Adafruit_VEML6070();
Adafruit_VEML6070 uv_3 = Adafruit_VEML6070();

/* creaccion del objeto barometro/altimetro */
BMP280 bmp280;

/* creación de un puerto serie para Emisor de radiofrecuencia */
SoftwareSerial radioLink( ANTENA_TX , ANTENA_RX); 

/* creación de un puerto serie para el GPS */
SoftwareSerial gpsPort(GPS_TX, GPS_RX);  //4 ,3

/* creación del objeto GPS */
Universal_GPS_inopya gps(&gpsPort);

/* Creamos un objeto del tipo i2c_eeprom_externa */
I2C_EEPROM_inopya memory_chip(CHIP_ID);   

/* creación de  objetos Temporizador_inopya */
Temporizador_inopya relojMuestras;
Temporizador_inopya relojEscucha;
Temporizador_inopya relojRecalibracionAltura;
//Temporizador_inopya relojControlSuelo;  // Reutilizaremos el 'relojRecalibracionAltura' que solo se usa durante el setup,
                                          // para el control de suelo y asi nos ahorramos 18 bytes de RAM




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
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, HIGH);
 
  pinMode(PIN_ALTAVOZ, OUTPUT);
  digitalWrite(PIN_RESET, LOW);
  
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

  // Fijando el umbral de lanzamiento
  
  enviar_mensaje(F("Cansat necesita que se le envie un umbral de lanzamiento (en metros)"));
   
  char orden_recibida = ' ';
   while(!Serial.available() || !radioLink.available()) {
    
    if(Serial.available()){
      altura_para_empezar_a_medir = Serial.parseInt();
    }
    if(radioLink.available()){
      altura_para_empezar_a_medir = radioLink.parseInt();
    }  
    if(altura_para_empezar_a_medir >= 1){
      Serial.println(altura_para_empezar_a_medir);
      radioLink.print(F("Fijado el umbral de lanzamiento a "));
      radioLink.print(altura_para_empezar_a_medir);
      radioLink.println(F(" metros\n"));
      break;
    }
  }
  
  // Esta parte espera a que el cohete despegue y alcance una altura de más de 'altura_para_empezar_a_medir' metros
  // para empezar la toma de datos
  // Tambien se puede salir de esta espera mediante una señal de lanzamiento por radio
  
  while(!orden_lanzamiento) {

    // Cada cierto tiempo recalibramos la altura_suelo para compensar las variaciones meteorológicas
    // Y comprobamos que el GPS recibe bien los datos
    if (relojRecalibracionAltura.estado() == false){
      medirAlturaYTemperatura();
      altura_suelo = altura;
      radioLink.print(F("Recalibrando el sensor de Altitud, Altura suelo medida: "));
      radioLink.print(altura_suelo);
      radioLink.println(F("\n- Presione L para empezar a tomar datos o D para ver los datos del lanzamiento anterior\n"));
      
      if(DEBUG_MODE){
        Serial.print(F("Recalibrando el sensor de Altitud, Altura suelo medida: "));
        Serial.println(altura_suelo);
        Serial.println(F("\n- Presione L para empezar a tomar datos o D para ver los datos del lanzamiento anterior\n"));
      }
      comunicar_posicion();
      relojRecalibracionAltura.begin(INTERVALO_RECALIBRACION_ALTURA); 
    }

    // Atiende las posibles ordenes entrantes por radio o puerto serie durante 2 segundos
    atenderPeticionesEntrantes(2000);
    
    if(orden_lanzamiento == true){
      break;
    }
    
    medirAlturaYTemperatura();

    if(DEBUG_MODE){
      Serial.print(altura_suelo);
      Serial.print(F(" , "));
      Serial.println(altura);
      Serial.print(F(" , "));
      Serial.println(altura - altura_suelo);
      delay(1000);    // INOPYA deshacerse de este delay si es posible...
    }
    
    if((altura - altura_suelo) > altura_para_empezar_a_medir){
      orden_lanzamiento = true;
      break;
    }
  
  }

  FLAG_uso_eeprom = true;
  
  enviar_mensaje(F("Cansat procediendo a la toma de datos"));
  
  if (DEBUG_MODE) {
    sonidoLanzamiento();
  }
  tiempo_base = millis();  //para incluir una referencia de tiempo (relativa) en las transmisiones
  //relojControlSuelo.begin(15000); 
  relojRecalibracionAltura.begin(15000); //reusamos este reloj apra ahorra unos bytes de ram
 
}

//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  DURANTE EL LANZAMIENTO 
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void loop() 
{ 
  /* control de tiempo para tareas de toma de datos y transmision a tierra */
  if(relojMuestras.estado()== false){
    relojMuestras.begin(INTERVALO_MUESTRAS); //Reinicio del reloj de toma de muestras y otras tareas.

    /* Actualizar datos de altura y temperatura */
    medirAlturaYTemperatura();

    /* Medimos los sensores UV */
    indiceUv = obtener_UV_max();

    // Guardamos los datos en la memoria EEPROM , si queda espacio
    // Si no queda espacio en la EEPROM esta función activará la baliza de rescate
    guardarDatosMemoria();

    /* Actualizamos el dato de altura maxima alcanzada, por si lo usamos para algo */
    if (altura > altura_maxima){
      altura_maxima = altura;
    }

    /* Controlar la llegada al suelo */
//    if (abs(altura - altura_anterior)<1){
//      if(contador_suelo>2){
//        //llegada al suelo!!  emitir los datos de GPS
//        baliza_Rescate();
//      }
//      contador_suelo++;
//    }
    
    /* control de altitud para detectar llegada al suelo (relojControlSuelo)*/ 
    if(relojRecalibracionAltura.estado()== false){
      relojRecalibracionAltura.begin(10000);
      if (abs(altura - altura_anterior)<1){
        if(contador_suelo>2){
          //llegada al suelo!!  emitir los datos de GPS
          baliza_Rescate();
        }
        contador_suelo++;
      }
      altura_anterior = altura;
    }
  
    /* Transmitir datos a tierra */
    uint32_t tiempo_relativo = millis()-tiempo_base; 

    radioLink.print(tiempo_relativo); radioLink.print(F(",\t\t"));
    radioLink.print(altura); radioLink.print(F(",\t\t"));
    radioLink.print(temperatura); radioLink.print(F(",\t\t"));
    radioLink.print(indiceUv);
    radioLink.println("");
   
    /* mostar datos por serial */ 
    if(DEBUG_MODE){
      Serial.print(tiempo_relativo);
      Serial.print("*");  
      Serial.print(altura);
      Serial.print("*");
      Serial.print(temperatura);
      Serial.print("*");
      Serial.print(indiceUv);
      Serial.println("");
    }
  }
                                                                                                                                                                                                                        
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
   SETUP BAROMETRO (ALTIMETRO)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

float inicializarBarometro()
{
  if (bmp280.initialize()){
    if(DEBUG_MODE){
      Serial.println(F("Sensor presion ok"));
    }
  }
  else{
    if(DEBUG_MODE){
      Serial.println(F("Fallo de Sensor, programa detenido"));
    }
    /* si no hay barometro, notificamos por radio y quedamos en bucle infinito */
    radioLink.println(F("Fallo de Sensor, programa detenido"));
    while (true) {}  
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
  
  return altura;
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   OBTENER ALTITUD Y TEMPERATURA   (datos obligatorios por las bases del consurso)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void medirAlturaYTemperatura()
{
  bmp280.triggerMeasurement();    //peticion de nueva medicion
  bmp280.awaitMeasurement();      //esperar a que el sensor termine la medicion
  bmp280.getAltitude(altura);
  bmp280.getTemperature(temperatura);
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   OBTENER INDICE ULTRAVIOLETA  (El experimento propio)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

float obtener_UV_max()
{
  float lectura_max= 0;               // variable para almacenar el maximo
  float lectura_sensor_1;             // variable para sensor 1  
  float lectura_sensor_2;             // variable para sensor 2
  float lectura_sensor_3;             // variable para sensor 3   

  /*  leemos el sensor 1 */
  tcaselect(PUERTO_UV_1);             
  lectura_sensor_1 = uv_1.readUV();
  if(lectura_sensor_1 > lectura_max){
    lectura_max = lectura_sensor_1;
  }

  /*  leemos el sensor 2 */
  tcaselect(PUERTO_UV_2);
  lectura_sensor_2 = uv_2.readUV();
  if(lectura_sensor_2 > lectura_max){
    lectura_max = lectura_sensor_2;
  }

  /*  leemos el sensor 3 */
  tcaselect(PUERTO_UV_3);
  lectura_sensor_3 = uv_3.readUV();
  if(lectura_sensor_3 > lectura_max){
    lectura_max = lectura_sensor_3;
  }

  /*  calculamos el indice uv max */
  float indice_UV = lectura_max/280.0;

  if (DEBUG_MODE){
    Serial.print("Sensores UV: "); 
    Serial.print(lectura_sensor_1/280.0); Serial.print(", ");
    Serial.print(lectura_sensor_2/280.0); Serial.print(", ");
    Serial.println(lectura_sensor_3/280.0);
    Serial.print(F("indice_UV max: ")); Serial.println(indice_UV);
  }

  return indice_UV;
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   EEPROM  --> GUARDAR DATOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void guardarDatosMemoria()
{
  
  /* Salvado de datos (MIENTRAS QUEDE MEMORIA) --  */
  if(FLAG_uso_eeprom==true){
    /* altura con dos decimales, como un entero */
    int altura_int = int(altura*100);
    /* temepratura con dos decimales, como un entero */
    int temperatura_int = int(temperatura*100);
    /* indice UV con 1 decimal, como un entero */  
    uint8_t indiceUv_int = indiceUv*10;
  
    /* Crear un empaquetado de los datos de interes en un 'struct' */
    CanSatDatos datos_actuales = { altura_int, temperatura_int, indiceUv_int };
  
    /* Guardar la muestra */
    memory_chip.save(puntero_eeprom, datos_actuales);   // grabamos un dato del tipo 'CanSatDatos'
    puntero_eeprom += DATA_SIZE;                        // incrementamos el puntero para escritura
    
    /* Si llenamos la eeprom, dejamos de grabar y desactivamos los permisos de acceso*/
    if(puntero_eeprom > EEPROM_SIZE || puntero_eeprom < POS_MEM_SESION){ 
      FLAG_uso_eeprom = false; // bloqueo de acceso para evitar sobreescribir
      orden_lanzamiento = false;
      baliza_Rescate();
    }
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   EEPROM  --> MOSTAR LISTADO DE DATOS CAPTURADOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void listar_datos()
{
  uint16_t puntero_lectura = POS_MEM_SESION;
  uint16_t contador_datos = 0;
  
  Serial.println();
  Serial.println(F("Tiempo (s), Altura (m), Temperatura (C) ,  Ultravioletas"));

  radioLink.println();
  radioLink.println(F("Tiempo (s), Altura (m), Temperatura (C), Ultravioletas"));
  
  while(puntero_lectura < EEPROM_SIZE){
    /* recuperar datos de la eeprom */
    CanSatDatos dato_leido;                         // creamos un strut para recuperar los datos
    memory_chip.load(puntero_eeprom, dato_leido);   // leemos un dato 
    
    /* incrementar el puntero de lectura de la eeprom */
    puntero_lectura += DATA_SIZE;
    
    /* tratar los datos recuperados */
    float altura_float = float(dato_leido.altura)/100.0;
    float temperatura_float = float(dato_leido.temperatura)/100.0;
    float indiceUV_float = float(dato_leido.uv)/10.0;
   
    /* mostar datos por puerto serie */
    Serial.print(contador_datos++); Serial.print(F(","));
    Serial.print(altura_float); Serial.print(F(","));
    Serial.print(temperatura_float);Serial.print(F(","));
    Serial.print(indiceUV_float);
    Serial.println("");

    /* mostar datos por puerto radio */
    radioLink.print(contador_datos++); radioLink.print(F(","));
    radioLink.print(altura_float); radioLink.print(F(","));
    radioLink.print(temperatura_float);radioLink.print(F(","));
    radioLink.print(indiceUV_float);
    radioLink.println("");
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   PUERTO SERIE Y RADIO ENLACE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

int atenderPeticionesEntrantes(int intervalo_miliseg) 
  
{
  // Se queda a la escucha del puerto serie durante un intervalo de tiempo determinado
  if (relojEscucha.estado() == false){
      relojEscucha.begin(intervalo_miliseg); //Para mostrar los mensajes cada 3 segundos
    }
    
  char orden_recibida = ' ';
   while(!Serial.available() || !radioLink.available()) {
    
    if(Serial.available()){
      orden_recibida = Serial.read();
    }
    if(radioLink.available()){
      orden_recibida = radioLink.read();
    }
    if(orden_recibida == 'd' or orden_recibida == 'D'){ 
      //Serial.flush();
      listar_datos();
    }
    if(orden_recibida == 'l' or orden_recibida == 'L'){ 
      orden_lanzamiento = true;
    }
    if (relojEscucha.estado() == false){
      break;
    }
   }
  return 0;
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   FORMATO DE SEPARADORES APRA ENVIO DE MENSAJES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void enviar_mensaje(String mensaje){

  radioLink.println(F("********************************************************************"));
  radioLink.print(F("Mensaje de Cansat SpaceSix: "));
  radioLink.println(mensaje);
  radioLink.println(F("********************************************************************\n"));
  
  if(DEBUG_MODE){
    Serial.println(F("======================================================================="));
    Serial.println(mensaje);
    Serial.println(F("=======================================================================\n"));
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   LANZAMIENTO
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void sonidoLanzamiento(){

  int cont = 0;
  while(cont < 5){
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
  /* obtener datos del GPS */
  gps.get(); 
  /* Transmitir datos de POSICION GPS */
  radioLink.print(F("Longitud ")); radioLink.print(gps.longitud,6);
  radioLink.print(F(" , Latitud ")); radioLink.println(gps.latitud,6);
  
  if(DEBUG_MODE){
    Serial.print(F("Longitud ")); Serial.print(gps.longitud,6);
    Serial.print(" , Latitud "); Serial.println(gps.latitud,6);
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   LOCALIZACION DURANTE EL RESCATE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void baliza_Rescate()
{
  //INOPYA REVISAR
  
  while(orden_lanzamiento == false ){
    
    /* Generar tono para localizacion */
    for(int i =0; i < 5; i++){
        tone(PIN_ALTAVOZ, 2100);  //frecuencia que emite un sonido bastante estridente
        delay (450);
        noTone(PIN_ALTAVOZ);
    }

    comunicar_posicion();
    
    enviar_mensaje(F("Cansat ha realizado su aterrizaje, presione D para ver los datos o L para un nuevo lanzamiento "));
   
    atenderPeticionesEntrantes(2000);
    if(orden_lanzamiento == true){
      puntero_eeprom = 0;
      FLAG_uso_eeprom = true;   
      if(DEBUG_MODE){
        Serial.println(F("Realizando un nuevo lanzamiento"));
      }
      //Reseteo el Arduino
      tiempo_base = millis();
      digitalWrite(PIN_RESET, LOW); 
    }
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   LOCALIZACION DURANTE EL RESCATE v1
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void baliza_Rescate_original()
{
  while(true){
    /* Generar tono para localizacion */
    tone(PIN_ALTAVOZ, 2100);  //frecuencia que emite un sonido bastante estridente
    delay (450);
    noTone(PIN_ALTAVOZ);

    /* obtener datos del GPS */
    //gps.get();   comunicar_posicion() incluye el refresco de datos del gps
    comunicar_posicion();  
    
    /* Transmitir datos de HORA GPS */
    if(gps.hora < 10){ radioLink.print(F("0")); }
    radioLink.print(gps.hora); radioLink.print(F(":"));
    if(gps.minuto < 10){ radioLink.print(F("0")); }
    radioLink.print(gps.minuto); radioLink.print(F(":"));
    if(gps.segundo < 10){ radioLink.print(F("0")); }
    radioLink.print(gps.segundo); 
    radioLink.print(F(" , "));  
    
    /* Transmitir datos de FECHA GPS */
    if(gps.dia < 10){ radioLink.print(F("0")); }
    radioLink.print(gps.dia); radioLink.print(F("/"));
    if(gps.mes < 10){ radioLink.print(F("0")); }
    radioLink.print(gps.mes); radioLink.print(F("/"));
    if(gps.year < 10){ radioLink.print(F("0")); }
    radioLink.print(gps.year);

    delay(5000);  //soy enemigo de los delay, pero.. por ahora se queda aqui
  }
}
