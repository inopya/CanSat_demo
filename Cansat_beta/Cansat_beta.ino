/*

  Este programa toma el relevo al Datalogger_en_EEPROM_del_328p
  
  que se puede encontrar en
  https://github.com/inopya/Datalogger_en_EEPROM_del_328p
  
  EN PROCESO......
  
*/

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCIONPARA IMPORTACION DE LIBRERIAS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


#include <Wire.h>                 // Utilidades para comunicaciones I2C
#include <i2c_BMP280.h>           // Biblioteca de funciones para el barometro
#include <EEPROM.h>
#include <Adafruit_VEML6070.h>
#include <SoftwareSerial.h>
#include <Temporizador_inopya.h>
#include <Universal_GPS_inopya.h>

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DEFINICION DE PINES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define PIN_PULSADOR         2
#define GPS_RX               3
#define GPS_TX               4
#define UV_1                 5        
#define UV_2                 6  
#define UV_3                 7        
#define PIN_ALTAVOZ          9
#define ANTENA_RX           10
#define ANTENA_TX           11

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE CREACION DE OBJETOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

/* creación de un puerto serie para el GPS */
SoftwareSerial gpsPort(GPS_RX, GPS_TX);

/* creación del objeto GPS */
Universal_GPS_inopya NEO_gps(&gpsPort);

/* creaccion del objeto barometro/altimetro */
BMP280 bmp280;

/* creación del objeto sensor uv */
Adafruit_VEML6070 sensor_UV6070 = Adafruit_VEML6070();
uint8_t uvSensorList[3] = {UV_1, UV_2, UV_3};

/* creación de un puerto serie para Emisor de radiofrecuencia */
SoftwareSerial radioLink(ANTENA_RX, ANTENA_TX); // RX, TX

/* creación de  objetos Temporizador_inopya */
Temporizador_inopya relojMuestras;
Temporizador_inopya relojControlSuelo;

//teporizadores de muestra apra generar un aparpadeo asimetrico un unled sin delay 
Temporizador_inopya intervaloParpadeo;
Temporizador_inopya ledOn;



   
/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/


#define INTERVALO_MUESTRAS    1000    // tiempo en milisegundos entre cada muestra
                                      // Recordad que disponemos de memoria para 204 muestras,
                                      // dependiendo del intervalo, tendremos mayor o menor tiempo de grabacion 
                        
/* creacion de un nuevo tipo de datos para contener las muestras */
struct CanSatDatos {  
                     int altura; 
                     int temperatura; 
                     uint8_t uv;  
                   };


                              

volatile boolean FLAG_estado_pulsador = false;      // bandera de control apra la interrupcion del pulsador

boolean FLAG_uso_eeprom = false;                    // Preserva la eeprom de usos innecesarios
uint8_t contador_suelo = 0;
int puntero_eeprom = 0;   
               
float altura;      
float temperatura;
float indiceUv;

float altura_suelo;
float altura_maxima = 0;
float altura_anterior = 10000;  //para el control de llaga a suelo, iniciada fuera de rango


//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//         FUNCION DE CONFIGURACION
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 


void setup() 
{
  pinMode(PIN_PULSADOR, INPUT);
  pinMode(UV_1, OUTPUT);
  pinMode(UV_2, OUTPUT);
  pinMode(UV_3, OUTPUT);
  pinMode(PIN_ALTAVOZ, OUTPUT);


  /* inicializamos el puerto serie para el PC  (pruebas) */
  Serial.begin(9600);
  
  /* inicializamos el puerto serie para el emisor RF */
  radioLink.begin(9600);

  /* inicializamos el puerto serie para elGPS) */
  NEO_gps.begin(9600);  //iniciamos el GPS a la velocidad standard

  NEO_gps.set_mode(0);

  /* cortamos la alimentacion a todos los sensores UV */
  for(uint8_t n=0;n<3;n++){
    digitalWrite(uvSensorList[n], HIGH);  //al ser un transistor PNP se desactiva con voltaje alto  
  }

  /* inicializamos el barometro */    
  if (bmp280.initialize()){
    Serial.println(F("Sensor ok"));
  }
  else{
    Serial.println(F("Fallo de Sensor"));
    while (true) {}  //si no hay barmetro el progrma queda en bucle infinito
  }

  /* Configuracion de calibracion */
  bmp280.setPressureOversampleRatio(8);     //2
  bmp280.setTemperatureOversampleRatio(1);
  bmp280.setFilterRatio(4);                //4
  bmp280.setStandby(0);                     // 0=desactivado, por tanto el sensor esta activo.
  
  /* Medidas bajo demanda */
  bmp280.setEnabled(0);                     //0=Desactivamos el modo automatico. Solo obtendremos respuesta
                                            //del sensor  tras una peticion con 'triggerMeasurement()
  
  
  delay(2000);  //  segundos para dar tiempo a reprogramar la placa cuando sea necesario (Necesario en arduino Micro)

  /* El programa en cualquier momento estará atento al estado del pulsador */
  attachInterrupt(digitalPinToInterrupt(PIN_PULSADOR), atenderInterrupcion, RISING);

  /* Permite el uso de un pulsador apra iniciar la toma de muestras (util en pruebas) */ 
//  // El programa espera en esta linea hasta que sea pulsado el pulsador
//  while(FLAG_estado_pulsador == false){
//    atenderPuertoSerie();
//    }

  /* Detecccion AUTOMATICA de chohete en en aire e inicio de toma de muestras */ 
  medirAlturaYTemperatura();
  altura_suelo = altura;

  while((altura - altura_suelo) < 500) {
    atenderPuertoSerie();   //para poder recuperar los datos de la eeprom
    medirAlturaYTemperatura();
    delay(1000);
    Serial.println("esperando");
    }

  FLAG_uso_eeprom = true;
  relojControlSuelo.begin(15000);

}



//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  BUCLE PRINCIPAL DEL PROGRAMA  
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 


void loop() 
{ 
  /* control de tiempo para tareas de toma de datos y transmision a tierra */
  if(relojMuestras.estado()== false){
    relojMuestras.begin(1000); //Reinicio del reloj de toma de muestras y otras tareas.

    /* Actualizar datos de altura y temperatura */
    medirAlturaYTemperatura();

    /* Actualizar el indice Uv */
    indiceUv = obtener_UV_max();

    /* Salvado de datos (MIENTRAS QUEDE MEMORIA) --  */
    if(FLAG_uso_eeprom==true){
      /* altura con dos decimales, como un entero */
      int altura_int = int(altura*100);
      /* temepratura con dos decimales, como un entero */
      int temperatura_int = int(temperatura*100);
      /* indice UV con 1 decimal, como un entero */  
      uint8_t indiceUv_int = indiceUv*10;
  
      /* empaquetado de los datos de interes en un 'struct' */
      CanSatDatos datos_actuales = { altura_int, temperatura_int, indiceUv_int };
      radioLink.println(altura_int);
      saveData(puntero_eeprom, datos_actuales);         //enviamos un dato del tipo 'CanSatDatos'
      puntero_eeprom+=5;                                //incrementamos el puntero para escritura
      /* Si llenamos la eeprom, dejamos de grabar y desactivamos los permisos de acceso*/
      if(puntero_eeprom > 1020 || puntero_eeprom < 5){ 
        FLAG_uso_eeprom = false; // bloqueo de acceso para evitar sobreescribir
      }
    }

    /* Actualizamos el dato de altura maxima alcanzada, por si lo usamos para algo */
    if (altura > altura_maxima){
      altura_maxima = altura;
    }

    /* Controlar la llegada al suelo */
    if (abs(altura - altura_anterior)<1){
      contador_suelo++;
      if(contador_suelo>2){
        //llegada al suelo!!  emitir los datos de GPS
        baliza_Rescate();
      }
    }    

    
    /* Transmitir datos a tierra */
    radioLink.print(altura);
    radioLink.print("*");
    radioLink.print(temperatura);
    radioLink.print("*");
    radioLink.println(indiceUv);
    

    /* mostar datos por serial (solo apra pruebas, eliminar del programa final */
    Serial.print(altura);
    Serial.print("*");
    Serial.print(temperatura);
    Serial.print("*");
    Serial.println(indiceUv);
  }

  /* control de alitud para detectar llegada al suelo */
  if(relojControlSuelo.estado()== false){
    relojControlSuelo.begin(10000);
    altura_anterior = altura;
  }


  //------> inicio de bloque de EJEMPLO de temporizadores inopya  
  if(intervaloParpadeo.estado()==false){
    intervaloParpadeo.begin(200);         // tiempo apagado + t. encendido
    ledOn.begin(150);                     // tiempo encendido
  }  
  digitalWrite(3, ledOn.estado());  //ojo este pin esta usado en el montaje real por un sensor UV
  //<------  fin de bloque de  EJEMPLO de temporizadores                                                                                                                                                                                                                           
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
   ###################################################################################################### 
        BLOQUE DE FUNCIONES: LECTURAS DE SENSORES, COMUNICACIONES... 
   ###################################################################################################### 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   ACTUALIZAR DATOS DE ATITUD Y TEMPERATURA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void medirAlturaYTemperatura()
{
  bmp280.triggerMeasurement();    //peticion de nueva medicion
  bmp280.awaitMeasurement();      //esperar a que el sensor termine la medicion
  bmp280.getAltitude(altura);
  bmp280.getTemperature(temperatura);
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   EEPROM
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  LEER DATOS ALMACENADOS EN EPPROM
//========================================================

CanSatDatos loadData(int posicion)
{
  CanSatDatos muestra;
  EEPROM.get(posicion, muestra);
  return muestra;
}


//========================================================
//  SALVAR DATOS EN LA EPPROM
//========================================================

void saveData(int posicion, CanSatDatos muestra)
{
  EEPROM.put(posicion, muestra);
}


//========================================================
//  LISTAR EL CONTENIDO DE LA EPPROM
//========================================================

void listar_datos()
{
  int puntero_lectura = 0;
  float contador = 0;
  
  Serial.println();
  Serial.println(F("orden, Altura (m), Temperatura (C), indice UV"));
  
  while(puntero_lectura < 1020){
    /* recuperar datos de la eeprom */
    CanSatDatos dato_leido;
    EEPROM.get(puntero_lectura, dato_leido);
    
    /* tratar los datos recuperados */
    float altura_float = float(dato_leido.altura)/100.0;
    float temperatura_float = float(dato_leido.temperatura)/100.0;
    float indiceUV_float = float(dato_leido.uv)/10.0;

    /* mostar datos por puerto serie */
    Serial.print(contador++); Serial.print(F(","));
    Serial.print(altura_float); Serial.print(F(","));
    Serial.print(temperatura_float);Serial.print(F(","));
    Serial.println(indiceUV_float);
    
    /* incrementar el puntero de lectura de la eeprom */
    puntero_lectura +=5;
  }
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   PUERTO SERIE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
// FUNCION PARA LECTURA DE CARACTERES POR PUERTO SERIE
//========================================================

int atenderPuertoSerie() 
{
  char orden_recibida = ' ';
   while(Serial.available()) {
    orden_recibida = Serial.read();
    
    /* listar datos de eeprom */
    if(orden_recibida == 'l' or orden_recibida == 'L'){ 
      Serial.flush();
      listar_datos();
    }
  }
  return 0;
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   ATENDER INTERRUPCIONES DEL PUSADOR
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void atenderInterrupcion()
{
  FLAG_estado_pulsador = !FLAG_estado_pulsador;
  delay(500); //Para evitar rebotes
} 


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   LOCALIZACION DURANTE EL RESCATE
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void baliza_Rescate()
{
  while(true){
    /* Generar tono para localizacion */
    tone(PIN_ALTAVOZ, 2100);  //frecuencia que emite un sonido bastante estridente
    delay (450);
    noTone(PIN_ALTAVOZ);

    /* obtener datos del GPS */
    NEO_gps.get(); 

    /* Transmitir datos de POSICION GPS */
    radioLink.print(NEO_gps.longitud,6);
    radioLink.print(F(" , "));
    radioLink.println(NEO_gps.latitud,6);
    radioLink.print(F(" , "));  
    
    /* Transmitir datos de HORA GPS */
    if(NEO_gps.hora < 10){ radioLink.print(F("0")); }
    Serial.print(NEO_gps.hora); radioLink.print(F(":"));
    if(NEO_gps.minuto < 10){ radioLink.print(F("0")); }
    radioLink.print(NEO_gps.minuto); radioLink.print(F(":"));
    if(NEO_gps.segundo < 10){ radioLink.print(F("0")); }
    radioLink.print(NEO_gps.segundo); 
    radioLink.print(F(" , "));  
    
    /* Transmitir datos de FECHA GPS */
    if(NEO_gps.dia < 10){ radioLink.print(F("0")); }
    radioLink.print(NEO_gps.dia); radioLink.print(F("/"));
    if(NEO_gps.mes < 10){ radioLink.print(F("0")); }
    radioLink.print(NEO_gps.mes); radioLink.print(F("/"));
    if(NEO_gps.year < 10){ radioLink.print(F("0")); }
    radioLink.print(NEO_gps.year);
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   OBTENER INDICE UV
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

float obtener_UV_max()
{
  float indice_UV_max = 0.0;                          //reseteamos el indice uv maximo
  
  for(uint8_t j=0;j<3;j++){
    digitalWrite(uvSensorList[j], HIGH);              //cortamos la alimentacion a todos los sensores
  }  
  
  for(uint8_t n=0;n<3;n++){

    digitalWrite(uvSensorList[n], LOW);               //alimentamos los sensores secuencialmente
    delay(10);                                        //pausa para estabilizar la alimentacion
    sensor_UV6070.begin(VEML6070_1_T);                //reiniciamos el sensor (si no no funciona correctamente)
    int lecturaUV_RAW = sensor_UV6070.readUV();       //obtener el valor de luz ultravioleta
    float indice_UV = lecturaUV_RAW/280.0;            //procesamos para obtener indice de radiacion UV

    digitalWrite(uvSensorList[n], HIGH);              //volvemos a apagar el sensor utilizado
    if (indice_UV > indice_UV_max){
      indice_UV_max = indice_UV;
    }

    // ¡¡¡ eliminar todos los print del programa final !!!!
    Serial.print(F("Sensor UV(")); Serial.print(n);
    Serial.print(F(") RAW:")); Serial.print(lecturaUV_RAW);
    Serial.print(F(" , UV index:")); Serial.println(indice_UV,2);
  }
  Serial.print(F("UV max. = ")); Serial.println(indice_UV_max); Serial.println();
  return indice_UV_max;
}


//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
