#include "arduino_freertos.h"
#include "queue.h"
#include "avr/pgmspace.h"

//------------COMUNIACIÓN POR I2C--------------
#include <Wire.h>
//---------------------------------------------


//-------GPS--------------
#include <Wire.h> //Needed for I2C to GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;
int RST_GPS = 14, SAFE_GPS = 13;
//------------------------

//--------------------IMU-----------------------
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
//----------------------------------------------


//------------Filtro--------------
#include "Kalman.h"
SensorKalman vela_kalman, timon_kalman;
//---------------------------------------------

//-------SERVO------------
#include <PWMServo.h>
PWMServo Servo_VELA;  // create servo object to control a servo
PWMServo Servo_TIMON;
//-------SERVO------------


//--------------DHT--------------
#include "DHT.h"
#define DHTPIN 39     // Pin donde está conectado el sensor
#define DHTTYPE DHT22   // Sensor DHT22

DHT dht(DHTPIN, DHTTYPE);

float humedad, temperatura;
//-------------------------------


//PINES PARA SERVOS
int PinServoVela = 23, PinServoTimon = 20;

//PINES PARA RC
int RCPinVela = 25, RCPinTimon = 24;
int RCPinSwitcheo = 6;

//VARIABLES PARA ANGULOS POR SERIAL
int vela, timon, filterVELA, filterTIMON;
int swticheo_anterior, vela_anterior, timon_anterior;

//VARIABLES PARA EL ENVIO POR TELEMETRIA
String dataSD = "", dataTELEMETRY = "";

//--------------VARIABLES--------------------
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long t0=millis();
float velN=0, velE=0, gvel=0;
float stdvel=0; //error de velocidad Acc
byte SIV;
double ax, ay, az, linearx, lineary, linearz, vgx, vgy, vgz, mx, my, mz, yaw, pitch, roll;
long t, altitude, hora, minutos, milisegundos;

// DEFINING THE VARIABLES FOR THE GPS NAVIGATION
const int sail_offset = 90, rudder_offset = 90;
long s_longitude, s_latitude; // represents the start position marked by the GPS
long c_longitude, c_latitude; // current position it updates with a fixed rate 
long targetLongitude= 0, targetLatitude=0; // Indicates the target position 

// DEFINING THE PARAMETERS FOR THE CONTROL LOOP
double bearing_setpoint,c_yaw_angle, rudder_angle, sail_angle;
//Define the aggressive and conservative Tuning Parameters
double aggKp=25, aggKi=0.8, aggKd=15;
double consKp=20, consKi=0.5, consKd=10.25;

void set_sail_direction(); // Defines the position of the control sail given the wind direction (no control at all)
double get_bearing_setpoint(); // Defines the heading setpoint for the vessel
bool target_reached(); // Defines if the target was reached
float calculateDistance(); // Calculates the distance that is left 
void tack_movement();



// Define the queue handle
QueueHandle_t dataSERVOS;
QueueHandle_t servos_angle;
QueueHandle_t SensorQueue;
QueueHandle_t GPSQueue;

// Define a struct to be sent between tasks
struct ReceptorDATA {
  int time_vela;
  int time_timon;
  int angulo_vela;
  int angulo_timon;
};

struct SensorsDATA{
  float humedad;
  float temperatura;
};

struct gpsDATA{
  double latitude;
  double longitude;
  double altitude;
  byte SIV;
};

//FUNCIONES EN PARALELO, presentacion 23 de Agosto
static void Read_receptor(void*) ;
static void Move_servos(void*);
static void Lectura_IMU(void*);
static void Read_sensors(void*);
static void Lectura_GPS(void*);

//Prototipo de funciones
static void Read_receptor1(void*);
static void Read_anemometro(void*);
static void Read_sensores_quedan(void*);
static void Send_telemetria(void*);
static void Control(void*);
static void control_luminario(void*);

//SERVO
void moveServo(int pin, int angulo);

void initSensors();


FLASHMEM __attribute__((noinline)) void setup() {
    
    //INICIAR SENSORE
    initSensors();
    
    dataSERVOS = xQueueCreate(20, sizeof(ReceptorDATA));

    SensorQueue = xQueueCreate(5, sizeof(SensorsDATA));

    GPSQueue = xQueueCreate(5, sizeof(gpsDATA));

    servos_angle = xQueueCreate(5, sizeof(ReceptorDATA));
    
    /*xTaskCreate( TaskFunction_t pxTaskCode,
                            const char * const pcName, /*lint !e971 Unqualified char types are allowed for strings and single characters only. *//*
                            const configSTACK_DEPTH_TYPE usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
                            TaskHandle_t * const pxCreatedTask )
    */
    xTaskCreate(Read_receptor, 
              "Read_receptor", 
              8192, 
              nullptr, 
              1, 
              nullptr);
             
    xTaskCreate(Move_servos, 
              "Move_servos", 
              2048, 
              nullptr, 
              1, 
              nullptr);
              
    xTaskCreate(Lectura_IMU, 
              "Lectura_IMU", 
              2048, 
              nullptr, 
              2, 
              nullptr);
              
    xTaskCreate(Lectura_GPS, 
              "Lectura_GPS", 
              4096, 
              nullptr, 
              2, 
              nullptr);
              
    xTaskCreate(Read_sensors, 
              "Read_sensors", 
              4096, 
              nullptr, 
              2, 
              nullptr);
    vTaskStartScheduler();   
}

void loop() {
}

void initSensors(){
    //Monitor Serial
    Serial.begin(115200);

    //Pin de pruebas
    pinMode(13, arduino::OUTPUT);
    digitalWriteFast(13,arduino::HIGH);

    //I2C Comunicacion
    Wire.begin();
    Wire1.begin();
    Wire2.begin();

    //IMU Inicializacion
    //---------------------------IMU------------------------------
    Serial.println("Orientation IMU Test"); Serial.println("");
    /* Initialise the sensor */
    if (!bno.begin()){
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    Serial.println("Orientation IMU DONE"); Serial.println("");
    delay(1000);
    //------------------------------------------------------------}


    //GPS Inicializacion
    if (myGNSS.begin(Wire1) == false) //Connect to the u-blox module using Wire port
      {
        Serial.println("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.");
        while (1);
      }
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    myGNSS.setNavigationFrequency(40);
    //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
    //------------------------------------------------------------}



    //SERIAL XBEE
    Serial2.begin(9600);
    Serial.println("Iniciando Sistema de telemetria");
    while (!Serial2) delay(10);  // wait for serial port to open!

    //SERVOS Inicializacion
    Servo_VELA.attach(PinServoVela, 500, 2500); // NO TOCAR
    Servo_VELA.write(90);
    Servo_TIMON.attach(PinServoTimon, 500, 2500); // NO TOCAR

    //Inicializacion pines receptor
    pinMode(RCPinVela,arduino::INPUT);
    pinMode(RCPinTimon,arduino::INPUT);
    pinMode(RCPinSwitcheo,arduino::INPUT);
      
    //------------------DHT22---------------------------
    dht.begin();
    //--------------------------------------------------

}



static void Read_receptor(void*) {
    ReceptorDATA dataToSend; //DATA QUEUE

    dataToSend.time_vela = pulseIn(RCPinVela,arduino::HIGH); //VELA
    dataToSend.time_timon = pulseIn(RCPinTimon,arduino::HIGH); //TIMON
    
    while(!(dataToSend.time_vela > 960 && dataToSend.time_vela < 2000 )){
      dataToSend.time_vela = pulseIn(RCPinVela,arduino::HIGH); //VELA
      Serial.println("ME QUEDO ACA 1 " + String(dataToSend.time_vela));
    }
  
    while(!(dataToSend.time_timon > 960 && dataToSend.time_timon < 2000 )){
      dataToSend.time_timon = pulseIn(RCPinVela,arduino::HIGH); //TIMON
      Serial.println("ME QUEDO ACA 2 " + String(dataToSend.time_timon));
    }
    
    vela_anterior = dataToSend.time_vela; //GUARDAR VALOR INICIAL
    timon_anterior = dataToSend.time_timon; //GUARDAR VALOR INICIAL

    
    while (true) {
        //Llenamos la estructura ReceptorDATA
        dataToSend.time_vela = pulseIn(RCPinVela,arduino::HIGH); //VELA
        dataToSend.time_timon = pulseIn(RCPinTimon,arduino::HIGH); //TIMON
      
        //Serial.print(String(dataToSend.time_vela) + "," + String(dataToSend.time_timon) + "," + String(vela_anterior) + "," + String(timon_anterior) );

        int tolerancia = 400;
              
        if(dataToSend.time_vela > 960 && dataToSend.time_vela < 2000  &&  dataToSend.time_vela > (vela_anterior-tolerancia) &&  dataToSend.time_vela < (vela_anterior+tolerancia)
            && dataToSend.time_timon > 960 && dataToSend.time_timon < 2000  &&  dataToSend.time_timon > (timon_anterior-tolerancia) &&  dataToSend.time_timon < (timon_anterior+tolerancia)){

            dataToSend.angulo_vela = map(dataToSend.time_vela,960, 2000, 0, 180); 
            dataToSend.angulo_timon = map(dataToSend.time_timon,960, 2000, 0, 180);

            vela_anterior = dataToSend.time_vela;
            timon_anterior = dataToSend.time_timon;

            //Enviamos al task de recibir datos de sensores
            xQueueSend(dataSERVOS, &dataToSend, pdMS_TO_TICKS(100));
            
            //Enviamos al task de mover servos
            xQueueSend(servos_angle, &dataToSend, pdMS_TO_TICKS(100));

            //Serial.print(" Angulos: " + String(dataToSend.angulo_vela) + "," + String(dataToSend.angulo_timon)  );
         }

        //Serial.println("");
        
        digitalWrite(arduino::LED_BUILTIN, arduino::LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
        
        digitalWrite(arduino::LED_BUILTIN, arduino::HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));

    }
}


static void Move_servos(void*) {
    //Structura para recibir data
    ReceptorDATA receivedData;

    int max16 = 2500;
    int min16 = 500;
    
    //SERVOS Inicializacion    
    Servo_VELA.attach(PinServoVela, 500, 2500); // NO TOCAR
    Servo_VELA.write(90);
    Servo_TIMON.attach(PinServoTimon, 500, 2500); // NO TOCAR
    
    while (true) {
      if(xQueueReceive(servos_angle, &receivedData, pdMS_TO_TICKS(100))){

        int angulo_VELA = receivedData.angulo_vela;
        int angulo_TIMON = receivedData.angulo_timon ;
        
        //moveServo(PinServoVela, angulo_VELA);  
        //moveServo(PinServoTimon, angulo_TIMON);  
        
        Servo_VELA.write(angulo_VELA);
        Servo_TIMON.write(angulo_TIMON);
        
        //Serial.println(" GA: " + String(receivedData.angulo_vela) + "," + String(receivedData.angulo_timon));
      }
    }
}

void moveServo(int pin, int angleArg){
  
  if (angleArg < 0) angleArg = 0;
  if (angleArg > 180) angleArg = 180;
  int angle = angleArg;
  int max16 = 2500;
  int min16 = 500;
  uint32_t us = ((( (max16 >> 4) - (min16>>4) ) * (46603) * angle) >> 11) + (min16 << 12); // us*256
  uint32_t duty = (us * 3355) >> 22;

  //#if TEENSYDUINO >= 137
  
  /*
  noInterrupts();
  uint32_t oldres = analogWriteResolution(12);
  analogWrite(pin, duty);
  analogWriteResolution(oldres);
  interrupts(); 
  */
  
  //#else

  noInterrupts();
  analogWriteFrequency(pin, 50);
  uint32_t oldres = analogWriteResolution(12);
  analogWrite(pin, duty);
  analogWriteResolution(oldres);
  interrupts();
  

  /*
  analogWriteResolution(12);
  analogWrite(pin, duty);
  */
}

static void Lectura_IMU(void*) {

    //SENSORS struct
    SensorsDATA sensorsREAD;
    gpsDATA gpsRECIEVE;
    ReceptorDATA sailAngles;
  
    //SERIAL XBEE
    Serial2.begin(9600);
    Serial.println("Iniciando Sistema de telemetria");
    while (!Serial2) delay(10);  // wait for serial port to open!

    
    while (true) {

      if( xQueueReceive(SensorQueue, &sensorsREAD, pdMS_TO_TICKS(100)) ){
         humedad = sensorsREAD.humedad;
         temperatura = sensorsREAD.temperatura;
      }

      if( xQueueReceive(GPSQueue, &gpsRECIEVE, pdMS_TO_TICKS(100)) ){
        c_latitude = gpsRECIEVE.latitude;
        c_longitude = gpsRECIEVE.longitude;
      }

      if( xQueueReceive(dataSERVOS, &sailAngles, pdMS_TO_TICKS(100)) ){
        sail_angle = sailAngles.angulo_vela;
        rudder_angle = sailAngles.angulo_timon;
      }
      
      //-----------codigo IMU-----------------
    
      sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData; 
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
      
      ax = accelerometerData.acceleration.x;
      ay = accelerometerData.acceleration.y;
      az = accelerometerData.acceleration.z;
      
      linearx = linearAccelData.acceleration.x;
      lineary = linearAccelData.acceleration.y;
      linearz = linearAccelData.acceleration.z;
    
      vgx = angVelocityData.gyro.x;
      vgy = angVelocityData.gyro.y;
      vgz = angVelocityData.gyro.z;//-------------yaw rate//vgz
      
      if(vgz==0){
        vgz=0.001;
      }
    
      mx = magnetometerData.magnetic.x;
      my = magnetometerData.magnetic.y;
      mz = magnetometerData.magnetic.z;
    
      yaw = orientationData.orientation.x;//-----------------
      pitch = orientationData.orientation.y;//--------------------
      roll = orientationData.orientation.z;//----------------

      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      
      dataTELEMETRY = String(yaw) + "," + String(pitch) + "," + String(roll) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(c_latitude) + "," + String(c_longitude) + "," + String(0)+ "," + String(0);
      dataTELEMETRY += "," + String(sail_angle) + "," + String(rudder_angle) + "," + String(0) + "," + String(0,3) + "," + String(bearing_setpoint);
      dataTELEMETRY += "," + String(humedad) + "," + String(temperatura) + "," + String(millis());
      
      if(Serial2.available()>0){ //SE ENVIA POR EL XBEE
       if(Serial2.read() == 'E'){
          Serial2.print(dataTELEMETRY);
       }
      }

      Serial.println(dataTELEMETRY);
      
      vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void Lectura_GPS(void*) {

    //Structura para el GPS
    gpsDATA gpsSend;
 
    while (true) { 
       long latitude = myGNSS.getLatitude();
       long longitude = myGNSS.getLongitude();
       long altitude = myGNSS.getAltitude();
       byte SIV = myGNSS.getSIV();
       
       gpsSend.latitude = latitude;
       gpsSend.longitude = longitude;
       gpsSend.altitude = altitude;
       gpsSend.SIV = SIV;

        //Enviamos la Queue
        
       xQueueSend(GPSQueue, &gpsSend, pdMS_TO_TICKS(100));
       
       //Serial.println(String(latitude) + "," + String(longitude) + "," + String(SIV)); 
        
       vTaskDelay(pdMS_TO_TICKS(100));
    }

 }

static void Read_sensors(void*){

  SensorsDATA sensorsSEND;
  
  while(true){
    //--------------------DHT22-------------------------
    humedad = dht.readHumidity(); //Leemos la Humedad
    temperatura = dht.readTemperature(); //Leemos la temperatura en grados Celsius
    //-------------------------------------------------

    sensorsSEND.humedad = humedad;
    sensorsSEND.temperatura =temperatura;
    
    xQueueSend(SensorQueue, &sensorsSEND, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(100));
  
  }
  
}
