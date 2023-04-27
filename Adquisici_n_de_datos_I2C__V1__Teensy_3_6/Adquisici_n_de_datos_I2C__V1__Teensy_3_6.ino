#include <stdbool.h>

//------------COMUNIACIÓN POR I2C--------------

#include <Wire.h>
//DATA I2C
byte DATAOUT[4] = {};

//---------COMUNICATION------------
#define device 97
byte dataIn[4] = {
};
int command = 0, i = 0;
//---------------------------------


//----------------------------------------------


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


//-------ADSWeather--------------
#include <ADSWeather.h>

#define windSpdPin 35
#define windDirPin 36
#define RAIN_PIN 6 

float windDir; 
long windSpeed;
String velocidadTOTAL = "";

ADSWeather ws1(RAIN_PIN , windDirPin, windSpdPin); //This should configure all pins correctly
//---------------------------------


int data_angulo = 0, estado_serial = 0;
String vela_angulo ="", timon_angulo = "";

void readAngles_serial();


//------SD CARD---------
#include <SD.h>
#include <SPI.h>

File myFile;
//----------------------



//--------------GPS--------------
#include "DHT.h"
#define DHTPIN 39     // Pin donde está conectado el sensor
#define DHTTYPE DHT22   // Sensor DHT22

DHT dht(DHTPIN, DHTTYPE);

float humedad, temperatura;
//-------------------------------


//--------------VARIABLES--------------------
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
long t0=millis();
float velN=0, velE=0, gvel=0;
float stdvel=0; //error de velocidad Acc
byte SIV;
double ax, ay, az, linearx, lineary, linearz, vgx, vgy, vgz, mx, my, mz, yaw, pitch, roll;
float shuntvoltage, busvoltage, current_mA, loadvoltage;
long t, latitude, longitude, altitude, hora, minutos, milisegundos;


int angulo_vela_i2c, angulo_timon_i2c;


int ledAZUL = 30;
bool stateLEDAZUL = false;
//---------------------------------------------




//-------------STATE MACHINE--------------------
enum STATE_MACHINE_VELERO{
  STATE_READ_SENSORS,
  STATE_MAKE_DATA,
  STATE_STORE_DATA_SD,
  STATE_SEND_TELEMETRY_DATA,
  STATE_RESET_GPS,
  STATE_SEND_ANGLES_RC_SWITCHER
}ActualState;

void stateMACHINE();

//------------------------------------------

String dataSD = "", dataTELEMETRY = "";


void setup()
{
  //SERIAL PORT
  Serial.begin(115200);
  Serial.println("Iniciando Sistema de adquisicion de datos");

  //SERIAL XBEE
  Serial2.begin(9600);

  //SERIAL SWITCHEO PCB
  Serial4.begin(9600);
  
  
  Serial.println("Iniciando Sistema de telemetria");
  while (!Serial2) delay(10);  // wait for serial port to open!
  
  Wire.begin();
  Wire1.begin();
  Wire2.begin();
  
  //---------------------------IMU------------------------------
  Serial.println("Orientation IMU Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin()){
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  Serial.println("Orientation IMU DONE"); Serial.println("");
  delay(1000);
  //------------------------------------------------------------

  pinMode(ledAZUL, OUTPUT);
  digitalWrite(ledAZUL, stateLEDAZUL);
  
  //---------------------------GPS------------------------------
  pinMode(RST_GPS, OUTPUT);
  pinMode(SAFE_GPS, OUTPUT);

  digitalWrite(RST_GPS, HIGH);
  digitalWrite(SAFE_GPS, HIGH);

  
  if (myGNSS.begin(Wire1) == false){
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    //while (1);
  }
  Serial.println("GPS DONE"); Serial.println("");
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
  myGNSS.setNavigationFrequency(40);
  //------------------------------------------------------------



  //-------------------------SD_CARD----------------------------
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  //------------------------------------------------------------



  //------------------DHT22---------------------------
  dht.begin();
  //--------------------------------------------------
  
  Wire1.setClock(200000);
  Wire.setClock(200000);
  
  
  delay(1000); 

  SIV = myGNSS.getSIV();
  while(SIV < 6){
    SIV = myGNSS.getSIV();
  }
  
  ActualState = STATE_READ_SENSORS; //INITIAL STATE

  Serial.println("Ya estoy listo");
  delay(1000);


  //-------ADSWeather--------------
  attachInterrupt(digitalPinToInterrupt(windSpdPin), ws1.countAnemometer, FALLING); //ws1.countAnemometer is the ISR for the anemometer.
  interrupts();
  //---------------------------------



  //----------------------I2C Slave-Master------------------------
  Wire2.begin(97);
  Wire2.onRequest(handleRequest);
  Wire2.onReceive(handleReceive);
  //---------------------------------------------------
}
  
void loop(){
  
 switch(ActualState){
  case STATE_READ_SENSORS:{
      //----------codigo GPS----------------
      lastTime=millis();
      t=lastTime-t0;
      latitude = myGNSS.getLatitude();//Serial.print(F(" (degrees * 10^-7)"));
      longitude = myGNSS.getLongitude();//Serial.print(F(" (degrees * 10^-7)"));
      altitude = myGNSS.getAltitude();//Serial.print(F(" (mm)")); 

      hora = myGNSS.getHour();
      minutos = myGNSS.getMinute();
      milisegundos = myGNSS.getMillisecond();
      
      SIV = myGNSS.getSIV();
    
      
      if (myGNSS.getNAVVELNED()){
        velN=(float)myGNSS.packetUBXNAVVELNED->data.velN/100.0;//m/s
        velE=(float)myGNSS.packetUBXNAVVELNED->data.velE/100.0;
        gvel=(float)myGNSS.packetUBXNAVVELNED->data.gSpeed/100.0;
        stdvel=(float)myGNSS.packetUBXNAVVELNED->data.sAcc/100.0;
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
    
      if(current_mA == 0){
        stateLEDAZUL = !stateLEDAZUL;
        digitalWrite(ledAZUL, stateLEDAZUL);
      }

      //--------------------DHT22-------------------------
      humedad = dht.readHumidity(); //Leemos la Humedad
      temperatura = dht.readTemperature(); //Leemos la temperatura en grados Celsius
      //-------------------------------------------------

      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);

      ws1.update();
      windSpeed = ws1.getWindSpeed();
      windDir = get_wind_direction(); 

      velocidadTOTAL = String(windSpeed / 10) + "." + String(windSpeed % 10);
    
      //----------------------I2C Comunication----------------------
      Wire2.requestFrom(device, 4);
      while(Wire2.available()){    // slave may send less than requested
        byte c = Wire2.read();
        dataIn[i] = c; 
        i ++;}
      i = 0;
    
      angulo_vela_i2c = dataIn[0];
      angulo_timon_i2c = dataIn[1];
      current_mA = dataIn[2];
      loadvoltage = dataIn[3];

      
      //CHANGE_STATE
      if(SIV < 5){
        ActualState = STATE_RESET_GPS; //ESPERO
      }else{
        ActualState = STATE_MAKE_DATA;
      }
    }break;
       
   case STATE_MAKE_DATA:{
      //-----SD DATA---------------
      dataSD = String(t) + "," + String(hora) + ":" + String(minutos) + ":" + String(milisegundos) + "," + String(latitude) + "," + String(longitude) + "," + String(altitude) + "," + String(velN,3) ;
      dataSD += "," + String(velE,3 ) + "," + String(gvel,3 ) + "," + String(roll)  + "," + String(pitch)+ "," + String(yaw);
      dataSD += "," + String(vgz,4) + "," + String(stdvel) + "," + String(SIV);
      dataSD += "," + String(current_mA,3)+ "," + String(loadvoltage);
      dataSD += "," + String(humedad) + "," + String(temperatura);
      dataSD += "," + String(angulo_vela_i2c) + "," + String(angulo_timon_i2c) + ",";
      dataSD += velocidadTOTAL + "," + String(windDir,3) + String("\n");
      //--------TELEMETRY_DATA---------
      dataTELEMETRY = String(yaw) + "," + String(pitch) + "," + String(roll) + "," + String(ax) + "," + String(ay) + "," + String(az) + "," + String(latitude) + "," + String(longitude) + "," + String(current_mA)+ "," + String(loadvoltage);
      dataTELEMETRY += "," + String(angulo_vela_i2c) + "," + String(angulo_timon_i2c) + "," + velocidadTOTAL + "," + String(windDir,3);
      //CHANGE_STATE
      Serial.print(dataSD);
      
      ActualState = STATE_STORE_DATA_SD;
   }break;
  
   case STATE_STORE_DATA_SD:{

      // open the file. 
      myFile = SD.open("data3.txt", FILE_WRITE);
      if (myFile) {
        myFile.print(dataSD);
      }else{
        Serial.println("error opening test.txt");
      }
      dataSD = "";
      
      //CHANGE_STATE
      ActualState = STATE_SEND_TELEMETRY_DATA;
   }break;

   case STATE_SEND_TELEMETRY_DATA:{

      if(Serial2.available()>0){
       if(Serial2.read() == 'E'){
          Serial2.print(dataTELEMETRY);
       }
      }
      //CHANGE_STATE
      ActualState = STATE_SEND_ANGLES_RC_SWITCHER;
   }break;

   case STATE_SEND_ANGLES_RC_SWITCHER:{
       DATAOUT[0] = 10; //VELA
       DATAOUT[1] = 100; //TIMON
       ActualState = STATE_READ_SENSORS;
   } break;
    
   case STATE_RESET_GPS:{
      digitalWrite(RST_GPS, LOW); 
      delay(800);
      digitalWrite(RST_GPS, HIGH);  //RESETEO
      
      Serial.println("ME QUEDO EN EL ESTADO DE RESETEAR GPS");
      delay(1000);

      
      SIV = myGNSS.getSIV();
      while(SIV < 6){
        SIV = myGNSS.getSIV();
      }
      
      ActualState = STATE_READ_SENSORS;
   }break;

   
 }
}


void handleReceive(int howMany)
{
  int i = 0;
}

void handleRequest()
{
  Wire2.write(DATAOUT, 2);
}


int get_wind_direction()
{
    unsigned int adc;
    adc = analogRead(windDirPin); // get the current reading from the sensor
    if (adc < 380) return (113);
    if (adc < 393) return (68);
    if (adc < 414) return (90);
    if (adc < 456) return (158);
    if (adc < 508) return (135);
    if (adc < 551) return (203);
    if (adc < 615) return (180);
    if (adc < 680) return (23);
    if (adc < 746) return (45);
    if (adc < 801) return (248);
    if (adc < 833) return (225);
    if (adc < 878) return (338);
    if (adc < 913) return (0);
    if (adc < 940) return (293);
    if (adc < 967) return (315);
    if (adc < 990) return (270);
    return (-1); // error, disconnected?
}
