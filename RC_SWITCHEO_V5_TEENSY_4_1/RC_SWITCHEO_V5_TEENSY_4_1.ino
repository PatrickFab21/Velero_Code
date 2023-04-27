//------------COMUNIACIÓN POR I2C--------------

#include <Wire.h>
//DATA I2C
byte DATAOUT[4] = {};

//---------COMUNICATION------------
#define device 97
byte dataIn[2] = {
};
int command = 0, i = 0;
//---------------------------------



//-------INA219-----------
#include <Adafruit_INA219.h>
Adafruit_INA219 ina219;

float shuntvoltage, busvoltage, current_mA, loadvoltage;
//------------------------


#include "Kalman.h"
SensorKalman vela_kalman, timon_kalman;


//-------SERVO------------
#include <PWMServo.h>

PWMServo Servo_VELA;  // create servo object to control a servo
PWMServo Servo_TIMON;
//-------SERVO------------


//-------------STATE MACHINE--------------------
enum STATE_MACHINE_RC_VELERO{
  STATE_READ_VALUE_SWITCHEO,
  STATE_CHANGE_VALUES_MANUAL,
  STATE_DETACH,
  STATE_ATTACH,
  STATE_CHANGE_VALUES_AUTO
}ActualState;


//------------------------------------------


//PINES PARA SERVOS
int PinServoVela = 8, PinServoTimon = 10;

//PINES PARA RC
int RCPinVela = 7, RCPinTimon = 4;
int RCPinSwitcheo = 6;

//VARIABLES PARA ANGULOS POR SERIAL
int vela, timon, filterVELA, filterTIMON;
int swticheo_anterior, vela_anterior, timon_anterior;


//ESTADOS
int estado1 = 0, estado2 = 0, estado3 = 0;
int estado_switcheo = 0, estado_serial = 0;

//TIEMPOS
float tiempo1 = 0, tiempo2 = 0, tiempo3 = 0;
double dt = 0;
uint32_t timer = 0;

void setup() {

  
  //--------------------------INA219-----------------------------
  if (! ina219.begin(&Wire2)) {
    Serial.println("Failed to find INA219 chip");}
  Serial.println("INA219 Inizialized");
  //------------------------------------------------------------

  
  pinMode(RCPinVela,INPUT);
  pinMode(RCPinTimon,INPUT);
  pinMode(RCPinSwitcheo,INPUT);

  Servo_VELA.attach(PinServoVela, 900, 2250); // NO TOCAR
  Servo_VELA.write(90);
  Servo_TIMON.attach(PinServoTimon, 500, 2500); // NO TOCAR

  vela_kalman.setDistance( pulseIn(RCPinVela,HIGH) ); 
  timon_kalman.setDistance( pulseIn(RCPinTimon,HIGH) );
  
  ActualState = STATE_READ_VALUE_SWITCHEO;
  timer = micros();

  Serial.println("LEER INICIALES");
  
  tiempo2 = pulseIn(RCPinVela,HIGH); //VELA
  tiempo3 = pulseIn(RCPinTimon,HIGH); //TIMON
  
  while(!(tiempo2 > 960 && tiempo2 < 2000 )){
    tiempo2 = pulseIn(RCPinVela,HIGH); //VELA
    Serial.println("ME QUEDO ACA 1 " + String(tiempo2));
  }

  while(!(tiempo3 > 960 && tiempo3 < 2000 )){
    tiempo3 = pulseIn(RCPinVela,HIGH); //TIMON
    Serial.println("ME QUEDO ACA 2 " + String(tiempo3));
  }
  
  vela_anterior = tiempo2; //GUARDAR VALOR INICIAL
  timon_anterior = tiempo3; //GUARDAR VALOR INICIAL

  //----------------------I2C Slave------------------------
  Wire.begin(97);
  Wire.onRequest(handleRequest);
  Wire.onReceive(handleReceive);
  //---------------------------------------------------
  timer = micros();
}

void loop() {

  switch(ActualState){

    case STATE_READ_VALUE_SWITCHEO:{
      tiempo2 = pulseIn(RCPinVela,HIGH); //VELA
      tiempo3 = pulseIn(RCPinTimon,HIGH); //TIMON
        
      
      //--------------------INA219-----------------------
      shuntvoltage = ina219.getShuntVoltage_mV();
      busvoltage = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
      loadvoltage = busvoltage + (shuntvoltage / 1000);
      //-------------------------------------------------

      DATAOUT[2] = current_mA;
      DATAOUT[3] = loadvoltage;
      
      
      if(tiempo2 > 960 && tiempo2 < 2000  &&  tiempo2 > (vela_anterior-250) &&  tiempo2 < (vela_anterior+250)
        && tiempo3 > 960 && tiempo3 < 2000  &&  tiempo3 > (timon_anterior-250) &&  tiempo3 < (timon_anterior+250)){
        vela = map(tiempo2,960, 2000, 0, 180); 
        timon = map(tiempo3,960, 2000, 0, 180);

        dt = (double)(micros() - timer)/1000000;
        timer = micros();
        filterVELA = vela_kalman.getDistance(vela, dt);
        filterTIMON = timon_kalman.getDistance(timon, dt);
          
        Servo_VELA.write(filterVELA);
        Servo_TIMON.write(filterTIMON);        
        //String(tiempo1) + "," + String(tiempo2) + "," + String(tiempo3) + "," +
        //String(vela) + "," + String(timon)
        Serial.println( String(vela) + "," + String(timon) ); 
        
        vela_anterior = tiempo2;
        timon_anterior = tiempo3;

        DATAOUT[0] = vela;
        DATAOUT[1] = timon;
        
        
        ActualState = STATE_READ_VALUE_SWITCHEO;     
      }else{
        ActualState = STATE_READ_VALUE_SWITCHEO;
      }
    
    } break;
  
    case STATE_CHANGE_VALUES_AUTO:{
        Wire.requestFrom(device, 2);
        while(Wire.available()){    // slave may send less than requested
          byte c = Wire.read();
          dataIn[i] = c; 
        i ++;}
      i = 0;  

      vela = dataIn[0];
      timon = dataIn[1];
      
      Servo_VELA.write(vela);
      Servo_TIMON.write(timon); 

      ActualState = STATE_CHANGE_VALUES_AUTO;
    }break;

  }  
}

void handleReceive(int howMany)
{
  int i = 0;
}

void handleRequest()
{
  Wire.write(DATAOUT, 4);
}
