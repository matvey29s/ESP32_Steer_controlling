#include <Wire.h>
#include "MAX30105.h"
#include "Pulse.h"
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_adc_cal.h"

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

const char* ssid = "Silvester";  //Название Wi-Fi сети
const char* password = "12345678";  //Пароль 

const char * udpAddress = "192.168.4.2"; // IP адрес сервера
unsigned int udpPort=4210; // Порт UDP
unsigned int localPort=48700; // Порт UDP

WiFiUDP udp;
WiFiUDP udp1;


MAX30105 particleSensor;
MAX30105 particleSensor1;

Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;

Pulse pulseIR_1;
Pulse pulseRed_1;
MAFilter bpm_1;

#define MOTOR 18 //GPIO выход на вибромотор

#define MAX_BRIGHTNESS 255
#define OPTIONS 7
#define SAMPLES_ANALOG 10

uint16_t sum_analog[4]={0,0,0,0};
uint16_t analog_result[4]; //Массив данных с АЦП 
uint8_t transmit_data[10]; //Пакет отправки данных
uint8_t transmit_data_1[6]; //Пакет отправки данных
uint8_t receiveBuffer[1]; //Прием данных


static const uint8_t heart_bits[] PROGMEM = { 0x00, 0x00, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe, 0xfe, 0xff, 
                                        0xfe, 0xff, 0xfc, 0x7f, 0xf8, 0x3f, 0xf0, 0x1f, 0xe0, 0x0f,
                                        0xc0, 0x07, 0x80, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00 };

//spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t spo2_table[184] PROGMEM =
        { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
          99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
          100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
          97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
          90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84,  84, 83, 82, 82, 81, 81, 
          80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
          66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
          49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
          28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
          3, 2, 1 } ;

long lastBeat = 0;    //Time of the last beat 
long displaytime = 0; //Time of the last display update

long lastBeat_1 = 0;    //Time of the last beat 
long displaytime_1 = 0; //Time of the last display update

int  beatAvg;
int  SPO2, SPO2f;
int  voltage;
bool filter_for_graph = false;
bool draw_Red = false;
uint8_t pcflag =0;
uint8_t istate = 0;
uint8_t sleep_counter = 0;

int  beatAvg_1;
int  SPO2_1, SPO2f_1;
int  voltage_1;
bool filter_for_graph_1 = false;
bool draw_Red_1 = false;
uint8_t pcflag_1 =0;
uint8_t istate_1 = 0;
uint8_t sleep_counter_1 = 0;

void setup()
{
  Serial.begin(115200);
  /*WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(100);
  }*/
  WiFi.softAP(ssid, password);
  delay(100);
  Wire1.setPins(17,16); // Объявляем 2ой I2C
  Wire1.begin();
  pinMode(MOTOR, OUTPUT);
  analogReadResolution(12); //Разрядность АЦП
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //I2C шина 1
  {
    Serial.write("1 sensor");
    while (1);
  }
  if (!particleSensor1.begin(Wire1, I2C_SPEED_FAST)) //I2C шина 2
  {
    Serial.write("2 sensor");
    while (1);
  }
  Serial.print("BEFORE SENSORS");
  xTaskCreatePinnedToCore(
                    Task1code,   /* Функция задачи. */
                    "Task1",     /* Ее имя. */
                    20000,       /* Размер стека функции */
                    NULL,        /* Параметры */
                    1,           /* Приоритет */
                    &Task1,      /* Дескриптор задачи для отслеживания */
                    1);          /* Указываем пин для данного ядра */                  
  delay(100);
  xTaskCreatePinnedToCore(
                    Task2code,   /* Функция задачи. */
                    "Task2",     /* Ее имя. */
                    5000,       /* Размер стека функции */
                    NULL,        /* Параметры */
                    1,           /* Приоритет */
                    &Task2,      /* Дескриптор задачи для отслеживания */
                    0);          /* Указываем пин для данного ядра */                  
  delay(100);
  xTaskCreatePinnedToCore(
                    Task3code,   /* Функция задачи. */
                    "Task3",     /* Ее имя. */
                    80000,       /* Размер стека функции */
                    NULL,        /* Параметры */
                    1,           /* Приоритет */
                    &Task3,      /* Дескриптор задачи для отслеживания */
                    0);          /* Указываем пин для данного ядра */                  
  delay(100); 



  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor1.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  udp.begin(udpPort);
  udp1.begin(localPort);
}

void Task1code( void * pvParameters ){
  for(;;){
    for(int i=0;5>i;i++) {
      sum_analog[0] += analogRead(32);
      sum_analog[1] += analogRead(33);
      sum_analog[2] += analogRead(34);
      sum_analog[3] += analogRead(35);
      delay(2);
    }
    analog_result[0]=sum_analog[0]/5;
    analog_result[1]=sum_analog[1]/5;
    analog_result[2]=sum_analog[2]/5;
    analog_result[3]=sum_analog[3]/5;

    transmit_data[1]=(analog_result[0] >> 8) & 0xFF;
    transmit_data[2]=analog_result[0] & 0xFF;
    transmit_data[3]=(analog_result[1] >> 8) & 0xFF;
    transmit_data[4]=analog_result[1] & 0xFF;
    transmit_data[5]=(analog_result[2] >> 8) & 0xFF;
    transmit_data[6]=analog_result[2] & 0xFF;
    transmit_data[7]=(analog_result[3] >> 8) & 0xFF;
    transmit_data[8]=analog_result[3] & 0xFF;
    
    memset(transmit_data,(uint8_t)160,1);
    memset(transmit_data+9,(uint8_t)254,1);

    udp1.beginPacket(udpAddress, localPort);
    udp1.write(transmit_data,sizeof(transmit_data));
    udp1.endPacket();


    memset(transmit_data, 0, sizeof(transmit_data));
    memset(sum_analog, 0, sizeof(sum_analog));
    memset(analog_result, 0, sizeof(analog_result));
    delay(90);
   
  } 
}
void Task2code( void * pvParameters ){
  for(;;){
    udp.parsePacket();
    if(udp.read(receiveBuffer, 1) > 0)
    {
          if (receiveBuffer[0]==50) 
            {
              digitalWrite(MOTOR, HIGH);
              delay(50);                 
              digitalWrite(MOTOR, LOW);
              delay(100); 
              digitalWrite(MOTOR, HIGH);
              delay(50);                 
              digitalWrite(MOTOR, LOW);
            }
          if (receiveBuffer[0]==100) 
            {
              digitalWrite(MOTOR, HIGH);
              delay(100);                 
              digitalWrite(MOTOR, LOW);
              delay(100);
              digitalWrite(MOTOR, HIGH);
              delay(100);                 
              digitalWrite(MOTOR, LOW);
              delay(100);
              digitalWrite(MOTOR, HIGH);
              delay(100);                 
              digitalWrite(MOTOR, LOW);
            }
          if (receiveBuffer[0]==150) 
            {
              digitalWrite(MOTOR, HIGH);
              delay(500);                 
              digitalWrite(MOTOR, LOW);
              delay(100); 
              digitalWrite(MOTOR, HIGH);
              delay(500);                 
              digitalWrite(MOTOR, LOW);
              delay(100); 
              digitalWrite(MOTOR, HIGH);
              delay(500);                 
              digitalWrite(MOTOR, LOW);
            }
    }
    delay(500);
    } 
}
void Task3code( void * pvParameters ){
  for(;;){
    particleSensor.check();
    long now = millis();
    long now_1 = millis();
    if (!particleSensor.available() && !particleSensor1.available() ) {
      delay(1);
    };
    uint32_t irValue = particleSensor.getIR(); 
    uint32_t redValue = particleSensor.getRed();
    particleSensor.nextSample();
    uint32_t irValue_1 = particleSensor1.getIR(); 
    uint32_t redValue_1 = particleSensor1.getRed();
    particleSensor1.nextSample();
    if (irValue<5000 and irValue_1<5000) {
        delay(100);
    } 
    else {
        int16_t IR_signal, Red_signal;
        bool beatRed, beatIR;
        int16_t IR_signal_1, Red_signal_1;
        bool beatRed_1, beatIR_1;
          if (irValue>5000) 
          {
            IR_signal =  pulseIR.ma_filter(pulseIR.dc_filter(irValue)) ;
            Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
            beatRed = pulseRed.isBeat(Red_signal);
            beatIR =  pulseIR.isBeat(IR_signal);

            if (draw_Red ? beatRed : beatIR){
            long btpm = 60000/(now - lastBeat);
            if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);
            lastBeat = now; 

            long numerator   = (pulseRed.avgAC() * pulseIR.avgDC())/256;
            long denominator = (pulseRed.avgDC() * pulseIR.avgAC())/256;
            int RX100 = (denominator>0) ? (numerator * 100)/denominator : 999;

            SPO2f = (10400 - RX100*17+50)/100;  

            if ((RX100>=0) && (RX100<184))
              SPO2 = pgm_read_byte_near(&spo2_table[RX100]);
            }
            if (beatAvg>50 && beatAvg<200 && SPO2>70) {
              memset(transmit_data_1+1,(uint8_t)beatAvg,1);
              memset(transmit_data_1+3,(uint8_t)SPO2,1);
            }
          }
          if(irValue_1>5000) 
          {
            IR_signal_1 =  pulseIR_1.ma_filter(pulseIR_1.dc_filter(irValue_1)) ;
            Red_signal_1 = pulseRed_1.ma_filter(pulseRed_1.dc_filter(redValue_1));
            beatRed_1 = pulseRed_1.isBeat(Red_signal_1);
            beatIR_1 =  pulseIR_1.isBeat(IR_signal_1);
            if (draw_Red_1 ? beatRed_1 : beatIR_1){
            long btpm_1 = 60000/(now_1 - lastBeat_1);
            if (btpm_1 > 0 && btpm_1 < 200) beatAvg_1 = bpm_1.filter((int16_t)btpm_1);
            lastBeat_1 = now_1; 

            long numerator_1   = (pulseRed_1.avgAC() * pulseIR_1.avgDC())/256;
            long denominator_1 = (pulseRed_1.avgDC() * pulseIR_1.avgAC())/256;
            int RX100_1 = (denominator_1>0) ? (numerator_1 * 100)/denominator_1 : 999;

            SPO2f_1 = (10400 - RX100_1*17+50)/100;  

            if ((RX100_1>=0) && (RX100_1<184))
              SPO2_1 = pgm_read_byte_near(&spo2_table[RX100_1]);
            }
            if (beatAvg_1>50 && beatAvg_1<200 && SPO2_1>70) {
              memset(transmit_data_1+4,(uint8_t)SPO2_1,1);
              memset(transmit_data_1+2,(uint8_t)beatAvg_1,1);
            }
          }

          memset(transmit_data_1,(uint8_t)80,1);
          memset(transmit_data_1+5,(uint8_t)152,1);

          udp1.beginPacket(udpAddress, localPort);
          udp1.write(transmit_data_1,sizeof(transmit_data_1));
          udp1.endPacket();

          memset(transmit_data_1, 0, sizeof(transmit_data_1));
        }
    delay(1);
    } 
}

void loop() {
    
}