



/*MQTT*/
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Time2.h>
#include "Timer.h"
#include <EEPROM.h>

#include <Wire.h>             //http://arduino.cc/en/Reference/Wire
#include <DS3232RTC.h> 


#include "RS485_protocol.h"
#include <SoftwareSerial.h>


SoftwareSerial rs485 (2, 3);  // receive pin, transmit pin


Timer t;


byte mac[]    = {  0x00, 0xaa, 0xbb, 0xcc, 0xde, 0x02 }; //MAC address of Ethernet shield, use arp -an to find by IP
byte server[] = { 192, 168, 1, 24 }; // IP Address of your MQTT Server. Where RasPi <nd Mosquitto is.
byte ip[]     = { 192, 168, 1, 202 }; // IP for this device. Arduino IP.
EthernetClient ethClient;
void callback(char* topic, byte* payload, unsigned int length);

PubSubClient client(server, 1883, callback, ethClient);
char message_buff[100];
char* clientId     = "<CLIENT-ID>";             // * set a random string (max 23 chars, will be the MQTT client id)
char* deviceId     = "<DEVICE-ID>";             // * set your device id (will be the MQTT client username)
char* deviceSecret = "<DEVICE-SECRET>";         // * set your device secret (will be the MQTT client password)
char* outTopic[]     = {
                         "irrigazione/timeRTC/1",      //comunica RTC time a OpenHab
                         "irrigazione/triggerTimeAck/1", //Ack. Dopo aver ricevuto l'ora in MQTT la ritrasmetto ad OpenHab per aver certezza 
                         "irrigazione/timeToRemain/1",   //Trasmette il tempo rimanente all'irragazione aggiornandolo alla frequenza UPDATEFREQUENCY
                         "irrigazione/status/1" ,         //Trasmette lo stato, attivo o spento dell'irrigazione.
                         "irrigazione/event",
                         
                      }; // * MQTT channel where physical updates are published
const char* inTopic[] = {"time/1",   //Serve a ricevere l'ora attuale aggiornata da internet
                        "irrigazione/triggerTime/1", //Riceve l'ora a cui parte l'irrigazione
                        "irrigazione/reset",
                       };
//-------------------------------------------------------------------------------

const char MODALITY_SELECTOR = 5;
const char SQW_PIN = 2;

unsigned int zona[] = {31,32,33, 34,35,36,37,38,39,40}; //pinout for relay output
unsigned int timerZona[] = {2,2,2,2,2,2,2,2,2,2}; // minuti per ogni zona
unsigned int NumZone = 10; //valore nel setup




/*int tm_min,tm_alarm_min,timeToIrrigation;
int anno, mese, giorno, ora, minuto, ora_allarme, minuto_allarme = 0;
String TimeToRemain;
unsigned int long tmp, count;
int tm_rtc_tmp;
*/

void(* Riavvia)(void) = 0;


void alarmIsr();
void startIrrigazione();
void tmPrint(tmElements_t tm);
void setupDS3231(ALARM_TYPES_t tipologiaAllarme, byte minuti, byte ore, byte giorno);
tmElements_t calculateDelay(int delayTime);
void mqttConnection();
void mqttSubscribe();
tmElements_t getMQTTdate(byte * payload,  unsigned int length);
void sendMQTTstring(String stringa, int topicnumber);
void sendMQTTdate(tmElements_t tm, int topicnumber);
void notifyTimeToStart();
void sendMQTTstatus();


void setupSmart(void);
void setupStupid();
void periodic();
void reset();
void notifyEndOfIrrigazione();
void setIrrigationDate(  tmElements_t tm );
tmElements_t getIrrigationDate();
tmElements_t getCurrentDate();
void rtcError();







void setup(void)
{
          Serial.begin(9600);
          Serial.println("IRRIGATION SETUP\n\n");
         
          for(int i=0; i< NumZone; i++) {
                  pinMode(zona[i], OUTPUT); //PULLUP??
                  digitalWrite(zona[i],LOW); 
                  delay(200);
           }
           
          //get mode, stupid or smart
           pinMode(MODALITY_SELECTOR, INPUT_PULLUP);
           //se a massa,modality=1 (automatico)
           if(digitalRead(MODALITY_SELECTOR)==HIGH){
              Serial.print(" - MODALITY SMART\n");
              setupSmart();
           }
           else{
            Serial.print(" - MODALITY STUPID\n");
            setupStupid();
                
           }
           
           
}
void setupSmart(void){   
          
           Ethernet.begin(mac, ip);
           mqttConnection();               
          
           tmElements_t tm_current_alarm =  getIrrigationDate();
    
            while(tm_current_alarm.Year  == 0 && tm_current_alarm.Month  == 0 && tm_current_alarm.Day  == 0 && tm_current_alarm.Hour  == 0 && tm_current_alarm.Minute  == 0 ){
                mqttConnection();      
            }
            Serial.print("Setup completed");
              
            int r = t.every(60000,  periodic , (void*)0);

}



void setupStupid(){

            startIrrigazione(); //così da farla partire subito, poi nel loop ogni 24 h si setta l'interrup          
            tmElements_t tm_alarm = calculateDelay(1440); //24h=24x60min
            Serial.print("RTC Time: ");
            tmPrint(getCurrentDate());
            Serial.print("Alarm Time: ");
            tmPrint(tm_alarm);
            setupDS3231(ALM2_MATCH_HOURS, tm_alarm.Minute, tm_alarm.Hour , 1);  

}
   


volatile boolean alarmIsrWasCalled = false;

void alarmIsr()
{
    alarmIsrWasCalled = true;
}

void loop(void)
{     
    
      if (alarmIsrWasCalled){

        if (RTC.alarm(ALARM_2)) {
            // setScenario(modality, scenario);
             Serial.println("Interrupt scattato a: ");
             tmPrint(getCurrentDate());
             /**
              * TODO: memorizzare lo storico delle irrigazioni qua.
              */
            
            startIrrigazione();
            notifyEndOfIrrigazione();
        }
        alarmIsrWasCalled = false; //TODO SPOSTA SOPRA??
    }
 
      mqttConnection();  
      t.update();
}

void periodic(){
   notifyTimeToStart();
}
 /**
 * IRRIGATION  FUNCTION
 */
 
void startIrrigazione(){
  Serial.println("\n START IRRIGAZIONE");
   for(int NumeroZona=0; NumeroZona<NumZone-1; NumeroZona++){
      unsigned int minutiPerZona=timerZona[NumeroZona];
      digitalWrite(zona[NumeroZona],HIGH); 
       
        for(int count = minutiPerZona; count > 1; count--){
          Serial.println("time to finish" + count);
          delay(60000); // 1 minuto   
        }
      
      digitalWrite(zona[NumeroZona],LOW);  
   }
      
}


/**
 * RTC FUNCTIONS
 */

void tmPrint(tmElements_t tm)
{
    // digital clock display of the time
    
    Serial.print(tm.Hour);
    Serial.print(":");
    Serial.print(tm.Minute);
    Serial.print(' ');
    Serial.print(tm.Day);
    Serial.print("/");
    Serial.print(tm.Month);
    Serial.print("/");
    Serial.print(tm.Year); 
    Serial.println(); 
}

 
void setupDS3231(ALARM_TYPES_t tipologiaAllarme, byte minuti, byte ore, byte giorno){ 
    Serial.println("Setup DS3231");
         /**
      * first control the i2c comunication with RTC and update clock.
      */  
     //setSyncProvider() causes the Time library to synchronize with the
    //external RTC by calling RTC.get() every five minutes by default.
    setSyncProvider(RTC.get);
   
    if (timeStatus() != timeSet){
        Serial.println(" RTC local sync FAIL!");
    }
    else{
       //Serial.println("RTC local Sync succed");
    }
   

   // printDateTime( RTC.get() );
   

    //Disable the default square wave of the SQW pin.
    RTC.squareWave(SQWAVE_NONE);

    //Attach an interrupt on the falling of the SQW pin.
    //digitalWrite(SQW_PIN, HIGH);    //redundant with the following line
    pinMode(SQW_PIN, INPUT_PULLUP);
    attachInterrupt(INT0, alarmIsr, FALLING);

    
    RTC.setAlarm(tipologiaAllarme, minuti, ore, giorno);    //daydate parameter should be between 1 and 7
    RTC.alarm(ALARM_2);                   //ensure RTC interrupt flag is cleared
    RTC.alarmInterrupt(ALARM_2, true);

    
}


tmElements_t calculateDelay(int delayTime){
        tmElements_t tm;

        tmElements_t tm_rtc = getCurrentDate();
        int month_length;
        month_length=32;//uno in più perché i giorni partono da 1 e non da zero.
        if(tm_rtc.Month==4 || tm_rtc.Month==6 || tm_rtc.Month==9 || tm_rtc.Month==11){
          month_length = 31;
        }
        if(tm_rtc.Month == 2){
          month_length = 29;
        }
        int var1=tm_rtc.Minute+(delayTime%60);
        tm.Minute=var1%60;
        int var2 = (tm_rtc.Hour+(delayTime/60)+var1/60);
        tm.Hour=var2%24;
        int var3=tm_rtc.Day + var2/24;
        tm.Day = var3%month_length+var3/month_length;//TODO  metti mounth lenght
        tm.Month = tm_rtc.Month+var3/month_length;
        tm.Year = tm_rtc.Year;
        return tm;      
}



/**
 * MQTT FUNCTIONS
 */

void mqttConnection() {
  // add reconnection logics
  if (!client.connected()) {
    // connection to MQTT server
    if (client.connect("ArduinoMQTTrele")) //clientId, deviceId, deviceSecret))
    {
      Serial.println("[PHYSICAL] Successfully connected with MQTT");
      mqttSubscribe(); // topic subscription
     
    }

  }

 
  
  client.loop();
}

void mqttSubscribe() {

  for (int i = 0; i < (sizeof(inTopic)/sizeof(int)); i++){
    client.subscribe(inTopic[i]);
    client.loop();
    Serial.print("subscribe: ");
    Serial.println(inTopic[i]);
  }
}
void mqttPublish(int i, char* messaggio) {
  // build the topic with the light item
   
    client.publish(outTopic[i], messaggio); 

}


tmElements_t getMQTTdate(byte * payload,  unsigned int length){

 
              tmElements_t tm;
              int intbuff[19]; //TODO FARLO VOLATILE?
              Serial.println("\nMESSAGE RECEIVED ON TOPIC TIME/1 SYNC ARDUINO TIMER WITH OPENHAB TIMER");
              int i;
              char stringbuff[19];
                  for(i=0; i<length; i++) {
                        intbuff[i] = (int)payload[i]-48; //convert ascii number in int.
                        stringbuff[i]=payload[i];
                      }  
                      stringbuff[i] = '\0';
                      String RTCtime = String(stringbuff);       
              //le variabili intere levale e lascia tm.Hour= (intbuff[8]...
          /*  int  anno =   ((intbuff[0])*1000+(intbuff[1])*100+(intbuff[2])*10+(intbuff[3]));
            int   mese =   (intbuff[5])*10+(intbuff[6]);
            int   giorno = (intbuff[8])*10+(intbuff[9]);
            int   ora =    (intbuff[11])*10+(intbuff[12]);
            int   minuto = (intbuff[14])*10+(intbuff[15]);*/

                  tm.Hour = (intbuff[11])*10+(intbuff[12]);           
                  tm.Minute = (intbuff[14])*10+(intbuff[15]);
                  tm.Day =  (intbuff[8])*10+(intbuff[9]);
                  tm.Month = (intbuff[5])*10+(intbuff[6]);
                  tm.Year =  ((intbuff[0])*1000+(intbuff[1])*100+(intbuff[2])*10+(intbuff[3]))-2000;  

                  return tm;
}
void sendMQTTstring(String stringa, int topicnumber){
            unsigned int str_len = stringa.length() +1;
             char cbuff[str_len];
             stringa.toCharArray(cbuff,str_len);

            mqttPublish(topicnumber, cbuff);
}
void sendMQTTdate(tmElements_t tm, int topicnumber){

    String stringa = String(tm.Hour)  + ":" + String(tm.Minute);
             unsigned int str_len = stringa.length() +1;
             char cbuff[str_len];
             stringa.toCharArray(cbuff,str_len);

            mqttPublish(topicnumber, cbuff);
}

void notifyTimeToStart(){
  tmElements_t tm_rtc = getCurrentDate();
 
  tmElements_t tm_alarm = getIrrigationDate();
  int timeToIrrigation;
  
  int currentTime =tm_rtc.Hour*60+tm_rtc.Minute;
  int startTime = tm_alarm.Hour*60+tm_alarm.Minute;
  
  if(currentTime <= startTime){
      timeToIrrigation = startTime - currentTime;
  }
  else{
    timeToIrrigation = (1440 - currentTime) + startTime;
  }
  String OreRemain = String(abs(timeToIrrigation/60));
  String MinutoRemain = String(abs((timeToIrrigation%60)));
  String TimeToRemain = OreRemain + ":" + MinutoRemain;
             
  sendMQTTstring(TimeToRemain,2);
                       
}
void  notifyEndOfIrrigazione(){
  
}



void callback(char* topic, byte* payload, unsigned int length) {
  
     if(strcmp(topic, inTopic[0])==0){
                   //TOPIC 0 è l'aggiornamento dell'ora corrente da internet.
                  //ricevuta la nuova ora da internet si controlla si è necessario sincronizzare il sistema rispetto a questo riferimento.                  
                    //1 leggere tempo dall'RTC, se non lo leggi segnali in mqtt malfunzionamento scrivendo ora impossibile tipo 25
                    //2 confronti l'ora RTC con ora del riferimento
                    //se differiscono allora setti l'ora di riferimento dentro al real time clok.
                    //se questo avviene in startup allora aggiorni lo scenario zero.
                tmElements_t tm = getMQTTdate(payload,length);
                tmElements_t tm_rtc = getCurrentDate();
            
                if(tm.Hour != tm_rtc.Hour || tm.Minute != tm_rtc.Minute || tm.Year != tm_rtc.Year){
                      Serial.print("Re-sync RTC clock with remote NTP clock");
                       RTC.write(tm); 
                       
                       
                }

                    
      }
        
      if(strcmp(topic, inTopic[1])==0){
      //TPOIC 1 è l'ora dell'allarme quando deve scattare
      
            tmElements_t tm_alarm = getMQTTdate(payload,length);
            
            setupDS3231(ALM2_MATCH_HOURS, tm_alarm.Minute, tm_alarm.Hour, 1);

           sendMQTTdate(tm_alarm,1);
           
           setIrrigationDate(tm_alarm);     //salva in eeprom 
                       
        }
            
      if(strcmp(topic, inTopic[2])==0){
        //TOPI del reset

        reset();
        Riavvia();
      }
        
        
   
}



void reset(){

  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
  
}

 
void setIrrigationDate(  tmElements_t tm ){

     
       EEPROM.write(0, tm.Year);  
       EEPROM.write(1, tm.Month);
       EEPROM.write(2, tm.Day);
       EEPROM.write(3, tm.Hour);
       EEPROM.write(4, tm.Minute);
     
     
} 

tmElements_t getIrrigationDate(){
       tmElements_t tm;
       tm.Year = EEPROM.read(0);
       tm.Month = EEPROM.read(1);           
       tm.Day = EEPROM.read(2);   
       tm.Hour = EEPROM.read(3);   
       tm.Minute = EEPROM.read(4);  
       return tm; 
}

tmElements_t getCurrentDate() {
  tmElements_t tm;
  if(RTC.read(tm)==0){//success
    return tm;
  }
  else {
    rtcError();
  }  
}

void rtcError(){
  Serial.print("RTC ERROR");
}





/*

void fWrite (const byte what)
  {
  rs485.write (what);  
  }

int fAvailable ()
  {
  return rs485.available ();  
  }

int fRead ()
  {
  return rs485.read ();  
  }
*/



