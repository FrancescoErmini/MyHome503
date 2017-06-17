



/*MQTT*/
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Time2.h>

#include <Wire.h>             //http://arduino.cc/en/Reference/Wire
#include <DS3232RTC.h> 

#define SMART 1
#define STUPID 0


#define SQW_PIN 2 //pin interrupt del DS3231
#define MODALITY_SELECTOR  53 //input pin to select smart or stupid mode
#define SCENARIO_SELECTOR 52 
#define MQTT_STATUS 50  //led mqtt
#define RTC_STATUS 51    //led rtc
#define ACS_STATUS 49   //led acs - verde=spenta blu-accesa
#define UPDATEFREQUENCY 120
#define SENSORECORRENTE 0 //acs712 analog pin 0
#define SOGLIACORRENTE 50
//00:aa:bb:cc:de:02
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
                         "irrigazione/status/1"          //Trasmette lo stato, attivo o spento dell'irrigazione.
                         
                      }; // * MQTT channel where physical updates are published
const char* inTopic[] = {"time/1",   //Serve a ricevere l'ora attuale aggiornata da internet
                        "irrigazione/triggerTime/1" //Riceve l'ora a cui parte l'irrigazione
                       };
//-------------------------------------------------------------------------------

unsigned int zona[] = {31,32,33, 34,35,36,37,38,39,40}; //pinout for relay output
unsigned int timerZona[] = {2,2,2,2,2,2,2,2,2,2}; // minuti per ogni zona
unsigned int NumZone = 10; //valore nel setup


tmElements_t tm_rtc; //Contiene la data letta dall'RTC
tmElements_t tm_alarm; //contiene minuti e ore a cui scatta allarme


boolean startup = false;
boolean MQTTstatus,modality, sensorecorrente, alarmReceiveSettings;

unsigned int scenario;
int tm_min,tm_alarm_min,timeToIrrigation;
int anno, mese, giorno, ora, minuto, ora_allarme, minuto_allarme = 0;
String TimeToRemain;
unsigned int long tmp, count;
int tm_rtc_tmp;

bool mqtt_old_status, mqtt_new_status, rtc_old_status, rtc_new_status,  acs_old_status, acs_new_status;




void alarmIsr();
void startIrrigazione();
void tmPrint(tmElements_t tm);
void setupDS3231(ALARM_TYPES_t tipologiaAllarme, byte minuti, byte ore, byte giorno);
void setScenario(boolean modality, unsigned int scenario);
void startIrrigazione();
boolean RTCupdate();
tmElements_t calculateDelay(int delayTime);
  boolean ACS712update();
void mqttConnection();
void mqttSubscribe();
tmElements_t getMQTTdate(byte * payload,  unsigned int length);
void sendMQTTstring(String stringa, int topicnumber);
void sendMQTTdate(tmElements_t tm, int topicnumber);
void notifyTimeToStart();
void sendMQTTstatus();


















void setup(void)
{
          Serial.begin(9600);
          Serial.println("IRRIGATION SETUP\n\n");
          //get mode, stupid or smart
           pinMode(MODALITY_SELECTOR, INPUT_PULLUP);
           //se a massa,modality=1 (automatico)
           if(digitalRead(MODALITY_SELECTOR)==HIGH){
              Serial.print(" - MODALITY SMART");
           modality = SMART;
           alarmReceiveSettings=false; //fintanto che la callback MQTT non arriva, e setta questo  true il loop non fa partire irrigazione.
           }
           else{
            Serial.print(" - MODALITY STUPID");
           modality= STUPID;
           alarmReceiveSettings=true; //nel caso stupido non si deve aspettare alcuna configurazione MQTT
                   pinMode(SCENARIO_SELECTOR, INPUT_PULLUP);
                   if(digitalRead(SCENARIO_SELECTOR)==HIGH){
                   Serial.print(" - SECANRIO 20 30 fixed");
                    scenario = 1;
                   }
                   else {
                    Serial.print(" - SCENARIO 24h from now");
                    scenario = 0;
                    
                   }
           }
           
            //RTC
           rtc_old_status, rtc_new_status=1;
           pinMode(RTC_STATUS, OUTPUT);
           digitalWrite(RTC_STATUS, HIGH);
           RTCupdate();
           //Relay
          
           for(int i=0; i< NumZone; i++) {
                  pinMode(zona[i], OUTPUT); //PULLUP??
                  digitalWrite(zona[i],LOW); 
                  delay(200);
           }
            //ACS712
           sensorecorrente=false;
           acs_old_status, acs_new_status=0;  //alla partenza ACS è nullo, perché i relay sono tutti spenti!
           pinMode(ACS_STATUS, OUTPUT);
           digitalWrite(ACS_STATUS, HIGH);

              
          tm_alarm.Minute=60; tm_alarm.Hour = 25;
          count=UPDATEFREQUENCY;
          MQTTstatus=false;
          mqtt_old_status, mqtt_new_status=1;
          pinMode(MQTT_STATUS, OUTPUT);
          digitalWrite(MQTT_STATUS, HIGH);
                    if(modality==SMART){
                         Ethernet.begin(mac, ip);
                         mqttConnection();           
                    }
                      
                   
          if(modality == STUPID) {
           if(scenario == 0){
            startIrrigazione(); //così da farla partire subito, poi nel loop ogni 24 h si setta l'interrupt
           }
          setScenario(modality, scenario);  
          }
          Serial.println("\n\n\n");
    /*settamodality 0. questomodality setta un ripetizione tutti i giorni all'ora indicata in tm.hours tm.minutes. 
    Se tm è stato aggiornato da intenet allora tm è l'ora nell'istante in cui si accende arduino altirmenti è l'ora indicata sopra.*/  
}


   


volatile boolean alarmIsrWasCalled = false;

void alarmIsr()
{
    alarmIsrWasCalled = true;
}

void loop(void)
{     
    
      if (alarmIsrWasCalled && alarmReceiveSettings){

        if (RTC.alarm(ALARM_2)) {
             setScenario(modality, scenario);
             Serial.println("Interrupt scattato a: ");
             RTCupdate();
             tmPrint(tm_rtc);
             /**
              * TODO: memorizzare lo storico delle irrigazioni qua.
              */
            
            startIrrigazione();
        }
        alarmIsrWasCalled = false; //TODO SPOSTA SOPRA??
    }

       count--;
       delay(1000);
       if(count==0){
             count = UPDATEFREQUENCY;
             RTCupdate();
             sensorecorrente=ACS712update();
             
             if(modality == SMART){
             sendMQTTstatus();
             }

       }

    
     if(modality == SMART){
       mqttConnection();  
     }
  
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

void setScenario(boolean modality, unsigned int scenario){

        if(modality == SMART){
             Serial.println("Modalità SMART\n");
              //prevengo che all'inizio venga settata ora improbabile inizializzando alarm con ore impossibii.
            RTCupdate();
            if(tm_alarm.Hour < 25 && tm_alarm.Minute < 60){
            Serial.print("RTC Time: ");
            tmPrint(tm_rtc);
            Serial.print("Alarm Time: ");
            tmPrint(tm_alarm);
            setupDS3231(ALM2_MATCH_HOURS, tm_alarm.Minute, tm_alarm.Hour, 1);
            }
        }
        else {//modality stupid
                  switch(scenario){
                  case 0: 
                  Serial.println("modality Stupido, scenario 0: irriga tra 24h a partire da adesso\n");
                   /***
                   * LO SCENARIO ZERO LO SETTO RISPETTO AL RTC, perché in caso di mancanza di connessione, setto l'allarme del RTC 15 minuti dopo rispetto all'ora letta sullo stesso RTC. Così si evita problemi.
                   */
                  /*Scenario 0: l'allarme scatta 15 minuti dopo rispetto all'orologio dell'arduino.
                  Funziona anche con orologio non sincronizzato. 
                  L'ora 'reale' a cui scatta è quella in cui si seleziona lo scenario*/
                      RTCupdate();
                      tm_alarm = calculateDelay(1440); //24h=24x60min
                      Serial.print("RTC Time: ");
                      tmPrint(tm_rtc);
                      Serial.print("Alarm Time: ");
                      tmPrint(tm_alarm);
                      setupDS3231(ALM2_MATCH_HOURS, tm_alarm.Minute, tm_alarm.Hour , 1);   
                    
                    
                  break;
                
                   case 1:
                     
                  break;
                
                   case 2:
                   setupDS3231(ALM2_MATCH_DATE, 30, 20, 1);
                  break;
                
                   case 3:
                    setupDS3231(ALM2_MATCH_MINUTES, 1, 0, 1);
                  break;
                
                   case 4:
                    setupDS3231(ALM2_MATCH_HOURS, 30, 20, 1);
                  break;
                
                  default:
                    Serial.println("Error, scnario not exist");
                    //TODO: metti un log su sd
                  break;
                }
        }
}

boolean RTCupdate(){
      if(RTC.read(tm_rtc)==0){
    //blink green led for RTC
   // Serial.println("RTC updated");
    rtc_new_status=0;
    //print date e ore in consle
    }
    else{
      rtc_new_status=1;
    }
      
       
       if(rtc_old_status==1 && rtc_new_status==0){
          digitalWrite(RTC_STATUS, LOW); //go to green
        }
         if(rtc_old_status==0 && rtc_new_status==1){
          digitalWrite(RTC_STATUS, HIGH); //go to red
        }
        rtc_old_status=rtc_new_status;


    if(rtc_new_status == 1){
    Serial.println("RTC ERROR");
    return false;
    }
    return true;
 }  



tmElements_t calculateDelay(int delayTime){
        tmElements_t tm;
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
 * ACS712
 */

boolean ACS712update(){
  /*
   *  (.0264 * analogRead(A0) -13.51) / 1000;
   */
  float average = 0;
  for(int i = 0; i < 10; i++) {
    average = average + (.0264 * analogRead(A0) -13.51) / 10;
            //5A mode, if 20A or 30A mode, need to modify this formula to 
              //(.19 * analogRead(A0) -25) for 20A mode and 
              //(.044 * analogRead(A0) -3.78) for 30A mode
            
    delay(1);
  }

   if(average>0.27){
    //blink green led for RTC
   // Serial.println("RTC updated");
    acs_new_status=1;
    //print date e ore in consle
    }
    else{
      acs_new_status=0;
    }
      
       
       if(acs_old_status==0 && acs_new_status==1){
          digitalWrite(ACS_STATUS, HIGH); //si è attivato ACS.
            Serial.println("IRRIGATION ON");
        }
         if(acs_old_status==1 && acs_new_status==0){
          digitalWrite(ACS_STATUS, LOW); //passa da accesa a spenta.
            Serial.println("IRRIGATION OFF");
          
        }
        acs_old_status=acs_new_status;


    if(acs_new_status == 0){
    Serial.println("CONTROL IRRIGATION is SPENTA");
    return false; //spenta
    }
    
    return true; //accesa
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
      MQTTstatus=true;
      mqtt_new_status=1; //change to red
     mqttSubscribe(); // topic subscription
     
    }
    else{
      mqtt_new_status=0; //change to green
    }  
  }

        if(mqtt_old_status==1 && mqtt_new_status==0){
          digitalWrite(MQTT_STATUS, LOW); //go to green
        }
         if(mqtt_old_status==0 && mqtt_new_status==1){
          digitalWrite(MQTT_STATUS, HIGH); //go to red
        }
        mqtt_old_status=mqtt_new_status;
        
  
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
              anno =   ((intbuff[0])*1000+(intbuff[1])*100+(intbuff[2])*10+(intbuff[3]));
              // anno =   ((intbuff[2])*10+(intbuff[3]));
               mese =   (intbuff[5])*10+(intbuff[6]);
               giorno = (intbuff[8])*10+(intbuff[9]);
               ora =    (intbuff[11])*10+(intbuff[12]);
               minuto = (intbuff[14])*10+(intbuff[15]);

               //togliere tm da variabile globale e dichiararla qui e basta??
                  tm.Hour = ora;             
                  tm.Minute = minuto;
                  tm.Day = giorno;
                  tm.Month = mese;
                  tm.Year = anno-2000;  

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
  RTCupdate();
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
             TimeToRemain = OreRemain + ":" + MinutoRemain;
              sendMQTTstring(TimeToRemain,2);
             
             
}
void sendMQTTstatus(){
                           notifyTimeToStart();
                           sendMQTTdate(tm_rtc,0);
                           
                                if(sensorecorrente==true){
                                  sendMQTTstring("Attiva",3);
                                }
                                else{
                                   sendMQTTstring("Spenta",3);
                                }
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
                if(RTCupdate()==true){
                    if(tm.Hour != tm_rtc.Hour || tm.Minute != tm_rtc.Minute || tm.Year != tm_rtc.Year){
                      Serial.print("Re-sync RTC clock with remote NTP clock");
                       RTC.write(tm); 
                       
                       
                    }
                }
                else{
                  //segnala errore
                }
                   
                   //TODO: sostituire stringbuffer con string persa da RTC
                   // mqttPublish(3,stringbuff);                  
                  
                    
      }
        
      if(strcmp(topic, inTopic[1])==0){
      //TPOIC 1 è l'ora dell'allarme quando deve scattare
            tm_alarm = getMQTTdate(payload,length);
            setScenario(modality, scenario);
           sendMQTTdate(tm_alarm,1);
           alarmReceiveSettings=true;
         /* Serial.println("\nSUBSCRIBE TO TOPIC UNO");
             for(i=0; i<length; i++) {
                        message_buff[i] = payload[i];
                        Serial.print("messaggio:");
                        Serial.println(message_buff[i]);
                        //TODO vedere cosa riceve message buffer
                      }
                     
                      message_buff[i] = '\0';
                      String msgString = String(message_buff);
                    
                       Serial.println("\n Messaggio completo ricevuto:");
                       Serial.print(String(msgString));*/
                       
        }
        
   
}


