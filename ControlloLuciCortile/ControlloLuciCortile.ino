#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "EmonLib.h" 
#define THRESHOLD_CURRENT 0.18
#define NumLight 5 // effective number of lights to control
#define MQTT_STATUS 22


byte mac[]    = {  0x20, 0x64, 0x32, 0x03, 0x38, 0x90 }; //MAC address of Ethernet shield, use arp -an to find by IP
byte server[] = { 192, 168, 1, 158 }; // IP Address of your MQTT Server. Where RasPi <nd Mosquitto is.
byte ip[]     = { 192, 168, 1, 201 }; // IP for this device. Arduino IP.

 
char* deviceId     = "<DEVICE-ID>";             // * set your device id (will be the MQTT client username)
char* deviceSecret = "<DEVICE-SECRET>";         // * set your device secret (will be the MQTT client password)
char * outTopic[]     = {"casa/luci_ext/status/1",
                         "casa/luci_ext/status/2",
                         "casa/luci_ext/status/3",
                         "casa/luci_ext/status/4",
                         "casa/luci_ext/status/5",
                         "casa/luci_ext/status/6" }; // * MQTT channel where physical updates are published


const char* inTopic[] = {"luce/ext"} ;                          
//char * inTopic      = "luce/1"; // * MQTT channel updates are received
char* clientId     = "<CLIENT-ID>";             // * set a random string (max 23 chars, will be the MQTT client id)
 boolean MQTTstatus;
 bool mqtt_old_status, mqtt_new_status;
EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);


unsigned int outputPinRelay[NumLight] = {31, 32,33, 34, 35}; // Digital output pin for relay switch inputs.
unsigned int vialettoPin = 36;
EnergyMonitor emon[NumLight];
unsigned int analogInputPinSCT013[NumLight]={0,1,2,3,4};
int rede = 110.0;
boolean currentStatus[NumLight];
boolean oldStatus[NumLight];
double  IrmsMax;
void setup()
{
  
  Serial.begin(9600);
  delay(500);
  Ethernet.begin(mac, ip);
  mqttConnection();
  
  for(int i=0; i<NumLight; i++){
     emon[i].current(analogInputPinSCT013[i], 29);
     pinMode(outputPinRelay[i], OUTPUT);
     digitalWrite(outputPinRelay[i],HIGH);
     oldStatus[i]=false;
   }
  pinMode(MQTT_STATUS, OUTPUT);
  digitalWrite(MQTT_STATUS, LOW);
  mqtt_old_status, mqtt_new_status = 1;
  for(int i=9; i>=0; i--){
     mqttConnection();
    delay(1500);
    Serial.print(" -");Serial.print(i);
  }
  for(int i=0; i<NumLight; i++){
      mqttPublish(i, "Spenta");
  }
  
  delay(1000);
   readCurrentSCT013(true);
   delay(1000);
   readCurrentSCT013(true);
   Serial.println("\nSTART");
   IrmsMax = IrmsMax + IrmsMax/2;
   Serial.print("\nSoglia calcolata: ");Serial.print(IrmsMax);
   
  
}
void loop() {
  mqttConnection();
  
  /**
   * TODO: se lo sposto funzioniona o da problemi. A ogni ciclo rialloca un double??
   */
  
  readCurrentSCT013(false);
  
  for(int i=0; i< NumLight; i++) {
            // if the input just went from LOW and HIGH and we've waited long enough to ignore
            // any noise on the circuit, toggle the output pin and remember the time //&& millis() - time > debounce
              if (currentStatus[i] == true && oldStatus[i] == false)  {
                 
                  Serial.print("\nLight turned on:");
                  Serial.print((i+1));
                  mqttPublish(i, "Accesa" );
                  debounce();
                }
               else if(currentStatus[i] == false && oldStatus[i] == true) {
               
                  Serial.print("\nLight turned off:");
                  Serial.print((i+1));
                  mqttPublish(i, "Spenta");
                  debounce();
                }
                oldStatus[i]=currentStatus[i];
  }
   
}


void readCurrentSCT013(boolean initiation){
        double Irms[NumLight];
        for(int i=0; i<NumLight; i++){
              Irms[i]=emon[i].calcIrms(1480);
              delay(100);
             if (Irms[i] > THRESHOLD_CURRENT){
              currentStatus[i] = true; //true = c'è corrente
             }
             else{
              currentStatus[i] = false;
             }
        }
       /*** 
        *  Leggi il valore a vuoto massimo. Poi dopo ci sommo la metà.
        *  Irms = Irms + Irms/2
        *  Il valore letto va poi messo nella Threshold marco.
        */
      if(initiation == true){
      IrmsMax = Irms[0];
      for(int i=1; i<NumLight; i++){
        if(Irms[1]>IrmsMax){
          IrmsMax = Irms[i];
        }
      }
      
    }
 }

void debounce() {
  int time = 0;
       while(time < 2)
      { 
        delay(1000);
        time++;
      }
}

/**
 * MQTT FUNCTIONS
 */
void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("\nPush Light number:");
  if(strcmp(topic, inTopic[0])==0){
    int intbuff[2];
    for(int i=0; i<length; i++) {
      intbuff[i] = (int)payload[i]-48; //convert ascii number in int.
    } 
       
  for(int i=0; i < NumLight; i++){
   int r=intbuff[0]-1;
    if( i == r ){
      // Serial.println("Chiudi relay numero: ");
       Serial.print((i+1));
       digitalWrite(outputPinRelay[i],LOW);
       delay(100);
       digitalWrite(outputPinRelay[i],HIGH);
    }
  }
  }//end if topic
}
 
void mqttConnection() {
  // add reconnection logics
  if (!client.connected()) {
    // connection to MQTT server
    if (client.connect("ArduinoMQTTrele")) //clientId, deviceId, deviceSecret))
    {
      Serial.println("[PHYSICAL] Successfully connected with MQTT");
      MQTTstatus=true;
      mqtt_new_status=0; //change to red
     mqttSubscribe(); 
     
    }
    else{
      mqtt_new_status=1; //change to green
    }  

    
  }
  
        if(mqtt_old_status==1 && mqtt_new_status==0){
          digitalWrite(MQTT_STATUS, HIGH); //go to green
          Serial.println("MQTT led green");
        }
         if(mqtt_old_status==0 && mqtt_new_status==1){
          digitalWrite(MQTT_STATUS, LOW); //go to red
          Serial.println("MQTT led red");
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

