
#include "Timer.h"
#include <EEPROM.h>
//#include <SoftReset.h>

#define TELNET_MODULE
#define GSM_MODULE
#define COM_MODULE 

#ifdef TELNET_MODULE
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#endif

#ifdef GSM_MODULE

#include "SIM900.h"

//#include "inetGSM.h"
#include "sms.h"
#include "call.h"

#endif

//#include "Time.h"

#define DEBUG_CMD
#define DEBUG
#define DEBUG_TELNET 1
#define DEBUG_NFC 1 
#define DEBUG_ERROR 1
boolean alreadyConnected = false;//we'll use a flag separate from client.connected
boolean newData;



Timer t;


 #ifdef GSM_MODULE
CallGSM call;
SMSGSM sms;
#endif


int alarm_interrupt = 20;                 //button to test alarm insertion

const int buzzer_pin = 46;
const int siren_pin = 13;
const int LED_ALARM_ON = 52;
const int LED_ALARM_OFF = 53;
const int LED_ERROR_ACK=48;
////const int LED_TELNET_OK = 50;
const int LED_ERROR_GSM  = 47;
///const int LED_GSM_OK = 48;
const int LED_ERROR_ACTION = 49;

const int EEPROM_status_addr = 0;          //where current alarm status is stored
const int EEPROM_scenario_addr = 1;       //where current scenario is stored

/* ===============================================      ALARM SKELETON      ================================================== */

/*
 
 * 
 * SCENARIO
 * Identifica scenari abbinati ad azioni o circostanze.
 * SCENARIO_DORMO - le finestre aperte al momento dell'inserimeto sono escluse. Volumetrici e barriere IR sono esclusi. 
 * SCENARIO_ESCO - esclude la barriera perimetrale esterna. ( si intende SCENARIO_ESCO per qualche ora, non settimane).
 * SCENARIO_VACANZA - non escludo nulla
 * AUTOMATICO - varia in relazione al dispositivo.
 * E.g
 * Se attivo allarme da sms, molto probabilmente sono lontano da cas, per cui lo scenario SCENARIO_PREDEFINITO è 'SCENARIO_VACANZA'.
 * Se attivo allarme da NFC, molto probabilmente sto uscendo di casa, per cui lo scenario PREDEFINTO è 'SCENARIO_ESCO'.
 * Se attivo allarme da Server (in Rete Locale), molto probabilmento sono in casa, per cui lo scenario PREDEFINTO è 'SCENARIO_DORMO'.
 */
 
enum status_t {
  STATUS_ON,
  STATUS_ON_SCENARIO_DORMO,
  STATUS_ON_SCENARIO_ESCO,
  STATUS_ON_SCENARIO_VACANZA,
  STATUS_OFF,
  STATUS_UNDEF
};

enum scenario_t {
  SCENARIO_PREDEFINITO,
  SCENARIO_DORMO,
  SCENARIO_ESCO,
  SCENARIO_VACANZA,
  SCENARIO_OFF,
  SCENARIO_UNDEF
};

enum device_t {                           
  dev_TELNET,
  dev_GSM,
  dev_COM,
  dev_SERVER,
  dev_null
};

enum gsm_status_t {
  GSM_OK,
  GSM_NOSIGNAL,
  GSM_OFF,
  
};
enum calling_t {
  CALLING_START,
  CALLING_STOP,
  CALLING_FAIL,
  CALLING_TIMEOUT,
};
/*enum call_t {
  CALL_ERROR,
  CALL_
};*/
enum action_t {
  START,
  INIT,
  CALL,
  END,
  FAIL
};

enum blink_t {
  LED_STATUS_ALARM,
  LED_STATUS_GSM,
  LED_STATUS_ACK,
  LED_STATUS_ACTION
};

enum error_t {
   ERROR_GSM,
   ERROR_ACK,
   ERROR_ETH,
   ERROR_ACTION,
   ERROR_NONE    
};

enum sensor_t {                          
  WINDOW,
  VOLUMETRIC,
  BARRIER,
  PERIMETRIC,
  VIBRATION
};

struct sensor_info_t {
 const char * name;
 const int ID;
 const sensor_t type;           // type = window, volumetric,  perimetric,  vibration        
 const int pin;
 boolean event;                  //per una finestra, essere aperta è event=true, per un volumetrico attivarsi è event=true
 boolean state;
 boolean old_state;
 boolean sensor_exclusion;  
};

struct contact_info_t {
 const char * name;             //name of the person
 const char * number;          //phone numbers
 int SIMposition;              //position in sim phonebook ( alternative use of number)
};




void(* Riavvia)(void) = 0;

/*=========================================================   SENSOR CENTRIC  ====================================================*/

/*
 * SENSOR CENTRIC
 * Il meccanismo di allarme è incentrato sul singolo sensore. ciascun sensore ha un nome mnemonico, un id, una tipologia ed un pin che rimangno invariati. 
 * Inoltre ogni sensore ha un stato attuale, uno stato precetente, un flag di esclusione.
 * L'idea è quella per cui è solo un sensore che fa scattare l'allarme.
 * Durante il ciclo di lettura dei sensori, quando lo stato del sensore passa da false a true
 * Se l'allarme è inserito e se il sensore i-esimo non è escluso dallo scenario in uso, viene settato un event
 * L'event fa da evento trigger per l'allarme, che a questo punto scatta.
 * 
 * L'esclusione dei sensori sulla base dello scenario avviene quando l'allarme viene inserito.
 * In questo modo l'algoritmo che decide se far scattare l'allarme risulta snello e pulito.
 * 
 * I cambi sensor[i] sono riembiti in fasi successive.
 * 1. nel setup() si setta ID,  pin e tipologia di sensore.
 * 2. durante la configurazione dello scenario ( che avviene quando viene inserito l'allarme)
 *    in base alle regole di quello scenario l'algoritmo decide se il sensore deve o no essere escluso settando sensor_exclusion.
 *    e.g Nello scenario 'SCENARIO_DORMO' per il sensore con ID 7, essendo un volumetrico, verrà settato sensor_exclusion = true. 
 * 3. Lo stato attuale e precedente del sensore è letto nel loop(). 
 */


const char * sensor_name[] = { "giu: porta ingresso",
                                "giu: lavanderia ",
                                "giu: garage ",
                                "mezzanino: bagno ",
                                "mezzanino: studio ",
                                "mezzanino: camera ",
                                "mezzanino: volumetrico  ",
                                "taverna: portone ",
                                "taverna: porta-finestra ",
                                "taverna: volumetrico ",
                                "taverna: finstra ",
                                "appendice: cucina ",
                                "appendice: volumetrico  ",
                                "appendice: bagno  ",
                                "appendice: camera ",
                                 "scale: finestra ",
                                 "scale: volumetrico ",
                                 "scale: laser ",
                                 "su: volumetrico" ,
                                 "su: camera dietro ",
                                 "su: camera davanti ",
                                 "su: bagno dietro ",
                                 "su: bagno davanti ",
                                 "su: porta ingresso ",
                                 "su: porta dietro ",
                                 "mansarda: davanti ",
                                 "mansarda: dietro ",
                                 
                                 
                                 
                                                               
                                };                              
sensor_info_t sensor[] = {
                              { .name = sensor_name[0], .ID=  1, .type = WINDOW,     .pin = 22 },
                              { .name = sensor_name[1], .ID=  2, .type = WINDOW,     .pin = 23 },
                              { .name = sensor_name[2], .ID=  3, .type = WINDOW,     .pin = 24 },
                              { .name = sensor_name[3], .ID=  4, .type = WINDOW,     .pin = 25 },
                              { .name = sensor_name[4], .ID=  5, .type = WINDOW,     .pin = 26 },
                              { .name = sensor_name[5], .ID=  6, .type = WINDOW,     .pin = 27 },
                              { .name = sensor_name[6], .ID=  7, .type = VOLUMETRIC, .pin = 28 },
                              { .name = sensor_name[7], .ID=  8, .type = WINDOW,     .pin = 29 },
                              { .name = sensor_name[8], .ID=  9, .type = WINDOW,     .pin = 30 },
                              { .name = sensor_name[9], .ID= 10, .type = VOLUMETRIC, .pin = 31 },
                              { .name = sensor_name[10], .ID= 11, .type = WINDOW,     .pin = 32 },
                              { .name = sensor_name[11], .ID= 12, .type = WINDOW,     .pin = 33 },
                              { .name = sensor_name[12], .ID= 13, .type = VOLUMETRIC, .pin = 34 },
                              { .name = sensor_name[13], .ID= 14, .type = PERIMETRIC, .pin = 35 },
                              { .name = sensor_name[14], .ID= 15, .type = VIBRATION,  .pin = 36 },
                              
                              { .name = sensor_name[15], .ID=  16, .type = WINDOW,     .pin = 37 },
                              { .name = sensor_name[16], .ID=  17, .type = VOLUMETRIC,     .pin = 38 },
                              { .name = sensor_name[17], .ID=  18, .type = PERIMETRIC,     .pin = 39 },
                              { .name = sensor_name[18], .ID=  19, .type = VOLUMETRIC,     .pin = 40 },
                              { .name = sensor_name[19], .ID=  20, .type = WINDOW,     .pin = 41 },
                              { .name = sensor_name[20], .ID=  21, .type = WINDOW,     .pin = 43 },
                              { .name = sensor_name[21], .ID=  22, .type = WINDOW,     .pin = 44 },
                              { .name = sensor_name[22], .ID=  23, .type = WINDOW,     .pin = 45 },
                              { .name = sensor_name[23], .ID=  24, .type = WINDOW,     .pin = 46 },
                              { .name = sensor_name[24], .ID=  25, .type = WINDOW,     .pin = 47 },
                                                            
                              { .name = sensor_name[25], .ID=  25, .type = WINDOW,     .pin = 47 },
                              { .name = sensor_name[26], .ID=  25, .type = WINDOW,     .pin = 47 },


                              
                          };


const int Ns = sizeof(  sensor )/sizeof(sensor_info_t);   

/*=========================================================   COMMAND  DRIVEN  ====================================================*/

/*
 * L'interazione con l'allarme avviene attraverso messaggi testuali, detti comandi.  
 * 
 */

                      
enum cmd_t {
  CMD_ERROR,        //non risponde al 
  CMD_NONE,
  CMD_WRONG,  //comando ricevuto, ma nessun significato nel testo
  CMD_STATUS,   // send 'ALARM' for wait status back..
  CMD_OFF,   // send 'ALARM OFF'
  CMD_ON,   // send 'ALARM ON, scenario SCENARIO_PREDEFINITO'
  CMD_ON_SCENARIO_DORMO,   // send 
  CMD_ON_SCENARIO_ESCO,
  CMD_ON_SCENARIO_VACANZA,
  CMD_ACK,
  CMD_NOT_AUTH,
  CMD_SENSORS_STATUS,
  CMD_GSM_STATUS
};

const int BUFSIZE = 20;
char buffer_incoming_command[BUFSIZE];                         // where to store incoming command
const char * incoming_command[] = {"status",//CMD_STATUS              // possibles commands received from the server
                                    "off",//CMD_OFF
                                    "on",//CMD_ON
                                    "on dormo",//CMD_ON_SCENARIO_DORMO
                                    "on esco",//CMD_ON_SCENARIO_ESCO
                                    "on vacanza",//CMD_ON_SCNEARIO_VACANZA
                                    "reset",//CMD_ERROR
                                     "1" , //CMD_ACK               //resp to ACK
                                     "sensori",//CMD_SENSOR_STATUS
                                     "gsm"
                                    };
 const char * outgoing_command[] = {"not used"  ,              // possible commands send from the server
                                    "stato off",
                                    "stato on",
                                    "stato on dormo",
                                    "stato on esco",
                                    "stato on vacanza",
                                    "errore",
                                    "comando sbagliato"
                                    };
struct command_t  {
      cmd_t cmd;
      status_t status;
      scenario_t scenario;
      const char * incoming_message;  
      const char * outgoing_message;   
                        
};

const command_t command[] = {
                             { .cmd = CMD_STATUS,              .status = STATUS_UNDEF, .scenario = SCENARIO_UNDEF,        .incoming_message = incoming_command[0], .outgoing_message = outgoing_command[0]},
                             { .cmd = CMD_OFF,                 .status = STATUS_OFF, .scenario = SCENARIO_OFF,          .incoming_message = incoming_command[1], .outgoing_message = outgoing_command[1] },
                             { .cmd = CMD_ON,                  .status = STATUS_ON, .scenario = SCENARIO_PREDEFINITO,     .incoming_message = incoming_command[2], .outgoing_message = outgoing_command[2] },
                             { .cmd = CMD_ON_SCENARIO_DORMO,   .status = STATUS_ON, .scenario = SCENARIO_DORMO,           .incoming_message = incoming_command[3], .outgoing_message = outgoing_command[3] },
                             { .cmd = CMD_ON_SCENARIO_ESCO,    .status = STATUS_ON, .scenario = SCENARIO_ESCO,            .incoming_message = incoming_command[4], .outgoing_message = outgoing_command[4] },
                             { .cmd = CMD_ON_SCENARIO_VACANZA, .status = STATUS_ON, .scenario = SCENARIO_VACANZA,         .incoming_message = incoming_command[5], .outgoing_message = outgoing_command[5] },
                             { .cmd = CMD_ERROR,               .status = STATUS_UNDEF, .scenario = SCENARIO_UNDEF,        .incoming_message = incoming_command[6], .outgoing_message = outgoing_command[6] },
                             { .cmd = CMD_WRONG,               .status = STATUS_UNDEF, .scenario = SCENARIO_UNDEF,        .incoming_message = incoming_command[0], .outgoing_message = outgoing_command[7] },
                             { .cmd = CMD_ACK,                 .status = STATUS_UNDEF, .scenario = SCENARIO_UNDEF,        .incoming_message = incoming_command[7], .outgoing_message = outgoing_command[0] },
                             { .cmd = CMD_SENSORS_STATUS,      .status = STATUS_UNDEF, .scenario = SCENARIO_UNDEF,        .incoming_message = incoming_command[8], .outgoing_message = outgoing_command[0] },
                             { .cmd = CMD_GSM_STATUS,          .status = STATUS_UNDEF, .scenario = SCENARIO_UNDEF,        .incoming_message = incoming_command[9], .outgoing_message = outgoing_command[0] }


  };

const int Nc = sizeof(  command )/sizeof(command_t);   

/*=========================================================   TELNET    ====================================================*/

// Comment if you don't want to use auth
//#define AUTH_PIN 1 
/*
 * L'autenticazione sul telnet è una autenticazione 'home-made' più scomoda che sicura. 
 * Il funzionamento è che una volta collegati al telnet, il primissimo messaggio che uno manda deve essere il pin numerico indicato sotto. 
 * Se questo messaggio non corrisponde al pin, la connessione viene chiusa.
 * Se corrisponde, la sessione telnet funziona normalmente. 
 * Tutte le volte che il client si riconnette deve inserire il pin.
 */
#ifdef AUTH_PIN
 boolean authorized = false;
 boolean enter_pin = false;
 const char pin[] = "2820";
#endif
unsigned int ack_counter;
int error_counter;
status_t old_status;


/*=========================================================   MQTT    ====================================================*/
// Update these with values suitable for your network.
byte mac[]    = {  0xEE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; //MAC address of Ethernet shield, use arp -an to find by IP
byte server[] = { 192, 168, 1, 101 }; // IP Address of your MQTT Server. Where RasPi <nd Mosquitto is.
byte ip[]     = { 192, 168, 1, 119 }; // IP for this device. Arduino IP.

 
EthernetClient ethClient;
void callback(char* topic, byte* payload, unsigned int length);
PubSubClient client(server, 1883, callback, ethClient);
//char message_buff[100];
char* deviceId     = "<DEVICE-ID>";             // * set your device id (will be the MQTT client username)
char* deviceSecret = "<DEVICE-SECRET>";         // * set your device secret (will be the MQTT client password)
char* clientId     = "<CLIENT-ID>"; 

const char * outTopic[]     = {     "myhome503/alarm/status",
                                     "myhome503/alarm/action",
                              };

         
const char * sensorsOutTopic[]     = {    
                              "myhome503/alarm/sensors/0",
                              "myhome503/alarm/sensors/1",
                              "myhome503/alarm/sensors/2",
                              "myhome503/alarm/sensors/3",
                              "myhome503/alarm/sensors/4",
                              "myhome503/alarm/sensors/5",
                              "myhome503/alarm/sensors/6",
                              "myhome503/alarm/sensors/7",
                              "myhome503/alarm/sensors/8",
                              "myhome503/alarm/sensors/9",
                              "myhome503/alarm/sensors/10",
                              "myhome503/alarm/sensors/11",
                              "myhome503/alarm/sensors/12",
                              "myhome503/alarm/sensors/13",
                              "myhome503/alarm/sensors/14",
                              "myhome503/alarm/sensors/15",
                              "myhome503/alarm/sensors/16",
                              "myhome503/alarm/sensors/17",
                              "myhome503/alarm/sensors/18",
                              "myhome503/alarm/sensors/19",
                              "myhome503/alarm/sensors/20",
                              "myhome503/alarm/sensors/21",
                              "myhome503/alarm/sensors/22",
                              "myhome503/alarm/sensors/23",
                              "myhome503/alarm/sensors/24",
                              "myhome503/alarm/sensors/25",
                              "myhome503/alarm/sensors/26",
                          };                        
const char * inTopic[]   ={"myhome503/alarm/cmd"}; 
bool mqtt_old_status, mqtt_new_status, mqtt_has_received_data;


                                   
/* ===========================================================  GSM (SMS)======================================================*/
                                   
/*
 * GSM BASED. 
 * 
 */
//char buffer_incoming_command_sms[100];                         // where to store incoming command
unsigned int call_routine_counter_max_loop = 50; 
unsigned int action_routine_counter_max_loop = 90;  // il tempo di durata del action routine i.e quanto suona la sirenza, fin quando si riprova a chiamare
unsigned int delay_before_start_call = 110;          // circa 20 sec, tempo di attesa prima di far partire la chiamata e la sirenza. Tempo per spegnere l'allarme da quando si apre la porta.
unsigned int tempo_di_non_risposta = 30;            // 30 sec, ovvero 1/2 min. se il telefono squilla a vuoto per un tempo superiore a questo viene considerato non risposta. 
                                                   
const contact_info_t family[] = {
                                  { .name = "Francesco",  .number = "3338710609", .SIMposition = 2 },
                                  { .name = "Stefano", .number = "3386741516", .SIMposition = 1 },
                                  { .name = "Caterina", .number = "3349790696", .SIMposition = 3 }
};

const int N = sizeof(family)/sizeof(contact_info_t); //number to call
boolean notified[N];
unsigned int gsm_error_counter;
char number[20];  //buffer to store the number
boolean sms_response_unlock, sms_unread; ///







/* =========================================================== ALARM ROUTINE  ======================================================*/
/*
 * 
 * Si occupa di leggere i piedini relativi ai sensori per stabilire lo stato di ogni sensore, false == OFF true == ON.
 * Lo stato attuale viene confrontato con quello passato, per accorgersi quando il sensore passa da stato spento ad uno stato accesso.
 * il singolo sensore setta l'evento 'intruso' se passa da spento ad accesso, l'allarme è inserito (STATUS_ON) e il sensore non è stato escluso 
 * dallo scenario in fase di attivazione (sensor_exclusion == false).
 * 
 * L'evento intruso sul sensore chiama actionRoutine().
 *
 * Scelte progettuali:
 * 1. fare la verifica sul valore di old_state permette che la funzione actionRoutine() 
 *    non parta più volte dopo che una finestra è stato aperta. 
 *    Questa scelta permette di inserire un timeout alla funzione actionRoutine.
 *    Per cui dopo che l'allarme è scattato le azioni (sirena e chiamate) non si ripeteranno
 *    all'infiito nel caso in cui nessuno spenga l'allarme.
 * 2. Il valore di old_state viene settato nel setup() in base al valore letto sul piedino di ingresso,
 *    invece che metterlo false indipendentemente dal valore letto sul piedino.
 *    L'effetto di questo è di evitare che l'allarme scatti subito dopo averlo inserito se si dimentica una finestra aperta.
 *    La consequenza negativa è che le finestre aperte non faranno scattare l'allarme se non vengono chiuse e poi riaperte.
 *    Pertanto tale punto è motlo discutibile.
 * 3. Tutto gira intormo al singolo sensore e alle sue proprietà.
 * 
 * 
 * 
 * 
 */


void alarmRoutine();
void actionRoutine();
cmd_t commandRoutine();


void setStatus( status_t current_status);
void setScenario(scenario_t current_scenario);
status_t getStatus();
scenario_t getScenario();
void configScenario(scenario_t scenarioX);
int associateCommand(char * message);
 
int receiveCommand(char * receive_buffer, device_t * dev);

void replyCommand(int cmdNum, device_t dev);
void sendStatus( device_t dev); 
void sendSensorStatus( device_t dev);
void sendGsmStatus(device_t dev);
  
void printSensors();

boolean _telnet(char * receive_buffer,const char * transmit_buffer);
  
boolean _serial(char * receive_buffer,const char * transmit_buffer);

boolean _sms(char * receive_buffer, const char * transmit_buffer);

calling_t _call();           

void gsmInit();

void error( error_t error);
            
void _blink(blink_t blink_status, uint8_t val);
  
void print_command( cmd_t cmd);

void print_status(status_t status);
 
void print_action(action_t act);
 

void siren(boolean siren_status);

void beep(int f);
  

void _DEBUG(String debugString);


void _DEBUGINT(int debugString);

void action();

void lights(boolean value);
  
boolean _sms(char * receive_buffer, const char * transmit_buffer);

boolean gsm_Signal_Strength();


 boolean gsm_AT_OK();


void mqttPublish(const char * topic, char* messaggio);


void mqttSubscribe();


void mqttConnection();









 
void alarmRoutine(){
   
        for(int i=0; i<Ns; i++){
          
          if( digitalRead(sensor[i].pin) == LOW){
                sensor[i].state = false;
          }
          
          else{ 
                sensor[i].state = true;
          }
         
          if( sensor[i].state == true && sensor[i].old_state == false){    // 1. se la finestra è apera        
            if( getStatus()==STATUS_ON ) {                                 // 2. se l'allarme è accesso
              if( sensor[i].sensor_exclusion == false ) {                  // 3. se il sensore non è escluso
                sensor[i].event = true;                                    // 4. il sensore genera l'evento 'intruso'
              }
            }
          }
           sensor[i].old_state = sensor[i].state;
          // 4. evalute 
             if( sensor[i].event == true ) {                               // 5. se è scattato l'evento intruso
               sensor[i].event = false;
                  actionRoutine();                                          // 6. parte l'azione post intrusion detect
            }
        }          
}

/* =========================================================== ACTION ROUTINE  ======================================================*/
/**
 * si occupa di gestire le azioni da svolgere dopo che l'intruso è stato rilevato e in particolare:
 *  a. rimanere in ascolto di eventuali comandi che vogliono spegnere l'allarme 
 *  b. chiamare i familiari fintanto che non rispondono. Se hanno già risposto non li chiamare nuovamente.
 *  c. chiude e apre il relay della sirena.
 *  d. notificare errori che impediscono di effettuare correttamente le chiamate
 *  
 *  Note:
 *  La FSM termina nello stato END ( Successo) solo se l'allarme viene spendo entro action_routine_counter_max_looop;
 */
void actionRoutine(){
       //1. accedni luci
       //2. per 20 sec fai bip bip
       //3. spegni luci
       //4. inizia le chiamate
       //5 accendi la sirena
       //6 manda sms, seriale log
       //7 chiama tutti
       // spegni sirena dopo timeout chiamata o in caso spegnimento allarme
       Serial.print("\nALARM ACTION");
       action_t act=START;
       lights(true);
        #ifdef COM_MODULE
       _serial(NULL, "alarm event");
       #endif
       for(int i=0; i<delay_before_start_call; i++){ // it's about 15-20 sec delay time before start calling
                          commandRoutine();                         // ci sono stati comandi che spengono l'allarme?

                          
                            //blink the red led
                            if( STATUS_OFF == getStatus() ){
                                  
                                  act = END;
                        
                                 
                            break;
                            }
                            else{
                                 // Serial.println("WAIT");
                                  beep(i);             //emetti un suono intermittente. Vedi beep()
                                  
                                  act = START;
                            }
        }
        lights(false);


        
         if ( act == END ) {        
                  beep(delay_before_start_call);
                #ifdef COM_MODULE
               _serial(NULL, "alarm stop");
               #endif
                                  
         return;
         }


         siren(true);
         #ifdef TELNET_MODULE
            Serial.println("Invia action event mqtt");
            mqttPublish(outTopic[1], "on");
         #endif
         #ifdef COM_MODULE
          _serial(NULL, "allarme started");
         #endif
         #ifdef GSM_MODULE
         calling_t calling_behaviour = _call();
          #endif

         
}


/* =========================================================== ROUTINE  FOR  COMMANDS  ======================================================*/
/*  La command routine agisce in tre parte:
 *   A -> RICEZIONE: lettura dei messaggi da sms e telnet e interpretazione del comando ricevuto
 *   B -> AZIONE:    quali azioni occorre fare in relazione al comando ricevuto
 *   C -> RISPOSTA:  trasmissione della risposta associata al comando ricevuto.
 *  
 *  A - L'allarme viene configurato con i messaggi che entrano, gli incoming_command[].
 *      Questi messaggi sono associati ad un comando cmd_t con get_command(buffer_incoming_command) .
 *  B - Dopo di che al comando ricevuto si fanno seguire le azioni che quel comando porta:
 *      - selezionare lo status on / off
 *      - selezionare lo scenario
 *      - configurare i sensori in base allo sceanrio ( in particolare escldere i sensori a seconda dello scenario)
 *      - scegliere la risposta da dare al comando
 *      
 *      Ad esempio CMD_ON_SCENARIO_DORMO deve da mettere lo stato 'on' e lo scenario 'dormo'.
 *      
 *  Scelte progettuali:
 *  1. Nei punti A e C la Routine dei comandi gestisce in modo uniforme i vari canali di comunicazione ( gsm, telnet).
 *     Al punto A il cmd ricevuto non dipende dal canale con cui lo ha ricevuto,   cmd = get_command(buffer_incoming_command).
 *     Per differenziare risposte o configurazioni si seleziona il dispositivo dev, ovvero il canale di comunicazione.
 *  Problemi e TODO.
 *   
 *  L'associazione tra cmd e messaggi input oppure output è assegnata alle funzioni get_command responseStatus e respinseCommand. 
 *  Queste funzioni dipendono dal 'posto' in cui è messo il messaggio nel buffer, e manca una associazione più stretta
 *  tra cmd e messaggi associati.
 *  
 */             
cmd_t commandRoutine(){
     
      device_t dev=dev_null;
      

     int cmdRx = receiveCommand(buffer_incoming_command, &dev);
   
    if(  cmdRx >= 0 ){
          #ifdef DEBUG_CMD
          print_command(command[cmdRx].cmd);
          #endif
    
     if( command[cmdRx].cmd == CMD_OFF || command[cmdRx].cmd == CMD_ON || command[cmdRx].cmd == CMD_ON_SCENARIO_DORMO ||  command[cmdRx].cmd == CMD_ON_SCENARIO_ESCO ||  command[cmdRx].cmd == CMD_ON_SCENARIO_VACANZA) {
        
          setStatus(command[cmdRx].status);
          setScenario(command[cmdRx].scenario );
          configScenario(command[cmdRx].scenario );
          replyCommand(cmdRx, dev);
     }
     else if( command[cmdRx].cmd == CMD_STATUS ) {
      
          sendStatus(dev);
 
     }
     else if(command[cmdRx].cmd == CMD_SENSORS_STATUS){
        sendSensorsStatus(dev); 
     }
     else if(command[cmdRx].cmd == CMD_GSM_STATUS){
        sendGsmStatus(dev); 
     }
     else if(command[cmdRx].cmd == CMD_ERROR){    
      delay(1000);
       Riavvia();
     }
     else if(command[cmdRx].cmd == CMD_WRONG){
       replyCommand(cmdRx, dev);
     }

    }

}

/* ========================================================         ALARM FUNCTION       =============================================================*/
/*
 * Stato e scenario corrente dell'allarem sono memorizzati in EEPROM. 
 * La scrittura di questi valori avviene solo in fase di attivazione dell'allarme (quindi tramite commandRoutine() ) 
 * La lettura avviene tutte le volte che l'allarme vuole mandare in broadcast lo stato agli altri nodi.
 * Anche a fronte di reset dell'arduino, lo stato memorizzato in EEPROM rimane e questo garantisce una memoria della configurazione precedentemente imposta.
 * 
 */

 
void setStatus( status_t current_status){

     if( current_status == STATUS_ON){
                  //Serial.print(F("\n write in EEPROM 1"));
      EEPROM.write(EEPROM_status_addr, 1);
     }
     else{
               // Serial.print(F("\n write in EEPROM 0"));
       EEPROM.write(EEPROM_status_addr, 0);
     }
} 

void setScenario(scenario_t current_scenario){   
     
     if( current_scenario == SCENARIO_DORMO){
     
      EEPROM.write(EEPROM_scenario_addr, 0);
     }
     if( current_scenario == SCENARIO_ESCO){
     
      EEPROM.write(EEPROM_scenario_addr, 1);
     }
     if( current_scenario == SCENARIO_VACANZA){
     
      EEPROM.write(EEPROM_scenario_addr, 2);
     }
     if( current_scenario == SCENARIO_OFF){
     
      EEPROM.write(EEPROM_scenario_addr, 3);
     }
     if( current_scenario == SCENARIO_PREDEFINITO){
     
      EEPROM.write(EEPROM_scenario_addr, 4);
     }
}

status_t getStatus(){
  
         if( 0  == EEPROM.read(EEPROM_status_addr) ) {
                    //  Serial.print(F("\n read from EEPROM status off"));
          return STATUS_OFF;
         }
         if( 1  == EEPROM.read(EEPROM_status_addr) ){
                   //  Serial.print(F("\nread from  EEPROM status on"));
          return STATUS_ON;
         }
}

scenario_t getScenario(){
   
     if( 0 == EEPROM.read(EEPROM_scenario_addr) ){
      return SCENARIO_DORMO;
     }
    if( 1 == EEPROM.read(EEPROM_scenario_addr) ){
      return SCENARIO_ESCO;
     }
    if( 2 == EEPROM.read(EEPROM_scenario_addr) ){
      return SCENARIO_VACANZA;
     }
     if( 3 == EEPROM.read(EEPROM_scenario_addr) ){
      return SCENARIO_OFF;
     }
      if( 4 == EEPROM.read(EEPROM_scenario_addr) ){
      return SCENARIO_PREDEFINITO;
     }

}
/*
 * E' possibile attivare l'allarme senza specificare lo scenario. 
 * In questo caso lo scenario verrà automaticamete selezionato in base al tipo di dispositivo.
 * E' verosimile che si si attiva l'allarme da sms siamo fuori casa, da qui SCENARIO_VACANZA.
 * Se invece si invia i comandi tramite telent sullo smartphone si può pensare che l'utente sia dentro casa (rete LAN), SCENARIO_DORMO.
 * Quello che qui non si vede è che l'NFC di default  manda il comando SCENARIO_ESCO. 
 * In tal caso la configurazione automatica non viene fatta sul server ma sul clinet.
 */

/*
 * Una volta ricevuto uno scenario è possibile configurare i sensori affinché ripettino quello scenario.
 * Ad esmpio SCENARIO_DOMRO deve disttivare tutti i volumetrici.
 */
void configScenario(scenario_t scenarioX){
     
    switch(scenarioX){
       case SCENARIO_VACANZA: //tutto attivo
       for(int i=0; i<Ns; i++){
        sensor[i].sensor_exclusion = false;
       }
       break;
       case SCENARIO_ESCO: //disattivo perimetrali
       
         for(int i=0; i<Ns; i++){
              
              if( sensor[i].type == PERIMETRIC ) {
                sensor[i].sensor_exclusion = true;
              }
              else{
                sensor[i].sensor_exclusion = false;
              }
         }       
      
      break;
      case SCENARIO_DORMO:
             for(int i=0; i<Ns; i++){
              
              if( sensor[i].type == VOLUMETRIC || sensor[i].type == BARRIER || sensor[i].state == true ) {
                sensor[i].sensor_exclusion = true;
              }
              else{
                sensor[i].sensor_exclusion = false;
              }
         }       

      
       break;
       case SCENARIO_PREDEFINITO:
       //TODO: è uguale ad dormo, da rifare a paicere.
            for(int i=0; i<Ns; i++){
              
              if( sensor[i].type == PERIMETRIC ) {
                sensor[i].sensor_exclusion = true;
              }
              else{
                sensor[i].sensor_exclusion = false;
              }
         }       
      

       break;
     
      case SCENARIO_OFF:
         for(int i=0; i<Ns; i++){
        sensor[i].sensor_exclusion = true;
       }
      break;
      
    }
 
 return;
}
/*
 * A partire dal testo ricevuto si guarda se questo corrisponde al testo associato al comando.
 * Se si ritpr
 * Nota:
 * Quando usi telent, le word arrivano nel formato 'a' 'b' 'c' '\r' '\0'
 * Quando usi sms, le word arrivano nel formato 'a' 'b' 'c' '\0'
 * Per cui per trovare la lunghezza della parola si usano un or con entrambe le ocndizioni di arresto.
 * Inoltra notare che len parte a contare da 0 per cui, dicendomi la posizione del carattere di fine stringa, mi dice la  lunghezza effettiva della parola.
 */

int associateCommand(char * message){
  //Serial.print("\nmess:"); Serial.print(message);
  for(int i=0; i<Nc; i++){
    int len=0;
  // Serial.print("\n");
    for(int p = 0; p<20; p++){
   // Serial.print(" c:");Serial.print(message[p], HEX);
      if(message[p] == '\0' || message[p] == '\r'){
        
        len = p;
        //Serial.print("\nlen:");
        //Serial.print(len);
        break;
      }
    }
    
    if(memcmp(message, command[i].incoming_message, (len)) == 0 ){ 
      return i;
      print_command(command[i].cmd);
    }
   
     
  }
  // se non hai fatto return prima è perché è sbagliato il programma
  for(int k=0; k<Nc;k++){
        if( command[k].cmd == CMD_WRONG){
          return k;
        }
      }
  //no commands match this message txt
}
/*
 *TODO
 */



int receiveCommand(char * receive_buffer, device_t * dev){
int res = -1;
          #ifdef TELNET_MODULE
            
              if ( _telnet(receive_buffer, NULL) == true ){  
                     res = associateCommand(receive_buffer);             
                     *dev = dev_TELNET; 
                     /*
                     Serial.print("receive_buffer: ");  Serial.print(receive_buffer);
                      Serial.print("\n result cmd: ");
                     Serial.print(res);
                     */
               }
           #endif

           
           #ifdef GSM_MODULE
               if( _sms(receive_buffer, NULL) == true ) {
                      res = associateCommand(receive_buffer); 
                      *dev = dev_GSM;                  
               }
           #endif

           
           #ifdef COM_MODULE
               if(_serial(receive_buffer, NULL) == true ) {
                      res = associateCommand(receive_buffer); 
                      *dev = dev_COM;
               }
           #endif
           if( res != -1){
            _DEBUG("\n\nreceived message:");
            _DEBUG(receive_buffer);
             _DEBUG("\nreceived command numebr:");
             _DEBUGINT(res);
              _DEBUG("\n");
           }
  return res;
}

void replyCommand(int cmdNum, device_t dev){
                      const char * reply; //= responseCommand(cmd);
                      
                                  
                       reply=command[cmdNum].outgoing_message;
                  
                       if( reply != outgoing_command[0] ) {

                            #ifdef TELNET_MODULE
                            if( dev == dev_TELNET){
                                _telnet(NULL, reply); 
                            }
                            #endif

                            #ifdef GSM_MODULE
                            if( dev == dev_GSM ){
                               _sms(NULL, reply );
                            }
                            #endif

                            #ifdef COM_MODULE
                            if( dev == dev_COM ){
                               _serial(NULL, reply );
                            }
                            #endif  
                           
                       }
}




/*
 * Svolge l'azione di inviare stato e scenario in broadcast.
 * Questa funzione viene sia chiamata in modo periodico da  statusRoutine() 
 * sia puo essere invocata in risposta al comando CMD_STATUS. 
 * 
 * Scelte progettuali:
 * 1. Nel caso di CMD_STATUS, la funzione non è da interpretare come risposta al comando (vedi responseCommand())
 *    bensì come l' 'azione'  che quel comando richiede.
 *  2. Usare outgoing_command[i] è orribile ma conveniente.
 *     Il fatto e' i messaggi di risposta ad un comando corrispondono con quelli di notifica dello stato e scenario.
 *     In alternativa basterebbe creare un buf dento la funzione che assempla il messaggio i.e 'stato on dormo' a seconda delle varibili lette
 */
void sendStatus( device_t dev) {    
    status_t currentStatus  = getStatus();
    scenario_t currentScenario = getScenario();


    for(int i=0; i< Nc; i++){
      if( command[i].status == currentStatus && command[i].scenario == currentScenario ) {



                            #ifdef TELNET_MODULE
                            if( dev == dev_TELNET){
                                _telnet(NULL, command[i].outgoing_message ); 
                            }
                            #endif

                            #ifdef GSM_MODULE
                            if( dev == dev_GSM ){
                               _sms(NULL, command[i].outgoing_message );
                            }
                            #endif

                            #ifdef COM_MODULE
                            if( dev == dev_COM ){
                               _serial(NULL, command[i].outgoing_message );
                            }
                            #endif
        
      }
    }
   
}     
/*
 * sendStatus è una azione che consiste nel risponde al clinet che ha inviato 'status' con un messaggio 
 * contenente la lista di tutti i sensori, l'ID associato, il nome mnemonico, il loro stato (on/off) e se sono esclusi dallo scenario corrente.
 */
void sendSensorsStatus(device_t dev){

    
    unsigned char buff[40];
    unsigned char stringTAB[] = " ";
    unsigned char stringEXC = 'x';
    unsigned char stringNULL = '\0';
    unsigned char * stringSTATUS;
    unsigned char stringID[2];
 
unsigned char stato_on[] = "on  ";
unsigned char stato_off[] = "off ";



     for(int i=0; i<Ns; i++){
         
          
           
            
          if( (sensor[i].ID) <10){
          stringID[0]='0';
          stringID[1] = (unsigned char)(sensor[i].ID) + '0';
          }
          if( (sensor[i].ID) >= 10 && (sensor[i].ID) < 20 ){
             stringID[0]='1';
           stringID[1] = (unsigned char)(sensor[i].ID - 10 ) + '0';
          }
           if( (sensor[i].ID) >= 20 && (sensor[i].ID) < 30 ){
             stringID[0]='2';
           stringID[1] = (unsigned char)(sensor[i].ID - 20) + '0';
          }
    
     
          if( sensor[i].state == false) {
           stringSTATUS = stato_off;
          }
          else if( sensor[i].state == true ) {
           stringSTATUS = stato_on;
          }
    
          int len = strlen(sensor[i].name);
          
          memcpy(buff,&stringID, 2);
          memcpy(buff+2,&stringTAB, 1);
          memcpy(buff+3,sensor[i].name,len);
          memcpy(buff+len+3,stringSTATUS,4);
              if( sensor[i].sensor_exclusion == true ){
              memcpy(buff+len+7, &stringEXC, 1);
              }
              else{
              memcpy(buff+len+7, &stringTAB, 1);
               }
          memcpy(buff+len+8, &stringNULL, 1);



          
          #ifdef TELNET_MODULE
          unsigned char buffMQTT[5];

          if( sensor[i].state == true &&  sensor[i].sensor_exclusion == false){
              memcpy(buffMQTT,"on", 2);
              memcpy(buffMQTT+2, "\0", 1);
           }
           if( sensor[i].state == true &&  sensor[i].sensor_exclusion == true){
              memcpy(buffMQTT,"onx", 3);
              memcpy(buffMQTT+3, "\0", 1);
           }
           if( sensor[i].state == false &&  sensor[i].sensor_exclusion == false){
              memcpy(buffMQTT,"off", 3);
              memcpy(buffMQTT+3, "\0", 1);
           }
           if( sensor[i].state ==false &&  sensor[i].sensor_exclusion == true){
              memcpy(buffMQTT,"offx", 4);
              memcpy(buffMQTT+4, "\0", 1);
           }
           
          if( dev == dev_TELNET){
          mqttPublish(sensorsOutTopic[i], (const char *)buffMQTT);
          // client.publish(topic[i],  (const char *)buff); 

          }
          #endif

          #ifdef COM_MODULE
          if( dev == dev_COM){
          _serial(NULL,  (const char *)buff);
          }
          #endif


         /* messo qui manderebbe tanti sms quanti sono i sensori...non molto bello
          *  #ifdef GSM_MODULE
          if( dev == dev_GSM ) {
            _sms(NULL, (const char *)buff);
          }
          #endif
          */
       }

        
       
}
   
void sendGsmStatus(device_t dev){
const char * buff;
const char gsm_not_defined[] = "gsm not defined\0";
const char gsm_ok[] = "gsm ok\0";
const char gsm_error[] = "gsm error\0";
const char gsm_no_signal[] = "gsm no signal\0";


buff = gsm_not_defined;

 #ifdef GSM_MODULE
 if (  gsm_AT_OK() == true ) {
  if( gsm_Signal_Strength() == true ) {
    buff = gsm_ok;
  }
  else{
    buff = gsm_no_signal;
  }
 }
 #endif

  
          #ifdef TELNET_MODULE
          if( dev == dev_TELNET){
          _telnet(NULL, (const char *)buff);
          }
          #endif

          #ifdef COM_MODULE
          if( dev == dev_COM){
          _serial(NULL,  (const char *)buff);
          }
          #endif


          #ifdef GSM_MODULE
          if( dev == dev_GSM ) {
            _sms(NULL, (const char *)buff);
          }
          #endif
}   
void printSensors(){
  
  for(int i=0; i<Ns; i++){
              Serial.print("\n\nSENSOR: ");
              Serial.print(sensor[i].ID);
              Serial.print("   type: ");
              Serial.print(sensor[i].type);
               Serial.print("   state: ");
                Serial.print(sensor[i].state);
                  Serial.print("   old state: ");
                Serial.print(sensor[i].old_state);
                Serial.print("   exclusion: ");
                Serial.print(sensor[i].sensor_exclusion);
               }
   
}
/*
 * Permette l'invio e la ricezione di messaggi tramite telnet.
 * Ritorna true se è presente un messaggio, fasle altrimenti.
 * 
 */

/* =================================================================== TELNET ( FUNCTIONS) =========================================== */









boolean _telnet(char * receive_buffer, const char * transmit_buffer){
  bool flag = false;
  mqttConnection();
   if( receive_buffer != NULL ){    
        if (  mqtt_has_received_data == true ) {
                flag = true;
             //  receive_buffer = buffer_incoming_command;
                    
              
               mqtt_has_received_data = false;
              // Serial.println(buffer_incoming_command);
              
           
        }
   }
  if( transmit_buffer != NULL ){
      mqttPublish(outTopic[0], transmit_buffer);
  }
  
  return flag;
}
void callback(char* topic, byte* payload, unsigned int length) {

  if(strcmp(topic, inTopic[0])==0){
                                  
             mqtt_has_received_data = true;

                 int i = 0;
                  for(i=0; i<length; i++) {
        
                            buffer_incoming_command[i] = (char)(payload[i]);
                           /*    Serial.print("\n");
                               Serial.print("rx:");
                               Serial.print(buffer_incoming_command[i]);        */                                               
                  }
                  buffer_incoming_command[i] = '\0';
    
   }
        
 // if(strcmp(topic, inTopic[1])==0){  }
}


void mqttConnection() {
  // add reconnection logics
  if (!client.connected()) {

    // connection to MQTT server
    if (client.connect("ArduinoMQTTAlarm")) //clientId, deviceId, deviceSecret))
    {
      Serial.println("[PHYSICAL] Successfully connected with MQTT");
    
     mqtt_new_status=true; //change to red
     mqttSubscribe(); // topic subscription
     
    }
    else{
      mqtt_new_status=false; //change to green
         //   Serial.println("bad");

    }  
  }

        if(mqtt_old_status==false && mqtt_new_status==true){
           Serial.println("MQTT: OK: reconnection succesful");

        }
         if(mqtt_old_status==true && mqtt_new_status==false){
          Serial.println("MQTT: ERROR: mqtt goes down");
        }
        mqtt_old_status=mqtt_new_status;
        
  
  client.loop();
}

void mqttSubscribe() {
    client.subscribe(inTopic[0]);
    client.loop();
 /* for (int i = 0; i < (sizeof(inTopic)/sizeof(int)); i++){
    client.subscribe(inTopic[i]);
    client.loop();
    Serial.print("subscribe: ");
    Serial.println(inTopic[i]);
  }*/
}
void mqttPublish(const char * topic,const char* messaggio) {
  // build the topic with the light item
   
    client.publish(topic, messaggio); 

}





































boolean _serial(char * receive_buffer,const char * transmit_buffer){

                static byte index = 0;
                char endMarker = '\n';
                char c;
                
                boolean rx=false;
                boolean data_available = false;
          if(receive_buffer != NULL){
                  while(Serial.available() > 0 && newData==false) // Don't read unless
                  {   
                          c = Serial.read(); // Read a character
                          if( c != endMarker){
                            receive_buffer[index] = c; // Store it
                            index++; // Increment where to write next
                            rx=true;
                                if(index >= 20){
                                  index=19;
                                }
                          }
                          else{
                             receive_buffer[index] = '\0'; // terminate the string
                             index = 0;
                             newData = true;     
                          }    
                  }

                  if(newData==true){
                     newData = false;
                   // cmd_t cmd = getCommand(receive_buffer);
                    //print_command(cmd);
                    data_available = true;
                  }
                  
                 
             }
    
    if( transmit_buffer != NULL ){
      Serial.print(transmit_buffer);
      Serial.print("\n\r");
    }
   return data_available;
}

 


 
/* =================================================================== GSM ( FUNCTIONS) =========================================== */
/*    
 *  Permette l'invio e la ricezione di sms.
 *   Verifica la presenza di un sms non letto, se è presente lo legge e lo salva nel bufffer_incoming_command. 
 *   Ritorna true quando è presente un nuovo sms, false altimentei.
 *  Nel caso di errore -1 e -2 incrementa un contatore che verrà poi letto da StutusRoutine()
 *  
 *  Note:
 *   dopo aver letto un sms, viene cancellato per evitare che si pieni la memoria.
 *   
 *  
 */
 #ifdef GSM_MODULE
void gsm_init(){

if (gsm.begin(9600)){
    _DEBUG("\nstatus=READY");
   _DEBUG("wait");
   
    }
  else {
    _DEBUG("\nstatus=IDLE");
    //gsmStatus();
   // error(ERROR_GSM);
    }

 }
 calling_t _call(){           
  
          _DEBUG("\nCALLING");
      
       char call_stato; 
       calling_t calling = CALLING_START;
       
        
      for( int i=0; i < N; i++){
        notified[i] == false;
      }
    
      unsigned int call_counter = N, call_target, call_responses=0, call_routine_counter=0, squilli_a_vuoto=0;  //dovento fare il modulo, è inutile partire da zero call_counter
  
      while(call_routine_counter <= call_routine_counter_max_loop){ //about 30 min timeout
              
              call_target = call_counter % N;                                 // 0..1..2..0..1..2..etc..identify family members. Each cycle has different family member.
              call_counter++;
              call_routine_counter++; //TODO: rimuovi call_counter, puoi usare action_routine _counter!

              if( call_routine_counter == call_routine_counter_max_loop ) calling = CALLING_TIMEOUT; // if it goes timeout, visit the FAIL state before leave the cycle.

              
              switch( calling){
                
                case CALLING_START: 
                     
                     
                     call_stato=call.CallStatusWithAuth(number,1,3);
                     
                     delay(120);
                     if( call_stato == CALL_NONE){
                         if( notified[call_target] == false ){
                              squilli_a_vuoto=0;
                              _DEBUG(F("\nCall: "));
                              _DEBUG(family[call_target].name);  
                              call.Call( family[call_target].SIMposition );
                                          
                                 while(1){ // prova a chiamare un membro della famiglia
                                    delay(1000);
                                    commandRoutine(); 
                                    call_stato=call.CallStatusWithAuth(number,1,3);
                                    squilli_a_vuoto++;    
                                   /*
                                    * La chiamata si interrompe se:
                                    *  1. il chiamato rispone.
                                    *  2. Si raggiunge il tempo massimo di squilli a vuoto
                                    *  3. è arrivato il comadno che spegne l'allarme
                                    *  
                                    *  Inoltre, tramite il vettore notified[] si tiene traccia di quelli che hanno risposto per evitare di chiamarli nuovamente
                                    */
                                          if(call.CallStatusWithAuth(number,1,3) == CALL_ACTIVE_VOICE){
                                              // se uno risponde prima del timeout, la linea torna libera
                                                notified[call_target] = true;
                                                _DEBUG(F("\n"));
                                                _DEBUG(family[call_target].name);
                                                _DEBUG(F(" has answer the phone"));
                                                ++call_responses;
                                                calling = CALLING_START; // volendo saltare la pausa tra una chiamata e la successiva allo stesso numero, basta mettere act=CALL
                                                delay(1000);
                                          break;
                                          }
                                          else if( squilli_a_vuoto > tempo_di_non_risposta){
                                                 call.HangUp();
                                                  _DEBUG(F("\n"));
                                                  _DEBUG(family[call_target].name);
                                                  _DEBUG(F(" hasn't answer the phone call in "));   
                                                  _DEBUG(F("tempo_di_non_risposta"));
                                                  _DEBUG(F(" secondi"));
                                                  calling = CALLING_START; //chiama il prossimo
                                                  delay(1000); //dai il tempo al gsm di chiudere la chiamata e tornare libero
                                           break;
                                           }
                                           else if ( STATUS_OFF == getStatus() ){
                                                  _DEBUG("\n Allarm shut down, then stop alarm");
                                                  calling = CALLING_STOP;
                                           break;
                                           }
                                             
                             
                             
                                 }//while
                         }
                                              
                          else{
                            if(call_responses == N ){
                              _DEBUG(F("\n Everyone has called"));
                              calling = CALLING_STOP;
                              // TODO: mettere qui meccanismo di notifica, tutti avvertiti o
                              //act = END; // if you want the siren to stop after everyone has been alerted. Otherwise
                            }
                        }
                      }
                      
                      else if( call_stato == CALL_NO_RESPONSE){
                        _DEBUG(F("\n ERROR. GSM DO NOT RESPONSE AT COMMAND."));
                       
                        calling = CALLING_FAIL;
                      }
                      else if( call_stato == CALL_INCOM_VOICE_NOT_AUTH ||  call_stato ==  CALL_INCOM_VOICE_AUTH  || CALL_COMM_LINE_BUSY){
                                      call.HangUp();
                                     _DEBUG(F("\n incoming call. hang up."));
                                      delay(120);
                       calling = CALLING_START;
                      }
                      else{
                          _DEBUG(F("\n ERROR. UNESPECTED BEHAVIOR OF CALL_STATO "));
                          calling = CALLING_FAIL;
                      }
            break;
            case CALLING_STOP:
                                return CALLING_STOP;
            break;

            case CALLING_FAIL:
                               return CALLING_FAIL;
            break;
            }
      }
      return calling;

}    


    
boolean _sms(char * receive_buffer, const char * transmit_buffer){

 boolean data_available = false;
  char SMS_position;
  char res = sms.IsSMSPresent(SMS_UNREAD);
  delay(110);
  if( res > 0 ) {
  SMS_position = res;
  sms_unread = true;
  }
  else if( res == 0 ) {
  sms_unread = false;
  }
  else if ( res == -1 || res == -2 ){
   // return GSM_ERROR;
   //function_error();
  // error(ERROR_GSM);
   //return;
   //gsm_error++;
  }
  
  
 
// TODO butta via cmd  nell header
  if( sms_unread == true  && receive_buffer != NULL ) {
    
         
           //sms_count++;
           char sms_state = sms.GetAuthorizedSMS(SMS_position, number, receive_buffer,20, 1, 3);
           delay(110);
            _DEBUG("\n ricevuto sms:");
            _DEBUG(receive_buffer); 
            _DEBUG("\n      SMS position:"); 
            Serial.print(SMS_position, HEX); 
           if ( sms_state == GETSMS_AUTH_SMS ) {
                    data_available = true;
                    _DEBUG("\n      AUTH:");
                    _DEBUG(number);
                    
           }
           else if ( sms_state == GETSMS_NOT_AUTH_SMS ) {
                    data_available = false; //do not read it.       
           }
        /*  if(0 == sms.DeleteSMS(SMS_position) ){
            //AT+CMGDA delete all sms
            Serial.print("\n      delete sms from sim.");
          }*/
           sms_response_unlock = true;  
          //.SendSMS(number, (char*)"OK");
  
  }
  
  if( sms_response_unlock == true  && transmit_buffer != NULL){
            sms_response_unlock = false;
            _DEBUG("\n     inviato sms:");
            _DEBUG((char*)transmit_buffer);
           sms.SendSMS(number, (char*)transmit_buffer);
         
  }
return data_available;

}


boolean gsm_Signal_Strength()
{ 
  
  
  
  int c = 0;
  char r[10];
  char db[2];
 // Serial1.write("AT+CMGF=1\r"); //set GSM to text mode
 // delay (150);  
  Serial1.write("AT+CSQ\r"); //Send command for signal report
  delay (200);   
  // expect resp: +CSQ: 10,0 //10 char exactly
  while(Serial1.available() > 0)  { //wait for responce
   r[c++] = Serial1.read();
   
  }
  
  int one = r[6] - '0';
  int two = r[7] - '0';

  
 int signal_modulo = one*10 + two;
 _DEBUG("Signal strength:"+signal_modulo);
 
 if( signal_modulo > 80 ) {
  return false;
 }
 else {
  return true;
 }

}

 boolean gsm_AT_OK(){
  
  
   Serial1.write("AT\r");
   int t = 0;
  
     while( Serial1.available() > 0 ){
           delay(100);
          t++;
         if( Serial1.read() == 'O' ||  Serial1.read() == 'K'){
          //gsm_status = GSM_OK;
          _DEBUG("GSM OK");
                
          return true;
        }
        
     }
     return false;
     
 }
 #endif








void error( error_t error){
            
          // _blink(BLINK_ERROR, HIGH );
        // if( error == ERROR_NONE){
         // error_counter = 0;
        //  digitalWrite(LED_TELNET_ERROR, LOW );
        //  digitalWrite( LED_GSM_ERROR, LOW );
        //  return;
         // analogWrite(LED_ACK, 0);
         // analogWrite(LED_GSM, 0);
         // analogWrite(LED_
       //  }
          
            //  t.update();                                 
                    switch(error){
                     
                      case ERROR_ACK:
                                                       #ifdef DEBUG_ERROR
                                                        Serial.print(F("\n ERROR_TELNET"));
                                                       #endif
                                                       
                                                      //if( telnet_reconnection() == true){
                                                       
                                                       //  _blink(, LOW);
                                                        // return;
                    
                                                      //}
                                                     
                                                      
                                                     
                      break;
                  
                      case ERROR_GSM:                 
                                                       #ifdef DEBUG_ERROR
                                                       Serial.print(F("\nGSM error"));
                                                       #endif
                                                       /*
                                                        * TODO NEED WIRING THE PWR BUTTON ON GSM
                                                        */
                                                      
                                                       //software_Reset();
                                                     
                  
                      break;  
                                              
                                                       
                      /*case ERROR_ALARM:               
                                                        #ifdef DEBUG_ERROR
                                                        Serial.print(F("\n INTRUSION DETECTE BUT NOBODY ANSWERD THE PHONE CALL OR SHUT DOWN, SO THE SYSTEM GOES TIMEOUT IN 30 min"));
                                                        
                                                        #endif
                                                        //analogWrite(LED_ALARM_ERROR, 255);
                                                        digitalWrite(LED_
                      break;*/
                      case ERROR_ACTION:
                                                       #ifdef DEBUG_ERROR
                                                             Serial.print(F("\nERROR ACTION"));
                                                          #endif
                                                      
                             
                      break;
                      default:
                                                         #ifdef DEBUG_ERROR
                                                             Serial.print(F("\nSOME UNKNOW ERROR OCCURS"));
                                                          #endif
                      break;
                     
                      
                    }
      


         if( error_counter >  100) {
        //  software_Reset();
         }
         return;
}

void _blink(blink_t blink_status, uint8_t val){
  switch(blink_status){
      case LED_STATUS_ALARM:
          digitalWrite(LED_ALARM_ON, !val);
          digitalWrite(LED_ALARM_OFF, val);
     break;
    
     case LED_STATUS_GSM:
           digitalWrite(LED_ERROR_GSM, val);
     break;
      case LED_STATUS_ACK: 
            digitalWrite(LED_ERROR_ACK, val);
      break;
      case LED_STATUS_ACTION:
          digitalWrite(LED_ERROR_ACTION, val);
       break;
  }
}

void print_command( cmd_t cmd){
//  Serial.print("\nOUTGOING CMD:");
  switch(cmd){
  case CMD_ERROR:                      Serial.print(F("\nCMD_ERROR"));
  break;
  case CMD_NONE:                       Serial.print(F("\nCMD_NONE"));
  break;
  case CMD_WRONG:                      Serial.print(F("\nCMD_WRONG"));
  break;
  case CMD_STATUS:                     Serial.print(F("\nCMD_STATUS"));
  break;
  case CMD_OFF:                        Serial.print(F("\nCMD_OFF"));
  break;
  case CMD_ON:                         Serial.print(F("\nCMD_ON"));
  break; 
  case CMD_ON_SCENARIO_DORMO:          Serial.print(F("\nCMD_ON_SCENARIO_DORMO"));
  break;   
  case CMD_ON_SCENARIO_ESCO:           Serial.print(F("\nCMD_ON_SCENARIO_ESCO"));
  break;
  case CMD_ON_SCENARIO_VACANZA:        Serial.print(F("\nCMD_ON_SCENARIO_VACANZA"));
  break;
  case CMD_NOT_AUTH:                   Serial.print(F("\nCMD_NOT_AUTH"));  
  break;
  }
}
void print_status(status_t status){
  switch(status){
   case STATUS_ON:                     Serial.print(F("\nSTATUS_ON"));
   break;
   case STATUS_ON_SCENARIO_DORMO:      Serial.print(F("\nSTATUS_ON_SCENARIO_DORMO"));
   break;
   case STATUS_ON_SCENARIO_ESCO:       Serial.print(F("\nSTATUS_ON_SCENARIO_ESCO"));
   break;
   case STATUS_ON_SCENARIO_VACANZA:    Serial.print(F("\nSTATUS_ON_SCENARIO_VACANZA"));
   break;
   case STATUS_OFF:                    Serial.print(F("\nSTATUS_OFF"));
   break;
   case STATUS_UNDEF:                  Serial.print(F("\nSTATUS_UNDEF"));       
   break;
  }
}
void print_action(action_t act){
  switch(act){
  case START:                          Serial.print(F("\nACTION_START"));      
  break;
  case INIT:                          Serial.print(F("\nACTION_INIT"));     
  break;
  case CALL:                          Serial.print(F("\nACTION_CALL"));     
  break;
  case END:                           Serial.print(F("\nACTION_END"));     
  break;
  case FAIL:                        Serial.print(F("\nACTION_FAIL"));     
  break;
  }
}

void siren(boolean siren_status){
  if(siren_status == true ){
         digitalWrite(siren_pin, HIGH);
  }
  else{
    digitalWrite(siren_pin, LOW);
  }
  
}
void beep(int f){
  /*if ( f == 0 || f==20 || f==40 || f==60 
        || f==70 || f==80 || f==90 || f==100 || f==110 
        || f==120 || f == 125 || f == 130 || f == 140 || f == 145
        || f == 148 || f==150 ) {
          analogWrite(46, 255); delay(200); analogWrite(46,0);
        }*/

             if( f == action_routine_counter_max_loop ) {
            //  Serial.print("BUZZ0");
          analogWrite(buzzer_pin,0);
          return;
        }
               if( f == (delay_before_start_call-1) ) {
            analogWrite(buzzer_pin,0);
            // Serial.print("BUZZ0");
            return;
          }
    if ( f == 0  || f==20 || f==40
        || f==60 || f==80 || f==100  ) {
          analogWrite(buzzer_pin, 255);
           Serial.print("BUZZ1");
        }   
   if ( f == 10 || f==30|| f==50 || f==70
      || f==90 || f == 110 ) {
          analogWrite(buzzer_pin,0);
          // Serial.print("BUZZ0");
        }
}

void _DEBUG(String debugString){
#ifndef DEBUG 
  return;
#else
   Serial.print(debugString);
   return;
#endif
}


void _DEBUGINT(int debugString){
#ifndef DEBUG
  return;
#else
   Serial.print(debugString);
   return;
#endif
}

void action() {
  for ( int i = 0; i<20; i++){
    Serial.print(" THIS IS SPARTA! ");
  }
}
void lights(boolean value){
  
}
















void setup() 
{  
 ack_counter = 0;   
  Serial.begin(9600);

Serial.print("\nALARM TEST\n");

  Serial.print("\n N number of call: "); Serial.print(N);
   Serial.print("\n Ns number of sensor: "); Serial.print(Ns);
  

pinMode(LED_ALARM_ON, OUTPUT);
pinMode(LED_ALARM_OFF, OUTPUT);

pinMode(LED_ERROR_GSM, OUTPUT);
pinMode(LED_ERROR_ACK, OUTPUT);
pinMode(LED_ERROR_ACTION, OUTPUT);


pinMode(siren_pin, OUTPUT);
#ifdef GPIO_MODULE
  pinMode(alarm_interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(alarm_interrupt), switchAlarm, CHANGE);

#endif
  delay(1000);
  
//  int r = t.every(60000,  statusRoutine , (void*)0);

  //memset(buffer_incoming_command_, 0, 20);

  /*===============================================        ALARM SKELETON   ( SETUP )      ====================================================*/


      if( Ns >= 32) {
      //  _DEBUG("error: too many sensors, i pin del mega non bastano!");
        return; 
      }
     for(int i = 0; i < Ns; i++){
      // sensor[i].ID = (i+1);
      // sensor[i].pin = (i+22);  
       pinMode(sensor[i].pin, INPUT_PULLUP);     // when pullup, read LOW when closed to GND and HIGH when the wiring is floating without reference.
      // sensor[i].type = get_sensor_type(sensor[i].ID);
       delay(100);
      if( digitalRead(sensor[i].pin) == LOW){  
       
      //  _DEBUG2("\nOLD STATE FALSE");
       sensor[i].old_state = false; //normal state, the contact is close
      }
      else{
     //     _DEBUG2("\nOLD STATE TRUE");
        sensor[i].old_state =true;
      }
      }

  /*===============================================        TELNET   ( SETUP )      ====================================================*/
  
/*  Serial.print("\nInitialize ethernet shield");
  // initialize the ethernet device
  Ethernet.begin(mac, ip, gateway, subnet);
  // start listening for clients
  server.begin();
  // Open serial communications and wait for port to open:
  Serial.println(Ethernet.localIP());
      */

/*===============================================       GSM   ( SETUP )      ====================================================*/
//gsmConnect();
       //pinMode(9, OUTPUT); 
      // pinMode(, OUTPUT);
       /*  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  delay(500);*/


#ifdef TELNET_MODULE
Ethernet.begin(mac, ip);
delay(100);
mqttConnection();           
#endif

  
#ifdef GSM_MODULE
 gsm_init();
#endif

  Serial.println("\n\nSTART LOOP\n\n");

         
}



/*
 * Il funzionamento delo sketch  è piuttosto semplice. 
 * Un sitema di allarme prevede due principali insiemi di azioni. 
 * Da un lato la lettura dei sensori e dall'altro l'interazione con l'utente.
 * L'interazione con l'utente è realizzata dalla commandRoutine(). Questa routine 
 * 1. verifica la presenza di comandi che l'utente ha inviato all'allarme o tramite Telnet o tramite sms.
 * 2. interpreta il significato di questi comandi ed effettua le azioni ad essi associate. 
 *    In particolare, l'attivazione dell'allarme e la selezione di uno scenario consistono nello scrivere in EEPROM i valori a cui si riferiscono.
 * 3. Risponde ai comandi inviati   
 * 
 * L'altra routine, allarmRoutine() si occupa di: 
 * 1. leggere i sensori
 * 2. leggere la EEPROM per conoscere lo stato dell'allarme (accesso/spento)
 * 3. se l'allarme è attivo e un sensore è passato da chiuso (off) ad aperto (on) 
 *    chiama actionRoutine() che si occupa delle azioni da svolgere in caso di rilevazione dell'intuso.
 * 
 */




void loop() 
{  
  


      
      alarmRoutine();
      commandRoutine();
     // statusRoutine();
      
      t.update();
   
      
} // end loop



