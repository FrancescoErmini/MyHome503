
#include "Timer.h"
#include <EEPROM.h>

#include <SPI.h>
#include <Ethernet.h>

#include "SIM900.h"

//#include "inetGSM.h"
#include "sms.h"
#include "call.h"

#include "Time.h"

#define DEBUG_CMD
#define DEBUG
#define DEBUG_TELNET 1
#define DEBUG_NFC 1 
#define DEBUG_ERROR 1

byte mac[] = { 
  0xEE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
  
IPAddress ip(192,168,1, 101);
IPAddress gateway(192,168,1, 1);
IPAddress subnet(255, 255, 255, 0);

EthernetServer server(23);
EthernetClient clients[4];

Timer t;

CallGSM call;
SMSGSM sms;

int alarm_interrupt = 20;                 //button to test alarm insertion

const int buzzer_pin = 46;

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
  TEST,
  SCENARIO_UNDEF
};

enum device_t {                           
  dev_TELNET,
  dev_GSM,
  dev_KEYPAD,
  dev_SERVER,
  dev_null
};


enum action_t {
  DELAY,
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


const char * sensor_name[] = { "ing: porta  ",
                                "ing: lavand ",
                                "ing: garage ",
                                "mez: bagno  ",
                                "mez: studio ",
                                "mez: camera ",
                                "mez: volum  ",
                                "tav: portne ",
                                "tav: prtfin ",
                                "tav: volum  ",
                                "tav: finstr ",
                                "app: cucina ",
                                "app: volum  ",
                                "app: bagno  ",
                                "app: camera "                                
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
  CMD_SENSORS_STATUS
};

const int BUFSIZE = 20;
char buffer_incoming_command[BUFSIZE];                         // where to store incoming command
const char * incoming_command[] = {"status",//CMD_STATUS              // possibles commands received from the server
                                    "off",//CMD_OFF
                                    "on",//CMD_ON
                                    "on dormo",//CMD_ON_SCENARIO_DORMO
                                    "on esco",//CMD_ON_SCENARIO_ESCO
                                    "on vacanza",//CMD_ON_SCNEARIO_VACANZA
                                    "errore",//CMD_ERROR
                                     "1" , //CMD_ACK               //resp to ACK
                                     "sensori"//CMD_SENSOR_STATUS
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
      const char * incoming_message;  
      const char * outgoing_message;   
                        
};

const command_t command[] = {
                             { .cmd = CMD_STATUS,              .incoming_message = incoming_command[0], .outgoing_message = outgoing_command[0]},
                             { .cmd = CMD_OFF,                 .incoming_message = incoming_command[1], .outgoing_message = outgoing_command[1] },
                             { .cmd = CMD_ON,                  .incoming_message = incoming_command[2], .outgoing_message = outgoing_command[2] },
                             { .cmd = CMD_ON_SCENARIO_DORMO,   .incoming_message = incoming_command[3], .outgoing_message = outgoing_command[3] },
                             { .cmd = CMD_ON_SCENARIO_ESCO,    .incoming_message = incoming_command[4], .outgoing_message = outgoing_command[4] },
                             { .cmd = CMD_ON_SCENARIO_VACANZA, .incoming_message = incoming_command[5], .outgoing_message = outgoing_command[5] },
                             { .cmd = CMD_ERROR,               .incoming_message = incoming_command[6], .outgoing_message = outgoing_command[6] },
                             { .cmd = CMD_WRONG,               .incoming_message = incoming_command[0], .outgoing_message = outgoing_command[7] },
                             { .cmd = CMD_ACK,                 .incoming_message = incoming_command[7], .outgoing_message = outgoing_command[0] },
                             { .cmd = CMD_SENSORS_STATUS,      .incoming_message = incoming_command[8], .outgoing_message = outgoing_command[0] }

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

                                   
/* ===========================================================  GSM (SMS)======================================================*/
                                   
/*
 * GSM BASED. 
 * 
 */
//char buffer_incoming_command_sms[100];                         // where to store incoming command

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
 */
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
     
       Serial.print("\nALARM ACTION");
       action_t act = DELAY;
       char call_stato; 
     
      for( int i=0; i < N; i++){
        notified[i] == false;
      }
    
      unsigned int call_counter = N, call_target, call_responses=0, action_routine_counter=0, squilli_a_vuoto=0;  //dovento fare il modulo, è inutile partire da zero call_counter
  
      while(action_routine_counter < action_routine_counter_max_loop){ //about 30 min timeout
              
              call_target = call_counter % N;                                 // 0..1..2..0..1..2..etc..identify family members. Each cycle has different family member.
              call_counter++;
              action_routine_counter++; //TODO: rimuovi call_counter, puoi usare action_routine _counter!

              if( action_routine_counter == action_routine_counter_max_loop ) act = FAIL; // if it goes timeout, visit the FAIL state before leave the cycle.

              
              switch(act){
                case DELAY:   //tempo di ritardo prima di iniziare a chiamare.
                      Serial.print(" . ");   
                      for(int i=0; i<delay_before_start_call; i++){ // it's about 15-20 sec delay time before start calling
                          commandRoutine();                         // ci sono stati comandi che spengono l'allarme?
                            //blink the red led
                            if( STATUS_OFF == getStatus() ){
                                  Serial.print("\n Allarm shut down, then stop alarm");
                                  act = END;
                                  beep(action_routine_counter_max_loop);   // serve a spegnere il beep quando si disattiva l'allarme prima del delay.
                            break;
                            }
                            else{
                                  Serial.println("WAIT");
                                  beep(i);             //emetti un suono intermittente. Vedi beep()
                                  act = CALL;
                            }
                      }
                     // siren(true);
                break;
                case CALL: 
                     
                     call_stato=call.CallStatusWithAuth(number,1,3);
                     delay(120);
                     if( call_stato == CALL_NONE){
                         if( notified[call_target] == false ){
                              squilli_a_vuoto=0;
                              Serial.print(F("\nCall: "));
                              Serial.print(family[call_target].name);  
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
                                                Serial.print(F("\n"));
                                                Serial.print(family[call_target].name);
                                                Serial.print(F(" has answer the phone"));
                                                ++call_responses;
                                                act = CALL; // volendo saltare la pausa tra una chiamata e la successiva allo stesso numero, basta mettere act=CALL
                                                delay(1000);
                                          break;
                                          }
                                          else if( squilli_a_vuoto > tempo_di_non_risposta){
                                                 call.HangUp();
                                                  Serial.print(F("\n"));
                                                  Serial.print(family[call_target].name);
                                                  Serial.print(F(" hasn't answer the phone call in "));   
                                                  Serial.print(tempo_di_non_risposta);
                                                  Serial.print(F(" secondi"));
                                                  act = CALL;  //chiama il prossimo
                                                  delay(1000); //dai il tempo al gsm di chiudere la chiamata e tornare libero
                                           break;
                                           }
                                           else if ( STATUS_OFF == getStatus() ){
                                                  Serial.print("\n Allarm shut down, then stop alarm");
                                                  act = END;
                                           break;
                                           }
                                             
                             
                             
                                 }//while
                         }
                                              
                          else{
                            if(call_responses == N ){
                              Serial.print(F("\n Everyone has called"));
                              // TODO: mettere qui meccanismo di notifica, tutti avvertiti o
                              //act = END; // if you want the siren to stop after everyone has been alerted. Otherwise
                            }
                        }
                      }
                      else if( call_stato == CALL_NO_RESPONSE){
                        Serial.print(F("\n ERROR. GSM DO NOT RESPONSE AT COMMAND."));
                       
                        act = FAIL;
                      }
                      else if( call_stato == CALL_INCOM_VOICE_NOT_AUTH ||  call_stato ==  CALL_INCOM_VOICE_AUTH  || CALL_COMM_LINE_BUSY){
                                      call.HangUp();
                                     Serial.print(F("\n incoming call. hang up."));
                                      delay(120);
                        act = DELAY;
                      }
                      else{
                          Serial.print(F("\n ERROR. UNESPECTED BEHAVIOR OF CALL_STATO "));
                          act = FAIL;
                      }
            break;
            
             case END:       
                           siren(false);
                           Serial.print(F("\n OK. ALARM ACTION LOOP STOP SUCCESS "));
                           error(ERROR_NONE);
             return;
             break;                                
             case FAIL:         
                              siren(false);
                              Serial.print(F("\n ERROR. ALARM ACTION LOOP STOP FAIL  "));

                              error(ERROR_ACTION);
             return;  
             break;
             /*
              * TODO: default:
              */
            }//switch
     }//while
                           
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
      cmd_t cmd;
      device_t dev;
      status_t status;
      scenario_t scenario;
     
      //// A
          // 1. if receive a message from telnet, store it in the incoming command buffer and return true
          if ( _telnet(buffer_incoming_command, NULL) == true ){  
                  // 2. transform the message received in the corresponding  type cmd_t
                 cmd = getCommand(buffer_incoming_command); 
                  // 3. set the device which receiced that command
                 dev = dev_TELNET;                             
           }
           else if( _sms(buffer_incoming_command, NULL) == true ) {
                  cmd = getCommand(buffer_incoming_command); 
                  dev = dev_GSM;
                 // Serial.print("\nSMS:");
                  //Serial.print(buffer_incoming_command);
                  
           }
           else {
                cmd = CMD_NONE;
                dev = dev_null;
           }
        
           
           //// B take action specifeied by the command received
           if(  cmd != CMD_NONE ){
                     switch(cmd){
                          case CMD_STATUS:
                                  status = getStatus();
                                  scenario=getScenario();
                                   sendStatus(status, scenario, dev); 
                                  //alarm_status = getAlarm(&scenario);
                                  Serial.print("\n CMD_STATUS"); 
                               //   statusRoutine();
                                  
                          break;
                          case CMD_OFF:
                                   setStatus(STATUS_OFF);
                                  // resp = responseCommand(CMD_OFF);
                                  // configScenario( INDEFINITO, dev);
                                   Serial.print("\n CMD_OFF");
                                  
                          break;
                          case CMD_ON: // if no scenario is specified, then choose the scenario depends from the  device
                                    setStatus(STATUS_ON);
                                    if( dev == dev_TELNET )   {       
                                      setScenario(SCENARIO_ESCO); 
                                      configScenario(SCENARIO_ESCO);
                                     // resp = responseCommand(CMD_ON_SCENARIO_ESCO);
                                      Serial.print("\n CMD_ON auto SCENARIO_ESCO");
                                      }
                                    else if( dev == dev_GSM )  {   
                                      setScenario(SCENARIO_VACANZA); 
                                      configScenario(SCENARIO_VACANZA); 
                                     // resp = responseCommand(CMD_ON_SCENARIO_VACANZA);
                                      Serial.print("\n CMD_ON auto SCENARIO_VACANZA");
                                      }
                                    else if( dev == dev_SERVER ) { 
                                      setScenario(SCENARIO_DORMO);
                                      configScenario(SCENARIO_DORMO); 
                                     // resp = responseCommand(CMD_ON_SCENARIO_DORMO);
                                      Serial.print("\n CMD_ON auto SCENARIO_DORMO");
                                      }
                                   
                          break;
                          case CMD_ON_SCENARIO_DORMO: 
                                  setStatus(STATUS_ON);                                 // 1.  set the alarm in status ON (write in EEPROM)
                                  setScenario( SCENARIO_DORMO);                         // 2. set the scenario (write in EEPROM)
                                  configScenario(SCENARIO_DORMO);                       // 3. exclude sensors depends on scenario
                                  //resp = responseCommand(CMD_ON_SCENARIO_DORMO);        // 4. choose the response to this command
                                  Serial.print("\n CMD_ON_SCENARIO_DORMO");
                          break;
                          case CMD_ON_SCENARIO_ESCO: 
                                    setStatus(STATUS_ON);
                                    setScenario( SCENARIO_ESCO);
                                    configScenario(SCENARIO_ESCO);
                                   // resp = responseCommand(CMD_ON_SCENARIO_ESCO);
                                    Serial.print("\n CMD_ON_SCENARIO_ESCO");
                          break;
                          case CMD_ON_SCENARIO_VACANZA: 
                                    setStatus(STATUS_ON);
                                    setScenario( SCENARIO_VACANZA);
                                    configScenario(SCENARIO_VACANZA);
                                   // resp = responseCommand(CMD_ON_SCENARIO_VACANZA);
                                    Serial.print("\n CMD_ON_SCENARIO_VACANZA");
                          break;
                          case CMD_ACK: 
                                     Serial.print("\n CDM_ACK");
                                     ack_counter=0;                                       // received ack, then restore ack_counter at zero
                                    // resp = responseCommand(CMD_ACK);
                          break;
                          case CMD_WRONG: 
                                    //  resp = responseCommand(CMD_WRONG);
                                       // no action needed..
                          break;
                          case CMD_SENSORS_STATUS:
                                      //  resp = responseCommand( CMD_SENSORS_STATUS );
                                        sendSensorsStatus();                            // assemble a list of all sensor and send ID,name and status on/off only via telnet
                                  
                          break;
                          
                        /*  default:  is_command = false;
                          break;*/
                     }
                     
                      //// C
                    /* if( cmd==CMD_STATUS || cmd==CMD_OFF || cmd==CMD_ON_SCENARIO_DORMO ||  cmd==CMD_ON_SCENARIO_ESCO ||  cmd==CMD_ON_SCENARIO_VACANZA){
                         status = getStatus();
                         scenario = getScenario();                     
                         resp = responseStatus(status, scenario);  // get which output message must be in response at the cmd received
                      }
                      else if( cmd == CMD_WRONG ) resp = 6;
                      */
                      const char * rispondi = responseCommand(cmd);
                    
                       if( rispondi != outgoing_command[0] ) {
                       // Serial.print("\n transmit something");
                               switch(dev){
                               // 5. response to the command received if the command have sense
                               case dev_TELNET: 
                                              _telnet(NULL, rispondi); 
                               break;
                               case dev_GSM: //Serial.print("\n now trqansmit:"); Serial.print(rispondi);
                                              _sms(NULL, rispondi );
                               break;
                              }                        
                       }
           
            }
}


/*
 * Scelte progettualu:
 * 1. Il ocntrollo degli errori viene fatto ogni minuto e per non interrompere il codice nel suo funzionameto normale. 
 *   Solo dopo più controlli falliti al check periorico si fa scattare l'errore. Lo svantaggio è che i tempi per rilevare l'errore sono
 *   di qualche minuto. D'altra parte le normali funzioni sono più snelle e veloci.
 *    
 *   gli errori controllati sono relativi al telent e al gsm.
 *   Eventuali errori telent sono riconosciuti attraverso un contatore ack. Periodicamente (ogni minuto) l'allarme manda un ack, inviando un messaggio con lo stato attuale in broadcast.
 *   L'arduino che gestisce l'NFC ( che è sempre attivo) risponde inviando il sui ID. 
 *   Tutte le volte che viene ricevuto un messaggio di quel tipo ( '1' associato a CMD_ACK) il contatore ack viene resettato a zero.
 *   La funzione statuwRoutine ogni minuto incremente il contatore di 1. Per cui se non riceve ack dopo 3 minuti, si puà dedurre che c'è un problema sulla connessione telnet.
 *   Non è dato sapere se questo problema sia sul 'server' o sul 'clinet', ma l'uso dell'ack è un modo sicuro per trovare u errore sul telnet.
 *   
 *   Gli errori del GSM sono rilevati diversamente. La funzione _sms() viene continuamente chiamata dalla commandRoutine() per sapere se ci sono sms non letti. 
 *   Se il modulo GSM non funziona alla funzione readSMS ritorna -1 o -2. Tutte le volte che questo accade vien incrementato un contatore error_gsm. 
 *   Ogni minuto viene letto lo stato di questo contatore. Se supera la solgia di 2, allora si resetta e si segnale l'errore sul gsm.
 *   
 */
void statusRoutine(){                               
       
     //  ack_counter++;
       //error_counter++;
      
       
    /*   if( ack_counter > 2 ) {
          Serial.print("\nLOOSE ACK");
          _blink( LED_STATUS_ACK, HIGH);
           error(ERROR_ACK);
          
        }
        else {
           //error_counter = 0;
          _blink( LED_STATUS_ACK, LOW);
        }

     */   

        status_t status = getStatus();
        scenario_t scenario = getScenario();
             
        if( status == STATUS_ON){
           digitalWrite(LED_ALARM_ON,  HIGH);
           digitalWrite(LED_ALARM_OFF, LOW);          
        }
        else if (status == STATUS_OFF){
           digitalWrite(LED_ALARM_ON,  LOW);
           digitalWrite(LED_ALARM_OFF, HIGH);   
        }
        else{
           digitalWrite(LED_ALARM_ON,  LOW);
           digitalWrite(LED_ALARM_OFF, LOW);   
        }
            
       sendStatus(status, scenario, dev_TELNET);


     if( gsmStatus() == true ) {
        Serial.print("\nGSM is OK");
     }
     else{
        Serial.print("\nGSM is ERROR");
     }
        


     
  /*      if ( gsm_error_counter > 2 ){
           gsm_error_counter = 0;
        // Serial.print("\nLOOSE ACK");
          _blink( LED_STATUS_GSM, HIGH );
          error(ERROR_GSM);
         
        }
        else {
           //error_counter = 0;
          _blink( LED_STATUS_GSM, LOW);
        }*/
       
  
       
        
}

void switchAlarm(){
 
        delay(1);
        if(digitalRead(alarm_interrupt) == LOW ){
          _DEBUG2("\nSet alarm true:");
          setStatus(STATUS_ON);
          setScenario(SCENARIO_PREDEFINITO);
          configScenario(SCENARIO_DORMO);
        }
        else{
          _DEBUG2("\nSet alarm false:");
          setStatus(STATUS_OFF);
          
          configScenario(SCENARIO_UNDEF);
        }
        
        
        return;
}


void setup() 
{  
 ack_counter = 0;   
  Serial.begin(115200);

Serial.print("\nALARM TEST\n");

  Serial.print("\n N number of call: "); Serial.print(N);
   Serial.print("\n Ns number of sensor: "); Serial.print(Ns);
  

pinMode(LED_ALARM_ON, OUTPUT);
pinMode(LED_ALARM_OFF, OUTPUT);

pinMode(LED_ERROR_GSM, OUTPUT);
pinMode(LED_ERROR_ACK, OUTPUT);
pinMode(LED_ERROR_ACTION, OUTPUT);



  pinMode(alarm_interrupt, INPUT);
  attachInterrupt(digitalPinToInterrupt(alarm_interrupt), switchAlarm, CHANGE);
  delay(1000);
  
  int r = t.every(60000,  statusRoutine , (void*)0);

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
  
  Serial.print("\nInitialize ethernet shield");
  // initialize the ethernet device
  Ethernet.begin(mac, ip, gateway, subnet);
  // start listening for clients
  server.begin();
  // Open serial communications and wait for port to open:
  Serial.println(Ethernet.localIP());
      

/*===============================================       GSM   ( SETUP )      ====================================================*/
if (gsm.begin(9600)){
    _DEBUG("\nstatus=READY");
    }
  else {
    _DEBUG("\nstatus=IDLE");
   // error(ERROR_GSM);
    }

  

         
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
       
         t.update();
   
      
} // end loop

   

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

}
/*
 * E' possibile attivare l'allarme senza specificare lo scenario. 
 * In questo caso lo scenario verrà automaticamete selezionato in base al tipo di dispositivo.
 * E' verosimile che si si attiva l'allarme da sms siamo fuori casa, da qui SCENARIO_VACANZA.
 * Se invece si invia i comandi tramite telent sullo smartphone si può pensare che l'utente sia dentro casa (rete LAN), SCENARIO_DORMO.
 * Quello che qui non si vede è che l'NFC di default  manda il comando SCENARIO_ESCO. 
 * In tal caso la configurazione automatica non viene fatta sul server ma sul clinet.
 */
scenario_t getDefaultScenario( device_t device) {
     
      if( device == dev_TELNET){
       return SCENARIO_DORMO;
      }
      else if( device == dev_GSM){
        return  SCENARIO_VACANZA;
      }
      else if( device == dev_SERVER){
        return  SCENARIO_DORMO;
      }
      else if( device == dev_KEYPAD || device == dev_null){
        return  TEST;
      }
}  
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
      case TEST:  for(int i=0; i<Ns; i++){
                         sensor[i].sensor_exclusion = true;
                   }
                        sensor[0].sensor_exclusion = false;
      break;
    }
 #ifdef DEBUG
 printSensors();
 #endif
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
cmd_t getCommand(char * message){
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
      return command[i].cmd;
      print_command(command[i].cmd);
    }
     
  }
  return CMD_WRONG; //no commands match this message txt
}
/*
 * Associa al comando cmd (ricevuto e già interpretato) il messaggio che deve essere inviato in risposta a quel comado.
 * Tale associazione è definta nell header in struct command_t command.
 * Nel caso un comando non necessiti di avere una risposta command[i].outgoing_message è NULL ( vedi coomand struct nell header). 
 */
const char * responseCommand(cmd_t cmd){
 
  for(int i=0; i<Nc; i++){
    if( command[i].cmd == cmd ) {
      
      return command[i].outgoing_message;
    }
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
void sendStatus(status_t currentStatus, scenario_t currentScenario, device_t dev) {      
     if( dev == dev_TELNET){                   
        if( currentStatus == STATUS_OFF ){
          _telnet(NULL, outgoing_command[1] );
        }
        else if(currentStatus == STATUS_ON && currentScenario == SCENARIO_DORMO){
           _telnet(NULL, outgoing_command[3] );
        }
        else if(currentStatus == STATUS_ON && currentScenario == SCENARIO_ESCO){
           _telnet(NULL,outgoing_command[4] );
        }

        else if(currentStatus == STATUS_ON && currentScenario == SCENARIO_VACANZA){
           _telnet(NULL, outgoing_command[5] );
        }
     }
     else if( dev == dev_GSM ){
           if( currentStatus == STATUS_OFF ){
          _sms(NULL, outgoing_command[1] );
        }
        else if(currentStatus == STATUS_ON && currentScenario == SCENARIO_DORMO){
           _sms(NULL, outgoing_command[3] );
        }
        else if(currentStatus == STATUS_ON && currentScenario == SCENARIO_ESCO){
           _sms(NULL,outgoing_command[4] );
        }

        else if(currentStatus == STATUS_ON && currentScenario == SCENARIO_VACANZA){
           _sms(NULL, outgoing_command[5] );
        }
     }

   
}     
/*
 * sendStatus è una azione che consiste nel risponde al clinet che ha inviato 'status' con un messaggio 
 * contenente la lista di tutti i sensori, l'ID associato, il nome mnemonico, il loro stato (on/off) e se sono esclusi dallo scenario corrente.
 */
void sendSensorsStatus(){
    
    unsigned char buff[30];
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
    
     
          if( sensor[i].state == false) {
           stringSTATUS = stato_off;
          }
          else if( sensor[i].state == true ) {
           stringSTATUS = stato_on;
          }
    
          
          memcpy(buff,&stringID, 2);
          memcpy(buff+2,&stringTAB, 1);
          memcpy(buff+3,sensor[i].name,12);
          memcpy(buff+15,stringSTATUS,4);
              if( sensor[i].sensor_exclusion == true ){
              memcpy(buff+19, &stringEXC, 1);
              }
              else{
              memcpy(buff+19, &stringTAB, 1);
               }
          memcpy(buff+20, &stringNULL, 1);
    
          _telnet(NULL, (const char *)buff);
       }

         gsmSignalStrength();
       
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
boolean _telnet(char * receive_buffer,const char * transmit_buffer){
  
      EthernetClient client = server.available();
      boolean data_available = false;

      /* #ifdef AUTH_PIN         
                                 
                                  if ( authorized == false && enter_pin == true) {
                                // delay(1000);                                    
                                 Serial.print("\nEnter PIN");
                                // server.write("Enter Pin: ");
                                 // client.print("Enter Pin: ");
                                  client.println();
                                  enter_pin = true;
                                 
                                  }
      #endif*/
          if (client) {
                  boolean newClient = true;
                  for (byte i=0;i<4;i++) {
                    if (clients[i]==client) {
                      newClient = false;
                      break;
                    }
                  }
                if (newClient) {
                  Serial.print("\nNew Client");
                  for (byte i=0;i<4;i++) {
                    if (!clients[i] && clients[i]!=client) {
                  //  if (!(clients[i].connected()) && clients[i]!=client) {
                       clients[i] = client;
                      client.flush();
                          clients[i].print("0");
                           clients[i].println();    
                      break;
                         // client.stop() invalidates the internal socket-descriptor, so next use of == will allways return false;
                         // clients[i].stop();
                        }
                        
                    //  clients[i] = client;
                     // client.stop();
                          //clients[i].print("welcome");  
                          //clients[i].println();
                     // break;
                    }
                  }
               int index = 0;
               data_available = false;
               if(receive_buffer != NULL ) {
                while( client.connected() && data_available == false ){
                          if (client.available() > 0) {
                                            char c = client.read();
                                            
                                             if (c == '\n' ){
                                              receive_buffer[index] = '\0';
                                              data_available = true;
                                             }
                                             
                                             else {
                                                        receive_buffer[index] = c;
                                                        index++;
                                                          if (index >= BUFSIZ){
                                                                 index = BUFSIZ -1;
                                                          }
                                             }
                              }
                        }
                }
                if(data_available == true ) {
                  Serial.print("\nrx: "); Serial.print(receive_buffer);
                }
                       #ifdef  AUTH_PIN
                                       if( strncmp( receive_buffer, pin, strlen(pin) ) == 0 && authorized == false ) {
                                        authorized = true;
                                        client.write("PIN OK\n\r");
                                        data_available = false; //the PIN is not a command to be read..
                                       }
                                       if( authorized == false ){
                                         client.write("PIN ERROR\n\r");
                                        for (byte i=0;i<4;i++) {
                                                    if (clients[i]==client) {
                                                        // client.stop() invalidates the internal socket-descriptor, so next use of == will allways return false;
                                                        clients[i].stop();
                                                      }
                                        }
                                        data_available = false; //the PIN is not a command to be read..
                                       }
                        #endif
          } // if( client) 
          if( transmit_buffer != NULL ){
                        // Serial.print("\n Transmit buffer NOT NULL");
                        Serial.print("\ntx: ");
                        Serial.print(transmit_buffer);
                        server.write(transmit_buffer);
                        server.write('\n');
                        server.write('\r');
          }
          for (byte i=0;i<4;i++) {
                      if (!(clients[i].connected())) {
                          // client.stop() invalidates the internal socket-descriptor, so next use of == will allways return false;
                          clients[i].stop();
                        }
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
            Serial.print("\n ricevuto sms:");Serial.print(receive_buffer); 
            Serial.print("\n      SMS position:"); Serial.print(SMS_position, HEX); 
           if ( sms_state == GETSMS_AUTH_SMS ) {
                    data_available = true;
                    Serial.print("\n      AUTH:");Serial.print(number);
                    
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
             Serial.print("\n     inviato sms:"); Serial.print((char*)transmit_buffer);
           sms.SendSMS(number, (char*)transmit_buffer);
         
  }
return data_available;
}

boolean gsmStatus(){
  boolean res = false;
  Serial1.write("AT\r"); //Send command for signal report
  delay (200);   
  // expect resp: +CSQ: 10,0 //10 char exactly
  int t = 0;
  while( Serial1.available() > 0 )  { //wait for responce
   t++;
   if( Serial1.read() == 'O' ||  Serial1.read() == 'K'){
  
    res = true;
    break;
   }
   else if( t > 10 ) {
    break;
   }
    delay(200);
  }
  return res;
}
int gsmSignalStrength()
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
  return (one*10 + two);

}









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
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
asm volatile ("  jmp 0");  
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
  case DELAY:                          Serial.print(F("\nACTION_DELAY"));      
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

  
}
void beep(int f){
  /*if ( f == 0 || f==20 || f==40 || f==60 
        || f==70 || f==80 || f==90 || f==100 || f==110 
        || f==120 || f == 125 || f == 130 || f == 140 || f == 145
        || f == 148 || f==150 ) {
          analogWrite(46, 255); delay(200); analogWrite(46,0);
        }*/

             if( f == action_routine_counter_max_loop ) {
              Serial.print("BUZZ0");
          analogWrite(buzzer_pin,0);
          return;
        }
               if( f == (delay_before_start_call-1) ) {
            analogWrite(buzzer_pin,0);
             Serial.print("BUZZ0");
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
           Serial.print("BUZZ0");
        }
}
/*
void beep(unsigned int cnt){

 if (( cnt % 10 )== 0) {
  Serial.print("\nBEEP UP");
          analogWrite(buzzer_pin,255); //l'ultimo deve spegnere!
        }
 
if( cnt == action_routine_counter_max_loop ) {
  analogWrite(buzzer_pin,0);
}
  
}


*/

void _DEBUG(String debugString){
#ifndef DEBUG 
  return;
#else
   Serial.print(debugString);
   return;
#endif
}
void _DEBUG2(String debugString){
#ifndef DEBUG2
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

