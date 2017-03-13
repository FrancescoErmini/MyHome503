
#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <Timer.h>

#include <Wire.h>
#include <PN532_I2C.h>
#include "PN532.h"

#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

#define DEBUG_CMD
#define DEBUG
#define DEBUG_TELNET 1
#define DEBUG_NFC 1 
#define DEBUG_ERROR 1

//#define AUTH_PIN 1 

byte mac[] = {
    0xAA, 0xAC, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 18);
  
IPAddress server(192, 168, 1, 101);

EthernetClient client;

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);

Timer t;


const unsigned long int count_max = 900000;
int error_counter;
int ack_counter;
boolean data_available = false;
boolean nfc_lock = false;
const int BUFSIZE = 20;
const char ID[] = "1";

const uint8_t ALARM_LED_GREEN = 8;
const uint8_t ALARM_LED_RED = 9;
const int RGB_RED = 5;
const int RGB_GREEN = 6;
const int RGB_BLU = 7;

uint8_t keyuniversal[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t keya[6]  = { 0x15, 0x06, 0x28, 0x04, 0x30, 0x04 };
uint8_t keyb[6] = { 0xE0, 0xF1, 0xC2, 0x15, 0x28, 0x30 };
/*
 * TODO: idea -> mettere le chiavi nel SD del w5100.
 * Note for me: those key are old one, you must change them!
 */
uint8_t secret[16] = {0x6f, 0x72, 0x61, 0x7a, 0x69, 0x6f, 0x6d, 0x61, 0x72, 0x67, 0x68, 0x65, 0x72, 0x69, 0x74, 0x61 }; // oraziomargherita
uint8_t secret1[16] = {0x73, 0x74, 0x65, 0x66, 0x61, 0x6e, 0x6f, 0x66, 0x72, 0x61, 0x6e, 0x63, 0x65, 0x73, 0x63, 0x6f};//stefanofrancesco
uint8_t secret2[16] = {0x63, 0x6f, 0x6d, 0x75, 0x6e, 0x69, 0x73, 0x74, 0x69, 0x64, 0x69, 0x6d, 0x65, 0x72, 0x64, 0x61 }; //comunistidimerda

char buffer_incoming_command[BUFSIZE];                         
const char * outgoing_command[] = {"status",             
                                  "off",
                                  "on",
                                  "on dormo",
                                  "on esco",
                                  "on vacanza",
                                  "errore"
                                  };
const char * incoming_command[]  = {"not used",                
                                  "stato off",
                                  "stato on",
                                  "stato on dormo",
                                  "stato on esco",
                                  "stato on vacanza",
                                  "errore"
                                  };



const int EEPROM_status_addr = 0;          //where current alarm status is stored
const int EEPROM_scenario_addr = 1;   
  
enum status_t {
    STATUS_ON,
    STATUS_OFF,
    STATUS_UNDEF
};
/*
 * SCENARIO
 * Identifica scenari abbinati ad azioni o circostanze.
 * SCENARIO_SCENARIO_DORMO - le finestre aperte al momento dell'inserimeto sono escluse. Volumetrici e barriere IR sono esclusi. 
 * SCENARIO_SCENARIO_ESCO - esclude la barriera perimetrale esterna. ( si intende SCENARIO_SCENARIO_ESCO per qualche ora, non settimane).
 * SCENARIO_SCENARIO_VACANZA - non escludo nulla
 * PREDEFINTO - varia in relazione al dispositivo.
 * E.g
 * Se attivo allarme da sms, molto probabilmente sono lontano da cas, per cui lo scenario SCENARIO_SCENARIO_PREDEFINITO è 'SCENARIO_SCENARIO_VACANZA'.
 * Se attivo allarme da NFC, molto probabilmente sto uscendo di casa, per cui lo scenario PREDEFINTO è 'SCENARIO_SCENARIO_ESCO'.
 * Se attivo allarme da Server (in Rete Locale), molto probabilmento sono in casa, per cui lo scenario PREDEFINTO è 'SCENARIO_SCENARIO_DORMO'.
 */
enum scenario_t {
    SCENARIO_DORMO,
    SCENARIO_ESCO,
    SCENARIO_VACANZA,
    SCENARIO_PREDEFINITO,
    TEST,
    SCENARIO_UNDEF
};

status_t old_status;
scenario_t old_scenario;

                   
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
  CMD_NOT_AUTH,
  
};

enum blink_t {
  BLINK_ACK,
  BLINK_STATUS,
  BLINK_NFC_LOCK,
  BLINK_NFC_READY,
  BLINK_ERROR
};
enum error_t {
   ERROR_W5100,
   ERROR_PN532,
   ERROR_TELNET,
   ERROR_ACK_LOOSES,
    
};


enum nfc_t {
  NFC_NONE,
  NFC_LOCK,
  NFC_AUTH,
  NFC_NOT_AUTH,
  NFC_ERROR
};



/*
 * L'intento è quello di leggere un tag NFC e un eventulae tastiera numerica per generare il comando da inviare al server.
 * A ogni ciclo, viene letto l'NFC. Se non ci sono tag, _nfc() ritorna NFC_NONE e la funzione termina subito.
 * Se il tag è presente, ed è stato autenticato con la chiave allara si passa a decodificare lo stato corrente, eventualmetne leggere da tastiera egenerare il cmd.
 * Se lo status è on, il comadno è CMD_OFF, intendendo che l'azione del nfc tag è quella di spengnere l'allarme.
 * Se lo stato è off, e non si usa il tastierino, il comando è CMD_ON_SCENARIO_ESCO.
 * Una volta trasmesso il comando tramite telnet al server, si attende che il server notifichi lo stato dell'allarme.
 * Questo garantisce che si abbia certezza del fatto che il server ha effettivamente settato quel comando, 
 * del fatto che funzioni tutto correttamente e del fatto che altri dispositivi telnet sono contemporaneamente notificati del nuovo stato direttamente dal server.
 * 
 *Una nota a parte mertia l'nfc unlock. 
 *Praticamente l'nfc legge un tag finché questo è presente, con conseguente letture multiple del tag anche solo avvicinadolo pochi secondi.
 *Questo è certamente dannoso perché non si sa qunate volte è stato letto e quindi non sappiamo il comando finale.
 *Ora, per evitare questo ho abbinato un sistema visivo che  usa i led a due eventi di unlock. 
 *In pratica, dopo la prima lettura autorizzata del tag i.e NFC_AUTH scatta il nfc_lock = true.
 *Per cui al ciclo successivo, nella funzione inputRoutine() finché nfc_lock è true fai 'return' e non passi da _nfc(), evitando così letture multiple del tag.
 *nfc_lock si sblocca dopo che è stata ricevuta risposta dal server e dopo che l tag viene allontaanto dal lettore.
 *I led, iniziano colore arancione se il tag è AUTH, poi diventa verde se dal server è arrivata la risposta, e si spengne se tutto va bene. 
 *Se diventa rosso invece è andato in timeout.
 */
 

nfc_t inputRoutine(void){
            
                 cmd_t cmd = CMD_NONE;
                 nfc_t nfc_status = NFC_NONE;
                 
                 if(nfc_lock == true){ //avoid multiple read of nfc tag after one read
                      return NFC_LOCK;
                 }
                 
                 nfc_status = _nfc(); //read nfc status
                 
                 if( nfc_status == NFC_AUTH ){
                  
                  _blink(BLINK_NFC_LOCK, HIGH); 
                  nfc_lock = true;; //lock nfc reading until the command form server arrives or timeout occurs.
                  
                  status_t status = getStatus();
                  scenario_t  scenario = _keypad();
                
                  cmd = setCommand(  status, scenario);   //interpreta gli input in un comando da mandare al servver
                  int cmd_num = getCommand(cmd);
                  telnet(NULL, outgoing_command[cmd_num]);
               
                
                
                  #ifdef DEBUG_TELNET
                  Serial.print(F("\nInput will generate the command: "));
                  print_command(cmd);
                  Serial.print(F("\nOutgoing command message is: "));
                  Serial.print(outgoing_command[cmd_num]);
                  Serial.print(F("\nWait the server that responde at this command with the new status..."));
                  #endif
                              
                   unsigned long int count = 0;
                    while(1){
                     count++;
                    // no delay here or it will fails!
                      if (count > count_max){
                          count = 0;
                                          #ifdef DEBUG_TELNET
                                          Serial.print(F("\nERROR: Timeout waiting response from server"));
                                          #endif
                                          _blink(BLINK_ERROR, HIGH);
                                          nfc_status = NFC_ERROR;
                                          ack_counter = 3;
                                          nfc_lock = false;
                                          
                      break;
                      }
                      else if( commandRoutine() != STATUS_UNDEF){ // ok then, received the status from the server
                                           
                                          #ifdef DEBUG_TELNET
                                          Serial.print(F("\nOK: new status received succesfully from server"));
                                          #endif
                                          _blink(BLINK_NFC_LOCK, LOW);  //orange off
                                          _blink(BLINK_NFC_READY, HIGH); //green on
                                           nfc_lock = false;      // first unlock, the status was received.                                      
                      }
                      if (nfc_lock == false){
                                         #ifdef DEBUG_NFC
                                         Serial.print("\nPlease, remove NFC tag now");
                                         #endif       
                            if( !(_nfc_tag_detected()) ){         // second unlock, the tag was removed from the tag reader
                                         #ifdef DEBUG_NFC
                                         Serial.print("\nNFC tag removed");
                                         Serial.print(F("\nExit while(1)"));
                                         #endif 
                                         _blink(BLINK_NFC_READY, LOW); //green off, ok
                                           
                             break;  // if both unlock, then exit while(1)
                            }
                      }
                    
                   }
                 }
return nfc_status;                 
}
/*
 * Se un messaggio telnet è presente, viene carpito da quel messaggio lo stato e lo scenario.
 * Se questo status e scenario hanno un senso, perché il messaggio arrivato aveva senso.
 * Si setta i nuvo scenario e status e si cambia il led rosso/verde.
 * Di risposta il nodo mando il proprio ID.
 */
status_t commandRoutine(){
     status_t current_status = STATUS_UNDEF;
     scenario_t current_scenario = SCENARIO_UNDEF;
     
     if( true == telnet(buffer_incoming_command, NULL) ) {
      
       current_status = updateStatus(buffer_incoming_command);
       current_scenario = updateScenario(buffer_incoming_command);
       
            if( current_status != STATUS_UNDEF || current_scenario != SCENARIO_UNDEF ){
                 ack_counter = 0;
                                      #ifdef DEBUG_TELNET
                                      Serial.print(F("\n Received status from server"));
                                      #endif
                
                 if( current_status != old_status || current_scenario != old_scenario){
                                     #ifdef DEBUG_TELNET
                                    Serial.print(F("\n Status changed"));
                                    #endif
                      setStatus(current_status);
                      setScenario(current_scenario);
                      if( current_status == STATUS_OFF ) {
                        _blink(BLINK_STATUS, LOW);
                      }
                      else{
                        _blink(BLINK_STATUS,HIGH);
                      }

                      
                 }
                  old_status = current_status;
                  old_scenario = current_scenario;
                                 #ifdef DEBUG_TELNET
                                  Serial.print(F("\n Received status from server, now send client ID"));
                                 #endif
                  // telnet(NULL, ID);
                   _blink(BLINK_ACK, HIGH ); //lo accendo perché vuol dire che ho ricevuto lo stato dal server. Poi lo spengo quando scatta il controllo ack.
            }
            
    }
    return current_status; 
}


/*
 * 
 * Viene chiamata come interrupt ogni minuto.  
 * Viene settata nel setup() e aggiornata nel loop con t.update()
 * Serve a trovare errori. 
 * il ack_counter è resettato a zero qunado viene ricevuto un messaggio telnet.
 * L'error_counter è incrementato ogni volta che viene chiamata la funzione error()
 * - check if there are ack looses
 * - check if pn532 don't responde 
 */

void statusRoutine(){
  
                     #ifdef DEBUG_TELNET
                     Serial.print(F("\n ACK evaluation ( 1min)"));
                     #endif
                     _blink(BLINK_ACK, LOW );                
                     ack_counter++;
                        
                     if( ack_counter > 2){
                        #ifdef DEBUG_TELNET
                             Serial.print(F("\nACK LOOSE"));
                         #endif
                        error(ERROR_TELNET);
                    }
                    if(!(nfc.getFirmwareVersion())){
                          #ifdef DEBUG_NFC
                             Serial.print(F("\nNFC ERROR"));
                         #endif
                        error(ERROR_PN532);
                    }

                    if( error_counter > 20 ){
                         #ifdef DEBUG_TELNET
                             Serial.print(F("\n"));
                         #endif
                         error_counter = 0;
                         software_Reset();
                    }
}


boolean telnet(char * receive_buffer, const char * transmit_buffer) {
         
         if( transmit_buffer != NULL){
               if (client.connected()) {
                client.print(transmit_buffer);
                client.println();
               } 
         
         }

  
         data_available = false;
         int index=0;
         if(receive_buffer != NULL ){
          //NEDD DEBUG HERE?
         
            while (client.available() && data_available == false ) {
           
               char c = client.read();
              
                if (c == '\n' ){ 
                                receive_buffer[index] = '\0';
                                
                                data_available = true;
                               }
               else { 
                          receive_buffer[index] = c;
                          index++;
                            if (index >= BUFSIZE){
                                   index = BUFSIZE -1;
                            }
              
               }
            }
         }
         if( data_available== true ) {
     
        #ifdef DEBUG_TELNET
        Serial.print(F("\nReceived message:"));
        Serial.print(receive_buffer);
        #endif
        }
     
          // if the server's disconnected, stop the client:
      /*   if (!client.connected()) {
            Serial.println();
            Serial.print(F("no connection\n");
            client.stop();
            delay(3000);
            boolean fault = false;
            for(int i=0; i<=240; i++){
              if( i == 240 ){
                fault = true;
              }
              if(client.connect(server,23) == true ) {
                break;
              }
              Serial.print(F("no connection\n");
              delay(10000);
            }
            
          }*/
 return data_available;         
}


boolean telnet_reconnection(){
 
           
             client.stop();
              if(client.connect(server,23) == true ) {
                return true;
              }
              return false;
}

scenario_t _keypad(){
 // while(millis() 10sec
  return SCENARIO_ESCO;
}


/*================================== NFC =============================*/

nfc_t  _nfc(){
  uint8_t success;                          // Flag to check if there was an error with the PN532
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  uint8_t currentblock;                     // Counter to keep track of which block we're on
  bool authenticated = false;               // Flag to indicate if the sector is authenticated
  uint8_t data[16];                         // Array to store block data during reads


  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
 
  if (success) {
   
    #ifdef DEBUG_NFC
    Serial.print(F("\nFound an ISO14443A card"));
    #endif

    
 
    for (uint8_t i = 0; i < uidLength; i++) {
    
    }
  
   
    if (uidLength == 4)
    {
      
      for (currentblock = 0; currentblock < 8; currentblock++)
      {
        
        if (nfc.mifareclassic_IsFirstBlock(currentblock)) authenticated = false;

        
        if (!authenticated)
        {
          // Starting of a new sector ... try to to authenticate
         // Serial.print(F("------------------------Sector "));Serial.print(currentblock/4, DEC);Serial.print(F("-------------------------"));
          if (currentblock == 0)
          {
             
              success = nfc.mifareclassic_AuthenticateBlock (uid, uidLength, currentblock, 1, keyuniversal);
          }
          else
          {
              
              success = nfc.mifareclassic_AuthenticateBlock (uid, uidLength, currentblock, 0, keya);
          }
          if (success)
          {
            authenticated = true;
          }
          else
          {
             #ifdef DEBUG_NFC
            Serial.print(F("Authentication error"));
            #endif
          }
        }
        
        if (!authenticated)
        {
       //   Serial.print(F("Block "));Serial.print(currentblock, DEC);
           #ifdef DEBUG_NFC
           Serial.print(F(" unable to authenticate"));
           #endif
        }
        else
        {
          // Authenticated ... we should be able to read the block now
          // Dump the data into the 'data' array
          success = nfc.mifareclassic_ReadDataBlock(currentblock, data);
          if (success)
          {
             // Read successful
           
            // Dump the raw data
            //   nfc.PrintHexChar(data, 16);
             // _blink(NFC,LOW);
            
               if ( currentblock == 5){
                      if(memcmp( data, secret,16 ) == 0 ){
                                #ifdef DEBUG_NFC
                                Serial.print(F("\n\n auth success with secret: "));
                                Serial.print(currentblock, DEC);
                                #endif
                        return NFC_AUTH;
                      }
                      else if(memcmp( data, secret1,16 ) == 0 ){
                                #ifdef DEBUG_NFC
                               Serial.print(F("\n\n auth success with secret 2"));
                               #endif
                        return NFC_AUTH;
                      }
                      else if(memcmp( data, secret2,16 ) == 0 ){
                                #ifdef DEBUG_NFC
                                Serial.print(F("\n\n auth success with secret 3"));
                                #endif
                        return NFC_AUTH;
                      }
               }
            
          }
          else
          { 
                  // Oops ... something happened
                  #ifdef DEBUG_NFC
                  Serial.print(F("Block "));Serial.print(currentblock, DEC);
                  Serial.print(F(" unable to read this block"));
                  #endif
            return NFC_NOT_AUTH;
          }
        }
      }
    }
    else
    {
      #ifdef DEBUG_NFC
      Serial.print(F("Ooops ... this doesn't seem to be a Mifare Classic card!"));
      #endif
    }
  }
  return NFC_NONE;
}

uint8_t _nfc_tag_detected(){
  uint8_t success;                          // Flag to check if there was an error with the PN532
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;  
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  return success;
}








void setup() {

  pinMode(ALARM_LED_GREEN, OUTPUT);
  pinMode(ALARM_LED_RED, OUTPUT);
  pinMode(RGB_RED, OUTPUT);
  pinMode(RGB_GREEN, OUTPUT);
  pinMode(RGB_BLU, OUTPUT);


  int r = t.every(60000,  statusRoutine , (void*)0);

  
  // start the Ethernet connection:
  Ethernet.begin(mac, ip);
  // Open serial communications and wait for port to open:
  #ifndef NO_SERIAL
  Serial.begin(115200);
  #endif
  delay(10000);
  Serial.print(F("connecting..."));

  if (client.connect(server, 23)) {
    Serial.print(F("connected"));
  } else {
    // if you didn't get a connection to the server:
    Serial.print(F("connection failed"));
    ack_counter = 3;
  }
  
  ack_counter = 0;
  old_status = STATUS_UNDEF;
  old_scenario = SCENARIO_UNDEF;
  //lastMillis = 0;
  delay(1000);


  Serial.print(F("\nStart NFC"));
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print(F("Didn't find PN53x board"));
      error(ERROR_PN532);
   
   }
 
 
  #ifdef DEBUG_NFC
  Serial.print(F("Found chip PN5")); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print(F("Firmware ver. ")); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  #endif
  // configure board to read RFID tags
  nfc.SAMConfig();
  
     
  error_counter = 0;

}




void loop(){
   
  nfc_t n = inputRoutine(); 
  status_t s = commandRoutine();
 
 #ifdef DEBUG_CMD
 if( s != STATUS_UNDEF ) print_status(s);
 if( n != NFC_NONE ) print_nfc(n);
 #endif
  
  t.update();
                        
}




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
status_t  updateStatus(char * stato){

   if( strncmp(stato, incoming_command[1], strlen(incoming_command[1])) == 0 ){ //STATO OFF
    return STATUS_OFF;
  }
  else if( strncmp(stato, incoming_command[2], strlen(incoming_command[2])) == 0 ){ //STATO ON
    return  STATUS_ON;
  }
  else if( strncmp(stato, incoming_command[3], strlen(incoming_command[3])) == 0 ){ //STATO ON
    return  STATUS_ON;
  }

  else if( strncmp(stato, incoming_command[3], strlen(incoming_command[3])) == 0 ){ //STATO ON
   return  STATUS_ON;
  }
  else if( strncmp(stato, incoming_command[4], strlen(incoming_command[4])) == 0 ){ //STATO ON
    return  STATUS_ON;
  }
  else {
    return STATUS_UNDEF;
  }
         
}
scenario_t  updateScenario(char * stato){

 
  if( strncmp(stato, incoming_command[1], strlen(incoming_command[1])) == 0 ){ //STATO OFF
    return SCENARIO_UNDEF;
    
  }
  else if( strncmp(stato, incoming_command[2], strlen(incoming_command[2])) == 0 ){ //STATO ON
     return SCENARIO_PREDEFINITO;
  }
  else if( strncmp(stato, incoming_command[3], strlen(incoming_command[3])) == 0 ){ //STATO ON
    return SCENARIO_DORMO;
  }

  else if( strncmp(stato, incoming_command[3], strlen(incoming_command[3])) == 0 ){ //STATO ON
   return SCENARIO_ESCO;
  }
  else if( strncmp(stato, incoming_command[4], strlen(incoming_command[4])) == 0 ){ //STATO ON
    return SCENARIO_VACANZA;
  }
  else {
    return SCENARIO_UNDEF;
  }
 
     
}

cmd_t setCommand( status_t current_status, scenario_t current_scenario){
 
    if( current_status == STATUS_ON ) { //se spento, lo accendi e viceversa.
      return CMD_OFF;
    }
    else if( current_status == STATUS_OFF && current_scenario == SCENARIO_PREDEFINITO) {
      return CMD_ON;
    }
    else if( current_status == STATUS_OFF && current_scenario == SCENARIO_DORMO) {
      return CMD_ON_SCENARIO_DORMO;
    }
    else if( current_status == STATUS_OFF && current_scenario == SCENARIO_ESCO) {
      return CMD_ON_SCENARIO_ESCO;
    }
    else if( current_status == STATUS_OFF && current_scenario == SCENARIO_VACANZA) {
      return CMD_ON_SCENARIO_VACANZA;
    }
    else{
      return CMD_WRONG;
    }

}
int getCommand(cmd_t cmd){
  switch( cmd ){
    
           case CMD_OFF: 
             return 1;
          break;
          case CMD_ON:  
            return 2;
          break;
          case CMD_ON_SCENARIO_DORMO: 
             return 3;
          break;// send 
          case CMD_ON_SCENARIO_ESCO:
           return 4;
          break;
          case CMD_ON_SCENARIO_VACANZA:
             return 5;
          break;
          case CMD_NOT_AUTH: 
            return 6;
          break;
      
  }
  
}




void error( error_t error){
            
           _blink(BLINK_ERROR, HIGH );
  
          #ifdef DEBUG_ERROR
             Serial.print(F(""));
             Serial.print(F("ERROR"));
             Serial.print(F(""));
           #endif
         
    
            //  t.update();                                 
                    switch(error){
                     
                      case ERROR_TELNET:
                                                       #ifdef DEBUG_ERROR
                                                        Serial.print(F("\n ERROR_TELNET"));
                                                       #endif
                                                      if( telnet_reconnection() == true){
                                                        ack_counter = 0;
                                                        error_counter = 0;
                                                         _blink(BLINK_ERROR, LOW);
                                                         return;
                                                      }
                      break;
                  
                      case ERROR_PN532:                 
                                                       #ifdef DEBUG_ERROR
                                                       Serial.print(F("\nNFC error"));
                                                       #endif
                                                       if( (nfc.getFirmwareVersion()) ){
                                                        error_counter = 0;
                                                         _blink(BLINK_ERROR, LOW);
                                                         return;
                                                      }
                                                       //software_Reset();
                      break;                          
                                                       
                      case ERROR_W5100:                
                                                        #ifdef DEBUG_ERROR
                                                        Serial.print(F("\nNFC error"));
                                                        #endif
                                                        //software_Reset();
                      break;
                    }
       
         return;
}

void _blink(blink_t blink_status, uint8_t val){
  switch(blink_status){
      case BLINK_ACK: 
            digitalWrite(RGB_RED, LOW);
           digitalWrite(RGB_GREEN, LOW);
           digitalWrite(RGB_BLU, !val);
           
      break;

      case BLINK_STATUS:
          digitalWrite(ALARM_LED_RED, !val);
          digitalWrite(ALARM_LED_GREEN, val);
     break;
     case BLINK_NFC_LOCK://orange
          digitalWrite(RGB_RED, !val);
          digitalWrite(RGB_BLU, LOW);
          digitalWrite(RGB_GREEN, !val); 
     break;
     case BLINK_NFC_READY:
          digitalWrite(RGB_RED, LOW);
          digitalWrite(RGB_BLU, LOW);
          digitalWrite(RGB_GREEN, !val); 
     break;
     case BLINK_ERROR:
           digitalWrite(RGB_BLU,LOW);
           digitalWrite(RGB_GREEN, LOW);
          digitalWrite(RGB_RED, !val); 
     break;
    
  }
}
void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
asm volatile ("  jmp 0");  
}  
void print_command( cmd_t cmd){
//  Serial.print("\nOUTGOING CMD:");
#ifdef DEBUG_CMD
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
#endif
}
void print_status(status_t status){
  #ifdef DEBUG_CMD
  switch(status){
   case STATUS_ON:                     Serial.print(F("\nSTATUS_ON"));
   break;
   case STATUS_OFF:                    Serial.print(F("\nSTATUS_OFF"));
   break;
   case STATUS_UNDEF:                  Serial.print(F("\nSTATUS_UNDEF"));       
   break;
  }
#endif
}
void print_nfc(nfc_t nfc){
  #ifdef DEBUG_CMD
  switch(nfc){
  case NFC_NONE:                        Serial.print(F("\nNFC_NONE"));
  break;
  case NFC_LOCK:                        Serial.print(F("\nNFC_LOCK"));
  break;
  case NFC_AUTH:                        Serial.print(F("\nNFC_AUTH"));
  break;
  case NFC_NOT_AUTH:                    Serial.print(F("\nNFC_NOT_AUTH"));
  break;    
  case NFC_ERROR:                       Serial.print(F("\nNFC_ERROR"));
  break;
  }
  #endif
}
/*void _DEBUG2(String debugString){
#ifndef DEBUG2
  return;
#else
   Serial.print(debugString);
   return;
#endif
}*/
