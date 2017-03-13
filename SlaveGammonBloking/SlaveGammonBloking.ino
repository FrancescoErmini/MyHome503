#include <SoftwareSerial.h>
#include "RS485_protocol.h"
/*
 * Il msg è composto da una tripla, ID + CMD + Value.
 * ID definisce l'identifier dello slave.
 * CMD definisce quale sensore deve inviare o ricevere dati
 * Value indica il valore o campo on/off da trasmettere o ricevere per il sensore
 */

const byte ID = 2; 

const byte ENABLE_PIN = 4;

enum sensor_t{
  REED,
  PIR,
  PHOTORES,
  LED,
  TEMP,
  ACK 
};

const byte sensors_pin[] = {12,10,11,13,9,5};
boolean  _status_old[6];
SoftwareSerial rs485 (2, 3);  // receive pin, transmit pin


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


void answer(byte cmd, byte value){
   delay(1);
   byte msg[4];
   msg[0]=0; //reply to Master id = 0
   msg[1]=cmd;
   msg[2]=value;
   msg[3]=ID;
   digitalWrite (ENABLE_PIN, HIGH);  // enable sending
   sendMsg (fWrite, msg, sizeof msg);
   digitalWrite (ENABLE_PIN, LOW);  // disable sending
   free(msg);
}
boolean readSensor(const byte pin){
  if(digitalRead(pin)==HIGH){
   return 1; //NOTA: il reed col pullup deve chiudere a massa!  
  }

  if(digitalRead(pin)==LOW){ 
    return 0; //scatta sul Reed, quando la finestra si apre il piedino rimane flottante e va giù!
  }
  
}
void _flash_led(int pin, unsigned int interval)  { 
                  digitalWrite(pin, HIGH);
                  delay(interval*1000);
                  digitalWrite(pin, LOW); 
}

 void _add_buf_event(byte * _event_holder, unsigned int i){
               ++(_event_holder[i]);
 }
 void _rst_buf_event(byte * _event_holder, unsigned int i){
            _event_holder[i]=0;   
 }
                    
    

void _alarm_event(boolean * _status, byte * _buf_event){
            
         //reed
        _status[REED]=digitalRead(sensors_pin[REED]);      //lettura del valore corrente         
        if(_status[REED] == LOW && _status_old[REED] == HIGH){ //Se avviene l'Evento e.g contatto finestra flottante, pir o photoresistor a vcc, col pullup portano da ingresso alto a basso. 
              _add_buf_event(_buf_event, REED);
             }
             
            _status_old[REED] == _status[REED];
         


          //pir
         _status[PIR]=digitalRead(sensors_pin[PIR]);      //lettura del valore corrente         
         if(_status[PIR] == LOW && _status_old[PIR] == HIGH){ //Se avviene l'Evento e.g contatto finestra flottante, pir o photoresistor a vcc, col pullup portano da ingresso alto a basso. 
              _add_buf_event(_buf_event, PIR);
             }
            _status_old[PIR] == _status[PIR];
         

         //photores
         _status[PHOTORES]=digitalRead(sensors_pin[PHOTORES]);      //lettura del valore corrente         
         if(_status[PHOTORES] == LOW && _status_old[PHOTORES] == HIGH){ //Se avviene l'Evento e.g contatto finestra flottante, pir o photoresistor a vcc, col pullup portano da ingresso alto a basso. 
              _add_buf_event(_buf_event, PHOTORES);
             }
             
            _status_old[PHOTORES] == _status[PHOTORES];
         
         
         //led

         //temp

         _buf_event[TEMP]=analogRead(sensors_pin[TEMP]);
         
        //ack
        _buf_event[ACK]=1;

         
        
}  




void setup()
{ Serial.begin(115200);
  rs485.begin (28800);
  pinMode (ENABLE_PIN, OUTPUT);  // driver output enable


  
  pinMode (sensors_pin[REED], INPUT_PULLUP);
  pinMode (sensors_pin[PIR], INPUT_PULLUP);
  pinMode (sensors_pin[PHOTORES], INPUT_PULLUP);
  pinMode (sensors_pin[LED], OUTPUT);
  //anaalog read temp.
  pinMode(sensors_pin[ACK], OUTPUT);
 
}
void loop()
{
  
  byte buf [10];
  byte _buf_event[6];
  boolean _status[6];
 

  _alarm_event(_status, _buf_event);


  for(int i=0; i<6; i++){
    //Serial.print((int)_buf_event[i]); Serial.print(" ");
  }
  byte received = recvMsg (fAvailable, fRead, buf, sizeof (buf));
  
       if (received) {
          if (buf [0] == ID) {
          // my device
                Serial.println("received message");
               sensor_t sensor = (sensor_t) buf[1];
                switch (sensor) { //which sensor is 
                  
                   case REED:
                        //quando la finestra è chiusa sul piedino fa contatto l'altra estremità a massa. Avendo il pullup, il piedino legge una  tensione alta.
                          answer(REED, _buf_event[REED]); //value = numero di volte che la finestra è stata aperta nel TRT
                           _rst_buf_event(_buf_event, REED);        
                         break;
                   case PIR:
                           answer(PIR,_buf_event[PIR]); //value = numero di volte che il PIR si è attivato
                           _rst_buf_event(_buf_event, PIR);  
                         break;
                   case PHOTORES:
                          answer(PHOTORES, _buf_event[PHOTORES]);
                           _rst_buf_event(_buf_event, PHOTORES);  
                        break;
                   case LED:
                        if(buf[2]==1){
                          _flash_led(sensors_pin[LED], 2);
                        }
                         break;
                   case TEMP:
                        answer(TEMP, _buf_event[TEMP]);
                   break;
                   case ACK:
                        answer(ACK, _buf_event[ACK]);
                         _flash_led(sensors_pin[ACK], 1);
                   default:
                   return; //unknown command
                   break;
                   
                
            }
            
         }
         else{
          return;
         }
         
      }
    
}

