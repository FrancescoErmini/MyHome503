#include "RS485_protocol.h"
#include <SoftwareSerial.h>

const byte ENABLE_PIN = 4;
const byte LED_PIN = 13;
unsigned int num_slaves=2;
unsigned int num_sensors=6;
//byte msg[10][3];
byte slaves[]={1,2};
int _pir_status, _photores_status;
enum sensor_t{
  REED,
  PIR,
  PHOTORES,
  LED,
  TEMP,
  ACK 
};
sensor_t sensor;
SoftwareSerial rs485 (2, 3);  // receive pin, transmit pin
boolean blinkled; 
// callback routines

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


void reed(byte node, byte sensor, byte value){
  Serial.print("\n From node ");
  Serial.print(node);
  Serial.print(" From sensor reed ");
  Serial.print(sensor);
  Serial.print(" read value ");
  Serial.print(value);
}
void pir(byte node, byte sensor, byte value){
  Serial.print("\n From node ");
  Serial.print(node);
  Serial.print(" From sensor pir ");
  Serial.print(sensor);
  Serial.print(" read value ");
  Serial.print(value);
}
void photores(byte node, byte sensor, byte value){
  Serial.print("\n From node ");
  Serial.print(node);
  Serial.print(" From sensor photores");
  Serial.print(sensor);
  Serial.print(" read value ");
  Serial.print(value);
}
void led(byte node, byte sensor, byte value){
  Serial.print("\n to node ");
  Serial.print(node);
  Serial.print(" to sensor led ");
  Serial.print(sensor);
  Serial.print(" write value ");
  Serial.print(value);
  byte led_on[] = {
    node,
    sensor,
    value
  };
   digitalWrite (ENABLE_PIN, HIGH);  // enable sending
   sendMsg (fWrite, led_on, sizeof(led_on));
   digitalWrite (ENABLE_PIN, LOW); 
  
}
void temp(byte node, byte sensor, byte value){
  Serial.print("\n From node ");
  Serial.print(node);
  Serial.print(" From sensor temp ");
  Serial.print(sensor);
  Serial.print(" read value ");
  Serial.print(value);
}
void ack(byte node, byte sensor, byte value){
  Serial.print("\n From node ");
  Serial.print(node);
  Serial.print(" From sensor ack ");
  Serial.print(sensor);
  Serial.print(" read value ");
  Serial.print(value);
     digitalWrite(LED_PIN, HIGH);
                                    delay(1000);
                                     digitalWrite(LED_PIN, LOW);
                                  
                                  
          
}
void setup()
{ Serial.begin(115200);
  rs485.begin (28800);
  pinMode (ENABLE_PIN, OUTPUT);  // driver output enable

  
  pinMode (LED_PIN, OUTPUT);  // built-in LED
  pinMode(12,INPUT);

 // num_slaves = sizeof(slaves)/sizeof(byte);
  //num_sensors = sizeof(sensor);
 /* byte msg[num_slaves*num_sensors][3];
    for(int s=0; s < num_slaves; s++){
        for(int c=0; c<num_sensors; c++){
             msg[s*num_sensors+c][0]=(byte)(s+1);
              msg[s*num_sensors+c][1]=(byte)c;
                        
          }
    }
    delay(4000);
  for(int i=0; i < num_sensors*num_slaves;i++){
                  int s = (i/num_sensors)+1;
                  Serial.print((int)msg[i][0]);
                     Serial.print("/");
                  Serial.print((int)msg[i][1]);
                  Serial.print(" ");
  }*/
/*
  byte p[]={
    1,
    0,
    0
    
  };
    digitalWrite (ENABLE_PIN, HIGH);  // enable sending
                  sendMsg (fWrite, p, sizeof(p));
                  digitalWrite (ENABLE_PIN, LOW);  // disable sending
                  */
}  // end of setup


void loop()
{    
      
         for(int i=0; i < num_sensors*num_slaves;i++){
                  int s = (i/num_sensors)+1;
                  byte dest_addr  = (byte)((i/num_sensors)+1);
                  byte sensor_type = (byte)(i%num_sensors);
                    byte pkt[] = {
                      dest_addr,
                      sensor_type
                      };
          byte buf[10];
         
                  digitalWrite (ENABLE_PIN, HIGH);  // enable sending
                  sendMsg (fWrite, pkt, sizeof(pkt));
                  digitalWrite (ENABLE_PIN, LOW);  // disable sending
                  delay(1);
                   byte received = recvMsg (fAvailable, fRead, buf, sizeof (buf));
                     if (received) {
                        if (buf [0] == 0){ //master id = 0
                       
                          switch((sensor_t)buf[1]){
                                    case REED:
                                              reed(buf[3], buf[1], buf[2] );
                                              
                                              break;
                                    case PIR:
                                              pir(buf[3], buf[1], buf[2] );
                                              _pir_status = (int)buf[2];
                                              break;
                                    case PHOTORES:
                                              photores(buf[3], buf[1], buf[2] );
                                              _photores_status = (int)buf[2];
                                              break;
                                   case LED: 
                                             blinkled = true;
                                              break;
                                    case TEMP:
                                              temp(buf[3], buf[1], buf[2] );
                                              break;
                                    case ACK: 
                                               ack(buf[3], buf[1], buf[2] );
                                              
                                               break;
                                  }
                                  //if( _pir_status > 0 && _photores_status > 0){
                                  if(digitalRead(12) == LOW){
                                                    led(dest_addr, LED , 1 );
                                  }
                                 
                            
                        }else{
                          return;
                        }
           }
      }
}                                    
/*
  
byte msg[3];
  // read potentiometer
  if(digitalRead(12)==LOW){
   msg[0]=1;
  }
  else{
    msg[0]=2;
  }
  msg[2]=1;
 msg[1]=2;
  for(int i=1; i<3; i++){
   // msg[0]=i;
    
  

     digitalWrite (ENABLE_PIN, HIGH);  // enable sending
  sendMsg (fWrite, msg, sizeof msg);
  digitalWrite (ENABLE_PIN, LOW);  // disable sending
delay(100);



  // receive response  
  byte buf [10];
  byte received = recvMsg (fAvailable, fRead, buf, sizeof buf);
if(buf[1]==1){
    digitalWrite (LED_PIN, HIGH); 
}
else{
    digitalWrite (LED_PIN,LOW); 
}
 // turn on LED if error    

    
  }

  // assemble message
  

 
  // only send once per successful change
  //if (received)
  //  old_level = level;

}  // end of loop*/
