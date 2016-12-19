#include <Ultrasonic.h>
#include <BRCClient.h>
#include <SPI.h>
#include <RFID.h>

//define pin
#define motor_right_back 3
#define motor_right_for 4
#define motor_left_back 5
#define motor_left_for 6
#define front_trig 26 
#define front_echo 27 
#define left_trig 22
#define left_echo 23
#define right_trig 24
#define right_echo 25

#define UART_RX 10
#define UART_TX 2
#define SPI_MISO 50
#define SPI_MOSI 51
#define SPI_SCLK 52
#define SPI_SS 53
#define MFRC522_RSTPD 11

//define dir
#define front_dir 0
#define left_dir 1
#define right_dir 2

//define AP(test)
#define AP_SSID    "Test_Server_AP"
#define AP_PASSWD  "testserverap"
#define TCP_IP     "192.168.43.1"

//define AP(programtheworld)
//#define AP_SSID    "programtheworld"
//#define AP_PASSWD  "screamlab"
//#define TCP_IP     "192.168.150.11"

//define information
#define TCP_PORT   5000
#define MY_COMM_ID 0x23
#define PARTNER_COMM_ID 0x24

Ultrasonic front(front_trig, front_echo);
Ultrasonic left(left_trig, left_echo);
Ultrasonic right(right_trig, right_echo);
BRCClient brcClient(UART_RX, UART_TX);
RFID rfid(SPI_SS, MFRC522_RSTPD);

int v = 130,last_cross, mapping_front=0, mapping_cross=0;
float front_dis, right_dis, left_dis;
bool back = false, front_sta, left_sta, right_sta,go=false;
bool mapping[6][6] = {false},inf_loop = false;

void for_back(int mot_1,int mot_2,int dur){
  analogWrite(mot_1, v);
  analogWrite(mot_2, v);
  delay(dur);
  analogWrite(mot_1,0);
  analogWrite(mot_2,0);
}

void turn(int mot,int dur){
  analogWrite(mot, v); 
  delay(dur);  
  analogWrite(mot,0);
}

void line(int front,int left,int right){
  if(front < 8){
    for_back(motor_right_back,motor_left_back,450);
    front_sta = true;
    left_sta = true;
    right_sta = true;
  }
  else if (left < 5) {
    turn(motor_right_back,190);
    front_sta = true;
    left_sta = true;
    right_sta = true;   
  }
  else if (right < 5) {
    turn(motor_left_back,190);
    front_sta = true;
    left_sta = true;
    right_sta = true;    
  }
}
void setup() {
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000L, MSBFIRST, SPI_MODE3));
  rfid.begin();

  Serial.begin(9600);
  pinMode(motor_right_back, OUTPUT);
  pinMode(motor_right_for, OUTPUT);
  pinMode(motor_left_back, OUTPUT);
  pinMode(motor_left_for, OUTPUT);
  analogWrite(motor_right_for, 0);
  analogWrite(motor_right_back, 0);
  analogWrite(motor_left_for, 0);
  analogWrite(motor_left_back, 0);
  while (!Serial)
    ;

  brcClient.begin(9600);
  brcClient.beginBRCClient(AP_SSID, AP_PASSWD, TCP_IP, TCP_PORT);
    
  if (brcClient.registerID(MY_COMM_ID))
    Serial.println("ID register OK");
  else
    Serial.println("ID register FAIL");

}

static uint8_t status;
static uint16_t card_type;
static uint8_t sn[MAXRLEN], snBytes;

void loop() {
  //測距
  long front_sec = front.timing(), left_sec = left.timing(), right_sec = right.timing();
  front_dis = front.convert(front_sec, Ultrasonic::CM);
  left_dis = left.convert(left_sec, Ultrasonic::CM);
  right_dis = right.convert(right_sec, Ultrasonic::CM);
  CommMsg msg;
  if (brcClient.receiveMessage(&msg)) {
    switch(msg.type){
      case MSG_ROUND_START:go=true;break;
      case MSG_ROUND_END: brcClient.endBRCClient(); go=false; break;
    }
  } 
  if ((status = rfid.findTag(&card_type)) == STATUS_OK) {
    Serial.print("OK! ");
    Serial.println(card_type);
    if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) {
      for (int i = 0; i < snBytes; ++i){
        Serial.print(sn[i], HEX);
        Serial.println();
        delay(1000);
      }      
      rfid.piccHalt();
    }
  } else
    Serial.println("No tag.");
   
  //按照路線判斷走
  if (go==true) {
    //直線校正
    line(front_dis,left_dis,right_dis); 
    //路線判斷
    if (front_dis <= 8) { front_sta = false;    
    }
    else {  front_sta = true;
    }   
    if (left_dis >= 30) { left_sta = true;
    }
    else {  left_sta = false;
    }
    if (right_dis >= 30) {  right_sta = true;
    }
    else {  right_sta = false;
    }  
    switch (front_sta) {
      case true:
        switch (left_sta) {
          case true:
            switch (right_sta) {
              case true:
                //TTT
                //worst situation
                 for_back(motor_right_for,motor_left_for,500);
                break;
              case false:  
                //TTF                
                 for_back(motor_right_for,motor_left_for,150);
                 turn(motor_right_for,560);
                 for_back(motor_right_for,motor_left_for,1000);                
                 last_cross = left_dir;                  
                break;
            }
            break;
          case false:
            switch (right_sta) {
              case true:
                //TFT                
                  for_back(motor_right_for,motor_left_for,150); 
                  turn(motor_left_for,560);                
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = right_dir ;                             
                break;
              case false:
                //TFF
                for_back(motor_right_for,motor_left_for,100);
                break;
            }
            break;
        }
        break;
      case false:
        switch (left_sta) {
          case true:
            switch (right_sta) {
              case true:
                //FTT
                //worst situation
                if(inf_loop){
                  turn(motor_right_for,560);                
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = front_dir;
                  inf_loop = false;
                }
                else{
                  turn(motor_left_for,560);                
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = front_dir;
                  inf_loop = true;
                }
                turn(motor_left_for,560);                
                for_back(motor_right_for,motor_left_for,1000);
                last_cross = front_dir;                
                break;
              case false:
                //FTF
                for_back(motor_right_for,motor_left_for,150);
                turn(motor_right_for,560); 
                for_back(motor_right_for,motor_left_for,1000);                
                break;
            }
            break;
          case false:
            switch (right_sta) {
              case true:
                //FFT
                for_back(motor_right_for,motor_left_for,150);
                turn(motor_left_for,560); 
                for_back(motor_right_for,motor_left_for,1000);                
                break;
              case false:
                //FFF                
                back = true;
                go = false;
                break;
            }
            break;
        }
        break;
    }
  }
  else if(back==true){
     brcClient.sendToClient(PARTNER_COMM_ID,"go!");
     back=false;   
  }
}




