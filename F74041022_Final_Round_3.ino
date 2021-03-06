#include <Ultrasonic.h>
#include <SPI.h>
#include <BRCClient.h>
#include <RFID.h>
#include <MapMsg.h>

#define front_trig 26
#define front_echo 27
#define left_trig 22
#define left_echo 23
#define right_trig 24
#define right_echo 25
#define motor_right_back 7
#define motor_right_for 6
#define motor_left_back 5
#define motor_left_for 4
#define LED 12

#define front_dir 0
#define left_dir 1
#define right_dir 2

#define UART_RX 10
#define UART_TX 2

//#define AP_SSID "PoPo"
//#define AP_PASSWD "07270727"
//#define TCP_IP "192.168.43.1"

#define AP_SSID    "Test_Server_AP"
#define AP_PASSWD  "testserverap"
#define TCP_IP     "192.168.43.1"

//#define AP_SSID    "programtheworld"
//#define AP_PASSWD  "screamlab"
//#define TCP_IP     "192.168.150.11"

#define TCP_PORT   5000
#define MY_COMM_ID 0x23


#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCLK 52
#define SPI_SS   53
#define MFRC522_RSTPD 11

Ultrasonic front(front_trig, front_echo);
Ultrasonic left(left_trig, left_echo);
Ultrasonic right(right_trig, right_echo);
BRCClient brcClient(UART_RX, UART_TX);
RFID rfid(SPI_SS, MFRC522_RSTPD);

int v = 150,last_cross, map_i, map_j;
float front_dis, right_dis, left_dis;
bool back = false, front_sta, left_sta, right_sta,go=false,finish=false;
bool mapping[6][6] = {false},inf_loop = false;

void for_back(int mot_1,int mot_2,int dur){
  analogWrite(mot_1, v);
  analogWrite(mot_2, v+30);
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
    for_back(motor_right_back,motor_left_back,430);
    front_sta = true;
    left_sta = true;
    right_sta = true;
  }
  else if (left < 5) {
    turn(motor_right_back,200);
    front_sta = true;
    left_sta = true;
    right_sta = true;
  }
  else if (right < 5 ){
    turn(motor_left_back,170);
    front_sta = true;
    left_sta = true;
    right_sta = true;
  }
}
void setup() {
  pinMode(LED,OUTPUT);
  pinMode(motor_right_back, OUTPUT);
  pinMode(motor_right_for, OUTPUT);
  pinMode(motor_left_back, OUTPUT);
  pinMode(motor_left_for, OUTPUT);
  analogWrite(motor_right_for, 0);
  analogWrite(motor_right_back, 0);
  analogWrite(motor_left_for, 0);
  analogWrite(motor_left_back, 0);

  Serial.begin(9600);

  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000L, MSBFIRST, SPI_MODE3));

  rfid.begin();

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
  //distance
  long front_sec = front.timing(), left_sec = left.timing(), right_sec = right.timing();
  front_dis = front.convert(front_sec, Ultrasonic::CM);
  left_dis = left.convert(left_sec, Ultrasonic::CM);
  right_dis = right.convert(right_sec, Ultrasonic::CM);

  Serial.println(front_dis);
  Serial.println(left_dis);
  Serial.println(right_dis);

  CommMsg msg;
  if (brcClient.receiveMessage(&msg)) {
    if (msg.type==MSG_ROUND_START){
      go=true;
    }
    else if(msg.type==MSG_ROUND_END){
      go=false;
    }
  }

  //way
  if(go==true){
  if ((status = rfid.findTag(&card_type)) == STATUS_OK) {
    digitalWrite(LED,HIGH);
    Serial.print("OK! ");
    Serial.println(card_type);
    if ((status = rfid.readTagSN(sn, &snBytes)) == STATUS_OK) {
      rfid.piccHalt();
      brcClient.requestMapData(sn);
      for_back(motor_right_for,motor_left_for,300);
    }
    delay(1000);
  }else{
    digitalWrite(LED,LOW);
    Serial.println("No tag.");
  }

  //line
  line(front_dis,left_dis,right_dis);
  //move forward
  if (!back) {
    if (front_dis <= 8) front_sta = false;
    else  front_sta = true;

    if (left_dis >= 35) left_sta = true;
    else  left_sta = false;

    if (right_dis >= 35)  right_sta = true;
    else  right_sta = false;

    switch (front_sta) {
      case true:
        switch (left_sta) {
          case true:
            switch (right_sta) {
              case true:
                //TTT
                for_back(motor_right_for,motor_left_for,1000);
                break;
              case false:
                //TTF
                if(front_dis>left_dis){
                  for_back(motor_right_for,motor_left_for,160);
                  turn(motor_right_for,560);
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = left_dir;
                }
                else{
                  for_back(motor_right_for,motor_left_for,500);
                  last_cross = right_dir;
                }
                break;
            }
            break;
          case false:
            switch (right_sta) {
              case true:
                //TFT
                if(front_dis>right_dis){
                  for_back(motor_right_for,motor_left_for,160);
                  turn(motor_left_for,560);
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = right_dir;
                }
                else{
                  for_back(motor_right_for,motor_left_for,500);
                  last_cross = left_dir;
                }
                break;
              case false:
                //TFF
                for_back(motor_right_for,motor_left_for,70);
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
                if(left_dis>right_dis){
                  for_back(motor_right_for,motor_left_for,160);
                  turn(motor_left_for,560);
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = front_dir;
                }
                else{
                  for_back(motor_right_for,motor_left_for,160);
                  turn(motor_right_for,560);
                  for_back(motor_right_for,motor_left_for,1000);
                  last_cross = front_dir;
                }
                break;
              case false:
                //FTF
                for_back(motor_right_for,motor_left_for,160);
                turn(motor_right_for,560);
                for_back(motor_right_for,motor_left_for,1000);
                break;
            }
            break;
          case false:
            switch (right_sta) {
              case true:
                //FFT
                for_back(motor_right_for,motor_left_for,160);
                turn(motor_left_for,560);
                for_back(motor_right_for,motor_left_for,1000);
                break;
              case false:
                //FFF
                if(right_dis>left_dis){
                  for_back(motor_right_back,motor_left_for,560);
                }
                else{
                  for_back(motor_right_for,motor_left_back,620);
                }
                back = true;
                break;
            }
            break;
        }
        break;
    }
  }
  else {
    if (left_dis >= 100) left_sta = true;
    else  left_sta = false;

    if (right_dis >= 100)  right_sta = true;
    else  right_sta = false;

    //backward
    if (left_sta != false || right_sta != false) {
      switch (last_cross) {
        case 0:
          for_back(motor_right_for,motor_left_for,1000);
          break;
        case 1:
          for_back(motor_right_for,motor_left_for,300);
          turn(motor_right_for,560);
          for_back(motor_right_for,motor_left_for,1000);
          break;
        case 2:
          for_back(motor_right_for,motor_left_for,300);
          turn(motor_left_for,560);
          for_back(motor_right_for,motor_left_for,1000);
          break;
      }
      back = false;
    }
    else {      
      for_back(motor_right_for,motor_left_for,70);
    }
  }
  }
}
