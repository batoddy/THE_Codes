#include <esp_now.h>
#include <WiFi.h>

esp_now_peer_info_t device;
#define CHANNEL 1

esp_now_peer_info_t peerInfo;
uint8_t reciever_Addr[] = {0x34,0x94,0x54,0xD5,0x7F,0x9C}; //0x70,0xB8,0xF6,0x3E,0x3A,0x68
char* data_20 = "qwerqwerqwerqwerqwe";
char recieved_data[20];
int latency = millis();

void data_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status){
   Serial.print("Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

}

void data_recieve_callback(const uint8_t *mac_addr, const uint8_t *incomingData, int len){
  for (int i = 0; i < 200; i++){
    recieved_data[i] = incomingData[i];
  }
  
  Serial.print("Data: ");
  Serial.println(recieved_data);
  Serial.println(millis() - latency);
  latency = millis();
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  WiFi.disconnect();

  if(esp_now_init()==ESP_OK)
    Serial.println("ESPNow Init Sucess!");
  else{
    Serial.println("ESPNow Init Fail!");
    ESP.restart();
  }
  memcpy(peerInfo.peer_addr, reciever_Addr, 6);
  peerInfo.channel = CHANNEL;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  esp_now_register_send_cb(data_send_callback);
  esp_now_register_recv_cb(data_recieve_callback);
}
void loop(){
  // esp_now_send(reciever_Addr,(uint8_t*)&data_20,sizeof(data_20));
}
