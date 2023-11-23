#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include <LoRa.h>

#include <RadioLib.h>

#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1
//-------------------THE-LORA-PARAMETERS------------------//

#define LORA_FREQUENCY 434.0
#define LORA_SPREADING 7
#define LORA_BANDWIDTH 250.0

//---------------------------------------------------------//

#define LORA_DATA_SIZE 81
#define WIFI_DATA_SIZE 9

// LoRa defined pins
#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5
#define RST 21
#define DI1 25
#define DIO 22

typedef struct GPS
{
  float utc_time;
  float date_val;
  float longtitude;
  float latitude;
  uint8_t satellite_val;
  float hdop_val;
} GPS;

typedef struct DCC_DATA
{
  uint32_t system_time;
  int flight_state;
  uint8_t stabilization_state;
  float altitude;
  float max_altitude;
  float vertical_velocity;
  
  float yaw;
  float pitch;
  float roll;
  float accel_z;
  float pressure;
  float DC_thrust;
  float servo1_angle;
  float servo2_angle;
  float servo3_angle;
  float servo4_angle;
  int8_t rocket_max_velocity;
  int8_t payload_max_velocity;
  uint8_t temperature;
  int16_t pressure_int2;
  int16_t accel_resultant;
  // float pressure;
  // float base_pressure;
  // float accel_x;
  //float accel_y;
  //float accel_z;
  //float gyro_x;
  //float gyro_y;
  //float gyro_z;
};

typedef struct CTC_DATA
{
  float latitude;
  float longtitude;
  float max_accel;
};

uint8_t wifi_connection_flag = 0;

union int16_to_uint8
{
  int16_t int16;
  int8_t u8[2];
};

union uint16_to_uint8
{
  int16_t u16;
  int8_t u8[2];
};

union float_to_uint8
{
  float float32;
  int8_t u8[4];
};

union uint32_t_to_uint8
{
  uint32_t u32;
  int8_t u8[4];
};

static const int RXPin = 17, TXPin = 16; // GPS Initilaize
static const uint32_t GPSBaud = 9600;    //

TinyGPSPlus gps; // The TinyGPSPlus object

SX1278 radio = new Module(5, 25, 21, 22);

SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

DCC_DATA DCC_data;
CTC_DATA ctc_data;
GPS LTC_gps;

uint8_t data_to_Lora[LORA_DATA_SIZE];

char rx_char;
char rx_data_porsion[10];
int char_to_data_counter = 0;
int dataer_to_data_counter = 0;
float rx_data;

// FUNCTION PROTOTYPES
void init_GPS();
void get_GPS();
void print_GPS();
//----------------------
void init_Lora();
void Lora_send();
void pack_data_for_Lora();
//----------------------
void read_from_UART();
//----------------------
void init_WIFI(); 
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len);
//----------------------
uint16_t CRCCalculator(const unsigned char *buf, unsigned int len);
int time1;
void setup()
{
  Serial.begin(115200);
  // Serial.setTimeout(2);
  init_GPS();
  init_Lora();
  init_WIFI();
  time1 = millis();
}

void loop()
{
  get_GPS();
  // Serial.println("Code loopin");
  // print_GPS();
  read_from_UART();
  pack_data_for_Lora();
    Lora_send();
   Serial.print("Lat: ");
   Serial.print(ctc_data.latitude);
   Serial.print("Lng: ");
   Serial.println(ctc_data.longtitude);
  
  /*
  if(DCC_data.flight_state == 0){
    if(millis() - time1 > 5000){
      pack_data_for_Lora();
      Lora_send();
      time1 = millis();
    }
  }
  else if(DCC_data.flight_state > 0){
    pack_data_for_Lora();
    Lora_send();
  }*/
  
  delay(50);
  // // Serial.print("CTC lat:\t");
  // // Serial.println(ctc_data.latitude);
  // // Serial.print("CTC lng:\t");
  // // Serial.println(ctc_data.longtitude);
  /*Serial.write(data_to_Lora,sizeof(data_to_Lora)+1);
  // // Serial.println("");*/
}
/////-------------------------SERIAL-READ----------------/////
char DCC_data_str[78];
bool str_finish_flag = false;
int rx_char_int;
void read_from_UART() // parsing the UART data from DCC
{
  while (Serial.available() != 0)
  {
    rx_char = Serial.read();
    // ss.println(rx_char);
    delayMicroseconds(150);
    if (rx_char != '\n')
    {

      if (rx_char != ',')
      {
        rx_data_porsion[char_to_data_counter] = rx_char;
        char_to_data_counter++;
      }
      else if (rx_char == ',')
      {
        char_to_data_counter = 0;
        //rx_data = atoi(rx_data_porsion);
        //DCC_data.system_time = rx_data;
        dataer_to_data_counter++;

        switch (dataer_to_data_counter)
        {
        case 1:
        {
          rx_data = atoi(rx_data_porsion);
          DCC_data.system_time = rx_data;
          //ss.print("Tick: ");
          //ss.println(DCC_data.system_time);
          break;
        }
        case 2:
        {
          rx_char_int = atoi(rx_data_porsion);
          DCC_data.flight_state = rx_char_int;
          //ss.println("flight_state: ");
          //ss.println(DCC_data.flight_state);
          break;
        }
        case 3:
        {
          rx_char_int = atoi(rx_data_porsion);
          DCC_data.stabilization_state = rx_char_int;
          //ss.println("stabilization_state: ");
          //ss.println(DCC_data.stabilization_state);
          break;
        }
        case 4:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.altitude = rx_data;
          //ss.print("Altitude: ");
          //ss.println(DCC_data.altitude);
          break;
        }
        case 5:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.pressure = rx_data;
          //ss.print("pressure: ");
          //ss.println(DCC_data.pressure);
          break;
        }
        case 6:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.vertical_velocity = rx_data;
          //ss.print("vertical_velocity: ");
          //ss.println(DCC_data.vertical_velocity);
          break;
        }
        case 7:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.accel_z = rx_data;
         // ss.print("resultant_accel: ");
         // ss.println(DCC_data.resultant_accel);
          break;
        }
        case 8:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.yaw = rx_data;
         // ss.print("Yaw: ");
         // ss.println(rx_data);
          break;
        }
        case 9:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.pitch = rx_data;
          //ss.print("pitch: ");
          //ss.println(rx_data);
          break;
        }
        case 10:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.roll = rx_data;
          //ss.print("roll: ");
          //ss.println(rx_data);
          break;
        }
        case 11:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.DC_thrust = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 12:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.servo1_angle = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 13:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.servo2_angle = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 14:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.servo3_angle = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 15:
        {
          rx_data = atof(rx_data_porsion);
          DCC_data.servo4_angle = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 16:
        {
          // rx_data = atof(rx_data_porsion);
          DCC_data.rocket_max_velocity = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 17:
        {
          // rx_data = atof(rx_data_porsion);
          DCC_data.payload_max_velocity = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 18:
        {
          rx_data = atoi(rx_data_porsion);
          DCC_data.temperature = rx_data;
          // ss.println(rx_data);
          break;
        }
        case 19:
        {
          rx_data = atoi(rx_data_porsion);
          DCC_data.pressure_int2 = rx_data;
          // ss.println(rx_data);
          break;
        }
       case 20:
        {
          rx_data = atoi(rx_data_porsion);
          DCC_data.accel_resultant= rx_data;
          // ss.println(rx_data);
          break;
        }
        
        }
      }
    }
    else if (rx_char == '\n')
    {
      dataer_to_data_counter = 0;
      break;
    }
    /*
    else if(rx_char == '\n'){
      if(str_finish_flag == true){
        dataer_to_data_counter = 0;
        str_finish_flag = false;
      }

      else if(str_finish_flag == false){
        str_finish_flag = true;
      }
      */
  }
  
}

/////---------------------------GPS----------------------/////
void init_GPS() // initing gps
{
  ss.begin(GPSBaud);
  // // Serial.println("GPS Begin!");
}

void get_GPS() // parsing gps data
{
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        // // Serial.println(F("No GPS detected: check wiring."));
        
      }
  LTC_gps.latitude = gps.location.lat();
  LTC_gps.longtitude = gps.location.lng();
  LTC_gps.satellite_val = gps.satellites.value();
  LTC_gps.hdop_val = gps.hdop.value();
  LTC_gps.utc_time = gps.time.value();
  LTC_gps.date_val = gps.date.value();
}

void print_GPS()
{
  // Serial.print("Sat: ");
  // Serial.print(LTC_gps.satellite_val);
  // Serial.print(" || UTC: ");
  // Serial.print(LTC_gps.utc_time);
  // Serial.print(" Lat: ");
  // Serial.print(LTC_gps.latitude);
  // Serial.print(" Lng: ");
  // Serial.print(LTC_gps.longtitude);
  // Serial.print(" Rx: ");
  // Serial.println(gps.charsProcessed());
}
/////--------------------------LORA------------------/////
void init_Lora() // initing lora
{
  // initialize SX1278 with default settings
  // Serial.print(F("[SX1278] Initializing ... "));
  SPI.begin(SCK, MISO, MOSI, SS);
  int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING);
  delay(50);
  // Begin method:
  // Carrier frequency: 434.0 MHz (for SX1276/77/78/79 and RFM96/98) or 915.0 MHz (for SX1272/73 and RFM95/97)
  // Bandwidth: 125.0 kHz (dual-sideband)
  // Spreading factor: 9
  // Coding rate: 4/7
  // Sync word: SX127X_SYNC_WORD (0x12)
  // Output power: 10 dBm
  // Preamble length: 8 symbols
  // Gain: 0 (automatic gain control enabled)
  if (state == RADIOLIB_ERR_NONE)
  {
    // Serial.println(F("success!"));
  }
  else
  {
    // Serial.print(F("failed, code "));
    // Serial.println(state);
  }
}

uint16_t pkg_no;
uint32_t system_time; // ms
uint32_t system_time2;

void pack_data_for_Lora() // packing data to lora
{   

  union float_to_uint8 f_to_u8;
  union uint16_to_uint8 u16_to_u8;
  union int16_to_uint8 int16_to_u8;
  union uint32_t_to_uint8 u32_to_u8;
  union uint32_t_to_uint8 a;

  u32_to_u8.u32 = 0;

  system_time = millis();
  u32_to_u8.u32 = system_time;             //------------------------
  data_to_Lora[0] = u32_to_u8.u8[0]; // tick float 4 byte ms
  data_to_Lora[1] = u32_to_u8.u8[1]; //
  data_to_Lora[2] = u32_to_u8.u8[2]; //
  data_to_Lora[3] = u32_to_u8.u8[3]; //

  u16_to_u8.u16 = pkg_no;
  data_to_Lora[4] = u16_to_u8.u8[0];
  data_to_Lora[5] = u16_to_u8.u8[1];

  data_to_Lora[6] = (uint8_t)DCC_data.flight_state; // flight state 1byte

  data_to_Lora[7] = (uint8_t)DCC_data.stabilization_state; // stabilization flag 1 byte

  f_to_u8.float32 =75 /*DCC_data.altitude*/;                 //----------------------
  data_to_Lora[8] = f_to_u8.u8[0]; // altitude int16 2 byte
  data_to_Lora[9] = f_to_u8.u8[1]; //
  data_to_Lora[10] = f_to_u8.u8[2];
  data_to_Lora[11] = f_to_u8.u8[3];
  
  f_to_u8.float32 = 12/*DCC_data.vertical_velocity*/;
  data_to_Lora[12] = f_to_u8.u8[0]; // velocity float 4 byte
  data_to_Lora[13] = f_to_u8.u8[1]; //
  data_to_Lora[14] = f_to_u8.u8[2]; //
  data_to_Lora[15] = f_to_u8.u8[3]; //

  f_to_u8.float32 =54 /*DCC_data.accel_z*/;
  data_to_Lora[16] = f_to_u8.u8[0]; // accel float 4 byte
  data_to_Lora[17] = f_to_u8.u8[1]; //
  data_to_Lora[18] = f_to_u8.u8[2]; //
  data_to_Lora[19] = f_to_u8.u8[3]; //

  f_to_u8.float32 = 54/*DCC_data.yaw*/;
  data_to_Lora[20] = f_to_u8.u8[0]; // yaw float 4 byte
  data_to_Lora[21] = f_to_u8.u8[1]; //
  data_to_Lora[22] = f_to_u8.u8[2]; //
  data_to_Lora[23] = f_to_u8.u8[3]; //

  f_to_u8.float32 = 15/*DCC_data.pitch*/;
  data_to_Lora[24] = f_to_u8.u8[0]; // pitch float 4 byte
  data_to_Lora[25] = f_to_u8.u8[1]; //
  data_to_Lora[26] = f_to_u8.u8[2]; //
  data_to_Lora[27] = f_to_u8.u8[3]; //

  f_to_u8.float32 = 12/*DCC_data.roll*/;
  data_to_Lora[28] = f_to_u8.u8[0]; // roll float 4 byte
  data_to_Lora[29] = f_to_u8.u8[1]; //
  data_to_Lora[30] = f_to_u8.u8[2]; //
  data_to_Lora[31] = f_to_u8.u8[3]; //

  f_to_u8.float32 = LTC_gps.utc_time;
  data_to_Lora[32] = f_to_u8.u8[0]; // utc time float 4 byte
  data_to_Lora[33] = f_to_u8.u8[1]; //
  data_to_Lora[34] = f_to_u8.u8[2]; //
  data_to_Lora[35] = f_to_u8.u8[3]; //

  f_to_u8.float32 = LTC_gps.latitude;
  data_to_Lora[36] = f_to_u8.u8[0]; // LTC_latitude float 4 byte
  data_to_Lora[37] = f_to_u8.u8[1]; //
  data_to_Lora[38] = f_to_u8.u8[2]; //
  data_to_Lora[39] = f_to_u8.u8[3]; //

  f_to_u8.float32 = LTC_gps.longtitude;
  data_to_Lora[40] = f_to_u8.u8[0]; // LTC_longtitude float 4 byte
  data_to_Lora[41] = f_to_u8.u8[1]; //
  data_to_Lora[42] = f_to_u8.u8[2]; //
  data_to_Lora[43] = f_to_u8.u8[3]; //

  f_to_u8.float32 = ctc_data.latitude;             // will changed with ctc data
  data_to_Lora[44] = f_to_u8.u8[0]; // CTC_latitude float 4 byte
  data_to_Lora[45] = f_to_u8.u8[1]; //
  data_to_Lora[46] = f_to_u8.u8[2]; //
  data_to_Lora[47] = f_to_u8.u8[3]; //

  f_to_u8.float32 = ctc_data.longtitude;             // will changed with ctc data
  data_to_Lora[48] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[49] = f_to_u8.u8[1]; //
  data_to_Lora[50] = f_to_u8.u8[2]; //
  data_to_Lora[51] = f_to_u8.u8[3]; //

  data_to_Lora[52] = (uint8_t)DCC_data.rocket_max_velocity; // CTC_longtitude float 4 byte
  data_to_Lora[53] = (uint8_t)(-1)*DCC_data.payload_max_velocity; //

  int16_to_u8.int16 = DCC_data.accel_resultant;
  data_to_Lora[54] = int16_to_u8.u8[0]; //
  data_to_Lora[55] = int16_to_u8.u8[1]; // v

  f_to_u8.float32 = DCC_data.DC_thrust;             // will changed with ctc data
  data_to_Lora[56] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[57] = f_to_u8.u8[1]; //
  data_to_Lora[58] = f_to_u8.u8[2]; //
  data_to_Lora[59] = f_to_u8.u8[3]; // v

  f_to_u8.float32 = DCC_data.servo1_angle;             // will changed with ctc data
  data_to_Lora[60] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[61] = f_to_u8.u8[1]; //
  data_to_Lora[62] = f_to_u8.u8[2]; //
  data_to_Lora[63] = f_to_u8.u8[3]; //

  f_to_u8.float32 = DCC_data.servo2_angle;             // will changed with ctc data
  data_to_Lora[64] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[65] = f_to_u8.u8[1]; //
  data_to_Lora[66] = f_to_u8.u8[2]; //
  data_to_Lora[67] = f_to_u8.u8[3]; //

  f_to_u8.float32 = DCC_data.servo3_angle;             // will changed with ctc data
  data_to_Lora[68] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[69] = f_to_u8.u8[1]; //
  data_to_Lora[70] = f_to_u8.u8[2]; //
  data_to_Lora[71] = f_to_u8.u8[3]; // v

  f_to_u8.float32 = DCC_data.servo4_angle;             // will changed with ctc data
  data_to_Lora[72] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[73] = f_to_u8.u8[1]; //
  data_to_Lora[74] = f_to_u8.u8[2]; //
  data_to_Lora[75] = f_to_u8.u8[3]; //
 
  data_to_Lora[76] = (uint8_t)DCC_data.temperature;         // will changed with ctc data
  
  
  data_to_Lora[77] = int16_to_u8.u8[0]; //
  data_to_Lora[78] = int16_to_u8.u8[1]; // v
  int16_to_u8.int16 = DCC_data.pressure_int2;
  /*
  data_to_Lora[79] = f_to_u8.u8[3]; //*/

  uint16_t crc = CRCCalculator(data_to_Lora, 4);
  u16_to_u8.u16 = crc;
  data_to_Lora[79] = u16_to_u8.u8[0];
  data_to_Lora[80] = u16_to_u8.u8[1];

  pkg_no += 1;
}

void Lora_send() // sending the data from LoRa
{
  // Serial.print(F("[SX1278] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to
  // 255 characters long

  // String batucuk = "hello niggı";
  // radio.transmit(batucuk);

  int state = radio.transmit(data_to_Lora, (LORA_DATA_SIZE + 1));
  // you can also transmit byte array up to 256 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE)
  {
    // the packet was successfully transmitted
    // Serial.println(F(" success!"));

    // print measured data rate
     Serial.print("System time:\t");
     Serial.println(system_time);
     Serial.print("Packet Number:\t");
     Serial.println(pkg_no);
     Serial.print(F("[SX1278] Datarate:\t"));
     Serial.print(radio.getDataRate());
     Serial.println(F(" bps"));
     Serial.print("[SX1278] Frequency:\t");
     Serial.print(LORA_FREQUENCY);
     Serial.println("MHz");
     Serial.print("[SX1278] Bandwidth:\t");
     Serial.print(LORA_BANDWIDTH);
     Serial.println("KHz");
     Serial.print("[SX1278] Spreading Factor:\t");
     Serial.println(LORA_SPREADING);
  }
  else if (state == RADIOLIB_ERR_PACKET_TOO_LONG)
  {
    // the supplied packet was longer than 256 bytes
     Serial.println(F("too long!"));
  }
  else if (state == RADIOLIB_ERR_TX_TIMEOUT)
  {
    // timeout occurred while transmitting packet
     Serial.println(F("timeout!"));
  }
  else
  {
    // some other error occurred
     Serial.print(F("failed, code "));
     Serial.println(state);
  }
}

uint16_t CRCCalculator(const unsigned char *buf, unsigned int len) // CRC func for packing checking
{
  uint16_t crc = 0xFFFF;
  char i = 0;

  while (len--)
  {
    crc ^= (*buf++);

    for (i = 0; i < 8; i++)
    {
      if (crc & 1)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/////-----------------WIFI----------------------/////
void init_WIFI(){
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_AP);
  
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_ Password", CHANNEL, 0);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    //Serial.println("ESPNow Init Success");
  }
  else {
    //Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    init_WIFI();
    // or Simply Restart
    //ESP.restart();
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}
  
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  union float_to_uint8 f_to_u8;
  Serial.print("Packet_Recieved..");
  
  // char *data = (char *)data;
  f_to_u8.u8[0] = data[0];
  f_to_u8.u8[1] = data[1];
  f_to_u8.u8[2] = data[2];
  f_to_u8.u8[3] = data[3];
  ctc_data.latitude = f_to_u8.float32;
  Serial.println(ctc_data.latitude);
  f_to_u8.u8[0] = data[4];
  f_to_u8.u8[1] = data[5];
  f_to_u8.u8[2] = data[6];
  f_to_u8.u8[3] = data[7];
  ctc_data.longtitude = f_to_u8.float32;
  Serial.println(ctc_data.longtitude);

}
