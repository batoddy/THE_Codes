#include <SPI.h>
#include <LoRa.h>
#include <RadioLib.h>

#define LORA_FREQUENCY 434.0
#define LORA_SPREADING 6
#define LORA_BANDWIDTH 512.0

#define LORA_DATA_SIZE 23

// LoRa defined pins
#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5
#define RST 21
#define DI1 25


typedef struct DCC_DATA
{
  uint32_t system_time;
  int flight_state;
  uint8_t stabilization_state;
  float altitude;
  float max_altitude;
  float vertical_velocity;
  
  float yaw;0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
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

SX1278 radio = new Module(5, 22, 21, 25);
uint8_t data_to_Lora[LORA_DATA_SIZE];

char rx_char;
char rx_data_porsion[10];
int char_to_data_counter = 0;
int dataer_to_data_counter = 0;
float rx_data;


//----------------------
void init_Lora();
void Lora_send();
void pack_data_for_Lora();
//----------------------


uint16_t CRCCalculator(const unsigned char *buf, unsigned int len);
int time1,delay_time;

void setup() {
   Serial.begin(115200);
  init_Lora();
  time1 = millis();
  

}

void loop() {
   pack_data_for_Lora();
   Lora_send();
   delay_time = millis()-time1;
   time1 = millis();
   Serial.print(delay_time);
   Serial.println(" ms");
   

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

  data_to_Lora[6] = 1; // flight state 1byte

  data_to_Lora[7] = 0; // stabilization flag 1 byte

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
 /* data_to_Lora[20] = f_to_u8.u8[0]; // yaw float 4 byte
  data_to_Lora[21] = f_to_u8.u8[1]; //
  data_to_Lora[22] = f_to_u8.u8[2]; //
  data_to_Lora[23] = f_to_u8.u8[3]; //*/

  f_to_u8.float32 = 15/*DCC_data.pitch*/;
 /* data_to_Lora[24] = f_to_u8.u8[0]; // pitch float 4 byte
  data_to_Lora[25] = f_to_u8.u8[1]; //
  data_to_Lora[26] = f_to_u8.u8[2]; //
  data_to_Lora[27] = f_to_u8.u8[3]; //*/

  f_to_u8.float32 = 12.4/*DCC_data.roll*/;
  /*data_to_Lora[28] = f_to_u8.u8[0]; // roll float 4 byte
  data_to_Lora[29] = f_to_u8.u8[1]; //
  data_to_Lora[30] = f_to_u8.u8[2]; //
  data_to_Lora[31] = f_to_u8.u8[3]; //*/

  f_to_u8.float32 = 12.15481/*LTC_gps.utc_time*/;
 /* data_to_Lora[32] = f_to_u8.u8[0]; // utc time float 4 byte
  data_to_Lora[33] = f_to_u8.u8[1]; //
  data_to_Lora[34] = f_to_u8.u8[2]; //
  data_to_Lora[35] = f_to_u8.u8[3]; //*/

  f_to_u8.float32 = 45.2452/*LTC_gps.latitude*/;
  /*data_to_Lora[36] = f_to_u8.u8[0]; // LTC_latitude float 4 byte
  data_to_Lora[37] = f_to_u8.u8[1]; //
  data_to_Lora[38] = f_to_u8.u8[2]; //
  data_to_Lora[39] = f_to_u8.u8[3]; //*/

  f_to_u8.float32 = 51.5454/*LTC_gps.longtitude*/;
  /*data_to_Lora[40] = f_to_u8.u8[0]; // LTC_longtitude float 4 byte
  data_to_Lora[41] = f_to_u8.u8[1]; //
  data_to_Lora[42] = f_to_u8.u8[2]; //
  data_to_Lora[43] = f_to_u8.u8[3]; //*/

  f_to_u8.float32 = 41.4528/*ctc_data.latitude*/;             // will changed with ctc data
 /* data_to_Lora[44] = f_to_u8.u8[0]; // CTC_latitude float 4 byte
  data_to_Lora[45] = f_to_u8.u8[1]; //
  data_to_Lora[46] = f_to_u8.u8[2]; //
  data_to_Lora[47] = f_to_u8.u8[3]; //*/

  f_to_u8.float32 = 44.582/*ctc_data.longtitude*/;             // will changed with ctc data
  /*data_to_Lora[48] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[49] = f_to_u8.u8[1]; //
  data_to_Lora[50] = f_to_u8.u8[2]; //
  data_to_Lora[51] = f_to_u8.u8[3]; //*/

 /* data_to_Lora[52] = 43.354/*(uint8_t)DCC_data.rocket_max_velocity*/; // CTC_longtitude float 4 byte
 /* data_to_Lora[53] = 45.657/*(uint8_t)(-1)*DCC_data.payload_max_velocity*/; //

  /*int16_to_u8.int16 = 23.324/*DCC_data.accel_resultant*/;
  /*data_to_Lora[54] = int16_to_u8.u8[0]; //
  data_to_Lora[55] = int16_to_u8.u8[1]; // v

  f_to_u8.float32 = 13.234/*DCC_data.DC_thrust*/;             // will changed with ctc data
 /* data_to_Lora[56] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[57] = f_to_u8.u8[1]; //
  data_to_Lora[58] = f_to_u8.u8[2]; //
  data_to_Lora[59] = f_to_u8.u8[3]; // v

  f_to_u8.float32 = 13.234/*DCC_data.servo1_angle*/;             // will changed with ctc data
 /* data_to_Lora[60] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[61] = f_to_u8.u8[1]; //
  data_to_Lora[62] = f_to_u8.u8[2]; //
  data_to_Lora[63] = f_to_u8.u8[3]; //

  f_to_u8.float32 = 13.234/*DCC_data.servo2_angle*/;             // will changed with ctc data
 /* data_to_Lora[64] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[65] = f_to_u8.u8[1]; //
  data_to_Lora[66] = f_to_u8.u8[2]; //
  data_to_Lora[67] = f_to_u8.u8[3]; //

  f_to_u8.float32 = 13.234/*DCC_data.servo3_angle*/;             // will changed with ctc data
 /* data_to_Lora[68] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[69] = f_to_u8.u8[1]; //
  data_to_Lora[70] = f_to_u8.u8[2]; //
  data_to_Lora[71] = f_to_u8.u8[3]; // v

  f_to_u8.float32 = 13.234/*DCC_data.servo4_angle*/;             // will changed with ctc data
 /* data_to_Lora[72] = f_to_u8.u8[0]; // CTC_longtitude float 4 byte
  data_to_Lora[73] = f_to_u8.u8[1]; //
  data_to_Lora[74] = f_to_u8.u8[2]; //
  data_to_Lora[75] = f_to_u8.u8[3]; //
 
  data_to_Lora[76] = 1/*(uint8_t)DCC_data.temperature*/;         // will changed with ctc data
  
  
 /* data_to_Lora[77] = int16_to_u8.u8[0]; //
  data_to_Lora[78] = int16_to_u8.u8[1]; // v
  int16_to_u8.int16 = 13/*DCC_data.pressure_int2*/;
  /*
  data_to_Lora[79] = f_to_u8.u8[3]; //*/

  uint16_t crc = CRCCalculator(data_to_Lora, 4);
  u16_to_u8.u16 = crc;
  data_to_Lora[21] = u16_to_u8.u8[0];
  data_to_Lora[22] = u16_to_u8.u8[1];

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
     Serial.println(F(" success!"));

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
