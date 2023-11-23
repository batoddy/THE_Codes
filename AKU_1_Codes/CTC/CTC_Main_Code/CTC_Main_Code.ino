#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
#define WIFI_DATA_SIZE 9

esp_now_peer_info_t slave;

// uint8_t broadcastAddress[] = {"0xA8, 0x42, 0xE3, 0x90, 0x81, 0xE4"};

union float_to_uint8
{
  float float32;
  int8_t u8[4];
};

typedef struct GPS
{
  float utc_time;
  float date_val;
  float longtitude;
  float latitude;
  uint8_t satellite_val;
  float hdop_val;
} GPS;


static const int RXPin = 16, TXPin = 17; // GPS Initilaize
static const uint32_t GPSBaud = 9600;    //

TinyGPSPlus gps; // The TinyGPSPlus object

SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

GPS CTC_gps;
esp_now_peer_info_t peerInfo;

// FUNCTION PROTOTYPES
void init_GPS();
void get_GPS();
void print_GPS();


void init_WIFI();
void pack_WIFI_data();
void send_via_WIFI();

////-----------------------////
void ScanForSlave();
bool manageSlave();
void deletePeer();

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
int time1;
void setup() {
  Serial.begin(115200);
  init_GPS();
  init_WIFI();

  time1 = millis();
}

void loop() {
  get_GPS();
  print_GPS();
  if(millis() - time1 > 10000){
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
      pack_WIFI_data();
      send_via_WIFI();
    } else {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }
  time1 = millis();
  }
}

void init_GPS() // initing gps
{
  ss.begin(GPSBaud);
  Serial.println("GPS Begin!");
}


void get_GPS() // parsing gps data
{
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("No GPS detected: check wiring."));
        
      }
  CTC_gps.latitude = gps.location.lat();
  CTC_gps.longtitude = gps.location.lng();
  CTC_gps.satellite_val = gps.satellites.value();
  CTC_gps.hdop_val = gps.hdop.value();
  CTC_gps.utc_time = gps.time.value();
  CTC_gps.date_val = gps.date.value();
}


void print_GPS()
{
  Serial.print("Sat: ");
  Serial.print(CTC_gps.satellite_val);
  Serial.print(" || UTC: ");
  Serial.print(CTC_gps.utc_time);
  Serial.print(" Lat: ");
  Serial.print(CTC_gps.latitude);
  Serial.print(" Lng: ");
  Serial.print(CTC_gps.longtitude);
  Serial.print(" Rx: ");
  Serial.println(gps.charsProcessed());
}

/////-----------------WIFI----------------------/////

void init_WIFI(){
  WiFi.mode(WIFI_STA);

   WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    // Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
     init_WIFI();
    // or Simply Restart
    //ESP.restart();
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);  
}

char data_to_LTC[WIFI_DATA_SIZE];

void pack_WIFI_data(){
  
  union float_to_uint8 f_to_u8;
  
  f_to_u8.float32 = CTC_gps.latitude;
  data_to_LTC[0] = f_to_u8.u8[0];
  data_to_LTC[1] = f_to_u8.u8[1];
  data_to_LTC[2] = f_to_u8.u8[2];
  data_to_LTC[3] = f_to_u8.u8[3];
  
  f_to_u8.float32 = CTC_gps.longtitude;
  data_to_LTC[4] = f_to_u8.u8[0];
  data_to_LTC[5] = f_to_u8.u8[1];
  data_to_LTC[6] = f_to_u8.u8[2];
  data_to_LTC[7] = f_to_u8.u8[3];
    
}

void send_via_WIFI(){

  const uint8_t *peer_addr = slave.peer_addr;

  esp_err_t result = esp_now_send(peer_addr, (uint8_t*)data_to_LTC, sizeof(data_to_LTC));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}



/////----------------------------ESPNOW-FUNCTIONS----------------------------/////

void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      // Slave already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}
