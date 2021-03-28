# MySensors

## Node List
### 1. Dummy
a test node without NodeManager framework, sends a solar voltage, and counts packetsOK and packetsNOK

uint16_t solarVoltage;        // child 1, V_VOLTAGE  
uint16_t packetsOK;           // child 1, V_VAR1  
uint16_t packetsNOK;          // child 1, V_VAR2  
uint8_t batLevel;             // internal  


### 2. Droppie
sensor handling the rainwater pump & tank

bool remoteOK;        // child 1  
uint16_t tankLiters;  // child 2  
bool tankEmpty;       // child 3  
bool batCharging;     // child 4  
uint8_t batLevel;     // internal  
uint32_t pumpStartsCount;   // child 5, V_VAR1  
uint32_t pumpOnTimeInSeconds; // child 5, V_VAR2  
uint32_t packetsCountOK;    // child 5, V_VAR3  
uint32_t packetsCountNOK;   // child 5, V_VAR4  

### 3. Tempie
weather data test sensor with BMP280 + SI7021

float bmp280Temperature;    // child 1  
float bmp280Pressure;       // child 2  
float si7021Temperature;    // child 4  
float si7021Humidity;       // child 5  
float batVoltage;           // child 201  
uint8_t batLevel;           // internal  

### 4. Wettie
a humidity sensor in the shower room, using NodeManager framework  

float dhtTemperature;   // child 1  
float dhtHumidity;      // child 2  
int32_t lightLevel;     // child 4, NodeManager sensor sends 0..100 level as int32_t  
float batVoltage;       // child 201  
uint8_t batLevel;       // internal  

### 5. Ventie
home ventilation sensor  
bool fanOn;         // child 1, V_STATUS  
char fanSpeed[7];   // child 1, V_HVAC_SPEED : "Min", "Normal","Max", "Auto" (and "Normal" not supported for us)  
bool alarmActive;   // child 2, V_STATUS  

## TODO gateway
- cloud logging verbeteren
- 