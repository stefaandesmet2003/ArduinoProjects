// a basic test to understand XnTcp connection to JMRI
// compile for esp8266

#include <Arduino.h>

// wifi stuff
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
// TCP server at port 61235 for XnTcp
WiFiServer server(61235);
WiFiClient client; // we ondersteunen maar 1 client = JMRI

enum parser_states
  {                            // actual state
     IDLE,
     WF_MESSAGE,
     WF_XOR,
     CHECK_STOLEN_LOK,
     CHECK_MANUAL_TURNOUT,
  } parser_state;

uint8_t pcc[16];          // pc_message speicher
uint8_t pcc_size, pcc_index;

uint8_t *pars_pcm;

//-- communication
uint8_t pcm_timeout[] = {0x01, 0x01};                 // Timeout
uint8_t pcm_overrun[] = {0x01, 0x06};                 // too many commands
uint8_t pcm_ack[] = {0x01, 0x04};                     // ack

// generell fixed messages
uint8_t pcm_datenfehler[] = {0x61, 0x80};             // xor wrong
uint8_t pcm_busy[] = {0x61, 0x81};                    // busy
uint8_t pcm_unknown[] = {0x61, 0x82};                 // unknown command
uint8_t pcm_BC_alles_aus[] = {0x61, 0x00};            // Kurzschluï¿½abschaltung
uint8_t pcm_BC_alles_an[] = {0x61, 0x01};             // DCC wieder einschalten
uint8_t pcm_BC_progmode[] = {0x61, 0x02};             // Progmode
uint8_t pcm_BC_locos_aus[] = {0x81, 0x00};            // Alle Loks gestoppt

uint8_t pcm_version[] = {0x63, 0x21, 0x36, 0x00};     // LZ100 Zentrale in Version 3.6
uint8_t pcm_liversion[] = {0x02, 0x10, 0x01};         // LI101F Version 1.0 Code 01
                             // {0x02, 0x30, 0x01};         // LIUSB 3.0


// variable messages
uint8_t pcm_status[] = {0x62, 0x22, 0x45};                  //wird von pc_send_status gebaut.
uint8_t pcm_build[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // max 6 bytes

void print_pcc () {
  for (int i=0;i<16;i++) {
    Serial.print(pcc[i], HEX); Serial.print(' ');
  }
  Serial.println();
}

// wifi version of pc_send_lenz
void pc_send_lenz(unsigned char *str)
{
  unsigned char n, total, my_xor;

  n = 0;
  my_xor = str[0];
  total = str[0] & 0x0F;

  client.write(str[0]);                   // send header
  while (n != total)
    {
        n++;
        my_xor ^= str[n];
        client.write(str[n]);              // send data
    }    
  client.write(my_xor);                   // send xor
} // pc_send_lenz

// dummy
void pc_send_status(){
  pcm_status[2] = 0;
  pc_send_lenz(pars_pcm = pcm_status);
} // pc_send_status


void parse_command(void)
{
  unsigned int addr;
  unsigned char speed = 0;
  unsigned char activate, coil;
  
  switch(pcc[0] / 16)   // this is opcode
  {
    default:
        break;
    case 0x2:
      switch(pcc[1])
      {
        default:
          break;
        case 0x21:
          // Softwareversion anfordern 0x21 0x21 0x00
          // 0 = "LZ 100";
          // 1 = "LH 200";
          // 2 = "DPC";
          // 3 = "Control Plus";
          pc_send_lenz(pars_pcm = pcm_version);
          return;
          break;
        case 0x24:
          // Status Zentrale anfordern 0x21 0x24 0x05 ---> wird von TC benutzt!
          pc_send_status();                          // Statusbyte zurï¿½ckliefern 
          return;            
          break;
        case 0x80:
          // Alles Aus 0x21 0x80 0xA1 ---> Nothalt
          pc_send_lenz(pars_pcm = pcm_ack);    // buggy?
          //TODO set_opendcc_state(RUN_OFF);
          // SDS uncomment for test
          pc_send_lenz(pars_pcm = pcm_BC_alles_aus);  // passiert mit state
          return;
          break;
        case 0x81:
          // Alles An 0x21 0x81 0xA0
          pc_send_lenz(pars_pcm = pcm_ack);    // buggy?
          //set_opendcc_state(RUN_OKAY);
          // SDS uncomment for test
          pc_send_lenz(pars_pcm = pcm_BC_alles_an);    // das wird bereits von set_opendcc_state gemacht
          return;
          break;
      }

    case 0xF:
      if (pcc[0] == 0xF0) // ask LI-Version
      {
        pc_send_lenz(pars_pcm = pcm_liversion);
        return;
      }
      switch(pcc[1])
      {
        default:
            break;
        case 0x01:   // ask / set slot addr
            if ((pcc[2] < 1) || (pcc[2] > 31)) pcc[2] = 1;  // if out of range: set to 1
            pc_send_lenz(&pcc[0]);
            return;
        case 0x02:    // setze Baud (getestet 19.05.2006)
                      // Antwort: F2 02 Baud, wie Aufruf (Antwort noch in der alten Baudrate, dann umschalten)
                      // BAUD = 1 19200 baud (Standardeinstellung)
                      // BAUD = 2 38400 baud
                      // BAUD = 3 57600 baud
                      // BAUD = 4 115200 baud
                      // nach BREAK (wird als 000) empfangen sollte Interface default auf 19200
                      // schalten
            if ((pcc[2] < 1) || (pcc[2] > 4)) pcc[2] = 1;
            pc_send_lenz(&pcc[0]);
            return;
      }
  }
  Serial.print("unknown command : ");
  print_pcc();

  pc_send_lenz(pars_pcm = pcm_unknown);   // wer bis hier durchfï¿½llt, ist unbekannt!
} // parse_command

void setup_wifi() {
  // Connect to WiFi network
  //WiFi.mode(WIFI_STA);
  WiFi.begin("TP-LINK", "7EB9E33C8C");
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
} // setup_wifi


void setup_mdns() {
  if (!MDNS.begin("espXpnet")) {
      Serial.println("Error setting up MDNS responder!");
      return;
  }
} // setup_mdns


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  
  // setup wifi
  Serial.println("Setting up wifi...");
  setup_wifi();
  setup_mdns();
  // Start TCP (HTTP) server
  server.begin();
  
} // setup

void server_loop(void)
{
  uint8_t my_check;

  if (!client.connected()) {
    // wait for client to connect
    Serial.println("waiting for client to connect");
    do {
      client = server.available();
    }
    while (!client);
    Serial.println("client connected");
  }
  
  if (!client.available()) return;
  switch (parser_state)
  {
    case IDLE:
      if (!client.available()) return;
      pcc[0] = client.read();                        // read header
      pcc_size = pcc[0] & 0x0F;
      pcc_index = 0;                                  // message counter
      //no_timeout.parser = 250;      // 250ms - see status.c; aanpassing voor arduino
      parser_state = WF_MESSAGE;
      break;

    case WF_MESSAGE:
      if (pcc_index == pcc_size)
        {
          parser_state = WF_XOR;
          return;
        }
      if (!client.available())
        return; // sds, no timeout handling for now
      /*
      {
        if (no_timeout.parser)  return;
        else
          {
            parser_state = IDLE;
            pc_send_lenz(pars_pcm = pcm_timeout);      // throw exception, if timeout reached
            return;
          }
      }
      */
      pcc_index++;
      pcc[pcc_index] = client.read();
      break;

    case WF_XOR:
      if (!client.available())
        return;
      /*
      {
        if (no_timeout.parser)  return;
        else
          {
            parser_state = IDLE;
            pc_send_lenz(pars_pcm = pcm_timeout);        // throw exception, if timeout reached
            return;
          }
      }
      */
      my_check = 0;  
      for (int i=0; i<=pcc_size; i++) my_check ^= pcc[i];   
      if (my_check != client.read())
        {
          // XOR is wrong!
          pc_send_lenz(pars_pcm = pcm_datenfehler);
          parser_state = IDLE;
          return;
        }
      
      parse_command();       // analyze received message and send code

      #if (XPRESSNET_ENABLED == 1)
          parser_state = CHECK_STOLEN_LOK;
      #else
          parser_state = IDLE;
      #endif
      break;
#if (XPRESSNET_ENABLED == 1)
    case CHECK_STOLEN_LOK:
      if (organizer_state.lok_stolen_by_handheld) 
        {
          send_stolen_loks();                            // report any lok taken over by an handheld            
        }
      parser_state = CHECK_MANUAL_TURNOUT;
      break;
    case CHECK_MANUAL_TURNOUT:
      if (organizer_state.turnout_by_handheld) 
        {
          send_manual_turnouts();                            // report any lok taken over by an handheld            
        }
      parser_state = IDLE;
      break;
#else
    case CHECK_STOLEN_LOK:
    case CHECK_MANUAL_TURNOUT:
      parser_state = IDLE;
      break;
#endif
  }
} // server_loop


void loop() {
  bool retval;
  
  //wifi part of the loop
  server_loop(); // test

} // End of loop
