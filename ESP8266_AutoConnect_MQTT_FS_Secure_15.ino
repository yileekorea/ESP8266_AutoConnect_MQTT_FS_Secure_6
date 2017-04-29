#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "yi_WiFiManager.h"          //https://github.com/tzapu/WiFiManager
#include <ESP8266httpUpdate.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <OneWire.h>
#include <SPI.h>
#include "gpio_MCP23S17.h"   // import library

#pragma GCC diagnostic ignored "-Wwrite-strings"
 
#define DBG_SERIAL      Serial
#define DEBUG_PRINT 1


//const uint8_t sclk = 14;
//const uint8_t mosi =13; //Master Output Slave Input ESP8266=Master,MCP23S08=slave 
const uint8_t MCP_CS = 15;
gpio_MCP23S17 mcp(MCP_CS,0x20);//instance

OneWire  ds(2);  // on pin 2 (a 4.7K resistor is necessary)

String clientName;
String subCommand;
String updateServer_fwImage;
String REMOTE_SERVER;
String SKETCH_BIN;

String form =                                             // String form to sent to the client-browser
  "<p>"
  "<center>"
  "<h1>Command to Io2Life! </h1>"
  "<img src='http://iot2ym.iptime.org:8000/jtbc.jpg'>"
  "<form action='msg'><p>Type Command: <input type='text' name='msg' size=50 autofocus> <input type='submit' value='Submit'></form>"
  "</center>";
#define BOOT_AFTER_UPDATE false

//for LED status
#include <Ticker.h>
Ticker ticker;
#define ESP_LED 2
#define R1_BUILTIN 4



//define your default values here, if there are different values in config.json, they are overwritten.
//length should be max size + 1 
/*
const char* fingerprint = "93:E6:74:63:96:C4:B2:B0:B2:BA:F3:7D:12:6D:51:C4:76:E5:D7:0E";
char mqtt_server[40]= "iot2better.iptime.org";
char mqtt_port[6] = "8883";
char mqtt_user_id[16] = "yiorange";
char mqtt_user_pwd[16] = "yiorange";
*/
/*
//iot2ym
const char* fingerprint = "30:06:C1:0B:38:46:A5:01:E9:5E:76:64:51:36:71:FA:20:B3:A7:7D";  //orange-one
char mqtt_server[40]= "iot2ym.iptime.org";
char mqtt_port[6] = "8883";
char mqtt_user_id[16] = "yipine";
char mqtt_user_pwd[16] = "yipine";
*/
//iot2better-macbook
const char* fingerprint = "E6:E0:09:FD:2F:2F:31:85:54:F2:EA:22:14:42:D6:1A:9C:44:36:15";  //mac book
char mqtt_server[40]= "iot2better.iptime.org";
char mqtt_port[6] = "8883";
char mqtt_user_id[16] = "yimac";
char mqtt_user_pwd[16] = "yimac";

//char blynk_token[33] = "8fa7f712af4648f9b7f4add8e3e2b015";
//default custom static IP
char static_ip[16]; // = "192.168.30.200";
char static_gw[16]; // = "192.168.30.1";
char static_sn[16]; // = "255.255.255.0";
char static_dns[16];// = "168.126.63.1";
//char static_dns[16] = "150.236.207.21";

//flag for saving data
bool shouldSaveConfig = false;

char * MQTT_SERVER = mqtt_server;
int MQTT_PORT = 8883;
char * MQTT_USER_ID = mqtt_user_id;
char * MQTT_USER_PWD = mqtt_user_pwd;

//boolean flagWifi = false;
boolean flagMqtt = false;

unsigned long mqttTry = 0;
unsigned long tempTry = 0;

byte numSensor = 0;
String sName[10];
float old_celsius[10]= {26,26,26,26,26,26,26,26,26,26};
float celsius[10];
float rStatus[] = {0,0,0,0,0,0,0,0}; //room status all OFF
float L_Temp[] = {26,26,26,26,26,26,26,26}; 


String MAC;
String topic_sub = "/S_";
String topic_pub = "/P_";
char* do_fw_update = "/do_fw_update";
char* do_reboot = "/do_reboot";
char* do_reset = "/do_reset";
char* do_format = "/do_format";

//char* updateServer = "http://iot2ym.iptime.org:8000/";
char* updateServer = "http://iot2better.iptime.org:8003/fwImage/";
char* fwImage = "io2life.bin";

long lastReconnectAttempt = 0;
long lastMsg = 0;
int test_para = 2000;
unsigned long startMills;

WiFiClientSecure wifiClient;

//PubSubClient * mqttClient;
void mqttCallback(char* topic_sub, byte* payload, unsigned int length);
PubSubClient mqttClient(MQTT_SERVER, MQTT_PORT, mqttCallback, wifiClient);
void sendmqttMsg(char* topictosend, String payload);

//WiFiServer server(80);
ESP8266WebServer server(80);                              // HTTP server will listen at port 80

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

String readFromOneWire();
void sendTempData();
void relayControl();
void verifyFingerprint();

void io2LIFEhttpUpdate(char* updateServer, char* fwImage)
{
  if (WiFi.status() == WL_CONNECTED) {
        updateServer_fwImage = updateServer;
        updateServer_fwImage += fwImage;
		
		REMOTE_SERVER = updateServer;
		SKETCH_BIN = fwImage;

		ESPhttpUpdate.rebootOnUpdate(BOOT_AFTER_UPDATE);
		//ESPhttpUpdate.rebootOnUpdate(true);

        t_httpUpdate_return ret = ESPhttpUpdate.update(updateServer_fwImage);
        //t_httpUpdate_return ret = ESPhttpUpdate.update(REMOTE_SERVER, 443, SKETCH_BIN);

        switch(ret) {
            case HTTP_UPDATE_FAILED:
                DBG_SERIAL.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                break;

            case HTTP_UPDATE_NO_UPDATES:
                DBG_SERIAL.println("HTTP_UPDATE_NO_UPDATES");
                break;

            case HTTP_UPDATE_OK:
                DBG_SERIAL.println("HTTP_UPDATE_OK");
				do_reboot_exe();
                break;
        }
    }
}
/*
 * subscribe
 */

void mqttCallback(char* topic_sub, byte* payload, unsigned int length)
{
	int i = 0;
	int nSensor; //main captured String 
    char buffer[80];
    int len = length >= 79 ? 79 : length;
    memcpy(buffer, payload, len);
    buffer[length] = 0;
	
    DBG_SERIAL.print(">> Topic: ");
    DBG_SERIAL.print(topic_sub);

    DBG_SERIAL.print(">> Payload: ");
    DBG_SERIAL.println(buffer);
	
	char * pch=0;
	//printf ("Looking for the ':' chars in \"%s\"...\n",buffer);
	pch=strchr(buffer,':');
	if(pch){
		nSensor = atoi(buffer);	// number of Sensors
		printf ("number of sensor %d\n",nSensor);
	  
		while (pch!=NULL)
		{
			L_Temp[i] = atof(pch+1);
			DBG_SERIAL.println(L_Temp[i]);
			//printf ("found at %d\n",pch-buffer+1);
			pch=strchr(pch+1,':');
			old_celsius[i] += 0.5;
			++i;
		}
	}
	else {
		subCommand = String(buffer);
		if (subCommand == do_fw_update) {
			DBG_SERIAL.print(">> do_fw_update_exe ");
			io2LIFEhttpUpdate(updateServer, fwImage);
		}
		else if (subCommand == do_reboot) {
			DBG_SERIAL.print(">> do_reboot_exe ");
			do_reboot_exe();
		}
	}
}

boolean reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
/*
  Serial.print("Attempting MQTT MQTT_USER_ID...");
  Serial.print(MQTT_USER_ID);
  Serial.print(MQTT_USER_PWD);
  Serial.print(clientName.c_str());
  Serial.print("Attempting MQTT MQTT_USER_PWD...");
*/
    // Attempt to connect
    //if (client.connect(clientId.c_str())) {
    if (mqttClient.connect(clientName.c_str(), MQTT_USER_ID, MQTT_USER_PWD)) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        if (mqttClient.publish((char *)topic_pub.c_str(), "hello world again...")) {
            DBG_SERIAL.println("publish ok2");
        }
        else {
            DBG_SERIAL.println("publish failed2");
        }
        // ... and resubscribe
    //topic_sub = "/" + clientName;
        if (mqttClient.subscribe((char *)topic_sub.c_str())) {
            DBG_SERIAL.println("Subscribe ok2");
        }
        else {
            DBG_SERIAL.println("Subscribe failed2");
        }
        
    } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
    }
  server.handleClient();	// checks for incoming messages
  }
    return mqttClient.connected();
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}
void macToTopic()
{
  String result;
  result = WiFi.macAddress();
  for (int i = 0; i < 16; ) {
  MAC += result.substring(i, i+2);
  i +=3;
  }
  topic_pub += MAC;
  topic_sub += MAC;
}

void tick()
{
  //toggle state
  int state = digitalRead(ESP_LED);  // get the current state of GPIO1 pin
  digitalWrite(ESP_LED, !state);     // set pin to the opposite state
}

//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(1.5, tick);
}

void handle_msg() 
{
  server.send(200, "text/html", form);                    // Send same page so they can send another msg

  String msg = server.arg("msg");
  Serial.println(msg);
  Serial.println(' ');                                  // new line in monitor

  if (msg == do_fw_update) {
      io2LIFEhttpUpdate(updateServer, fwImage);
    }
  else if (msg == do_reboot) {
    do_reboot_exe();
    }
  else if (msg == do_reset) {
    Serial.println("WILL reset ESP system soon!!!!");
    //WiFiManager
    WiFiManager wifiManager;
    //reset settings - for testing
    Serial.println("reset settings----------");
    wifiManager.resetSettings();
  do_reboot_exe();
    }
  else if (msg == do_format) {
  digitalWrite(ESP_LED, LOW);
  Serial.print("SPIFFS.format() ...");

  SPIFFS.format();
  do_reboot_exe();
  }
}
void do_reboot_exe() 
{
    Serial.println("WILL reboot ESP system soon!!!!");
  pinMode(0, OUTPUT);
  digitalWrite(0, HIGH);
    delay(5000);
    ESP.reset();
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();
  
  //flagWifi = false;
  flagMqtt = false;

  //set led pin as output
  pinMode(ESP_LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.2, tick);

  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");


  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user_id, json["mqtt_user_id"]);
          strcpy(mqtt_user_pwd, json["mqtt_user_pwd"]);
//          strcpy(blynk_token, json["blynk_token"]);

          if(json["ip"]) {
            Serial.println("setting custom ip from config");
            //static_ip = json["ip"];
            //static_gw = json["gateway"];
            //static_sn = json["subnet"];
            strcpy(static_ip, json["ip"]);
            strcpy(static_gw, json["gateway"]);
            strcpy(static_sn, json["subnet"]);
            //strcat(static_ip, json["ip"]);
            //Serial.println(static_ip);
/*            Serial.println("converting ip");
            IPAddress ip = ipFromCharArray(static_ip);
            Serial.println(ip);*/
          } else {
            Serial.println("no custom ip in config");
          }
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  ticker.attach(0.5, tick);

  Serial.println(static_ip);
  Serial.println(mqtt_server);
  Serial.println(mqtt_port);
  Serial.println(mqtt_user_id);
  Serial.println(mqtt_user_pwd);
  
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user_id("user_id", "mqtt user id", mqtt_user_id, 16);
  WiFiManagerParameter custom_mqtt_user_pwd("user_pwd", "mqtt user pwd", mqtt_user_pwd, 16);
  //WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 34);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //Serial.println("reset settings----------");
  //wifiManager.resetSettings();

  //set config save notify callback
  Serial.println("saveConfigCallback----------");
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  /*
  if (static_ip) {
      //set static ip
      Serial.println("set Static IP address");
      IPAddress _ip,_gw,_sn,_dns;
      _ip.fromString(static_ip);
      _gw.fromString(static_gw);
      _sn.fromString(static_sn);
      _dns.fromString(static_dns);
    
      //wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
      //wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn, _dns);
  }
*/

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user_id);
  wifiManager.addParameter(&custom_mqtt_user_pwd);
  //wifiManager.addParameter(&custom_blynk_token);

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  wifiManager.setMinimumSignalQuality();
  
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(120);

  //--------------------------------------------------------------------------------------------------
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  //--------------------------------------------------------------------------------------------------
 
  
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  Serial.println("autoConnect----------");
  //if (!wifiManager.autoConnect()) {
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey");
  //flagWifi = true;

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user_id, custom_mqtt_user_id.getValue());
  strcpy(mqtt_user_pwd, custom_mqtt_user_pwd.getValue());
//  strcpy(blynk_token, custom_blynk_token.getValue());

  MQTT_SERVER = mqtt_server;
  String inString = "";
  inString += mqtt_port;
  MQTT_PORT = inString.toInt();

  MQTT_USER_ID = mqtt_user_id;
  MQTT_USER_PWD = mqtt_user_pwd;

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user_id"] = mqtt_user_id;
    json["mqtt_user_pwd"] = mqtt_user_pwd;
//    json["blynk_token"] = blynk_token;

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  ticker.detach();
  //keep LED on
  digitalWrite(ESP_LED, LOW);

  // Generate client name based on MAC address and last 8 bits of microsecond counter
  // String clientName;
  clientName += "Io2L-";
  //uint8_t mac[6];
  //WiFi.macAddress(mac);
  clientName += WiFi.macAddress();
  //clientName += macToStr(mac);
  //clientName += "-";
  //clientName += String(micros() & 0xff, 16);
/*
  topic_sub += topic_pub.substring(0, 2);
  topic_sub += topic_pub.substring(3, 5);
  topic_sub += topic_pub.substring(6, 8);
  topic_sub += topic_pub.substring(9, 11);
  topic_sub += topic_pub.substring(12, 14);
  topic_sub += topic_pub.substring(15, 17);
 */
  macToTopic();
  Serial.println(topic_pub);
  Serial.println(topic_sub);

  // check the fingerprint of io.adafruit.com's SSL cert
  verifyFingerprint();
  delay(3000); 

  // Set up mDNS responder:
  // - first argument is the domain name, in this example
  //   the fully-qualified domain name is "esp8266.local"
  // - second argument is the IP address to advertise
  //   we send our IP address on the WiFi network
  if (!MDNS.begin("io2life")) {
    Serial.println("Error setting up MDNS responder!");
    while(1) { 
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  
  // Start TCP (HTTP) server
  server.begin();
  Serial.println("TCP server started");
  
  // Add service to MDNS-SD
  MDNS.addService("http", "tcp", 80);

  
  // Set up the endpoints for HTTP server,  Endpoints can be written as inline functions:
  server.on("/", []()
  {
    server.send(200, "text/html", form);
  });

  server.on("/msg", handle_msg);          // And as regular external functions:
  server.begin();

  Serial.print("Attempting SPI mcp.begin()...");
  Serial.println();
  mcp.begin(0);//x.begin(1) will override automatic SPI initialization

  pinMode(R1_BUILTIN, OUTPUT);
  mcp.gpioPinMode(0x0F);
  
  digitalWrite(R1_BUILTIN, HIGH);
  mcp.gpioPort(0xFF);
  delay(1000);
  mcp.gpioPort(0x00);
  digitalWrite(R1_BUILTIN, LOW);

}

void loop() {
  numSensor = 0;
  if (WiFi.status() == WL_CONNECTED) {
    if ( flagMqtt == false ) {
        if ( mqttTry == 0 || millis() - mqttTry >= 50000UL ) // 5sec
        {
            if ( mqttClient.connect(clientName.c_str(), MQTT_USER_ID, MQTT_USER_PWD) )
            {
                flagMqtt = true;
                DBG_SERIAL.println("Connected to MQTT broker");

                // Publish
				if (mqttClient.publish((char *)topic_pub.c_str(), "hello world...")) {
					DBG_SERIAL.println("publish ok");
				}
				else {
					DBG_SERIAL.println("publish failed");
				}

                // Subscribe
                if (mqttClient.subscribe((char *)topic_sub.c_str())) {
                    DBG_SERIAL.println("Subscribe ok");
                }
                else {
                    DBG_SERIAL.println("Subscribe failed");
                }
            }
            else {
                DBG_SERIAL.println("MQTT connect failed");
            }
        }
    server.handleClient();  // checks for incoming messages
    }
    else {
      if (!mqttClient.connected()) {
          reconnect();
      }
      mqttClient.loop();
    }
	
	if ( flagMqtt == true && (tempTry == 0 || ((millis() - tempTry) > 9000UL)))  // 4sec
	{
		String pl = readFromOneWire();
		sendTempData();
		relayControl();
		tempTry = millis();
	}

	server.handleClient();		// checks for incoming messages

  }

	/*
	digitalWrite(R1_BUILTIN, LOW);
    delay(2500);  
  for (int i=4;i<8;i++){
    mcp.gpioDigitalWrite(i,LOW);
	int pin_read = mcp.gpioDigitalRead(i);
	uint16_t ReadAddress = mcp.gpioReadAddress(i);
	Serial.print(i);
	Serial.print("-pin_read:");
    Serial.print(pin_read);
	Serial.println();
	Serial.print(i);
	Serial.print("-ReadAddress:");    
	Serial.print(ReadAddress);
	Serial.println();
	Serial.println();
    delay(2500);
  }
  
	digitalWrite(R1_BUILTIN, HIGH);
    delay(2500);
  for (int i=4;i<8;i++){
    mcp.gpioDigitalWrite(i,HIGH);
	int pin_read = mcp.gpioDigitalRead(i);
	uint16_t ReadAddress = mcp.gpioReadAddress(i);
	Serial.print(i);
	Serial.print("-pin_read:");
	Serial.print(pin_read);
	Serial.println();
	Serial.print(i);
	Serial.print("-ReadAddress:");    
	Serial.print(ReadAddress);
	Serial.println();
	Serial.println();
    delay(2500);
  }
*/

}


/*
 * sendmqttMsg sending
 */
void sendmqttMsg(char* topictosend, String payload)
{

  if (mqttClient.connected()) {
    if (DEBUG_PRINT) {
      Serial.print("Sending payload: ");
      Serial.print(payload);
    }

    unsigned int msg_length = payload.length();

    if (DEBUG_PRINT) {
      Serial.print(" length: ");
      Serial.println(msg_length);
    }

    byte* p = (byte*)malloc(msg_length);
    memcpy(p, (char*) payload.c_str(), msg_length);

    if ( mqttClient.publish(topictosend, p, msg_length)) {
      if (DEBUG_PRINT) {
        Serial.println("Publish length TEMP ok");
      }
      free(p);
      //return 1;
    } else {
      if (DEBUG_PRINT) {
        Serial.println("Publish length TEMP failed");
      }
      free(p);
      //return 0;
    }
  }
}

/*
 * Temperature measurement
 */
String readFromOneWire()
{
  String payload;

    //byte numSensor = 0;
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];

    
    while (ds.search(addr)) {
    ////    measure ();
    //Serial.print("ROM =");
    for ( i = 0; i < 8; i++) {
        //Serial.write(' ');
        //Serial.print(addr[i], HEX);
		sName[numSensor] += String(addr[i], HEX);
    }
    //Serial.print(sName[numSensor]);
      
    if (OneWire::crc8(addr, 7) != addr[7]) {
        Serial.println("CRC is not valid!");
        return payload;
    }
    Serial.println();
    
    // the first ROM byte indicates which chip
    switch (addr[0]) {
          case 0x10:
          Serial.println("  Chip = DS18S20");  // or old DS1820
          type_s = 1;
          break;
    case 0x28:
          Serial.println("  Chip = DS18B20");
          type_s = 0;
          break;
    case 0x22:
          Serial.println("  Chip = DS1822");
          type_s = 0;
          break;
    default:
          Serial.println("Device is not a DS18x20 family device.");
          return payload;
    }
        
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1);  // start conversion, with parasite power on at the end
    
    //delay(1000);     // maybe 750ms is enough, maybe not
    delay(200);     // maybe 187.5ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.
    
    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);     // Read Scratchpad
    /*
    Serial.print("  Data = ");
    Serial.print(present, HEX);
    Serial.print(" ");
    */
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
        //Serial.print(data[i], HEX);
        //Serial.print(" ");
    }
    /*
    Serial.print(" CRC=");
    Serial.print(OneWire::crc8(data, 8), HEX);
    Serial.println();
    */
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
			// "count remain" gives full 12 bit resolution
			raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        // default is 12 bit resolution, 750 ms conversion time
		// but set as 10bits
		raw = raw & ~3; // 10 bit res, 187.5 ms
    }
    celsius[numSensor] = (float)raw / 16.0;
	//////////////////////////////////////////////////////////////////////////
	if(L_Temp[numSensor]){
		Serial.print("  L_Temp[] ====> ");
		Serial.print(L_Temp[numSensor]);
	}	

	if(L_Temp[numSensor] <= celsius[numSensor]){
		rStatus[numSensor] = 0;
		
	}else{
		rStatus[numSensor] = L_Temp[numSensor];
		if(L_Temp[numSensor]){
			Serial.print("  rStatus[] -----> ");
			Serial.print(rStatus[numSensor]);
		}
	}
	//////////////////////////////////////////////////////////////////////////
	
    //fahrenheit[numSensor] = celsius[numSensor] * 1.8 + 32.0;
    //Serial.print("  ID = ");
    //Serial.print(id[numSensor]);
    Serial.print("  Temps = ");
    Serial.print(celsius[numSensor]);
    Serial.print(" Celsius, ");
    numSensor += 1;

  }
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(1000);

    return payload;
}

void relayControl() {
    byte i;
	if(L_Temp[0] <= celsius[0]){
		digitalWrite(R1_BUILTIN,LOW);
	} else {
		digitalWrite(R1_BUILTIN,HIGH);
	}
	for ( i = 1; i < (numSensor-1) ; i++) {
		if(L_Temp[i] <= celsius[i]){
			mcp.gpioDigitalWrite(i+3,LOW);
		} else {
			mcp.gpioDigitalWrite(i+3,HIGH);		
		}
	}
}

void sendTempData() {
    byte i;
    for ( i = 0; i < numSensor ; i++) {
		if(old_celsius[i] != celsius[i]){

			char pChrBuffer[5];
			String payload = "{\"tbl_name\":";
			//payload += "\"tRoomTemp_";
			payload += "\"";
			payload += MAC;
			payload += "\"";

			payload += ",\"id\":";
			payload += i+1;   // id
			
			payload += ",\"numSensor\":";
			payload += numSensor;   // numSensor
			
			payload += ",\"cTemps\":";
			if ( isnan(celsius[i]) )
				payload += "0";
			else {
				dtostrf(celsius[i] , 3, 1, pChrBuffer);
				payload += pChrBuffer;   // *C
			}

				//"{\"temp_1\":";
				payload += ",\"sName\":";
				payload += "\"";
				payload += sName[i];   // sensor name
				payload += "\"";

				payload += ",\"cStatus\":";
				//payload += "\"";
				payload += rStatus[i];   // room status
				//payload += "\"";
				payload += "}";

				//payload += "\":";
//		if(old_celsius[i] != celsius[i]){
			sendmqttMsg((char *)topic_pub.c_str(), (char *)payload.c_str());
		}
		sName[i] = "";
		old_celsius[i] = celsius[i];
	}
}

void verifyFingerprint() {
  //const char* host = MQTT_SERVER;

  Serial.print("Connecting to ");
  Serial.println(MQTT_SERVER);

  WiFiManager wifiManager;

  if (! wifiClient.connect(MQTT_SERVER, MQTT_PORT)) {
  Serial.println(MQTT_SERVER);
  Serial.println(MQTT_PORT);

  Serial.println("MQTT Connection failed. Halting execution.");
  wifiManager.resetSettings();
  wifiManager.startConfigPortal("AutoConnectAP","password");
  
    //while(1);
  }

  if (wifiClient.verify(fingerprint, MQTT_SERVER)) {
    Serial.println("Connection secure.");
  } else {
  Serial.println("verify Connection insecure! Halting execution.");
  wifiManager.resetSettings();
  wifiManager.startConfigPortal("AutoConnectAP","password");

    //while(1);
  }
}
