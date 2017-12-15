
#ifndef COM_H
#define COM_H

#include <Arduino.h>
//#include "MDNS.h"
#include <WiFi.h>
#include <vector>
#include <SimpleBLE.h>


#define MAX_CLIENTS 4
#define BEEMO_PORT 18330
#define LOG_DEBUG false //5ms log delay

#define WL_MAC_ADDR_LENGTH 6

#define SSID_AP "Frisbeem"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks;
//BLECharacteristic *pCharacteristic;

void handleWiFiEvent(WiFiEvent_t event);

class COM { //, public Subject{
  //In which we send information

public:
  int numberOfClients;

  //Event Timing
  unsigned long old_time;
  int tickCount = 1000;
  int _tick = tickCount - 1; //Tick One Less than tickCount will print first time

  String mac;

  WiFiClient client;
  SimpleBLE ble;
  // BLEServer *pServer;
  // BLEService *pService;

  //Message Parsing
  String lastMsg;
  String unParsedMsg;

  //Boolean Values
  bool mdns_success;
  bool debugMode = true;
  bool writeNow = true; //Tells log to write. True Means First Time will print
  bool initial_connection = false;
  int wait_count = 0;
  int wait_lim = 20;

  //Important Functions
  void log(String message,bool force=false);
  void telemetry(String pck, String message);
  void initialize();
  void setup_wifi();
  void setup_ble_beacon();
  void update_ble_beacon();
  void update();
  void open();
  void close();
  void flush();
  String read();
  void parseStringForMessage(String inputString);
  void handleCommand(String pk, String sk, String arg);

  void scanWifiNetworks();

  void tick();

  //Initialize Sub Funcitons
  //void initialize_mdns();
  void initialize_server();

  //Telemetry Functions
  void send_telemetry();
  void send_time();
  void send_mag();
  void send_acl();
  void send_acl_rl();
  void send_gyro();
  void send_vel();
  void send_pos();

};


#endif //COM_H
