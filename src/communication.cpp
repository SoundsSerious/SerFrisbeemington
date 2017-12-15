#include "globals.h"

WiFiServer server(BEEMO_PORT);



void handleWiFiEvent(WiFiEvent_t event){
    switch(event) {
        case SYSTEM_EVENT_AP_START:
            frisbeem.com.log("AP Started",true);
            WiFi.softAPsetHostname(SSID_AP);
            break;
        case SYSTEM_EVENT_AP_STOP:
            frisbeem.com.log("AP Stopped",true);
            break;
        case SYSTEM_EVENT_STA_START:
            frisbeem.com.log("STA Started",true);
            WiFi.setHostname(SSID_AP);
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            frisbeem.com.log("STA Connected",true);
            WiFi.enableIpV6();
            break;
        case SYSTEM_EVENT_AP_STA_GOT_IP6:
            frisbeem.com.log("STA IPv6: "+WiFi.localIPv6().toString(),true);
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            frisbeem.com.log("STA IPv4: "+WiFi.localIP().toString(),true);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            frisbeem.com.log("STA Disconnected",true);
            break;
        case SYSTEM_EVENT_STA_STOP:
            frisbeem.com.log("STA Stopped",true);
            break;
        default:
            break;
    }
}

void COM::initialize(){
  Serial.begin( 115200 ); //Open Serial...Mmm breakfast
  // delay(300);

  // log("Initlaize:");
  // log(WiFi.localIP());
  // log(WiFi.subnetMask());
  // log(WiFi.gatewayIP());
  // log(WiFi.SSID());

//   Turned Off For Manual
    initialize_server();
  // initialize_mdns();
}

void COM::initialize_server(){
  setup_wifi();
  setup_ble_beacon();
  server.begin();
}

void COM::setup_wifi(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID_AP);
  WiFi.onEvent(handleWiFiEvent);
  scanWifiNetworks();
  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  // mac = WiFi.softAPmacAddress();
  // String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
  //                String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  // macID.toUpperCase();
  // String AP_NameString = ssid + macID;
  //
  // char AP_NameChar[AP_NameString.length() + 1];
  // memset(AP_NameChar, 0, AP_NameString.length() + 1);
  //
  // for (int i=0; i<AP_NameString.length(); i++)
  //   AP_NameChar[i] = AP_NameString.charAt(i);

  //WiFi.softAP(AP_NameChar, password);

}

void COM::setup_ble_beacon(){
  ble.begin(SSID_AP);
  // BLEDevice::init("Frisbeem");
  //
  // // Create the BLE Server
  // BLEServer *pServer = BLEDevice::createServer();
  // pServer->setCallbacks(new MyServerCallbacks());
  //
  //
  // BLEService *pService = pServer->createService(SERVICE_UUID);
  // pCharacteristic = pService->createCharacteristic(
  //                     CHARACTERISTIC_UUID,
  //                     BLECharacteristic::PROPERTY_READ   |
  //                     BLECharacteristic::PROPERTY_WRITE  |
  //                     BLECharacteristic::PROPERTY_INDICATE|
  //                     BLECharacteristic::PROPERTY_NOTIFY
  //                   );
  //
  // pCharacteristic->addDescriptor(new BLE2902());
  // pService->start();
  //
  // pCharacteristic->setValue(&cc, 1);
}

void COM::update_ble_beacon(){
  // pCharacteristic->setValue(&light, 1);
  // pCharacteristic -> notify();
}

void COM::flush()
{
    Serial.flush();
}

void COM::scanWifiNetworks(){
  log("scan start "+String(micros()));

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  log("scan done");
  if (n == 0) {
      log("no networks found");
  } else {
      log(String(n)+" networks found");
      for (int i = 0; i < 10; ++i) {
          // Print SSID and RSSI for each network found
          log( "SSID:\t" + String(WiFi.SSID(i)));
          log( "RSSI:\t" + String(WiFi.RSSI(i)));
          delay(10);
      }
  }
  log("scan end "+String(micros()));
}

// void COM::initialize_mdns(){
//   log("Initlaizing MDNS");
//   subServices.push_back("printer");
//   log("Setting MDNS Host Name");
//   mdns_success = mdns.setHostname( hostname );
//
//   if (mdns_success) {
//     log("Host Name Set Successfully");
//     mdns_success = mdns.addService("tcp", "beem", BEEMO_PORT, "frisbeem", subServices);
//   }
//
//   mdns.addTXTEntry("frsibeem");
//
//   if (mdns_success) {
//     log("Starting MDNS");
//     mdns_success = mdns.begin();
//     log("MDNS Started");
//   }
// }

void COM::tick(){
  _tick += 1;
  log("tick...");
  if (_tick == tickCount){
    writeNow = true;
  }
  else{
    writeNow = false;
  }
  if (_tick > tickCount){
    _tick = 0;
  }
}

void COM::update(){
  log("Sending MDNS Information");
  //mdns.processQueries();
  read();
  //scanWifiNetworks();
  //send_telemetry();
}

void COM::open(){
  // Check if a client has connected
  initial_connection = false;
  log("Checking For Client");
  client = server.available();
  if (!client) {
    log("No Client Found");
    return;
  }
  else{
    log("Client Found",true);
    initial_connection = true;
  }
}

void COM::close(){
  if (initial_connection){
    client.stop();
    initial_connection = false;
  }
}

String COM::read(){
   String currentLine = "";
   if (initial_connection) {                             // if you get a client,
     log("New Client",true);           // print a message out the serial port
                   // make a String to hold incoming data from the client
    bool firstTime = true;
    wait_count = 0;
     while (client.connected()) {            // loop while the client's connected

      if (firstTime) {log("Client Connected",true);}

       if (client.available()) {             // if there's bytes to read from the client,
         wait_count = 0;
         char c = client.read();             // read a byte, then
         log("Client Read: "+String(c),true);
       if (c == '\n') {
         if (currentLine.length() == 0) {
           currentLine = "";
           //break;
         }
         else {    // if you got a newline, then clear currentLine:
           currentLine += c;
           parseStringForMessage(currentLine);
           currentLine = "";
         }
       }
       else if (c != '\r') {  // if you got anything else but a carriage return character,
         currentLine += c;      // add it to the end of the currentLine
       }
     }

     //While loop Post Counters
     else{ wait_count++; }
     if (wait_count > wait_lim){break;} //Exit No Client Data
     firstTime = false;
   }
   if (currentLine.length() > 0) {log( currentLine, true);}
   return currentLine;
  }
  return String();
}

void COM::parseStringForMessage(String inputString){
  //Messages Will End With A \r\n and will be of the format
  //PRIMARY_CMD\tSECONDARY_CMD\tARGUMENT\r\n
  //PRIMARY_CMD and SECONDARY_CMD are each 3 char (for development... int later)

  //Combine The Unparsed String & inputString
  //We will check for an ending, if one exists will parse the arguments
  //If It Doesn't Exist We'll reset unParsedMsg = messageString to work at a
  //Later Iteration
  String messageString = unParsedMsg + inputString;
  log(messageString,true);
  int currentIndex = 0;
  int it = 0;
  //ßlog(messageString,true);
  while( true ){ //This is dangerous to loop all the time
    int endIndex = messageString.indexOf("\n",currentIndex);
    //log("END Index "+String(endIndex),true);
    if (endIndex == -1){ //Line Feed Character Return Not Found
      log("MSG End Not Found");
      unParsedMsg = messageString.substring(currentIndex);
      unParsedMsg.replace("\r","");
      unParsedMsg.replace("\n","");
      break;
    }
    else{
      //Check Primary Keys return "" (empty char if none). Format Message
      String currentMessage = messageString.substring( currentIndex, endIndex);
      currentMessage.replace("\r","");
      currentMessage.replace("\n","");
      String primary_key = currentMessage.substring(0,3);
      String secondary_key = currentMessage.substring(4,7);
      String argument = currentMessage.substring(8);
      //log(String(it)+" From: "+currentMessage, true);
      //log(String(it)+" Got: PK||"+primary_key+"||SK||"+secondary_key+"||ARG||"+argument, true);

      //For Now We'll Handle The Command. Well want to event the sheeeet out of this later
      handleCommand( primary_key, secondary_key, argument);
      currentIndex = endIndex+1;
      it++;
    }
  }//while
  client.println('RECV');
}

void COM::handleCommand(String pk, String sk, String arg)
{
  log(" Got: PK||"+pk+"||SK||"+sk+"||ARG||"+arg, true);
  if (pk == "GAM" & sk == "SEL")
  { unsigned int selectedGame = (unsigned int)(arg.toInt());
    COMEvent * comEvent = new COM_GameSelect(selectedGame);
    frisbeem.event_queue.addEvent( comEvent );
  }
  else{
    COMEvent * comEvent = new COMEvent(pk,sk,arg);
    frisbeem.event_queue.addEvent( comEvent );
  }

}
  // if (pk.equals("PWR")){
  //   if (sk.equals("OFF")){ frisbeem.lights._on = false;}
  //   if (sk.equals("ONN")){ frisbeem.lights._on = true;}
  // }
  // if (pk.equals("TEL")){
  //   if (sk.equals("CAL"))
  //   {
  //     // Calibrate gyro and accelerometers, load biases in bias registers
  //     //frisbeem.mpu.calibrateMPU9250(frisbeem.mpu.gyroBias, frisbeem.mpu.accelBias);
  //     delay(1000);
  //     //frisbeem.mpu.initMPU9250();
  //     //frisbeem.mpu.Axy_lp = 0;
  //     //frisbeem.mpu.Axy_lp = 0;
  //    }
  // }
// }

void COM::log(String message, bool force){
  //Super Debug Mode Will Try Both Serial And WiFi-zle if it's turn
  //We will default to serial always for zeee robust debugging
  if ( writeNow || force){
    Serial.println( "LOG:\t"+message );
    if (initial_connection){
      server.println( "LOG:\t"+message );
    }
  }
  /*#ifdef LOG_DEBUG
    log("debug delay");
    delay(10);
  #endif*/
}

void COM::telemetry(String pck, String message){
  //Send telemetry every opprotunity only on wifi
  if ( initial_connection ){
    server.println( "TEL:\t"+pck+":\t"+ message );

  }
  //Normally Commented... for debug
  if ( writeNow ){
    Serial.println( "TEL:\t"+pck+":\t"+ message );
  }
}

void COM::send_telemetry(){
  send_time();
  send_gyro();
  send_mag();
  send_acl();
  send_acl_rl();
  send_vel();
  send_pos();
}

void COM::send_time(){
  //Send Gyro Values
  telemetry("TME",  String(micros()));
}

void COM::send_mag(){
  //Send Gyro Values
  telemetry("MAG",  String(frisbeem.mpu.M.x)+","+
                    String(frisbeem.mpu.M.y)+","+
                    String(frisbeem.mpu.M.z)+";");
}

void COM::send_gyro(){
  //Send Gyro Values
  telemetry("GYR",  String(frisbeem.mpu.G.x)+","+
                    String(frisbeem.mpu.G.y)+","+
                    String(frisbeem.mpu.G.z)+";");
}
void COM::send_acl(){
  //Send ACL Values
  telemetry("ACL",  String(frisbeem.mpu.A.x)+","+
                    String(frisbeem.mpu.A.y)+","+
                    String(frisbeem.mpu.A.z)+";");
}

void COM::send_acl_rl(){
  //Send ACL Values
  telemetry("ARL",  String(frisbeem.mpu.Awrld.x)+","+
                    String(frisbeem.mpu.Awrld.y)+","+
                    String(frisbeem.mpu.Awrld.z)+";");
}

void COM::send_vel(){
  //Send ACL Values
  telemetry("VEL",  String(frisbeem.mpu.V.x)+","+
                    String(frisbeem.mpu.V.y)+","+
                    String(frisbeem.mpu.V.z)+";");
}

void COM::send_pos(){
  //Send ACL Values
  telemetry("POS",  String(frisbeem.mpu.X.x)+","+
                    String(frisbeem.mpu.X.y)+","+
                    String(frisbeem.mpu.X.z)+";");
}
