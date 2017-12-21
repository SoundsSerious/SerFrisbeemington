#include "globals.h"

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

void onRequest(AsyncWebServerRequest *request){
  //Handle Unknown Request
  frisbeem.com.log("Unknown Page: 404",true);
  request->send(404);
}

void onBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
  //Handle body
  frisbeem.com.log("Handle Body Message",true);
}

void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
  //Handle upload
  frisbeem.com.log("Upload",true);
}

void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  //Handle WebSocket event
  if(type == WS_EVT_CONNECT){
    //client connected
    //os_printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    frisbeem.com.log( "ws connect C:" +String(client->id()),true);
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    //client disconnected
    frisbeem.com.log( "ws disconnect S: "+ String(server->url()) +"C:" +String(client->id()),true) ;
  } else if(type == WS_EVT_ERROR){
    //error was received from the other end
    frisbeem.com.log( "ws error S: "+ String(server->url()) +"C:" +String(client->id()),true) ;
  } else if(type == WS_EVT_PONG){
    //pong message was received (in response to a ping request maybe)
    frisbeem.com.log( "ws pong S: "+ String(server->url()) +"C:" +String(client->id()),true) ;
  } else if(type == WS_EVT_DATA){
    //data packet
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      //os_printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);
      if(info->opcode == WS_TEXT){
        data[len] = 0;
        frisbeem.com.log( "msg S: "+ String(server->url()) +"C:" +String(client->id()) + String((char*)data),true) ;
        //os_printf("%s\n", (char*)data);
      } else {
        for(size_t i=0; i < info->len; i++){
          //os_printf("%02x ", data[i]);
        }
        //os_printf("\n");
      }
      if(info->opcode == WS_TEXT)
        client->text("I got your text message");
      else
        client->binary("I got your binary message");
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0){
        if(info->num == 0){}//os_printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        //os_printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }
      //os_printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);
      if(info->message_opcode == WS_TEXT){
        data[len] = 0;
        //os_printf("%s\n", (char*)data);
        frisbeem.com.log( "msg S: "+ String(server->url()) +"C:" +String(client->id()) + String((char*)data),true) ;
      } else {
        for(size_t i=0; i < len; i++){
          //os_printf("%02x ", data[i]);
        }
        //os_printf("\n");
      }

      if((info->index + len) == info->len){
        //os_printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if(info->final){
          //os_printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
          if(info->message_opcode == WS_TEXT)
            client->text("I got your text message");
          else
            client->binary("I got your binary message");
        }
      }
    }
  }
}


void COM::initialize(){
    if (LOG_DEBUG){Serial.begin( 115200 );} //Open Serial...Mmm breakfast
    initialize_server();
    setup_ble_beacon();
}

void COM::initialize_server(){
  setup_wifi();

  // attach AsyncWebSocket
  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // attach AsyncEventSource
  server.addHandler(&events);

  // respond to GET requests on URL /heap
  server.on("/heap", HTTP_GET, [this](AsyncWebServerRequest *request){
    this -> log("GET HEAP",true);
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  // upload a file to /upload
  server.on("/upload", HTTP_POST, [this](AsyncWebServerRequest *request){
    this -> log("POST UPLOAD",true);
    request->send(200);
  }, onUpload);

  // send a file when /index is requested
  server.on("/index", HTTP_ANY, [this](AsyncWebServerRequest *request){
    //request->send(SPIFFS, "/index.htm");
    this -> log("INDEX",true);
    request->send(200, "/index.htm");
  });

  // send a file when /index is requested
  server.on("/tel/activate", HTTP_ANY, [this](AsyncWebServerRequest *request){
    //request->send(SPIFFS, "/index.htm");
    this -> log("TEL_ACTIVE",true);
    startTelemetryServer();
    request->send(200);
  });

  // send a file when /index is requested
  server.on("/tel/deactivate", HTTP_ANY, [this](AsyncWebServerRequest *request){
    //request->send(SPIFFS, "/index.htm");
    this -> log("TEL_DEACTIVATE",true);
    stopTelemetryServer();
    request->send(200);
  });

  // HTTP basic authentication
  // server.on("/login", HTTP_GET, [](AsyncWebServerRequest *request){
  //   this -> log("GET LOGIN",true);
  //   if(!request->authenticate(http_username, http_password))
  //       return request->requestAuthentication();
  //   request->send(200, "text/plain", "Login Success!");
  // });

  server.onNotFound(onRequest);
  server.onFileUpload(onUpload);
  server.onRequestBody(onBody);

  server.begin();
  if (LOG_DEBUG){ startTelemetryServer();}
}

void COM::startTelemetryServer()
{
  telemetry_activated = true;
  telemetry_server.begin();
}

void COM::stopTelemetryServer()
{
  telemetry_activated = false;
  telemetry_server.stop();
}

void COM::setup_wifi(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID_AP);
  WiFi.onEvent(handleWiFiEvent);
  scanWifiNetworks();
}

void COM::setup_ble_beacon(){
  ble.begin(SSID_AP);
}

void COM::update_ble_beacon(){
  // pCharacteristic->setValue(&light, 1);
  // pCharacteristic -> notify();
}

void COM::flush()
{
    if (LOG_DEBUG){Serial.flush();}
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

void COM::open(){
  //Determine Number Of Clients
  clientCount = 0; // Needs To Be Zero For Loop
  for (int i=lastActive; i<MAX_CLIENTS+lastActive; i++){
    _loopClient = ws.hasClient(i);
    activeClients[i-1] = _loopClient; //Diffferent Indexing For Array
    if (_loopClient){clientCount++;}
    if(!clientsActive && _loopClient) {
        firstActive = i;
        lastActive =  i;
    }
    if(clientCount > 0){clientsActive = true;}
  }
  log("FirstActive: "+String(firstActive));
  log("LastActive: "+String(lastActive));
  log("Active WSClients: "+String(ws.count()));
  log("Active Clients: "+String(clientCount));
}

void COM::close(){
  clientsActive = false;
  firstActive = 0;
}

void COM::tick(){
  _tick += 1;
  _telk += 1;
  log("TICK: --> "+String( micros()-startLoop));
  log("TOCK: --> "+String( frisbeem.mpu.cycle_count ));
  log("CPS: --> "+String( frisbeem.mpu.cycle_count /(micros()/1E6) ));
  if (_tick == tickCount){
    writeNow = true;
  }
  else{
    writeNow = false;
  }
  if (_tick > tickCount){
    _tick = 0;
  }
  //Telemtery Counting
  if (_telk == telCount){
    telNow = true;
  }
  else{
    telNow = false;
  }
  if (_telk > telCount){
    _telk = 0;
  }
}

void COM::start_cycle()
{
  startLoop = micros();
}

void COM::update(){
  log("Update COM");
  serveTelemetry();
}

void COM::broadcastClients(String msg)
{
  ws.textAll(msg);
}

void COM::broadcastPrimary(String msg)
{ //Get This Guy Going At A Clip
  //Assume We Have An Active First Client - ID Starts at 1 baby
  ws.text(firstActive,msg);
  //ws.binary(firstActive,(char*) msg.c_str());
  //AsyncWebSocketClient *c = ws.client(lastActive);
  //AsyncClient *cl = c -> client();
  //cl -> write( (char*) msg.c_str() );

}


void COM::log(String message, bool force){
  //Super Debug Mode Will Try Both Serial And WiFi-zle if it's turn
  //We will default to serial always for zeee robust debugging
  if ( writeNow || force){
    if (LOG_DEBUG){Serial.println( "LOG:\t"+message  );}
    broadcastPrimary( "LOG:\t"+message );
  }
}

////////////////////////////////////////////////////////////////////////////////
//Telemetry

void COM::serveTelemetry()
{
  if (telemetry_activated || LOG_DEBUG)
  {
    WiFiClient telemetry_client = telemetry_server.available();   // listen for incoming clients

    if (telemetry_client || LOG_DEBUG) {                             // if you get a client,
      log("Telemetry Client");          // print a message out the serial port
      String currentLine = "";                // make a String to hold incoming data from the client
        if (telemetry_client.connected() || LOG_DEBUG)
        {            // loop while the client's connected
          //send_telemetry();
          str_send_telemetry();
        }
      // Clean it up
      telemetry_client.stop();
    }
  }
}

void COM::directTelemetry(TEL_TYPES type, unsigned long time)
{
  char *btime;
  btime = (char *) &time;
  //Send Dis Ish
  telemetry_client.print((char)type);
  telemetry_client.println(btime);
}

void COM::directTelemetry(TEL_TYPES type, float x, float y, float z)
{
  char *X,*Y,*Z;
  X = (char *) &x;
  Y = (char *) &y;
  Z = (char *) &z;
  //Send Dis Ish
  telemetry_client.print((char)type);
  telemetry_client.print(X);
  telemetry_client.print(Y);
  telemetry_client.println(Z);
}

void COM::send_telemetry(){
  send_time();
  send_gyro();
  send_mag();
  send_acl();
  send_acl_rl();
  send_acl_wlrd();
  send_vel();
  send_pos();
}

void COM::send_time(){
  //Send Gyro Values
  directTelemetry(TME, micros());
}

void COM::send_mag(){
  //Send Gyro Values
  directTelemetry(MAG,frisbeem.mpu.M.x,
                      frisbeem.mpu.M.y,
                      frisbeem.mpu.M.z);
}

void COM::send_gyro(){
  //Send Gyro Values
  directTelemetry(GYRO, frisbeem.mpu.G.x,
                        frisbeem.mpu.G.y,
                        frisbeem.mpu.G.z);
}
void COM::send_acl(){
  //Send ACL Values
  directTelemetry(ALIN, frisbeem.mpu.A.x,
                        frisbeem.mpu.A.y,
                        frisbeem.mpu.A.z);
}

void COM::send_acl_rl(){
  //Send ACL Values
  directTelemetry(AWORLD, frisbeem.physics.Alin.x,
                          frisbeem.physics.Alin.y,
                          frisbeem.physics.Alin.z);

}

void COM::send_acl_wlrd(){
  //Send ACL Values
  directTelemetry(AWORLD, frisbeem.physics.Awrld.x,
                          frisbeem.physics.Awrld.y,
                          frisbeem.physics.Awrld.z);

}

void COM::send_vel(){
  //Send ACL Values
  directTelemetry(VEL,frisbeem.physics.V.x,
                      frisbeem.physics.V.y,
                      frisbeem.physics.V.z);
}

void COM::send_pos(){
  //Send ACL Values
  directTelemetry(POS,frisbeem.physics.X.x,
                      frisbeem.physics.X.y,
                      frisbeem.physics.X.z);
}


void COM::str_send_telemetry(){
  str_send_time();
  str_send_gyro();
  str_send_mag();
  str_send_acl();
  str_send_acl_rl();
  str_send_vel();
  str_send_pos();
}

void COM::telemetry(String pck, String message){
  //Send telemetry every opprotunity only on wifi
  //Normally Commented... for debug
  if ( telNow ){
    if (LOG_DEBUG){
      Serial.println( "TEL:\t"+pck+":\t"+ message );}
    if (telNow && telemetry_client) {
      telemetry_client.println( "TEL:\t"+pck+":\t"+ message );}
  }

}

void COM::str_send_time(){
  //Send Gyro Values
  telemetry("TME",  String(micros()));
}

void COM::str_send_gyro(){
  //Send Gyro Values
  telemetry("GYR",  String(frisbeem.mpu.G.x)+","+
                    String(frisbeem.mpu.G.y)+","+
                    String(frisbeem.mpu.G.z)+";");
}

void COM::str_send_mag(){
  //Send Gyro Values
  telemetry("MAG",  String(frisbeem.mpu.M.x)+","+
                    String(frisbeem.mpu.M.y)+","+
                    String(frisbeem.mpu.M.z)+";");
}

void COM::str_send_acl(){
  //Send ACL Values
  telemetry("ACL",  String(frisbeem.mpu.A.x)+","+
                    String(frisbeem.mpu.A.y)+","+
                    String(frisbeem.mpu.A.z)+";");
}

void COM::str_send_acl_rl(){
  //Send ACL Values
  telemetry("AKM",  String(frisbeem.physics.A.x)+","+
                    String(frisbeem.physics.A.y)+","+
                    String(frisbeem.physics.A.z)+";");
}

void COM::str_send_vel(){
  //Send ACL Values
  telemetry("VEL",  String(frisbeem.physics.V.x)+","+
                    String(frisbeem.physics.V.y)+","+
                    String(frisbeem.physics.V.z)+";");
}

void COM::str_send_pos(){
  //Send ACL Values
  telemetry("POS",  String(frisbeem.physics.X.x)+","+
                    String(frisbeem.physics.X.y)+","+
                    String(frisbeem.physics.X.z)+";");
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
