#include "frisbeem.h"

void Frisbeem::initlaize(){
   //Initalize communication
   com.initialize();
   com.log("Communication Started...");

  // //Update MPU & Physics
   com.log("Go For Brains");
   mpu.initialize();
   mpu.update();
   com.log("Go For World");
   physics.initialize();

   com.log("Go For Fun");
   games_manager.setup_games();
  // //Update Strip
   com.log("Go For Lights");
   lights.initlaize();
   com.log("Go For Loop");
   lights.allOff();
   delay(100);
   lights.green();
   delay(100);
   
   com.start_cycle();
}

void Frisbeem::update(){
  //Open COM to end client if conditions are correct
  com.log("Handshakes, Formalities, Ect...");
  com.open();
  //Tick The Log So It Can Output Periodically

  com.tick();
  com.log("Beeming Into Space...");
  //Update COM layer
  com.update();

  //Handle Other Stuff
  com.log("Updating...");
  //Update MPU
  start = micros();
  //Physics Update Inner Loop
  while ( micros() - start < renderInterval) {
    com.log("Updating MPU");
    mpu.update();

    com.log("Figguring It Out");
    physics.update();

    com.log("Update Game");
    games_manager.update();

    com.log("Do Da Queue");
    event_queue.processEntries(*this);
  }
  //create newEvent and process events (circular buffer)
  com.log("Puttin On The High Beems!");
  games_manager.render();
  //processMotion();
  //Notify Observers Of new Event
  // com.log("Hollerin'");
  //Send Event To Current Motion State
  // com.log("Making Decisions");

  //Close COM to end client
  com.close();
}

// void Frisbeem::notify( Event &event)
// {
//   com.log("Event -> Frisbeem",true);
// }

void Frisbeem::notify( COMEvent &event)
{
  com.log("COM -> Frisbeem",true);
}

void Frisbeem::notify( COM_GameSelect &event)
{
  //Select Game
  com.log("Switching Games: "+ String(event.selectedGame),true);
  this -> games_manager.switchToGame( (unsigned int) event.selectedGame );
}
// void Frisbeem::updateThetaOffset()
// {
//   thisTime = micros();
//
//   //Integrate For Theta
//   if ( abs(mpu.G.z) > 0.1){
//     thetaOffset -= mpu.G.z * (thisTime - lastTime) / 1000000;
//   }
//   //Catch & Adjust For Theta Over Limit
//   if (thetaOffset > 360){
//     thetaOffset -= 360;
//   }
//   else if (thetaOffset < 0){
//     thetaOffset += 360;
//   }
//
//   lightOffset = thetaOffset / degPerPixel;
//
//   //Preserve Time Calculations
//   lastTime = thisTime;
// }

// void Frisbeem::processMotion()
// { //Generate A New Event & Add to circular buffer after deleting current item
//   //Serial.print("Getting New Event #");Serial.println(_eventCount);
//   currentMotionEvent = genNextEvent();
//   currentMotionEvent.visit( *this );//Subject Call notify()
//   //currentMotionEvent.visit( &motionState );//StateSwitchCall
//   //_motionState._states.back() -> handleInput( currentMotionEvent ); //Hack For Motion Direct
//   //currentMotionEvent.visit( _powerState);//StateSwitchCall
//
// }
//
// MotionEvent Frisbeem::genNextEvent()
// { //For Now Look At Omegea (w)
//   return MotionEvent( mpu.G.z );
// }
