#include "games.h"
#include "globals.h"

//Declare Games
Game Test_Game_0;
PACMAN pacman;
// TestGame2 Test_Game_2;


//Game Class Defaults
void Game::render()
{
  frisbeem.com.log( "Rendering: "+ this -> name );
  for (int i=0; i<3;i++){
    frisbeem.lights.colorAll( CRGB(0,20,50));
    frisbeem.lights.toggleStrip();
  }
}
void Game::initialize()
{
  frisbeem.com.log( "Initializing: "+ this -> name );
}

void Game::update()
{
  frisbeem.com.log( "Game: "+ this -> name );
}
void Game::leave()
{
  frisbeem.com.log( "Leaving Game: "+this->name);
}

ObserverType Game::type()
{
  return this -> _type;
}


void Game::onNotify(Event &event){ frisbeem.com.log((this->name)+" " + event.type());}
void Game::onNotify(MotionEvent &event){ frisbeem.com.log((this->name)+" " + event.type());}
void Game::onNotify(COMEvent &event){ frisbeem.com.log((this->name)+" " + event.type());}


//Other Games Yo
void PACMAN::update()
{
  now = micros();
  omega = frisbeem.mpu.G.z;
  if (fabs(omega) < omega_thresh) { omega = 0;}
  theta += omega*(now - lastTime)/1E6;
  if (theta > 360.0){ theta -= 360.0; }
  if (theta < 0.0  ){ theta += 360.0; }
  lastTime = now;

  frisbeem.com.log( "Game: "+ this -> name );
  if (frisbeem.mpu.spin ){ doDaPacman = true; }
  else { doDaPacman = false;
         theta = 0;
       }
  if ( (now - lastMouthEvent) > openMouthTime)
  {
    mouthOpen = !mouthOpen;
    if (mouthOpen){
      if (mouthAngle < mouthOpenAngle){
        mouthAngle += dmAdus * (now-lastTime);
        if (mouthAngle > mouthOpenAngle){ mouthAngle = mouthOpenAngle; }
      }
    }
    if (!mouthOpen){
      if (mouthAngle > mouthCloseAngle){
        mouthAngle -= dmAdus * (now-lastTime);
        if (mouthAngle < mouthCloseAngle){ mouthAngle = mouthCloseAngle; }
      }
    }
  }
}

void PACMAN::render()
{
  frisbeem.com.log( "Doing Pac Man: "+ this -> name );
  frisbeem.com.log( "Omega: "+ String(omega));
  frisbeem.com.log( "Ang: "+ String(theta));
  if (doDaPacman){
    for (int i=0; i<3;i++){
      if ( i == 0 ){
                    frisbeem.lights.strip0_active();
                    frisbeem.lights.activateStrip();
                    //Change Relative Angle Based On Arm Position
                    dTheta = 0;
                  }
      if ( i == 1 ){
                    frisbeem.lights.strip1_active();
                    frisbeem.lights.activateStrip();
                    if ( omega <0 ){dTheta = 120;}
                    if ( omega >0 ){dTheta = -120;}
                  }
      if ( i == 2 ){
                    frisbeem.lights.strip2_active();
                    frisbeem.lights.activateStrip();
                    if ( omega <0 ){dTheta = 240;}
                    if ( omega >0 ){dTheta = -240;}
                  }

      //Do Some LED Arm Angle Math
      LED_Ang = (theta + dTheta);
      if (LED_Ang < 0.0) { LED_Ang += 360.0; }
      if (LED_Ang > 360.0) { LED_Ang -= 360.0; }

      //Pacman lightup logic body
      if (  LED_Ang>= (360.0 - mouthAngle)  || LED_Ang <= mouthAngle  ) // Don't Light Up
      {
        frisbeem.com.log( "OFF i: "+ String(i)+" Ang: "+String(LED_Ang));
        frisbeem.lights.fillActiveStrip(mouthColor);
      }
      else{ //Light Up!
        frisbeem.com.log( "ON i: "+ String(i)+" Ang: "+String(LED_Ang));
        //Eye logic
        if (  LED_Ang>= (eye_angle - dThetaEye)  && LED_Ang <= (eye_angle + dThetaEye)  )
        {
          fill_solid( &frisbeem.lights.active_leds[0], NUM_LEDS, pacManColor );
          frisbeem.lights.active_leds[eye_index] = eyeColor;//Eye
          frisbeem.lights.active_leds[NUM_LEDS-1] = mouthColor;//Edge
          if (  LED_Ang>= (eye_angle - dThetaEye/2)  && LED_Ang <= (eye_angle + dThetaEye/2)  )
          {
            frisbeem.lights.active_leds[eye_index+1] = mouthColor;//Eye
            frisbeem.lights.active_leds[eye_index-1] = mouthColor;//Eye
            frisbeem.lights.active_leds[eye_index] = eyeColorBright;//Eye
          }
          frisbeem.lights.writeStrip( &frisbeem.lights.active_leds[0]);
        }
        else{
          frisbeem.lights.fillActiveStrip(pacManColor,mouthColor); // Light Up
        }

      }

    }
  }
  else{
    frisbeem.lights.colorAll(restColor);
  }
}

void PACMAN::onNotify(MotionEvent &event)
{
}

void PACMAN::initialize()
{
  frisbeem.com.log( "Initializing: "+ this -> name );
  dTheta = 0;
  theta = 0;
}

void PACMAN::leave()
{
  frisbeem.com.log( "Leaving Game: "+this->name);
}

// void TestGame2::update()
// {
//   frisbeem.com.log( "Game: "+ this -> name );
// }


//Game Manager Defaults
GameManager::GameManager(){
  // frisbeem.com.log("Setting Up Games");
  // setup_games();
}

void GameManager::switchToGame(unsigned int newGameIndex) {
  frisbeem.com.log("GM "+String(gameIndex)+"->"+String(newGameIndex),true);
  frisbeem.com.log("Leaving...",true);
  Serial.flush();
  leave();
  gameIndex = newGameIndex;
  frisbeem.com.log("Initializing...",true);
  Serial.flush();
  initialize();
}

Game* GameManager::currentGame()
{
  return _games.at(gameIndex);
};

void GameManager::addGame( Game *game)
{
  _games.push_back(game);
}

void GameManager::setup_games(){
  addGame( &pacman );
  addGame( &Test_Game_0 );
  // addGame( &Test_Game_2 );
  initialize(0);
}

void GameManager::initialize() {
  frisbeem.addObserver(_games.at(gameIndex));
  _games.at(gameIndex) -> initialize();
}

void GameManager::initialize( unsigned int customIndex) {
  frisbeem.addObserver(_games.at(customIndex));
  frisbeem.com.log( "Game Type Init: " + String( _games.at(customIndex) -> name),true);
  _games.at(customIndex) -> initialize();
}

void GameManager::leave() {
  frisbeem.removeObserver(_games.at(gameIndex));
  _games.at(gameIndex) -> leave();
}

void GameManager::update() {
  _games.at(gameIndex) -> update();
}

void GameManager::render() {
  _games.at(gameIndex) -> render();
}
