#ifndef GAMES_H
#define GAMES_H

#include <Arduino.h>
#include "observer.h"
#include "event.h"
#include <vector>
#include "FastLED.h"

class COMEvent;

class IGame: public Observer
{
  public:
    virtual void initialize() = 0;
    virtual void render() = 0;
    virtual void update() = 0;
    virtual void leave() = 0;
};

class Game: public IGame
{
  public:
    Game(){};
    virtual ~Game(){};
    String name = "Default Name";
    virtual void render();
    virtual void initialize();
    virtual void update();
    virtual void leave();
    virtual ObserverType type();
    ObserverType _type = GAME;

    //Observer Function Overloading
    virtual void onNotify(Event &event);
    virtual void onNotify(MotionEvent &event);
    virtual void onNotify(COMEvent &event);
};

class PACMAN: public Game
{
  public:
    String name = "PACMAN";
    virtual void render();
    virtual void initialize();
    virtual void update();
    virtual void leave();
    CRGB pacManColor = CRGB(5,1,0);
    CRGB mouthColor = CRGB(0,0,2);
    CRGB eyeColor = CRGB(0,0,15);
    CRGB eyeColorBright = CRGB(0,0,50);
    CRGB restColor = CRGB(1,0,50);
    float omega = 0;
    unsigned long now;
    unsigned long lastTime;

    float omega_thresh = 25;
    float dTheta = 0;
    float dAng_LED = 120;
    float LED_Ang = 0;
    float theta = 0;

    uint8_t eye_index = 3;
    float eye_angle = 65;
    float dThetaEye = 8;

    bool mouthOpen = true;
    unsigned long lastMouthEvent;
    unsigned long openMouthTime = 1000000;
    float mouthOpenAngle = 30.0;
    float mouthCloseAngle = 10.0;
    float dmouthAngledt = 50; //deg/s
    float dmAdus = dmouthAngledt / 1E6;
    float mouthAngle = mouthOpenAngle; //Open To Start

    float doDaPacman = false;
    virtual void onNotify(MotionEvent &event);
};

// class TestGame2: public Game
// {
//   public:
//     String name = "Test Game 2";
//     virtual void render();
//     virtual void initialize();
//     virtual void update();
//     virtual void leave();
// };

class GameManager: public IGame
{
  //Class that passes argument to current state
public:
  GameManager();
  ~GameManager(){};
  unsigned int gameIndex = 0;
  std::vector<Game*>  _games;
  //Important Funcitons
  Game* currentGame();
  void addGame( Game *game);
  void setup_games();
  virtual void switchToGame(unsigned int newGameIndex);

  //Game Interface Methods
  virtual void initialize();
  virtual void initialize( unsigned int customIndex);
  virtual void leave();
  virtual void update();
  virtual void render();


};


#endif
