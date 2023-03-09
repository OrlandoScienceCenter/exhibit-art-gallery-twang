// Required libs
#include "FastLED.h"
#include "Wire.h"
#include <WiFi.h>
#include <esp_now.h>

//#include "toneAC.h"
#include "iSin.h"
#include "RunningMedian.h"

// Included libs
#include "Enemy.h"
#include "Particle.h"
#include "Spawner.h"
#include "Lava.h"
#include "Boss.h"
#include "Conveyor.h"

// ESP-NOW stuff
//#define DEBUG_MODE_ON       // Comment this out for no debug serial messages

typedef struct structEspNowPacket                                           // Structure example to send data, must match the sender structure
{
  int joystickAveragedTilt;
  int joystickPeakWobble;

  bool buttonPressed = false;
} 
structEspNowPacket;

structEspNowPacket pedestalData;                               // Create a structEspNowPacket called pedestalData to deserialize the incoming packet from the sender over esp-now

uint8_t packetLength;

bool ledState = false;

bool resetButtonStatus = false;

#ifdef DEBUG_MODE_ON

  unsigned long lastTime = 0;  
  unsigned long timerDelay = 1000;                             // send readings timer

#endif 

// LED setup
#define NUM_LEDS             692  // Will not show a blue dot at 693 or greater. Wat.

#define DATA_PIN 13   // Green wire through resistor       (GPIO D33 on ESP32)
#define CLOCK_PIN 14  // Yellow wire through resistor      (GPIO D25 on ESP32)

#define LED_COLOR_ORDER      BGR   
#define BRIGHTNESS           150
#define DIRECTION            1      // 0 = right to left, 1 = left to right

#define MIN_REDRAW_INTERVAL  16     // Min redraw interval (ms) 33 = 30fps / 16 = 63fps
#define USE_GRAVITY          0      // 0/1 use gravity (LED strip going up wall)
#define BEND_POINT           0      // 0/1000 point at which the LED strip goes up the wall
#define LED_TYPE             APA102 // type of LED strip to use(APA102 - DotStar, WS2811 - NeoPixel)

#define RTS_PIN 6

// GAME
long previousMillis = 0;           // Time of the last redraw
int levelNumber = 0;
long lastInputTime = 0;
#define TIMEOUT              30000 
#define LEVEL_COUNT          9
#define MAX_VOLUME           10
iSin isin = iSin();

// JOYSTICK
#define JOYSTICK_ORIENTATION 1     // 0, 1 or 2 to set the angle of the joystick
#define JOYSTICK_DIRECTION   0     // 0/1 to flip joystick direction
#define ATTACK_THRESHOLD     70 // The threshold that triggers an attack
#define JOYSTICK_DEADZONE    5     // Angle to ignore

int joystickTilt = 0;              // Stores the angle of the joystick
int joystickWobble = 0;            // Stores the max amount of acceleration (wobble)

// WOBBLE ATTACK
#define ATTACK_WIDTH        140     // Width of the wobble attack, world is 1000 wide
#define ATTACK_DURATION     500    // Duration of a wobble attack (ms)
long attackMillis = 0;             // Time the attack started
bool attacking = 0;                // Is the attack in progress?
#define BOSS_WIDTH          40

// PLAYER
#define MAX_PLAYER_SPEED    10     // Max move speed of the player
char* stage;                       // what stage the game is at (PLAY/DEAD/WIN/GAMEOVER)
long stageStartTime;               // Stores the time the stage changed for stages that are time based
int playerPosition;                // Stores the player position
int playerPositionModifier;        // +/- adjustment to player position
bool playerAlive;
long killTime;
int lives = 1;

// POOLS
int lifeLEDs[3] = {52, 50, 40};
Enemy enemyPool[10] = {
    Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy()
};
int const enemyCount = 10;
Particle particlePool[40] = {
    Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle(), Particle()
};
int const particleCount = 40;
Spawner spawnPool[2] = {
    Spawner(), Spawner()
};
int const spawnCount = 2;
Lava lavaPool[4] = {
    Lava(), Lava(), Lava(), Lava()
};
int const lavaCount = 4;
Conveyor conveyorPool[2] = {
    Conveyor(), Conveyor()
};
int const conveyorCount = 2;
Boss boss = Boss();

CRGB leds[NUM_LEDS];

RunningMedian MPUAngleSamples = RunningMedian(5);
RunningMedian MPUWobbleSamples = RunningMedian(5);

// char incomingSerialLine[50];
// int readPos = 0;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)      // Callback function that will be executed when data is received
{
  memcpy(&pedestalData, incomingData, sizeof(pedestalData));

  toggleBuiltInLed();

  packetLength = len;

  joystickTilt = pedestalData.joystickAveragedTilt;

  joystickWobble = pedestalData.joystickPeakWobble;

  resetButtonStatus = pedestalData.buttonPressed;

  #ifdef DEBUG_MODE_ON
  
    debugPrintToSerialJoystickValues();

  #endif
}

void setup()
{
    Serial.begin(115200);

    // From wemos D1 mini
    //Serial3.begin(9600);
    
    // Fast LED
    FastLED.addLeds<LED_TYPE, DATA_PIN, CLOCK_PIN, LED_COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.setDither(1);
    
    // Life LEDs
    for(int i = 0; i<3; i++){
        pinMode(lifeLEDs[i], OUTPUT);
        digitalWrite(lifeLEDs[i], HIGH);
    }

    initializeEspNow();

    Serial.println(F("Init v113, bout to load level."));
    
    loadLevel();
}

void loop()
{
    long mm = millis();
    int brightness = 0;
    /*
    if(stage == "PLAY"){
        if(attacking){
            SFXattacking();
        }else{
            SFXwobble(joystickwobble);
        }
    }else if(stage == "DEAD"){
        SFXdead();
    }
    */

    if (mm - previousMillis >= MIN_REDRAW_INTERVAL) {
        
        getInput();
        
        long frameTimer = mm;
        previousMillis = mm;
        
        if (resetButtonStatus)
        {
          stage = "DEAD";
          
          resetButtonStatus = false;
        }

        if(abs(joystickWobble) > JOYSTICK_DEADZONE){
            lastInputTime = mm;
            if(stage == "SCREENSAVER"){
                levelNumber = -1;
                stageStartTime = mm;
                stage = "WIN";
            }
        }else{
            if(lastInputTime+TIMEOUT < mm){
                stage = "SCREENSAVER";
            }
        }
        if(stage == "SCREENSAVER"){
            screenSaverTick();
        }else if(stage == "PLAY"){
            // PLAYING
            if(attacking && attackMillis+ATTACK_DURATION < mm) attacking = 0;
            
            // If not attacking, check if they should be
            if(!attacking && joystickWobble > ATTACK_THRESHOLD){
                attackMillis = mm;
                attacking = 1;
            }
            
            // If still not attacking, move!
            playerPosition += playerPositionModifier;
            if(!attacking){
                int moveAmount = (joystickWobble/6.0);
                if(DIRECTION) moveAmount = -moveAmount;
                moveAmount = constrain(moveAmount, -MAX_PLAYER_SPEED, MAX_PLAYER_SPEED);
                playerPosition -= moveAmount;
                if(playerPosition < 0) playerPosition = 0;
                if(playerPosition >= 1000 && !boss.Alive()) {
                    // Reached exit!
                    levelComplete();
                    return;
                }
            }
            
            if(inLava(playerPosition)){
                die();
            }
            
            // Ticks and draw calls
            FastLED.clear();
            tickConveyors();
            tickSpawners();
            tickBoss();
            tickLava();
            tickEnemies();
            drawPlayer();
            drawAttack();
            drawExit();
        }else if(stage == "DEAD"){
            // DEAD
            FastLED.clear();
            if(!tickParticles()){
                loadLevel();
            }
        }else if(stage == "WIN"){
            // LEVEL COMPLETE
            FastLED.clear();
            if(stageStartTime+500 > mm)
            {
                int mappedStageStartTime = map(((mm-stageStartTime)), 0, 500, NUM_LEDS, 0);
                int n = max(mappedStageStartTime, 0);

                for(int i = NUM_LEDS; i>= n; i--)
                {
                    brightness = 255;
                    leds[i] = CRGB(0, brightness, 0);
                }

                //SFXwin();
            }
            else if(stageStartTime+1000 > mm)
            {
                int mappedStageStartTime = map(((mm-stageStartTime)), 500, 1000, NUM_LEDS, 0);
                int n = max(mappedStageStartTime, 0);
            
                for(int i = 0; i< n; i++)
                {
                    brightness = 255;
                    leds[i] = CRGB(0, brightness, 0);
                }

                //SFXwin();

            }else if(stageStartTime+1200 > mm){
                leds[0] = CRGB(0, 255, 0);
            }else{
                nextLevel();
            }
        }else if(stage == "COMPLETE"){
            FastLED.clear();
            //SFXcomplete();
            if(stageStartTime+500 > mm)
            {
                int mappedStageStartTime = map(((mm-stageStartTime)), 0, 500, NUM_LEDS, 0);
                int n = max(mappedStageStartTime, 0);

                for(int i = NUM_LEDS; i>= n; i--)
                {
                    brightness = (sin(((i*10)+mm)/500.0)+1)*255;
                    leds[i].setHSV(brightness, 255, 50);
                }
            }else if(stageStartTime+5000 > mm){
                for(int i = NUM_LEDS; i>= 0; i--){
                    brightness = (sin(((i*10)+mm)/500.0)+1)*255;
                    leds[i].setHSV(brightness, 255, 50);
                }
            }else if(stageStartTime+5500 > mm)
            {
                int mappedStageStartTime = map(((mm-stageStartTime)), 5000, 5500, NUM_LEDS, 0);
                int n = max(mappedStageStartTime, 0);
              
                for(int i = 0; i< n; i++)
                {
                    brightness = (sin(((i*10)+mm)/500.0)+1)*255;
                    leds[i].setHSV(brightness, 255, 50);
                }
            }else{
                nextLevel();
            }
        }else if(stage == "GAMEOVER"){
            // GAME OVER!
            FastLED.clear();
            stageStartTime = 0;
        }
        

//        Serial.print(millis()-mm);
//        Serial.print(" - ");
        FastLED.show();
//        Serial.println(millis()-mm);
    }
}

void debugPrintToSerialJoystickValues()
{
  Serial.println();
  Serial.println();

  Serial.print(F("Bytes received: "));
  Serial.println(packetLength);

  Serial.print(F("Tilt from pedestal: "));
  Serial.print(pedestalData.joystickAveragedTilt);

  Serial.print(F(", peak recent wobble value from pedestal: "));
  Serial.print(pedestalData.joystickPeakWobble);
  
  Serial.print(F(", button state: "));
  Serial.println(pedestalData.buttonPressed);
}
     
void initializeEspNow()
{
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }
  
  // Once ESPNow is successfully Init, register for recieve callback to get recieved packet
  //esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void toggleBuiltInLed()
{
  //digitalWrite(LED_BUILTIN, ledState);
  
  ledState = !ledState;
}

  // int incomingByte;
  // int currentPos = 0;

  // if (Serial3.available() > 0)
  // {
  //   delay(50);
    
  //   while (Serial3.available() > 0) 
  //   {
  //     incomingByte = Serial3.read();
  
  //     char incomingChar = char(incomingByte);
  
  //     incomingSerialLine[currentPos] = incomingChar;
      
  //     currentPos++;
        
  //     if (incomingChar == '.')
  //     {
  //       incomingSerialLine[currentPos] = '\0';
        
  //       break;
  //     }
  //   }


  //   if (DEBUG_MODE_ON)
  //   {
  //     Serial.println();
  //     Serial.println();
  
  //     Serial.print("incomingSerialLine: ");
  //     Serial.println(incomingSerialLine);
  //   }
     
  //   setJoystickTilt();
  //   setJoystickWobble();

  //   for (int i = 0; i < sizeof(incomingSerialLine); i++)
  //   {
  //     incomingSerialLine[i] = '\0';
  //   }
  //} 
// }

// void setJoystickTilt()
// {
//     char tiltString[50];

//     int tiltStringCheckPos = 0;

//     int tiltStringSavePos = 0;
  
//     while (char(incomingSerialLine[tiltStringCheckPos]) != 't')
//     {
//       tiltStringCheckPos++;
//     }
    
//     tiltStringCheckPos++;


//     if (DEBUG_MODE_ON)
//     {
//       Serial.print("First char is: ");
//       Serial.println(incomingSerialLine[tiltStringCheckPos]);
//     }
    
//     // Now wobbleStringCheckPos should be at the letter after 't'
//     while (isDigit(incomingSerialLine[tiltStringCheckPos]))
//     {
      
//       if (DEBUG_MODE_ON)
//       {
//         Serial.print("Adding: ");
//         Serial.print(incomingSerialLine[tiltStringCheckPos]);
//       }
      
//       tiltString[tiltStringSavePos] = incomingSerialLine[tiltStringCheckPos];
      
//       tiltStringSavePos++;
//       tiltStringCheckPos++;
//     }
    
//     tiltStringSavePos++;
//     tiltString[tiltStringSavePos] = '\0';

// //    Serial.print("TiltString: ");
// //    Serial.println(tiltString);

//     joystickTilt = atoi(tiltString);


//     if (DEBUG_MODE_ON)
//     {
//       Serial.print("joystickTilt: ");
//       Serial.println(joystickTilt);
//     }
// }

// void setJoystickWobble()
// {
//     char wobbleString[50];

//     int wobbleStringCheckPos = 0;

//     int wobbleStringSavePos = 0;
  
//     while (char(incomingSerialLine[wobbleStringCheckPos]) != 'w')
//     {
//       wobbleStringCheckPos++;
//     }
    
//     wobbleStringCheckPos++;


//     if (DEBUG_MODE_ON)
//     {
//       Serial.print("First char is: ");
//       Serial.println(incomingSerialLine[wobbleStringCheckPos]);
//     }
    
//     // Now wobbleStringCheckPos should be at the letter after 't'
//     while (isDigit(incomingSerialLine[wobbleStringCheckPos]))
//     {
      
//       if (DEBUG_MODE_ON)
//       {
//         Serial.print("Adding: ");
//         Serial.print(incomingSerialLine[wobbleStringCheckPos]);
//       }
      
//       wobbleString[wobbleStringSavePos] = incomingSerialLine[wobbleStringCheckPos];
      
//       wobbleStringSavePos++;
//       wobbleStringCheckPos++;
//     }
    
//     wobbleStringSavePos++;
//     wobbleString[wobbleStringSavePos] = '\0';

// //    Serial.print("wobbleString: ");
// //    Serial.println(wobbleString);

//     joystickWobble = atoi(wobbleString);


//     if (DEBUG_MODE_ON)
//     {
//       Serial.print("joystickwobble: ");
//       Serial.println(joystickWobble);
//     }
  
// }


// ---------------------------------
// ------------ LEVELS -------------
// ---------------------------------
void loadLevel(){
    updateLives();
    cleanupLevel();
    playerPosition = 0;
    playerAlive = 1;
    switch(levelNumber){
        case 0:
            // Left or right?
            playerPosition = 200;
            spawnEnemy(1, 0, 0, 0);
            break;
        case 1:
            // Slow moving enemy
            spawnEnemy(900, 0, 1, 0);
            break;
        case 2:
            // Spawning enemies at exit every 2 seconds
            spawnPool[0].Spawn(1000, 3000, 2, 0, 0);
            break;
        case 3:
            // Lava intro
            spawnLava(400, 490, 2000, 2000, 0, "OFF");
            spawnPool[0].Spawn(1000, 5500, 3, 0, 0);
            break;
        case 4:
            // Sin enemy
            spawnEnemy(700, 1, 7, 275);
            spawnEnemy(500, 1, 5, 250);
            break;
        case 5:
            // Conveyor
            spawnConveyor(100, 600, -1);
            spawnEnemy(800, 0, 0, 0);
            break;
        case 6:
            // Conveyor of enemies
            spawnConveyor(50, 1000, 1);
            spawnEnemy(300, 0, 0, 0);
            spawnEnemy(400, 0, 0, 0);
            spawnEnemy(500, 0, 0, 0);
            spawnEnemy(600, 0, 0, 0);
            spawnEnemy(700, 0, 0, 0);
            spawnEnemy(800, 0, 0, 0);
            spawnEnemy(900, 0, 0, 0);
            break;
        case 7:
            // Lava run
            spawnLava(195, 300, 2000, 2000, 0, "OFF");
            spawnLava(350, 455, 2000, 2000, 0, "OFF");
            spawnLava(510, 610, 2000, 2000, 0, "OFF");
            spawnLava(660, 760, 2000, 2000, 0, "OFF");
            spawnPool[0].Spawn(1000, 3800, 4, 0, 0);
            break;
        case 8:
            // Sin enemy #2
            spawnEnemy(700, 1, 7, 275);
            spawnEnemy(500, 1, 5, 250);
            spawnPool[0].Spawn(1000, 5500, 4, 0, 3000);
            spawnPool[1].Spawn(0, 5500, 5, 1, 10000);
            spawnConveyor(100, 900, -1);
            break;
        case 9:
            // Boss
            spawnBoss();
            break;
    }
    stageStartTime = millis();
    stage = "PLAY";
}

void spawnBoss(){
    boss.Spawn();
    moveBoss();
}

void moveBoss(){
    int spawnSpeed = 2500;
    if(boss._lives == 2) spawnSpeed = 2000;
    if(boss._lives == 1) spawnSpeed = 1500;
    spawnPool[0].Spawn(boss._pos, spawnSpeed, 3, 0, 0);
    spawnPool[1].Spawn(boss._pos, spawnSpeed, 3, 1, 0);
}

void spawnEnemy(int pos, int dir, int sp, int wobble){
    for(int e = 0; e<enemyCount; e++){
        if(!enemyPool[e].Alive()){
            enemyPool[e].Spawn(pos, dir, sp, wobble);
            enemyPool[e].playerSide = pos > playerPosition?1:-1;
            return;
        }
    }
}

void spawnLava(int left, int right, int ontime, int offtime, int offset, char* state){
    for(int i = 0; i<lavaCount; i++){
        if(!lavaPool[i].Alive()){
            lavaPool[i].Spawn(left, right, ontime, offtime, offset, state);
            return;
        }
    }
}

void spawnConveyor(int startPoint, int endPoint, int dir){
    for(int i = 0; i<conveyorCount; i++){
        if(!conveyorPool[i]._alive){
            conveyorPool[i].Spawn(startPoint, endPoint, dir);
            return;
        }
    }
}

void cleanupLevel(){
    for(int i = 0; i<enemyCount; i++){
        enemyPool[i].Kill();
    }
    for(int i = 0; i<particleCount; i++){
        particlePool[i].Kill();
    }
    for(int i = 0; i<spawnCount; i++){
        spawnPool[i].Kill();
    }
    for(int i = 0; i<lavaCount; i++){
        lavaPool[i].Kill();
    }
    for(int i = 0; i<conveyorCount; i++){
        conveyorPool[i].Kill();
    }
    boss.Kill();
}

void levelComplete(){
    stageStartTime = millis();
    stage = "WIN";
    if(levelNumber == LEVEL_COUNT) stage = "COMPLETE";
    lives = 3;
    updateLives();
}

void nextLevel(){
    levelNumber ++;
    if(levelNumber > LEVEL_COUNT) levelNumber = 0;
    loadLevel();
}

void gameOver(){
    levelNumber = 0;
    loadLevel();
}

void die(){
    playerAlive = 0;
    if(levelNumber > 0) lives --;
    updateLives();
    if(lives == 0){
        levelNumber = 0;
        lives = 3;
    }
    for(int p = 0; p < particleCount; p++){
        particlePool[p].Spawn(playerPosition);
    }
    stageStartTime = millis();
    stage = "DEAD";
    killTime = millis();
}

// ----------------------------------
// -------- TICKS & RENDERS ---------
// ----------------------------------
void tickEnemies(){
    for(int i = 0; i<enemyCount; i++){
        if(enemyPool[i].Alive()){
            enemyPool[i].Tick();
            // Hit attack?
            if(attacking){
                if(enemyPool[i]._pos > playerPosition-(ATTACK_WIDTH/2) && enemyPool[i]._pos < playerPosition+(ATTACK_WIDTH/2)){
                   enemyPool[i].Kill();
                   //SFXkill();
                }
            }
            if(inLava(enemyPool[i]._pos)){
                enemyPool[i].Kill();
                //SFXkill();
            }
            // Draw (if still alive)
            if(enemyPool[i].Alive()) {
                leds[getLED(enemyPool[i]._pos)] = CRGB(255, 0, 0);
            }
            // Hit player?
            if(
                (enemyPool[i].playerSide == 1 && enemyPool[i]._pos <= playerPosition) ||
                (enemyPool[i].playerSide == -1 && enemyPool[i]._pos >= playerPosition)
            ){
                die();
                return;
            }
        }
    }
}

void tickBoss(){
    // DRAW
    if(boss.Alive()){
        boss._ticks ++;
        for(int i = getLED(boss._pos-BOSS_WIDTH/2); i<=getLED(boss._pos+BOSS_WIDTH/2); i++){
            leds[i] = CRGB::DarkRed;
            leds[i] %= 100;
        }
        // CHECK COLLISION
        if(getLED(playerPosition) > getLED(boss._pos - BOSS_WIDTH/2) && getLED(playerPosition) < getLED(boss._pos + BOSS_WIDTH)){
            die();
            return; 
        }
        // CHECK FOR ATTACK
        if(attacking){
            if(
              (getLED(playerPosition+(ATTACK_WIDTH/2)) >= getLED(boss._pos - BOSS_WIDTH/2) && getLED(playerPosition+(ATTACK_WIDTH/2)) <= getLED(boss._pos + BOSS_WIDTH/2)) ||
              (getLED(playerPosition-(ATTACK_WIDTH/2)) <= getLED(boss._pos + BOSS_WIDTH/2) && getLED(playerPosition-(ATTACK_WIDTH/2)) >= getLED(boss._pos - BOSS_WIDTH/2))
            ){
               boss.Hit();
               if(boss.Alive()){
                   moveBoss();
               }else{
                   spawnPool[0].Kill();
                   spawnPool[1].Kill();
               }
            }
        }
    }
}

void drawPlayer(){
    leds[getLED(playerPosition)] = CRGB(0, 255, 0);
}

void drawExit(){
    if(!boss.Alive()){
        leds[NUM_LEDS-1] = CRGB(0, 0, 255);
    }
}

void tickSpawners(){
    long mm = millis();
    for(int s = 0; s<spawnCount; s++){
        if(spawnPool[s].Alive() && spawnPool[s]._activate < mm){
            if(spawnPool[s]._lastSpawned + spawnPool[s]._rate < mm || spawnPool[s]._lastSpawned == 0){
                spawnEnemy(spawnPool[s]._pos, spawnPool[s]._dir, spawnPool[s]._sp, 0);
                spawnPool[s]._lastSpawned = mm;
            }
        }
    }
}

void tickLava(){
    int A, B, p, i, brightness, flicker;
    long mm = millis();
    Lava LP;
    for(i = 0; i<lavaCount; i++){
        flicker = random8(5);
        LP = lavaPool[i];
        if(LP.Alive()){
            A = getLED(LP._left);
            B = getLED(LP._right);
            if(LP._state == "OFF"){
                if(LP._lastOn + LP._offtime < mm){
                    LP._state = "ON";
                    LP._lastOn = mm;
                }
                for(p = A; p<= B; p++){
                    leds[p] = CRGB(3+flicker, (3+flicker)/1.5, 0);
                }
            }else if(LP._state == "ON"){
                if(LP._lastOn + LP._ontime < mm){
                    LP._state = "OFF";
                    LP._lastOn = mm;
                }
                for(p = A; p<= B; p++){
                    leds[p] = CRGB(150+flicker, 100+flicker, 0);
                }
            }
        }
        lavaPool[i] = LP;
    }
}

bool tickParticles(){
    bool stillActive = false;
    for(int p = 0; p < particleCount; p++){
        if(particlePool[p].Alive()){
            particlePool[p].Tick(USE_GRAVITY);
            leds[getLED(particlePool[p]._pos)] += CRGB(particlePool[p]._power, 0, 0);
            stillActive = true;
        }
    }
    return stillActive;
}

void tickConveyors(){
    int b, dir, n, i, ss, ee, led;
    long m = 10000+millis();
    playerPositionModifier = 0;
    
    for(i = 0; i<conveyorCount; i++){
        if(conveyorPool[i]._alive){
            dir = conveyorPool[i]._dir;
            ss = getLED(conveyorPool[i]._startPoint);
            ee = getLED(conveyorPool[i]._endPoint);
            for(led = ss; led<ee; led++){
                b = 5;
                n = (-led + (m/100)) % 5;
                if(dir == -1) n = (led + (m/100)) % 5;
                b = (5-n)/2.0;
                if(b > 0) leds[led] = CRGB(0, 0, b);
            }
            
            if(playerPosition > conveyorPool[i]._startPoint && playerPosition < conveyorPool[i]._endPoint){
                if(dir == -1){
                    playerPositionModifier = -(MAX_PLAYER_SPEED-4);
                }else{
                    playerPositionModifier = (MAX_PLAYER_SPEED-4);
                }
            }
        }
    }
}

void drawAttack(){
    if(!attacking) return;
    int n = map(millis() - attackMillis, 0, ATTACK_DURATION, 100, 5);
    for(int i = getLED(playerPosition-(ATTACK_WIDTH/2))+1; i<=getLED(playerPosition+(ATTACK_WIDTH/2))-1; i++){
        leds[i] = CRGB(0, 0, n);
    }
    if(n > 90) {
        n = 255;
        leds[getLED(playerPosition)] = CRGB(255, 255, 255);
    }else{
        n = 0;
        leds[getLED(playerPosition)] = CRGB(0, 255, 0);
    }
    leds[getLED(playerPosition-(ATTACK_WIDTH/2))] = CRGB(n, n, 255);
    leds[getLED(playerPosition+(ATTACK_WIDTH/2))] = CRGB(n, n, 255);
}

int getLED(int pos){
    // The world is 1000 pixels wide, this converts world units into an LED number
    return constrain((int)map(pos, 0, 1000, 0, NUM_LEDS-1), 0, NUM_LEDS-1);
}

bool inLava(int pos){
    // Returns if the player is in active lava
    int i;
    Lava LP;
    for(i = 0; i<lavaCount; i++){
        LP = lavaPool[i];
        if(LP.Alive() && LP._state == "ON"){
            if(LP._left < pos && LP._right > pos) return true;
        }
    }
    return false;
}

void updateLives(){
    // Updates the life LEDs to show how many lives the player has left
    for(int i = 0; i<3; i++){
       digitalWrite(lifeLEDs[i], lives>i?HIGH:LOW);
    }
}


// ---------------------------------
// --------- SCREENSAVER -----------
// ---------------------------------
void screenSaverTick(){
    int n, b, c, i;
    long mm = millis();
    int mode = (mm/20000)%2;
    
    for(i = 0; i<NUM_LEDS; i++){
        leds[i].nscale8(250);
    }
    if(mode == 0){
        // Marching green <> orange
        n = (mm/250)%10;
        b = 10+((sin(mm/500.00)+1)*20.00);
        c = 20+((sin(mm/5000.00)+1)*33);
        for(i = 0; i<NUM_LEDS; i++){
            if(i%10 == n){
                leds[i] = CHSV( c, 255, 150);
            }
        }
    }else if(mode == 1){
        // Random flashes
        randomSeed(mm);
        for(i = 0; i<NUM_LEDS; i++){
            if(random8(200) == 0){
                leds[i] = CHSV( 25, 255, 100);
            }
        }
    }
}

// ---------------------------------
// ----------- JOYSTICK ------------
// ---------------------------------
void getInput(){

  char joystickTiltString[6];

  int characterPosition = 0;

  while (Serial1.available() > 0)           //Wait for user input
  { 
    int byteReceived;

delay(15);
    
    byteReceived = Serial1.read();       //Read user input and hold it in a variable

    Serial.println("Next char: ");
    Serial.println(char(byteReceived));
    Serial.println("int: ");
    Serial.println(byteReceived);

//    if (byteReceived == '\n') {
//        joystickTiltString[characterPosition] = '\0';
//        break;
//    } 
//    
//    joystickTiltString[characterPosition] = char(byteReceived);   //Print the character value of user input 
//
//    characterPosition++;
  }

//  Serial.print(F("Raw val read: "));
//  Serial.println(joystickTiltString);
//    
//  joystickTilt = atoi(joystickTiltString);
//
//  Serial.print(F("Got joystickTilt vlaue of: "));
//  Serial.println(joystickTilt);
  
  if(JOYSTICK_DIRECTION == 1) {
      joystickTilt = 0-joystickTilt;
  }
  
  //joystickWobble = abs(MPUWobbleSamples.getHighest());
}


// ---------------------------------
// -------------- SFX --------------
// ---------------------------------
/*void SFXtilt(int amount){ 
    int f = map(abs(amount), 0, 90, 80, 900)+random8(100);
    if(playerPositionModifier < 0) f -= 500;
    if(playerPositionModifier > 0) f += 200;
    toneAC(f, min(min(abs(amount)/9, 5), MAX_VOLUME));
    
}
void SFXattacking(){
    int freq = map(sin(millis()/2.0)*1000.0, -1000, 1000, 500, 600);
    if(random8(5)== 0){
      freq *= 3;
    }
    toneAC(freq, MAX_VOLUME);
}
void SFXdead(){
    int freq = max(1000 - (millis()-killTime), 10);
    freq += random8(200);
    int vol = max(10 - (millis()-killTime)/200, 0);
    toneAC(freq, MAX_VOLUME);
}
void SFXkill(){
    toneAC(2000, MAX_VOLUME, 1000, true);
}
void SFXwin(){
    int freq = (millis()-stageStartTime)/3.0;
    freq += map(sin(millis()/20.0)*1000.0, -1000, 1000, 0, 20);
    int vol = 10;//max(10 - (millis()-stageStartTime)/200, 0);
    toneAC(freq, MAX_VOLUME);
}

void SFXcomplete(){
    noToneAC();
}
*/
