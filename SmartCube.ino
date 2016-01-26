#include "beta-cube-library-v3.1/beta-cube-library-v3.1.h"
#include "MQTT/MQTT.h"
#include <math.h>
#include <string>
#include <bitset>

#undef max
#undef min

#include <stdint.h>
#include <vector>

//set up the pin that controls the LEDs, the type of LEDs (WS2812B) and the number of LEDs in the cube (8*8*8=512)
#define PIXEL_PIN D0
#define PIXEL_COUNT 512
#define PIXEL_TYPE WS2812B
#define SIDE 8 //Change this for a cube thats bigger then 8x8x8
#define MODE D3

//SYSTEM_MODE(SEMI_AUTOMATIC);  //don't connect to the internet on boot
#define MODE D3
#define MICROPHONE 12
#define GAIN_CONTROL 11
#define MAX_POINTS 20
#define SPEED 0.22

Cube cube = Cube();

/******************************
 * general settings definitions
 * ***************************/
 
//DEBUG
bool enablePrintStatements = false;
 
//Cube && Animation settings
int maxBrightness = 100;
int AutocyleTime = 22222;
bool flipEnabled = true;
 
//EEPROM
bool saveEnabled = false; //Keep settings and mode after powering off
 
//Particle
bool particleFeeds = false; //Publish data to particle feeds and dashboard
bool particleApi = true; //Required to use external functionality
String particleLockFeed = "";
String particleAnimationFeed = "";
String particlePowerFeed = "";
 
//MQTT
bool mqttEnabled = false;
char* mqttHost = "";
int mqttPort = 1883;
String mqttUsername = "";
String mqttPassword = "";

//Change to "" if not being used
String lockFeed = "";
String animationFeed = "";
String powerFeed = "";

/******************************
 * animation id definitions
 * ***************************/
#define FIREWORKS 0
#define ZPLASMA 1
#define SQUARRAL 2
#define SNAKE 3
#define ROUTINES 4
int activeAnimation = 0;

/******************************
 * function definitions
 * ***************************/
void background(Color col);
void setBrightness(int brightness);
Color getPixel(int x, int y, int z);
void setPixel(int x, int y, int z, Color col);
Color ColorMap(float val, float min, float maximum);
Color lerpColor(Color a, Color b, int val, int min, int maximum);
void add(Point& a, Point& b);

 /**********************************
 * Api Settings definitions *
 * *********************************/
 bool animationLocked = false;
 bool isPowered = true;
 
 /**********************************
 * flip variables *
 * ********************************/
 //accelerometer pinout
#define X 13
#define Y 14
#define Z 15
#define FACEPLANT 2300
#define UPSIDE_DOWN 1850
#define RIGHTSIDE_UP 2400
#define LEFT_SIDE 1800
#define RIGHT_SIDE 2400
#define FLIP_TIMEOUT 3000
#define FLIP_DEBOUNCE 250

long lastFaceplant=-1*FLIP_TIMEOUT;
bool upsideDown=false;
bool sideways=false;
int upsideDownTime=-1*FLIP_TIMEOUT;
long lastAutoCycle=0;
int lastLeft=-1*FLIP_TIMEOUT;
int lastRight=-1*FLIP_TIMEOUT;
int accelerometer[3];
long lastChange=0;
bool lastCycleState = true;
 
 /*******************************
 * fade variables *
 * ****************************/
bool fading=false;
int fadeValue=255;
int fadeSpeed=2;

/******************************
 * Adafruit.io definitions *
 * ****************************/
void MQTTcallback(char* topic, byte* payload, unsigned int length);
MQTT client(mqttHost, mqttPort, MQTTcallback);

/******************************
 * Mode definitions
 * ***************************/
#define NORMAL 0
#define STREAMING 1
 int mode = NORMAL;
 
/******************************
 * fireworks variables *
 * ****************************/
int centerX, centerY, centerZ;
int launchX, launchZ;
int brightness=50;
float radius=0;
float speed;
bool showRocket;
bool exploded;
float xInc, yInc, zInc;
float rocketX, rocketY, rocketZ;
float launchTime;
int maximumSize;
Color rocketColor, fireworkColor; 

/********************************
 * zplasma variables *
 * *****************************/
float phase = 0.0;
float phaseIncrement = 0.035; // Controls the speed of the moving Points. Higher == faster
float ColorStretch = 0.23; // Higher numbers will produce tighter Color bands 
float plasmaBrightness = 0.2;
Color plasmaColor;

/*********************************
 * squarral variables *
 * ******************************/

#define TRAIL_LENGTH 50

int frame=0;
Color pixelColor;
Point position, increment, pixel;
Point trailPoints[TRAIL_LENGTH];
int posX, posY, posZ;
int incX, incY, incZ;
int squarral_zInc=1;
int bound=0;
int boundInc=1;
unsigned char axis=0;
bool rainbow=true;

/*******************************
 * snake variables *
 * ****************************/

int deathFrame;
int snakeFrameCount;
int initialSnakeLength;
int snakeSpeed;

struct voxel {
  int x;
  int y;
  int z;

  voxel(int x=0, int y=0, int z=0) 
    : x(x), y(y), z(z)
  {
  };

  bool operator==(const voxel& v) const
  {
      return (v.x == x && v.y == y && v.z == z);
  };

  bool operator!=(const voxel& v) const
  {
      return (v.x != x || v.y != y || v.z != z);
  };

};
    
voxel operator+(const voxel& v1, const voxel& v2) {
  return voxel(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);    
}

std::vector<voxel> snake;
std::vector<voxel> treats;
voxel* snakeDirection;

bool canMove(voxel* direction);
bool containsVoxel(std::vector<voxel> &vectorList, voxel &voxel);
double distance(voxel v, voxel w);

std::vector<voxel> possibleDirections = {
  { 1,  0,  0},
  {-1,  0,  0},
  { 0,  1,  0},
  { 0, -1,  0},
  { 0,  0,  1},
  { 0,  0, -1}
};

/**************************************
 * startup and loop functions *
 * ************************************/

void setup() {
    Serial.begin(9600);
    
    pinMode(7,OUTPUT);
    digitalWrite(7, HIGH);
    
    // seed the random number generator.  THINGS WILL NEVER BE THE SAME AGAIN
    uint32_t seed = millis(); 
    srand(seed);
    
    //Load settings from EEPROM
    loadSettings();
    
    //Init things
    initCube();
    initEventHandlers();
    initParticleVariables();
    initMQTT();
    initMicrophone();
    initAnimations();
}

void initAnimations() {
    initFireworks();
    initSquarral();
    initSnake();
}

void initCube() {
    cube.begin();
    cube.initButtons();
    cube.onlineOfflineSwitch();
}


void initMicrophone()
{
  pinMode(GAIN_CONTROL, OUTPUT);
  digitalWrite(GAIN_CONTROL, LOW);
}

void initEventHandlers() {
    // register the reset handler
    //System.on(reset, reset_handler);
}

void loop() {
    if (mqttEnabled) {
        if (client.isConnected()) {
            // resume normal operation
            RGB.control(false);
            client.loop();   
        } else {
            // take control of the LED
            RGB.control(true);
            RGB.color(255, 0, 0);
        }
    }
    
    if (isPowered==false) {
        background(black);
        cube.show();
        return;
    }
    
    if(fading)
        fade();
        
    if (mode == STREAMING) {
        cube.listen();
    }
    
    if (mode == NORMAL) {
        background(black);
        switch(activeAnimation) {
            case(FIREWORKS):
                updateFireworks();
                break;
            case (ZPLASMA):
                zPlasma();
                break;
            case (SQUARRAL):
                squarral();
                break;
            case (SNAKE):
                runSnake();
                break;
            default:
                updateFireworks();
                break;
        }
    }
        
    //check to see how if the cube has been flipped if its enabled
    if (flipEnabled) {
        checkFlipState();
    }
        
    cube.show();
    
    if(fading) {
        fadeValue-=fadeSpeed;
        //if we're done fading)
        if(fadeValue<=0)
        {
            fading=false;
            fadeValue=255;
        }
        else
            fade();
    }
    
    if(!animationLocked)
        if(millis()-lastAutoCycle>AutocyleTime)   //in autocycle, change demos every 15 seconds
        {
            incrementAnimation();
            lastAutoCycle=millis();
        }
}

/***********************************
 * event functions and definitions *
 * *********************************/

void reset_handler(system_event_t event, int param)
{
    MQTTPublish(powerFeed, "OFF");
    ParticlePublish(particlePowerFeed, "OFF");
}

/***********************************
 * debug functions and definitions *
 * *********************************/
 
void debug(String message) {
    if (enablePrintStatements) {
        Serial.println(message);
    }
}

/**************************************
 * settings functions and definitions *
 * ************************************/

void saveSettings() {
    if (saveEnabled) {
        EEPROM.update(1, mode);
        EEPROM.update(2, maxBrightness);
        EEPROM.update(3, (int) animationLocked);
    }
}

void loadSettings() {
    if (saveEnabled) {
        mode = EEPROM.read(1);
        maxBrightness = EEPROM.read(2);
        
        int lockData = EEPROM.read(3);
        if (lockData == 1) {
            animationLocked = true;
        } else {    
            animationLocked = false;
        }
    }
}

/**************************************
 * MQTT API functions and definitions *
 * ************************************/

void initMQTT() {
    if (mqttEnabled) {
        client.connect(mqttHost, mqttUsername, mqttPassword);   
        
        if (client.isConnected()) {
            if (lockFeed != "")
                client.subscribe(lockFeed);
            if (animationFeed != "")
                client.subscribe(animationFeed);
            if (powerFeed != "") {
                client.subscribe(powerFeed);
                client.publish(powerFeed, "ON");
            }
        }
    }
}

void MQTTcallback(char* topic, byte* payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    String message(p);
    debug("Received MQTT data (" + message + ") from " + topic);
}

void MQTTPublish(String feed, String payload) {
    if (mqttEnabled) {
        client.publish(feed, payload);
    }
}

/******************************************
 * Particle API functions and definitions *
 * ***************************************/

//initializes the shared variables and functions that are accessible through the particle API
void initParticleVariables() {
    if (particleApi) {
        Particle.variable("mode", &mode, INT);
        Particle.variable("maxBrightness", &maxBrightness, INT);
        Particle.variable("autocyleTime", &AutocyleTime, INT);
        
        Particle.function("setLock", (int (*)(String)) lockAnimation);
        Particle.function("setPower", (int (*)(String)) setPower);
        Particle.function("setAnimation", (int (*)(String)) setAnimation);
        Particle.function("setBrightness", (int (*)(String)) setBrightness);
        Particle.function("enableDebugMode", (int (*)(String)) enableDebugMode);
        Particle.function("getMicrophone", (int (*)(String)) getMicrophone);
    }
    
    if (particleFeeds) {
        Particle.subscribe(particleLockFeed, ParticleHandler);
        Particle.subscribe(particleAnimationFeed, ParticleHandler);
        Particle.subscribe(particlePowerFeed, ParticleHandler);
    }
    Particle.connect();
}

void ParticlePublish(String feed, String data) {
    if (particleFeeds) {
        Particle.publish(feed, data);
    }
}

void ParticleHandler(const char *event, const char *data) {
    if (event == particleLockFeed) {
        if (data == "ON") 
            animationLocked = true;
        if (data == "OFF")
            animationLocked = false;
    }
    
    if (event == particlePowerFeed) {
        if (data == "ON")
            isPowered = true;
        if (data == "OFF")
            isPowered = false;
    }
    
    if (event == particleAnimationFeed) {

    }
}

int lockAnimation(String _incoming) {
    String mqttpush = "";
    if (_incoming == "true") {
        mqttpush = "ON";
        animationLocked = true;
        debug("Enabling the animation lock");
    } 
    if (_incoming == "false") {
        mqttpush = "OFF";
        animationLocked = false;
        debug("Disabling the animation lock");
    }
    if (animationFeed != "" && mqttpush != "") { 
        MQTTPublish(lockFeed, mqttpush);
        ParticlePublish(particleLockFeed, mqttpush);
    }
    saveSettings();
    return (int) animationLocked;
}

int setPower(String _incoming) {
    String mqttpush = "";
    if (_incoming == "true") {
        mqttpush = "ON";
        isPowered = true;
        debug("Exiting sleep mode");
    }
    if (_incoming == "false") {
        mqttpush = "OFF";
        isPowered = false;
        debug("Entering sleep mode");
    }
    if (powerFeed != "" && mqttpush != "") {
        MQTTPublish(powerFeed, mqttpush);
        ParticlePublish(particlePowerFeed, mqttpush);
    }
    return (int) isPowered;
}

int setAnimation(String _incoming) {
    int id = _incoming.toInt();
    activeAnimation = id;
    client.publish(animationFeed, activeAnimation + "");
    ParticlePublish(particleAnimationFeed, activeAnimation + "");
    debug("Setting animation to " + activeAnimation);
    return activeAnimation;
}

int setBrightness(String _incoming) {
    int bright = _incoming.toInt();
    maxBrightness = bright;
    debug("Setting maxBrightness to " + activeAnimation);
    return maxBrightness;
}

int enableDebugMode(String _incoming) {
    if (_incoming == "true") {
        enablePrintStatements = true;
    }
    
    if (_incoming == "false") {
        enablePrintStatements = false;
    }
    return (int) enablePrintStatements;
}

int getMicrophone() {
    return (int) analogRead(MICROPHONE)-2048;
}

 /**********************************
 * Cube Functions  *
 * ********************************/

void fade() {
    Color pixelColor;
        for(int x=0;x<SIDE;x++)
            for(int y=0;y<SIDE;y++)
                for(int z=0;z<SIDE;z++)
                    {
                        pixelColor=getPixel(x,y,z);
                        if(pixelColor.red>0)
                            pixelColor.red--;
                        if(pixelColor.green>0)
                            pixelColor.green--;
                        if(pixelColor.blue>0)
                            pixelColor.blue--;
                        setPixel(x,y,z, pixelColor);    
                    }
}


//sets a pixel at position (x,y,z) to the col parameter's Color
void setPixel(int x, int y, int z, Color col) {
    cube.setVoxel(x, y, z, col);
}

void setBrightness(int brightness) {
    if (brightness > maxBrightness)
        brightness = maxBrightness;
    cube.maxBrightness = brightness;
}

//returns the Color value currently displayed at the x,y,z location
Color getPixel(int x, int y, int z) {
    Color pixelColor = cube.getVoxel(x, y, z);
    return pixelColor;
}

void background(Color col) {
    cube.background(col);
}

//returns a Color from a set of Colors fading from blue to green to red and back again
//the Color is returned based on where the parameter *val* falls between the parameters
//*min* and *maximum*.  If *val* is min, the function returns a blue Color.  If *val* is halfway
//between *min* and *maximum*, the function returns a yellow Color.  
Color ColorMap(float val, float min, float maximum)
{
  float range=1024;
  val=range*(val-min)/(maximum-min);
  Color Colors[6];
  Colors[0].red=0;
  Colors[0].green=0;
  Colors[0].blue=maxBrightness;
  
  Colors[1].red=0;
  Colors[1].green=maxBrightness;
  Colors[1].blue=maxBrightness;
  
  Colors[2].red=0;
  Colors[2].green=maxBrightness;
  Colors[2].blue=0;
  
  Colors[3].red=maxBrightness;
  Colors[3].green=maxBrightness;
  Colors[3].blue=0;
  
  Colors[4].red=maxBrightness;
  Colors[4].green=0;
  Colors[4].blue=0;

  Colors[5].red=maxBrightness;
  Colors[5].green=0;
  Colors[5].blue=maxBrightness;
  
  if (val<=range/6)
    return(lerpColor(Colors[0], Colors[1], val, 0, range/6));
  else if (val<=2*range/6)
    return(lerpColor(Colors[1], Colors[2], val, range/6, 2*range/6));
  else if (val<=3*range/6)
    return(lerpColor(Colors[2], Colors[3], val, 2*range/6, 3*range/6));
  else if (val<=4*range/6)
    return(lerpColor(Colors[3], Colors[4], val, 3*range/6, 4*range/6));
  else if (val<=5*range/6)
    return(lerpColor(Colors[4], Colors[5], val, 4*range/6, 5*range/6));
  else
    return(lerpColor(Colors[5], Colors[0], val, 5*range/6, range));
}


//returns a Color that's an interpolation between Colors a and b.  The Color
//is controlled by the position of val relative to min and maximum -- if val is equal to min,
//the resulting Color is identical to Color a.  If it's equal to maximum, the resulting Color 
//is identical to Color b.  If val is (maximum-min)/2, the resulting Color is the average of
//Color a and Color b
Color lerpColor(Color a, Color b, int val, int min, int maximum)
{
    Color lerped;
    lerped.red=a.red+(b.red-a.red)*(val-min)/(maximum-min);
    lerped.green=a.green+(b.green-a.green)*(val-min)/(maximum-min);
    lerped.blue=a.blue+(b.blue-a.blue)*(val-min)/(maximum-min);
    return lerped;
}


/****************************************
 * flip functions *
 * **************************************/
 
 void checkFlipState()
 {
    updateAccelerometer();
    if(accelerometer[0]>FACEPLANT)  //if the cube is upside-down, set the upside-down flag and mark the time when it was flipped
    {
        lastFaceplant=millis();
    }
    if(accelerometer[1]<LEFT_SIDE)  //if the cube is flipped to either side
    {
        lastLeft=millis();
    }
    if(accelerometer[1]>RIGHT_SIDE)
    {
        lastRight=millis();
    }

    if(accelerometer[2]>RIGHTSIDE_UP)
    {
        if(((millis()-lastFaceplant)<FLIP_TIMEOUT)&&(millis()-lastFaceplant>FLIP_DEBOUNCE))
        {
            animationLocked=false;
            lastFaceplant=millis()-FLIP_TIMEOUT;
            Color flash;
            flash.red=maxBrightness;
            flash.green=maxBrightness;
            flash.blue=maxBrightness;
            background(flash);
        }
        if(((millis()-lastLeft)<FLIP_TIMEOUT)&&(millis()-lastChange>FLIP_DEBOUNCE))
        {
            animationLocked=false;
            lastChange=millis();
            decrementAnimation();
            lastLeft=millis()-FLIP_TIMEOUT;
        }
        if(((millis()-lastRight)<FLIP_TIMEOUT)&&(millis()-lastChange>FLIP_DEBOUNCE))
        {
            animationLocked=false;
            lastChange=millis();
            incrementAnimation();
            lastRight=millis()-FLIP_TIMEOUT;
        }
    }
 }
 
void updateAccelerometer()
{
    for(int i=0;i<3;i++)
        accelerometer[i]=analogRead(X+i);
}

void setFadeSpeed()
{
    if(animationLocked)
        fadeSpeed=2;
    else
        fadeSpeed=20;
}
 
 void incrementAnimation()
 {
    activeAnimation++;
    setFadeSpeed();
    fading=true;
    if(activeAnimation>=ROUTINES)
        activeAnimation=0;
    client.publish(animationFeed, activeAnimation + "");
 }
 
  void decrementAnimation()
 {
    activeAnimation--;
    setFadeSpeed();
    fading=true;
    if(activeAnimation<0)
        activeAnimation=ROUTINES-1;
    client.publish(animationFeed, activeAnimation + "");
 }

/***************************************
 * fireworks functions *
 * ***********************************/
 
void updateFireworks() {
//loop through all the pixels, calculate the distance to the center Point, and turn the pixel on if it's at the right radius
  for(int x=0;x<SIDE;x++)
    for(int y=0;y<SIDE;y++) 
        for(int z=0;z<SIDE;z++)
        {
            if(showRocket)
                if(abs(distance(x,y,z,rocketX, rocketY, rocketZ)-radius)<0.05)
                     setPixel(x,y,z, rocketColor);                
            if(exploded)
                if(abs(distance(x,y,z,centerX, centerY, centerZ)-radius)<0.1)
                  setPixel(x,y,z, fireworkColor);
        }

        if(exploded)
            radius+=speed;  //the sphere gets bigger
        if(showRocket)
        {
            rocketX+=xInc;
            rocketY+=yInc;
            rocketZ+=zInc;
        }
        //if our sphere gets too large, restart the animation in another random spot
        if(radius>maximumSize)
            prepRocket();
        if(abs(distance(centerX,centerY,centerZ,rocketX, rocketY, rocketZ)-radius)<2)
            {
                showRocket=false;
                exploded=true;
            }
        
}

float distance(float x, float y, float z, float x1, float y1, float z1) {
    return(sqrt(pow(x-x1,2)+pow(y-y1,2)+pow(z-z1,2)));
}

void prepRocket() {
    radius=0;
    centerX=rand()%8;
    centerY=rand()%8;
    centerZ=rand()%8;
    fireworkColor.red=rand()%brightness;
    fireworkColor.green=rand()%brightness;
    fireworkColor.blue=rand()%brightness;
    launchX=rand()%8;
    launchZ=rand()%8;
    rocketX=launchX;
    rocketY=0;
    rocketZ=launchZ;
    launchTime=15+rand()%25;
    xInc=(centerX-rocketX)/launchTime;
    yInc=(centerY-rocketY)/launchTime;
    zInc=(centerZ-rocketZ)/launchTime;
    showRocket=true;
    exploded=false;
    speed=0.15;
    maximumSize=2+rand()%6;
}

void initFireworks() {
    rocketColor.red=255;
    rocketColor.green=150;
    rocketColor.blue=100;
    prepRocket();
}

/********************************
 * zplasma functions *
 * *****************************/
 
 void zPlasma() {
      
    phase += phaseIncrement;
    // The two Points move along Lissajious curves, see: http://en.wikipedia.org/wiki/Lissajous_curve
    // We want values that fit the LED grid: x values between 0..8, y values between 0..8, z values between 0...8
    // The sin() function returns values in the range of -1.0..1.0, so scale these to our desired ranges.
    // The phase value is multiplied by various constants; I chose these semi-randomly, to produce a nice motion.
    Point p1 = { (sin(phase*1.000)+1.0) * 4, (sin(phase*1.310)+1.0) * 4.0,  (sin(phase*1.380)+1.0) * 4.0};
    Point p2 = { (sin(phase*1.770)+1.0) * 4, (sin(phase*2.865)+1.0) * 4.0,  (sin(phase*1.410)+1.0) * 4.0};
    Point p3 = { (sin(phase*0.250)+1.0) * 4, (sin(phase*0.750)+1.0) * 4.0,  (sin(phase*0.380)+1.0) * 4.0};
    
    byte row, col, dep;
    
    // For each row
    for(row=0; row<SIDE; row++) {
        float row_f = float(row); // Optimization: Keep a floating Point value of the row number, instead of recasting it repeatedly.
        
        // For each column
        for(col=0; col<SIDE; col++) {
            float col_f = float(col); // Optimization.
            
            // For each depth
            for(dep=0; dep<SIDE; dep++) {
                float dep_f = float(dep); // Optimization.
                
                // Calculate the distance between this LED, and p1.
                Point dist1 = { col_f - p1.x, row_f - p1.y,  dep_f - p1.z }; // The vector from p1 to this LED.
                float distance1 = sqrt( dist1.x*dist1.x + dist1.y*dist1.y + dist1.z*dist1.z);
                
                // Calculate the distance between this LED, and p2.
                Point dist2 = { col_f - p2.x, row_f - p2.y,  dep_f - p2.z}; // The vector from p2 to this LED.
                float distance2 = sqrt( dist2.x*dist2.x + dist2.y*dist2.y + dist2.z*dist2.z);
                
                // Calculate the distance between this LED, and p3.
                Point dist3 = { col_f - p3.x, row_f - p3.y,  dep_f - p3.z}; // The vector from p3 to this LED.
                float distance3 = sqrt( dist3.x*dist3.x + dist3.y*dist3.y + dist3.z*dist3.z);
                
                // Warp the distance with a sin() function. As the distance value increases, the LEDs will get light,dark,light,dark,etc...
                // You can use a cos() for slightly different shading, or experiment with other functions.
                float Color_1 = distance1; // range: 0.0...1.0
                float Color_2 = distance2;
                float Color_3 = distance3;
                float Color_4 = (sin( distance1 * distance2 * ColorStretch )) + 2.0 * 0.5;
                // Square the Color_f value to weight it towards 0. The image will be darker and have higher contrast.
                Color_1 *= Color_1 * Color_4;
                Color_2 *= Color_2 * Color_4;
                Color_3 *= Color_3 * Color_4;
                Color_4 *= Color_4;
                // Scale the Color up to 0..7 . Max brightness is 7.
                //strip.setPixelColor(col + (8 * row), strip.Color(Color_4, 0, 0) );
                plasmaColor.red=Color_1*plasmaBrightness;
                plasmaColor.green=Color_2*plasmaBrightness;
                plasmaColor.blue=Color_3*plasmaBrightness;
                
                setPixel(row,col,dep,plasmaColor);       
            }
        }
    }
}

/********************************
 * Squarral functions *
 * *****************************/

void initSquarral() 
{
  position={0,0,0};
  increment={1,0,0};
}

void squarral() 
{
    add(position, increment);
    if((increment.x==1)&&(position.x==SIDE-1-bound))
        increment={0,1,0};
    if((increment.x==-1)&&(position.x==bound))
        increment={0,-1,0};
    if((increment.y==1)&&(position.y==SIDE-1-bound))
        increment={-1,0,0};
    if((increment.y==-1)&&(position.y==bound))
    {
        increment={1,0,0};
        position.z+=squarral_zInc;
        bound+=boundInc;
        if((position.z==3)&&(squarral_zInc>0))
          boundInc=0;
        if((position.z==4)&&(squarral_zInc>0))
          boundInc=-1;
        if((position.z==3)&&(squarral_zInc<0))
          boundInc=-1;
        if((position.z==4)&&(squarral_zInc<0))
          boundInc=0;
        
        if((position.z==0)||(position.z==SIDE-1))
            boundInc*=-1;
            
        if((position.z==SIDE-1)||(position.z==0))
        {
            squarral_zInc*=-1;
            if(squarral_zInc==1)
            {
                axis=rand()%6;
                if(rand()%5==0)
                    rainbow=true;
                else
                    rainbow=false;
            }
        }
    }
    
    posX=position.x;
    posY=position.y;
    posZ=position.z;
    
    incX=increment.x;
    incY=increment.y;
    incZ=increment.z;
    
    for(int i=TRAIL_LENGTH-1;i>0;i--)
    {
        trailPoints[i].x=trailPoints[i-1].x;
        trailPoints[i].y=trailPoints[i-1].y;
        trailPoints[i].z=trailPoints[i-1].z;
    }
    trailPoints[0].x=pixel.x;
    trailPoints[0].y=pixel.y;
    trailPoints[0].z=pixel.z;
    switch(axis){
        case(0):
            pixel.x=position.x;
            pixel.y=position.y;
            pixel.z=position.z;
            break;
        case(1):
            pixel.x=position.z;
            pixel.y=position.x;
            pixel.z=position.y;
            break;
        case(2):
            pixel.x=position.y;
            pixel.y=position.z;
            pixel.z=position.x;
            break;
        case(3):
            pixel.x=position.z;
            pixel.y=SIDE-1-position.x;
            pixel.z=position.y;
            break;
        case(4):
            pixel.x=position.y;
            pixel.y=position.z;
            pixel.z=SIDE-1-position.x;
            break;
        case(5):
            pixel.x=position.x;
            pixel.y=SIDE-1-position.y;
            pixel.z=position.z;
            break;
    }
        
    pixelColor=ColorMap(frame%1000,0,1000);
    setPixel((int)pixel.x, (int)pixel.y, (int)pixel.z, pixelColor);
    for(int i=0;i<TRAIL_LENGTH;i++)
    {
        Color trailColor;
        if(rainbow)
        {
            trailColor=ColorMap((frame+(i*1000/TRAIL_LENGTH))%1000,0,1000);
            //fade the trail to black over the length of the trail
            trailColor.red=trailColor.red*(TRAIL_LENGTH-i)/TRAIL_LENGTH;
            trailColor.green=trailColor.green*(TRAIL_LENGTH-i)/TRAIL_LENGTH;
            trailColor.blue=trailColor.blue*(TRAIL_LENGTH-i)/TRAIL_LENGTH;
        }
        else
        {
            trailColor.red=pixelColor.red*(TRAIL_LENGTH-i)/TRAIL_LENGTH;
            trailColor.green=pixelColor.green*(TRAIL_LENGTH-i)/TRAIL_LENGTH;
            trailColor.blue=pixelColor.blue*(TRAIL_LENGTH-i)/TRAIL_LENGTH;
        }
        setPixel((int)trailPoints[i].x, (int)trailPoints[i].y, (int)trailPoints[i].z, trailColor);
    }
    frame++;
}

void add(Point& a, Point& b)
{
    a.x+=b.x;
    a.y+=b.y;
    a.z+=b.z;
}

/***************************************
 * snake functions *
 * ***********************************/

double distance(voxel v, voxel w) {
    return sqrt(
      pow((v.x - w.x), 2) +
      pow((v.y - w.y), 2) +
      pow((v.z - w.z), 2)
    );
};

bool containsVoxel(std::vector<voxel> &vectorList, voxel &voxel) {
  for (auto it = vectorList.begin(); it != vectorList.end(); ++it) {
    if (*it == voxel) {
      return true;
    }
  }
  return false;
}

bool canMove(voxel* direction) {
  voxel next = snake[0] + *direction;
  return (next.x >= 0 && next.y >= 0 && next.z >= 0 && next.x <= 7 && next.y <= 7 && next.z <= 7 && !containsVoxel(snake, next));
}

void addTreat() {
  voxel treat;
  while (true) {
    treat = { random(0, 7), random(0, 7), random(0, 7) };
    if (containsVoxel(snake, treat) || containsVoxel(treats, treat)) {
      continue;
    } else {
      treats.push_back(treat);
      break;
    }
  }
}


void updateDirection() {
  // Mostly reuse the same direction but sometimes turn at random
  if (canMove(snakeDirection) && random(0, 100) < 80) {
    return;
  }

  std::vector<voxel*> allowedDirections;
  for(auto it = possibleDirections.begin(); it != possibleDirections.end(); ++it) {
    if (canMove(&(*it))) {
      allowedDirections.push_back(&(*it));
    }
  }

  if (allowedDirections.size() == 0) {
    snakeDirection = NULL;
    return;
  }

  double leastDistance = 65536.0;
  double tdistance;
  voxel next;
  for(auto it = allowedDirections.begin(); it != allowedDirections.end(); ++it) {
    next = snake[0] + **it;
    tdistance = distance(next, treats[0]);
    if (tdistance < leastDistance) {
      leastDistance = tdistance;
      snakeDirection = *it;
    }
  }
}

void resetSnake() {
  background(black);
  snakeDirection = &possibleDirections[0];
  initialSnakeLength = 5;
  snake.clear();
  snake.emplace_back(0, 0, 0);
  snakeSpeed = 1;
  deathFrame = 0;
  snakeFrameCount = 0;
}

void moveSnake() {
  updateDirection();
  if (snakeDirection == NULL) {
    deathFrame = snakeFrameCount;
    return;
  }

  bool grow = (snake.size() < initialSnakeLength);
  voxel front = snake[0] + *snakeDirection;
  for(auto it = treats.begin(); it != treats.end(); ++it) {
    if (*it == front) {
      treats.erase(it);
      grow = true;
      break;
    }
  }

  snake.insert(snake.begin(), front);
  if(!grow) {
    snake.pop_back();
  } 

  if (!treats.size()) {
    addTreat();
  }
}

void runSnake() {
  delay(30);
  Color segmentColor;
  snakeFrameCount++;

  if (!deathFrame) {
    moveSnake();
  } else {
    if (snakeFrameCount - deathFrame > 100) {
      resetSnake();
    }
  }

  background(black);
  for(auto it = snake.begin(); it != snake.end(); ++it) {
    if (deathFrame && snakeFrameCount % 16 < 8) {
      segmentColor = Color(255, 255, 255);
    } else if (deathFrame > 0) {
      segmentColor = Color(255, 0, 0);
    } else {
      segmentColor = Color((it->x+1)*255/8, (it->y+1)*255/8, (it->z+1)*255/8);
    }
    cube.setVoxel(it->x, it->y, it->z, segmentColor);
  }

  if (!deathFrame) {
    for(auto it = treats.begin(); it != treats.end(); ++it) {
      cube.setVoxel(it->x, it->y, it->z, Color(150, 255, 0));
    }
  }
}

void initSnake() {
    resetSnake();    
}
