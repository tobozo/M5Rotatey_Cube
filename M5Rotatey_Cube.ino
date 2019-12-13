
#include <M5Stack.h> // https://github.com/tobozo/ESP32-Chimera-Core
#include <M5StackUpdater.h> // https://github.com/tobozo/M5Stack-SD-Updater
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#define processing_out false
#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

#define tft M5.Lcd

MPU9250 IMU;
TFT_eSprite sprite = TFT_eSprite( &tft );


#define DEBUG false


float MPIDEG = 360 / 2.0 / PI;

double offsetX = 1, offsetY = 1, offsetZ = 1;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float lastAngleX, lastAngleY, lastAngleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float /*gx, gy, gz,*/ dpsX, dpsY, dpsZ;

// Accel and gyro data

const double halfC = M_PI / 180;
const float DEG2RAD = 180 / PI;

// Overall scale and perspective distance
uint8_t sZ = 4, scale = 48, scaleMax = 48;
float sphereSize = 6.0;
// screen center coordinates (calculated from screen dimensions)
uint16_t spriteWidth = 236;
uint16_t spriteHeight = 236;
uint8_t centerX = spriteWidth/2;
uint8_t centerY = spriteHeight/2;

typedef struct {
    double x;
    double y;
    double z;
} Coord3DSet;

typedef struct {
    double x;
    double y;
} Coord2DSet;

typedef struct {
    uint16_t id1;
    uint16_t id2;
} Lines;  


/* https://codepen.io/ge1doot/pen/grWrLe */

static Coord3DSet CubePoints3DArray[] = {
  {  1,  1,  1 },
  {  1,  1, -1 },
  {  1, -1,  1 },
  {  1, -1, -1 },
  { -1,  1,  1 },
  { -1,  1, -1 },
  { -1, -1,  1 },
  { -1, -1, -1 },

  {  1,  1,  0 },
  {  1,  0,  1 },
  {  0,  1,  1 },

  {  -1,  1,  0 },
  {  -1,  0,  1 },
  {  0,  -1,  1 },

  {  1,  -1,  0 },
  {  1,  0,  -1 },
  {  0,  1,  -1 },

  {  -1,  -1,  0 },
  {  -1,  0,  -1 },
  {  0,  -1,  -1 },

  //{0, 0, 0}
  
};

static Coord3DSet CubePoints2DArray[] = {
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },

  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  { 0,0 },
  
  { 0,0 },
  { 0,0 },
  { 0,0 },

  //{ 0,0 }
};

static Lines LinesArray[] = {
  { 0, 1 },
  { 0, 2 },
  { 0, 4 },
  { 1, 3 },
  { 1, 5 },
  { 2, 3 },
  { 2, 6 },
  { 3, 7 },
  { 4, 5 },
  { 4, 6 },
  { 5, 7 },
  { 6, 7 }
/*
  { 1, 4 },
  { 2, 3 },
  { 1, 6 },
  { 2, 5 },
  { 2, 8 },
  { 6, 4 },
  { 4, 7 },
  { 3, 8 },
  { 1, 7 },
  { 3, 5 },
  { 5, 8 },
  { 7, 6 }
 */
  
};

// used for sorting points by depth
uint16_t zsortedpoints[21] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

uint16_t totalpoints = sizeof(CubePoints3DArray) / sizeof(CubePoints3DArray[0]);
uint16_t totallines = sizeof(LinesArray) / sizeof(LinesArray[0]);


// Calculate angle from accel/gyro
void calcRotation() {

  bool imuChanged = false;
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();
    IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres(); // 1 / 2^17
    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;
    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();

    IMU.magbias[0] = +470.; // User environmental x-axis correction in milliGauss TODO axis??
    IMU.magbias[1] = +120.; // User environmental x-axis correction in milliGauss
    IMU.magbias[2] = +125.;

    IMU.mx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0] -
              IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1] -
              IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2] -
              IMU.magbias[2];
    imuChanged = true;
    IMU.updateTime();
    MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx*DEG_TO_RAD,
                          IMU.gy*DEG_TO_RAD, IMU.gz*DEG_TO_RAD, IMU.my,
                          IMU.mx, IMU.mz, IMU.deltat);
  } else {
    return;
  }

  // Calculate the elapsed time from the last calculation
  interval = millis() - preInterval;
  preInterval = millis();
  // Calculate angle from acceleration sensor
  acc_angle_y = atan2(IMU.ax, IMU.az + abs(IMU.ay)) * -DEG2RAD;
  acc_angle_x = atan2(IMU.ay, IMU.az + abs(IMU.ax)) * DEG2RAD;
  // numerical integral
  gyro_angle_x += (IMU.gx + offsetX) * (interval * 0.001);
  gyro_angle_y += (IMU.gy + offsetY) * (interval * 0.001);
  gyro_angle_z += (IMU.gz + offsetZ) * (interval * 0.001);
  // complementary filter
  angleX = (0.996 * gyro_angle_x) + (0.004 * acc_angle_x);
  angleY = (0.996 * gyro_angle_y) + (0.004 * acc_angle_y);
  angleZ = gyro_angle_z;
  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;
  //Serial.printf("[%10.4f, %10.4f]\n", acc_angle_x, acc_angle_y);
}




void setup() {

  M5.begin();
  Wire.begin();
  M5.ScreenShot.init( &tft, M5STACK_SD );
  M5.ScreenShot.begin();
  // build has buttons => enable SD Updater at boot
  if(digitalRead(BUTTON_A_PIN) == 0) {
    Serial.println("Will Load menu binary");
    updateFromFS();
    ESP.restart();
  }


  // Read the WHO_AM_I register, this is a good test of communication
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
  Serial.println("MPU9250 is online...");
  // Start by performing self test and reporting values
  IMU.MPU9250SelfTest(IMU.SelfTest);
  // Calibrate gyro and accelerometers, load biases in bias registers
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  Serial.println("MPU9250 initialized for active data mode....");
  // Read the WHO_AM_I register of the magnetometer, this is a good communication test
  byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
  Serial.print(" I should be "); Serial.println(0x48, HEX);
  // Get magnetometer calibration from AK8963 ROM
  IMU.initAK8963(IMU.magCalibration);

  // store initial position
  calcRotation(); // read from MPU
  lastAngleX = angleX;
  lastAngleY = angleY;
  lastAngleZ = angleZ;

  Serial.println("Starting screen");
  if( psramInit() ) {
    sprite.setPsram( false );
  }
  sprite.setColorDepth( 16 );
  sprite.setPsram( false );
  sprite.setTextSize(1);
  sprite.setTextColor(GREEN ,BLACK);
  sprite.createSprite( spriteWidth, spriteHeight );

}

void loop() {
  calcRotation();
  cubeloop(); 
}



void cubeloop() {

  float diffAngleX, diffAngleY, diffAngleZ;

  diffAngleX = lastAngleX - angleX;
  diffAngleY = lastAngleY - angleY;
  diffAngleZ = lastAngleZ - angleZ;

  vectorRotateXYZ((double)(diffAngleY+0.1)*halfC, 1); // X
  vectorRotateXYZ((double)(diffAngleX+0.1)*halfC, 2); // Y
  vectorRotateXYZ((double)diffAngleZ*halfC, 3); // Z

  zSortPoints();
  sprite.fillSprite( TFT_BLACK );
  //meshPlot();
  spherePlot();
  fps(1);
  msOverlay();
  sprite.pushSprite( tft.width()/2 - spriteWidth/2, tft.height()/2 - spriteHeight/2 );
  M5.update();
  if( M5.BtnB.wasPressed() ) {
    M5.ScreenShot.snap(); 
  }

  lastAngleX = angleX;
  lastAngleY = angleY;
  lastAngleZ = angleZ;

}
void vectorRotateXYZ(double angle, int axe) {
  int8_t m1; // coords polarity
  uint8_t i1, i2; // coords index
  double t1, t2;
  uint16_t i;
  for( i=0; i<totalpoints; i++ ) {
    switch(axe) {
      case 2: // X
        m1 = -1;
        t1 = CubePoints3DArray[i].y;
        t2 = CubePoints3DArray[i].z;
        CubePoints3DArray[i].y = t1*cos(angle)+(m1*t2)*sin(angle);
        CubePoints3DArray[i].z = (-m1*t1)*sin(angle)+t2*cos(angle);
      break;
      case 1: // Y
        m1 = -1;
        t1 = CubePoints3DArray[i].x;
        t2 = CubePoints3DArray[i].z;
        CubePoints3DArray[i].x = t1*cos(angle)+(m1*t2)*sin(angle);
        CubePoints3DArray[i].z = (-m1*t1)*sin(angle)+t2*cos(angle);
      break;
      case 3: // Z
        m1 = 1;
        t1 = CubePoints3DArray[i].x;
        t2 = CubePoints3DArray[i].y;
        CubePoints3DArray[i].x = t1*cos(angle)+(m1*t2)*sin(angle);
        CubePoints3DArray[i].y = (-m1*t1)*sin(angle)+t2*cos(angle);
      break;
    }
  }
}

/* sort xyz by z depth */
void zSortPoints() {
  bool swapped;
  uint16_t temp;
  float radius, nextradius;
  do {
    swapped = false;
    for(uint16_t i=0; i!=totalpoints-1; i++ ) {
      radius     = (-CubePoints3DArray[zsortedpoints[i]].z+3)*2;
      nextradius = (-CubePoints3DArray[zsortedpoints[i+1]].z+3)*2;
      if (radius > nextradius) {
        temp = zsortedpoints[i];
        zsortedpoints[i] = zsortedpoints[i + 1];
        zsortedpoints[i + 1] = temp;
        swapped = true;
      }
    }
  } while (swapped);
}



uint16_t luminance(uint16_t color, uint8_t luminance) {
  // Extract rgb colours and stretch range to 0 - 255
  uint16_t r = (color & 0xF800) >> 8; r |= (r >> 5);
  uint16_t g = (color & 0x07E0) >> 3; g |= (g >> 6);
  uint16_t b = (color & 0x001F) << 3; b |= (b >> 5);

  b = ((b * (uint16_t)luminance + 255) >> 8) & 0x00F8;
  g = ((g * (uint16_t)luminance + 255) >> 8) & 0x00FC;
  r = ((r * (uint16_t)luminance + 255) >> 8) & 0x00F8;

  return (r << 8) | (g << 3) | (b >> 3);
}


void drawSphere( uint16_t x, uint16_t y, uint16_t radius, uint16_t color ) {
  int16_t _radius = radius;
  int halfradius = radius / 2;
  sprite.drawCircle( x, y, radius+1, TFT_BLACK);
  sprite.fillCircle( x, y, radius, color);
  while( _radius > 0 ) {
    int gap = (radius - _radius)/2;
    byte lumval = map( _radius, 0, radius, 255, 64 );
    uint16_t translatedColor = luminance( color, lumval );
    sprite.fillCircle( x+gap, y-gap, _radius, translatedColor);
    _radius--;
  }
}

/* draw scaled spheres from background to foreground */
void spherePlot() {
  uint16_t i;
  int radius, halfradius;
  int transid;
  for( i=0; i<totalpoints; i++ ) {
    transid = zsortedpoints[i];
    CubePoints2DArray[transid].x = centerX + scale/(1+CubePoints3DArray[transid].z/sZ)*CubePoints3DArray[transid].x; 
    CubePoints2DArray[transid].y = centerY + scale/(1+CubePoints3DArray[transid].z/sZ)*CubePoints3DArray[transid].y;
    radius = (-CubePoints3DArray[transid].z+3)* sphereSize;
    byte depthlumval = map( int(CubePoints3DArray[transid].z)*128, -128, 128, 255, 64 );
    uint16_t depthColor = luminance( TFT_YELLOW, depthlumval );
    drawSphere( CubePoints2DArray[transid].x, CubePoints2DArray[transid].y, radius, depthColor );
  }
}

/* draw lines between given pairs of points */
void meshPlot() {
  uint16_t i;
  uint16_t id1, id2;
  for( i=0; i<totallines; i++ ) {
    id1 = LinesArray[i].id1;
    id2 = LinesArray[i].id2;
    sprite.drawLine(CubePoints2DArray[id1].x, CubePoints2DArray[id1].y, CubePoints2DArray[id2].x, CubePoints2DArray[id2].y, TFT_WHITE);
  }
}

unsigned int fpsall = 30;

static inline void fps(const int seconds){
  // Create static variables so that the code and variables can
  // all be declared inside a function
  static unsigned long lastMillis;
  static unsigned long frameCount;
  static unsigned int framesPerSecond;
  
  // It is best if we declare millis() only once
  unsigned long now = millis();
  frameCount ++;
  if (now - lastMillis >= seconds * 1000) {
    framesPerSecond = frameCount / seconds;
    //Serial.println(framesPerSecond);
    fpsall = framesPerSecond;
    frameCount = 0;
    lastMillis = now;
  }
}

void msOverlay() {
  //tft.setTextAlignment(TEXT_ALIGN_RIGHT);
  //tft.setFont(ArialMT_Plain_10);
  sprite.setTextColor( WHITE );
  sprite.drawString( String( String(fpsall)+"fps" ), 0, 0 );
  sprite.setTextColor(GREEN, GREEN);
  sprite.setCursor(10, 0); 
  sprite.print("     x      y      z ");
  sprite.setCursor(10,  12);
  sprite.printf("% 6d % 6d % 6d mg   \r\n",  (int)(1000*IMU.ax), (int)(1000*IMU.ay), (int)(1000*IMU.az));
  sprite.setCursor(10,  24);
  sprite.printf("% 6d % 6d % 6d o/s  \r\n", (int)(IMU.gx), (int)(IMU.gy), (int)(IMU.gz));
}


