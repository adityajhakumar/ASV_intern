#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>
#include <EEPROM.h>
#include <utility/imumaths.h>

#define EEPROM_SIZE      512
#define CALIB_FLAG_ADDR   0
#define OFFSETS_ADDR      1
#define GPS_RX_PIN       16
#define GPS_TX_PIN       17
#define GPS_BAUD        9600

const float dt = 0.20f;                 // Time step in seconds
const float R_gps = 6.0f;               // GPS measurement noise variance
const float Q_base = 0.05f;             // Process noise variance
const float velocityDamp = 0.90f;       // Velocity damping factor
const float accelThreshold = 0.1f;      // Acceleration magnitude threshold to detect motion
const float speedThreshold = 0.5f;      // GPS speed threshold to detect motion (m/s)
const unsigned long gpsTimeout = 3000;  // Timeout to declare GPS lost (ms)
const double EARTH_RADIUS_M = 6371000.0;
const double DEG2RAD = 3.14159265358979323846 / 180.0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);

float X[4] = { 0.0f, 0.0f, 0.0f, 0.0f };  // State: posX, posY, velX, velY

float P[4][4] = {
  { 10.0f, 0, 0, 0 },
  { 0, 10.0f, 0, 0 },
  { 0, 0, 1.0f, 0 },
  { 0, 0, 0, 1.0f }
};

double originLat = 0.0;
double originLon = 0.0;
bool originSet = false;
bool gpsAvailable = false;
unsigned long lastGPSFixTime = 0;

void loadCalibration();
void saveCalibration();
void printCalibrationStatus();
void convertLatLonToXY(double lat, double lon, float &outX, float &outY);
void convertXYToLatLon(float x, float y, double &outLat, double &outLon);
void kalmanPredict(float accelX, float accelY);
void kalmanUpdate(float measX, float measY);
float distanceBetween(float x1, float y1, float x2, float y2);

void setup() {
  Serial.begin(115200);
  delay(100);
  EEPROM.begin(EEPROM_SIZE);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
    while (1) delay(10);
  }

  delay(50);
  bno.setExtCrystalUse(true);
  loadCalibration();

  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  lastGPSFixTime = millis();
  gpsAvailable = false;
}

void loop() {
  // Read GPS data
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  printCalibrationStatus();

  static bool offsetsSaved = false;
  uint8_t sysCal, gyroCal, accelCal, magCal;
  bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);

  if (!offsetsSaved && sysCal == 3 && gyroCal == 3 && accelCal == 3 && magCal == 3) {
    saveCalibration();
    offsetsSaved = true;
  }

  bool newGPSFix = false;
  double lat = 0.0, lon = 0.0;
  double gpsSpeed = 0.0;

  // Check if new GPS fix is available
  if (gps.location.isUpdated() && gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    gpsSpeed = gps.speed.mps();
    newGPSFix = true;
    gpsAvailable = true;
    lastGPSFixTime = millis();

    if (!originSet) {
      originLat = lat;
      originLon = lon;
      originSet = true;
      Serial.println("Origin set.");
    }
  }

  imu::Vector<3> linearAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float ax = linearAcc.x();
  float ay = linearAcc.y();
  float accelMag = sqrt(ax * ax + ay * ay);

  bool isMoving = (accelMag > accelThreshold) || (newGPSFix && gpsSpeed > speedThreshold);

  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> accelEarth = quat.rotateVector(linearAcc);
  float ax_e = accelEarth.x();
  float ay_e = accelEarth.y();

  if (isMoving) {
    kalmanPredict(ax_e, ay_e);
  } else {
    // No movement, just damp velocity and propagate position
    X[0] += X[2] * dt;
    X[1] += X[3] * dt;
    X[2] *= velocityDamp;
    X[3] *= velocityDamp;

    // Increase uncertainty due to process noise
    for (int i = 0; i < 4; i++) {
      P[i][i] += Q_base;
    }
  }

  float estX = X[0];
  float estY = X[1];

  if (newGPSFix && originSet && isMoving) {
    float measX, measY;
    convertLatLonToXY(lat, lon, measX, measY);
    kalmanUpdate(measX, measY);
  }

  // If GPS hasn't updated for a while, mark unavailable
  if ((millis() - lastGPSFixTime) > gpsTimeout) {
    gpsAvailable = false;
  }

  if (originSet) {
    // Convert fused state position to lat/lon
    double fusedLat, fusedLon;
    convertXYToLatLon(X[0], X[1], fusedLat, fusedLon);

    // Convert estimated position (IMU only) assuming last known without GPS update
    // Here, estimated = fused state before kalmanUpdate, so we use estX/Y saved before update
    double estLat, estLon;
    convertXYToLatLon(estX, estY, estLat, estLon);

    if (gpsAvailable) {
      // Calculate position error in meters
      float posError = distanceBetween(X[0], X[1], estX, estY);

      Serial.println("--- Position ---");
      Serial.print("Fused (GPS+IMU): ");
      Serial.print(fusedLat, 6);
      Serial.print(", ");
      Serial.println(fusedLon, 6);

      Serial.print("Estimated (IMU only): ");
      Serial.print(estLat, 6);
      Serial.print(", ");
      Serial.println(estLon, 6);

      Serial.print("Position Error (meters): ");
      Serial.println(posError, 3);
      Serial.println();
    } else {
      Serial.print("Estimated (IMU only, GPS lost): ");
      Serial.print(estLat, 6);
      Serial.print(", ");
      Serial.println(estLon, 6);
      Serial.println();
    }
  }

  delay((int)(dt * 1000));
}

void loadCalibration() {
  byte flag = EEPROM.read(CALIB_FLAG_ADDR);
  if (flag == 0x55) {
    adafruit_bno055_offsets_t storedOffsets;
    EEPROM.get(OFFSETS_ADDR, storedOffsets);
    bno.setSensorOffsets(storedOffsets);
    Serial.println("Calibration loaded from EEPROM.");
  } else {
    Serial.println("No calibration data in EEPROM.");
  }
}

void saveCalibration() {
  adafruit_bno055_offsets_t saveOffsets;
  bno.getSensorOffsets(saveOffsets);
  EEPROM.put(OFFSETS_ADDR, saveOffsets);
  EEPROM.write(CALIB_FLAG_ADDR, 0x55);
  EEPROM.commit();
  Serial.println("Calibration saved to EEPROM.");
}

void printCalibrationStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print("Calib: ");
  Serial.print(sys);
  Serial.print(",");
  Serial.print(gyro);
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.println(mag);
}

void convertLatLonToXY(double lat, double lon, float &outX, float &outY) {
  if (!originSet) { outX = 0; outY = 0; return; }
  double dLat = (lat - originLat) * DEG2RAD;
  double dLon = (lon - originLon) * DEG2RAD;
  double cosLat0 = cos(originLat * DEG2RAD);
  outY = (float)(dLat * EARTH_RADIUS_M);
  outX = (float)(dLon * EARTH_RADIUS_M * cosLat0);
}

void convertXYToLatLon(float x, float y, double &outLat, double &outLon) {
  if (!originSet) { outLat = 0; outLon = 0; return; }
  double cosLat0 = cos(originLat * DEG2RAD);
  outLat = originLat + (y / EARTH_RADIUS_M) * (180.0 / M_PI);
  outLon = originLon + (x / (EARTH_RADIUS_M * cosLat0)) * (180.0 / M_PI);
}

void kalmanPredict(float accelX, float accelY) {
  // State prediction: pos + vel*dt + 0.5*accel*dt^2; vel + accel*dt
  X[0] += X[2] * dt + 0.5f * accelX * dt * dt;
  X[1] += X[3] * dt + 0.5f * accelY * dt * dt;
  X[2] = velocityDamp * (X[2] + accelX * dt);
  X[3] = velocityDamp * (X[3] + accelY * dt);

  // Increase covariance uncertainty with process noise Q
  for (int i = 0; i < 4; i++) {
    P[i][i] += Q_base;
  }
}

void kalmanUpdate(float zX, float zY) {
  // Innovation/residual
  float residX = zX - X[0];
  float residY = zY - X[1];

  // Calculate Kalman gains for position states
  float Kx = P[0][0] / (P[0][0] + R_gps);
  float Ky = P[1][1] / (P[1][1] + R_gps);

  // Update states with measurement
  X[0] += Kx * residX;
  X[1] += Ky * residY;

  // Update covariance matrix diagonals for positions
  P[0][0] *= (1.0f - Kx);
  P[1][1] *= (1.0f - Ky);

  // Optionally, you can update velocity covariances as well if needed
}

float distanceBetween(float x1, float y1, float x2, float y2) {
  float dx = x1 - x2;
  float dy = y1 - y2;
  return sqrt(dx * dx + dy * dy);
}
