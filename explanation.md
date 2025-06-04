## **What this code does in simple terms:**

This Arduino program uses two sensors:

* **BNO055 IMU** (Inertial Measurement Unit) — measures acceleration and orientation.
* **GPS module** — gives latitude and longitude coordinates.

It **combines the data from both** sensors with a **Kalman filter**, a mathematical tool that merges noisy measurements over time, to give a better, smoother estimate of the device’s position.

---

## Step-by-step detailed explanation of each part

---


### 1. **Global Variables and Constants**

* **`EEPROM_SIZE`, `CALIB_FLAG_ADDR`, `OFFSETS_ADDR`** — for saving and loading calibration data in non-volatile memory.

* **`GPS_RX_PIN`, `GPS_TX_PIN`, `GPS_BAUD`** — pins and baud rate for GPS serial communication.

* **`dt`** — time interval (0.2 seconds) between each update.

* **`R_gps`** — GPS measurement noise (uncertainty).

* **`Q_base`** — process noise for Kalman filter (uncertainty added every step).

* **`velocityDamp`** — slows down velocity over time to avoid drift.

* **`accelThreshold`, `speedThreshold`** — thresholds to decide if the device is moving.

* **`gpsTimeout`** — time (3 seconds) after which GPS is considered unavailable if no new data.

* **`EARTH_RADIUS_M`, `DEG2RAD`** — constants for converting between lat/lon and meters.

* `Adafruit_BNO055 bno` — IMU sensor object.

* `TinyGPSPlus gps` — GPS parser object.

* `HardwareSerial GPSSerial` — hardware serial for GPS communication.

* **State vector `X[4]`**:
  `[posX, posY, velX, velY]` — position and velocity in meters in 2D plane.

* **Covariance matrix `P`** (4x4): estimates uncertainty for each state variable.

* `originLat`, `originLon` — reference GPS coordinates to convert lat/lon to local X/Y.

* `originSet` — flag if origin coordinates are set.

* `gpsAvailable` — flag if GPS is currently working.

* `lastGPSFixTime` — timestamp of the last valid GPS reading.


## **What is the Purpose of this Code?**

This Arduino program combines two sensors:

* **GPS sensor:** Gives raw latitude and longitude coordinates but with noise (errors).
* **IMU sensor (BNO055):** Gives acceleration and orientation data, but acceleration alone drifts over time.

It uses a **Kalman filter** — a special algorithm that merges noisy sensor data over time — to produce a **better, smoother, and more accurate estimate of the device’s position and velocity.**

---

# Key Concepts 

### 1. **GPS Coordinates vs. Local Coordinates**

* GPS gives you latitude and longitude (like a global address).
* Latitude and longitude are angles, not distances.
* For calculations, you convert lat/lon into a local flat coordinate system (X, Y in meters).
* This lets the Kalman filter work with simple linear math.

---

### 2. **IMU Acceleration and Orientation**

* The IMU measures acceleration **relative to itself** (device frame).
* The device can tilt and rotate, so acceleration readings need to be rotated into the Earth frame (so “up” is always vertical, etc.).
* We use the IMU’s quaternion (a math object representing orientation) to rotate acceleration vectors.

---

### 3. **Kalman Filter in a Nutshell**

* The Kalman filter keeps track of the system’s **state** (position and velocity here).
* It has two steps every cycle:

  **Predict Step:**

  * Use previous state + control inputs (acceleration) to predict the new state.
  * Also update how uncertain you are about the prediction.

  **Update Step:**

  * When you get a GPS measurement, compare it to your prediction.
  * Adjust your predicted state towards the GPS measurement based on uncertainties (trusting more accurate data more).

---

# Now, step-by-step breakdown of the code:

---

## 1. Global Variables and Constants — What each means

```cpp
#define EEPROM_SIZE 40
#define CALIB_FLAG_ADDR 0
#define OFFSETS_ADDR 1

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD 9600

#define dt 0.2           // Time between updates in seconds (200ms)
#define R_gps 4          // GPS measurement noise covariance (how noisy GPS is)
#define Q_base 0.1       // Process noise covariance (uncertainty added each step)
#define velocityDamp 0.95 // Velocity damping factor to slow drift

#define accelThreshold 0.3    // Threshold acceleration (m/s^2) to decide if moving
#define speedThreshold 0.2    // Threshold GPS speed (m/s) to decide if moving
#define gpsTimeout 3000       // Max time (ms) without GPS before marking GPS unavailable

#define EARTH_RADIUS_M 6371000  // Earth's radius in meters
#define DEG2RAD 0.0174533       // Degrees to radians conversion factor
```

* **`EEPROM_SIZE`**: EEPROM is memory inside Arduino that keeps data even if power is off. We use it to save calibration info so it doesn’t have to recalibrate every startup.
* **`GPS_RX_PIN` and `GPS_TX_PIN`**: Physical pins connected to the GPS serial port.
* **`dt`**: How often the program updates positions.
* **`R_gps` and `Q_base`**: Parameters defining how much noise/error we expect from GPS and the process.
* **`velocityDamp`**: We multiply velocity by this number every loop to prevent it from growing forever (imitates friction or drag).
* **`accelThreshold` and `speedThreshold`**: Minimum acceleration or speed that says “the device is moving.”
* **`gpsTimeout`**: If no GPS fix received within this time, treat GPS as not available.

---

## 2. Objects for Sensors

```cpp
Adafruit_BNO055 bno = Adafruit_BNO055();
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
```

* `bno`: Interface with BNO055 IMU sensor.
* `gps`: Parses GPS data stream.
* `GPSSerial`: Serial port used to communicate with GPS module.

---

## 3. Kalman Filter State Variables

```cpp
float X[4] = {0, 0, 0, 0};  // State vector: posX, posY, velX, velY
float P[4][4];               // Covariance matrix, uncertainty in state estimate
```

* **`X`** stores current estimate of position and velocity.
* **`P`** stores how uncertain we are about each element of `X`.

---

## 4. Origin for Coordinate Conversion

```cpp
double originLat = 0.0, originLon = 0.0;
bool originSet = false;
```

* The first GPS coordinate read is used as origin (0,0).
* All lat/lon are converted to meters relative to this origin for simple math.

---

## 5. Flags and timers

```cpp
bool gpsAvailable = false;
unsigned long lastGPSFixTime = 0;
```

* Tracks if GPS is currently valid.
* Keeps timestamp of last valid GPS reading to detect GPS loss.

---

## 6. Setup function — what happens when Arduino starts

```cpp
void setup() {
  Serial.begin(115200);
  bno.begin();
  bno.setExtCrystalUse(true);
  loadCalibration();
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  lastGPSFixTime = millis();
}
```

* Initialize serial for debugging.
* Initialize IMU and enable external crystal for better accuracy.
* Load saved calibration from EEPROM so we don’t have to recalibrate every time.
* Start GPS serial communication.
* Record current time for GPS timeout tracking.

---

## 7. Loop function — what happens every cycle

```cpp
void loop() {
  // 1. Read GPS data continuously
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }
  
  // 2. Print calibration status for debugging
  printCalibrationStatus();

  // 3. Save calibration if fully calibrated
  if (bno.getCalibration() == 0xFF) {
    saveCalibration();
  }

  // 4. If GPS has new location, get lat/lon and speed
  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    float speed = gps.speed.mps();

    // Set origin if not set
    if (!originSet) {
      originLat = lat;
      originLon = lon;
      originSet = true;
    }

    lastGPSFixTime = millis();
    gpsAvailable = true;
  }

  // 5. Read acceleration
  imu::Vector<3> linearAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float accMagnitude = sqrt(linearAcc.x() * linearAcc.x() + linearAcc.y() * linearAcc.y() + linearAcc.z() * linearAcc.z());

  // 6. Determine if moving
  bool isMoving = (accMagnitude > accelThreshold) || (gps.speed.mps() > speedThreshold);

  // 7. Rotate acceleration to Earth frame
  imu::Quaternion quat = bno.getQuat();
  // Rotation math to convert device-frame accel to earth-frame accel
  float accelX = ...
  float accelY = ...

  // 8. Kalman predict step if moving
  if (isMoving) {
    kalmanPredict(accelX, accelY);
  } else {
    // 9. Else no movement: update position from velocity and damp velocity
    ...
  }

  // 10. Kalman update step if GPS available and moving
  if (gpsAvailable && isMoving) {
    float gpsX, gpsY;
    convertLatLonToXY(lat, lon, gpsX, gpsY);
    kalmanUpdate(gpsX, gpsY);
  }

  // 11. Check GPS timeout
  if (millis() - lastGPSFixTime > gpsTimeout) {
    gpsAvailable = false;
  }

  // 12. Convert filtered state back to lat/lon for output
  double fusedLat, fusedLon;
  convertXYToLatLon(X[0], X[1], fusedLat, fusedLon);

  // 13. Print results
  Serial.print("Fused Position: ");
  Serial.print(fusedLat, 6);
  Serial.print(", ");
  Serial.println(fusedLon, 6);

  delay(dt * 1000);
}
```

---

## 8. The Math Behind Rotation of Acceleration (Step 7)

The IMU provides acceleration relative to itself (rotated as the device moves).

We want acceleration relative to Earth coordinates.

This is done by:

* Getting the **quaternion** from IMU (`bno.getQuat()`), which represents orientation.
* Using quaternion math to rotate acceleration vector from device frame to Earth frame.

---

## 9. Kalman Predict Step — How it works mathematically

```cpp
void kalmanPredict(float accelX, float accelY) {
  // 1. Predict new position using:
  // x_new = x_old + vel * dt + 0.5 * accel * dt^2
  X[0] += X[2] * dt + 0.5 * accelX * dt * dt;
  X[1] += X[3] * dt + 0.5 * accelY * dt * dt;

  // 2. Predict new velocity:
  X[2] = velocityDamp * (X[2] + accelX * dt);
  X[3] = velocityDamp * (X[3] + accelY * dt);

  // 3. Update covariance matrix P by adding process noise Q_base
  for each i in 0..3:
    P[i][i] += Q_base;
}
```

---

## 10. Kalman Update Step — Using GPS measurements to correct

```cpp
void kalmanUpdate(float zX, float zY) {
  // z = measurement vector (GPS position)
  // X = predicted state

  // 1. Calculate measurement residual:
  yX = zX - X[0];
  yY = zY - X[1];

  // 2. Calculate innovation covariance S (uncertainty):
  S = P[0][0] + R_gps;

  // 3. Calculate Kalman gain K:
  K0 = P[0][0] / S;
  K1 = P[1][1] / S;

  // 4. Update state estimate:
  X[0] += K0 * yX;
  X[1] += K1 * yY;

  // 5. Update covariance:
  P[0][0] = (1 - K0) * P[0][0];
  P[1][1] = (1 - K1) * P[1][1];
}
```

* This blends predicted position with GPS measurement weighted by uncertainty.

---

## 11. Coordinate Conversion Functions

* **Latitude/Longitude to local XY (meters):**

  Uses haversine-based approximation and Earth radius to calculate meters offset from origin.

* **Local XY (meters) to Latitude/Longitude:**

  Inverse calculation to convert back for printing or GPS comparison.

---

## 12. Summary of the Entire Logic

* On every cycle:

  * Read latest GPS data, IMU acceleration and orientation.
  * Decide if the device is moving or stationary.
  * Use acceleration to predict the new position and velocity.
  * Use GPS measurements to correct the position when available.
  * Convert back to GPS coordinates and print for user.
  * Save calibration data if necessary.




# Summary of What Each Function Does (for easy reference):

| Function                   | Purpose                                                   | Called By       |
| -------------------------- | --------------------------------------------------------- | --------------- |
| `setup()`                  | Initialize sensors and communication                      | Arduino runtime |
| `loop()`                   | Main update loop: read sensors, predict & update position | Arduino runtime |
| `loadCalibration()`        | Load saved sensor calibration from EEPROM                 | `setup()`       |
| `saveCalibration()`        | Save sensor calibration to EEPROM                         | `loop()`        |
| `printCalibrationStatus()` | Print IMU calibration status                              | `loop()`        |
| `convertLatLonToXY()`      | Convert GPS lat/lon to local XY in meters                 | `loop()`        |
| `convertXYToLatLon()`      | Convert local XY back to GPS lat/lon                      | `loop()`        |
| `kalmanPredict()`          | Predict new position & velocity from acceleration         | `loop()`        |
| `kalmanUpdate()`           | Correct predicted position with GPS measurement           | `loop()`        |
| `distanceBetween()`        | Calculate distance between two XY points                  | `loop()`        |


