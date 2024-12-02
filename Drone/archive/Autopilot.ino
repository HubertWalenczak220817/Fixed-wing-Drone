#include <PID_v1.h>
#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// PID Parameters
const double MAX_CLIMB_RATE = 15; // m/s
const double MAX_BANK = 30; // deg
const double MAX_DEFLECTION_RATE_ELEVATOR = 1; // %/s 2=100%
const double MAX_DEFLECTION_RATE_AILERON = 0.8; // %/s 2=100%
const double MAX_THROTTLE_RATE = 0.9; // %/s 1=100%
const double MAX_G_FORCE_P = 3;
const double MAX_G_FORCE_N = -1;
const double PITCH_DOWN_LIMIT = -15;
const double PITCH_UP_LIMIT = 25;

double _altitudeSetpoint, _headingSetpoint, _velocitySetpoint, _des_ClimbRate, _des_Pitch, _bankSetpoint;
//double _altitude /* m */, _climbRate /* m/s */, _velocity /* m/s */, _heading /* deg */;
//double _roll /* deg */, _pitch /* deg */, _lat /* deg */, _lon /* deg */, _gForce;
double _throttle, _aileron, _rudder, _elevator;
double _lastElevator, _lastThrottle, _lastAileron;

const double Vp = 0.15, Vi = 10, Vd = 1.5;
const double Hp = 0.11, Hi = 0, Hd = 0.002;
const double Ep = 2, Ei = 1, Ed = 0.005;
//const double Ep = 0.045, Ei = 0.1, Ed = 0.005;
//const double Headp = 0.11, Headi = 0, Headd = 0.002;
const double Ap = 0.008, Ai = 0.001, Ad = 0.005;
const double Rp = 2, Ri = 5, Rd = 1;

struct AInfo{double pitch; double roll; double heading; double altitude; double velocity; double lat; double lon; double gForce; double climbRate;} AircraftInfo;

int first = 10;
unsigned long lastTime = 0;
// Create PID object
PID _throttlePID(&AircraftInfo.velocity, &_throttle, &_velocitySetpoint, Vp, Vi, Vd, DIRECT);

PID _altitudePID(&AircraftInfo.altitude, &_des_ClimbRate, &_altitudeSetpoint, Hp, Hi, Hd, DIRECT);
//PID _elevatorPID(&AircraftInfo.climbRate, &_elevator, &_des_ClimbRate, Ep, Ei, Ed, REVERSE);
PID _elevatorPID(&AircraftInfo.pitch, &_elevator, &_des_Pitch, Ep, Ei, Ed, REVERSE);

//PID _headingPID(&_heading, &_bankSetpoint, &_headingSetpoint, Headp, Headi, Headd, DIRECT);
PID _bankPID(&AircraftInfo.roll, &_aileron, &_bankSetpoint, Ap, Ai, Ad, DIRECT);

PID _rudderPID(&AircraftInfo.heading, &_rudder, &_headingSetpoint, Rp, Ri, Rd, DIRECT);

void display_init()
{
  Wire.begin();
  u8g2.begin();

  char data[32] = {0};
  
  u8g2.clearBuffer();					// clear the internal memory
	u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font

  sprintf(data, "Display");
	u8g2.drawStr(3, 15, data);
  sprintf(data, "initialized");
	u8g2.drawStr(3, 30, data);

  u8g2.sendBuffer(); // transfer internal memory to the display
  delay(2000);
}

void display(double lat, double lon, double altitude, double velocity)
{
	char data[32] = {0};

/*
	Serial.print("lat=");
	Serial.print(lat);
	Serial.print(" deg, lon=");
	Serial.print(lon);
	Serial.print(" deg, H=");
	Serial.print(altitude);
	Serial.print(" m");

	Serial.printf(", V=%.3f m/s",velocity);
*/
	// display bme680 sensor data on OLED
	u8g2.clearBuffer();					// clear the internal memory
	//u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font

	memset(data, 0, sizeof(data));
	sprintf(data, "lat=%.3f deg", lat);
	u8g2.drawStr(3, 15, data);

	memset(data, 0, sizeof(data));
	snprintf(data, 32, "lon=%.3f deg", lon);
	u8g2.drawStr(3, 30, data);

	memset(data, 0, sizeof(data));
	sprintf(data, "H=%.1f m", altitude);
	u8g2.drawStr(3, 45, data);

	memset(data, 0, sizeof(data));
  sprintf(data, "V=%.2f m/s", velocity);
  u8g2.drawStr(3, 60, data);

	u8g2.sendBuffer(); // transfer internal memory to the display
}

double convertAltitudeToVerticalSpeed(double deltaAltitude, double reference = 5) {
  if (reference < 2)
      reference = 2;

  if (deltaAltitude > reference || deltaAltitude < -reference)
      return deltaAltitude / reference;
  if (deltaAltitude > 0.1)
      return max(0.05, deltaAltitude * deltaAltitude / (reference * reference));
  if (deltaAltitude < -0.1)
      return min(-0.05, -deltaAltitude * deltaAltitude / (reference * reference));
  return 0;
}

int parseFlightGearData(String &input, AInfo* aInfo) {
  int startIndex = 0;
  int tabIndex = input.indexOf('\t');

  // Extract _pitch
  aInfo->pitch = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract _roll
  tabIndex = input.indexOf('\t', startIndex);
  aInfo->roll = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract _heading
  tabIndex = input.indexOf('\t', startIndex);
  aInfo->heading = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract _altitude
  tabIndex = input.indexOf('\t', startIndex);
  aInfo->altitude = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract _velocity
  tabIndex = input.indexOf('\t', startIndex);
  aInfo->velocity = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract _latitude
  tabIndex = input.indexOf('\t', startIndex);
  aInfo->lat = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract clibmRate
  tabIndex = input.indexOf('\t', startIndex);
  aInfo->lon = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract G-Force
  tabIndex = input.indexOf('\t', startIndex);
  aInfo->climbRate = input.substring(startIndex, tabIndex).toFloat();
  startIndex = tabIndex + 1;

  // Extract _longitude (last element)
  aInfo->gForce = input.substring(startIndex).toFloat();

  return 0;
}

void setup()
{
  // Initialize the built in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Serial for debug output
  Serial.begin(115200);

  time_t serial_timeout = millis();
  lastTime = serial_timeout;
  // On nRF52840 the USB serial is not available immediately
  while (!Serial)
  {
    if ((millis() - serial_timeout) < 5000)
    {
      delay(100);
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    else
    {
      break;
    }
  }

  // Set the target _pitch (e.g., to maintain level flight at 0 degrees)

  _altitudeSetpoint = 1000;
  _velocitySetpoint = 60;
  _headingSetpoint = 0;
  // Initialize PID cont_roller
  
  _throttlePID.SetMode(AUTOMATIC);
  _throttlePID.SetOutputLimits(0.1,1);

  _altitudePID.SetMode(AUTOMATIC);
  _altitudePID.SetOutputLimits(-MAX_CLIMB_RATE, MAX_CLIMB_RATE);
  _elevatorPID.SetMode(AUTOMATIC);
  _elevatorPID.SetOutputLimits(-1, 1);

  //_headingPID.SetMode(AUTOMATIC);
  //_headingPID.SetOutputLimits(-MAX_BANK, MAX_BANK);
  _bankPID.SetMode(AUTOMATIC);
  _bankPID.SetOutputLimits(-1, 1);

  _rudderPID.SetMode(AUTOMATIC);
  _rudderPID.SetOutputLimits(-1, 1);

  display_init();
}

void loop()
{
  unsigned long now = millis();
  double deltaTime = (now - lastTime) / 1000.0;  // Convert ms to seconds
  lastTime = now;
  static unsigned long bankDelay = now;
  // Parse incoming data
  String incomingData = Serial.readStringUntil('\n');
  Serial.print(incomingData);
  Serial.print("\n");

  parseFlightGearData(incomingData, &AircraftInfo);
  while (AircraftInfo.climbRate && first) {
    Serial.print("0.2");
    Serial.print("\t");
    Serial.print("0");
    Serial.print("\t");
    Serial.print("0");
    Serial.print("\t");
    Serial.print("\n");
    delay(1000);
    bankDelay = now;
    first--;
  }
  if (now - bankDelay > 30000) {
    //_bankSetpoint = 15;
  }
  // Step 2: Use the current _pitch as the PID input
  //Input = _pitch;

  // Step 3: Compute _throttle PID Output
  //_velocityPID.Compute();
  //Serial.print("_throttle=");
  //Serial.print(_velocityOutput);
  //Serial.print("\t");
  // ============ //
  // _throttle PID //
  // ============ //  
  // Compute _throttle PID
  _throttlePID.Compute();
  
  double maxDeltaOutput = MAX_THROTTLE_RATE * deltaTime;

  // Limit the rate of change of the output
  double deltaOutput = _throttle - _lastThrottle; // Apply the limited output change

  // Limit Deflection
  deltaOutput = constrain(deltaOutput, -maxDeltaOutput, maxDeltaOutput);

  _throttle = _lastThrottle + deltaOutput;

  _lastThrottle = _throttle;

  // ============ //
  // ELEVATOR PID //
  // ============ //  
  // Compute elevator PID
  //_altitudePID.Compute();
  double deltaAltitude = _altitudeSetpoint - AircraftInfo.altitude;
  _des_ClimbRate = convertAltitudeToVerticalSpeed(deltaAltitude, 10);
  _des_ClimbRate = constrain(_des_ClimbRate, -MAX_CLIMB_RATE, MAX_CLIMB_RATE);
  
  double deltaVertSpeed = _des_ClimbRate - AircraftInfo.climbRate;
  double adjustment = deltaVertSpeed / AircraftInfo.velocity * 180 / 3.14;

  _des_Pitch = AircraftInfo.pitch + adjustment;
  _des_Pitch = constrain(_des_Pitch, -PITCH_DOWN_LIMIT, PITCH_DOWN_LIMIT);

  _elevatorPID.Compute();

  /*if (AircraftInfo.gForce > MAX_G_FORCE_P) {
    double scale = MAX_G_FORCE_P / AircraftInfo.gForce;
    _elevator *= scale; // Scale down elevator to limit G-force
  } else if (AircraftInfo.gForce < MAX_G_FORCE_N) {
    double scale = MAX_G_FORCE_N / AircraftInfo.gForce;
    _elevator *= scale; // Scale down elevator to limit G-force
  }*/

  maxDeltaOutput = MAX_DEFLECTION_RATE_ELEVATOR * deltaTime;

  // Limit the rate of change of the output
  deltaOutput = _elevator - _lastElevator; // Apply the limited output change

  // Limit Deflection
  deltaOutput = constrain(deltaOutput, -maxDeltaOutput, maxDeltaOutput);

  _elevator = _lastElevator + deltaOutput;

  _lastElevator = _elevator;

  if (AircraftInfo.roll > 90 || AircraftInfo.roll < -90) {
    _elevator *= -1.1;
  }

  // =========== //
  // aileron PID //
  // =========== //
  //_headingPID.Compute();

  double headingError = _headingSetpoint - AircraftInfo.heading;
  // Normilize to range -180 to 180
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;

  _bankSetpoint = .5 * headingError;

  if (_bankSetpoint > MAX_BANK) _bankSetpoint = MAX_BANK;
  if (_bankSetpoint < -MAX_BANK) _bankSetpoint = -MAX_BANK;

  _bankPID.Compute();
  maxDeltaOutput = MAX_DEFLECTION_RATE_AILERON * deltaTime;

  // Limit the rate of change of the output
  deltaOutput = _aileron - _lastAileron; // Apply the limited output change

  // Limit Deflection
  deltaOutput = constrain(deltaOutput, -maxDeltaOutput, maxDeltaOutput);

  _aileron = _lastAileron + deltaOutput;

  _lastAileron = _aileron;


  // ========== //
  // rudder PID //
  // ========== //  
  //_rudderPID.Compute();
  if ((AircraftInfo.roll < -5) or (AircraftInfo.roll > 5)) {
    double targetRudder = _aileron / 2;
    if (targetRudder < _rudder) _rudder -= 0.015;
    if (targetRudder > _rudder) _rudder += 0.015;
  }


  display(AircraftInfo.gForce, _des_ClimbRate, AircraftInfo.climbRate, AircraftInfo.altitude);
  
  Serial.print(_throttle);  //Send _throttle position
  Serial.print("\t");
  Serial.print(_elevator);  //Send elevator position
  Serial.print("\t");
  Serial.print(_aileron);  //Send _aileron position
  Serial.print("\t");
  Serial.print(_rudder);  //Send _rudder position
  //Serial.print("\t");
  Serial.print("\n"); 
}