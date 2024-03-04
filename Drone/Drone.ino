/**
 * @file LoRaP2P_TX.ino
 * @author rakwireless.com
 * @brief Transmitter node for LoRa point to point communication
 * @version 0.1
 * @date 2020-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */

// Radio
#include <Arduino.h>
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>

// 9deg of freedom MPU
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68  // library: http://librarymanager/All#MPU9250_WE
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

// GPS
#include <TinyGPS.h>    //http://librarymanager/All#TinyGPS
TinyGPS gps;
String tmp_data = "";
int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

// Presure sensor
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>  // Click here to get the library: http://librarymanager/All#Adafruit_LPS2X

Adafruit_LPS22 g_lps22hb;

#include <cmath>

// Function declarations
void OnTxDone(void);
void OnTxTimeout(void);

#ifdef NRF52_SERIES
#define LED_BUILTIN 35
#endif

// Define LoRa parameters
#define RF_FREQUENCY 868300000	// Hz
#define TX_OUTPUT_POWER 14		// dBm
#define LORA_BANDWIDTH 1		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;
static uint8_t TxdBuffer[64];
const double rho = 1.225;  // density of air in kg/m^3 (approximately at sea level)

void setup()
{
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
	// Initialize Serial for debug output
	time_t timeout = millis();
	Serial.begin(115200);
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
		{
            delay(100);
        }
        else
        {
            break;
        }
	}
  
  Wire.begin();
  if (!myMPU9250.init()) {
    Serial.println("MPU9250 does not respond");
    while (1) delay(100);
  }
  else {
    Serial.println("MPU9250 is connected");
  }
  // Initialize GPS
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1000);

  Serial1.begin(9600);
  while (!Serial1)
    ;
  Serial.println("GPS uart init ok!");

  // Calibrating 9deg of freedom
  Serial.println("RAK1905 Test!");
  
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);  

  // Configuring Radio
	Serial.println("=====================================");
	Serial.println("LoRap2p Tx Test");
	Serial.println("=====================================");
	// Initialize LoRa chip.
	lora_rak4630_init();
	// Initialize the Radio callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = NULL;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = NULL;
	RadioEvents.RxError = NULL;
	RadioEvents.CadDone = NULL;

	// Initialize the Radio
	Radio.Init(&RadioEvents);

	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);

	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
	send();

  // Initialize pressure sensor

  Serial.println("Adafruit LPS22 test!");

  // Try to initialize!
  if (!g_lps22hb.begin_I2C(0x5c)) 
  {
    Serial.println("Failed to find LPS22 chip");
    while (1) 
    { 
      delay(10); 
    }
  }

  Serial.println("LPS22 Found!");

  g_lps22hb.setDataRate(LPS22_RATE_10_HZ);
  Serial.print("Presure data rate set to: ");

  switch (g_lps22hb.getDataRate()) 
  {
    case LPS22_RATE_ONE_SHOT: Serial.println("One Shot / Power Down"); 
      break;
    case LPS22_RATE_1_HZ: Serial.println("1 Hz"); 
      break;
    case LPS22_RATE_10_HZ: Serial.println("10 Hz"); 
      break;
    case LPS22_RATE_25_HZ: Serial.println("25 Hz"); 
      break;
    case LPS22_RATE_50_HZ: Serial.println("50 Hz"); 
      break;

  }
}

float calculateAltitude(float pressure, float groundpressure) {
    // Calculate the altitude using the barometric formula
    float altitude = (1 - pow((pressure / groundpressure), 0.190284)) * 44330.8;

    return altitude;
}

double calculateSpeed(double pressureOnComing, double pressureAmbient) {
  double V;
  return V = sqrt((2 / rho) * (pressureOnComing - pressureAmbient));
}

void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") != -1)
    {
        direction_E_W = 0;
    }
    else
    {
        direction_E_W = 1;
    }
    
    if (tmp.indexOf(",S,") != -1)
    {
        direction_S_N = 0;
    }
    else
    {
        direction_S_N = 1;
    }
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
	Serial.println("OnTxDone");
	delay(5000);
	send();
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
	Serial.println("OnTxTimeout");
}

void send()
{
	TxdBuffer[0] = 'H';
	TxdBuffer[1] = 'e';
	TxdBuffer[2] = 'l';
	TxdBuffer[3] = 'l';
	TxdBuffer[4] = 'o';
	Radio.Send(TxdBuffer, 5);
}

void loop()
{
  // GPS stuf
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // Read acceleration and angles
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat angle = myMPU9250.getAngles();

  //TO-DO:
  // - Test

  /* For g-values the corrected raws are used */
  Serial.print("g-x      = ");
  Serial.print(gValue.x);
  Serial.print("  |  g-y      = ");
  Serial.print(gValue.y);
  Serial.print("  |  g-z      = ");
  Serial.println(gValue.z);

  /* Angles are also based on the corrected raws. Angles are simply calculated by
     angle = arcsin(g Value) */
  Serial.print("Angle x  = ");
  Serial.print(angle.x);
  Serial.print("  |  Angle y  = ");
  Serial.print(angle.y);
  Serial.print("  |  Angle z  = ");
  Serial.println(angle.z);

  Serial.print("Orientation of the module: ");
  Serial.println(myMPU9250.getOrientationAsString());

  // TO-DO: 
  // - Test
  // - Add altitude
  // - Add speed
  Serial.println("#######");
  Serial.println("# GPS #");
  Serial.println("#######");

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      tmp_data += c;
      if (gps.encode(c))
        newData = true;
    }
  }
  direction_parse(tmp_data);
  tmp_data = "";
  
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    if(direction_S_N == 0)
    {
      Serial.print("(S):");
    }
    else
    {
      Serial.print("(N):");
    }
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    if(direction_E_W == 0)
    {
      Serial.print(" (E):");
    }
    else
    {
      Serial.print(" (W):");
    }
    Serial.print("LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  // Read presusre and temp
  sensors_event_t temp;
  sensors_event_t pressure;
  g_lps22hb.getEvent(&pressure, &temp);
  Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.println(" degrees C");
  Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa");
  Serial.println("");

  //TO-DO:
  // - Recieve data from ground station
  // - Load CSV for flight plan coordinates
  // - Calculate difference current position -> target position
  // - Calculate height from pressure difference (Ground station/Drone) | Done - To be tested
  // - Calculate speed from pressure difference (oncoming - ambient)

  // Calculate height
  // Place holder, recive from radio
  float groundStationPressure = 1000.0;  // Example pressure in hPa

  // Calculate altitude using the received pressure on the drone
  float droneAltitude = calculateAltitude(pressure.pressure, groundStationPressure);

  // Place holders - TO-DO: Figure out how to get 2 barometers working
  double speed = calculateSpeed(pressure.pressure, pressure.pressure);

  // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions. 
  // Handle Radio events
	Radio.IrqProcess();

	// We are on FreeRTOS, give other tasks a chance to run
	delay(100);
	yield();

}

