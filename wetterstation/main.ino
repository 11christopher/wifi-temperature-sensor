/*
 Christopher's Wetterstation
 09-2016

 Board: 	D1 mini Pro - A mini wifi board with 16MB flash, external antenna connector and built-in ceramic antenna based on ESP-8266EX
 Sensor: BME280 - temperature, humidity, pressure

 ACHTUNG: bei der Berechnung der Seehöhe muß man den aktuellen Luftdruckwert (Pa)
 auf Meeresniveau als Parameter übergeben >> sonst kommt Mist heraus
 */

#include <stdint.h>
#include "SparkFunBME280.h"
//Library allows either I2C or SPI, so include both.
#include "Wire.h"
#include "SPI.h"
#include <math.h>
#include <ESP8266WiFi.h>

// Define serial baudrate
#define SERIAL_BAUD         115200

//// Various PIN definitions
//#define LED_PIN             9
//#define BATTERY_PIN         A7
//#define USE_MOSFET          0
//#define BATTERY_ENABLE_PIN  A3



//definitions for temperature, humidity, preassure, altitute measurement

#define a 0.00602      // ACHTUNG: 0.0065 gilt nur für Standardbedingungen h=0m T=15°C p=1013,25hPa
// besser aktuell berechnen: a= (Tz-Tb)/(hz-hb)   z=Zielpunkt   b=Bezugspunkt (kann Ort am Meer sein)
#define pSL 100800.0

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// network details
const char* ssid = "";
const char* password = "";

// Web Server on port 80
WiFiServer server(80);

//Global sensor object
BME280 mySensor;
float La, RH, TP, E, H, T, pNN, pH;

//Global voltage object
float voltage;

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------

//void blink(byte times, byte mseconds) {
//    pinMode(LED_PIN, OUTPUT);
//    for (byte i=0; i<times; i++) {
//        if (i>0) delay(mseconds);
//        digitalWrite(LED_PIN, HIGH);
//        delay(mseconds);
//        digitalWrite(LED_PIN, LOW);
//    }
//}

void hardwareSetup() {
	Serial.begin(SERIAL_BAUD);
//    pinMode(LED_PIN, OUTPUT);
//    pinMode(BATTERY_PIN, INPUT);
//    #if USE_MOSFET
//        pinMode(BATTERY_ENABLE_PIN, INPUT);
//    #endif
	delay(1);
}

// -----------------------------------------------------------------------------
// BME280 sensor
// -----------------------------------------------------------------------------

void mySensorReadAllValues() {

	//Start with temperature, as that data is needed for accurate compensation.
	//Reading the temperature updates the compensators of the other functions
	//in the background.
	mySensor.begin();

	T = mySensor.readTempC();
	Serial.print("Temperatur: ");
	Serial.print(T, 2);
	Serial.println(" °C");

	/*Serial.print("Temperature: ");
	 Serial.print(mySensor.readTempF(), 2);
	 Serial.println(" degrees F");*/

	pH = mySensor.readFloatPressure() / 100.0;
	Serial.print("Luftdruck vor Ort: ");
	Serial.print(pH, 2);
	Serial.println(" hPa");

	//  H=mySensor.readFloatAltitudeMeters(pSL);
	H = mySensor.readFloatAltitudeMeters();

	E = 6.112 * exp((17.62 * T) / (243.12 + T));
	pNN =
			pH
					* exp(
							9.80665 * H
									/ (287.058
											* ((273.15 + T) + 0.12 * E
													+ a * H / 2)));

	Serial.print("Luftdruck reduziert: ");
	Serial.print(pNN, 2);
	Serial.println(" hPa");

	Serial.print("Höhe über Meer: ");
	Serial.print(H, 2);
	Serial.println("m");

	/*Serial.print("Altitude: ");
	 Serial.print(mySensor.readFloatAltitudeFeet(), 2);
	 Serial.println("ft"); */

	RH = mySensor.readFloatHumidity();
	Serial.print("Relative Luftfeuchte: ");
	Serial.print(RH, 2);
	Serial.println(" %");

	La = RH * E / (461.51 * (273.15 + T)) * 10000.0;

	Serial.print("Absolute Luftfeuchte: ");
	Serial.print(La, 2);
	Serial.println(" g/m³");

	TP = 243.12 * ((17.62 * T) / (243.12 + T) + log(RH / 100.0))
			/ ((17.62 * 243.12) / (243.12 + T) - log(RH / 100.0));
	Serial.print("Taupunkt: ");
	Serial.print(TP, 2);
	Serial.println(" °C");

	Serial.println();
}

void mySensorSetup() {

	//***Driver settings********************************//
	//commInterface can be I2C_MODE or SPI_MODE
	//specify chipSelectPin using arduino pin names
	//specify I2C address.  Can be 0x77(default) or 0x76

	//For I2C, enable the following and disable the SPI section
	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = 0x76;

	//For SPI enable the following and dissable the I2C section
	//  mySensor.settings.commInterface = SPI_MODE;
	//  mySensor.settings.chipSelectPin = 10;

	//***Operation settings*****************************//

	//renMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	mySensor.settings.runMode = 1; //Forced mode, energy saving

	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	mySensor.settings.tStandby = 0;

	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	mySensor.settings.filter = 0;

	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.tempOverSample = 1;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.pressOverSample = 1;

	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	mySensor.settings.humidOverSample = 1;

	Serial.print("Starting BME280... result of .begin(): 0x");

	//Calling .begin() causes the settings to be loaded
	delay(10); //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
	Serial.println(mySensor.begin(), HEX);

	Serial.print("Displaying ID, reset and ctrl regs\n");

	Serial.print("ID(0xD0): 0x");
	Serial.println(mySensor.readRegister(BME280_CHIP_ID_REG), HEX);
	Serial.print("Reset register(0xE0): 0x");
	Serial.println(mySensor.readRegister(BME280_RST_REG), HEX);
	Serial.print("ctrl_meas(0xF4): 0x");
	Serial.println(mySensor.readRegister(BME280_CTRL_MEAS_REG), HEX);
	Serial.print("ctrl_hum(0xF2): 0x");
	Serial.println(mySensor.readRegister(BME280_CTRL_HUMIDITY_REG), HEX);

	Serial.print("\n\n");

	Serial.print("Displaying all regs\n");
	uint8_t memCounter = 0x80;
	uint8_t tempReadData;
	for (int rowi = 8; rowi < 16; rowi++) {
		Serial.print("0x");
		Serial.print(rowi, HEX);
		Serial.print("0:");
		for (int coli = 0; coli < 16; coli++) {
			tempReadData = mySensor.readRegister(memCounter);
			Serial.print((tempReadData >> 4) & 0x0F, HEX); //Print first hex nibble
			Serial.print(tempReadData & 0x0F, HEX); //Print second hex nibble
			Serial.print(" ");
			memCounter++;
		}
		Serial.print("\n");
	}

	Serial.print("\n\n");

	Serial.print("Displaying concatenated calibration words\n");
	Serial.print("dig_T1, uint16: ");
	Serial.println(mySensor.calibration.dig_T1);
	Serial.print("dig_T2, int16: ");
	Serial.println(mySensor.calibration.dig_T2);
	Serial.print("dig_T3, int16: ");
	Serial.println(mySensor.calibration.dig_T3);

	Serial.print("dig_P1, uint16: ");
	Serial.println(mySensor.calibration.dig_P1);
	Serial.print("dig_P2, int16: ");
	Serial.println(mySensor.calibration.dig_P2);
	Serial.print("dig_P3, int16: ");
	Serial.println(mySensor.calibration.dig_P3);
	Serial.print("dig_P4, int16: ");
	Serial.println(mySensor.calibration.dig_P4);
	Serial.print("dig_P5, int16: ");
	Serial.println(mySensor.calibration.dig_P5);
	Serial.print("dig_P6, int16: ");
	Serial.println(mySensor.calibration.dig_P6);
	Serial.print("dig_P7, int16: ");
	Serial.println(mySensor.calibration.dig_P7);
	Serial.print("dig_P8, int16: ");
	Serial.println(mySensor.calibration.dig_P8);
	Serial.print("dig_P9, int16: ");
	Serial.println(mySensor.calibration.dig_P9);

	Serial.print("dig_H1, uint8: ");
	Serial.println(mySensor.calibration.dig_H1);
	Serial.print("dig_H2, int16: ");
	Serial.println(mySensor.calibration.dig_H2);
	Serial.print("dig_H3, uint8: ");
	Serial.println(mySensor.calibration.dig_H3);
	Serial.print("dig_H4, int16: ");
	Serial.println(mySensor.calibration.dig_H4);
	Serial.print("dig_H5, int16: ");
	Serial.println(mySensor.calibration.dig_H5);
	Serial.print("dig_H6, uint8: ");
	Serial.println(mySensor.calibration.dig_H6);

	Serial.println();
	delay(5000);
}


// -----------------------------------------------------------------------------
// Battery monitoring
// -----------------------------------------------------------------------------
void readBatteryVoltage(){
	int sensorValue = analogRead(A0);
	voltage = sensorValue / 192.02;

	Serial.print("Voltage: ");
	Serial.print(voltage);
	Serial.print(" | ");
	Serial.print("Sensor: ");
	Serial.println(sensorValue);

}


// -----------------------------------------------------------------------------
// ESP8266
// -----------------------------------------------------------------------------

void scanWifi(){
	  // Set WiFi to station mode and disconnect from an AP if it was previously connected
	  WiFi.mode(WIFI_STA);
	  WiFi.disconnect();
	  delay(100);

	  Serial.println("scan start");

	  // WiFi.scanNetworks will return the number of networks found
	  int n = WiFi.scanNetworks();
	  Serial.println("scan done");
	  if (n == 0)
	    Serial.println("no networks found");
	  else
	  {
	    Serial.print(n);
	    Serial.println(" networks found");
	    for (int i = 0; i < n; ++i)
	    {
	      // Print SSID and RSSI for each network found
	      Serial.print(i + 1);
	      Serial.print(": ");
	      Serial.print(WiFi.SSID());
	      Serial.print(" (");
	      Serial.print(WiFi.RSSI());
	      Serial.print(")");
	      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
	      delay(10);
	    }
	  }
	  Serial.println("");
}

void connectWifi(){
	  WiFi.mode(WIFI_STA);
	  WiFi.disconnect();
	  delay(100);
	Serial.print("Connecting to ");
	  Serial.println(ssid);

	  WiFi.begin(ssid, password);

	  while (WiFi.status() != WL_CONNECTED) {
	    delay(500);
	    Serial.print(".");
	  }
	  Serial.println("");
	  Serial.println("WiFi connected");
}

void startWebServer(){
	  // Starting the web server
	  server.begin();
	  Serial.println("Web server running. Waiting for IP address...");
	  delay(10000);

	  // Printing the ESP IP address
	  Serial.println(WiFi.localIP());
}

void serveClient(){
	  // Listenning for new clients
	  WiFiClient client = server.available();

	  if (client) {
	    Serial.println("New client");
	    // bolean to locate when the http request ends
	    boolean blank_line = true;
	    while (client.connected()) {
	      if (client.available()) {
	        char c = client.read();

	        if (c == '\n' && blank_line) {
//	            getWeather();
	            client.println("HTTP/1.1 200 OK");
	            client.println("Content-Type: text/html");
	            client.println("Connection: close");
	            client.println();
	            // your actual web page that displays temperature
	            client.println("<!DOCTYPE HTML>");
	            client.println("<html>");
	            client.println("<head><META HTTP-EQUIV=\"refresh\" CONTENT=\"5\"></head>");
	            client.println("<body><h1>ESP8266 Weather Web Server</h1>");
	            client.println("<table border=\"2\" width=\"456\" cellpadding=\"10\"><tbody><tr><td>");
	            client.println("<h3>Temperatur: ");
	            client.println(T, 2);
	            client.println("&deg;C</h3><h3>Luftdruck vor Ort: ");
	            client.println(pH, 2);
	            client.println("hPa</h3><h3>Luftdruck reduziert: ");
	            client.println(pNN, 2);
	            client.println("hPa</h3><h3>Relative Luftfeuchte: ");
	            client.println(RH, 2);
	            client.println("%</h3><h3>Absolute Luftfeuchte: ");
	            client.println(La);
	            client.println("g/m³</h3><h3>Taupunkt: ");
	            client.println(TP, 2);
	            client.println("&deg;C</h3><h3>Akku-Spannung: ");
	            client.println(voltage, 2);
	            client.println("V</h3></td></tr></tbody></table></body></html>");
	            break;
	        }
	        if (c == '\n') {
	          // when starts reading a new line
	          blank_line = true;
	        }
	        else if (c != '\r') {
	          // when finds a character on the current line
	          blank_line = false;
	        }
	      }
	    }
	    // closing the client connection
	    delay(1);
	    client.stop();
	    Serial.println("Client disconnected.");
	  }
}

// -----------------------------------------------------------------------------
// Flash
// -----------------------------------------------------------------------------

//void flashSetup() {
//    if (flash.initialize()) {
//        flash.sleep();
//    }
//}

// -----------------------------------------------------------------------------
// Messages
// -----------------------------------------------------------------------------

//void sendSensor() {
//
//    char buffer[8];
//
//    // Trigger reading
//    bmeForceRead();
//
//    // Temperature
//    dtostrf(bme.readTempC(), 4, 1, buffer);
////    radio.send((char *) "TMP", buffer, (uint8_t) 2);
//    Serial.println(buffer);
//
//    // Humidity
//    itoa((int) bme.readFloatHumidity(), buffer, 10);
////    radio.send((char *) "HUM", buffer, (uint8_t) 2);
//    Serial.println(buffer);
//
//    // Pressure
//    dtostrf(bme.readFloatPressure() / 100.0F, 6, 1, buffer);
////    radio.send((char *) "PRS", buffer, (uint8_t) 2);
//
//}

//void sendBattery() {
//
//    unsigned int voltage;
//
//    // LowPowerLabs WeatherShield v2 can use p-mosfet to enable
//    // power monitoring, but it ships without this feature. If you
//    // add the required components (a p-mosfet plus 2 resistors)
//    // change the USE_MOSFET to 1 in the settings.h file
//    #if USE_MOSFET
//        pinMode(BATTERY_ENABLE_PIN, OUTPUT);
//        digitalWrite(BATTERY_ENABLE_PIN, LOW);
//    #endif
//    voltage = analogRead(BATTERY_PIN);
//    #if USE_MOSFET
//        pinMode(BATTERY_ENABLE_PIN, INPUT);
//    #endif
//
//    // Map analog reading to VIN value
//    voltage = BATTERY_RATIO * voltage;
//
//    char buffer[6];
//    itoa((int) voltage, buffer, 10);
//    radio.send((char *) "BAT", buffer, (uint8_t) 2);
//
//}

//void send()  {
//
//    // Send current sensor readings
//    sendSensor();
//
////    // Send battery status once every 10 messages, starting with the first one
////    static unsigned char batteryCountdown = 0;
////    if (batteryCountdown == 0) sendBattery();
////    batteryCountdown = (batteryCountdown + 1) % 10;
////
////    // Radio back to sleep
////    radio.sleep();
//
//    // Show visual notification
//    blink(1, NOTIFICATION_TIME);
//
//}

// -----------------------------------------------------------------------------
// Common methods
// -----------------------------------------------------------------------------

void setup() {
	hardwareSetup();
	mySensorSetup();
//	scanWifi();
	connectWifi();
	startWebServer();
//    flashSetup();
}

void loop() {

	mySensorReadAllValues();
	readBatteryVoltage();
	serveClient();

//    // Sleep loop
//    for (byte i = 0; i < SLEEP_COUNT; i++) {
//
//        // Sleep for 8 seconds (the maximum the WDT accepts)
//        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
//
//    }

	delay(3000);
}
