// -----------------------------------------------------------------------------
// Settings
// -----------------------------------------------------------------------------


// Comment to stop sending debug messages to serial console
#define DEBUG

// Define serial baudrate
#define SERIAL_BAUD         115200

// Various PIN definitions
#define LED_PIN             9
#define BATTERY_PIN         A7
#define USE_MOSFET          0
#define BATTERY_ENABLE_PIN  A3


// Flash LED for this amount of milliseconds after every message sent
#define NOTIFICATION_TIME   5

// Battery monitoring circuitry:
// Vi -- R1 -- A1 (Vo) -- R2 -- D12 (GND)
//
// These values have been measured with a multimeter:
// R1 = 470k
// R2 = 1000k
//
// Formulae:
// Vo = Vi * R2 / (R1 + R2)
// Vi = Vo * (R1 + R2) / R2
// Vo = X * 3300 / 1024;
//
// Thus:
// Vi = X * 3300 * (1000 + 470) / 1024 / 1000;
#define BATTERY_RATIO       4.7373
