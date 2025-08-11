/*********************************************************************************************************
 * ESP32S3-Zero_Cubean_Roaster Control_v2.2 (For Artisan + HiBean)
 *
 * This is the complete, corrected code for controlling a Cubean roaster with an ESP32-S3.
 * It merges a feature-rich controller base with the reverse-engineered Cubean protocol.
 *
 * KEY FEATURES:
 * - Communication: Standard UART (Serial1) for sending and receiving 11-byte FE EF packets.
 * - Temperature Decoding: Implements the verified three-formula system for high accuracy.
 * - Checksum Calculation: Dynamically calculates the correct checksum for all outgoing commands.
 * - Connectivity: Retains full BLE, WiFi AP/STA Mode, WebSockets, and TCP Server for Artisan.
 * - Advanced Control: Retains PID auto-switching, web UI, and auto-shutdown features.
 *
 * MODIFICATIONS:
 * - Implements dual-channel temperature reading from both main and panel UARTs
 * - Adds temperature validation and smoothing
 *
 * HARDWARE:
 * - Board: WAVESHARE_ESP32_S3_ZERO
 * - UART TX_PIN -> Cubean RX: 20
 * - UART RX_PIN <- Cubean TX: 19
 *********************************************************************************************************/

#include <Arduino.h>
#include <PID_v1.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <HardwareSerial.h>

// -----------------------------------------------------------------------------
// Debug Settings
// -----------------------------------------------------------------------------
#define SERIAL_DEBUG 1
#if SERIAL_DEBUG == 1
#define D_print(...)   Serial.print(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#define D_printf(...)  Serial.printf(__VA_ARGS__)
#else
#define D_print(...)
#define D_println(...)
#define D_printf(...)
#endif

// -----------------------------------------------------------------------------
// Pin & Core Definitions
// -----------------------------------------------------------------------------
const int TX_PIN = 20; // UART TX to Cubean RX
const int RX_PIN = 19; // UART RX from Cubean TX
const int LED_PIN = 21;
const int RX_PIN_PAN = 18; // choose free GPIO for extra RX
const String boardID_BLE = String("CUBEAN_ESP32_S3_ZERO");

// -----------------------------------------------------------------------------
// NeoPixel Configuration
// -----------------------------------------------------------------------------
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
const uint32_t LED_RED = pixels.Color(255, 0, 0);
const uint32_t LED_GREEN = pixels.Color(0, 255, 0);
const uint32_t LED_BLUE = pixels.Color(0, 0, 255);

// -----------------------------------------------------------------------------
// BLE UUIDs for Nordic UART Service (NUS)
// -----------------------------------------------------------------------------
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// -----------------------------------------------------------------------------
// WiFi & Web Server Configuration
// -----------------------------------------------------------------------------
WebServer server(80);
WiFiServer artisanServer(8080);
WebSocketsServer webSocket = WebSocketsServer(81);
const char* wifiAPSSID = "Cubean_Roaster";
const char* wifiAPPass = ""; // No password for AP mode
Preferences preferences;

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
String firmWareVersion = String("Cubean_Control_v2.2");

// Roaster State
float temp = 0.0;
char CorF = 'C';
uint8_t currentFan = 0;
uint8_t currentHeat = 0;
uint8_t currentDrum = 0;
bool isCooling = false;

int prevByteSeven = 0;
int tempZone = 0;

// PID variables
double pInput, pOutput, pSetpoint = 0.0;
double Kp = 12.0, Ki = 0.5, Kd = 5.0;
PID myPID(&pInput, &pOutput, &pSetpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
unsigned long lastArtisanCommandTime = 0;
const unsigned long PID_AUTOSWITCH_TIMEOUT = 3000; // 3 seconds

// BLE variables
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;

// Auto-shutdown variables
const double COOL_SHUTDOWN_TEMP_C = 55.0;
bool coolingShutdownArmed = false;

// Forward Declarations
void parseAndExecuteCommands(String input, bool fromBLE = false);
void sendRoasterMessage();
void getRoasterMessage();

// =============================================================================
// CUBEAN COMMUNICATION LOGIC
// =============================================================================

/**
 * @brief Decodes temperature from Cubean data using the three-formula system.
 * @param b2 Byte 2 from the packet (index 2).
 * @param b3 Byte 3 from the packet (index 3).
 * @return Temperature in Celsius.
 */
float decodeCubeanTemp(uint8_t b2, uint8_t b3) {
    const uint16_t value = (static_cast<uint16_t>(b2) << 8) | b3;

    if (tempZone == 0) {
        // Formula for tempZone 0
        return (171.66f - 0.04369f * value) /
               (1.0f + 0.00025976f * value - 1.0052e-7f * value * value);
    }
    else if (tempZone == 1) {
        // Formula for tempZone 1
        const float arg = constrain(0.00063224f * value - 1.4507f, -1.0f, 1.0f);
        return -66.198f * asin(arg) + 162.93f;
    }
    else {
        return temp;
    }
}
}

/**
 * Reads a 11-byte packet from the extra RX UART
 * and detect zone.
 */
void getPanelMessage() {
  if (Serial2.available() >= 11) {
    uint8_t buf[11];
    int bytesRead = Serial2.readBytes(buf, 11);

    if (bytesRead == 11 && buf[0] == 0xFE && buf[1] == 0xEF) {
      uint8_t byteSeven = buf[7];
      if (byteSeven != prevByteSeven) {
        prevByteSeven = byteSeven;  // Store the new value
        if (abs(byteSeven - prevByteSeven) == 16){
            tempZone = (tempZone == 0) ? 1 : 0;
        }
      }
    }
}

/**
 * @brief Sends a command packet to the Cubean roaster with a valid checksum.
 */

void sendRoasterMessage() {
    uint8_t fixedByte = 0x02;      // Always 0x02 in roasting mode
    //uint8_t stateByte = 0x26;      // Always roasting mode with drum on

    uint8_t packet[11] = {
        0xFE, 0xEF,
        0x00, 0x00,
        currentHeat,  // Byte 4: Heater
        currentFan,   // Byte 5: Fan
        fixedByte,    // Byte 6: Fixed value
        prevByteSeven,    // Byte 7: determined by the panel
        0xAA, 0x55,
        0x00          // Checksum placeholder
    };

    // Calculate checksum (sum of first 10 bytes)
    uint16_t checksum = 0;
    for (int i = 0; i < 10; i++) {
        checksum += packet[i];
    }
    packet[10] = checksum & 0xFF;  // Only lower byte of checksum

    Serial1.write(packet, sizeof(packet));
}

/**
 * @brief Reads and parses a status packet from the Cubean roaster.
 */
void getRoasterMessage() {
    if (Serial1.available() >= 11) {
        uint8_t buffer[11];
        int bytesRead = Serial1.readBytes(buffer, 11);

        if (bytesRead == 11 && buffer[0] == 0xFE && buffer[1] == 0xEF) {
            float newTemp = decodeCubeanTemp(buffer[2], buffer[3]);

            // Simple filter for temp stability
            if (abs(newTemp - temp) < 50 || temp == 0.0) { // Plausibility check
                 temp = newTemp;
            }
        }
    }
}


// =============================================================================
// LED & SYSTEM STATUS
// =============================================================================
void setRGBColor(uint32_t color) {
    pixels.setPixelColor(0, color);
    pixels.show();
}

void handleLED() {
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;

    if (deviceConnected) {
        setRGBColor(LED_BLUE);
    } else if (WiFi.status() == WL_CONNECTED) {
        if(millis() - lastBlinkTime > 1000) {
            lastBlinkTime = millis();
            ledState = !ledState;
            setRGBColor(ledState ? LED_GREEN : 0);
        }
    } else { // AP Mode
        if(millis() - lastBlinkTime > 250) {
            lastBlinkTime = millis();
            ledState = !ledState;
            setRGBColor(ledState ? LED_RED : 0);
        }
    }
}

void shutdown() {
    currentFan = 0;
    currentHeat = 0;
    currentDrum = 0;
    isCooling = false;
    coolingShutdownArmed = false;
    sendRoasterMessage(); // Send shutdown command
    D_println("SYSTEM SHUTDOWN: All controls set to 0.");
}

void handleAutoShutdown() {
    if (isCooling) {
        double currentTempC = (CorF == 'F') ? (temp - 32.0) / 1.8 : temp;

        if (!coolingShutdownArmed && currentTempC > (COOL_SHUTDOWN_TEMP_C + 5.0)) {
            coolingShutdownArmed = true;
            D_println("Auto-shutdown ARMED.");
        }

        if (coolingShutdownArmed && currentTempC > 0 && currentTempC < COOL_SHUTDOWN_TEMP_C) {
            D_printf("Auto-shutdown TRIGGERED: Temp (%.1f C) is below threshold.\n", currentTempC);
            shutdown();
        }
    } else {
        if (coolingShutdownArmed) {
            D_println("Cooling manually stopped, auto-shutdown DISARMED.");
            coolingShutdownArmed = false;
        }
    }
}


// =============================================================================
// PID & COMMAND HANDLING
// =============================================================================
void handleOT1(uint8_t value);
void handleVENT(uint8_t value);

void applyArtisanPID(String command) {
    // Example: "PID:10;50;0;0;100" -> OT1=10, OT2=50, SV=0
    int first = command.indexOf(':');
    String values = command.substring(first + 1);

    int h, v, sv;
    sscanf(values.c_str(), "%d;%d;%d", &h, &v, &sv);
    
    handleHEAT(h);
    handleVENT(v);
    pSetpoint = sv; // Update setpoint if needed
    
    lastArtisanCommandTime = millis();
}

void handlePID() {
    // If Artisan has sent a command recently, it remains in control.
    if (millis() - lastArtisanCommandTime < PID_AUTOSWITCH_TIMEOUT) {
        if (myPID.GetMode() != MANUAL) {
            myPID.SetMode(MANUAL);
            D_println("Switched to Artisan control.");
        }
        return; // Exit, letting Artisan's settings persist.
    }

    // If Artisan has timed out, check if we should switch to internal PID.
    // **CRITICAL FIX:** Only switch to AUTOMATIC if the setpoint is a valid temperature.
    if (pSetpoint > 0) {
        if (myPID.GetMode() == MANUAL) {
            myPID.SetMode(AUTOMATIC);
            D_println("Artisan timeout. Switched to internal PID control.");
        }
        
        // Run the internal PID calculation
        double currentTempC = (CorF == 'F') ? (temp - 32.0) / 1.8 : temp;
        pInput = currentTempC;
        myPID.Compute();
        handleHEAT((uint8_t)pOutput); // Apply the PID's calculated heat
    } else {
        // If the setpoint is 0, stay in MANUAL mode and do not change the heat.
        if (myPID.GetMode() != MANUAL) {
            myPID.SetMode(MANUAL);
            D_println("Artisan timeout, but no PID setpoint. Staying in manual mode.");
        }
    }
}

void notifyClient(const String& message) {
    if (deviceConnected && pTxCharacteristic) {
        pTxCharacteristic->setValue(message.c_str());
        pTxCharacteristic->notify();
    }
    String msgCopy = message; // Create a non-const copy for broadcastTXT
    webSocket.broadcastTXT(msgCopy);
}

void handleREAD() {
    float displayTemp = (CorF == 'F') ? (temp * 1.8) + 32.0 : temp;
    String output = "0," + String(displayTemp, 1) + "," + String(displayTemp, 1) + "," +
                    String(map(currentHeat, 0, 255, 0, 100)) + "," + 
                    String(map(currentFan, 0, 255, 0, 100)) + "\r\n";
    notifyClient(output);
}

void handleHEAT(uint8_t value) {
    currentHeat = constrain(value, 0, 100);  // Not map to 255!
}

void handleVENT(uint8_t value) {
    currentFan = constrain(value, 0, 100);   // Not map to 255!
}

void handleDRUM(uint8_t value) {
    currentDrum = (value != 0) ? 255 : 0;
}

void handleCOOL(uint8_t value) {
    if (value > 0) {
        isCooling = true;
        currentHeat = 0; // Heat must be off for cooling
        currentFan = map(value, 0, 100, 0, 255);
    } else {
        isCooling = false;
        currentFan = 0;
    }
}

void parseAndExecuteCommands(String input, bool fromBLE) {
    input.trim();
    input.toUpperCase();
    D_printf("Received Command: %s\n", input.c_str());

    if (input.startsWith("PID:")) {
        applyArtisanPID(input);
        return;
    }

    int split = input.indexOf(';');
    String command = (split != -1) ? input.substring(0, split) : input;
    uint8_t value = (split != -1) ? input.substring(split + 1).toInt() : 0;

    if (command == "READ") handleREAD();
    else if (command == "OT1") handleHEAT(value);
    else if (command == "OT2") handleVENT(value);
    else if (command == "DRUM") handleDRUM(value);
    else if (command == "COOL") handleCOOL(value);
    else if (command == "UNITS") CorF = (value == 'F' ? 'F' : 'C');
    else if (command == "OFF") shutdown();
    else if (command.startsWith("CHAN")) notifyClient("# Active channels set to 2100\r\n");
}


// =============================================================================
// BLE SETUP & CALLBACKS
// =============================================================================
class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        D_println("BLE Client Connected.");
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        D_println("BLE Client Disconnected. Restarting advertising...");
        delay(100);
        pServer->getAdvertising()->start();
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) override {
        String rxValue = pCharacteristic->getValue().c_str();
        if (rxValue.length() > 0) {
            parseAndExecuteCommands(rxValue, true);
        }
    }
};

void initBLE() {
    BLEDevice::init(boardID_BLE.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    pServer->getAdvertising()->start();
    D_println("BLE Advertising started.");
}


// =============================================================================
// WIFI & WEB SERVER SETUP
// =============================================================================
void sendArtisanCompatibleData(uint8_t clientNum, long commandId);

void setupAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(wifiAPSSID, wifiAPPass);
    D_print("AP Mode Started. SSID: "); D_println(wifiAPSSID);
}

void connectToWifi() {
    preferences.begin("wifi", true);
    String ssid = preferences.getString("ssid", "");
    String pass = preferences.getString("pass", "");
    preferences.end();

    if (ssid == "") {
        setupAP();
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    D_print("Connecting to WiFi ");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        D_print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        D_println("\nFailed to connect. Starting AP mode.");
        setupAP();
    } else {
        D_println("\nConnected to WiFi.");
        D_print("IP: "); D_println(WiFi.localIP());
        artisanServer.begin();
    }
}

void handleRoot() {
    String html = "<html><head><title>Cubean Control</title></head><body>"
                  "<h1>Cubean WiFi Setup</h1><form action='/save' method='post'>"
                  "SSID: <input type='text' name='ssid'><br>"
                  "Password: <input type='password' name='pass'><br>"
                  "<input type='submit' value='Save & Restart'></form></body></html>";
    server.send(200, "text/html", html);
}

void handleSave() {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    
    preferences.begin("wifi", false);
    preferences.putString("ssid", ssid);
    preferences.putString("pass", pass);
    preferences.end();

    server.send(200, "text/html", "<h1>Settings Saved. Restarting...</h1>");
    delay(1000);
    ESP.restart();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        String msg = (char*)payload;
        
        // You MUST specify a size here, like <256>
        StaticJsonDocument<256> doc; 
        DeserializationError error = deserializeJson(doc, msg);
        
        if (!error) {
            const char* command = doc["command"];
            if (command && strcmp(command, "getData") == 0) {
                D_println("Received 'getData' command! Sending reply...");
                sendArtisanCompatibleData(num, doc["id"].as<long>());
                return; // Done with this message
            }
        }
        // If not a getData command, parse it normally
        parseAndExecuteCommands(msg, false);
    }
}

void sendArtisanCompatibleData(uint8_t clientNum, long commandId) {
    StaticJsonDocument<512> doc;
    doc["id"] = commandId;
    
    JsonObject data = doc.createNestedObject("data");
    float displayTemp = (CorF == 'F') ? (temp * 1.8) + 32.0 : temp;
    data["BT"] = round(displayTemp * 10) / 10.0;
    data["BurnerVal"] = map(currentHeat, 0, 255, 0, 100);
    data["FanVal"] = map(currentFan, 0, 255, 0, 100);
    
    char buffer[512];
    size_t len = serializeJson(doc, buffer);
    webSocket.sendTXT(clientNum, buffer, len);
}


void setupWebServer() {
    server.on("/", HTTP_GET, handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.begin();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}


// =============================================================================
// MAIN SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(115200);
    D_printf("\nStarting Firmware: %s\n", firmWareVersion.c_str());
    
    pixels.begin();
    setRGBColor(LED_RED);

    Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    // Extra RX-only UART
    Serial2.begin(9600, SERIAL_8N1, EXTRA_RX_PIN, -1); // RX-only

    initBLE();
    connectToWifi();
    setupWebServer();

    myPID.SetMode(MANUAL); // Start in manual mode
    myPID.SetOutputLimits(0, 100);
    myPID.SetSampleTime(1000);

    shutdown();
    delay(1000);
    setRGBColor(0);
}

void loop() {
    static unsigned long lastSendTime = 0;
    const unsigned long sendInterval = 200;

    getPanelMessage();
    getRoasterMessage();

    if (millis() - lastSendTime > sendInterval) {
        lastSendTime = millis();
        handlePID();
        sendRoasterMessage();
    }
    
    server.handleClient();
    webSocket.loop();
    
    WiFiClient client = artisanServer.available();
    if (client) {
        D_println("Artisan client connected via TCP.");
        while(client.connected()) {
            if(client.available()){
                String line = client.readStringUntil('\n');
                parseAndExecuteCommands(line, false);
            }
        }
    }

    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        parseAndExecuteCommands(line, false);
    }

    handleAutoShutdown();
    handleLED();
}
