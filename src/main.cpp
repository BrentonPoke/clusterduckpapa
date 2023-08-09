#include <CdpPacket.h>
#include <PapaDuck.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ctime>
#include <arduino-timer.h>
#include <string>
#include <influx.h>
#include <chrono>
#include <Adafruit_SSD1306.h>
//#include "unishox2.h"

#define OLED_RESET     -1
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3c



// Use pre-built papa duck.
PapaDuck duck;
//DuckDisplay* display = NULL;
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
Point telemetry("TBeam Transmissions");
// create a timer with default settings
auto timer = timer_create_default();
//const char* ssid = "ASUS-X82U2.4";
//const char* pass = "Lotus Born";
const char* ssid = "CIT-IOT";
const char* pass = ".P<N~FgCu0a/_";
const char* mqtt_server = "35.7.120.10";
const int MQTT_CONNECTION_DELAY_MS = 5000;
const int WIFI_CONNECTION_DELAY_MS = 500;
const char* ntpServer = "pool.ntp.org";



WiFiClient wifiClient;
PubSubClient mqttClient(mqtt_server, 1883, wifiClient);

#ifdef ARDUINO_TBeam
#include <DuckLogger.h>
#define XPOWERS_CHIP_AXP192
#include <XPowersLib.h>
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
XPowersPMU PMU;
bool heartbeat(void*) {
    telemetry.clearFields();
    telemetry.clearTags();
    telemetry.addTag("DeviceID", "PAPADUCK");
    telemetry.addField("voltage", PMU.getBattVoltage());
    telemetry.addField("percentage", PMU.getBatteryPercent());
    if (!client.writePoint(telemetry)) {
        display.println("Heartbeat Failed");
        display.print("Voltage: ");
        display.print(PMU.getBattVoltage());
        display.println(" mV");
        display.display();
        logerr(client.getLastErrorMessage());
    } else {
        loginfo("Heartbeat write succeeded");
        display.print("Voltage: ");
        display.print(PMU.getBattVoltage());
        display.println(" mV");
        display.println("Heartbeat Success");
        display.display();
    }
    return true;
}
#endif

String duckTypeToString(int duckType);

/**
 * @brief Establish the connection to the wifi network the Papa Duck can reach
 *
 */
void setup_wifi() {
    Serial.println();
    Serial.print("[PAPI] Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        delay(WIFI_CONNECTION_DELAY_MS);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("[PAPI] WiFi connected");
    Serial.println("[PAPI] IP address: "+WiFi.localIP().toString());
}
/**
 * @brief Invoked by the MQTT server when a message has been received
 */
void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("[PAPI] Message arrived on topic: ");
    Serial.print(topic);
}
/**
 * @brief Periodically attempts to re-establish the MQTT connection
 *
 */
void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("[PAPI] Attempting MQTT connection...");
        if (mqttClient.connect("PAPADUCK")) {
            Serial.println("[PAPI] connected");
            mqttClient.subscribe("status");
        } else {
            Serial.print("[PAPI] failed, rc=");
            Serial.println("[PAPI] try again in 5 seconds");
            delay(MQTT_CONNECTION_DELAY_MS);
        }
    }
}
void loop() {
//    if (!mqttClient.connected()) {
//        reconnect();
//    }
//    mqttClient.loop();
    timer.tick();
    duck.run();
}

// DMS locator URL requires a topicString, so we need to convert the topic
// from the packet to a string based on the topics code
std::string toTopicString(byte topic) {

    std::string topicString;

    switch (topic) {
        case topics::status:
            topicString = "status";
            break;
        case topics::cpm:
            topicString = "portal";
            break;
        case topics::sensor:
            topicString = "sensor";
            break;
        case topics::alert:
            topicString = "alert";
            break;
        case topics::location:
            topicString = "gps";
            break;
        case topics::health:
            topicString = "health";
            break;
        default:
            topicString = "status";
    }

    return topicString;
}

String convertToHex(byte* data, unsigned int size) {
    String buf = "";
    buf.reserve(size * 2); // 2 digit hex
    const char* cs = "0123456789ABCDEF";
    for (int i = 0; i < size; i++) {
        byte val = data[i];
        buf += cs[(val >> 4) & 0x0F];
        buf += cs[val & 0x0F];
    }
    return buf;
}

/**
 * @brief Convert received packet into a JSON object we can send over the MQTT connection
 *
 * @param packet A Packet that contains the received message
 */
void quackJson(const std::vector<byte>& packetBuffer) {
    auto start = millis();
    auto packetSize = packetBuffer.size();
    CdpPacket packet = CdpPacket(packetBuffer);
    DynamicJsonDocument doc(350);
    // Here we treat the internal payload of the CDP packet as a string
    // but this is mostly application dependent.
    // The parsingf here is optional. The Papa duck could simply decide to
    // forward the CDP packet as a byte array and let the Network Server (or DMS) deal with
    // the parsing based on some business logic.

    std::string payload(packet.data.begin(), packet.data.end());
    std::string sduid(packet.sduid.begin(), packet.sduid.end());
    std::string dduid(packet.dduid.begin(), packet.dduid.end());

    std::string muid(packet.muid.begin(), packet.muid.end());

    Serial.println("[PAPI] Packet Received:");
    Serial.println("[PAPI] sduid:   " + String(sduid.c_str()));
    Serial.println("[PAPI] dduid:   " + String(dduid.c_str()));
    Serial.println("[PAPI] topic:   " + String(toTopicString(packet.topic).c_str()));
    Serial.println("[PAPI] muid:    " + String(muid.c_str()));
    Serial.println("[PAPI] data:    " + String(payload.c_str()));
    Serial.println("[PAPI] duck:    " + String(packet.duckType));


    //test of EMS ideas...

    DynamicJsonDocument nestdoc(229);
    //char decompress[229];
    Serial.printf("Payload: %s\n",payload.c_str());
    Serial.printf("Payload Size: %d\n", payload.size());
    //unishox2_decompress_simple(payload,int(payload.length()),decompress);
    deserializeJson(nestdoc, packet.data.data());
    doc["Payload"] = nestdoc;
    doc["PacketSize"] = packetSize;
    doc["PayloadSize"] = payload.size();
    doc["ReceiveDelay"] = millis() - start;
    //doc["duckType"].set(packet.duckType);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("New Message");
    display.println(sduid.c_str());
    display.println(muid.c_str());
    display.println(toTopicString(packet.topic).c_str());
    display.display();

    String jsonstat;
    serializeJson(doc,jsonstat);

    Serial.println(jsonstat);

    //if (mqttClient.publish(cdpTopic.c_str(),jsonstat.c_str(), true)) {
    //    Serial.println("[PAPIDUCK] Packet forwarded:");
    //    Serial.println(jsonstat.c_str());
    //    Serial.println("");
    //    Serial.println("[PAPIDUCK] Publish ok");
    //    display->drawString(0, 60, "Publish ok");
    //    display->sendBuffer();
    //} else {


        telemetry.clearFields();
        telemetry.clearTags();
        telemetry.addTag("DeviceID", nestdoc["Device"]);
        telemetry.addField("MessageID", muid.c_str());
        telemetry.addField("BatteryLevel", nestdoc["level"].as<int>());
        telemetry.addField("voltage", nestdoc["Voltage"].as<int>());
        telemetry.addField("PacketSize", packetSize);
        telemetry.addField("PayloadSize", payload.size());
        telemetry.addField("SequenceNum", nestdoc["seqNum"].as<int>());
        telemetry.addField("satellites", nestdoc["GPS"]["satellites"].as<int>());
        telemetry.addField("SequenceID", nestdoc["seqID"].as<String>());
        telemetry.addField("latitude", nestdoc["GPS"]["lat"].as<double>(), 8);
        telemetry.addField("longitude", nestdoc["GPS"]["lon"].as<double>(), 8);
        telemetry.addField("altitude", nestdoc["GPS"]["alt"].as<float>());
        telemetry.addField("speed", nestdoc["GPS"]["speed"].as<float>());
        telemetry.addField("TransmissionTime", nestdoc["GPS"]["time"].as<unsigned long int>());
        telemetry.addField("MCUdelay", nestdoc["MCUdelay"].as<unsigned long int>());
        telemetry.addField("ReceiveDelay", (millis() - start));
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto nano = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
        Serial.printf("Time: %lld \n", nano);
        telemetry.setTime(nano);
//    if (!client.writePoint(telemetry)) {
//        display->drawString(0, 60, "Write Failure");
//        display->sendBuffer();
//        Serial.println(client.getLastErrorMessage());
//    }
//    else
//    {
//        Serial.print("InfluxDB write succeeded: ");
//        display->drawString(0, 60, "Write Success");
//        display->sendBuffer();
//        Serial.print("InfluxDB write succeeded: ");
//    }
    if (!client.writePoint(telemetry)) {
        display.println("Write Failure");
        display.display();
        Serial.println(client.getLastErrorMessage());
    } else {
        Serial.println("InfluxDB write succeeded");
        display.println("Write Success");
        display.display();
    }
    // }
}

void handleDuckData(std::vector<byte> packetBuffer) {
        quackJson(packetBuffer);
}
void setup() {
    //duck.enableAcks(true);
    std::string deviceId("TBEAM");
    std::vector<byte> devId;
    devId.insert(devId.end(), deviceId.begin(), deviceId.end());

#ifdef ARDUINO_TBeam
    bool result = PMU.begin(Wire, AXP192_SLAVE_ADDRESS, 21, 22);

    if (!result) {
        logdbg("PMU is not online...");
    } else {
        logdbg("PMU online");
    }

    timer.every(600000, heartbeat);

#endif

    //display = DuckDisplay::getInstance();
    // DuckDisplay instance is returned unconditionally, if there is no physical
    // display the functions will not do anything
    //display->setupDisplay(duck.getType(), devId);
    // the default setup is equivalent to the above setup sequence
    duck.setupSerial(115200);
    duck.setupRadio();
    duck.setDeviceId(devId);
    setup_wifi();
    duck.setupDns();
    configTime(0, 0, ntpServer,"time.nist.gov");
    duck.onReceiveDuckData(handleDuckData);

    pinMode(37, INPUT);
   // client.begin("192.168.1.74",wifiClient);
    //mqttClient.setServer(mqtt_server, 1883);
//    mqttClient.setCallback(callback);
    //mqttClient.setKeepAlive(30);
    Serial.print("[PAPI] Setup OK!");
    display.begin(SSD1306_SWITCHCAPVCC,SCREEN_ADDRESS);

    display.display();
    delay(3000);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Clusterduck Protocol");
    display.println("DT: "+duckTypeToString(duck.getType()));
    display.println(String("v")+duckutils::getCDPVersion().c_str());
    display.println("----------------");
    display.println(String("ID: ")+deviceId.c_str());
    display.display();
}

String duckTypeToString(int duckType) {
    String duckTypeStr = "";
    switch (duckType) {
    case DuckType::PAPA:
        duckTypeStr = "Papa";
    case DuckType::LINK:
        duckTypeStr = "Link";
    case DuckType::DETECTOR:
        duckTypeStr = "Detector";
    case DuckType::MAMA:
        duckTypeStr = "Mama";
    default:
        duckTypeStr = "Duck";
    }
    return duckTypeStr;
}