#include <CdpPacket.h>
#include <PapaDuck.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ctime>
#include <arduino-timer.h>
#include <string>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <algorithm>
#include <influx.h>
#include <chrono>
//#include "unishox2.h"

#define LORA_FREQ 915.0 // Frequency Range. Set for US Region 915.0Mhz
#define LORA_TXPOWER 20 // Transmit Power
//// LORA HELTEC PIN CONFIG
#define LORA_MISO 19
#define LORA_CS_PIN 18
#define LORA_MOSI 27
#define LORA_SCK 5
#define LORA_RST 14
#define LORA_IRQ 26
#define ESP32

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_ADDRESS 0x3c


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
// Use pre-built papa duck.
PapaDuck duck;
//DuckDisplay* display = NULL;
//InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
//Point telemetry("Duck Transmissions");
// create a timer with default settings
auto timer = timer_create_default();
const char* ssid = "ASUS-X82U2.4";
const char* pass = "Lotus Born";
const char* mqtt_server = "crash-override";
const int MQTT_CONNECTION_DELAY_MS = 5000;
const int WIFI_CONNECTION_DELAY_MS = 500;
const char* ntpServer = "pool.ntp.org";

WiFiClient wifiClient;
PubSubClient mqttClient(mqtt_server, 1883, wifiClient);


/**
 * @brief Callback method invoked when a packet was received from the mesh
 *
 * @param packet data packet that contains the received message
 */


void post();

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
        if (mqttClient.connect("PAPADUCK1")) {
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
    display.setTextSize(1);
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

    if (mqttClient.publish(toTopicString(packet.topic).c_str(),jsonstat.c_str(), true)) {
        Serial.println("[PAPIDUCK] Packet forwarded:");
        Serial.println(jsonstat.c_str());
        Serial.println("");
        Serial.println("[PAPIDUCK] Publish ok");
        display.println("Publish ok");
        display.display();
    } else {


//        telemetry.clearFields();
//        telemetry.clearTags();
//        telemetry.addTag("DeviceID", nestdoc["Device"]);
//        telemetry.addField("MessageID", muid.c_str());
//        telemetry.addField("BatteryLevel", nestdoc["level"].as<int>());
//        telemetry.addField("voltage", nestdoc["voltage"].as<int>());
//        telemetry.addField("PacketSize", packetSize);
//        telemetry.addField("PayloadSize", payload.size());
//        telemetry.addField("SequenceNum", nestdoc["seqNum"].as<int>());
//        telemetry.addField("satellites", nestdoc["GPS"]["satellites"].as<int>());
//        telemetry.addField("SequenceID", nestdoc["seqID"].as<String>());
//        telemetry.addField("latitude", nestdoc["GPS"]["lat"].as<double>(), 8);
//        telemetry.addField("longitude", nestdoc["GPS"]["lon"].as<double>(), 8);
//        telemetry.addField("altitude", nestdoc["GPS"]["alt"].as<float>());
//        telemetry.addField("speed", nestdoc["GPS"]["speed"].as<float>());
//        telemetry.addField("TransmissionTime", nestdoc["GPS"]["time"].as<unsigned long int>());
//        telemetry.addField("MCUdelay", nestdoc["MCUdelay"].as<unsigned long int>());
//        telemetry.addField("ReceiveDelay", (millis() - start));
//        auto now = std::chrono::system_clock::now();
//        auto duration = now.time_since_epoch();
//        auto nano = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
//        Serial.printf("Time: %lld \n", nano);
//        telemetry.setTime(nano);

//    if (!client.writePoint(telemetry)) {
//        display->drawString(0, 60, "Write Failure");
//        display->sendBuffer();
//        Serial.println(client.getLastErrorMessage());
//    } else {
//        Serial.println("InfluxDB write succeeded");
//        display->drawString(0, 60, "Write Success");
//        display->sendBuffer();
//    }
     }
}

void handleDuckData(std::vector<byte> packetBuffer) {
    auto packet = CdpPacket(packetBuffer);
    auto packetSize = packetBuffer.size();
    std::string sduid(packet.sduid.begin(), packet.sduid.end());
    packet.data.shrink_to_fit();

    uint32_t packet_data_crc = packet.dcrc;
    uint32_t computed_data_crc = CRC32::calculate(packet.data.data(), packet.data.size());
    Serial.printf("Packet_Data: %s \n",packet.data.data());
    //Serial.printf("Computed_Data: %s \n",data_section.data());
    Serial.printf("Packet_Data_CRC: %u \n",packet_data_crc);
    Serial.printf("Computed_Data_CRC: %u \n",computed_data_crc);
    Serial.println("[PAPI] got packet: " +
                   convertToHex(packetBuffer.data(), packetBuffer.size()));
    if (String(packet_data_crc) != String(computed_data_crc)) {
        logerr("data crc mismatch: received: " + String(packet_data_crc) +
               " calculated:" + String(computed_data_crc));
//        InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
//        Point telemetry("Corrupt Duck Transmissions");
//        telemetry.addTag("DeviceID",sduid.c_str());
//        telemetry.addField("PacketSize",packetSize);
//        telemetry.addField("PayloadSize",packet.data.size());
//        telemetry.addField("Packet_Data_CRC",packet_data_crc);
//        telemetry.addField("Computed_Data_CRC",computed_data_crc);
//        if (!client.writePoint(telemetry)) {
//            Serial.println(client.getLastErrorMessage());
//        }
//        else {
//            Serial.println("InfluxDB write succeeded: ");
//        }
    } else
        quackJson(packetBuffer);
}
void setup() {
    //duck.enableAcks(true);
    Wire.begin(OLED_SDA, OLED_SCL);
    std::string deviceId("PAPADUCK");
    std::vector<byte> devId;
    devId.insert(devId.end(), deviceId.begin(), deviceId.end());
    display.begin(SSD1306_SWITCHCAPVCC,SCREEN_ADDRESS);
   // display = DuckDisplay::getInstance();
    // DuckDisplay instance is returned unconditionally, if there is no physical
    // display the functions will not do anything
   // display->setupDisplay(duck.getType(), devId);
    // the default setup is equivalent to the above setup sequence
    duck.setupSerial(115200);
    //SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS_PIN);
    duck.setupRadio(LORA_FREQ, LORA_CS_PIN, LORA_TXPOWER);
    duck.setDeviceId(devId);
    setup_wifi();
    duck.setupDns();
    configTime(0, 0, ntpServer,"time.nist.gov");
    duck.onReceiveDuckData(handleDuckData);

   // client.begin("192.168.1.74",wifiClient);
//    mqttClient.setServer(mqtt_server, 1883);
//    mqttClient.setCallback(callback);
    //mqttClient.setKeepAlive(30);
    Serial.print("[PAPI] Setup OK!");

    display.display();
}