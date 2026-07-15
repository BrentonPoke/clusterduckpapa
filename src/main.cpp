#include <CdpPacket.h>
#include <Ducks/PapaDuck.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <arduino-timer.h>
#include <string>
#include <Wire.h>
#include <AdafruitDisplay.h>
#include <influx.h>
#include <chrono>
#include <vector>
//#include "unishox2.h"

#define LORA_FREQ 915.0 // Frequency Range. Set for US Region 915.0Mhz
#define LORA_TXPOWER 20 // Transmit Power
//// LORA HELTEC PIN CONFIG

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3c


//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
// Use pre-built papa duck.
PapaDuck duck("PAPADUCK");
AdafruitDisplay screen(SCREEN_WIDTH, SCREEN_HEIGHT,-1,DuckType::PAPA);
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
Point telemetry("Duck Transmissions");
// create a timer with default settings
auto timer = timer_create_default();
const char* ssid = "ASUS-X82U2.4";
const char* pass = "Lotus Born";
const char* mqtt_server = "192.168.1.74";
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
    if (!mqttClient.connected()) {
        reconnect();
    }
   mqttClient.loop();
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
        case topics::gps:
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
void quackJson(const CdpPacket& packet) {
    auto start = millis();
    size_t packetSize = sizeof(CdpPacket);
    JsonDocument doc;
    // Here we treat the internal payload of the CDP packet as a string
    // but this is mostly application dependent.
    // The parsingf here is optional. The Papa duck could simply decide to
    // forward the CDP packet as a byte array and let the Network Server (or DMS) deal with
    // the parsing based on some business logic.

    std::string payload(packet.data.begin(), packet.data.end());
    std::string sduid(packet.sduid.begin(), packet.sduid.end());
    std::string dduid(packet.dduid.begin(), packet.dduid.end());

    std::string muid(packet.muid.begin(), packet.muid.end());

    loginfo_ln("[PAPI] Packet Received:");
    loginfo_ln("[PAPI] sduid:   %s" , sduid.c_str());
    loginfo_ln("[PAPI] dduid:   %s" , dduid.c_str());
    loginfo_ln("[PAPI] topic:   %s" , toTopicString(packet.topic));
    loginfo_ln("[PAPI] muid:    %s" , muid.c_str());
    loginfo_ln("[PAPI] data:    %s" , payload.c_str());
    loginfo_ln("[PAPI] duck:    %d" , DuckDisplay<AdafruitDisplay>::duckTypeToString(static_cast<DuckType>(packet.duckType)));


    //test of EMS ideas...

    JsonDocument nestdoc;
    //char decompress[229];
    Serial.printf("Payload: %s\n",payload.c_str());
    Serial.printf("Payload Size: %d\n", payload.size());
    //unishox2_decompress_simple(payload,int(payload.length()),decompress);
    deserializeJson(nestdoc, packet.data.data());
    doc["Payload"] = nestdoc;
    doc["PacketSize"] = packetSize;
    doc["PayloadSize"] = payload.size();
    doc["ReceiveDelay"] = millis() - start;
    doc["duckType"]  = packet.duckType;
    screen.clear();
    screen.display.println("New Message");
    screen.display.println(sduid.c_str());
    screen.display.println(muid.c_str());
    screen.display.println(toTopicString(packet.topic).c_str());
    screen.display.display();

    String jsonstat;
    serializeJson(doc,jsonstat);

    Serial.println(jsonstat);

    if (mqttClient.publish(toTopicString(packet.topic).c_str(),jsonstat.c_str(), true)) {
        Serial.println("[PAPIDUCK] Packet forwarded:");
        Serial.println(jsonstat.c_str());
        Serial.println("");
        Serial.println("[PAPIDUCK] Publish ok");
        screen.display.println("Publish ok");
        screen.display.display();
    } else {


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

    if (!client.writePoint(telemetry)) {
        screen.display.println("Write Failure");
        screen.display.display();
        Serial.println(client.getLastErrorMessage());
    } else {
        Serial.println("InfluxDB write succeeded");
        screen.display.println("Write Success");
        screen.display.display();
    }
     }
}

void handleDuckData(CdpPacket packetBuffer) {
        quackJson(packetBuffer);
}
void setup() {
    //duck.enableAcks(true);
    //Wire.begin(OLED_SDA, OLED_SCL);
    std::string deviceId("PAPADUCK");
    //display.begin(SSD1306_SWITCHCAPVCC,SCREEN_ADDRESS);
    // DuckDisplay instance is returned unconditionally, if there is no physical
    // display the functions will not do anything
    // the default setup is equivalent to the above setup sequence
    duck.setupWithDefaults();
    //SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS_PIN);
    setup_wifi();
    configTime(0, 0, ntpServer,"time.nist.gov");
    duck.onReceiveDuckData(handleDuckData);
    pinMode(37, INPUT);
    //client.begin("192.168.1.74",wifiClient);
    mqttClient.setServer(mqtt_server, 1883);
//    mqttClient.setCallback(callback);
    //mqttClient.setKeepAlive(30);
    loginfo_ln("[PAPI] Setup OK!");
    screen.launch();
    screen.showDefaultScreen();

}