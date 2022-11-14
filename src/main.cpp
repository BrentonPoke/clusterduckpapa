#include <CdpPacket.h>
#include <PapaDuck.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <arduino-timer.h>
#include <string>
#include <algorithm>
#include <influx.h>
//#include "unishox2.h"

#define LORA_FREQ 915.0 // Frequency Range. Set for US Region 915.0Mhz
#define LORA_TXPOWER 20 // Transmit Power
// LORA HELTEC PIN CONFIG
#define LORA_CS_PIN 18
#define LORA_DIO0_PIN 26
#define LORA_DIO1_PIN -1 // unused
#define LORA_RST_PIN 14



// Use pre-built papa duck.
PapaDuck duck;
DuckDisplay* display = NULL;

// create a timer with default settings
auto timer = timer_create_default();
//const char* user = "CIT-IOT";
//const char* pass = "xup|VgbV4^i#E";
const char* user = "ASUS-X82U2.4";
const char* pass = "Lotus Born";
const char* mqtt_server = "ritter";
const int MQTT_CONNECTION_DELAY_MS = 5000;
const int WIFI_CONNECTION_DELAY_MS = 500;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);



/**
 * @brief Callback method invoked when a packet was received from the mesh
 *
 * @param packet data packet that contains the received message
 */


/**
 * @brief Establish the connection to the wifi network the Papa Duck can reach
 *
 */
void setup_wifi() {
    Serial.println();
    Serial.print("[PAPI] Connecting to ");
    Serial.println(user);
    WiFi.begin(user, pass);
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
        if (mqttClient.connect("ESP32Client")) {
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
        case topics::location:
            topicString = "gps";
            break;
        case topics::health:
            topicString = "health";
            break;
        default:
            topicString = "TTGO";
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

    std::string cdpTopic = toTopicString(packet.topic);

    //test of EMS ideas...

    DynamicJsonDocument nestdoc(229);
    //char decompress[229];
    Serial.printf("Payload: %s\n",payload.c_str());
    Serial.printf("Payload Size: %d\n", payload.size());
    //unishox2_decompress_simple(payload,int(payload.length()),decompress);
    deserializeJson(nestdoc, payload);
    auto currentMillis = millis() - start;
    doc["DeviceID"] = nestdoc["Device"];
    doc["MessageID"] = muid;
    doc["Payload"] = nestdoc;
    doc["PacketSize"] = packetSize;
    doc["PayloadSize"] = payload.size();
    doc["TXTime"] = nestdoc["GPS"]["time"].as<unsigned long>()*1000L + currentMillis;
    doc["duckType"].set(packet.duckType);
    doc["topic"].set(gps);
    display->clear();
    display->drawString(0, 10, "New Message");
    display->drawString(0, 20, sduid.c_str());
    display->drawString(0, 30, muid.c_str());
    display->drawString(0, 40, cdpTopic.c_str());
    display->sendBuffer();

    std::string jsonstat;
    serializeJson(doc,jsonstat);

    Serial.println(jsonstat.c_str());

    InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
    Point telemetry("Duck Transmissions");

    telemetry.clearFields();
    telemetry.addTag("DeviceID",nestdoc["Device"]);
    telemetry.addTag("Gender",nestdoc["G"]);
    telemetry.addField("MessageID",muid.c_str());
    telemetry.addField("PacketSize",packetSize);
    telemetry.addField("PayloadSize",payload.size());
    telemetry.addField("SequenceNum",nestdoc["seqNum"].as<int>());
    telemetry.addField("satellites",nestdoc["satellites"].as<int>());
    telemetry.addField("SequenceID",nestdoc["seqID"].as<String>());
    telemetry.addField("latitude",nestdoc["GPS"]["lat"].as<double>());
    telemetry.addField("longitude",nestdoc["GPS"]["lon"].as<double>());
    telemetry.addField("altitude",nestdoc["GPS"]["alt"].as<float>());
    telemetry.addField("speed",nestdoc["GPS"]["speed"].as<float>());
    telemetry.addField("TransmissionTime",nestdoc["GPS"]["time"].as<unsigned long>() + (millis() - start));
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

    if (mqttClient.publish("gps",jsonstat.c_str(), true)) {
        Serial.println("[PAPIDUCK] Packet forwarded:");
        Serial.println(jsonstat.c_str());
        Serial.println("");
        Serial.println("[PAPIDUCK] Publish ok");
        display->drawString(0, 60, "Publish ok");
        display->sendBuffer();
    } else if (!client.writePoint(telemetry)) {
        display->drawString(0, 60, "Write Failure");
        display->sendBuffer();
        Serial.println(client.getLastErrorMessage());
    }
    else {
        Serial.println("InfluxDB write succeeded: ");
        display->drawString(0, 60, "Write Success");
        display->sendBuffer();
    }
   // else {
   //     Serial.println("[PAPIDUCK] Publish failed");
   //     display->drawString(0, 60, "Publish failed");
   //     display->sendBuffer();
   // }
}

void handleDuckData(std::vector<byte> packetBuffer) {
    auto packet = CdpPacket(packetBuffer);
    auto packetSize = packetBuffer.size();
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
        InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
        Point telemetry("Corrupt Duck Transmissions");
        telemetry.addTag("DeviceID",packet.sduid.data());
        telemetry.addField("PacketSize",packetSize);
        telemetry.addField("PayloadSize",packet.data.size());
        telemetry.addField("Packet_Data_CRC",packet_data_crc);
        telemetry.addField("Computed_Data_CRC",computed_data_crc);
        if (!client.writePoint(telemetry)) {
            Serial.println(client.getLastErrorMessage());
        }
        else {
            Serial.println("InfluxDB write succeeded: ");
        }
    } else
        quackJson(packetBuffer);
}
void setup() {
    //duck.enableAcks(true);
    std::string deviceId("PAPADUCK");
    std::vector<byte> devId;
    devId.insert(devId.end(), deviceId.begin(), deviceId.end());
    display = DuckDisplay::getInstance();
    // DuckDisplay instance is returned unconditionally, if there is no physical
    // display the functions will not do anything
    display->setupDisplay(duck.getType(), devId);
    // the default setup is equivalent to the above setup sequence
    duck.setupSerial(115200);
    duck.setupRadio(LORA_FREQ, LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN,
                    LORA_DIO1_PIN, LORA_TXPOWER);
    duck.setDeviceId(devId);


    duck.onReceiveDuckData(handleDuckData);
    setup_wifi();
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(callback);
    //mqttClient.setKeepAlive(30);
    Serial.print("[PAPI] Setup OK!");
    display->showDefaultScreen();
}
