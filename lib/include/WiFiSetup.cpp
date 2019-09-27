//
// Created by Zhenyi Ye on 2019-08-27.
//

#include "WiFiSetup.h"

// Server Info
char python_server[40] = "192.168.86.96";
char python_port[6] = "9999";

// init flag
bool init_flag = true;

//flag for saving data
bool shouldSaveConfig = false;

// Callback to save configuration
void saveConfigCallback () {
    Serial.println("Should save config");
    shouldSaveConfig = true;
}

// Function to read configuration from file
void readConfiguration(){
    //read configuration from FS json
    Serial.println("mounting FS...");

    if (SPIFFS.begin()) {
        Serial.println("mounted file system");
        if (SPIFFS.exists("/config.json")) {
            //file exists, reading and loading
            Serial.println("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile) {
                Serial.println("opened config file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes(buf.get(), size);
                DynamicJsonBuffer jsonBuffer;
                JsonObject& json = jsonBuffer.parseObject(buf.get());
                json.printTo(Serial);
                if (json.success()) {
                    Serial.println("\nparsed json");

                    strcpy(python_server, json["python_server"]);
                    strcpy(python_port, json["python_port"]);

                } else {
                    Serial.println("failed to load json config");
                }
                configFile.close();
            }
        }
    } else {
        Serial.println("failed to mount FS");
        return;
    }
}

// Function to Configure WiFi
void configWifi(){

    AsyncWiFiManagerParameter custom_python_server("server", "python server", python_server, 40);
    AsyncWiFiManagerParameter custom_python_port("port", "python port", python_port, 6);

    // Init Wifi
    AsyncWebServer server(80);
    DNSServer dns;

    AsyncWiFiManager wifi_manager(&server, &dns);

    //set config save notify callback
    wifi_manager.setSaveConfigCallback(saveConfigCallback);

    //add parameters
    wifi_manager.addParameter(&custom_python_server);
    wifi_manager.addParameter(&custom_python_port);


    if (init_flag)
        {
            if(!wifi_manager.autoConnect("IMU Measurement Exp")) {
                Serial.println("failed to connect and hit timeout");
                delay(3000);
                //reset and try again, or maybe put it to deep sleep
                ESP.restart();
                delay(5000);
            }
            init_flag = false;

        }
    else {
            if(!wifi_manager.startConfigPortal("IMU Measurement Exp")) {
                Serial.println("failed to connect and hit timeout");
                delay(3000);
                //reset and try again, or maybe put it to deep sleep
                ESP.restart();
                delay(5000);
        }
    }

    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");

    //read custom values
    strcpy(python_server, custom_python_server.getValue());
    strcpy(python_port, custom_python_port.getValue());

    //save the custom parameters to FS
    if (shouldSaveConfig) {
        Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["python_server"] = python_server;
        json["python_port"] = python_port;

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
            Serial.println("failed to open config file for writing");
        }

        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
        shouldSaveConfig = false;
        //end save
    }

    Serial.println("local ip");
    Serial.println(WiFi.localIP());
}
