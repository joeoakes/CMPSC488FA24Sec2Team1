#include <WiFi.h>
#include "esp_camera.h"
#include <M5CoreS3.h>

#define WIFI_SSID     "bruh"
#define WIFI_PASSWORD "bruh1234"
#define SERVER_PORT   80

IPAddress ip;
WiFiServer server(SERVER_PORT);

void setup(void) {
    CoreS3.begin();

    CoreS3.Display.println("WiFi Connecting...");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(500);
    }
    CoreS3.Display.println("WiFi Connected.");
    Serial.println("\r\n WiFi Connected.");
    CoreS3.Display.printf("Connected to %s\n", WIFI_SSID);

    ip = WiFi.localIP();
    CoreS3.Display.println(ip);

    server.begin();
    CoreS3.Display.printf("Server started on port %d\n", SERVER_PORT);

    if (!CoreS3.Camera.begin()) {
        CoreS3.Display.println("Camera Init Fail");
    }
    CoreS3.Display.println("Camera Init Success");

    CoreS3.Camera.sensor->set_framesize(CoreS3.Camera.sensor, FRAMESIZE_QVGA);
}

void loop(void) {
    // listen for incoming clients
    WiFiClient conn = server.available();
    if (conn) {
        if (conn.connected()) {
            CoreS3.Display.println("Connected to client");
        }

        // conn.write("I LOVE 488\n");

        unsigned long time = millis();
        CoreS3.Camera.free();
        CoreS3.Display.printf("Time: %d\n", millis() - time);
        time = millis();
        if (!CoreS3.Camera.get()) {
            CoreS3.Display.println("Failed to get picture");
            conn.stop();
            return;
        }
        CoreS3.Display.printf("Time: %d\n", millis() - time);
        time = millis();

        uint8_t *out_jpg   = NULL;
        size_t out_jpg_len = 0;
        // frame2jpg(CoreS3.Camera.fb, 255, &out_jpg, &out_jpg_len);
        CoreS3.Display.printf("Time: %d\n", millis() - time);
        time = millis();

        // conn.write(out_jpg, out_jpg_len);
        conn.write(CoreS3.Camera.fb);
        CoreS3.Display.printf("Time: %d\n", millis() - time);
        time = millis();
        // free(out_jpg);
        CoreS3.Display.println("Image sent");

        // while (conn.connected()) {
        //     while (conn.available() > 0) {
                // read data from the connected client
                // CoreS3.Display.write(conn.read()); 
            // }
            //Send Data to connected client
            // while (Serial.available() > 0) {
            //     conn.write(Serial.read());
            // }
        // }
        conn.stop();
        Serial.println("Client disconnected");
    }
}
