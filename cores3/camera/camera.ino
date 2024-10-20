#include "M5CoreS3.h"
#include "esp_camera.h"

void setup() {
    auto cfg = M5.config();
    CoreS3.begin(cfg);

    if (!CoreS3.Camera.begin()) {
        Serial.println("Camera Init Fail");
    }
    Serial.println("Camera Init Success");

    CoreS3.Camera.sensor->set_framesize(CoreS3.Camera.sensor, FRAMESIZE_QVGA);
}

void loop() {
    if (CoreS3.Camera.get()) {
        CoreS3.Display.pushImage(0, 0, CoreS3.Display.width(),
                                 CoreS3.Display.height(),
                                 (uint16_t *)CoreS3.Camera.fb->buf);
        CoreS3.Camera.free();
    }
}
