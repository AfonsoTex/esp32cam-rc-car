#include "config.h"

#include <stdio.h>        // printf / sprintf
#include "nvs_flash.h"    // NVS partition init (non-volatile storage in flash)
#include "nvs.h"          // NVS read/write API for WiFi credentials
#include <WiFi.h>         // STA/AP mode, network scan, TCP sockets
#include "esp_camera.h"   // OV2640 driver and JPEG frame capture

// ── OV2640 camera pin map (AI-Thinker ESP32-CAM)
// The ESP32-CAM board carries the ESP32-S module and the OV2640 sensor
// wired together on the same PCB.
// These pin numbers describe how the OV2640 sensor is physically wired
// to the ESP32 on this board. The wiring is fixed in the PCB and cannot
// be changed. The camera driver cannot detect it, so it must be told.
// Wrong number here = driver talks to the wrong pin = camera never responds.
// Source: ESP32_CAM_V1.6 schematic (docs/ESP32_CAM_V1_6.pdf).
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


// Layout version of the credentials record saved in NVS. Bump by hand
// when the struct fields below change, so old records can be told apart.
// (Written on save; not yet checked on read.)
#define WIFI_STORAGE_VERSION 1

typedef struct {
    uint8_t version;
    char ssid[32];
    char password[64];
} my_wifi_credentials_t;


// Debug only: prints every WiFi network stored in flash.
void dump_nvs_to_serial() {
    nvs_handle_t h;
    uint32_t count = 0;

    if (nvs_open("wifi_storage_id", NVS_READONLY, &h) != ESP_OK) {
        Serial.println("[NVS] Namespace empty or not found");
        return;
    }
    nvs_get_u32(h, "wifi_count", &count);
    Serial.printf("[NVS] Total networks: %lu\n", count);

    for (uint32_t i = 0; i < count; i++) {
        char key[16];
        sprintf(key, "wifi_%lu", i);
        my_wifi_credentials_t net;
        size_t size = sizeof(my_wifi_credentials_t);
        if (nvs_get_blob(h, key, &net, &size) == ESP_OK) {
            Serial.printf("[NVS] [%lu] SSID: %s | PASS: %s\n", i, net.ssid, net.password);
        }
    }
    nvs_close(h);
}

// Fixed size buffer: String/malloc would fragment the heap over thousands of
// iterations, and the camera needs large contiguous blocks per JPEG frame.
char buffer[128];

// IP of the PC running the Python servers. The ESP32 is the client:
// on boot it connects out to this address, so it must know it up front.
// Set this in config.h — it changes with your network.
const char *destino = DESTINO_IP;

// Port where comandos.py listens. Video uses 1884 (see camara.py).
// IP picks the machine, port picks which program on it.
#define SERVER_PORT 1883
#define VIDEO_PORT 1884

// Last time any data arrived from the PC. Silence for >1 s stops the
// motors; >5 s drops the TCP connection.
unsigned long lastHeartbeat = 0;
unsigned long wifiLostTimestamp = 0;
bool trackingLostWifi = false;
unsigned long lastReconnectAttempt = 0;

// TCP server, used ONLY in Access Point mode (initial WiFi setup).
// With no known network, the ESP32 becomes the AP "ESP32_CAM_AFONSO".
// You connect to it and send "WIFI:ssid,password\n" to this port to
// store a new network in NVS flash.
WiFiServer ESPserver(1883);

// Normal mode: the ESP32 is the client on both connections.
WiFiClient client;         // commands from comandos.py, port 1883
WiFiClient clienteVideo;   // JPEG frames to camara.py, port 1884


// clock frequency, pixel format, frame size, number of frame buffers.
// Declared empty here, filled field by field in setup(), then handed to
// esp_camera_init(&config).
camera_config_t config;

// ── H-bridge (L293D) ──────────────────────────────────────────────
// EN pins carry PWM and set how much power each side gets.
// IN pins are digital only and set the direction, shared by both motors:
// both go forward or both go backward, never one each way.
// Steering comes from feeding the two sides different PWM duty.
const int EN1_pin = 14;   // PWM, left side
const int EN2_pin = 15;   // PWM, right side
const int IN1_pin = 12;   // direction pair: HIGH/LOW = forward
const int IN2_pin = 13;   //                 LOW/HIGH = backward

// LEDC hardware channels for the two PWM outputs.
const int PWM_CHANNEL_1 = 0;
const int PWM_CHANNEL_2 = 1;

void stop_motors() {
    ledcWrite(EN1_pin, 0);
    ledcWrite(EN2_pin, 0);
}

void pinos_setup() {
    pinMode(IN1_pin, OUTPUT);
    pinMode(IN2_pin, OUTPUT);

    digitalWrite(IN1_pin, LOW);
    digitalWrite(IN2_pin, LOW);
}

void pwm_channel_setup() {
    // ledcAttachChannel also sets the pin as output, so no pinMode needed.
    ledcAttachChannel(EN1_pin, 1000, 8, PWM_CHANNEL_1);
    ledcAttachChannel(EN2_pin, 1000, 8, PWM_CHANNEL_2);
}

void setup_flash_memory() {
    esp_err_t return_error_value = nvs_flash_init();
    if (return_error_value == ESP_ERR_NVS_NO_FREE_PAGES || return_error_value == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

// Credentials of the config network the ESP32 creates when it has no
// known WiFi. Set in config.h.
const char *ap_ssid     = AP_SSID;
const char *ap_password = AP_PASSWORD;

void enable_access_point() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_password);
    Serial.println("\n--- ACCESS POINT MODE ---");
    Serial.print("IP: "); Serial.println(WiFi.softAPIP());
    ESPserver.begin();
}


// Stores one WiFi network in NVS flash. Called from AP mode with the
// ssid and password parsed from the WIFI: command.
// Fills the struct, reads the current count, saves the record under key
// "wifi_<count>", then increments the count. Numbered keys let several
// networks be stored and read back later by dump / the scan on boot.
void Write_to_flash(char *ssid, char *password) {
    nvs_handle_t my_handle;
    my_wifi_credentials_t my_network;
    uint32_t count;

    memset(&my_network, 0, sizeof(my_wifi_credentials_t));
    my_network.version = WIFI_STORAGE_VERSION;

    strncpy(my_network.ssid, ssid, sizeof(my_network.ssid) - 1);
    strncpy(my_network.password, password, sizeof(my_network.password) - 1);

    esp_err_t err = nvs_open("wifi_storage_id", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        Serial.println("Error opening NVS handle!");
        return;
    }

    err = nvs_get_u32(my_handle, "wifi_count", &count);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        count = 0;
        nvs_set_u32(my_handle, "wifi_count", count);
    }

    char key[16];
    sprintf(key, "wifi_%lu", count);

    err = nvs_set_blob(my_handle, key, &my_network, sizeof(my_wifi_credentials_t));
    if (err == ESP_OK) {
        count++;
        nvs_set_u32(my_handle, "wifi_count", count);
        nvs_commit(my_handle);
    } else {
        Serial.printf("Error writing blob: %s\n", esp_err_to_name(err));
    }
    dump_nvs_to_serial();
    nvs_close(my_handle);
}


// Loop that runs in AP mode. Reads and handles the commands you send
// from Packet Sender. Infinite: only the ESP.restart() below leaves it.
// WIFI:ssid,password -> Write_to_flash() stores the network
// RESET:NOW          -> reboots the chip
void loop_config_mode() {
    Serial.println("Entering infinite config loop...");

    while (true) {
        WiFiClient configClient = ESPserver.available();

        if (configClient) {
            Serial.println("Config client detected.");

            while (configClient.connected() || configClient.available() > 0) {
                if (configClient.available() > 0) {
                    memset(buffer, 0, sizeof(buffer));
                    int n = configClient.readBytesUntil('\n', buffer, sizeof(buffer) - 1);

                    if (n > 0) {
                        buffer[n] = '\0';
                        if (buffer[n - 1] == '\r') buffer[n - 1] = '\0';

                        if (strncmp(buffer, "WIFI:", 5) == 0) {
                            char *ssid = buffer + 5;
                            char *virgula = strchr(ssid, ',');
                            if (virgula != NULL) {
                                *virgula = '\0';
                                char *password = virgula + 1;
                                Write_to_flash(ssid, password);
                                configClient.println("OK: Network saved.");
                            }
                        } else if (strncmp(buffer, "RESET:NOW", 9) == 0) {
                            configClient.println("OK: Restarting...");
                            delay(500);
                            ESP.restart();
                        }
                    }
                }
            }
            Serial.println("Client disconnected. Waiting...");
        }
        delay(10);
    }
}


// Turns speed and steering (both -1..1, from the PC) into direction and
// per-side power. IN pins set direction for both motors; the EN duty of
// each side is cut by the steering amount
void motor_logic(float speed, float steering) {
    Serial.printf("MOTOR: speed=%.2f steering=%.2f\n", speed, steering);

    // Direction
    if(speed == 0){
        digitalWrite(IN1_pin, LOW);
        digitalWrite(IN2_pin, LOW);
    }
    else if(speed > 0){
        digitalWrite(IN1_pin, HIGH);
        digitalWrite(IN2_pin, LOW);
    }
    else if(speed < 0){
        digitalWrite(IN1_pin, LOW);
        digitalWrite(IN2_pin, HIGH);
    }

    // Velocity
    int en_esquerda = 0;
    int en_direita = 0;

    if(steering >= 0){
        en_esquerda = (int)(speed * 255);
        en_direita  = (int)(speed * 255 * (1 - steering));
    }
    else{
        en_direita  = (int)(speed * 255);
        en_esquerda = (int)(speed * 255 * (1 + steering));
    }

    ledcWrite(EN1_pin, abs(en_esquerda));
    ledcWrite(EN2_pin, abs(en_direita));
    Serial.printf("EN_LEFT=%d EN_RIGHT=%d\n", abs(en_esquerda), abs(en_direita));
}

void task_camara(void *parameter) {
    Serial.printf("Camera task running on Core %d\n", xPortGetCoreID());

    while (true) {
        if (!clienteVideo.connected()) {
            Serial.println("[VIDEO] Trying to connect to PC...");
            if (clienteVideo.connect(destino, VIDEO_PORT)) {
                Serial.println("[VIDEO] Connected!");
            } else {
                Serial.println("[VIDEO] Connection failed");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                continue;
            }
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (fb == NULL) {
            Serial.println("[VIDEO] Frame capture failed");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        uint32_t tamanho = fb->len;
        clienteVideo.write((uint8_t*)&tamanho, 4);
        clienteVideo.write(fb->buf, fb->len);

        esp_camera_fb_return(fb);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void processar_comando(char* data) {
    Serial.print("Command received to process: ");
    Serial.println(data);

    // Parses a "MOV:x,DIR:y" line and drives the motors with x and y.
    // data+4 skips "MOV:"; strchr finds the comma; writing '\0' over the
    // comma splits the string so mov holds only x. virgula+5 skips ",DIR:".
    // atof turns each number text into a float.

    char *mov = data + 4;
    char *virgula = strchr(mov, ',');

    float move = 0.0;
    float direc = 0.0;

    if (virgula != NULL) {
        *virgula = '\0';
        char *dir_str = virgula + 5;   // assumes fixed ",DIR:" prefix

        move  = atof(mov);
        direc = atof(dir_str);
        motor_logic(move, direc);
    }
}

void setup() {
    Serial.begin(115200);
    pinos_setup();          // motor direction pins
    pwm_channel_setup();    // motor PWM on LEDC channels 0/1
    setup_flash_memory();   // init NVS partition
    dump_nvs_to_serial();   // debug: print stored networks

    // ── Camera configuration: fill the config struct field by field ───
    // Pin map (source: ESP32_CAM_V1.6 schematic, docs/ESP32_CAM_V1_6.pdf)
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;

    config.xclk_freq_hz = 20000000;      // 20 MHz clock the ESP32 generates and
                                         // feeds to the sensor (it has no
                                         // oscillator of its own). 20 MHz is the
                                         // OV2640 datasheet recommended value.
    config.ledc_timer   = LEDC_TIMER_1;  // LEDC timer+channel used to produce that
    config.ledc_channel = LEDC_CHANNEL_2;// XCLK. Channel 2, so it does not clash
                                         // with channels 0/1 driving the motors.
    config.pixel_format = PIXFORMAT_JPEG; // sensor outputs JPEG, already compressed
    config.frame_size   = FRAMESIZE_VGA;  // 640x480
    config.jpeg_quality = 30;             // 0=best/largest, 63=worst; higher = smaller frame
    config.fb_count     = 2;              // two frame buffers: capture one while sending the other
    config.fb_location  = CAMERA_FB_IN_PSRAM; // frames go in PSRAM; too big for internal RAM
    config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY; // fetch a new frame only when a buffer is free

    esp_err_t err_cam = esp_camera_init(&config);
    if (err_cam != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err_cam);
    } else {
        Serial.println("Camera OK");
    }

    // ── WiFi: try saved networks, or fall back to AP mode ─────────────
    nvs_handle_t my_handle_read;
    uint32_t count = 0;
    esp_err_t err = nvs_open("wifi_storage_id", NVS_READONLY, &my_handle_read);

    // No stored networks: either the namespace was never created (no network
    // ever saved) or the wifi_count key is missing. Both mean zero networks,
    // so fall into AP mode and stay in the config loop until you send one.
    // enable_access_point(): ESP32 creates its own network (ESP32_CAM_AFONSO),
    //   takes IP 192.168.4.1, opens the TCP server on port 1883.
    // loop_config_mode(): infinite loop reading your commands, saves the
    //   network you send to flash. Never returns — only leaves on chip reset
    //   (the RESET:NOW you send from Packet Sender).
    if (err != ESP_OK || nvs_get_u32(my_handle_read, "wifi_count", &count) != ESP_OK) {
        nvs_close(my_handle_read);
        enable_access_point();
        loop_config_mode();
    } else {
        int numero_redes_ar = WiFi.scanNetworks();
        bool conectou = false;
        my_wifi_credentials_t net;                    // buffer for each network read from flash
        size_t size = sizeof(my_wifi_credentials_t);

        // Match saved networks against what's on the air.
        // Outer loop: networks currently in range (i). Inner loop: networks
        // saved in flash (j, count of them). For each network in range, look
        // for a saved one with the same SSID; on a match, try to connect.
        for (int i = 0; i < numero_redes_ar && !conectou; i++) {
            for (uint32_t j = 0; j < count; j++) {

                // Build the NVS key for saved network j: "wifi_0", "wifi_1", ...
                char key[16];
                sprintf(key, "wifi_%lu", j);

                // match found: this saved network is in range. try to connect.
                if (nvs_get_blob(my_handle_read, key, &net, &size) == ESP_OK
                    && strcmp(net.ssid, WiFi.SSID(i).c_str()) == 0) {

                    Serial.printf("Trying: %s\n", net.ssid);
                    WiFi.begin(net.ssid, net.password);

                    // wait up to 10 s for the connection
                    unsigned long t0 = millis();
                    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
                        delay(500);
                        Serial.print(".");
                    }

                    if (WiFi.status() == WL_CONNECTED) {
                        Serial.printf("\nConnected to: %s\n", net.ssid);
                        conectou = true;
                    } else {
                        Serial.printf("\nFailed: %s\n", net.ssid);
                        WiFi.disconnect();
                        delay(200);
                    }
                    break;   // SSID matched; stop scanning flash for this network
                }
            }
        }

        nvs_close(my_handle_read);

        // none of the saved networks was in range or accepted the connection:
        // fall back to AP mode, same as the no-networks case above.
        if (!conectou) {
            Serial.println("All networks failed. Fallback mode.");
            enable_access_point();
            loop_config_mode();
        }

        // WiFi is up: launch the camera streaming task pinned to Core 0,
        // so heavy JPEG sending never blocks the motor commands on Core 1.
        xTaskCreatePinnedToCore(
            task_camara,    // function the task runs
            "task_camara",  // name, debug only
            10000,          // stack size in bytes
            NULL,           // no argument
            1,              // priority
            NULL,           // task handle not kept
            0               // Core 0
        );
    }
}



void loop() {
    // Layer 1 — is WiFi connected?
    // If not: stop the motors and start timing the outage (trackingLostWifi
    // makes sure the instant is recorded only once). After 10 s down,
    // ESP.restart(). The return blocks everything else — no WiFi, nothing to
    // do. If connected, clear the flag so the next outage is timed fresh.
    if (WiFi.status() != WL_CONNECTED) {
        stop_motors();
        if (!trackingLostWifi) {
            wifiLostTimestamp = millis();
            trackingLostWifi = true;
        }
        if (millis() - wifiLostTimestamp > 10000) ESP.restart();
        return;
    }
    trackingLostWifi = false;

    // Layer 2 — is the TCP link to the PC up?
    // If down: stop the motors and retry connect once every 5 s (connect
    // blocks while waiting, so calling it every loop would trap us here).
    // Reset lastHeartbeat either way so the fresh link isn't seen as silent.
    if (!client.connected()) {
        stop_motors();
        if (millis() - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = millis();
            Serial.printf("[%lu] Trying to reach the PC...\n", millis());
            if (client.connect(destino, SERVER_PORT)) {
                Serial.printf("[%lu] Connected successfully!\n", millis());
                lastHeartbeat = millis();
            } else {
                lastHeartbeat = millis();
                Serial.printf("[%lu] CONNECTION FAILED!\n", millis());
            }
        }
        return;
    }

    // Layer 3 — silence watchdog. PC sends HB every 0.2 s, so silence = trouble.
    if (millis() - lastHeartbeat > 1000) {
        stop_motors();                    // 1 s: safety stop
    }

    if (millis() - lastHeartbeat > 5000) {
        Serial.printf("[%lu] HEARTBEAT TIMEOUT - closing!\n", millis());
        client.stop();                    // 5 s: assume dead, drop the socket
        lastReconnectAttempt = millis();
    }

    // Layer 4 — read all pending lines; only MOV: lines drive the motors.
    if (client.available() > 0) {
        while (client.available() > 0) {
            memset(buffer, 0, sizeof(buffer));
            client.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
            lastHeartbeat = millis();
        }

        if (strncmp(buffer, "MOV:", 4) == 0) {
            processar_comando(buffer);
        }
    }
}
