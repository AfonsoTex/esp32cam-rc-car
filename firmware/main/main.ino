#include "config.h"
#include "control_protocol.h"
#include "video_protocol.h"
#include <WiFiUdp.h>
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

// UDP control and video ports on the normal WiFi connection.
// The AP configuration TCP server remains separate below.
#define CONTROL_PORT 1883
#define VIDEO_PORT 1884
#define HELLO_INTERVAL_MS 1000UL
#define CONTROL_WATCHDOG_MS 500UL

unsigned long wifiLostTimestamp = 0;
bool trackingLostWifi = false;
unsigned long lastHelloSent = 0;
ControlSequenceState udpControlSequenceState;
unsigned long lastUdpControlLog = 0;
unsigned long lastValidUdpCommand = 0;
volatile bool udpCommandApplied = false;
float lastAppliedUdpMove = 0.0F;
float lastAppliedUdpDirection = 0.0F;
IPAddress controlPeerIp;
bool controlPeerIpValid = false;

// TCP server, used ONLY in Access Point mode (initial WiFi setup).
// With no known network, the ESP32 becomes the AP "ESP32_CAM_AFONSO".
// You connect to it and send "WIFI:ssid,password\n" to this port to
// store a new network in NVS flash.
WiFiServer ESPserver(1883);

// Normal mode uses separate UDP sockets for control and video.
WiFiUDP udpControl;
WiFiUDP udpVideo;

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

/*void task_camara(void *parameter) {
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
*/

bool send_video_frame_fragmented(
    const camera_fb_t *frame,
    uint32_t frame_id
) {
    if (
        frame == nullptr
        || frame->len == 0
        || frame->len > VIDEO_FRAME_MAX_SIZE
    ) {
        return false;
    }

    const uint32_t frame_size =
        static_cast<uint32_t>(frame->len);
    const uint16_t fragment_count =
        video_fragment_count(frame_size);

    if (fragment_count == 0) {
        return false;
    }

    uint8_t header[VIDEO_HEADER_SIZE];

    for (
        uint16_t fragment_index = 0;
        fragment_index < fragment_count;
        ++fragment_index
    ) {
        const size_t offset =
            static_cast<size_t>(fragment_index)
            * VIDEO_FRAGMENT_PAYLOAD_SIZE;
        const size_t remaining = frame->len - offset;
        const size_t payload_size =
            remaining < VIDEO_FRAGMENT_PAYLOAD_SIZE
            ? remaining
            : VIDEO_FRAGMENT_PAYLOAD_SIZE;

        if (
            !encode_video_header(
                header,
                sizeof(header),
                frame_id,
                fragment_index,
                fragment_count,
                frame_size
            )
        ) {
            return false;
        }

        if (udpVideo.beginPacket(destino, VIDEO_PORT) == 0) {
            return false;
        }

        const bool write_succeeded =
            udpVideo.write(header, sizeof(header))
                == sizeof(header)
            && udpVideo.write(
                frame->buf + offset,
                payload_size
            ) == payload_size;

        const bool send_succeeded =
            udpVideo.endPacket() == 1;

        if (!write_succeeded || !send_succeeded) {
            return false;
        }

        taskYIELD();
    }

    return true;
}


void task_camara(void *parameter) {
    udpVideo.begin(VIDEO_PORT);

    uint32_t next_frame_id = 0;
    unsigned long last_error_log = 0;

    while (true) {
        if (!udpCommandApplied) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        camera_fb_t *frame = esp_camera_fb_get();

        if (frame == nullptr) {
            Serial.println("[VIDEO] Frame capture failed");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        const uint32_t frame_id = next_frame_id++;
        const bool sent = send_video_frame_fragmented(
            frame,
            frame_id
        );

        esp_camera_fb_return(frame);

        if (!sent && millis() - last_error_log >= 1000UL) {
            Serial.println(
                "[VIDEO] Fragmented frame send failed"
            );
            last_error_log = millis();
        }

        vTaskDelay(33 / portTICK_PERIOD_MS);
    }
}


void send_control_hello_if_due() {
    unsigned long now = millis();

    // Unsigned subtraction remains valid when millis() wraps around.
    if (lastHelloSent != 0 && now - lastHelloSent < HELLO_INTERVAL_MS) {
        return;
    }
    lastHelloSent = now;

    if (udpControl.beginPacket(destino, CONTROL_PORT) == 0) {
        return;
    }

    udpControl.print("HELLO:");
    udpControl.print(CONTROL_PROTOCOL_VERSION);
    udpControl.endPacket();
}

void process_udp_control_packets() {
    int packet_size = 0;

    while ((packet_size = udpControl.parsePacket()) > 0) {
        if (!controlPeerIpValid || udpControl.remoteIP() != controlPeerIp) {
            while (udpControl.available() > 0) {
                udpControl.read();
            }
            continue;
        }

        if (packet_size > static_cast<int>(CONTROL_PACKET_MAX_SIZE)) {
            while (udpControl.available() > 0) {
                udpControl.read();
            }
            continue;
        }

        uint8_t packet[CONTROL_PACKET_MAX_SIZE];
        const int bytes_read = udpControl.read(
            packet,
            static_cast<size_t>(packet_size)
        );

        if (bytes_read != packet_size) {
            while (udpControl.available() > 0) {
                udpControl.read();
            }
            continue;
        }

        ControlCommand command{};
        const ControlParseResult parse_result = parse_control_command(
            packet,
            static_cast<size_t>(bytes_read),
            command
        );

        if (parse_result != ControlParseResult::OK) {
            continue;
        }

        if (!accept_control_sequence(command, udpControlSequenceState)) {
            continue;
        }

        const unsigned long now = millis();
        lastValidUdpCommand = now;

        if (
            !udpCommandApplied
            || command.move != lastAppliedUdpMove
            || command.direction != lastAppliedUdpDirection
        ) {
            motor_logic(command.move, command.direction);
            lastAppliedUdpMove = command.move;
            lastAppliedUdpDirection = command.direction;
            udpCommandApplied = true;
        }

        if (now - lastUdpControlLog >= 1000UL) {
            Serial.printf(
                "[CONTROL][UDP] session=%lu seq=%lu move=%.2f dir=%.2f\n",
                static_cast<unsigned long>(command.session),
                static_cast<unsigned long>(command.sequence),
                command.move,
                command.direction
            );
            lastUdpControlLog = now;
        }
    }
}


void enforce_udp_control_watchdog() {
    if (!udpCommandApplied) {
        return;
    }

    if (millis() - lastValidUdpCommand <= CONTROL_WATCHDOG_MS) {
        return;
    }

    stop_motors();
    udpCommandApplied = false;
    Serial.println("[CONTROL][UDP] WATCHDOG TIMEOUT - motors stopped");
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
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 30;           // 0=best/largest, 63=worst; higher = smaller frame
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

        controlPeerIpValid = controlPeerIp.fromString(destino);
        if (!controlPeerIpValid) {
            Serial.println("[CONTROL] DESTINO_IP is not a valid IPv4 address");
        }

        // Bind the UDP control channel after normal WiFi connection.
        // AP configuration mode returns earlier and keeps its TCP server unchanged.
        if (udpControl.begin(CONTROL_PORT)) {
            Serial.printf("[CONTROL] UDP listening on port %d\n", CONTROL_PORT);
        } else {
            Serial.println("[CONTROL] Failed to open UDP control port");
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
        udpCommandApplied = false;
        stop_motors();
        if (!trackingLostWifi) {
            wifiLostTimestamp = millis();
            trackingLostWifi = true;
        }
        if (millis() - wifiLostTimestamp > 10000) ESP.restart();
        return;
    }
    trackingLostWifi = false;

    // Advertise this ESP32 and process all pending UDP control packets.
    send_control_hello_if_due();
    process_udp_control_packets();
    enforce_udp_control_watchdog();
    delay(1);

}
