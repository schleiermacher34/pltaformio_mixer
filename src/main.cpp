#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include "esp_log.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ArduinoJson.h>
#include <nvs.h>
#include <nvs_flash.h>
#include "esp_ota_ops.h"  // Include ESP-IDF OTA operations
#include <ModbusRTU.h>    // Include emelianov's ModbusRTU library
#include <esp_system.h>
#include <esp_partition.h>
#include <mbedtls/sha256.h>
// Place the restartDevice function
void restartDevice() {
    Serial.println("Device will restart in 5 seconds...");
    Serial.flush();
    delay(5000); // Wait 5 seconds
    ESP.restart();
}

// Place the vApplicationStackOverflowHook function
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    Serial.printf("Stack Overflow in task: %s\n", pcTaskName);
    Serial.flush();
    delay(5000); // Wait 5 seconds
    abort(); // Stop execution
}

/* Heap memory check */
void check_heap() {
    Serial.print("Free heap: ");
    Serial.println(esp_get_free_heap_size());
}

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5
#define VSYNC_PIN 3 // Pin IO3 is used for VSYNC

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (5) // Adjusted delay for stability
#define LVGL_TASK_STACK_SIZE    (10 * 1024) // Increased stack size for LVGL
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * ESP_PANEL_LCD_V_RES / 10)

/* Define constants for the program data */
#define MAX_STEPS 3  // Maximum steps in a program
// OTA update server details
// Device current firmware version
#define CURRENT_FIRMWARE_VERSION "1.0.0"
const char* host = "https://schleiermacher34.pythonanywhere.com/firmware/check_update/"; // Corrected: No port needed for PythonAnywhere




/* Task and Semaphore Handles */
TaskHandle_t otaTaskHandle = NULL;
SemaphoreHandle_t lvgl_mux = NULL; // LVGL mutex
SemaphoreHandle_t wifi_connect_semaphore = NULL;
SemaphoreHandle_t wifi_scan_semaphore = NULL;
SemaphoreHandle_t otaSemaphore = NULL;
SemaphoreHandle_t save_program_semaphore = NULL; // Semaphore for saving program
SemaphoreHandle_t run_program_semaphore = NULL;
SemaphoreHandle_t nvs_mutex;
SemaphoreHandle_t vsync_semaphore;


// Forward declaration of the OTA update task
void otaUpdateTask(void* parameter);

/* Structure to hold the program data */
struct ProgramData {
    uint16_t startRpm;           // Starting RPM from ui_Screen3_Roller9
    uint16_t repeatCount;        // Repeat count from ui_Screen3_Roller18
    uint16_t times[MAX_STEPS];   // Time for each step
    uint16_t speeds[MAX_STEPS];  // Speeds for each step
    bool directions[MAX_STEPS];  // Directions for each step
};

typedef struct {
    bool direction;       // true for forward, false for reverse
    uint16_t speed;       // Speed in RPM
    bool autoMode;        // true for auto, false for manual
    // Add other settings as needed
} MotorConfig;

ESP_Panel *panel = NULL;

/* Modbus Configuration */
#define MODBUS_TX_PIN 44        // Example GPIO pin for UART1 TX
#define MODBUS_RX_PIN 43      // Example GPIO pin for UART1 RX
#define MODBUS_DE_RE_PIN 4      // GPIO4 (DE/RE control pin)

// Error related registers
#define DRIVE_STATUS_REGISTER 0x2100
#define ERROR_CODE_REGISTER 0x2101
#define ALARM_STATUS_REGISTER 0x2226

HardwareSerial ModbusSerial(1); // Use UART1
ModbusRTU mb;                   // ModbusRTU instance

#define MAX_SPEED_RPM 60        // Maximum speed in RPM
#define MIN_SPEED_RPM 10        // Minimum speed in RPM
#define MOTOR_POLES 2           // Number of motor poles (adjust as per your motor)
#define BASE_FREQUENCY 50.0     // Base frequency in Hertz (adjust as per your motor)

// Global variables for UI updates
volatile bool modbusErrorFlag = false;
volatile uint8_t modbusErrorCode = 0;
volatile bool mototimeUpdated = false;
volatile uint16_t mototimeValue = 0;

/* Function to Convert RPM to Hertz */
float rpmToHertz(int rpm) {
    // Formula: Frequency (Hz) = (RPM × Number of Poles) / 120
    return (rpm * MOTOR_POLES) / 12.0;
}




#if ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}
#else
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

bool notify_lvgl_flush_ready(void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
#endif /* ESP_PANEL_LCD_BUS_TYPE */

#if ESP_PANEL_USE_LCD_TOUCH
/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();

        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = point.x;
        data->point.y = point.y;

        Serial.printf("Touch point: x %d, y %d\n", point.x, point.y);
    }
}
#endif

bool lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

bool ota_in_progress = false;

void lvgl_port_task(void *arg) {
    Serial.println("Starting LVGL task");

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        lvgl_port_lock(-1); // Lock LVGL due to non-thread-safe API

        // Handle LVGL timers
        task_delay_ms = lv_timer_handler();

        // Check and handle Modbus error updates
        if (modbusErrorFlag) {
            lv_label_set_text_fmt(ui_Screen2_Label7, "Modbus error: 0x%02X", modbusErrorCode);
            modbusErrorFlag = false; // Reset the flag after handling
        }

        // Check and handle mototime updates
        if (mototimeUpdated) {
            lv_label_set_text_fmt(ui_Screen1_Label11, " %u", mototimeValue);
            mototimeUpdated = false; // Reset the flag after handling
        }

        lvgl_port_unlock();

        // Adjust task delay
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }

 if (ota_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(300)); // Increase delay during OTA to reduce screen updates
        } else {
            vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_MIN_DELAY_MS));
        }

        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}
void IRAM_ATTR vsync_isr() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(vsync_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void setupVsync() {
    vsync_semaphore = xSemaphoreCreateBinary();
    pinMode(VSYNC_PIN, INPUT_PULLUP); // Set IO3 as input with pull-up resistor
    attachInterrupt(VSYNC_PIN, vsync_isr, RISING); // Trigger on rising edge of VSYNC
}


/* UI Elements */
// Settings Screen (Screen2)
extern lv_obj_t *ui_Screen2_Button8;    // Scan button
extern lv_obj_t *ui_Screen2_Button9;    // Connect button
extern lv_obj_t *ui_Screen2_Button10;   // Update button
extern lv_obj_t *ui_Screen2_Dropdown2;  // Wi-Fi list
extern lv_obj_t *ui_Screen2_TextArea2;  // Wi-Fi password TextArea
extern lv_obj_t *ui_Screen2_Label7;     // Log messages label

// Motor Control Screen (Screen3)
extern lv_obj_t *ui_Screen3_Button13;   // Start motor button
extern lv_obj_t *ui_Screen3_Button2;    // Stop motor button
extern lv_obj_t *ui_Screen3_Button3;    // Change speed button
extern lv_obj_t *ui_Screen3_Switch1;    // Direction switch (forward/reverse)
extern lv_obj_t *ui_Screen3_Roller1;    // Speed roller (RPM)
extern lv_obj_t *ui_Screen3_Dropdown1;  // Motor selection dropdown (Right/Left)

// Programs Screen (Screen5)
extern lv_obj_t *ui_Screen4_Dropdown1;  // Program selection dropdown
extern lv_obj_t *ui_Screen3_Roller9;    // Start speed roller (RPM)
extern lv_obj_t *ui_Screen3_Roller12;   // Time roller 1 (minutes)
extern lv_obj_t *ui_Screen3_Roller13;   // Time roller 2 (minutes)
extern lv_obj_t *ui_Screen3_Roller14;   // Time roller 3 (minutes)
extern lv_obj_t *ui_Screen3_Roller15;   // Speed roller 1 (RPM)
extern lv_obj_t *ui_Screen3_Roller16;   // Speed roller 2 (RPM)
extern lv_obj_t *ui_Screen3_Roller17;   // Speed roller 3 (RPM)
extern lv_obj_t *ui_Screen3_Switch2;    // Direction switch 1 (forward/reverse)
extern lv_obj_t *ui_Screen3_Switch8;    // Direction switch 2 (forward/reverse)
extern lv_obj_t *ui_Screen3_Switch9;    // Direction switch 3 (forward/reverse)
extern lv_obj_t *ui_Screen5_Button3;    // Save program button
extern lv_obj_t *ui_Screen1_Label11;
extern lv_obj_t *ui_Screen1_Label2;
extern lv_obj_t *ui_Screen3_Button5;
extern lv_obj_t *ui_Screen2_Label1;
extern lv_obj_t *ui_screen2_Image3;
extern lv_obj_t *ui_screen2_Panel1;
extern lv_obj_t *ui_screen2_Panel4;




bool verifyChecksum(WiFiClient* stream, int contentLength, const char* expectedChecksum) {
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts_ret(&ctx, 0); // 0 for SHA-256, 1 for SHA-224

    int remaining = contentLength;
    uint8_t buff[1024];
    while (remaining > 0) {
        int len = stream->readBytes(buff, sizeof(buff));
        if (len > 0) {
            mbedtls_sha256_update_ret(&ctx, buff, len);
            remaining -= len;
        } else {
            Serial.println("Stream read error during checksum calculation");
            mbedtls_sha256_free(&ctx);
            return false;
        }
    }

    uint8_t hash[32];
    mbedtls_sha256_finish_ret(&ctx, hash);
    mbedtls_sha256_free(&ctx);

    // Convert hash to hex string
    char hashString[65];
    for (int i = 0; i < 32; i++) {
        sprintf(&hashString[i * 2], "%02x", hash[i]);
    }
    hashString[64] = '\0';

    Serial.print("Calculated checksum: ");
    Serial.println(hashString);
    Serial.print("Expected checksum: ");
    Serial.println(expectedChecksum);

    return strcmp(hashString, expectedChecksum) == 0;
}



uint16_t orig_anim_speed = 100; // Default animation speed
uint16_t orig_refresh_rate = 60; // Default refresh rate

void reduce_display_refresh_rate() {
    lvgl_port_lock(-1);


     lv_anim_del_all();
    // lv_disp_t *disp = lv_disp_get_default();
    // if (disp) {
    //     // Store original animation speed and refresh rate
    //     orig_anim_speed = lv_disp_get_anim_speed(disp);
    //     orig_refresh_rate = lv_disp_get_refresh_rate(disp);

    //     // Reduce animation speed and refresh rate
    //     lv_disp_set_anim_speed(disp, 0);    // Stop animations
    //     lv_disp_set_refresh_rate(disp, 10); // Reduce refresh rate to 10 Hz
    // }

    lvgl_port_unlock();
}


void restore_display_refresh_rate() {
    lvgl_port_lock(-1);

    // lv_disp_t *disp = lv_disp_get_default();
    // if (disp) {
    //     // Restore original animation speed and refresh rate
    //     lv_disp_set_anim_speed(disp, orig_anim_speed);
    //     lv_disp_set_refresh_rate(disp, orig_refresh_rate);
    // }

    lvgl_port_unlock();
}


void hide_ui_elements_for_ota() {
    // Lock LVGL
    lvgl_port_lock(-1);

    // Hide unnecessary UI elements during OTA
    lv_obj_add_flag(ui_Screen2_Dropdown2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Screen2_Button8, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Screen2_Button9, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Screen2_Button10, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Screen2_Panel3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Screen2_Button12, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x151515), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_flag(ui_Screen2_Panel4, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Label1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Image3, LV_OBJ_FLAG_HIDDEN);
    // Unlock LVGL
    lvgl_port_unlock();
}

void show_ui_elements_after_ota() {
    // Lock LVGL
    lvgl_port_lock(-1);

    // Show UI elements again
    lv_obj_clear_flag(ui_Screen2_Dropdown2, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Button8, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Button9, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Button10, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Panel3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Button12, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_bg_color(ui_Screen2, lv_color_hex(0x1F0675), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_add_flag(ui_Screen2_Label1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ui_Screen2_Image3, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ui_Screen2_Panel4, LV_OBJ_FLAG_HIDDEN);

    // Unlock LVGL
    lvgl_port_unlock();
}


lv_obj_t* ota_progress_bar;

void create_ota_progress_bar() {
    lvgl_port_lock(-1); // Lock LVGL

    // Create a progress bar on the display
    ota_progress_bar = lv_bar_create(lv_scr_act());  // Create on the active screen
    lv_obj_set_size(ota_progress_bar, 200, 30);       // Set size
    lv_obj_align(ota_progress_bar, LV_ALIGN_CENTER, 0, 0); // Align to center
    lv_bar_set_range(ota_progress_bar, 0, 10);       // Set range 0 to 100
    lv_bar_set_value(ota_progress_bar, 0, LV_ANIM_OFF); // Initialize to 0%
     Serial.printf("ota_progress_bar created at address: %p\n", (void*)ota_progress_bar);

    lvgl_port_unlock(); // Unlock LVGL
}



volatile bool ota_progress_bar_deleting = false;

void update_ota_progress_bar(int progress) {
    if (ota_progress_bar_deleting) {
        // Skip updating if the progress bar is being deleted
        return;
    }

    if (xSemaphoreTake(vsync_semaphore, portMAX_DELAY) == pdTRUE) {
        // Wait until VSYNC occurs to update the progress bar

        lvgl_port_lock(-1); // Lock LVGL

        if (ota_progress_bar != NULL) {
            lv_bar_set_value(ota_progress_bar, progress, LV_ANIM_OFF);  // Update progress bar
        }

        lvgl_port_unlock(); // Unlock LVGL
    }
}

void remove_ota_progress_bar() {
    ota_progress_bar_deleting = true;

    lvgl_port_lock(-1); // Lock LVGL

    if (ota_progress_bar != NULL) {
        lv_anim_del(ota_progress_bar, NULL);
        lv_obj_remove_event_cb(ota_progress_bar, NULL);
        lv_obj_del_async(ota_progress_bar);
        ota_progress_bar = NULL;
        Serial.println("ota_progress_bar deleted successfully.");
    } else {
        Serial.println("ota_progress_bar is already NULL.");
    }

    lvgl_port_unlock(); // Unlock LVGL

    ota_progress_bar_deleting = false;
}




void performOTAUpdate(const String& firmwareUrl) {
    Serial.println("Starting OTA update...");

    WiFiClientSecure client; // Use WiFiClient for HTTP, WiFiClientSecure for HTTPS
   client.setInsecure(); // Uncomment if using HTTPS without certificate validation

    HTTPClient http;
    if (http.begin(client, firmwareUrl)) {
        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
            int contentLength = http.getSize();
            WiFiClient* stream = http.getStreamPtr();

            if (contentLength > 0) {
                const esp_partition_t* updatePartition = esp_ota_get_next_update_partition(NULL);
                if (updatePartition == NULL) {
                    Serial.println("Could not find valid OTA partition.");
                    http.end();
                    return;
                }

                esp_ota_handle_t otaHandle = 0;
                esp_err_t err = esp_ota_begin(updatePartition, OTA_SIZE_UNKNOWN, &otaHandle);
                if (err != ESP_OK) {
                    Serial.printf("esp_ota_begin failed: %s\n", esp_err_to_name(err));
                    http.end();
                    return;
                }

                Serial.println("OTA update in progress...");

                int written = 0;
                uint8_t buff[1024] = { 0 };

                // **Set the stream timeout before the OTA update loop**
                stream->setTimeout(5000); // Set timeout to 5000 milliseconds (5 seconds)

                int retryCount = 0;
                const int maxRetries = 5;

                while (http.connected() && written < contentLength) {
                    int len = stream->readBytes(buff, sizeof(buff));
                    if (len > 0) {
                        // Reset retry count after a successful read
                        retryCount = 0;

                        err = esp_ota_write(otaHandle, buff, len);
                        if (err != ESP_OK) {
                            Serial.printf("esp_ota_write failed: %s\n", esp_err_to_name(err));
                            esp_ota_abort(otaHandle);
                            http.end();
                            return;
                        }
                        written += len;

                        // Update progress bar
                        int progress = (written * 100) / contentLength;
                        update_ota_progress_bar(progress);

                        Serial.printf("Written %d of %d bytes (%d%% complete)\n", written, contentLength, progress);
                    } else {
                        if (retryCount < maxRetries) {
                            retryCount++;
                            Serial.printf("Stream read timeout, retrying... (%d/%d)\n", retryCount, maxRetries);
                            continue; // Retry reading
                        } else {
                            Serial.println("Maximum retries reached. Aborting OTA update.");
                            esp_ota_abort(otaHandle);
                            http.end();
                            return;
                        }
                    }
                }

                // Verify that the entire content was received
                if (written != contentLength) {
                    Serial.println("Download incomplete. Aborting OTA update.");
                    esp_ota_abort(otaHandle);
                    http.end();
                    return;
                }

                if (esp_ota_end(otaHandle) != ESP_OK) {
                    Serial.println("Error with OTA end");
                    http.end();
                    return;
                }

                err = esp_ota_set_boot_partition(updatePartition);
                if (err != ESP_OK) {
                    Serial.printf("esp_ota_set_boot_partition failed: %s\n", esp_err_to_name(err));
                    http.end();
                    return;
                }

                Serial.println("OTA update successful. Rebooting...");
                http.end();

                // Optionally display 100% on progress bar
                update_ota_progress_bar(100);

                // Delay to allow the UI to update before rebooting
                vTaskDelay(pdMS_TO_TICKS(1000));

                esp_restart();
            } else {
                Serial.println("Content-Length is zero or not provided");
            }
        } else {
            Serial.printf("HTTP request failed: %d\n", httpCode);
        }

        http.end();
    } else {
        Serial.println("Unable to connect to firmware URL");
    }
}


void checkForUpdates() {
    Serial.println("Checking for firmware updates...");

    String updateCheckUrl = host; // Use the host as the update check URL

    WiFiClientSecure client;
    client.setInsecure(); // Disable certificate validation (for testing only)

    HTTPClient http;
    if (http.begin(client, updateCheckUrl)) {
        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            Serial.println("Received payload:");
            Serial.println(payload);

            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload);
            if (error) {
                Serial.print("deserializeJson() failed: ");
                Serial.println(error.c_str());
                http.end();
                return;
            }

            bool updateAvailable = doc["update_available"];
            String newVersion = doc["version"];
            String firmwareDownloadUrl = doc["url"];

            if (updateAvailable) {
                Serial.printf("Update available! New version: %s\n", newVersion.c_str());
                Serial.printf("Firmware URL: %s\n", firmwareDownloadUrl.c_str());

                // Close the http connection from the update check
                http.end();

                // Now, call performOTAUpdate with firmwareDownloadUrl
                performOTAUpdate(firmwareDownloadUrl);

            } else {
                Serial.println("No updates available.");
            }
        } else {
            Serial.printf("Error checking for updates, HTTP response code: %d\n", httpCode);
        }

        http.end();
    } else {
        Serial.println("Unable to connect to update server");
    }
}

void otaUpdateTask(void* parameter) {
    while (true) {
        // Wait indefinitely for the semaphore to be given
        if (xSemaphoreTake(otaSemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("Starting OTA update task...");

            // Check if WiFi is connected
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("WiFi not connected. Aborting OTA update.");
                continue; // Go back to waiting for the semaphore
            }
            ota_in_progress = true; // OTA is starting
            // Hide UI elements and display progress bar
            hide_ui_elements_for_ota();
            create_ota_progress_bar();

            // Adjust display settings to prevent blinking
            reduce_display_refresh_rate();

            // Proceed with the OTA update
                        // Call checkForUpdates() to handle the OTA update process
            checkForUpdates();

            // Restore display settings after OTA update
            restore_display_refresh_rate();

            // Remove the progress bar and restore UI elements
            
            show_ui_elements_after_ota();
            remove_ota_progress_bar();

            ota_in_progress = false;
            Serial.println("OTA update task completed.");
        }
    }
}





void event_handler_ota_update(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("OTA Update Button Pressed.");

        if (otaSemaphore != NULL) {
            xSemaphoreGive(otaSemaphore);
            Serial.println("OTA update triggered.");
        } else {
            Serial.println("OTA semaphore is NULL. Cannot trigger OTA update.");
        }
    }
}


/* Event Handler for Save Program Button */
void event_handler_save_program_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.println("Save Program button pressed, giving semaphore...");
        
        // Give the semaphore to trigger the save program task
        xSemaphoreGive(save_program_semaphore);
    }
}

// Modbus Callback Function
bool modbusCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    if (event != Modbus::EX_SUCCESS) {
        Serial.printf("Modbus error: 0x%02X\n", event);
        modbusErrorCode = event;
        modbusErrorFlag = true; // Set the flag
    } else {
        Serial.println("Modbus transaction successful");
        modbusErrorFlag = false; // Clear the flag
    }
    return true;
}


uint8_t getSelectedMotorID() {
  uint16_t selectedIndex;
    lvgl_port_lock(-1);
    selectedIndex = lv_dropdown_get_selected(ui_Screen3_Dropdown1);
    lvgl_port_unlock();

    Serial.printf("Selected motor index: %d\n", selectedIndex);

    if (selectedIndex == 0) {
        return 1; // Slave ID for right motor
    } else if (selectedIndex == 1) {
        return 2; // Slave ID for left motor
    } else {
        Serial.println("Unknown motor selected, defaulting to Right motor");
        return 1; // Default to right motor
    }
}

esp_err_t saveProgramToNVS() {
    ProgramData programData;
    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open the NVS partition for read/write operations
    err = nvs_open("program_storage", NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        return err;
    }

    // Get the selected program index from the dropdown (0, 1, 2)
    uint16_t programIndex;
    lvgl_port_lock(-1);
    programIndex = lv_dropdown_get_selected(ui_Screen4_Dropdown1);
    lvgl_port_unlock();

    Serial.printf("Saving program %d to NVS...\n", programIndex);

    // Lock the interface to safely interact with UI components
    lvgl_port_lock(-1);

    // Read the user input from the UI components (Rollers and Switches)
    programData.startRpm = lv_roller_get_selected(ui_Screen3_Roller9);
    programData.times[0] = lv_roller_get_selected(ui_Screen3_Roller12);
    programData.times[1] = lv_roller_get_selected(ui_Screen3_Roller13);
    programData.times[2] = lv_roller_get_selected(ui_Screen3_Roller14);
    programData.speeds[0] = lv_roller_get_selected(ui_Screen3_Roller15);
    programData.speeds[1] = lv_roller_get_selected(ui_Screen3_Roller16);
    programData.speeds[2] = lv_roller_get_selected(ui_Screen3_Roller17);
    programData.directions[0] = lv_obj_has_state(ui_Screen3_Switch2, LV_STATE_CHECKED);
    programData.directions[1] = lv_obj_has_state(ui_Screen3_Switch8, LV_STATE_CHECKED);
    programData.directions[2] = lv_obj_has_state(ui_Screen3_Switch9, LV_STATE_CHECKED);
    programData.repeatCount = lv_roller_get_selected(ui_Screen3_Roller18);

    // Unlock the interface after UI values are read
    lvgl_port_unlock();

    // Save program data to NVS for the selected program (0, 1, or 2)
    char key[20];

    // Save start RPM
    snprintf(key, sizeof(key), "prog%d_startRpm", programIndex);
    err = nvs_set_u16(nvsHandle, key, programData.startRpm);
    if (err != ESP_OK) {
        Serial.printf("Failed to save start RPM: %s\n", esp_err_to_name(err));
        nvs_close(nvsHandle);
        return err;
    }

    // Save repeat count
    snprintf(key, sizeof(key), "prog%d_repeatCnt", programIndex);
    err = nvs_set_u16(nvsHandle, key, programData.repeatCount);
    if (err != ESP_OK) {
        Serial.printf("Failed to save repeat count: %s\n", esp_err_to_name(err));
        nvs_close(nvsHandle);
        return err;
    }

    // Save times, speeds, and directions for each step
    for (int i = 0; i < MAX_STEPS; i++) {
        // Save time
        snprintf(key, sizeof(key), "prog%d_time%d", programIndex, i);
        err = nvs_set_u16(nvsHandle, key, programData.times[i]);
        if (err != ESP_OK) {
            Serial.printf("Failed to save time for step %d: %s\n", i, esp_err_to_name(err));
            nvs_close(nvsHandle);
            return err;
        }

        // Save speed
        snprintf(key, sizeof(key), "prog%d_speed%d", programIndex, i);
        err = nvs_set_u16(nvsHandle, key, programData.speeds[i]);
        if (err != ESP_OK) {
            Serial.printf("Failed to save speed for step %d: %s\n", i, esp_err_to_name(err));
            nvs_close(nvsHandle);
            return err;
        }

        // Save direction
        snprintf(key, sizeof(key), "prog%d_dir%d", programIndex, i);
        err = nvs_set_u8(nvsHandle, key, programData.directions[i] ? 1 : 0);
        if (err != ESP_OK) {
            Serial.printf("Failed to save direction for step %d: %s\n", i, esp_err_to_name(err));
            nvs_close(nvsHandle);
            return err;
        }
    }

    // Commit the changes to NVS
    err = nvs_commit(nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Failed to commit changes to NVS: %s\n", esp_err_to_name(err));
        nvs_close(nvsHandle);
        return err;
    }

    // Close the NVS handle
    nvs_close(nvsHandle);

    // Print a message to the terminal indicating the program was saved
    Serial.printf("Program %d successfully saved to NVS.\n", programIndex);

    return ESP_OK;
}


/* Task to save the program data to NVS */
void saveProgramTask(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(save_program_semaphore, portMAX_DELAY)) {
            Serial.println("Semaphore received, saving program data...");

            // Call the function to save the program data
            esp_err_t result = saveProgramToNVS();
            if (result == ESP_OK) {
                Serial.println("Program data successfully saved.");
            } else {
                Serial.printf("Failed to save program data: %s\n", esp_err_to_name(result));
            }
        }
    }
}

esp_err_t loadProgramFromNVS(uint16_t programIndex, ProgramData *programData) {
    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("program_storage", NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS: %s\n", esp_err_to_name(err));
        return err;
    }

    // Load start RPM
    char key[20];
    snprintf(key, sizeof(key), "prog%d_startRpm", programIndex);
    err = nvs_get_u16(nvsHandle, key, &programData->startRpm);
    if (err != ESP_OK) {
        Serial.printf("Failed to load start RPM: %s\n", esp_err_to_name(err));
        nvs_close(nvsHandle);
        return err;
    }

    // Load repeat count
    snprintf(key, sizeof(key), "prog%d_repeatCnt", programIndex);
    err = nvs_get_u16(nvsHandle, key, &programData->repeatCount);
    if (err != ESP_OK) {
        Serial.printf("Failed to load repeat count: %s\n", esp_err_to_name(err));
        nvs_close(nvsHandle);
        return err;
    }

    // Load times, speeds, and directions
    for (int i = 0; i < MAX_STEPS; i++) {
        snprintf(key, sizeof(key), "prog%d_time%d", programIndex, i);
        err = nvs_get_u16(nvsHandle, key, &programData->times[i]);
        if (err != ESP_OK) {
            Serial.printf("Failed to load time for step %d: %s\n", i, esp_err_to_name(err));
            nvs_close(nvsHandle);
            return err;
        }

        snprintf(key, sizeof(key), "prog%d_speed%d", programIndex, i);
        err = nvs_get_u16(nvsHandle, key, &programData->speeds[i]);
        if (err != ESP_OK) {
            Serial.printf("Failed to load speed for step %d: %s\n", i, esp_err_to_name(err));
            nvs_close(nvsHandle);
            return err;
        }

        snprintf(key, sizeof(key), "prog%d_dir%d", programIndex, i);
        uint8_t dir;
        err = nvs_get_u8(nvsHandle, key, &dir);
        programData->directions[i] = (dir != 0);
        if (err != ESP_OK) {
            Serial.printf("Failed to load direction for step %d: %s\n", i, esp_err_to_name(err));
            nvs_close(nvsHandle);
            return err;
        }
    }

    nvs_close(nvsHandle);
    return ESP_OK;
}


void runProgram(const ProgramData *programData) {
    for (uint16_t repeat = 0; repeat < programData->repeatCount; repeat++) {
        Serial.printf("Running program, repeat %d of %d\n", repeat + 1, programData->repeatCount);
        
        for (int i = 0; i < MAX_STEPS; i++) {
            uint16_t motorID = getSelectedMotorID();  // Get the motor ID (assuming function exists)
            Serial.printf("Step %d: Speed = %d RPM, Time = %d minutes, Direction = %s\n",
                          i + 1, programData->speeds[i], programData->times[i],
                          programData->directions[i] ? "Forward" : "Reverse");

            // Set motor direction via Modbus
            uint16_t motorDirection = programData->directions[i] ? 18 : 34;
            mb.writeHreg(motorID, 0x2000, motorDirection, modbusCallback);  // Assuming modbusCallback exists

            // Set motor speed via Modbus (convert RPM to frequency if needed)
            uint16_t frequencyValue = (programData->speeds[i] * 100) / 60;  // Assuming VFD frequency in 0.01 Hz
            mb.writeHreg(motorID, 0x2001, frequencyValue, modbusCallback);  // Set motor speed

            // Wait for the time period to complete (convert minutes to milliseconds)
            vTaskDelay(pdMS_TO_TICKS(programData->times[i] * 60 * 1000));  // Time in minutes
        }
    }

    // Stop the motor after program execution
    uint16_t motorID = getSelectedMotorID();
    mb.writeHreg(motorID, 0x2000, 1, modbusCallback);  // Stop motor
    Serial.println("Program completed, motor stopped.");
}


void runProgramTask(void *pvParameters) {
    ProgramData *programData = (ProgramData*)pvParameters;
    runProgram(programData);
    free(programData); // Free the allocated memory after use
    vTaskDelete(NULL); // Delete the task when done
}

esp_err_t saveMotorConfigToNVS(uint8_t motorID, const MotorConfig *config) {
    nvs_handle_t nvsHandle;
    esp_err_t err;

    xSemaphoreTake(nvs_mutex, portMAX_DELAY);

    // Open NVS handle
    err = nvs_open("motor_config", NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        xSemaphoreGive(nvs_mutex);
        return err;
    }

    // Create a unique key for each motor
    char key[16];

    // Save direction
    snprintf(key, sizeof(key), "motor%d_dir", motorID);
    err = nvs_set_u8(nvsHandle, key, config->direction ? 1 : 0);
    if (err != ESP_OK) goto nvs_error;

    // Save speed
    snprintf(key, sizeof(key), "motor%d_speed", motorID);
    err = nvs_set_u16(nvsHandle, key, config->speed);
    if (err != ESP_OK) goto nvs_error;

    // Save auto mode
    snprintf(key, sizeof(key), "motor%d_auto", motorID);
    err = nvs_set_u8(nvsHandle, key, config->autoMode ? 1 : 0);
    if (err != ESP_OK) goto nvs_error;

    // Commit changes
    err = nvs_commit(nvsHandle);
    if (err != ESP_OK) goto nvs_error;

    nvs_close(nvsHandle);
    nvs_close(nvsHandle);

    // Give the NVS mutex
    xSemaphoreGive(nvs_mutex);

    return ESP_OK;

nvs_error:
    Serial.printf("Failed to save motor configuration: %s\n", esp_err_to_name(err));
    nvs_close(nvsHandle);
    return err;
}

void event_handler_save_motor_config(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Save Motor Configuration button pressed.");

        MotorConfig config;
        uint8_t motorID = getSelectedMotorID(); // Assuming this function returns the motor ID

        // Lock LVGL before accessing UI elements
        lvgl_port_lock(-1);

        // Get direction from switch
        config.direction = lv_obj_has_state(ui_Screen3_Switch1, LV_STATE_CHECKED);

        // Get speed from roller
        config.speed = lv_roller_get_selected(ui_Screen3_Roller1) + MIN_SPEED_RPM;

        // Get auto/manual mode from switch
        config.autoMode = lv_obj_has_state(ui_Screen3_Switch3, LV_STATE_CHECKED);

        // Unlock LVGL
        lvgl_port_unlock();

        // Save configuration to NVS
        esp_err_t err = saveMotorConfigToNVS(motorID, &config);
        if (err == ESP_OK) {
            Serial.println("Motor configuration saved successfully.");
            // Optionally, display a message on the UI
        } else {
            Serial.println("Failed to save motor configuration.");
            // Optionally, display an error message on the UI
        }
    }
}

esp_err_t loadMotorConfigFromNVS(uint8_t motorID, MotorConfig *config) {
    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open("motor_config", NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        return err;
    }

    char key[16];
    uint8_t value_u8;
    uint16_t value_u16;

    // Load direction
    snprintf(key, sizeof(key), "motor%d_dir", motorID);
    err = nvs_get_u8(nvsHandle, key, &value_u8);
    if (err != ESP_OK) goto nvs_error;
    config->direction = (value_u8 != 0);

    // Load speed
    snprintf(key, sizeof(key), "motor%d_speed", motorID);
    err = nvs_get_u16(nvsHandle, key, &value_u16);
    if (err != ESP_OK) goto nvs_error;
    config->speed = value_u16;

    // Load auto mode
    snprintf(key, sizeof(key), "motor%d_auto", motorID);
    err = nvs_get_u8(nvsHandle, key, &value_u8);
    if (err != ESP_OK) goto nvs_error;
    config->autoMode = (value_u8 != 0);

    nvs_close(nvsHandle);
    Serial.printf("Motor %d configuration loaded from NVS.\n", motorID);
    return ESP_OK;

nvs_error:
    Serial.printf("Failed to load motor configuration: %s\n", esp_err_to_name(err));
    nvs_close(nvsHandle);
    return err;
}


// Assuming ui_Screen3_Dropdown1 is the motor selection dropdown
void event_handler_motor_selection(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
        Serial.println("Motor selection changed.");

        uint8_t motorID = getSelectedMotorID();

        // Load the configuration for the selected motor
        MotorConfig config;
        esp_err_t err = loadMotorConfigFromNVS(motorID, &config);
        if (err == ESP_OK) {
            Serial.println("Applying saved motor configuration.");

            // Lock LVGL before updating UI elements
            lvgl_port_lock(-1);

            // Set direction switch
            if (config.direction) {
                lv_obj_add_state(ui_Screen3_Switch1, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(ui_Screen3_Switch1, LV_STATE_CHECKED);
            }

            // Set speed roller
            lv_roller_set_selected(ui_Screen3_Roller1, config.speed - MIN_SPEED_RPM, LV_ANIM_OFF);

            // Set auto/manual switch
            if (config.autoMode) {
                lv_obj_add_state(ui_Screen3_Switch3, LV_STATE_CHECKED);
            } else {
                lv_obj_clear_state(ui_Screen3_Switch3, LV_STATE_CHECKED);
            }

            // Unlock LVGL
            lvgl_port_unlock();

            Serial.println("Motor configuration applied.");
        } else {
            Serial.println("No saved configuration for this motor.");
            // Optionally, reset UI elements to default values
        }
    }
}


/* Event Handler for Start Motor Button with Direction */
void event_handler_start_motor_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
 Serial.println("Start button pressed...");

         // Check if auto mode is selected
        bool isAutoMode = lv_obj_has_state(ui_Screen3_Switch3, LV_STATE_CHECKED);
        if (isAutoMode) {
            // Auto mode is selected, get the selected program from Roller3
            uint16_t programIndex = lv_roller_get_selected(ui_Screen3_Roller3);
            Serial.printf("Auto mode selected, running program %d\n", programIndex);

            // Allocate memory for ProgramData
            ProgramData *programData = (ProgramData*)malloc(sizeof(ProgramData));
            if (programData == NULL) {
                Serial.println("Failed to allocate memory for ProgramData.");
                return;
            }

            // Load the selected program from NVS
            esp_err_t result = loadProgramFromNVS(programIndex, programData);
            if (result == ESP_OK) {
                // Create a task to run the program
                xTaskCreate(runProgramTask, "Run Program Task", 2046, (void*)programData, 1, NULL);
                lv_obj_set_style_bg_color(ui_Screen1_Button4, lv_color_hex(0x151515),LV_PART_MAIN | LV_STATE_DEFAULT);
            } else {
                Serial.println("Failed to load program data.");
                free(programData);
            }
            
        } else {
         uint8_t motorID = getSelectedMotorID();

        // Lock the interface to safely interact with UI components
        lvgl_port_lock(-1);
        
        // Check if the switch is in the "forward" position
        bool isForward = lv_obj_has_state(ui_Screen3_Switch1, LV_STATE_CHECKED);
        lvgl_port_unlock(); // Unlock the interface after UI state is read

        // Determine the value to write to the VFD register (0x2000)
        uint16_t directionValue;
        if (isForward) {
            directionValue = 34; // Forward, setting value to 18
            Serial.println("Direction: Forward");
        } else {
            directionValue = 18; // Reverse, setting value to 34
            Serial.println("Direction: Reverse");
        }

        // Write the direction value to the VFD's register (0x2000) using Modbus
        uint16_t result = mb.writeHreg(motorID, 0x2000, directionValue, modbusCallback);

        // Check if Modbus write was successful
        if (result == 0) {
            Serial.println("Modbus write success.");
        } else {
            Serial.print("Modbus write failed with error: ");
            Serial.println(result); // Print error code if write fails
        }

        // Continue with other start motor logic
        Serial.printf("Sending start command to motor ID %d, register 0x%04X, value 0x%04X\n", motorID, 0x2000, directionValue);

        if (result == 0) {
            Serial.println("Start motor command with direction sent.");
            lvgl_port_lock(-1);
            lv_label_set_text(ui_Screen2_Label7, "Motor started.");
            lvgl_port_unlock();
        } else {
            Serial.println("Failed to send start motor command.");
            lvgl_port_lock(-1);
            lv_label_set_text(ui_Screen2_Label7, "Failed to start motor.");
            lvgl_port_unlock();
        }

        Serial.println("Motor started with the selected direction.");
        lv_obj_set_style_bg_color(ui_Screen1_Button4, lv_color_hex(0x151515),LV_PART_MAIN | LV_STATE_DEFAULT);
        }
    }
}


/* Event Handler for Stop Motor Button */
void event_handler_stop_motor_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        uint8_t motorID = getSelectedMotorID();

        // Write value to register to stop the motor
        uint16_t address = 0x2000; // Control register address (confirm with VFD manual)
        uint16_t value = 1;   // Command to stop motor (confirm with VFD manual)

        Serial.printf("Sending stop command to motor ID %d, register 0x%04X, value 0x%04X\n", motorID, address, value);

        if (mb.writeHreg(motorID, address, value, modbusCallback)) {
            Serial.println("Stop motor command sent.");
            lvgl_port_lock(-1);
            lv_label_set_text(ui_Screen2_Label7, "Motor stopped.");
            lvgl_port_unlock();
        } else {
            Serial.println("Failed to send stop motor command.");
            lvgl_port_lock(-1);
            lv_label_set_text(ui_Screen2_Label7, "Failed to stop motor.");
            lvgl_port_unlock();
        }
    }
}

/* Event Handler for Change Speed Button */
void event_handler_change_speed_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        uint8_t motorID = getSelectedMotorID();

        // Get the speed value from the Roller
        lvgl_port_lock(-1);
        uint16_t rpm = lv_roller_get_selected(ui_Screen3_Roller1) + MIN_SPEED_RPM;
        lv_label_set_text_fmt(ui_Screen1_Label11, " %u", rpm);       
        lvgl_port_unlock();
    
        // Convert RPM to Hertz
        float frequency = rpmToHertz(rpm);

        // Ensure frequency does not exceed VFD limits
        if (frequency > BASE_FREQUENCY) {
            frequency = BASE_FREQUENCY;
        }

        uint16_t frequencyValue = (uint16_t)(frequency * 100); // Assuming VFD accepts frequency in 0.01 Hz units

        // Send the frequency value to the VFD via Modbus
        uint16_t address = 0x2001; // Frequency reference register address (confirm with VFD manual)
        uint16_t value = frequencyValue; // The frequency value to set

        Serial.printf("Setting motor %d speed to %d RPM (%0.2f Hz). Register 0x%04X, value 0x%04X\n", motorID, rpm, frequency, address, value);

        if (mb.writeHreg(motorID, address, value, modbusCallback)) {
            Serial.println("Speed command sent.");
            lvgl_port_lock(-1);
            lv_label_set_text(ui_Screen2_Label7, "Speed command sent.");
            lvgl_port_unlock();
        } else {
            Serial.println("Failed to send speed command.");
            lvgl_port_lock(-1);
            lv_label_set_text(ui_Screen2_Label7, "Failed to send speed command.");
            lvgl_port_unlock();
        }
    }
}

void update_arc_values_from_label() {
    // Lock the LVGL mutex
    lvgl_port_lock(-1);

    // Get the text from the labels
    const char* label_text1 = lv_label_get_text(ui_Screen1_Label2);
    const char* label_text2 = lv_label_get_text(ui_Screen1_Label11);

    // Check if either label text is NULL
    if (label_text1 == NULL || label_text2 == NULL) {
        Serial.println("One or both label texts are NULL");
        lvgl_port_unlock();
        return;
    }

    Serial.printf("Label 1 text: %s\n", label_text1);
    Serial.printf("Label 2 text: %s\n", label_text2);

    // Unlock LVGL mutex as we don't need to hold it during parsing
    lvgl_port_unlock();

    // Convert label texts to numerical values
    char *endptr1;
    char *endptr2;
    int value1 = strtol(label_text1, &endptr1, 10);
    int value2 = strtol(label_text2, &endptr2, 10);

    // Check for invalid conversion
    if (*endptr1 != '\0') {
        Serial.printf("Invalid number in Label 1 text: %s\n", label_text1);
        value1 = 0; // Set to default or handle error as needed
    }

    if (*endptr2 != '\0') {
        Serial.printf("Invalid number in Label 2 text: %s\n", label_text2);
        value2 = 0; // Set to default or handle error as needed
    }

    // Clamp values to valid range (e.g., 0 to 100)
    if (value1 < 0) value1 = 0;
    if (value1 > 100) value1 = 100;

    if (value2 < 0) value2 = 0;
    if (value2 > 100) value2 = 100;

    Serial.printf("Converted value1: %d\n", value1);
    Serial.printf("Converted value2: %d\n", value2);

    // Lock LVGL mutex before updating UI elements
    lvgl_port_lock(-1);

    // Set the value of the arcs
    lv_arc_set_value(ui_Screen1_Arc1, value1);
    lv_arc_set_value(ui_Screen1_Arc2, value2);

    // Unlock the LVGL mutex
    lvgl_port_unlock();

    Serial.println("Arc values updated from labels.");
}





/* Wi-Fi Connection Task */
void wifi_connect_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(wifi_connect_semaphore, portMAX_DELAY)) {
            // Get the selected SSID from the dropdown
            char ssid[64];
            char password[64];
            lvgl_port_lock(-1);
            lv_dropdown_get_selected_str(ui_Screen2_Dropdown2, ssid, sizeof(ssid));
            const char *password_tmp = lv_textarea_get_text(ui_Screen2_TextArea2); // Get password from TextArea
            strncpy(password, password_tmp, sizeof(password));
            password[sizeof(password) - 1] = '\0'; // Ensure null-termination
            lvgl_port_unlock();

            Serial.printf("Attempting to connect to: %s\n", ssid);

            // Attempt to connect to the selected network
            WiFi.begin(ssid, password);

            // Wait for connection
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);
                Serial.print(".");
                attempts++;
            }

            // Check if connected
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\nWiFi connected successfully!");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());

                // Update the UI with connection success
                lvgl_port_lock(-1);
                lv_label_set_text_fmt(ui_Screen2_Label7, "Connected to %s", ssid);
                lvgl_port_unlock();
            } else {
                Serial.println("\nWiFi connection failed.");

                // Update the UI with connection failure
                lvgl_port_lock(-1);
                lv_label_set_text_fmt(ui_Screen2_Label7, "Failed to connect to %s", ssid);
                lvgl_port_unlock();
            }
        }
    }
}

/* Wi-Fi Scan Task */
void wifi_scan_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(wifi_scan_semaphore, portMAX_DELAY)) {
            Serial.println("Starting WiFi scan...");



            // Clear existing dropdown items
            lvgl_port_lock(-1);
            lv_dropdown_clear_options(ui_Screen2_Dropdown2);
            lvgl_port_unlock();

            // Start Wi-Fi scan
            int n = WiFi.scanNetworks();  // Scan for available networks
            Serial.println("Scan completed");

            // Check if any networks were found
            if (n == 0) {
                Serial.println("No networks found");
                lvgl_port_lock(-1);
                lv_dropdown_set_options(ui_Screen2_Dropdown2, "No networks found");
                lvgl_port_unlock();
            } else {
                Serial.printf("%d networks found:\n", n);
                String ssidList = "";  // Initialize empty string for storing SSIDs

                // Loop through the found networks and append them to ssidList
                for (int i = 0; i < n; ++i) {
                    String ssid = WiFi.SSID(i);  // Get the SSID of the network
                    Serial.printf("%d: %s (%d dBm)\n", i + 1, ssid.c_str(), WiFi.RSSI(i));
                    ssidList += ssid + "\n";  // Add a newline for each SSID
                }

                // Update dropdown options with available SSIDs
                lvgl_port_lock(-1);
                lv_dropdown_set_options(ui_Screen2_Dropdown2, ssidList.c_str());
                lvgl_port_unlock();
            }

            WiFi.scanDelete();  // Clean up the scanned networks from memory
        }
    }
}


/* Event Handlers for Wi-Fi Buttons */
void event_handler_scan_button(lv_event_t * e) {
    Serial.println("Scan button pressed, giving semaphore to WiFi scan task...");
    xSemaphoreGive(wifi_scan_semaphore);  // Unblock the Wi-Fi scan task
}

void event_handler_connect_button(lv_event_t * e) {
    Serial.println("Connect button pressed, giving semaphore to WiFi connection task...");
    xSemaphoreGive(wifi_connect_semaphore);  // Unblock the Wi-Fi connection task
}


/* Modbus Task */
void modbusTask(void *pvParameters) {
    while (1) {
        mb.task();
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
    }
}



uint16_t speed_value = 0;
// Mototime Callback Function
bool mototimeCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    if (event == Modbus::EX_SUCCESS) {
        Serial.println("Mototime read successful.");
        mototimeValue = speed_value; // Update the value
        mototimeUpdated = true;      // Set the flag
    } else {
        Serial.printf("Mototime read error: 0x%02X\n", event);
    }
    return true;
}

/* Task to read mototime from VFD */
void vfdReadTask(void *pvParameters) {
    while (1) {
        // Read mototime from the VFD
        uint16_t address = 0x2103; // Replace with the correct register address
        uint8_t motorID = getSelectedMotorID();
        Serial.println("Sending mototime read request.");

        if (mb.readHreg(motorID, address, &speed_value, 1, mototimeCallback)) {
            // The read operation was successfully initiated
            Serial.println("Mototime read command sent.");
        } else {
            Serial.println("Failed to send mototime read command.");
        }

        // Wait some time before next read
        vTaskDelay(pdMS_TO_TICKS(10000)); // Read every 5 seconds
    }
}

String getSerialFromServer() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin("https://schleiermacher34.pythonanywhere.com/devices/api/assign_serial_number/"); // Change to your server's URL
        int httpCode = http.GET();
        if (httpCode > 0) {
            String response = http.getString();
            Serial.println(response);
            // Parse the response to get the serial number
            // Save the serial number to NVS after receiving it
        } else {
            Serial.printf("Failed to retrieve serial number: %d\n", httpCode);
        }
        http.end();
    } else {
        Serial.println("WiFi not connected");
    }
    return ""; // Return the serial number or empty string on failure
}

void saveSerialToNVS(const char* serial) {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvsHandle);
    if (err == ESP_OK) {
        err = nvs_set_str(nvsHandle, "serial", serial);
        if (err == ESP_OK) {
            nvs_commit(nvsHandle);
            Serial.println("Serial number saved successfully.");
        } else {
            Serial.println("Failed to save serial number.");
        }
        nvs_close(nvsHandle);
    }
}

void sendAlarmToServer(uint16_t alarm_status) {
    // Create an HTTP client
    HTTPClient http;
    
    // Specify the server endpoint for error logging
    String serverUrl = "http://schleiermacher34.pythonanywhere.com/api/send-alarm";

    // Open a connection to the server
    http.begin(serverUrl);

    // Set the content type to JSON
    http.addHeader("Content-Type", "application/json");

    // Create a JSON object with the alarm status and other relevant info
    StaticJsonDocument<200> jsonDoc;
    jsonDoc["serial_number"] = "your_serial_number";  // Get serial number from NVS or config
    jsonDoc["alarm_status"] = alarm_status;

    // Convert the JSON object to a string
    String jsonString;
    serializeJson(jsonDoc, jsonString);

    // Send the POST request with the JSON payload
    int httpResponseCode = http.POST(jsonString);

    // Check the response from the server
    if (httpResponseCode > 0) {
        Serial.printf("POST request sent, response code: %d\n", httpResponseCode);
    } else {
        Serial.printf("Error in sending POST: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    // Close the connection
    http.end();
}



void sendErrorToServer(uint16_t errorCode) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin("http://schleiermacher34.pythonanywhere.com/log_error");
        http.addHeader("Content-Type", "application/json");
        DynamicJsonDocument doc(1024);
        doc["serial"] = "your_serial"; // Retrieve serial from NVS
        doc["error_code"] = errorCode;
        String requestBody;
        serializeJson(doc, requestBody);
        int httpCode = http.POST(requestBody);
        if (httpCode > 0) {
            Serial.printf("Server response: %s\n", http.getString().c_str());
        } else {
            Serial.println("Error sending log to server.");
        }
        http.end();
    } else {
        Serial.println("WiFi not connected");
    }
}
String readSerialFromNVS() {
    nvs_handle_t nvsHandle;
    char serial[32];
    size_t length = sizeof(serial);
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvsHandle);
    if (err == ESP_OK) {
        err = nvs_get_str(nvsHandle, "serial", serial, &length);
        if (err == ESP_OK) {
            Serial.printf("Serial number: %s\n", serial);
            return String(serial);
        }
        nvs_close(nvsHandle);
    }
    return "";
}

void checkVfdStatus() {
    uint16_t error_code;
    uint16_t motor_id = getSelectedMotorID();

    if (mb.readHreg(motor_id, 0x2101, &error_code, 1)) {
        char errorStr[10];
        sprintf(errorStr, "%d", error_code);  // Convert error code to string
        lv_label_set_text(ui_Screen2_Label8, errorStr);  // Update label with error code
        sendErrorToServer(error_code);  // Send error to the server
    } else {
        Serial.println("Error reading VFD Error Code.");
    }

    uint16_t alarm_status;
    if (mb.readHreg(motor_id, 0x2226, &alarm_status, 1)) {
        char alarmStr[10];
        sprintf(alarmStr, "%d", alarm_status);  // Convert alarm status to string
        lv_label_set_text(ui_Screen2_Label8, alarmStr);  // Update label with alarm status
        sendAlarmToServer(alarm_status);  // Make sure this function is defined
    } else {
        Serial.println("Error reading VFD Alarm Status.");
    }
}




void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.printf("Free heap: %u bytes\n", esp_get_free_heap_size());

    Serial.println("Setup starting...");
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Create LVGL mutex */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();

    /* Initialize LVGL buffers */
    static lv_disp_draw_buf_t draw_buf;
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    if (!buf) {
        Serial.println("Error: Failed to allocate LVGL buffer");
        while (1); // Halt if the buffer fails to allocate
    }
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    /* Initialize the display device */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

#if ESP_PANEL_USE_LCD_TOUCH
    /* Initialize the input device */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_tp_read;
    lv_indev_drv_register(&indev_drv);
#endif

/* Initialize bus and device of panel */
panel->init();  // Call the init function directly, since it returns void



#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    /* Register a function to notify LVGL when the panel is ready to flush */
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    /* Initialize IO expander */
    Serial.println("Initialize IO expander");
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create LVGL task */
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY + 1, NULL); // Set higher priority for LVGL task





    /* Initialize UI */
    ui_init();
        // Initialize NVS
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    // Initialize Wi-Fi in station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(); // Disconnect from any previous connections
    delay(100); // Short delay to ensure Wi-Fi is initialized

        // Initialize the NVS mutex
    nvs_mutex = xSemaphoreCreateMutex();
   // Check if the firmware is pending verification
    esp_ota_img_states_t otaState;
    const esp_partition_t* runningPartition = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(runningPartition, &otaState) == ESP_OK) {
        if (otaState == ESP_OTA_IMG_PENDING_VERIFY) {
            Serial.println("Firmware is pending verification. Confirming now...");

            // Perform any additional checks here (e.g., sensor initialization)
            // If everything is okay, confirm the firmware
            esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
            if (err == ESP_OK) {
                Serial.println("Firmware marked as valid.");
            } else {
                Serial.printf("Failed to mark firmware as valid: %s\n", esp_err_to_name(err));
                // Optionally, initiate a rollback
                // esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

 if(!buf) {
    Serial.println("Error: Failed to allocate LVGL buffer");
    while (1); // Halt if the buffer fails to allocate
}

    /* Attach event handlers (check for null pointers) */

    if (ui_Screen2_Button8) {
        lv_obj_add_event_cb(ui_Screen2_Button8, event_handler_scan_button, LV_EVENT_CLICKED, NULL);
    } else {
        Serial.println("ui_Screen2_Button8 is NULL");
    }

    if (ui_Screen2_Button9) {
        lv_obj_add_event_cb(ui_Screen2_Button9, event_handler_connect_button, LV_EVENT_CLICKED, NULL);
    } else {
        Serial.println("ui_Screen2_Button9 is NULL");
    }

    if (ui_Screen2_Button10) {
        lv_obj_add_event_cb(ui_Screen2_Button10, event_handler_ota_update, LV_EVENT_CLICKED, NULL);
    } else {
        Serial.println("ui_Screen2_Button10 is NULL");
    }

    if (ui_Screen3_Button13) {
        lv_obj_add_event_cb(ui_Screen3_Button13, event_handler_start_motor_button, LV_EVENT_CLICKED, NULL);
    } else {
        Serial.println("ui_Screen3_Button13 is NULL");
    }

    if (ui_Screen3_Button2) {
        lv_obj_add_event_cb(ui_Screen3_Button2, event_handler_stop_motor_button, LV_EVENT_CLICKED, NULL);
    } else {
        Serial.println("ui_Screen3_Button2 is NULL");
    }

    if (ui_Screen3_Button3) {
        lv_obj_add_event_cb(ui_Screen3_Button3, event_handler_change_speed_button, LV_EVENT_CLICKED, NULL);
    } else {
        Serial.println("ui_Screen3_Button3 is NULL");
    }
    if(ui_Screen5_Button3){
        lv_obj_add_event_cb(ui_Screen5_Button3, event_handler_save_program_button, LV_EVENT_CLICKED, NULL);
    }else{
        Serial.println("ui_screen5_button3 is null");
    }
    if (ui_Screen3_Button5) {
    lv_obj_add_event_cb(ui_Screen3_Button5, event_handler_save_motor_config, LV_EVENT_CLICKED, NULL);
    } else {
    Serial.println("ui_Screen3_Button5 is NULL");
    }
    if (ui_Screen3_Dropdown1) {
    lv_obj_add_event_cb(ui_Screen3_Dropdown1, event_handler_motor_selection, LV_EVENT_VALUE_CHANGED, NULL);
  } else {
    Serial.println("ui_Screen3_Dropdown1 is NULL");
  }
  if (ui_Screen1_Button4){
    lv_obj_add_event_cb(ui_Screen1_Button4, event_handler_stop_motor_button, LV_EVENT_CLICKED, NULL);
  }else{
    Serial.println("ui_Screen1_Button4 is NULL");
  }

  setupVsync();
    save_program_semaphore = xSemaphoreCreateBinary();
    
    wifi_connect_semaphore = xSemaphoreCreateBinary();
    wifi_scan_semaphore = xSemaphoreCreateBinary();
    // Create the OTA update task

    /* Create tasks */
    xTaskCreate(wifi_connect_task, "WiFi Connect Task", 2046, NULL, 1, NULL);
    xTaskCreate(wifi_scan_task, "WiFi Scan Task", 2046, NULL, 1, NULL);
    // xTaskCreate(guiTask, "GUI Task", 2046, NULL, 1, NULL);
  // xTaskCreate(otaUpdateTask, "OTA Update Task", 16384, NULL, 1, &otaTaskHandle); // Increase stack size
   // Create the OTA semaphore
    otaSemaphore = xSemaphoreCreateBinary();

    // Create the OTA update task
    xTaskCreate(otaUpdateTask, "OTA Update Task", 16384, NULL, 1, &otaTaskHandle);
    xTaskCreate(saveProgramTask, "Save Program Task", 2046, NULL, 1, NULL);

    /* Initialize Modbus Serial */
    pinMode(MODBUS_DE_RE_PIN, OUTPUT);
    digitalWrite(MODBUS_DE_RE_PIN, LOW); // Receiver enabled by default

    // Initialize Modbus serial port with new TX and RX pins
    ModbusSerial.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    if (!ModbusSerial) {
        Serial.println("Modbus Serial initialization failed");
        while (1); // Halt if Modbus initialization fails
    }

    mb.begin(&ModbusSerial, MODBUS_DE_RE_PIN); // Initialize Modbus with DE/RE pin
    mb.master(); // Set as Modbus master

    // Create Modbus task
    xTaskCreate(modbusTask, "Modbus Task", 2046, NULL, 2, NULL);

    Serial.println("Modbus initialized");
    Serial.println("Setup done");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
