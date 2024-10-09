#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
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

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

/* Server configuration */
const char* host = "192.168.1.100"; // Replace with your Django server IP
const int port = 8000;
String currentVersion = "1.0.0"; // Current firmware version

/* Task and Semaphore Handles */
TaskHandle_t otaTaskHandle = NULL;
SemaphoreHandle_t lvgl_mux = NULL; // LVGL mutex
SemaphoreHandle_t wifi_connect_semaphore = NULL;
SemaphoreHandle_t wifi_scan_semaphore = NULL;

ESP_Panel *panel = NULL;

// Forward declaration of otaUpdateTask
void otaUpdateTask(void* parameter);

/* Modbus Configuration */
#define MODBUS_TX_PIN 44        // GPIO44 (U0TXD)
#define MODBUS_RX_PIN 43        // GPIO43 (U0RXD)
#define MODBUS_DE_RE_PIN 4      // GPIO4 (DE/RE control pin)

HardwareSerial ModbusSerial(0); // Use UART0
ModbusRTU mb;                   // ModbusRTU instance

#define MAX_SPEED_RPM 60        // Maximum speed in RPM
#define MIN_SPEED_RPM 10        // Minimum speed in RPM
#define MOTOR_POLES 2           // Number of motor poles (adjust as per your motor)
#define BASE_FREQUENCY 50.0     // Base frequency in Hertz (adjust as per your motor)

/* Function to Convert RPM to Hertz */
float rpmToHertz(int rpm) {
    // Formula: Frequency (Hz) = (RPM × Number of Poles) / 120
    return (rpm * MOTOR_POLES) / 120.0;
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

void lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

void lvgl_port_task(void *arg)
{
    Serial.println("Starting LVGL task");

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_port_unlock();
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

/* UI Elements */
// Settings Screen (Screen2)
extern lv_obj_t *ui_Screen2_Button8;    // Scan button
extern lv_obj_t *ui_Screen2_Button9;    // Connect button
extern lv_obj_t *ui_Screen2_Button10;   // Update button
extern lv_obj_t *ui_Screen2_Dropdown2;  // Wi-Fi list
extern lv_obj_t *ui_Screen2_Label21;    // Messages label

// Right Motor Control Screen (Screen3)
extern lv_obj_t *ui_Screen3_Button13;   // Start motor button
extern lv_obj_t *ui_Screen3_Button2;    // Stop motor button
extern lv_obj_t *ui_Screen3_Button3;    // Change speed button
extern lv_obj_t *ui_Screen3_Switch1;    // Direction switch (forward/reverse)
extern lv_obj_t *ui_Screen3_Roller1;    // Speed roller (RPM)


// Left Motor Control Screen (Screen6)
extern lv_obj_t *ui_Screen6_Button4;   // Start motor button
extern lv_obj_t *ui_Screen6_Button5;    // Stop motor button
extern lv_obj_t *ui_Screen6_Button6;    // Change speed button
extern lv_obj_t *ui_Screen6_Switch4;    // Direction switch (forward/reverse)
extern lv_obj_t *ui_Screen6_Roller1;    // Speed roller (RPM)


/* Modbus Callback Function */
bool modbusCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    if (event != Modbus::EX_SUCCESS) {
        Serial.printf("Modbus error: 0x%02X\n", event);
        // Update status labels accordingly
        lvgl_port_lock(-1);
        lv_label_set_text_fmt(ui_Screen2_Label21, "Modbus error: 0x%02X", event);
        lv_label_set_text_fmt(ui_Screen2_Label21, "Modbus error: 0x%02X", event);
        lvgl_port_unlock();
    } else {
        Serial.println("Modbus transaction successful");
        // Additional handling if needed
    }
    return true;
}

/* Event Handler for Start Motor Button */
void event_handler_start_motor_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        uint8_t motorID;
        lv_obj_t *switch_obj;
        lv_obj_t *label_status;

        // Determine which screen the event came from
        lv_obj_t *obj = lv_event_get_target(e);
        if (obj == ui_Screen3_Button13) {
            motorID = 1; // Right motor
            switch_obj = ui_Screen3_Switch1;
            label_status = ui_Screen2_Label21;
        } else if (obj == ui_Screen6_Button4) {
            motorID = 2; // Left motor
            switch_obj = ui_Screen6_Switch4;
            label_status = ui_Screen2_Label21;
        } else {
            return; // Unknown button
        }

        // Write value to register to start the motor
        uint16_t address = 0x2000; // Control register address (confirm with VFD manual)
        uint16_t value = 2;   // Command to start motor (confirm with VFD manual)

        // Set direction based on switch state
        lvgl_port_lock(-1);
        bool direction = lv_obj_has_state(switch_obj, LV_STATE_CHECKED);
        lvgl_port_unlock();

        if (direction) {
            // Reverse direction
            value |= 0x0002; // Set the reverse bit (confirm with VFD manual)
        }

        Serial.printf("Sending start command to slave ID %d, register 0x%04X, value 0x%04X\n", motorID, address, value);

        if (mb.writeHreg(motorID, address, value, modbusCallback)) {
            Serial.println("Start motor command sent.");
            lvgl_port_lock(-1);
            lv_label_set_text(label_status, "Motor start command sent.");
            lvgl_port_unlock();
        } else {
            Serial.println("Failed to send start motor command.");
            lvgl_port_lock(-1);
            lv_label_set_text(label_status, "Failed to send start command.");
            lvgl_port_unlock();
        }
    }
}

/* Event Handler for Stop Motor Button */
void event_handler_stop_motor_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        uint8_t motorID;
        lv_obj_t *label_status;

        // Determine which screen the event came from
        lv_obj_t *obj = lv_event_get_target(e);
        if (obj == ui_Screen3_Button2) {
            motorID = 1; // Right motor
            label_status = ui_Screen2_Label21;
        } else if (obj == ui_Screen6_Button5) {
            motorID = 2; // Left motor
            label_status = ui_Screen2_Label21;
        } else {
            return; // Unknown button
        }

        // Write value to register to stop the motor
        uint16_t address = 0x2000; // Control register address (confirm with VFD manual)
        uint16_t value = 1;   // Command to stop motor (confirm with VFD manual)

        Serial.printf("Sending stop command to slave ID %d, register 0x%04X, value 0x%04X\n", motorID, address, value);

        if (mb.writeHreg(motorID, address, value, modbusCallback)) {
            Serial.println("Stop motor command sent.");
            lvgl_port_lock(-1);
            lv_label_set_text(label_status, "Motor stop command sent.");
            lvgl_port_unlock();
        } else {
            Serial.println("Failed to send stop motor command.");
            lvgl_port_lock(-1);
            lv_label_set_text(label_status, "Failed to send stop command.");
            lvgl_port_unlock();
        }
    }
}

/* Event Handler for Change Speed Button */
void event_handler_change_speed_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        uint8_t motorID;
        lv_obj_t *roller_obj;
        lv_obj_t *label_status;

        // Determine which screen the event came from
        lv_obj_t *obj = lv_event_get_target(e);
        if (obj == ui_Screen3_Button3) {
            motorID = 1; // Right motor
            roller_obj = ui_Screen3_Roller1;
            label_status = ui_Screen2_Label21;
        } else if (obj == ui_Screen6_Button6) {
            motorID = 2; // Left motor
            roller_obj = ui_Screen6_Roller1;
            label_status = ui_Screen2_Label21;
        } else {
            return; // Unknown button
        }

        // Get the speed value from the Roller
        lvgl_port_lock(-1);
        uint16_t rpm = lv_roller_get_selected(roller_obj) + MIN_SPEED_RPM;
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
            lv_label_set_text(label_status, "Speed command sent.");
            lvgl_port_unlock();
        } else {
            Serial.println("Failed to send speed command.");
            lvgl_port_lock(-1);
            lv_label_set_text(label_status, "Failed to send speed command.");
            lvgl_port_unlock();
        }
    }
}

/* Wi-Fi Connection Task */
void wifi_connect_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(wifi_connect_semaphore, portMAX_DELAY)) {
            // Get the selected SSID from the dropdown
            char ssid[64];
            lvgl_port_lock(-1);
            lv_dropdown_get_selected_str(ui_Screen2_Dropdown2, ssid, sizeof(ssid));
            const char *password = ""; // Assume no password or get from a TextArea if available
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
                lv_label_set_text_fmt(ui_Screen2_Label21, "Connected to %s", ssid);
                lvgl_port_unlock();
            } else {
                Serial.println("\nWiFi connection failed.");

                // Update the UI with connection failure
                lvgl_port_lock(-1);
                lv_label_set_text_fmt(ui_Screen2_Label21, "Failed to connect to %s", ssid);
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

void update_button_event_handler(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        // Disable the button to prevent multiple presses
        lvgl_port_lock(-1);
        lv_obj_add_state(ui_Screen2_Button10, LV_STATE_DISABLED);
        lvgl_port_unlock();

        // Start the OTA update task
        xTaskCreate(
            otaUpdateTask,
            "OTA Update Task",
            8192,
            NULL,
            1,
            &otaTaskHandle
        );
    }
}

/* Modbus Task */
void modbusTask(void *pvParameters) {
    while (1) {
        mb.task();
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
    }
}

/* LVGL Task */
void guiTask(void* pvParameter) {
    (void) pvParameter;
    for (;;) {
        // Lock the mutex before calling lv_task_handler()
        lvgl_port_lock(-1);
        lv_task_handler();
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize LVGL buffers */
    static lv_disp_draw_buf_t draw_buf;
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf);
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
    panel->init();
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

    /* Create LVGL mutex */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();

    /* Create LVGL task */
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    /* Initialize NVS */
    nvs_flash_init();

    /* Check OTA update state */
    esp_ota_img_states_t ota_state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // We need to confirm the new app
            Serial.println("New app is pending verification...");
            // For now, we'll assume the app is working correctly
            esp_ota_mark_app_valid_cancel_rollback();
            Serial.println("App marked as valid.");
        }
    }

    /* Initialize UI */
    ui_init();

    /* Attach event handlers */
    // Settings Screen
    lv_obj_add_event_cb(ui_Screen2_Button8, event_handler_scan_button, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Screen2_Button9, event_handler_connect_button, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Screen2_Button10, update_button_event_handler, LV_EVENT_CLICKED, NULL);

    // Right Motor Control Screen (Screen3)
    lv_obj_add_event_cb(ui_Screen3_Button13, event_handler_start_motor_button, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Screen3_Button2, event_handler_stop_motor_button, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Screen3_Button3, event_handler_change_speed_button, LV_EVENT_CLICKED, NULL);

    // Left Motor Control Screen (Screen6)
    lv_obj_add_event_cb(ui_Screen6_Button4, event_handler_start_motor_button, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Screen6_Button5, event_handler_stop_motor_button, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_Screen6_Button6, event_handler_change_speed_button, LV_EVENT_CLICKED, NULL);

    /* Create semaphores */
    wifi_connect_semaphore = xSemaphoreCreateBinary();
    wifi_scan_semaphore = xSemaphoreCreateBinary();

    /* Create tasks */
    xTaskCreate(wifi_connect_task, "WiFi Connect Task", 4096, NULL, 1, NULL);
    xTaskCreate(wifi_scan_task, "WiFi Scan Task", 4096, NULL, 1, NULL);
    xTaskCreate(guiTask, "GUI Task", 4096, NULL, 1, NULL);
    xTaskCreate(modbusTask, "Modbus Task", 4096, NULL, 1, NULL); // Modbus task

    /* Initialize Modbus Serial */
    pinMode(MODBUS_DE_RE_PIN, OUTPUT);
    digitalWrite(MODBUS_DE_RE_PIN, LOW); // Receiver enabled by default

    // Initialize Modbus serial port with new TX and RX pins
    ModbusSerial.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);

    mb.begin(&ModbusSerial, MODBUS_DE_RE_PIN); // Initialize Modbus with DE/RE pin
    mb.master(); // Set as Modbus master

    Serial.println("Modbus initialized");
    Serial.println("Setup done");
}

void loop() {
    // Keep the system in a low-power state while waiting for tasks to run
    vTaskDelay(pdMS_TO_TICKS(1000));
}
