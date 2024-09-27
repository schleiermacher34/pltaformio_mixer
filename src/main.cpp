#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <ui.c>
#include <HTTPClient.h>
#include <Update.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

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

/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
// #include <demos/lv_demos.h>
// #include <examples/lv_examples.h>

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)
const char* host = "127.0.0.1"; // Your server IP
const int port = 8000;
const char* firmware_version = "1.0.0"; // Current firmware version
TaskHandle_t otaTaskHandle = NULL; // Handle for the OTA task
SemaphoreHandle_t xGuiSemaphore; // Mutex for LVGL

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex

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

// Semaphore to control when the WiFi connection task runs
SemaphoreHandle_t wifi_connect_semaphore;

extern lv_obj_t *ui_Screen1_Dropdown1;  // Dropdown for available networks
//extern lv_obj_t *ui_Screen1_TextArea2;  // Text area for the password

// WiFi connection task
void wifi_connect_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given (unblock the task)
        if (xSemaphoreTake(wifi_connect_semaphore, portMAX_DELAY)) {
            // Get the selected SSID from the dropdown
            char ssid[64];
           lv_dropdown_get_selected_str(ui_Screen1_Dropdown1, ssid, sizeof(ssid));

            // Get the password from the text area
            const char *password = lv_textarea_get_text(ui_Screen1_TextArea2);

            Serial.print("Attempting to connect to: ");
            Serial.println(ssid);
            Serial.print("Using password: ");
            Serial.println(password);

            // Attempt to connect to the selected network
            WiFi.begin(ssid, password);

            // Wait for connection
            int attempts = 0;
            while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                delay(500);  // Give some delay between connection attempts
                Serial.print(".");
                attempts++;
            }

            // Check if connected
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\nWiFi connected successfully!");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());

                // Optionally: Update the UI with connection success
                lv_obj_t *label = lv_label_create(lv_scr_act());
                lv_label_set_text_fmt(label, "Connected to %s", ssid);
                lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);  // Align in the center of the screen
            } else {
                Serial.println("\nWiFi connection failed.");
                
                // Optionally: Update the UI with connection failure
                lv_obj_t *label = lv_label_create(lv_scr_act());
                lv_label_set_text_fmt(label, "Failed to connect to %s", ssid);
                lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);  // Align in the center of the screen
            }
        }
    }
}

// Event handler for the Connect button
void event_handler_connect_button(lv_event_t * e) {
    Serial.println("Connect button pressed, giving semaphore to WiFi connection task...");
    xSemaphoreGive(wifi_connect_semaphore);  // Unblock the WiFi connection task
}


SemaphoreHandle_t wifi_scan_semaphore;  // Binary semaphore for controlling the scan task
bool is_scan_running = false;  // Track the state of the scan task

void event_handler_scan_button(lv_event_t * e);

void wifi_scan_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given (unblock the task)
        if (xSemaphoreTake(wifi_scan_semaphore, portMAX_DELAY)) {
            Serial.println("Starting WiFi scan...");

            // Call the event handler for WiFi scanning
            event_handler_scan_button(NULL);  // This triggers the scan when the task runs

            Serial.println("WiFi scan completed, task now blocked.");
        }
        // Task will now block until the semaphore is given again
    }
}

// Full definition of event_handler_scan_button
void event_handler_scan_button(lv_event_t * e) {
    Serial.println("Scan button pressed, starting WiFi scan...");

    // Clear existing dropdown items
    lv_dropdown_clear_options(ui_Screen1_Dropdown1);

    int n = WiFi.scanNetworks();  // Scan for available networks
    Serial.println("Scan completed");

    // Check if any networks were found
    if (n == 0) {
        Serial.println("No networks found");
        lv_dropdown_set_options(ui_Screen1_Dropdown1, "No networks found");
    } else {
        Serial.println("Networks found:");
        String ssidList = "";  // Initialize empty string for storing SSIDs

        // Loop through the found networks and append them to ssidList
        for (int i = 0; i < n; ++i) {
            String ssid = WiFi.SSID(i);  // Get the SSID of the network
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(ssid);
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));  // Signal strength
            Serial.println(" dBm)");

            // Append SSID to the list
            ssidList += ssid + "\n";  // Add a newline for each SSID
        }

        // Update dropdown options with available SSIDs
        lv_dropdown_set_options(ui_Screen1_Dropdown1, ssidList.c_str());
    }

    WiFi.scanDelete();  // Clean up the scanned networks from memory
}
extern lv_obj_t *ui_Screen1_Button4; 

void guiTask(void* pvParameter) {
    (void) pvParameter;
    for (;;) {
        // Take the mutex before calling lv_task_handler()
        if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY) == pdTRUE) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Adjust the delay as needed
    }
}

void performOTA() {
    HTTPClient http;
    String firmwareUrl = String("http://") + host + ":" + String(port) + "/firmware/";

    http.begin(firmwareUrl);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
        int contentLength = http.getSize();
        WiFiClient* stream = http.getStreamPtr();

        if (Update.begin(contentLength)) {
            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                lv_label_set_text(ui_Screen1_Label5, "Updating...");
                xSemaphoreGive(xGuiSemaphore);
            }

            size_t written = Update.writeStream(*stream);

            if (written == contentLength) {
                Serial.println("OTA Update Success!");

                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                    lv_label_set_text(ui_Screen1_Label5, "Update successful!");
                    xSemaphoreGive(xGuiSemaphore);
                }
            } else {
                Serial.println("OTA Update Failed.");

                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                    lv_label_set_text(ui_Screen1_Label5, "Update failed.");
                    xSemaphoreGive(xGuiSemaphore);
                }
            }

            if (Update.end()) {
                if (Update.isFinished()) {
                    Serial.println("Update finished. Rebooting.");

                    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                        lv_label_set_text(ui_Screen1_Label5, "Rebooting...");
                        xSemaphoreGive(xGuiSemaphore);
                    }
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    ESP.restart();
                } else {
                    Serial.println("Update not finished. Something went wrong.");

                    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                        lv_label_set_text(ui_Screen1_Label5, "Update incomplete.");
                        xSemaphoreGive(xGuiSemaphore);
                    }
                }
            } else {
                Serial.printf("Error Occurred. Error #: %d\n", Update.getError());

                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                    lv_label_set_text(ui_Screen1_Label5, "Update error.");
                    xSemaphoreGive(xGuiSemaphore);
                }
            }
        } else {
            Serial.println("Not enough space for OTA");

            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                lv_label_set_text(ui_Screen1_Label5, "Not enough space for update.");
                xSemaphoreGive(xGuiSemaphore);
            }
        }
    } else {
        Serial.println("Failed to download firmware.");

        if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_label_set_text(ui_Screen1_Label5, "Download failed.");
            xSemaphoreGive(xGuiSemaphore);
        }
    }
    http.end();
}


void checkForUpdates() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        String versionUrl = String("http://") + host + ":" + String(port) + "/version/";

        http.begin(versionUrl);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_OK) {
            String newVersion = http.getString();
            newVersion.trim();

            if (newVersion.equals(firmware_version)) {
                Serial.println("No new updates.");

                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                    lv_label_set_text(ui_Screen1_Label5, "No new updates.");
                    xSemaphoreGive(xGuiSemaphore);
                }
            } else {
                Serial.println("Update available!");

                if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                    lv_label_set_text(ui_Screen1_Label5, "Update available!");
                    xSemaphoreGive(xGuiSemaphore);
                }
                performOTA();
            }
        } else {
            Serial.println("Failed to check for updates.");

            if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
                lv_label_set_text(ui_Screen1_Label5, "Update check failed.");
                xSemaphoreGive(xGuiSemaphore);
            }
        }
        http.end();
    } else {
        Serial.println("Not connected to Wi-Fi");

        if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_label_set_text(ui_Screen1_Label5, "Not connected to Wi-Fi.");
            xSemaphoreGive(xGuiSemaphore);
        }
    }
}


void otaUpdateTask(void* parameter) {
    // Indicate the start of the update
    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        lv_label_set_text(ui_Screen1_Label5, "Checking for updates...");
        xSemaphoreGive(xGuiSemaphore);
    }

    // Perform the OTA update
    checkForUpdates();

    // Re-enable the update button
    if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        lv_obj_add_flag(ui_Screen1_Button4, LV_OBJ_FLAG_CLICKABLE);
        xSemaphoreGive(xGuiSemaphore);
    }

    // Delete the task when done
    vTaskDelete(NULL);
}
static void update_button_event_handler(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        // Disable the button to prevent multiple presses
        if (xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_obj_clear_flag(ui_Screen1_Button4, LV_OBJ_FLAG_CLICKABLE);
            xSemaphoreGive(xGuiSemaphore);
        }

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



void setup() {
 Serial.begin(115200); /* prepare for possible serial debug */

    String LVGL_Arduino = "Hello LVGL! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am ESP32_Display_Panel");

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize LVGL buffers */
    static lv_disp_draw_buf_t draw_buf;
    /* Using double buffers is more faster than single buffer */
    /* Using internal SRAM is more fast than PSRAM (Note: Memory allocated using `malloc` may be located in PSRAM.) */
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    /* Initialize the display device */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
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
    /* This is useful for refreshing the screen using DMA transfers */
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    /**
     * These development boards require the use of an IO expander to configure the screen,
     * so it needs to be initialized in advance and registered with the panel for use.
     *
     */
    Serial.println("Initialize IO expander");
    /* Initialize IO expander */
    // ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);

    // Turn off backlight
    // expander->digitalWrite(USB_SEL, LOW);
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create a task to run the LVGL task periodically */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
    ui_init();

    xGuiSemaphore = xSemaphoreCreateMutex();
    if (xGuiSemaphore == NULL) {
        Serial.println("Failed to create GUI semaphore");
        // Handle the error accordingly
    }

    // Start the GUI task
    xTaskCreate(guiTask, "gui", 4096, NULL, 1, NULL);

    // Attach the event handler to the button
    lv_obj_add_event_cb(ui_Screen1_Button4, update_button_event_handler, LV_EVENT_ALL, NULL);
    // Attach the event handler for the scan button
    lv_obj_add_event_cb(ui_Screen1_Button2, event_handler_scan_button, LV_EVENT_CLICKED, NULL);
    
    /* Create the LVGL task to periodically handle UI
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);*/

        // Initialize the semaphore (binary)
    wifi_scan_semaphore = xSemaphoreCreateBinary();
    
    // Create the WiFi scan task (it will wait on the semaphore)
    xTaskCreate(wifi_scan_task, "WiFi Scan Task", 4096, NULL, 1, NULL);

    // Create the binary semaphore for WiFi connection task control
    wifi_connect_semaphore = xSemaphoreCreateBinary();

    // Create the WiFi connection task (it will wait on the semaphore)
    xTaskCreate(wifi_connect_task, "WiFi Connect Task", 4096, NULL, 1, NULL);

    // Attach the event handler for the connect button
    lv_obj_add_event_cb(ui_Screen1_Button1, event_handler_connect_button, LV_EVENT_CLICKED, NULL);
    
    Serial.println("Setup done");
}

void loop() {
    // Keep the system in a low-power state while waiting for tasks to run
    delay(1000);
}
