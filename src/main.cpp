#define USE_DMA

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <memory.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

// config for various devices
#include "lcd_config.h"
#ifdef M5STACK_CORE2
// cross platform i2c init codewitch-honey-crisis/htcw_esp_i2c
#include <esp_i2c.hpp>
// core2 power management codewitch-honey-crisis/htcw_m5core2_power
#include <m5core2_power.hpp>
#endif

// import the appropriate namespace for our other libraries
#ifdef ARDUINO
namespace arduino {}
using namespace arduino;
#else
namespace esp_idf {}
using namespace esp_idf;
#endif
#ifdef M5STACK_CORE2
// declare the power management driver
m5core2_power power(esp_i2c<1, 21, 22>::instance);
#endif

// lcd data
// This works out to be 32KB - the max DMA transfer size
static const size_t lcd_transfer_buffer_size = 320*24*2; // (10 refreshes per fill)
// for sending data to the display
static uint8_t *lcd_transfer_buffer = nullptr;
#ifdef USE_DMA
static uint8_t *lcd_transfer_buffer2 = nullptr;
#endif
// 0 = no flushes in progress, otherwise flushing
static volatile int lcd_flushing = 0;
static esp_lcd_panel_handle_t lcd_handle = nullptr;
#ifdef USE_DMA
static int lcd_dma_sel = 0;
#endif
static uint8_t *lcd_current_buffer() {
#ifdef USE_DMA
    return lcd_dma_sel ? lcd_transfer_buffer2 : lcd_transfer_buffer;
#else
    return lcd_transfer_buffer;
#endif

}
static void lcd_switch_buffers() {
#ifdef USE_DMA
    lcd_dma_sel++;
    if (lcd_dma_sel > 1) {
        lcd_dma_sel = 0;
    }
#endif
}
// indicates the LCD DMA transfer is complete
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                            esp_lcd_panel_io_event_data_t *edata,
                            void *user_ctx) {
    lcd_flushing = 0;
    return true;
}
// waits for there to be a free buffer
static void lcd_wait_dma() { 
    while (lcd_flushing > 0);
 }

// indicates we're about to start drawing
static void lcd_begin_draw() {
    lcd_flushing = 1;
}
// flush a bitmap to the display
static void lcd_flush_bitmap(int x1, int y1, int x2, int y2,
                             const void *bitmap) {
    // adjust end coordinates for a quirk of Espressif's API (add 1 to each)
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2 + 1, y2 + 1,
                              (void *)bitmap);
}
// initialize the screen using the esp panel API
// htcw_gfx no longer has intrinsic display driver support
// for performance and flash size reasons
// here we use the ESP LCD Panel API for it
static void lcd_panel_init() {
#ifdef LCD_PIN_NUM_BCKL
    if(LCD_PIN_NUM_BCKL>-1) {
        gpio_set_direction((gpio_num_t)LCD_PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)4, LCD_BCKL_OFF_LEVEL);
    }
#endif
    // configure the SPI bus
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = LCD_PIN_NUM_CLK;
    buscfg.mosi_io_num = LCD_PIN_NUM_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    // declare enough space for the transfer buffers + 8 bytes SPI DMA overhead
    buscfg.max_transfer_sz = lcd_transfer_buffer_size + 8;

    // Initialize the SPI bus
    spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = LCD_PIN_NUM_DC;
    io_config.cs_gpio_num = LCD_PIN_NUM_CS;
    io_config.pclk_hz = LCD_PIXEL_CLOCK_HZ; // 40Mhz
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config,
                             &io_handle);

    lcd_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = LCD_PIN_NUM_RST;
    if(LCD_COLOR_SPACE==ESP_LCD_COLOR_SPACE_RGB) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        panel_config.rgb_endian = LCD_RGB_ENDIAN_RGB;
#else
        panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
#endif
    } else {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
#else
        panel_config.color_space = ESP_LCD_COLOR_SPACE_BGR;
#endif
    }
    panel_config.bits_per_pixel = LCD_BIT_DEPTH;

    // Initialize the LCD configuration
    if (ESP_OK !=
        LCD_PANEL(io_handle, &panel_config, &lcd_handle)) {
        printf("Error initializing LCD panel.\n");
        while (1) vTaskDelay(5);
    }

    // Reset the display
    esp_lcd_panel_reset(lcd_handle);

    // Initialize LCD panel
    esp_lcd_panel_init(lcd_handle);
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, LCD_SWAP_XY);
    esp_lcd_panel_set_gap(lcd_handle, LCD_GAP_X, LCD_GAP_Y);
    esp_lcd_panel_mirror(lcd_handle, LCD_MIRROR_X, LCD_MIRROR_Y);
    esp_lcd_panel_invert_color(lcd_handle, LCD_INVERT_COLOR);
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, true);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif
#ifdef LCD_PIN_NUM_BCKL
    // Turn on backlight (Different LCD screens may need different levels)
    if(LCD_PIN_NUM_BCKL>-1) gpio_set_level((gpio_num_t)4, LCD_BCKL_ON_LEVEL);
#endif

    // initialize the transfer buffers
    lcd_transfer_buffer = (uint8_t *)malloc(lcd_transfer_buffer_size);
    if (lcd_transfer_buffer == nullptr) {
        puts("Out of memory initializing primary transfer buffer");
        while (1) vTaskDelay(5);
    }
    memset(lcd_transfer_buffer, 0, lcd_transfer_buffer_size);
#ifdef USE_DMA
    // initialize the transfer buffers
    lcd_transfer_buffer2 = (uint8_t *)malloc(lcd_transfer_buffer_size);
    if (lcd_transfer_buffer2 == nullptr) {
        puts("Out of memory initializing transfer buffer 2");
        while (1) vTaskDelay(5);
    }
    memset(lcd_transfer_buffer2, 0, lcd_transfer_buffer_size);
#endif
}


#ifdef ARDUINO
// entry point (arduino)
void setup() {
    Serial.begin(115200);
    setCpuFrequencyMhz(240);
#else
// arduino compat shims:
static uint32_t millis() { return pdTICKS_TO_MS(xTaskGetTickCount()); }
void loop();
static void loop_task(void* arg) {
    while(1) {
        static int count = 0;
        loop();
        // tickle the watchdog periodically
        if (count++ == 4) {
            count = 0;
            vTaskDelay(5);
        }
    }
}
// entry point (esp-idf)
extern "C" void app_main() {
#endif
#ifdef M5STACK_CORE2
    // initialize the AXP192 in the core 2
    power.initialize();
#endif
    // initialize the LCD
    lcd_panel_init();
    // for ESP-IDF
#ifndef ARDUINO
    TaskHandle_t loop_handle;
    xTaskCreate(loop_task,"loop_task",4096,nullptr,20,&loop_handle);
#endif
}
void loop() {
    // timestamp for start
    uint32_t ms = millis();
    static char szfps[64]={0};
    // render the frame    
    static bool toggle = false;
    for(int y = 0;y<LCD_HEIGHT;y+=24) {
        lcd_wait_dma();
        lcd_begin_draw();
        memset(lcd_current_buffer(),0xFF*toggle,lcd_transfer_buffer_size);
        lcd_flush_bitmap(0,y,319,y+23,lcd_current_buffer());
        lcd_switch_buffers();
        toggle = !toggle;
    }
    toggle = !toggle;
   
    
    
    // The following keeps track of statistics
    static int frames = 0;
    static uint32_t fps_ts = 0;
    static uint32_t total_ms = 0;
    ++frames;

    if (ms >= fps_ts + 1000)
    {
        fps_ts = ms;
        
        if(frames==0) {
            snprintf(szfps, sizeof(szfps), "fps: < 1, %d ms", (int)total_ms);
        } else {
            snprintf(szfps, sizeof(szfps), "fps: %d, avg: %d ms", (int)frames,(int)total_ms/frames);
        }
    
        puts(szfps);
        frames = 0;
        total_ms = 0;
    }

    total_ms+=(millis()-ms);
}