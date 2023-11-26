/*******************************************************************************
 * LVGL Widgets
 * This is a widgets demo for LVGL - Light and Versatile Graphics Library
 * import from: https://github.com/lvgl/lv_demos.git
 *
 * This was created from the project here 
 * https://www.makerfabs.com/sunton-esp32-s3-4-3-inch-ips-with-touch.html
 * 
 * Dependent libraries:
 * LVGL: https://github.com/lvgl/lvgl.git

 * Touch libraries:
 * FT6X36: https://github.com/strange-v/FT6X36.git
 * GT911: https://github.com/TAMCTec/gt911-arduino.git
 * XPT2046: https://github.com/PaulStoffregen/XPT2046_Touchscreen.git
 *
 * LVGL Configuration file:
 * Copy your_arduino_path/libraries/lvgl/lv_conf_template.h
 * to your_arduino_path/libraries/lv_conf.h
 * Then find and set:
 * #define LV_COLOR_DEPTH     16
 * #define LV_TICK_CUSTOM     1
 *
 * For SPI display set color swap can be faster, parallel screen don't set!
 * #define LV_COLOR_16_SWAP   1
 *
 * Optional: Show CPU usage and FPS count
 * #define LV_USE_PERF_MONITOR 1
 ******************************************************************************/
//#include "lv_demo_widgets.h"
//#include <Arduino.h>
#include <lvgl.h>
#include <demos/lv_demos.h>
#include <Arduino_GFX_Library.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define TFT_BL 2
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin
 
unsigned long previousMillis = 0;  
const long interval = 5000;

void Create_Screen1(void);
void Create_Screen2(void);
void Create_Screen3(void);
void button_event_cb(lv_event_t *event);

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */,
    0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 8 /* hsync_back_porch */,
    0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 8 /* vsync_back_porch */,
    0 /* pclk_active_neg */, 13000000 /* prefer_speed */);
    
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    800 /* width */, 480 /* height */, rgbpanel);

#include "touch.h"


void button_event_cb(lv_event_t *event);


/* Change to your screen resolution */
static uint32_t screenWidth;
static uint32_t screenHeight;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  if (touch_has_signal())
  {
    if (touch_touched())
    {
      data->state = LV_INDEV_STATE_PR;
      /*Set the coordinates*/
      data->point.x = touch_last_x;
      data->point.y = touch_last_y;
    }
    else if (touch_released())
    {
      data->state = LV_INDEV_STATE_REL;
    }
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

lv_obj_t *screen1;
lv_obj_t *screen2;
lv_obj_t *screen3;

void setup()
{
  Serial.begin(115200);

    // Init touch device
    // Init Display
    gfx->begin();
    #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    #endif
    gfx->fillScreen(RED);
    delay(500);
    // gfx->fillScreen(GREEN);
    // delay(500);
    // gfx->fillScreen(RED);
    // delay(500);
    // gfx->fillScreen(GREEN);
    // delay(500);
    // gfx->fillScreen(RED);
    // delay(500);

    lv_init();
    delay(10);
    touch_init();

    screenWidth = gfx->width();
    screenHeight = gfx->height();

    #ifdef ESP32
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    #else
    disp_draw_buf = (lv_color_t *)malloc(sizeof(lv_color_t) * screenWidth * screenHeight / 4);
    #endif
    if (!disp_draw_buf)
    {
        printf("LVGL disp_draw_buf allocate failed!\n");
    }
    else
    {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * screenHeight / 4);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /* Initialize the (dummy) input device driver */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    //******************************* MY CODE *******************************************
    //Dim the TFT backlight
    analogWrite(TFT_BL, 255 / 1.5); // values go from 0 to 255,

    //Three(3) Screen Example
    Create_Screen1();       //Create and load the first screen
    lv_scr_load(screen1);

    printf("Setup done\n");
  }
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    //espnow_senddata();
  }
}


//**********************  PROCEDURES  *************************************************
void Create_Screen1(void)
{
  screen1 = lv_obj_create(NULL);

  lv_obj_t * button1 = lv_btn_create(screen1);
  lv_obj_add_event_cb(button1, button_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(button1, 170, 80);
  lv_obj_set_pos(button1, 32, 100);
  lv_obj_t * label1 = lv_label_create(button1);
  lv_label_set_text(label1, "Test");

  lv_obj_t * button2 = lv_btn_create(screen1);
  lv_obj_add_event_cb(button2, button_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(button2, 170, 80);
  lv_obj_set_pos(button2, 249, 100);
  lv_obj_t * label2 = lv_label_create(button2);
  lv_label_set_text(label2, "Goto_Screen2");
}

void Create_Screen2(void){
  screen2 = lv_obj_create(NULL);

  lv_obj_t * button3 = lv_btn_create(screen2);
  lv_obj_add_event_cb(button3, button_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(button3, 170, 80);
  lv_obj_set_pos(button3, 249, 100);
  lv_obj_t * label3 = lv_label_create(button3);
  lv_label_set_text(label3, "Goto_Screen3");
}

void Create_Screen3(void){
  screen3 = lv_obj_create(NULL);

  lv_obj_t * button4 = lv_btn_create(screen3);
  lv_obj_add_event_cb(button4, button_event_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(button4, 170, 80);
  lv_obj_set_pos(button4, 249, 100);
  lv_obj_t * label4 = lv_label_create(button4);
  lv_label_set_text(label4, "Goto_Screen1");
}


void button_event_cb(lv_event_t *event){

  lv_obj_t *btn = lv_event_get_target(event);
  lv_obj_t * label = lv_obj_get_child(btn, 0);
  const char * text = lv_label_get_text(label);
  printf ("Button_Touched: %s \n", text);
  printf("[APP] Minimum memory: %s bytes\n", String(esp_get_minimum_free_heap_size()));

  if(strcmp(text, "Goto_Screen2") == 0) {
    printf ("Goto Screen: %s \n", text);

    printf ("Load Screen2:\n");
    Create_Screen2();
    lv_scr_load(screen2);

    printf("[APP] Free memory: %s bytes\n", String(esp_get_free_heap_size()));
    //printf ("Clean Screen1:\n");
    //lv_obj_clean(screen1);
    printf ("Delete Screen1:\n");
    lv_obj_del(screen1);
    printf("[APP] Free memory: %s bytes\n", String(esp_get_free_heap_size()));

  }
  if(strcmp(text, "Goto_Screen3") == 0) {
    printf ("Goto Screen: %s \n", text);

    printf ("Load Screen3:\n");
    Create_Screen3();
    lv_scr_load(screen3);

    printf("[APP] Free memory: %s bytes\n", String(esp_get_free_heap_size()));
    printf ("Delete Screen2:\n");
    lv_obj_del(screen2);
    printf("[APP] Free memory: %s bytes\n", String(esp_get_free_heap_size()));

  }
  if(strcmp(text, "Goto_Screen1") == 0) {
    printf ("Goto Screen: %s \n", text);

    printf ("Create Screen1:\n");
    Create_Screen1();
    lv_scr_load(screen1);

    printf("[APP] Free memory: %s bytes\n", String(esp_get_free_heap_size()));
    printf ("Delete Screen3:\n");
    lv_obj_del(screen3);
    printf("[APP] Free memory: %s bytes\n", String(esp_get_free_heap_size()));

  }
  else{
    return;
  }

}