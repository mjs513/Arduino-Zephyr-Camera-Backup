#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>
#include <zephyr/devicetree.h>
#include <zephyr/multi_heap/shared_multi_heap.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/video.h>
#include <zephyr/logging/log.h>

#include <elapsedMillis.h>

#include "Adafruit_GFX.h"
#include "Arduino_GigaDisplay_GFX.h"
#define GC9A01A_CYAN 0x07FF
#define GC9A01A_RED 0xf800
#define GC9A01A_BLUE 0x001F
#define GC9A01A_GREEN 0x07E0
#define GC9A01A_MAGENTA 0xF81F
#define GC9A01A_WHITE 0xffff
#define GC9A01A_BLACK 0x0000
#define GC9A01A_YELLOW 0xFFE0
#define ALIGN_PTR(p, a) ((p & (a - 1)) ? (((uintptr_t)p + a) & ~(uintptr_t)(a - 1)) : p)

#define CANVAS_ROTATION 0
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

inline uint16_t HTONS(uint16_t x) {
  return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
}
//#define HTONS(x) (((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00))

uint16_t gray_to_rgb565(uint8_t gray) {
    uint8_t r = gray >> 3;       // 5 bits for red
    uint8_t g = gray >> 2;       // 6 bits for green
    uint8_t b = gray >> 3;       // 5 bits for blue

    return (r << 11) | (g << 5) | b;
}

// Function to swap bytes in a 16-bit RGB565 value
uint16_t swapBytes(uint16_t color) {
    return (color >> 8) | (color << 8);
}

//#define CAMERA_HM0360
#define CAMERA_GRAYSCALE44
#define CAMERA_HM01B0

#ifdef CAMERA_HM01B0
#define CAMERA_WIDTH 326
#define CAMERA_HEIGHT 244
#else
#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240
#endif
#define SCALE 2

#include <camera.h>
 
Camera cam;

// The buffer used to capture the frame
FrameBuffer fb;

#ifdef ROTATE_CAMERA_IMAGE
uint16_t *rotate_buffer = nullptr;
#endif


// The buffer used to rotate and resize the frame
GigaDisplay_GFX display;

void blinkLED(uint32_t count = 0xFFFFFFFF) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (count--) {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(50);                        // wait for a second
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED off by making the voltage LOW
    delay(50);                        // wait for a second
  }
}

void fatal_error(const char *msg) {
  Serial.println(msg);
  pinMode(LED_BUILTIN, OUTPUT);
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial && millis() < 5000) {}
  Serial.begin(115200);
  
  Serial.println("Before camera start");
  Serial.flush();

  display.begin();
  elapsedMicros em;
  display.setRotation(1);
  display.fillScreen(GC9A01A_BLUE);
  Serial.println(em, DEC);

  //cam.debug(Serial);
#if defined(CAMERA_HM0360) && !defined(CAMERA_GRAYSCALE44)
  if (!cam.begin(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_GRAYSCALE, false)) {
#elif defined(CAMERA_GRAYSCALE44)
  if (!cam.begin(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_GRAYSCALE_4, false)) {
#else
  if (!cam.begin(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_RGB565, false)) {
#endif
    fatal_error("Camera begin failed");
  }
  cam.setVerticalFlip(true);
  cam.setHorizontalMirror(false);

  Serial.print("Camera Width: "); Serial.print(CAMERA_WIDTH);
  Serial.print(" Height: "); Serial.println(CAMERA_HEIGHT);
  Serial.print("Screen Width: "); Serial.print(display.width());
  Serial.print(" Height: "); Serial.println(display.height());
  #if defined(SCALE)
     Serial.print("Scale: "); Serial.println(SCALE);
  #endif
  Serial.println("end setup");
  Serial.flush();

}

uint32_t display_time_sum = 0;
uint8_t display_time_count = 0;

void loop() {

  // Grab frame and write to another framebuffer
  if (cam.grabFrame(fb, 100)) {
    //Serial.println("Camera frame received");
    if (Serial.available()) {
      while (Serial.read() != -1) {}
      //MemoryHexDump(Serial, fb.getBuffer(), 1024, true, "Start of Camera Buffer\n");
      Serial.println("*** Paused ***");
      while (Serial.read() == -1) {}
      while (Serial.read() != -1) {}
    }

#if defined(CAMERA_HM0360) && !defined(CAMERA_GRAYSCALE44)
    uint8_t *pixels = (uint8_t *)fb.getBuffer();
    //Serial.print("Nibbles: "); Serial.println(fb.getBufferSize());
		//uint8_t *pbIn = (uint8_t*)fb.getBuffer();
		//uint8_t *pixels = pbIn;
		//for (uint32_t i = 0; i < fb.getBufferSize(); i+=2) {
		//	uint8_t b =  ((pbIn[i] & 0x0f)) + ((pbIn[i+1] & 0xf0) << 4);
		//	*pixels++ = b;
		//}
#else
    uint16_t *pixels = (uint16_t *)fb.getBuffer();
#endif
    elapsedMicros emDisplay;
    //for (int i = 0; i < CAMERA_WIDTH*CAMERA_HEIGHT; i++) pixels[i] = HTONS(pixels[i]);
    #if defined(SCALE) && (SCALE > 1)
    // Quick and dirty scale.
   #if defined(CAMERA_HM0360) && !defined(CAMERA_GRAYSCALE44)
      display.drawGrayscaleBitmapScaled(CAMERA_WIDTH, CAMERA_HEIGHT, SCALE, pixels);
    #elif defined(CAMERA_GRAYSCALE44) //4bit hm01b0 and 4bit HM0360
      display.drawGrayscale4BitmapScaled(CAMERA_WIDTH, CAMERA_HEIGHT, SCALE, pixels);
    #else
      display.drawRGBBitmapScaled(CAMERA_WIDTH, CAMERA_HEIGHT, SCALE, pixels);
    #endif  //scaled
    #elif (defined(SCALE) && (SCALE == 1)) || !defined(SCALE)
    #if defined(CAMERA_HM0360)
      display.drawGrayscaleBitmap((display.width() - CAMERA_WIDTH) / 2, (display.height() - CAMERA_HEIGHT) / 2,  pixels, CAMERA_WIDTH, CAMERA_HEIGHT);
    #else
      display.drawRGBBitmap((display.width() - CAMERA_WIDTH) / 2, (display.height() - CAMERA_HEIGHT) / 2,  pixels, CAMERA_WIDTH, CAMERA_HEIGHT);
    #endif  //not scaled
    #endif

    cam.releaseFrame(fb);
    display_time_sum += emDisplay;
    display_time_count++;
    if (display_time_count == 128) {
      Serial.print("Avg display Time: ");
      Serial.print(display_time_sum / display_time_count);
      Serial.print(" fps:");
      Serial.println(128000000.0 / float(display_time_sum), 2);
      display_time_sum = 0;
      display_time_count = 0;
    }

  } else {
    digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);
  }
  delay(1);
}