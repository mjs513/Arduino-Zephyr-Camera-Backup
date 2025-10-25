#define CAMERA_USES_MONO_PALETTE
#define use_4bit

#include "camera.h"
Camera cam;
FrameBuffer fb;

#define CAM_WIDTH 326
#define CAM_HEIGHT 244

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

void setup(void) {
  while(!Serial && millis() < 5000);
  Serial.begin(115200);    
  delay(5000);

#if defined(CAMERA_USES_MONO_PALETTE)
#if defined(use_4bit)
  if (!cam.begin(CAM_WIDTH, CAM_HEIGHT, CAMERA_GRAYSCALE_4, false)) {
#else
  if (!cam.begin(CAM_WIDTH, CAM_HEIGHT, CAMERA_GRAYSCALE, false)) {
#endif
#else
  if (!cam.begin(CAM_WIDTH, CAM_HEIGHT, CAMERA_RGB565, false)) {
#endif
    fatal_error("Camera begin failed");
  }
  Serial.println("Camera Started....");
  cam.setVerticalFlip(true);
  cam.setHorizontalMirror(false);
  Serial.println("Camera Started....");
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#define F(x) x

inline uint16_t HTONS(uint16_t x) {
#if defined(DVP_CAMERA_OV2640) || defined(DVP_CAMERA_OV5640)
    return x;
#else  //byte reverse
    return ((x >> 8) & 0x00FF) | ((x << 8) & 0xFF00);
#endif
}


void send_image(uint8_t *frame_buffer, uint16_t frame_width, uint16_t frame_height)
{

    Serial.write(0xFF);
    Serial.write(0xAA);

    // BUGBUG:: maybe combine with the save to SD card code
    unsigned char bmpFileHeader[14] = { 'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0 };
    unsigned char bmpInfoHeader[40] = { 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 24, 0 };

    int rowSize = 4 * ((3 * frame_width + 3) / 4);  // how many bytes in the row (used to create padding)
    int fileSize = 54 + frame_height * rowSize;     // headers (54 bytes) + pixel data

    bmpFileHeader[2] = (unsigned char)(fileSize);
    bmpFileHeader[3] = (unsigned char)(fileSize >> 8);
    bmpFileHeader[4] = (unsigned char)(fileSize >> 16);
    bmpFileHeader[5] = (unsigned char)(fileSize >> 24);

    bmpInfoHeader[4] = (unsigned char)(frame_width);
    bmpInfoHeader[5] = (unsigned char)(frame_width >> 8);
    bmpInfoHeader[6] = (unsigned char)(frame_width >> 16);
    bmpInfoHeader[7] = (unsigned char)(frame_width >> 24);
    bmpInfoHeader[8] = (unsigned char)(frame_height);
    bmpInfoHeader[9] = (unsigned char)(frame_height >> 8);
    bmpInfoHeader[10] = (unsigned char)(frame_height >> 16);
    bmpInfoHeader[11] = (unsigned char)(frame_height >> 24);

    Serial.write(bmpFileHeader, sizeof(bmpFileHeader));  // write file header
    Serial.write(bmpInfoHeader, sizeof(bmpInfoHeader));  // " info header

    unsigned char bmpPad[rowSize - 3 * frame_width];
    for (int i = 0; i < (int)(sizeof(bmpPad)); i++) {  // fill with 0s
        bmpPad[i] = 0;
    }

#ifdef CAMERA_USES_MONO_PALETTE
    uint8_t *frameBuffer = (uint8_t*)frame_buffer;
    uint8_t *pfb = frameBuffer;
#if defined(use_4bit)
		for (uint32_t i = 0; i < fb.getBufferSize(); i+=2) {
			uint8_t b =  ((frameBuffer[i] & 0xf) << 0) + ((frameBuffer[i+1] & 0xf) << 4);
			*pfb++ = b;
		}
#endif
    uint8_t img[3];
    for (int y = frame_height - 1; y >= 0; y--) {  // iterate image array
        pfb = &frameBuffer[y * frame_width];
        for (int x = 0; x < frame_width; x++) {
            // r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            img[2] = *pfb;  // r
            img[1] = *pfb;  // g
            img[0] = *pfb;  // b
            Serial.write(img, 3);
            delayMicroseconds(1);
            pfb++;
        }
        Serial.write(bmpPad, (4 - (frame_width * 3) % 4) % 4);  // and padding as needed
    }
#else
    uint16_t *frameBuffer = (uint16_t*)frame_buffer;
    uint16_t *pfb = (uint16_t *)frameBuffer;
    uint8_t img[3];
    for (int y = frame_height - 1; y >= 0; y--) {  // iterate image array
        pfb = &frameBuffer[y * frame_width];
        for (int x = 0; x < frame_width; x++) {
            //r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3
            uint16_t pixel = (*pfb++);
            img[2] = (pixel >> 8) & 0xf8;  // r
            img[1] = (pixel >> 3) & 0xfc;  // g
            img[0] = (pixel << 3);         // b
            Serial.write(img, 3);
            delayMicroseconds(1);

        }
        Serial.write(bmpPad, (4 - (frame_width * 3) % 4) % 4);  // and padding as needed
    }
#endif
    Serial.write(0xBB);
    Serial.write(0xCC);

    Serial.print(("ACK CMD CAM Capture Done. END\n"));
    //camera.setHmirror(0);

    delay(50);
}


void maybe_send_image(uint8_t *frameBuffer, uint16_t frame_width, uint16_t frame_height) {
	int ch;
	while ((ch = Serial.read()) != -1) {
		switch(ch) {
      printk("Recvd: %x\n", ch);
			#ifdef LATER
        case 0x10:
            {
                Serial.println(F("ACK CMD CAM start jpg single shoot. END"));
                send_jpeg();
                Serial.println(F("READY. END"));
            }
            break;
      #endif
        case 0x30:
            {
              printk("Send BMP: %u %u\n", frame_width, frame_height);
              Serial.println(F("ACK CMD CAM start single shoot ... "));
              send_image(frameBuffer, frame_width, frame_height);
              Serial.println(F("READY. END"));
            }
            break;
        default:
          break;
      }
	}
}

void loop() {
  if (cam.grabFrame(fb)) {
    maybe_send_image(fb.getBuffer(), CAM_WIDTH, CAM_HEIGHT);
    cam.releaseFrame(fb);
  }
}
