/*
 * Copyright 2025 Arduino SA
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Camera driver.
 */
#include "Arduino.h"
#include "camera.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video-controls.h>

//#define DEBUG
//#define TRY_FIXED_CAMERA_BUFFER

/* Hack for 4 bit bitmap */
#if (DT_PROP(DT_NODELABEL(hm0360), data_bits) == 4) ||  \
      (DT_PROP(DT_NODELABEL(hm01b0), data_bits) == 4)
#define GRAY_IMAGE_FUDGE_SIZE (660*250)
#endif

FrameBuffer::FrameBuffer() : vbuf(NULL) {

}

uint32_t FrameBuffer::getBufferSize() {
    if (this->vbuf) {
        return this->vbuf->bytesused;
    }
}

uint8_t* FrameBuffer::getBuffer() {
    if (this->vbuf) {
        return this->vbuf->buffer;
    }
}

Camera::Camera() : vdev(NULL), byte_swap(false), yuv_to_gray(false) {
	for (size_t i = 0; i < ARRAY_SIZE(this->vbuf); i++) {
		this->vbuf[i] = NULL;
    }
}

#if defined(CONFIG_VIDEO)
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>

int camera_ext_clock_enable(void) {
	int ret;
	uint32_t rate;
	const struct device *cam_ext_clk_dev = DEVICE_DT_GET(DT_NODELABEL(pwmclock));

	if (!device_is_ready(cam_ext_clk_dev)) {
		return -ENODEV;
	}

	ret = clock_control_on(cam_ext_clk_dev, (clock_control_subsys_t)0);
	if (ret < 0) {
		return ret;
	}

	ret = clock_control_get_rate(cam_ext_clk_dev, (clock_control_subsys_t)0, &rate);
	if (ret < 0) {
		return ret;
	}

	return 0;
}
#else
#ERROR "CONFIG_VIDEO is not defined for this variant"
#endif

bool Camera::begin(uint32_t width, uint32_t height, uint32_t pixformat, bool byte_swap) {
  #if DT_HAS_CHOSEN(zephyr_camera)
  this->vdev = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera));
  #endif
  
	// start the clock
	int ret;

	if (!this->vdev) {
		return false;
	}
  
	camera_ext_clock_enable();
	delay(50);


  //if (!this->vdev || !device_is_ready(this->vdev)) {
  //    return false;
  //}
	if (!device_is_ready(this->vdev)) {
		// device probably has zephyr,deferred-init
		// On GIGA and Portenta H7 and probably others starts DCIM object
		if ((ret = device_init(this->vdev)) < 0) {
			printk("device_init camera(%p) failed:%d\n", this->vdev, ret);
			return false;
		}
	}
  
	// Now see if the actual camera is defined in choosen. And see if it is ready
#if DT_HAS_CHOSEN(zephyr_camera_sensor)
	const struct device *camera_sensor = DEVICE_DT_GET(DT_CHOSEN(zephyr_camera_sensor));
	if (!device_is_ready(camera_sensor)) {
		if ((ret = device_init(camera_sensor)) < 0) {
			printk("device_init camera sensor(%p) failed:%d\n", camera_sensor, ret);
			return false;
		}
	}
#endif


  switch (pixformat) {
      case CAMERA_RGB565:
          this->byte_swap = byte_swap;
          pixformat = VIDEO_PIX_FMT_RGB565;
          break;
      case CAMERA_GRAYSCALE:
          // There's no support for mono sensors.
          this->yuv_to_gray = false;
          pixformat = VIDEO_PIX_FMT_GREY;
          break;
      case CAMERA_GRAYSCALE_4:
          // There's no support for mono sensors.
          this->yuv_to_gray = false;
          pixformat = VIDEO_PIX_FMT_Y4;
          break;
      default:
          break;
  }

  // Get capabilities
  struct video_caps caps;
  if (video_get_caps(this->vdev, &caps)) {
      return false;
  }

  for (size_t i=0; caps.format_caps[i].pixelformat != NULL; i++) {
      const struct video_format_cap *fcap = &caps.format_caps[i];
		printk("INFO:   %c%c%c%c width [%u; %u; %u] height [%u; %u; %u]\n",
			(char)fcap->pixelformat, (char)(fcap->pixelformat >> 8),
			(char)(fcap->pixelformat >> 16), (char)(fcap->pixelformat >> 24),
			fcap->width_min, fcap->width_max, fcap->width_step, fcap->height_min,
			fcap->height_max, fcap->height_step);
      
      if (fcap->width_min == width &&
          fcap->height_min == height &&
          fcap->pixelformat == pixformat) {
          break;
      }
      
      if (caps.format_caps[i+1].pixelformat == NULL) {
		printk("ERRINFO:   %c%c%c%c width [%u; %u; %u] height [%u; %u; %u]\n",
			(char)fcap->pixelformat, (char)(fcap->pixelformat >> 8),
			(char)(fcap->pixelformat >> 16), (char)(fcap->pixelformat >> 24),
			fcap->width_min, fcap->width_max, fcap->width_step, fcap->height_min,
			fcap->height_max, fcap->height_step);
          Serial.println("The specified format is not supported");
          return false;
      }
  }

  // Set format.
  static struct video_format fmt = {
      .pixelformat = pixformat,
      .width = width,
      .height = height,
      .pitch = width * 2,
  };

	if (video_set_format(this->vdev, &fmt)) {
    Serial.println("Failed to set video format");
		return false;
	}
  
  size_t bsize;
  bsize = fmt.pitch * fmt.height;
#if DT_HAS_COMPAT_STATUS_OKAY(himax_hm0360) && (DT_PROP(DT_NODELABEL(hm0360), data_bits) == 8)
    setFrameRate(15);
#elif DT_HAS_COMPAT_STATUS_OKAY(himax_hm0360) && (DT_PROP(DT_NODELABEL(hm0360), data_bits) == 4) 
    setFrameRate(15);
    if (GRAY_IMAGE_FUDGE_SIZE > bsize) bsize = GRAY_IMAGE_FUDGE_SIZE;
#elif DT_HAS_COMPAT_STATUS_OKAY(himax_hm001b0) && (DT_PROP(DT_NODELABEL(hm01b0), data_bits) == 4) 
	/* BUGBUG:: HM01b0 4 bit mode. */
	if (GRAY_IMAGE_FUDGE_SIZE > bsize) bsize = GRAY_IMAGE_FUDGE_SIZE;
#else
   Serial.println("Data Bits selected not supported");
#endif
   printk("BSIZE: %d\n", bsize);
	// Allocate video buffers.
	for (size_t i = 0; i < ARRAY_SIZE(this->vbuf); i++) {
		this->vbuf[i] = video_buffer_aligned_alloc(bsize, CONFIG_VIDEO_BUFFER_POOL_ALIGN,  K_FOREVER);

		if (this->vbuf[i] == NULL) {
            Serial.println("Failed to allocate video buffers");
			return false;
		}
    
#ifdef TRY_FIXED_CAMERA_BUFFER
    if (i == 0) {
        // REAL hack change the buffer over to our fixed buffer...
        uint8_t* pb = (uint8_t*)malloc(bsize + 15);
        if (pb == nullptr) {
            Serial.println("Failed to allocate fixed video buffers");
        } else {
            // release the SDRAM
            //k_heap_free(&video_buffer_pool,this->vbuf[i]->buffer);
            #define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))
            this->vbuf[i]->buffer = (uint8_t*)ALIGN_UP((uint32_t)pb, 32);
        }
    }
#endif
    
		video_enqueue(this->vdev, this->vbuf[i]);
	}

	/* Start video capture */
	printk("Starting  capture\n");
	if (video_stream_start(this->vdev, VIDEO_BUF_TYPE_OUTPUT)) {
		printk("ERROR: Unable to start capture (interface)\n") ;
		return 0;
	}

  return true;
}

bool Camera::grabFrame(FrameBuffer &fb, uint32_t timeout) {
    uint8_t err;
    
    if (this->vdev == NULL) {
        return false;
    }

    if (video_dequeue(this->vdev, &fb.vbuf, K_MSEC(timeout))) {
        return false;
    }
    
    if (this->byte_swap) {
        uint16_t *pixels = (uint16_t *) fb.vbuf->buffer;
        for (size_t i=0; i<fb.vbuf->bytesused / 2; i++) {
            pixels[i] = __REVSH(pixels[i]);
        }
    }

    if (this->yuv_to_gray) {
        uint8_t *pixels = (uint8_t *) fb.vbuf->buffer;
        for (size_t i=0; i<fb.vbuf->bytesused / 2; i++) {
            pixels[i] = pixels[i*2];
        }
        fb.vbuf->bytesused /= 2;
    }

#if defined(DEBUG)
    printk("Bytes used: %d\n", fb.vbuf->bytesused);
    static uint8_t dump_frames = 2;
    if (dump_frames) {
        dump_frames--;
        for (uint8_t i = 0; i < 18; i++) {
            for (uint8_t j = 0; j < 80; j++) {
                printk("%x, ", *fb.vbuf->buffer++);
            }
            printk("\n");
            if ((i & 0x7) == 0x7) printk("\n");
        }
    }
#endif

    return true;
}

bool Camera::releaseFrame(FrameBuffer &fb) {
    if (this->vdev == NULL) {
        return false;
    }

    if (video_enqueue(this->vdev, fb.vbuf)) {
        return false;
	}

    return true;
}

bool Camera::setFrameRate(int framerate) {
	struct video_frmival frmival;
	frmival.numerator = framerate;
	frmival.denominator = 1;
	if (video_set_frmival(this->vdev, &frmival)){
		printk("ERROR: Unable to set up frame rate\n") ;
		return false;
	}

  return true;
}

bool Camera::setVerticalFlip(bool flip_enable) {
  struct video_control ctrl = {.id = VIDEO_CID_VFLIP, .val = flip_enable};
    return video_set_ctrl(this->vdev, &ctrl) == 0;
}

bool Camera::setHorizontalMirror(bool mirror_enable) {
    struct video_control ctrl = {.id = VIDEO_CID_HFLIP, .val = mirror_enable};
    return video_set_ctrl(this->vdev, &ctrl) == 0;
}
