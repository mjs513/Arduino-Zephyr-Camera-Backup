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
 */
#ifndef __CAMERA_H__
#define __CAMERA_H__
/** 
 * @enum CameraPixelFormat
 * @brief Camera pixel format enumeration.
 * 
 * The different formats use different numbers of bits per pixel:
 * - Grayscale (8-bit)
 * - RGB565 (16-bit)
 */
enum CameraPixelFormat {
    CAMERA_RGB565,       /**< RGB565 format (16-bit). */
    CAMERA_GRAYSCALE,    /**< Grayscale format (8-bit). */
    CAMERA_GRAYSCALE_4, /**< Grayscale format (4-bit). */
};

/**
 * @class FrameBuffer
 * @brief Frame buffer class for storing captured frames.
 */
class FrameBuffer {
    private:
        struct video_buffer *vbuf;

    public:
        /**
         * @brief Construct a new FrameBuffer object.
         */
        FrameBuffer();

        /**
         * @brief Get the buffer size in bytes.
         * @return uint32_t The buffer size in bytes.
         */
        uint32_t getBufferSize();

        /**
         * @brief Get a pointer to the frame buffer.
         * 
         * This can be used to read the pixels from the frame buffer.
         * For example, to print all pixels to the serial monitor as hex values.
         * 
         * @return uint8_t* Pointer to the frame buffer.
         */
        uint8_t* getBuffer();

    friend class Camera;
};

/**
 * @class Camera
 * @brief The main class for controlling a camera.
 */
class Camera {
    private:
        bool byte_swap;
        bool yuv_to_gray;
        const struct device *vdev;
        struct video_buffer *vbuf[CONFIG_VIDEO_BUFFER_POOL_NUM_MAX];

    public:
        /**
         * @brief Construct a new Camera object.
         */
        Camera();

        /**
         * @brief Initialize the camera.
         * 
         * @param width Frame width in pixels.
         * @param height Frame height in pixels.
         * @param pixformat Initial pixel format (default: CAMERA_RGB565).
         * @param byte_swap Enable byte swapping (default: false).
         * @return true if the camera is successfully initialized, otherwise false.
         */
        bool begin(uint32_t width, uint32_t height, uint32_t pixformat = CAMERA_RGB565, bool byte_swap = false);

        /**
         * @brief Capture a frame.
         * 
         * @param fb Reference to a FrameBuffer object to store the frame data.
         * @param timeout Time in milliseconds to wait for a frame (default: 5000).
         * @return true if the frame is successfully captured, otherwise false.
         */
        bool grabFrame(FrameBuffer &fb, uint32_t timeout = 5000);

        /**
         * @brief Release a frame buffer.
         * 
         * @param fb Reference to a FrameBuffer object to release.
         * @return true if the frame buffer is successfully released, otherwise false.
         */
        bool releaseFrame(FrameBuffer &fb);
        
        /**
         * @brief Sets frame rate for the HM0360
         * 
         * @param frame rate as an int.
         * @return true if the frame rate is changed successfully released, otherwise false.
         */
        bool setFrameRate(int framerate);

        /**
         * @brief Flip the camera image vertically.
         * 
         * @param flip_enable Set to true to enable vertical flip, false to disable.
         * @return true on success, false on failure.
         */
        bool setVerticalFlip(bool flip_enable);

        /**
         * @brief Mirror the camera image horizontally.
         * 
         * @param mirror_enable Set to true to enable horizontal mirror, false to disable.
         * @return true on success, false on failure.
         */
        bool setHorizontalMirror(bool mirror_enable);
};

#endif // __CAMERA_H__
