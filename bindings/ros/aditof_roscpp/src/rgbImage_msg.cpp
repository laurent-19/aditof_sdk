/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "rgbImage_msg.h"
#include <algorithm>
#include <cmath>
using namespace aditof;

// Fast Bayer (BGGR) to RGB8 conversion using simple nearest-neighbor, minimal branching
void bayerToRGB(const uint16_t* bayer, uint8_t* rgb, int w, int h) {
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int idx = y * w + x;
            int ridx = idx * 3;
            uint16_t c = bayer[idx];
            uint8_t r = 0, g = 0, b = 0;

            bool evenRow = (y & 1) == 0;
            bool evenCol = (x & 1) == 0;

            if (evenRow && evenCol) { // Blue
                b = c >> 8;
                g = ((x + 1 < w ? bayer[idx + 1] : c) + (y + 1 < h ? bayer[idx + w] : c)) >> 9;
                r = (x + 1 < w && y + 1 < h) ? (bayer[idx + w + 1] >> 8) : 0;
            } else if (evenRow && !evenCol) { // Green (on blue row)
                g = c >> 8;
                b = (x > 0 ? bayer[idx - 1] : c) >> 8;
                r = (y + 1 < h ? bayer[idx + w] : c) >> 8;
            } else if (!evenRow && evenCol) { // Green (on red row)
                g = c >> 8;
                r = (x > 0 ? bayer[idx - 1] : c) >> 8;
                b = (y > 0 ? bayer[idx - w] : c) >> 8;
            } else { // Red
                r = c >> 8;
                g = ((x > 0 ? bayer[idx - 1] : c) + (y > 0 ? bayer[idx - w] : c)) >> 9;
                b = (x > 0 && y > 0) ? (bayer[idx - w - 1] >> 8) : 0;
            }

            rgb[ridx] = r;
            rgb[ridx + 1] = g;
            rgb[ridx + 2] = b;
        }
    }
}

RgbImageMsg::RgbImageMsg() {}

RgbImageMsg::RgbImageMsg(const std::shared_ptr<aditof::Camera> &camera,
                         aditof::Frame *frame, std::string encoding,
                         ros::Time tStamp) {
    imgEncoding = encoding;
    FrameDataToMsg(camera, frame, tStamp);
}

void RgbImageMsg::FrameDataToMsg(const std::shared_ptr<Camera> &camera,
                                 aditof::Frame *frame, ros::Time tStamp) {
    FrameDetails fDetails;
    frame->getDetails(fDetails);

    uint16_t *frameData = getFrameData(frame, aditof::FrameDataType::RGB);
    if (!frameData) {
        return;
    }

    if (fDetails.rgbWidth <= 0 || fDetails.rgbHeight <= 0) {
        return;
    }
    
    rgbFrameEnhancement(frameData, fDetails.rgbWidth, fDetails.rgbHeight);

    setMetadataMembers(fDetails.rgbWidth, fDetails.rgbHeight, tStamp);
    setDataMembers(camera, frameData);
}

void RgbImageMsg::setMetadataMembers(int width, int height, ros::Time tStamp) {
    msg.header.stamp = tStamp;
    msg.header.frame_id = "aditof_rgb_img";
    msg.width = width;
    msg.height = height;
    msg.encoding = imgEncoding;
    msg.is_bigendian = false;

    int pixelByteCnt = sensor_msgs::image_encodings::bitDepth(imgEncoding) / 8 *
                       sensor_msgs::image_encodings::numChannels(imgEncoding);
    msg.step = width * pixelByteCnt;
    msg.data.resize(msg.step * height);
}

void RgbImageMsg::setDataMembers(const std::shared_ptr<Camera> &camera,
                                 uint16_t *frameData) {
    if (msg.encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) {
        // Convert Bayer 16-bit data to RGB8
        uint16_t minVal = *std::min_element(frameData, frameData + msg.width * msg.height);
        uint16_t maxVal = *std::max_element(frameData, frameData + msg.width * msg.height);
        
        // Apply basic enhancement to improve contrast
        uint16_t *enhancedData = new uint16_t[msg.width * msg.height];
        float scale = (maxVal > minVal) ? 65535.0f / (maxVal - minVal) : 1.0f;
        
        for (size_t i = 0; i < static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height); i++) {
            int enhanced = (frameData[i] - minVal) * scale;
            enhancedData[i] = std::min(65535, std::max(0, enhanced));
        }
        
        // Convert Bayer to RGB
        bayerToRGB(enhancedData, msg.data.data(), msg.width, msg.height);
        delete[] enhancedData;
    }
    else if (msg.encoding.compare(sensor_msgs::image_encodings::BAYER_BGGR16) == 0) {
        std::memcpy(msg.data.data(), frameData, msg.width * msg.height * sizeof(uint16_t));
    }
    else {
        ROS_ERROR("Image encoding invalid or not available");
    }
}

void RgbImageMsg::publishMsg(const ros::Publisher &pub) { pub.publish(msg); }
