/*******************************************************************************
* Copyright 2018 ROBIT CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/**********************  Maintainer: Hae-Bum Jung  ****************************/

#ifndef DARKNETDETECTOR_H
#define DARKNETDETECTOR_H

extern "C" {
#include "/home/robit/catkin_ws/src/yolov3_ver2/darkent/src/utils.h"
#include "/home/robit/catkin_ws/src/yolov3_ver2/darkent/src/parser.h"
}

void detector_init(char *cfgfile, char *weightfile);

float* test_detector_file(char *filename, float thresh, float hier_thresh, int* num_output_class);

// *data is a image date buffer in which image data was stored as [bgrbgrbgr...bgr] by rows
float* test_detector_uchar(unsigned char *data, int w, int h, int c, float thresh, float hier_thresh, int* num_output_class);

void detector_uninit();

double what_is_the_time_now();

#endif // DARKNETDETECTOR_H
