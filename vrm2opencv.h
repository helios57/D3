#ifndef VRM2OPENCV_H
#define VRM2OPENCV_H

#include <vrmusbcam2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>

using namespace cv;

int toCvType(VRmColorFormat f_color_format);
Mat getMat(const VRmImage *p_source_img);

#endif //VRM2OPENCV_H
