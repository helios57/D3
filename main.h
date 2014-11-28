// ==============================================================================================
// This file is part of the VRmagic USB2.0 Camera Demo Application
// ==============================================================================================
#ifndef VRMUSBCAMDEMO_H
#define VRMUSBCAMDEMO_H

// include the api of the library
#include <vrmusbcam2.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cstdlib>
#include <cstring>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "vrm2opencv.h"
#include "timerutil.h"
#include "vrm.h"

#ifdef LINUX
#include "opencv2/highgui.hpp"
#include "opencv2/highgui/highgui_c.h"
#endif

#endif //VRMUSBCAMDEMO_H
