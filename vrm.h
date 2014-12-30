/*
 * vrm.h
 *
 *  Created on: Jul 12, 2014
 *      Author: helios
 */

#ifndef VRM_H_
#define VRM_H_
#include <vrmusbcam2.h>
#include <iostream>
void LogExit();

// small helper macro to check function for success and call LogExit when function fails
#define VRMEXECANDCHECK(function)\
{\
	if (VRM_SUCCESS!=function)\
	LogExit();\
}

struct VrmStatus {
		VRmUsbCamDevice device;
		VRmDWORD port;
		VRmImageFormat target_format;
		VRmImage* p_source_img;
		VRmDWORD* frames_dropped, frame_counter;
		VRmBOOL* fp_ready;
		VRmPropId gainPropertyId;
		int gain;
};

void getNetxImage(VrmStatus* status);
void initCamera(VrmStatus* status);
void startGrabber(VrmStatus* status);
void getMostRecentImage(VrmStatus* status);
void unlockImage(VrmStatus* status);
void getFrameCounter(VrmStatus* vrmStatus);
void closeDevice(const VrmStatus* vrmStatus);

#endif /* VRM_H_ */
