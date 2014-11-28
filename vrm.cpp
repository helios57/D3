/*
 * vrm.cpp
 *
 *  Created on: Jul 12, 2014
 *      Author: helios
 */

#include "vrm.h"
using namespace std;

void LogExit() {
	cerr << "VRmUsbCam Error: " << VRmUsbCamGetLastError() << "\nApplication exit" << endl;
	exit(-1);
}

void getNetxImage(VrmStatus* status) {
	// Lock next (raw) image for read access, convert it to the desired
	// format and unlock it again, so that grabbing can go on
	if (!VRmUsbCamLockNextImageEx2(status->device, status->port, &status->p_source_img, status->frames_dropped, 0)) {
		// in case of an error, check for trigger timeouts and trigger stalls.
		// both can be recovered, so continue. otherwise exit the app
		if (VRmUsbCamLastErrorWasTriggerTimeout())
			cerr << "trigger timeout" << endl;
		else if (VRmUsbCamLastErrorWasTriggerStall())
			cerr << "trigger stall" << endl;
		else
			LogExit();
	}
}

void initCamera(VrmStatus* status) {
	// at first, be sure to call VRmUsbCamCleanup() at exit, even in case
	// of an error
	atexit(VRmUsbCamCleanup);
	// read libversion (for informational purposes only)
	VRmDWORD libversion;
	VRMEXECANDCHECK(VRmUsbCamGetVersion(&libversion));
	cout << "" << endl << "(libversion" << libversion << ")" << endl << endl;
	// uncomment one of the following lines to enable logging features of VRmUsbCam (for customer support)
	//VRmUsbCamEnableLogging(); // save logfile to default location
	//VRmUsbCamEnableLoggingEx("mylogfile.log"); //save logfile to user defined location
	// check for connected devices
	VRmDWORD size = 0;
	VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListSize(&size));
	// open first usable device
	VRmDeviceKey* p_device_key = 0;
	for (VRmDWORD i = 0; i < size && !status->device; ++i) {
		VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListEntry(i, &p_device_key));
		if (!p_device_key->m_busy) {
			VRMEXECANDCHECK(VRmUsbCamOpenDevice(p_device_key, &status->device));
		}
		VRMEXECANDCHECK(VRmUsbCamFreeDeviceKey(&p_device_key));
	}
	// display error when no camera has been found
	if (!status->device) {
		cerr << "No suitable VRmagic device found!" << endl;
		exit(-1);
	}
	VRMEXECANDCHECK(VRmUsbCamGetSensorPortListEntry(status->device, 0, &status->port));
	VRmImageFormat source_format;
	// get the current source format
	VRMEXECANDCHECK(VRmUsbCamGetSourceFormatEx(status->device, status->port, &source_format));
	const char* source_color_format_str;
	VRMEXECANDCHECK(VRmUsbCamGetStringFromColorFormat(source_format.m_color_format, &source_color_format_str));
	cout << "Selected source format: " << source_format.m_width << "x" << source_format.m_height << " (" << source_color_format_str << ")" << endl;
	// At first we convert to 8-bit gray format. If this is a B/W camera, this 1st conversion at least
	// applies the VRmagic LUT to the image
	VRmDWORD number_of_target_formats, i;
	if (!VRmUsbCamGetTargetFormatListSizeEx2(status->device, status->port, &number_of_target_formats))
		LogExit();

	for (i = 0; i < number_of_target_formats; ++i) {
		if (!VRmUsbCamGetTargetFormatListEntryEx2(status->device, status->port, i, &status->target_format))
			LogExit();

		if (status->target_format.m_color_format == VRM_GRAY_8)
			break;
	}
}

void startGrabber(VrmStatus* status) {
	if (!VRmUsbCamNewImage(&status->p_source_img, status->target_format))
		LogExit();

	if (!VRmUsbCamResetFrameCounter(status->device))
		LogExit();

	// start grabber at first
	if (!VRmUsbCamStart(status->device))
		LogExit();
}

void unlockImage(VrmStatus* status) {
	if (!VRmUsbCamUnlockNextImage(status->device, &status->p_source_img))
		LogExit();
}

void getMostRecentImage(VrmStatus* status) {
	//get the most recent Image
	VRmUsbCamIsNextImageReadyEx(status->device, status->port, status->fp_ready);
	while (status->fp_ready) {
		if (status->p_source_img) {
			VRmUsbCamGetFrameCounter(status->p_source_img, &status->frame_counter);
			cout << "frame skipped clock():" << clock() << "  m_time_stamp: " << status->p_source_img->m_time_stamp << "  frame_counter: " << status->frame_counter << endl;
			VRmUsbCamUnlockNextImage(status->device, &status->p_source_img);
		}
		getNetxImage(status);
		VRmUsbCamIsNextImageReadyEx(status->device, status->port, status->fp_ready);
	}
}

void getFrameCounter(VrmStatus* status) {
	VRmUsbCamGetFrameCounter(status->p_source_img, &status->frame_counter);
}

void closeDevice(const VrmStatus* status) {
	VRMEXECANDCHECK(VRmUsbCamCloseDevice(status->device));
}
