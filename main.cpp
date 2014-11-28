// ==============================================================================================
// This file is part of the VRmagic VRmUsbCam2 C API Demo Application
// ==============================================================================================
// Main Function
// ----------------------------------------------------------------------------------------------

#include "main.h"
#include "MavlinkBridge.h"

using namespace std;
using namespace d3;

int main(int argc, char** argv) {
	MavlinkBridge mavlink;
	mavlink.start();

	VrmStatus* vrmStatus = new VrmStatus();
	vector<Rect> objs;

	initCamera(vrmStatus);
	startGrabber(vrmStatus);
	cout << "Reading from camera..." << endl;

#ifdef LINUX
	cvNamedWindow("result");
	startWindowThread();
	Scalar color = CV_RGB(0, 0, 255);
#endif

	CascadeClassifier* face_cascade = new CascadeClassifier();
	face_cascade->load("cascade.xml");

	cout << "detectTime;midx;midy;x;y;width;height;roll;pitch;correctureX;correctureY" << endl;
	do {
		getNetxImage(vrmStatus);
		getMostRecentImage(vrmStatus);

		// Note: p_source_img may be null in case a recoverable error
		// (like a trigger timeout) occured.
		if (vrmStatus->p_source_img) {
			getFrameCounter(vrmStatus);
			//cout << "frame grabbed clock():" << clock() << "  m_time_stamp: " << vrmStatus->p_source_img->m_time_stamp << "  frame_counter: " << vrmStatus->frame_counter << endl;
			// see, if we had to drop some frames due to data transfer stalls. if so,
			// output a message
			//if (vrmStatus->frames_dropped)
			//	cout << "- " << vrmStatus->frames_dropped << " frames dropped-" << endl;

			//Get time of capture
			struct timespec monotime;
			clock_gettime(CLOCK_MONOTONIC, &monotime);

			Mat mat = getMat(vrmStatus->p_source_img);
			objs.clear();
			timespec detectTimer = timer_start();
			//int detecteSize = 0;
			for (int i = 512; i > 32; i /= 2) {
				//detecteSize = i;
				face_cascade->detectMultiScale(mat, objs, 1.1, 2, 0, Size(i, i));
				if (!objs.empty()) {
					break;
				}
			}
			long int detectTime = timer_end(detectTimer);

			int height = vrmStatus->p_source_img->m_image_format.m_height;
			int width = vrmStatus->p_source_img->m_image_format.m_width;
			//cout << "detectTime" << detectTime << "ms detectSize: " << detecteSize << endl;
			if (objs.size() > 0) {
				int maxX = 0;
				int maxY = 0;
				vector<Rect>::const_iterator max;
				for (vector<Rect>::const_iterator r = objs.begin(); r != objs.end(); r++) {
					if (r->width >= maxX && r->height >= maxY) {
						maxX = r->width;
						maxY = r->height;
						max = r;
					}
				}
#ifdef LINUX
				for (vector<Rect>::const_iterator r = objs.begin(); r != objs.end(); r++) {
					if (r != max) {
						rectangle(mat, cvPoint(cvRound(r->x), cvRound(r->y)), cvPoint(cvRound((r->x + r->width - 1)), cvRound((r->y + r->height - 1))), CV_RGB(0, 0, 128), 1, 8, 0);
					}
				}
#endif
				int midX = max->x + max->width / 2 - width / 2;
				int midY = max->y + max->height / 2 - height / 2;
				mavlink_d3_pitchroll_t pitchRoll = mavlink.getPitchRoll();
				float corrX = midX * 0.0005f + pitchRoll.roll;
				float corrY = midY * 0.0005f + pitchRoll.pitch;
				cout << detectTime << ";" << midX << ";" << midY << ";" << max->x << ";" << max->y;
				cout << ";" << max->width << ";" << max->height;
				cout << ";" << pitchRoll.roll << ";" << pitchRoll.pitch << ";" << corrX << ";" << corrY;
				cout << endl;
				mavlink.send_target(monotime.tv_nsec/1000000,corrX * 10, corrY * 10);
#ifdef LINUX
				rectangle(mat, cvPoint(cvRound(max->x), cvRound(max->y)), cvPoint(cvRound((max->x + max->width - 1)), cvRound((max->y + max->height - 1))), color, 3, 8, 0);
#endif
			}
#ifdef LINUX
			imshow("result", mat);
			waitKey(1);
#endif
			unlockImage(vrmStatus);
			//mavlink_attitude_t at = mavlink.getAttitude();
			//printf("attitude: time=%f roll=%f pitch=%f yaw=%f rollspeed=%f pitchspeed=%f yawspeed=%f\n", //
			//(float) at.time_boot_ms, at.roll, at.pitch, at.yaw, at.rollspeed, at.pitchspeed, at.yawspeed);	//
		}
	} while (1);
#ifdef LINUX
	cvDestroyWindow("result");
#endif
	closeDevice(vrmStatus);
	cout << "exit." << endl;

	mavlink.stop();
	return 0;
}
