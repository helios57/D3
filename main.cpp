// ==============================================================================================
// This file is part of the VRmagic VRmUsbCam2 C API Demo Application
// ==============================================================================================
// Main Function
// ----------------------------------------------------------------------------------------------

#include "main.h"
#include "MavlinkBridge.h"

using namespace std;
using namespace d3;

void checkAndAdjustGain(Mat& mat, VrmStatus* vrmStatus) {
	double sum = 0;
	int count = 0;
	for (int i = 0; i < mat.rows; i += 10) {
		for (int j = 0; j < mat.cols; j += 10) {
			sum += mat.at<uchar>(i, j);
			count++;
		}
	}
	double meanValueDouble = sum / count;
	if (meanValueDouble < 50.0f && vrmStatus->gain < 200) {
		vrmStatus->gain++;
		setGain(vrmStatus);
	}
	if (meanValueDouble > 100.0f && vrmStatus->gain > 12) {
		vrmStatus->gain--;
		setGain(vrmStatus);
	}
}

int main(int argc, char** argv) {
	time_t starttime;
	time(&starttime);
	char datetime[80];
	struct tm tstruct = *localtime(&starttime);
	strftime(datetime, sizeof(datetime), "%Y%m%d%H%M%S", &tstruct);
	string dateTimeString = datetime;

	ofstream logfile;
	logfile.open(dateTimeString + ".log", ios_base::app);
	logfile << "started at " << dateTimeString << endl;
	logfile << "sleeping for 20 sec to let the usb initialize" << endl;

#ifndef LINUX
	sleep(20);
#endif

	MavlinkBridge mavlink;
	mavlink.start();

	VrmStatus* vrmStatus = new VrmStatus();
	vector<Rect> objs;

	initCamera(vrmStatus);
	startGrabber(vrmStatus);
	logfile << "Reading from camera..." << endl;

#ifdef LINUX
	cvNamedWindow("result");
	startWindowThread();
#endif

	CascadeClassifier* face_cascade = new CascadeClassifier();
	face_cascade->load("cascade.xml");

	getNetxImage(vrmStatus);

#ifdef LINUX
	int width = vrmStatus->p_source_img->m_image_format.m_width;
	int height = vrmStatus->p_source_img->m_image_format.m_height;
	Scalar color = CV_RGB(0, 0, 255);
	VideoWriter videoOut(dateTimeString + ".avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(width, height), false);
	if (!videoOut.isOpened()) {
		logfile << "VideoWriter could not be opened" << endl;
	}
#endif
	logfile << "frameCounter;hwTime;detectTimeUs;midx;midy;x;y;width;height;roll;pitch;correctureX;correctureY,wrongDetection,detectedSize,gain" << endl;

	int lastMidX = 0;
	int lastMidY = 0;
	do {
		getNetxImage(vrmStatus);
		getMostRecentImage(vrmStatus);

		// Note: p_source_img may be null in case a recoverable error
		// (like a trigger timeout) occured.
		if (vrmStatus->p_source_img) {
			getFrameCounter(vrmStatus);
			//logfile << "frame grabbed clock():" << clock() << "  m_time_stamp: " << vrmStatus->p_source_img->m_time_stamp << "  frame_counter: " << vrmStatus->frame_counter << endl;
			// see, if we had to drop some frames due to data transfer stalls. if so,
			// output a message
			//if (vrmStatus->frames_dropped)
			//	logfile << "- " << vrmStatus->frames_dropped << " frames dropped-" << endl;

			struct timespec timeOfCapture;
			clock_gettime(CLOCK_MONOTONIC, &timeOfCapture);
			mavlink_d3_pitchroll_t pitchRoll = mavlink.getPitchRoll();
			float roll = pitchRoll.roll;
			float pitch = pitchRoll.pitch;

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
			int detectedSize = objs.size();
			//logfile << "detectTime" << detectTime << "ms detectSize: " << detecteSize << endl;
			if (detectedSize > 0) {
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

				bool wrongDetection = (abs(lastMidX - midX) + abs(lastMidY - midY)) > 200;
				lastMidX = midX;
				lastMidY = midY;

				float corrX = midX * 0.000660996 + midX * midX * 3.0e-8 + roll;
				float corrY = midY * 0.000676192 + midY * midY * 2.5e-8 + pitch;
				uint64_t hwTime = timeOfCapture.tv_sec * 1000000 + timeOfCapture.tv_nsec / 1000;
				logfile << vrmStatus->frame_counter << ";" << hwTime << ";" << detectTime << ";" << midX << ";" << midY << ";" << max->x << ";" << max->y;
				logfile << ";" << max->width << ";" << max->height;
				logfile << ";" << roll << ";" << pitch << ";" << corrX << ";" << corrY;
				logfile << ";" << wrongDetection << ";" << detectedSize << ";" << vrmStatus->gain << endl;
				if (!wrongDetection) {
					mavlink.send_target(hwTime, corrX, corrY);
				}
#ifdef LINUX
				rectangle(mat, cvPoint(cvRound(max->x), cvRound(max->y)), cvPoint(cvRound((max->x + max->width - 1)), cvRound((max->y + max->height - 1))), color, 3, 8, 0);
#endif
			}
			else {
				//Nothing found, check gain
				getGain(vrmStatus);
				checkAndAdjustGain(mat, vrmStatus);
			}
#ifdef LINUX
			putText(mat, to_string(vrmStatus->frame_counter), Point(0, 30), FONT_HERSHEY_SCRIPT_SIMPLEX, 1, color);
			videoOut.write(mat);
			imshow("result", mat);
			waitKey(1);
#endif
			unlockImage(vrmStatus);
		}
	}
	while (1);
#ifdef LINUX
	cvDestroyWindow("result");
#endif
	closeDevice(vrmStatus);
	logfile << "exit." << endl;

	mavlink.stop();
	return 0;
}
