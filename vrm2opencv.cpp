#include "vrm2opencv.h"

int toCvType(VRmColorFormat f_color_format) {
	int cv_type(-1);
	switch (f_color_format) {
	case VRM_ARGB_4X8:
		cv_type = CV_8UC4;
		break;
	case VRM_BGR_3X8:
		cv_type = CV_8UC3;
		break;
	case VRM_BAYER_BGGR_8:
	case VRM_BAYER_GBRG_8:
	case VRM_BAYER_GRBG_8:
	case VRM_BAYER_RGGB_8:
	case VRM_GRAY_8:
		cv_type = CV_8UC1;
		break;
	case VRM_BAYER_BGGR_16:
	case VRM_BAYER_GBRG_16:
	case VRM_BAYER_GRBG_16:
	case VRM_BAYER_RGGB_16:
	case VRM_GRAY_16:
		cv_type = CV_16UC1;
		break;
	default:
		break;
	}
	return cv_type;
}

Mat getMat(const VRmImage *p_source_img) {
	_VRmImageFormat format = p_source_img->m_image_format;
	VRmColorFormat color_format = format.m_color_format;
	CvSize cvSize = CvSize(format.m_width, format.m_height);
	int cvColor = toCvType(color_format);
	Mat openCVImg(cvSize, cvColor, (void*) p_source_img->mp_buffer, p_source_img->m_pitch);
	return openCVImg;
}

//C++ wrapper Version
//VRmColorFormat color_format = p_source_image->get_ImageFormat().get_ColorFormat();
//cv::Mat openCVImg(cvSize(p_source_image->get_ImageFormat().get_Size().m_width,
//p_source_image->get_ImageFormat().get_Size().m_height),toCvType(color_format),
//(void*)p_source_image->get_Buffer(),p_source_image->get_Pitch());
