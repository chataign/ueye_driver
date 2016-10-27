#include <stdexcept>
#include <vector>
#include <ctime>

#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>

#include "ueye_camera.h"

namespace ueye
{
#define UEYE_TRY( FUNC, ... ) { INT err = (FUNC)(__VA_ARGS__); if ( err != IS_SUCCESS ) { \
	throw std::runtime_error( std::string(#FUNC)+" failed, error="+std::to_string(err) ); } }

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct ColorMode
{
	INT ueye_mode;
	uint16_t bits_per_pixel;
};

static const std::map< std::string, ColorMode > color_modes = {
	{ sensor_msgs::image_encodings::MONO8, { IS_CM_MONO8,         8 } },
	{ sensor_msgs::image_encodings::BGR8,  { IS_CM_BGR8_PACKED,  24 } },
	{ sensor_msgs::image_encodings::RGB8,  { IS_CM_RGB8_PACKED,  24 } }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Camera::Camera( const UEYE_CAMERA_INFO& device_info, const DeviceSettings& device_settings )
	: device_info_( device_info )
	, camera_info_( boost::make_shared<sensor_msgs::CameraInfo>() )
{
	ROS_INFO("opening camera id=%d serial=%s", device_info_.dwCameraID, device_info_.SerNo );
	UEYE_TRY( is_InitCamera, (HIDS*) &device_info_.dwCameraID, NULL );

    auto color_mode = color_modes.at(device_settings.color_mode).ueye_mode;
    
	UEYE_TRY( is_SetDisplayMode, device_info_.dwCameraID, IS_SET_DM_DIB );
	UEYE_TRY( is_SetColorMode, device_info_.dwCameraID, color_mode );
	ROS_INFO("set color mode=%s (%d)", device_settings.color_mode.c_str(), color_mode );

	ROS_INFO("AOI: tl=(%d,%d) width=%d height=%d", 
	    device_settings.aoi_rect.s32X, 
	    device_settings.aoi_rect.s32Y, 
	    device_settings.aoi_rect.s32Width, 
	    device_settings.aoi_rect.s32Height );

	UEYE_TRY( is_AOI, device_info_.dwCameraID, IS_AOI_IMAGE_SET_AOI, 
	    (void*)&device_settings.aoi_rect, sizeof(device_settings.aoi_rect) );

    // (from UEye documentation)
    // In general, the pixel clock is set once when opening the camera and will not be changed. 
    // Note that, if you change the pixel clock, the setting ranges for frame rate and exposure 
    // time also changes. If you change a parameter, the following order is recommended:
    // 1. Change pixel clock.
    // 2. Query frame rate range and, if applicable, set new value.
    // 3. Query exposure time range and, if applicable, set new value.

    UINT pixel_clock_range[3]; // [ min, max, increment ]
    ZeroMemory( pixel_clock_range, sizeof(pixel_clock_range) );
     
    UEYE_TRY( is_PixelClock, device_info_.dwCameraID, IS_PIXELCLOCK_CMD_GET_RANGE, 
        (void*)pixel_clock_range, sizeof(pixel_clock_range) );

	try { UEYE_TRY(is_PixelClock, device_info_.dwCameraID, IS_PIXELCLOCK_CMD_SET, 
		(void*)&device_settings.pixel_clock, sizeof(device_settings.pixel_clock) ) }
	catch( const std::exception& e ) { ROS_WARN( "failed to set pixel clock: %s", e.what() ); }

    UINT pixel_clock_actual; 

    UEYE_TRY( is_PixelClock, device_info_.dwCameraID, IS_PIXELCLOCK_CMD_GET, 
        (void*)&pixel_clock_actual, sizeof(pixel_clock_actual) );

	ROS_INFO("pixel clock: requested=%dMHz actual=%d (min=%u max=%u)", 
	    device_settings.pixel_clock, pixel_clock_actual, pixel_clock_range[0], pixel_clock_range[1] );

	//Set the frame rate. First, see what's the maximum (it varies with pixelclock)
	double min_time, max_time, interval;
	UEYE_TRY(is_GetFrameTimeRange, device_info_.dwCameraID, &min_time, &max_time, &interval );
	ROS_INFO("Maximum possible frame rate: %.1f", 1/min_time);

	double actual_rate=0;
	UEYE_TRY(is_SetFrameRate, device_info_.dwCameraID, device_settings.frame_rate, &actual_rate );
	ROS_INFO("frame rate: requested=%.1fHz actual=%.1fHz", device_settings.frame_rate, actual_rate );

	frame_ = std::make_shared<CameraFrame>( *this, device_settings );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Camera::~Camera()
{
	frame_.reset();	// must call FreeImageMem() before ExitCamera()
	UEYE_TRY( is_ExitCamera, device_info_.dwCameraID );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_master_gain( uint8_t master_gain )
{
	if ( is_SetGainBoost( device_info_.dwCameraID, IS_GET_SUPPORTED_GAINBOOST ) != IS_SET_GAINBOOST_ON )
		{ ROS_WARN("hardware gain not supported"); return false; }

	if ( master_gain == 0 )	// disable gain
	{
		UEYE_TRY( is_SetGainBoost, device_info_.dwCameraID, IS_SET_GAINBOOST_OFF );
	}
	else
	{
		ROS_INFO( "Master gain set to %d, gain boost activated", master_gain );
		UEYE_TRY( is_SetGainBoost, device_info_.dwCameraID, IS_SET_GAINBOOST_ON );
		UEYE_TRY( is_SetHardwareGain, device_info_.dwCameraID, master_gain,
			IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER );
	}

	return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Camera::start_capture( bool external_trigger )
{
	if ( external_trigger )
	{
		UEYE_TRY( is_EnableEvent, device_info_.dwCameraID, IS_SET_EVENT_FRAME );
		UEYE_TRY( is_SetExternalTrigger, device_info_.dwCameraID, IS_SET_TRIGGER_HI_LO );
		UEYE_TRY( is_CaptureVideo, device_info_.dwCameraID, IS_DONT_WAIT );
	}
	else // freerun mode
	{
		UEYE_TRY( is_EnableEvent, device_info_.dwCameraID, IS_SET_EVENT_FRAME );
		UEYE_TRY( is_CaptureVideo, device_info_.dwCameraID, IS_WAIT );
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Camera::start_capture( const CaptureSettings& settings )
{
    set_exposure( settings.exposure );
    set_master_gain( settings.master_gain );
    set_blacklevel( settings.blacklevel );
    set_gamma( settings.gamma );
    if ( settings.hardware_gamma ) set_hardware_gamma();
    
    start_capture( settings.external_trigger );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const CameraFrame* Camera::get_frame( int timeout_ms )
{
	switch ( is_WaitEvent( device_info_.dwCameraID, IS_SET_EVENT_FRAME, timeout_ms ) )
	{
	case IS_SUCCESS:
		frame_->update_timestamp();
		return frame_.get();
	case IS_TIMED_OUT:
		return NULL;
	default:
		UEYE_TRY( is_DisableEvent, device_info_.dwCameraID, IS_SET_EVENT_FRAME );
		throw std::runtime_error("failed to get frame");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_exposure( double exposure )
{
	double max_exposure;
	//Set the exposure time. First see what's the maximum (it varies with framerate)
	UEYE_TRY(is_Exposure, device_info_.dwCameraID, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX,
			(void*)&max_exposure, sizeof(max_exposure));
	ROS_INFO("Maximum exposure (given by frame rate) is %.1f ", max_exposure);

	//Set the exposure time it to this maximum.
	UEYE_TRY(is_Exposure,device_info_.dwCameraID, IS_EXPOSURE_CMD_SET_EXPOSURE,
			(void*)&exposure, sizeof(exposure));
	ROS_INFO("Exposure set to %.1f", exposure);

	return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_blacklevel(int blacklevel)
{
	//Set the new offset
	UEYE_TRY(is_Blacklevel, device_info_.dwCameraID, IS_BLACKLEVEL_CMD_SET_OFFSET,
			(void*)&blacklevel, sizeof(blacklevel));
	ROS_INFO("Black level offset set to %d", blacklevel);

	return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_gamma(INT gamma)
{
	//Set software gamma. Hardware gamma correction should suffice
	UEYE_TRY(is_Gamma, device_info_.dwCameraID, IS_GAMMA_CMD_SET, (void*) &gamma, sizeof(gamma));
	INT gamma_get;
	UEYE_TRY(is_Gamma, device_info_.dwCameraID, IS_GAMMA_CMD_GET, (void*) &gamma_get, sizeof(gamma_get));
	ROS_INFO("Software gamma set to %d", gamma_get);

	return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_hardware_gamma()
{
	//	Enable hardware gamma correction
	UEYE_TRY(is_SetHardwareGamma, device_info_.dwCameraID, IS_SET_HW_GAMMA_ON );
	ROS_INFO("Hardware gamma correction set");

	return true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

CameraFrame::CameraFrame( Camera& camera, const DeviceSettings& device_settings )
	: camera_id_( camera.device_info_.dwCameraID )
	, buffer_id_(0)
{
	uint16_t bits_per_pixel = color_modes.at(device_settings.color_mode).bits_per_pixel;

    image_ = boost::make_shared<sensor_msgs::Image>();
    image_->header.frame_id = device_settings.frame_id;
	image_->is_bigendian = false;
	image_->encoding = device_settings.color_mode;
	image_->width  = device_settings.aoi_rect.s32Width;
	image_->height = device_settings.aoi_rect.s32Height;
	image_->step = image_->width * bits_per_pixel / 8;
	image_->data.resize( image_->height * image_->step );

	ROS_INFO("allocating %dx%d image depth=%d",
		image_->width, image_->height, bits_per_pixel );

	// set the camera buffer to be that of the internal
	// sensor_msgs::Image object to avoid unecessary data copies

	UEYE_TRY( is_SetAllocatedImageMem, camera_id_,
		image_->width, image_->height, bits_per_pixel,
		(char*) &image_->data[0], &buffer_id_ );

	UEYE_TRY( is_SetImageMem, camera_id_, (char*) &image_->data[0], buffer_id_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

CameraFrame::~CameraFrame()
{
	// is_FreeImageMem() does not release the memory, it only removes it from driver management
	// the memory itself is release when the sensor_msgs::Image object is destroyed
	UEYE_TRY( is_FreeImageMem, camera_id_, (char*) &image_->data[0], buffer_id_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ros::Time CameraFrame::get_timestamp() const
{
	UEYEIMAGEINFO image_info;
	UEYE_TRY( is_GetImageInfo, camera_id_, buffer_id_, &image_info, sizeof(image_info) );
	UEYETIME utime = image_info.TimestampSystem;

	struct tm tm;
	tm.tm_year = utime.wYear - 1900;
	tm.tm_mon  = utime.wMonth - 1;
	tm.tm_mday = utime.wDay;
	tm.tm_hour = utime.wHour;
	tm.tm_min  = utime.wMinute;
	tm.tm_sec  = utime.wSecond;

	return ros::Time( mktime(&tm), utime.wMilliseconds * 1e6 );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::vector<UEYE_CAMERA_INFO> Camera::get_camera_list()
{
	std::vector<UEYE_CAMERA_INFO> cameras;

	INT num_cameras;
	UEYE_TRY( is_GetNumberOfCameras, &num_cameras );

	cameras.resize(num_cameras);
	if ( cameras.empty() ) return cameras;

	UEYE_CAMERA_LIST* cam_list = (UEYE_CAMERA_LIST*) new BYTE [sizeof (DWORD) + num_cameras * sizeof (UEYE_CAMERA_INFO) ];
	cam_list->dwCount = num_cameras;

	UEYE_TRY( is_GetCameraList, cam_list );

	for ( int i = 0; i < num_cameras; ++i )
		cameras[i] = cam_list->uci[i];

	delete [] cam_list;
	return cameras;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}	// namespace ueye

