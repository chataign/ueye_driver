#include <stdexcept>
#include <vector>
#include <ctime>
using namespace std;

#include <ros/console.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>

#include <ueye_driver/ueye_camera.h>

namespace ueye
{
#define UEYE_TRY( FUNC, ... ) { INT err = (FUNC)(__VA_ARGS__); if ( err != IS_SUCCESS ) { \
	throw runtime_error( string(#FUNC)+" failed, error="+to_string(err) ); } }

#define CAM_INFO( FORMAT, ... ) ROS_INFO( "[cam=%s]" # FORMAT, device_info_.SerNo, __VA_ARGS__ )
#define CAM_WARN( FORMAT, ... ) ROS_WARN( "[cam=%s]" # FORMAT, device_info_.SerNo, __VA_ARGS__ )

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct ColorMode
{
	INT ueye_mode;
	uint16_t bits_per_pixel;
};

static const map< string, ColorMode > color_modes = {
	{ sensor_msgs::image_encodings::MONO8, { IS_CM_MONO8,         8 } },
	{ sensor_msgs::image_encodings::BGR8,  { IS_CM_BGR8_PACKED,  24 } },
	{ sensor_msgs::image_encodings::RGB8,  { IS_CM_RGB8_PACKED,  24 } }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Camera::Camera( const UEYE_CAMERA_INFO& device_info, const DeviceSettings& device_settings )
	: device_info_( device_info )
	, camera_info_( boost::make_shared<sensor_msgs::CameraInfo>() )
	, configured_(false)
	, timeout_ms_(0)
{
	CAM_INFO("opening camera id=%d", device_info_.dwCameraID );
	UEYE_TRY( is_InitCamera, (HIDS*) &device_info_.dwCameraID, NULL );

    set_color_mode(device_settings.color_mode);
    set_aoi(device_settings.aoi_rect);

	frame_ = make_shared<CameraFrame>( *this, device_settings );
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

void Camera::enable_reconfigure( ros::NodeHandle& reconfigure_nh )
{
    reconfigure_server_ = make_shared<reconfigure_server_t>(reconfigure_nh);
    reconfigure_server_->setCallback( boost::bind( &Camera::reconfigure_callback, this, _1, _2 ) );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Camera::start_capture()
{
    //lock_guard<mutex> scoped_lock(mutex_);
    
    // external trigger
    //	UEYE_TRY( is_EnableEvent, device_info_.dwCameraID, IS_SET_EVENT_FRAME );
    //	UEYE_TRY( is_SetExternalTrigger, device_info_.dwCameraID, IS_SET_TRIGGER_HI_LO );
    //	UEYE_TRY( is_CaptureVideo, device_info_.dwCameraID, IS_DONT_WAIT );

    // freerun mode
	UEYE_TRY( is_EnableEvent, device_info_.dwCameraID, IS_SET_EVENT_FRAME );
	UEYE_TRY( is_CaptureVideo, device_info_.dwCameraID, IS_WAIT );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Camera::apply_config( const CaptureConfig& config )
{
    //lock_guard<mutex> scoped_lock(mutex_);
    
    timeout_ms_ = config.timeout_ms;
    
    // (from UEye documentation)
    // In general, the pixel clock is set once when opening the camera and will not be changed. 
    // Note that, if you change the pixel clock, the setting ranges for frame rate and exposure 
    // time also changes. If you change a parameter, the following order is recommended:
    // 1. Change pixel clock.
    // 2. Query frame rate range and, if applicable, set new value.
    // 3. Query exposure time range and, if applicable, set new value.

    set_pixelclock( config.pixelclock );
    set_framerate( config.framerate );
    set_exposure( config.exposure );
    set_hardware_gain( config.hardware_gain );
    set_blacklevel( config.blacklevel );
    set_gamma( config.gamma );
    set_hardware_gamma( config.hardware_gamma );
    
    configured_ = true;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const CameraFrame* Camera::get_frame()
{
    //lock_guard<mutex> scoped_lock(mutex_);
        
    if (!configured_)
    {
        ROS_WARN_THROTTLE( 1, "[%s] camera not configured!", device_info_.SerNo );
        return NULL;
    }
    
	switch ( is_WaitEvent( device_info_.dwCameraID, IS_SET_EVENT_FRAME, timeout_ms_ ) )
	{
	case IS_SUCCESS:
		frame_->update_timestamp();
		return frame_.get();
	case IS_TIMED_OUT:
		return NULL;
	default:
		UEYE_TRY( is_DisableEvent, device_info_.dwCameraID, IS_SET_EVENT_FRAME );
		throw runtime_error("camera=" + string(device_info_.SerNo) + ", failed to get frame");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_color_mode( const string& color_mode )
{
    auto umode = color_modes.at(color_mode).ueye_mode;
    
	UEYE_TRY( is_SetDisplayMode, device_info_.dwCameraID, IS_SET_DM_DIB );
	INT res = is_SetColorMode( device_info_.dwCameraID, umode );

	switch( res )
	{
	    case IS_SUCCESS:
        	CAM_INFO("set color mode=%s (%d)", color_mode.c_str(), umode );
	        return true;
        case IS_NOT_SUPPORTED:
        case IS_INVALID_PARAMETER:
	        ROS_WARN("color mode=%s not supported", color_mode.c_str() );
	        return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting color mode=" + color_mode );
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

IS_RECT Camera::get_aoi() const
{
    IS_RECT aoi;
    UEYE_TRY( is_AOI, device_info_.dwCameraID, IS_AOI_IMAGE_GET_AOI, (void*)&aoi, sizeof(aoi) );
    return aoi;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_aoi( const IS_RECT& aoi )
{
    CAM_INFO("setting AOI: tl=(%d,%d) width=%d height=%d", 
        aoi.s32X, aoi.s32Y, aoi.s32Width, aoi.s32Height );

	INT res = is_AOI( device_info_.dwCameraID, IS_AOI_IMAGE_SET_AOI, (void*)&aoi, sizeof(aoi) );

	switch( res )
	{
	    case IS_SUCCESS:
	        return true;
        case IS_NOT_SUPPORTED:
	        ROS_WARN("AOI not supported");
	        return false;
	    case IS_INVALID_PARAMETER:
	        ROS_WARN("requested AOI is invalid");
	        return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting AOI");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

double Camera::get_exposure() const
{
    double exposure;
	UEYE_TRY( is_Exposure, device_info_.dwCameraID, IS_EXPOSURE_CMD_GET_EXPOSURE, 
	    (void*)&exposure, sizeof(exposure));
	return exposure;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

IS_RANGE_F64 Camera::get_exposure_range() const
{
    IS_RANGE_F64 range;
	UEYE_TRY( is_Exposure, device_info_.dwCameraID, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE, 
	    (void*)&range, sizeof(range) );
	return range;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_exposure( double exposure )
{
	INT res = is_Exposure( device_info_.dwCameraID, IS_EXPOSURE_CMD_SET_EXPOSURE,
			(void*)&exposure, sizeof(exposure) );

    auto exposure_range = get_exposure_range();

	switch( res )
	{
	    case IS_SUCCESS:
	        CAM_INFO("set exposure=%.1f (min=%.1f max=%.1f)", 
	            exposure, exposure_range.f64Min, exposure_range.f64Max );
	        return true;
        case IS_NOT_SUPPORTED:
	        CAM_WARN("exposure not supported (requested=%.1f)", exposure );
	        return false;
	    case IS_INVALID_PARAMETER:
	        CAM_WARN("requested exposure=%.1f is invalid (min=%.1f max=%.1f)", 
	            exposure, exposure_range.f64Min, exposure_range.f64Max );
	        return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting exposure");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int Camera::get_blacklevel() const
{
    int blacklevel;
	UEYE_TRY( is_Blacklevel, device_info_.dwCameraID, IS_BLACKLEVEL_CMD_GET_OFFSET,
			(void*)&blacklevel, sizeof(blacklevel) );
	return blacklevel;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

IS_RANGE_S32 Camera::get_blacklevel_range() const
{
    IS_RANGE_S32 range;
    UEYE_TRY( is_Blacklevel, device_info_.dwCameraID, IS_BLACKLEVEL_CMD_GET_OFFSET_RANGE, 
        (void*)&range, sizeof(range) );
    return range;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_blacklevel( int blacklevel )
{
	INT res = is_Blacklevel( device_info_.dwCameraID, IS_BLACKLEVEL_CMD_SET_OFFSET, 
	    (void*)&blacklevel, sizeof(blacklevel) );
	
	switch( res )
	{
	    case IS_SUCCESS:
	        return true;
        case IS_NOT_SUPPORTED:
	        ROS_WARN("blacklevel not supported");
	        return false;
	    case IS_INVALID_PARAMETER:
	    {
            auto range = get_blacklevel_range();
	        ROS_WARN("requested blacklevel=%d is invalid (min=%d max=%d)", 
	            blacklevel, range.s32Min, range.s32Max );
	        return false;
        }
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting blacklevel");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

INT Camera::get_gamma() const
{
	INT gamma;
	UEYE_TRY( is_Gamma, device_info_.dwCameraID, IS_GAMMA_CMD_GET, 
	    (void*)&gamma, sizeof(gamma) );
	return gamma;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_gamma( INT gamma )
{
	INT res = is_Gamma( device_info_.dwCameraID, IS_GAMMA_CMD_SET, 
	    (void*)&gamma, sizeof(gamma) );
	
	switch( res )
	{
	    case IS_SUCCESS:
	        CAM_INFO("set gamma=%d, range=[1,1000]", gamma );
	        return true;
        case IS_NOT_SUPPORTED:
	        ROS_WARN("gamma not supported");
	        return false;
	    case IS_INVALID_PARAMETER:
	        CAM_WARN("requested gamma=%d is invalid (range=[1,1000])", (int)gamma );
	        return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting gamma");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

double Camera::get_framerate() const
{
	double framerate=0;
	UEYE_TRY( is_SetFrameRate, device_info_.dwCameraID, IS_GET_FRAMERATE, &framerate );
	return framerate;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

IS_RANGE_F64 Camera::get_framerate_range() const
{
	double min_time, max_time, interval;
	UEYE_TRY( is_GetFrameTimeRange, device_info_.dwCameraID, &min_time, &max_time, &interval );
	
	IS_RANGE_F64 range;
	range.f64Min = 1/max_time;
	range.f64Max = 1/min_time;
	range.f64Inc = 1/interval;
	
	return range;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_framerate( double framerate )
{
	double framerate_actual=0;
	INT res = is_SetFrameRate( device_info_.dwCameraID, framerate, &framerate_actual );
	auto framerate_range = get_framerate_range();
	        
	switch( res )
	{
	    case IS_SUCCESS:
	        CAM_INFO("set framerate=%.1fHz requested=%.1f (min=%.1f max=%.1f)", 
	            framerate_actual, framerate, framerate_range.f64Min, framerate_range.f64Max );
	        return true;
	    case IS_INVALID_PARAMETER:
	        ROS_WARN("requested framerate=%.1f is invalid (range=[%.1f,%.1f])", 
	            framerate, framerate_range.f64Min, framerate_range.f64Max );
	        return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting framerate");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

UINT Camera::get_pixelclock() const
{
    UINT pixelclock; 

    UEYE_TRY( is_PixelClock, device_info_.dwCameraID, IS_PIXELCLOCK_CMD_GET, 
        (void*)&pixelclock, sizeof(pixelclock) );
            
    return pixelclock;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

IS_RANGE_S32 Camera::get_pixelclock_range() const
{
    UINT pixelclock_range[3]; // [ min, max, increment ]
    ZeroMemory( pixelclock_range, sizeof(pixelclock_range) );
     
    UEYE_TRY( is_PixelClock, device_info_.dwCameraID, IS_PIXELCLOCK_CMD_GET_RANGE, 
        (void*)pixelclock_range, sizeof(pixelclock_range) );

    IS_RANGE_S32 range;
    range.s32Min = pixelclock_range[0];
    range.s32Max = pixelclock_range[1];
    range.s32Inc = pixelclock_range[2];
    
    return range;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_pixelclock( UINT pixelclock )
{
	INT res = is_PixelClock( device_info_.dwCameraID, IS_PIXELCLOCK_CMD_SET, (void*)&pixelclock, sizeof(pixelclock) );

	switch( res )
	{
	    case IS_SUCCESS:
	        CAM_INFO( "set pixelclock=%dMHz", (int)pixelclock );
	        return true;
        case IS_NOT_SUPPORTED:
	        ROS_WARN("pixelclock not supported");
	        return false;
	    case IS_INVALID_PARAMETER:
	    {
	        auto range = get_pixelclock_range();
	        ROS_WARN("requested pixelclock=%d is invalid (range=[%d,%d] inc=%d)", 
	            (int)pixelclock, range.s32Min, range.s32Max, range.s32Inc );
	        return false;
        }
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting pixelclock");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_hardware_gamma( bool enable )
{
	INT res = is_SetHardwareGamma( device_info_.dwCameraID, enable ? IS_SET_HW_GAMMA_ON : IS_SET_HW_GAMMA_OFF );

	switch( res )
	{
	    case IS_SUCCESS:
	        CAM_INFO("set hardware gamma %s", enable ? "ON" : "OFF" );
	        return true;
        case IS_NOT_SUPPORTED:
	        ROS_WARN("hardware gamma not supported");
	        return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting hardware gamma");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::get_gain_boost() const
{
	return is_SetGainBoost( device_info_.dwCameraID, IS_GET_GAINBOOST ) == IS_SET_GAINBOOST_ON;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_gain_boost( bool enable )
{
	INT res = is_SetGainBoost( device_info_.dwCameraID, enable ? IS_SET_GAINBOOST_ON : IS_SET_GAINBOOST_OFF );

	switch( res )
	{
	    case IS_SUCCESS:
	        CAM_INFO("set gain boost %s", enable ? "ON" : "OFF" );
	        return true;
        case IS_NOT_SUPPORTED:
	        ROS_WARN("gain boost not supported");
	        return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting gain boost");
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

UINT Camera::get_hardware_gain() const
{
	return is_SetHardwareGain( device_info_.dwCameraID, IS_GET_MASTER_GAIN,
	    IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_hardware_gain( uint8_t hardware_gain )
{
    bool gain_boost = ( hardware_gain > 0 );
    if ( !set_gain_boost(gain_boost) ) return false;

	INT res = is_SetHardwareGain( device_info_.dwCameraID, hardware_gain,
		IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER );

	switch( res )
	{
	    case IS_SUCCESS:
	        CAM_INFO("set hardware gain=%d", (int)hardware_gain );
	        return true;
        case IS_NOT_SUPPORTED:
	        ROS_WARN("hardware gain not supported");
	        return false;
        case IS_INVALID_PARAMETER:
	        ROS_WARN("requested hardware gain=%d is invalid (range=[0,100])", hardware_gain );
            return false;
        default:
            throw runtime_error("unknown error=" + to_string(res) + " setting gain boost");
	}
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

	ROS_INFO("[%s] allocating %dx%d image depth=%d",
		camera.device_info_.SerNo, image_->width, image_->height, bits_per_pixel );

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

vector<UEYE_CAMERA_INFO> Camera::get_camera_list()
{
	vector<UEYE_CAMERA_INFO> cameras;

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

