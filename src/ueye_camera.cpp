#include "ueye_camera.h"

#include <stdexcept>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <ueye.h>

namespace ueye
{

// https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html?sdk_fehlermeldungen.html

#define UEYE_TRY( FUNC, ... ) { INT err = (FUNC)(__VA_ARGS__); if ( err != IS_SUCCESS ) \
    throw std::runtime_error("call failed="+std::string(#FUNC)+" error="+std::to_string(err)); }

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct ColorMode
{
    INT ueye_mode;
    uint16_t bits_per_pixel;
};

static const std::map< std::string, ColorMode > color_modes = {
    { sensor_msgs::image_encodings::MONO8, ColorMode{ IS_CM_MONO8,         8 } },
    { sensor_msgs::image_encodings::BGR8,  ColorMode{ IS_CM_BGR8_PACKED,  24 } },
    { sensor_msgs::image_encodings::RGB8,  ColorMode{ IS_CM_RGB8_PACKED,  24 } } };

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static const std::map<BinningMode,int> binning_rates = { 
    { BinningMode_Off, IS_BINNING_DISABLE                                  },
    { BinningMode_2X,  IS_BINNING_2X_VERTICAL  | IS_BINNING_2X_HORIZONTAL  },
    { BinningMode_4X,  IS_BINNING_4X_VERTICAL  | IS_BINNING_4X_HORIZONTAL  },
    { BinningMode_8X,  IS_BINNING_8X_VERTICAL  | IS_BINNING_8X_HORIZONTAL  }, 
    { BinningMode_16X, IS_BINNING_16X_VERTICAL | IS_BINNING_16X_HORIZONTAL } }; 

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Camera::Camera( int camera_id, uint16_t aoi_width, uint16_t aoi_height, 
                const std::string& encoding, float frame_rate, BinningMode binning )
: handle_( (HIDS) camera_id )
{
    UEYE_TRY( is_InitCamera, (HIDS*) &handle_, NULL );
    UEYE_TRY( is_SetDisplayMode, handle_, IS_SET_DM_DIB );
    
    double actual_frame_rate=0;
    IS_RECT rect = { 0, 0, aoi_width, aoi_height };

    UEYE_TRY( is_AOI, handle_, IS_AOI_IMAGE_SET_AOI, &rect, sizeof(rect) );
    UEYE_TRY( is_SetFrameRate, handle_, frame_rate, &actual_frame_rate );
    UEYE_TRY( is_SetColorMode, handle_, color_modes.at(encoding).ueye_mode );
    UEYE_TRY( is_SetBinning, handle_, binning_rates.at(binning) );
//    UEYE_TRY( is_SetSubSampling,     handle_, IS_SUBSAMPLING_DISABLE );
//    UEYE_TRY( is_SetSensorScaler, handle_, IS_ENABLE_SENSOR_SCALER, 1.0 );

    ROS_INFO("frame rate: requested=%.1f actual=%.1f", frame_rate, actual_frame_rate );

    frame_ = std::make_shared<CameraFrame>( *this, aoi_width, aoi_height, encoding );

    start_capture( RunMode_FreeRun );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Camera::~Camera()
{
    UEYE_TRY( is_ExitCamera, handle_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

float Camera::get_sensor_scaling() const
{
    SENSORSCALERINFO scaler_info;
    UEYE_TRY( is_GetSensorScalerInfo, handle_, &scaler_info, sizeof(scaler_info) );
    return scaler_info.dblCurrFactor;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

uint32_t Camera::get_buffer_pitch() const
{
    int buffer_pitch=0;
    UEYE_TRY( is_GetImageMemPitch, handle_, &buffer_pitch );
    return buffer_pitch;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_gain( uint8_t master_gain )
{
    bool supported = is_SetGainBoost( handle_, IS_GET_SUPPORTED_GAINBOOST ) == IS_SET_GAINBOOST_ON;
    if ( !supported ) { ROS_WARN("hardware gain not supported"); return false; }

    if ( master_gain == 0 )
    {   
        UEYE_TRY( is_SetGainBoost, handle_, IS_SET_GAINBOOST_OFF );
        return true;
    }
    else 
    {
        UEYE_TRY( is_SetGainBoost, handle_, IS_SET_GAINBOOST_ON );
        ROS_INFO( "setting master gain=%d", master_gain );

        UEYE_TRY( is_SetHardwareGain, handle_, master_gain, 
            IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER );
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::is_capturing() const
{
    return ( is_CaptureVideo( handle_, IS_GET_LIVE ) == TRUE );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Camera::start_capture( RunMode mode )
{
    ROS_INFO("starting camera in mode=%d", mode );

    switch( mode )
    {
        case RunMode_FreeRun:
            UEYE_TRY( is_EnableEvent, handle_, IS_SET_EVENT_FRAME );
            UEYE_TRY( is_CaptureVideo, handle_, IS_WAIT );
            break;
        
        case RunMode_ExternalTrigger:
            UEYE_TRY( is_EnableEvent, handle_, IS_SET_EVENT_FRAME );
            UEYE_TRY( is_SetExternalTrigger, handle_, IS_SET_TRIGGER_HI_LO );
            UEYE_TRY( is_CaptureVideo, handle_, IS_DONT_WAIT );
            break;
            
        default:
            throw std::runtime_error("invalid mode to set");
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const CameraFrame* Camera::get_frame( int timeout_ms )
{
    switch( is_WaitEvent( handle_, IS_SET_EVENT_FRAME, timeout_ms ) )
    {
        case IS_SUCCESS:
            frame_->update_timestamp();
            return frame_.get();
        case IS_TIMED_OUT:
            return NULL;
        default:
            UEYE_TRY( is_DisableEvent, handle_, IS_SET_EVENT_FRAME );
            throw std::runtime_error("failed to get frame");
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

uint16_t Camera::get_binning_rate() const
{
    INT rate_flag = is_SetBinning( handle_, IS_GET_BINNING );
    
    switch( rate_flag )
    {
        case IS_BINNING_DISABLE:
            return 1;
        case IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL:
            return 2;
        case IS_BINNING_4X_VERTICAL | IS_BINNING_4X_HORIZONTAL:
            return 4;
        case IS_BINNING_8X_VERTICAL | IS_BINNING_8X_HORIZONTAL:
            return 8;
        case IS_BINNING_16X_VERTICAL | IS_BINNING_16X_HORIZONTAL:
            return 16;
    }
    
    throw std::runtime_error( "unsupported binning rate flag=" + std::to_string(rate_flag) );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

CameraFrame::CameraFrame( Camera& camera, int cols, int rows, const std::string& encoding )
: camera_(camera)
, buffer_id_(0)
{
    uint16_t bits_per_pixel = color_modes.at(encoding).bits_per_pixel;
    float binning = camera_.get_binning_rate();
    float scaling = camera_.get_sensor_scaling();

    image_.is_bigendian = false;
    image_.encoding = encoding;
    image_.width  = cols / scaling / binning;
    image_.height = rows / scaling / binning;
    image_.step = image_.width * bits_per_pixel / 8;
    image_.data.resize( image_.height * image_.step );

    ROS_INFO("allocating %dx%d image depth=%d binning=%.1f scaling=%.1f", 
        image_.width, image_.height, bits_per_pixel, binning, scaling );

    // set the camera buffer to be that of the internal 
    // sensor_msgs::Image object to avoid unecessary data copies
    
    UEYE_TRY( is_SetAllocatedImageMem, camera_.handle_, 
                image_.width, image_.height, bits_per_pixel, 
                (char*) &image_.data[0], &buffer_id_ );
                
    UEYE_TRY( is_SetImageMem, camera_.handle_, (char*) &image_.data[0], buffer_id_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

CameraFrame::~CameraFrame() 
{
    // FreeImageMem does not release the memory, but it will be when image object is destroyed
    UEYE_TRY( is_FreeImageMem, camera_.handle_, (char*) &image_.data[0], buffer_id_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ros::Time CameraFrame::get_timestamp() const
{
    UEYEIMAGEINFO image_info;
    UEYE_TRY( is_GetImageInfo, camera_.handle_, buffer_id_, &image_info, sizeof(image_info) );
    UEYETIME utime = image_info.TimestampSystem;
    
    struct tm tm;
    tm.tm_year = utime.wYear - 1900;
    tm.tm_mon  = utime.wMonth - 1;
    tm.tm_mday = utime.wDay;
    tm.tm_hour = utime.wHour;
    tm.tm_min  = utime.wMinute;
    tm.tm_sec  = utime.wSecond;

    return ros::Time( mktime(&tm), utime.wMilliseconds*1e6 );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace ueye

