#include <stdexcept>
#include <vector>
#include <ctime>

#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>

#include <ueye.h>
#include "ueye_camera.h"

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
    { sensor_msgs::image_encodings::MONO8, { IS_CM_MONO8,         8 } },
    { sensor_msgs::image_encodings::BGR8,  { IS_CM_BGR8_PACKED,  24 } },
    { sensor_msgs::image_encodings::RGB8,  { IS_CM_RGB8_PACKED,  24 } } };

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::vector<UEYE_CAMERA_INFO> get_camera_list();
std::vector<IMAGE_FORMAT_INFO> get_image_formats( int camera_id );

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Camera::Camera( int camera_id, int32_t format_id, float frame_rate, const std::string& color_mode )
: camera_id_( (HIDS) camera_id )
{
    for ( auto cam : get_camera_list() )
        ROS_INFO("camera_id=%d device_id=%d sensor=%d serial='%s' model='%s'", 
            cam.dwCameraID, cam.dwDeviceID, cam.dwSensorID, cam.SerNo, cam.Model );

    ROS_INFO("opening camera id=%d", camera_id );
    UEYE_TRY( is_InitCamera, (HIDS*) &camera_id_, NULL );
        
    std::vector<IMAGE_FORMAT_INFO> formats = get_image_formats(camera_id_);
    
    for ( auto format : formats )
        ROS_INFO("format=%02d x%.1f %s", format.nFormatID, format.dSensorScalerFactor, format.strFormatName );
    
    auto format = std::find_if( formats.begin(), formats.end(), 
        [&format_id]( IMAGE_FORMAT_INFO info ){ return info.nFormatID == format_id; } );
        
    if ( format == formats.end() )
        throw std::invalid_argument( "unsupported image format=" + std::to_string(format_id) );
    
    double actual_frame_rate=0;

    UEYE_TRY( is_ImageFormat, camera_id_, IMGFRMT_CMD_SET_FORMAT, &format_id, sizeof(int32_t) );
    UEYE_TRY( is_SetDisplayMode, camera_id_, IS_SET_DM_DIB );
    UEYE_TRY( is_SetFrameRate, camera_id_, frame_rate, &actual_frame_rate );
    UEYE_TRY( is_SetColorMode, camera_id_, color_modes.at(color_mode).ueye_mode );
     
    ROS_INFO("frame rate: requested=%.1fHz actual=%.1fHz", frame_rate, actual_frame_rate );
    frame_ = std::make_shared<CameraFrame>( *this, format->nWidth, format->nHeight, color_mode );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

Camera::~Camera()
{
    frame_.reset(); // TODO
    UEYE_TRY( is_ExitCamera, camera_id_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

bool Camera::set_master_gain( uint8_t master_gain )
{
    if ( is_SetGainBoost( camera_id_, IS_GET_SUPPORTED_GAINBOOST ) != IS_SET_GAINBOOST_ON )
        { ROS_WARN("hardware gain not supported"); return false; }

    if ( master_gain == 0 ) // disable gain
    {   
        UEYE_TRY( is_SetGainBoost, camera_id_, IS_SET_GAINBOOST_OFF );
    }
    else 
    {
        ROS_INFO( "setting master gain=%d", master_gain );
        UEYE_TRY( is_SetGainBoost, camera_id_, IS_SET_GAINBOOST_ON );
        UEYE_TRY( is_SetHardwareGain, camera_id_, master_gain, 
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
        UEYE_TRY( is_EnableEvent, camera_id_, IS_SET_EVENT_FRAME );
        UEYE_TRY( is_SetExternalTrigger, camera_id_, IS_SET_TRIGGER_HI_LO );
        UEYE_TRY( is_CaptureVideo, camera_id_, IS_DONT_WAIT );
    }
    else // freerun mode
    {
        UEYE_TRY( is_EnableEvent, camera_id_, IS_SET_EVENT_FRAME );
        UEYE_TRY( is_CaptureVideo, camera_id_, IS_WAIT );
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const CameraFrame* Camera::get_frame( int timeout_ms )
{
    switch( is_WaitEvent( camera_id_, IS_SET_EVENT_FRAME, timeout_ms ) )
    {
        case IS_SUCCESS:
            frame_->update_timestamp();
            return frame_.get();
        case IS_TIMED_OUT:
            return NULL;
        default:
            UEYE_TRY( is_DisableEvent, camera_id_, IS_SET_EVENT_FRAME );
            throw std::runtime_error("failed to get frame");
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

CameraFrame::CameraFrame( Camera& camera, int image_width, int image_height, const std::string& color_mode )
: camera_(camera)
, buffer_id_(0)
{
    uint16_t bits_per_pixel = color_modes.at(color_mode).bits_per_pixel;

    image_.is_bigendian = false;
    image_.encoding = color_mode;
    image_.width  = image_width;
    image_.height = image_height;
    image_.step = image_.width * bits_per_pixel / 8;
    image_.data.resize( image_.height * image_.step );

    ROS_INFO("allocating %dx%d image depth=%d", 
        image_.width, image_.height, bits_per_pixel );

    // set the camera buffer to be that of the internal 
    // sensor_msgs::Image object to avoid unecessary data copies
    
    UEYE_TRY( is_SetAllocatedImageMem, camera_.camera_id_, 
                image_.width, image_.height, bits_per_pixel, 
                (char*) &image_.data[0], &buffer_id_ );
                
    UEYE_TRY( is_SetImageMem, camera_.camera_id_, (char*) &image_.data[0], buffer_id_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

CameraFrame::~CameraFrame() 
{
    // is_FreeImageMem() does not release the memory, it only removes it from driver management
    // the memory itself is release when the sensor_msgs::Image object is destroyed
    UEYE_TRY( is_FreeImageMem, camera_.camera_id_, (char*) &image_.data[0], buffer_id_ );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

ros::Time CameraFrame::get_timestamp() const
{
    UEYEIMAGEINFO image_info;
    UEYE_TRY( is_GetImageInfo, camera_.camera_id_, buffer_id_, &image_info, sizeof(image_info) );
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

std::vector<UEYE_CAMERA_INFO> get_camera_list()
{
    std::vector<UEYE_CAMERA_INFO> cameras;
    
    INT num_cameras;
    UEYE_TRY( is_GetNumberOfCameras, &num_cameras );

    cameras.resize(num_cameras);
    if ( cameras.empty() ) return cameras;
    
    UEYE_CAMERA_LIST* cam_list = (UEYE_CAMERA_LIST*) new BYTE [sizeof (DWORD) + num_cameras * sizeof (UEYE_CAMERA_INFO) ];
    cam_list->dwCount = num_cameras;  
    
    UEYE_TRY( is_GetCameraList, cam_list );
    
    for ( int i=0; i< num_cameras; ++i )
        cameras[i] = cam_list->uci[i];
        
   delete [] cam_list; 
   return cameras;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

std::vector<IMAGE_FORMAT_INFO> get_image_formats( int camera_id )
{
    UINT num_formats=0;
    UEYE_TRY( is_ImageFormat, camera_id, IMGFRMT_CMD_GET_NUM_ENTRIES, &num_formats, sizeof(num_formats) );
    
    UINT num_bytes = sizeof(IMAGE_FORMAT_LIST) + (num_formats-1) * sizeof(IMAGE_FORMAT_INFO);

    IMAGE_FORMAT_LIST* format_list = (IMAGE_FORMAT_LIST*) malloc(num_bytes);
    format_list->nSizeOfListEntry = sizeof(IMAGE_FORMAT_INFO);
    format_list->nNumListElements = num_formats;

    UEYE_TRY( is_ImageFormat, camera_id, IMGFRMT_CMD_GET_LIST, format_list, num_bytes );

    std::vector<IMAGE_FORMAT_INFO> formats(num_formats);
    
    for ( UINT i=0; i< num_formats; ++i )
        formats[i] = format_list->FormatInfo[i];
        
    return formats;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace ueye

