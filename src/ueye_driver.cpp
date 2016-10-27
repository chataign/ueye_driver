#include <ros/ros.h>
#include <ros/names.h>

#include <thread>
#include <memory>
using namespace std;

#include <opencv/cv.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "ueye_camera.h"

#include <nodelet/nodelet.h>

namespace ueye_driver_ns {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct DriverNodelet : public nodelet::Nodelet
{
    bool capture_alive_;
    thread capture_thread_;
    
    DriverNodelet(): capture_alive_(false) {}
    virtual ~DriverNodelet() { stop(); }
    
    virtual void onInit()
    {
        ros::NodeHandle& priv_nh = getPrivateNodeHandle();
        
	    string serial_no, camera_name, topic_name;
	    double publish_rate;
	    ueye::DeviceSettings device_settings;
	    ueye::CaptureSettings capture_settings;

	    priv_nh.param<double>   ( "publish_rate",   publish_rate,                           25 );
	    priv_nh.param<string>   ( "serial_no",      serial_no,                              "" );
	    priv_nh.param<string>   ( "camera_name",    camera_name,                            "camera" );
	    priv_nh.param<string>   ( "topic_name",     topic_name,                             "image_raw" );
        priv_nh.param<int>      ( "pixel_clock",    device_settings.pixel_clock,            35 );
        priv_nh.param<double>   ( "frame_rate",     device_settings.frame_rate,             25 );
        priv_nh.param<int>      ( "aoi_width",      device_settings.aoi_rect.s32Width,      1080 );
        priv_nh.param<int>      ( "aoi_height",     device_settings.aoi_rect.s32Height,     1080 );
        priv_nh.param<int>      ( "aoi_x",          device_settings.aoi_rect.s32X,          20 );
        priv_nh.param<int>      ( "aoi_y",          device_settings.aoi_rect.s32Y,          50 );
        priv_nh.param<string>   ( "frame_id",       device_settings.frame_id,               "base_link" );
        priv_nh.param<string>   ( "color_mode",     device_settings.color_mode,             "mono8" );
        priv_nh.param<bool>     ( "external_trigger", capture_settings.external_trigger,    false );
        priv_nh.param<bool>     ( "hardware_gamma", capture_settings.hardware_gamma,        false );
        priv_nh.param<int>      ( "timeout_ms",     capture_settings.timeout_ms,            100 );
        priv_nh.param<int>      ( "master_gain",    capture_settings.master_gain,           0 );
        priv_nh.param<int>      ( "blacklevel",     capture_settings.blacklevel,            90 );
        priv_nh.param<int>      ( "gamma",          capture_settings.gamma,                 100 );
        priv_nh.param<double>   ( "exposure",       capture_settings.exposure,              40 );

	    auto camera_topic = ros::names::clean( camera_name + "/" + topic_name );
	    auto available_cameras = ueye::Camera::get_camera_list();

	    NODELET_INFO("found %lu available cameras, connecting to camera serial='%s'", 
		    available_cameras.size(), serial_no.c_str() );

	    for ( auto cam : available_cameras )
		    NODELET_INFO("id=%d serial='%s' model='%s'", cam.dwCameraID, cam.SerNo, cam.Model );

	    auto camera_info = find_if( available_cameras.begin(), available_cameras.end(), 
		    [&serial_no]( const UEYE_CAMERA_INFO& cam_info ) { return string(cam_info.SerNo) == serial_no; } );
	
	    if ( camera_info == available_cameras.end() ) 
		    { ROS_ERROR("invalid camera id"); return; }
	      
        capture_thread_ = thread( bind( &DriverNodelet::capture_loop, this, 
            *camera_info, device_settings, capture_settings, camera_topic, ros::Rate(publish_rate) ) );
    }
    
    void capture_loop(   UEYE_CAMERA_INFO device_info, 
                            ueye::DeviceSettings device_settings, 
                            ueye::CaptureSettings capture_settings, 
                            string camera_topic, ros::Rate publish_rate )
    {
	    ueye::Camera camera( device_info, device_settings );
	    
	    image_transport::ImageTransport it( getNodeHandle() );
	    auto pub = it.advertiseCamera( camera_topic, 10 );

        NODELET_INFO("starting capture thread");
        camera.start_capture( capture_settings );
        capture_alive_ = true;
        
        while ( capture_alive_ && ros::ok() )
	    {
		    const ueye::CameraFrame* frame = camera.get_frame( capture_settings.timeout_ms );
		    if (frame) pub.publish( frame->get_image(), camera.get_info() );

		    publish_rate.sleep();
	    }
    }

    void stop()
    {
        capture_alive_ = false;
        
        if ( capture_thread_.joinable() )
            capture_thread_.join();

        NODELET_INFO("stopped capture thread");
        capture_thread_ = thread();
    }
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace ueye_driver_ns

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ueye_driver_ns::DriverNodelet, nodelet::Nodelet ) 

