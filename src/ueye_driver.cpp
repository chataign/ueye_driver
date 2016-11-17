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

class UeyeDriver : public nodelet::Nodelet
{
    bool capture_alive_;
    thread capture_thread_;
    shared_ptr<ueye::Camera> camera_;
    
public:

    UeyeDriver(): capture_alive_(false) {}
    virtual ~UeyeDriver() { stop(); }
    
    virtual void onInit()
    {
        ros::NodeHandle& priv_nh = getPrivateNodeHandle();
        
	    double publish_rate;
	    bool enable_reconfigure;
	    string serial_no, camera_name, topic_name;
	    ueye::DeviceSettings device_settings;

	    priv_nh.param<bool>      ( "enable_reconfigure",   enable_reconfigure,               true );
	    priv_nh.param<double>   ( "publish_rate",   publish_rate,                           25 );
	    priv_nh.param<string>   ( "serial_no",      serial_no,                              "" );
	    priv_nh.param<string>   ( "camera_name",    camera_name,                            "camera" );
	    priv_nh.param<string>   ( "topic_name",     topic_name,                             "image_raw" );
        priv_nh.param<int>      ( "aoi_width",      device_settings.aoi_rect.s32Width,      1080 );
        priv_nh.param<int>      ( "aoi_height",     device_settings.aoi_rect.s32Height,     1080 );
        priv_nh.param<int>      ( "aoi_x",          device_settings.aoi_rect.s32X,          220 );
        priv_nh.param<int>      ( "aoi_y",          device_settings.aoi_rect.s32Y,          50 );
        priv_nh.param<string>   ( "frame_id",       device_settings.frame_id,               "base_link" );
        priv_nh.param<string>   ( "color_mode",     device_settings.color_mode,             "mono8" );

  	    auto camera_topic = ros::names::clean( camera_name + "/" + topic_name );
	    auto available_cameras = ueye::Camera::get_camera_list();

	    NODELET_INFO("found %u available cameras, connecting to camera serial='%s'", 
		    (unsigned)available_cameras.size(), serial_no.c_str() );

	    for ( auto cam : available_cameras )
		    NODELET_INFO("id=%d serial='%s' model='%s'", cam.dwCameraID, cam.SerNo, cam.Model );

	    auto camera_info = find_if( available_cameras.begin(), available_cameras.end(), 
		    [&serial_no]( const UEYE_CAMERA_INFO& cam_info ) { return string(cam_info.SerNo) == serial_no; } );
	
	    if ( camera_info == available_cameras.end() ) 
		    { ROS_ERROR("invalid camera id"); return; }
	      
	    camera_ = make_shared<ueye::Camera>( *camera_info, device_settings );
        if ( enable_reconfigure ) camera_->enable_reconfigure(priv_nh); // see rqt_reconfigure

        capture_thread_ = thread( bind( &UeyeDriver::capture_loop, this, 
            camera_topic, ros::Rate(publish_rate) ) );
    }
        
    void capture_loop( string camera_topic, ros::Rate publish_rate )
    {
        if ( !camera_ )
            throw runtime_error("camera hasnt been initialized!");
            
	    image_transport::ImageTransport it( getNodeHandle() );
	    auto pub = it.advertiseCamera( camera_topic, 10 );

        NODELET_INFO("starting capture thread");
        camera_->start_capture();
        capture_alive_ = true;

        while ( capture_alive_ && ros::ok() )
	    {
		    const ueye::CameraFrame* frame = camera_->get_frame();
		    if (frame) pub.publish( frame->get_image(), camera_->get_info() );

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
PLUGINLIB_EXPORT_CLASS( ueye_driver_ns::UeyeDriver, nodelet::Nodelet ) 

