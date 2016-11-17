#include <ros/ros.h>
#include <ros/names.h>

#include <memory>
using namespace std;

#include <ueye_driver/ueye_camera.h>
#include <ueye_driver/capture_thread.h>

#include <nodelet/nodelet.h>

namespace ueye_driver_ns {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class UeyeDriver : public nodelet::Nodelet
{
    shared_ptr<ueye::Camera> camera_;
    shared_ptr<ueye::CaptureThread> capture_thread_;
    
public:

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

	    auto available_cameras = ueye::Camera::get_camera_list();

	    NODELET_INFO("found %u available cameras, connecting to camera serial='%s'", 
		    (unsigned)available_cameras.size(), serial_no.c_str() );

	    for ( auto cam : available_cameras )
		    NODELET_INFO("id=%d serial='%s' model='%s'", cam.dwCameraID, cam.SerNo, cam.Model );

	    auto camera_info = find_if( available_cameras.begin(), available_cameras.end(), 
		    [&serial_no]( const UEYE_CAMERA_INFO& cam_info ) { return string(cam_info.SerNo) == serial_no; } );
	
	    if ( camera_info == available_cameras.end() ) 
		    { ROS_ERROR("camera serial='%s' not found", serial_no.c_str() ); return; }
	      
	    camera_ = make_shared<ueye::Camera>( *camera_info, device_settings );
        if ( enable_reconfigure ) camera_->enable_reconfigure(priv_nh); // see rqt_reconfigure

        image_transport::ImageTransport it( getNodeHandle() );
  	    auto camera_topic = ros::names::clean( camera_name + "/" + topic_name );
        auto camera_pub = it.advertiseCamera( camera_topic, 10 );

        capture_thread_ = make_shared<ueye::CaptureThread>( *camera_ );
        capture_thread_->start( camera_pub, ros::Rate(publish_rate) );
    }
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace ueye_driver_ns

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ueye_driver_ns::UeyeDriver, nodelet::Nodelet ) 

