#include <ros/ros.h>
#include <ros/names.h>

#include <thread>
#include <memory>
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
    std::thread capture_thread_;
    
    DriverNodelet(): capture_alive_(false) {}
    virtual ~DriverNodelet() { stop(); }
    
    virtual void onInit()
    {
        ros::NodeHandle& priv_nh = getPrivateNodeHandle();
        
	    std::string serial_no, camera_name, topic_name;
	    double publish_rate;

	    priv_nh.param<double>( "publish_rate", publish_rate, 25 );
	    priv_nh.param<std::string>( "serial_no", serial_no, "" );
	    priv_nh.param<std::string>( "camera_name", camera_name, "camera" );
	    priv_nh.param<std::string>( "topic_name", topic_name, "image_raw" );

	    auto camera_topic = ros::names::clean( camera_name + "/" + topic_name );
	    auto available_cameras = ueye::Camera::get_camera_list();

	    NODELET_INFO("found %lu available cameras, connecting to camera serial='%s'", 
		    available_cameras.size(), serial_no.c_str() );

	    for ( auto cam : available_cameras )
		    NODELET_INFO("id=%d serial='%s' model='%s'", cam.dwCameraID, cam.SerNo, cam.Model );

	    auto camera_info = std::find_if( available_cameras.begin(), available_cameras.end(), 
		    [&serial_no]( const UEYE_CAMERA_INFO& cam_info ) { return std::string(cam_info.SerNo) == serial_no; } );
	
	    if ( camera_info == available_cameras.end() ) 
		    { ROS_ERROR("invalid camera id"); return; }
	      
        capture_thread_ = std::thread( std::bind( &DriverNodelet::capture_loop, this, 
            *camera_info, ueye::DeviceSettings(priv_nh), ueye::CaptureSettings(priv_nh), 
            camera_topic, ros::Rate(publish_rate) ) );
    }
    
    void capture_loop(   UEYE_CAMERA_INFO device_info, 
                            ueye::DeviceSettings device_settings, 
                            ueye::CaptureSettings capture_settings, 
                            std::string camera_topic, ros::Rate publish_rate )
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
        capture_thread_ = std::thread();
    }
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace ueye_driver_ns

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ueye_driver_ns::DriverNodelet, nodelet::Nodelet ) 

