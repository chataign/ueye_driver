#include <ros/ros.h>
#include <ros/names.h>

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
    virtual void onInit()
    {
	    //ros::init( argc, argv, ROS_PACKAGE_NAME );
        ros::NodeHandle& priv_nh = getPrivateNodeHandle();
        
	    bool external_trigger = false, hardware_gamma = false;
	    int queue_size, master_gain, timeout_ms, pixel_clock, blacklevel, gamma;
	    std::string serial_no, camera_name, color_mode, image_topic, frame_id;
	    double exposure, frame_rate, publish_rate;
	    IS_RECT aoi_rect;

	    priv_nh.param<int>( "queue_size", queue_size, 10 );
	    priv_nh.param<int>( "pixel_clock", pixel_clock, 84 );
	    priv_nh.param<double>( "exposure", exposure, 40 );
	    priv_nh.param<int>( "blacklevel", blacklevel, 90 );
	    priv_nh.param<int>( "gamma", gamma, 100 );
	    priv_nh.param<bool>( "hardware_gamma", hardware_gamma, false );
	    priv_nh.param<int>( "master_gain", master_gain, 0 );
	    priv_nh.param<int>( "timeout_ms", timeout_ms, 100 );
	    priv_nh.param<bool>( "external_trigger", external_trigger, false );
	    priv_nh.param<double>( "frame_rate", frame_rate, 25 );
	    priv_nh.param<double>( "publish_rate", publish_rate, frame_rate );
	    priv_nh.param<int>( "aoi_width", aoi_rect.s32Width, 1080 );
	    priv_nh.param<int>( "aoi_height", aoi_rect.s32Height, 1080 );
	    priv_nh.param<int>( "aoi_x", aoi_rect.s32X, 20 );
	    priv_nh.param<int>( "aoi_y", aoi_rect.s32Y, 50 );
	    priv_nh.param<std::string>( "serial_no", serial_no, "" );
	    priv_nh.param<std::string>( "frame_id", frame_id, "base_link" );
	    priv_nh.param<std::string>( "camera_name", camera_name, "camera" );
	    priv_nh.param<std::string>( "image_topic", image_topic, "image_raw" );
	    priv_nh.param<std::string>( "color_mode", color_mode, "mono8" );

	    image_transport::ImageTransport it( getNodeHandle() );
	    auto pub = it.advertiseCamera( ros::names::clean( camera_name + "/" + image_topic ), queue_size );

	    auto available_cameras = ueye::Camera::get_camera_list();

	    NODELET_INFO("found %lu available cameras, connecting to camera serial='%s'", 
		    available_cameras.size(), serial_no.c_str() );

	    for ( auto cam : available_cameras )
		    NODELET_INFO("id=%d serial='%s' model='%s'", cam.dwCameraID, cam.SerNo, cam.Model );

	    auto camera_info = std::find_if( available_cameras.begin(), available_cameras.end(), 
		    [&serial_no]( const UEYE_CAMERA_INFO& cam_info ) { return std::string(cam_info.SerNo) == serial_no; } );
	
	    if ( camera_info == available_cameras.end() ) 
		    { ROS_ERROR("invalid camera id"); return; }
	
	    ueye::Camera camera( *camera_info, frame_id, frame_rate, color_mode, pixel_clock, aoi_rect ) ;

	    NODELET_INFO("Publish_rate set to %.1f \t [equals frame actual frame rate set]", publish_rate);
	    ros::Rate spinner(publish_rate);

	    camera.set_exposure( exposure );
	    camera.set_master_gain( master_gain );
	    camera.set_blacklevel(blacklevel);
	    camera.set_gamma(gamma);
	    if (hardware_gamma) camera.set_hardware_gamma();
	    camera.start_capture( external_trigger );

	    while ( ros::ok() )
	    {
		    const ueye::CameraFrame* frame = camera.get_frame(timeout_ms);
		    if (frame) pub.publish( frame->get_image(), camera.get_info() );

		    ros::spinOnce();
		    spinner.sleep();
	    }
    }
};

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace ueye_driver_ns

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ueye_driver_ns::DriverNodelet, nodelet::Nodelet ) 

