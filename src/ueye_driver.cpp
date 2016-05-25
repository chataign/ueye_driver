#include <ros/ros.h>
#include <memory>
#include <signal.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "ueye_camera.h"

bool _running=true;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    ros::init( argc, argv, ROS_PACKAGE_NAME, ros::init_options::NoSigintHandler );
    ros::NodeHandle nh, local_nh("~");

    auto terminate = [](int) { _running=false; };

	signal( SIGINT,  terminate );
	signal( SIGTERM, terminate );

    bool external_trigger=false;
    float frame_rate, publish_rate;
	int master_gain, image_format, timeout_ms;
	std::string camera_name, color_mode, image_topic;
		
	local_nh.param<int>( "master_gain", master_gain, 0 );
	local_nh.param<int>( "image_format", image_format, 20 );
	local_nh.param<int>( "timeout_ms", timeout_ms, 100 );
	local_nh.param<bool>( "external_trigger", external_trigger, false );
	local_nh.param<float>( "frame_rate", frame_rate, 30 );
	local_nh.param<float>( "publish_rate", publish_rate, frame_rate );
	local_nh.param<std::string>( "camera_name", camera_name, "camera" );
	local_nh.param<std::string>( "image_topic", image_topic, "image_raw" );
	local_nh.param<std::string>( "color_mode", color_mode, "mono8" );

    image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher pub = it.advertiseCamera( camera_name + "/" + image_topic, 1 );

	ros::Rate spinner(publish_rate);
    sensor_msgs::CameraInfo dummy_info; // TODO
        
	ueye::Camera camera( 0, image_format, frame_rate, color_mode );

    camera.set_master_gain( master_gain );
    camera.start_capture( external_trigger );
    
	while( ros::ok() && _running )
	{
		const ueye::CameraFrame* frame = camera.get_frame(timeout_ms);		
		if (frame) pub.publish( frame->get_image(), dummy_info );
        
	    ros::spinOnce();
		spinner.sleep();
	}
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

