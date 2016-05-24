#include <ros/ros.h>

#include "ueye_camera.h"

#include <image_transport/image_transport.h>

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    ros::init( argc, argv, ROS_PACKAGE_NAME );
    ros::NodeHandle nh, local_nh("~");

    float frame_rate, publish_rate;
	int master_gain, binning, timeout_ms, image_width, image_height;
	std::string camera_name, camera_topic, frame_id, color_mode;
		
	local_nh.param<int>( "master_gain", master_gain, 0 );
	local_nh.param<int>( "binning", binning, ueye::BinningMode_Off );
	local_nh.param<int>( "timeout_ms", timeout_ms, 100 );
	local_nh.param<int>( "image_width", image_width, 1600 );
	local_nh.param<int>( "image_height", image_height, 1200 );
	local_nh.param<float>( "frame_rate", frame_rate, 30 );
	local_nh.param<float>( "publish_rate", publish_rate, frame_rate );
	local_nh.param<std::string>( "camera_name", camera_name, "camera" );
	local_nh.param<std::string>( "camera_topic", camera_topic, "image_raw" );
	local_nh.param<std::string>( "frame_id", frame_id, "/camera" );
	local_nh.param<std::string>( "color_mode", color_mode, "mono8" );

	if ( binning < 0 || binning >= ueye::BinningMode_End )
		binning = ueye::BinningMode_Off;
		
    image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher pub = it.advertiseCamera( camera_name + "/" + camera_topic, 1 );

	ros::Rate spinner(publish_rate);

	ueye::Camera camera( 0, image_width, image_height, color_mode, frame_rate, (ueye::BinningMode)binning );
    camera.set_master_gain( master_gain );

	while( ros::ok() )
	{
		const ueye::CameraFrame* frame = camera.get_frame(timeout_ms);		
		if (frame) pub.publish( frame->get_image(), camera.get_info() );
        
	    ros::spinOnce();
		spinner.sleep();
	}
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

