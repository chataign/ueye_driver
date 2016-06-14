#include <ros/ros.h>
#include <memory>
#include <signal.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "ueye_camera.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
	ros::init( argc, argv, ROS_PACKAGE_NAME, ros::init_options::NoSigintHandler );
	ros::NodeHandle nh, local_nh("~");

	bool external_trigger = false;
	float frame_rate, publish_rate, aoi_ratio;
	int camera_id, master_gain, image_format, timeout_ms;
	std::string camera_name, color_mode, image_topic;

	local_nh.param<int>( "camera_id", camera_id, 0 );
	local_nh.param<int>( "master_gain", master_gain, 0 );
	local_nh.param<int>( "image_format", image_format, 20 );
	local_nh.param<int>( "timeout_ms", timeout_ms, 100 );
	local_nh.param<bool>( "external_trigger", external_trigger, false );
	local_nh.param<float>( "frame_rate", frame_rate, 30 );
	local_nh.param<float>( "publish_rate", publish_rate, frame_rate );
	local_nh.param<float>( "aoi_ratio", aoi_ratio, 1.0 );
	local_nh.param<std::string>( "camera_name", camera_name, "camera" );
	local_nh.param<std::string>( "image_topic", image_topic, "image_raw" );
	local_nh.param<std::string>( "color_mode", color_mode, "mono8" );

	image_transport::ImageTransport it(nh);
	auto pub = it.advertiseCamera( camera_name + "/" + image_topic, 1 );

	ros::Rate spinner(publish_rate);

	auto available_cameras = ueye::Camera::get_camera_list();
	ROS_INFO("found %lu available cameras", available_cameras.size() );

	for ( auto cam : available_cameras )
		ROS_INFO("id=%d serial='%s' model='%s'", cam.dwCameraID, cam.SerNo, cam.Model );

	auto camera_info = std::find_if( available_cameras.begin(), available_cameras.end(), 
		[&camera_id]( const UEYE_CAMERA_INFO& cam_info ) { return cam_info.dwCameraID == (DWORD)camera_id; } );
	
	if ( camera_info == available_cameras.end() ) 
		{ ROS_ERROR("invalid camera id"); return 0; }
	
	ueye::Camera camera( *camera_info, image_format, frame_rate, color_mode, aoi_ratio );

	camera.set_master_gain( master_gain );
	camera.start_capture( external_trigger );

	while ( ros::ok() )
	{
		const ueye::CameraFrame* frame = camera.get_frame(timeout_ms);
		if (frame) pub.publish( frame->get_image(), camera.get_info() );

		ros::spinOnce();
		spinner.sleep();
	}
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

