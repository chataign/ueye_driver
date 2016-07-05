#include <ros/ros.h>
#include <ros/names.h>

#include <memory>
#include <opencv/cv.h>

#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include "ueye_camera.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
	ros::init( argc, argv, ROS_PACKAGE_NAME );
	ros::NodeHandle nh, local_nh("~");

	bool external_trigger = false, hardware_gamma = false;
	int queue_size, master_gain, timeout_ms, pixel_clock, blacklevel, gamma;
	std::string serial_no, camera_name, color_mode, image_topic, frame_id;
	double exposure, frame_rate, publish_rate;
	IS_RECT aoi_rect;

	local_nh.param<int>( "queue_size", queue_size, 10 );
	local_nh.param<int>( "pixel_clock", pixel_clock, 84 );
	local_nh.param<double>( "exposure", exposure, 40 );
	local_nh.param<int>( "blacklevel", blacklevel, 90 );
	local_nh.param<int>( "gamma", gamma, 100 );
	local_nh.param<bool>( "hardware_gamma", hardware_gamma, false );
	local_nh.param<int>( "master_gain", master_gain, 0 );
	local_nh.param<int>( "timeout_ms", timeout_ms, 100 );
	local_nh.param<bool>( "external_trigger", external_trigger, false );
	local_nh.param<double>( "frame_rate", frame_rate, 25 );
	local_nh.param<double>( "publish_rate", publish_rate, frame_rate );
	local_nh.param<int>( "aoi_width", aoi_rect.s32Width, 1080 );
	local_nh.param<int>( "aoi_height", aoi_rect.s32Height, 1080 );
	local_nh.param<int>( "aoi_x", aoi_rect.s32X, 20 );
	local_nh.param<int>( "aoi_y", aoi_rect.s32Y, 50 );
	local_nh.param<std::string>( "serial_no", serial_no, "" );
	local_nh.param<std::string>( "frame_id", frame_id, "base_link" );
	local_nh.param<std::string>( "camera_name", camera_name, "camera" );
	local_nh.param<std::string>( "image_topic", image_topic, "image_raw" );
	local_nh.param<std::string>( "color_mode", color_mode, "mono8" );

	image_transport::ImageTransport it(nh);
	auto pub = it.advertiseCamera( ros::names::clean( camera_name + "/" + image_topic ), queue_size );

	auto available_cameras = ueye::Camera::get_camera_list();

	ROS_INFO("found %lu available cameras, connecting to camera serial='%s'", 
		available_cameras.size(), serial_no.c_str() );

	for ( auto cam : available_cameras )
		ROS_INFO("id=%d serial='%s' model='%s'", cam.dwCameraID, cam.SerNo, cam.Model );

	auto camera_info = std::find_if( available_cameras.begin(), available_cameras.end(), 
		[&serial_no]( const UEYE_CAMERA_INFO& cam_info ) { return std::string(cam_info.SerNo) == serial_no; } );
	
	if ( camera_info == available_cameras.end() ) 
		{ ROS_ERROR("invalid camera id"); return 0; }
	
	ueye::Camera camera( *camera_info, frame_id, frame_rate, color_mode, pixel_clock, aoi_rect ) ;

	ROS_INFO("Publish_rate set to %.1f \t [equals frame actual frame rate set]", publish_rate);
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

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

