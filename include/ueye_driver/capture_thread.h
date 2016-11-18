#ifndef UEYE_CAPTURE_THREAD_H
#define UEYE_CAPTURE_THREAD_H

#include "ueye_camera.h"

#include <thread>
#include <memory>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

namespace ueye
{
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct CaptureThread
{
    Camera& camera_;
    bool capture_alive_;
    std::thread thread_;
    
    CaptureThread( Camera& camera ): camera_(camera), capture_alive_(false) {}
    virtual ~CaptureThread() { stop(); }

    void start( image_transport::CameraPublisher camera_pub, ros::Rate publish_rate ) 
    {
        thread_ = std::thread( bind( &CaptureThread::loop, this, camera_pub, publish_rate ) );
    }
    
    void loop( image_transport::CameraPublisher camera_pub, ros::Rate publish_rate )
    {
        ROS_INFO("starting capture thread");
        camera_.start_capture();
        capture_alive_ = true;

        while ( capture_alive_ && ros::ok() )
        {
	        const ueye::CameraFrame* frame = camera_.get_frame();
	        if (frame) camera_pub.publish( frame->get_image(), camera_.get_info() );
	        publish_rate.sleep();
        }
    }

    void stop() 
    { 
        capture_alive_ = false;

        if ( thread_.joinable() ) thread_.join();
        thread_ = std::thread();

        ROS_INFO("stopped capture thread.");
    }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}	// namespace ueye

#endif	// UEYE_CAPTURE_THREAD_H
