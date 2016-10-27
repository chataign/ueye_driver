#ifndef UEYE_DRIVER_CAMERA_H
#define UEYE_DRIVER_CAMERA_H

#include <string>
#include <memory>
#include <cstdint>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <ros/time.h>
#include <ueye.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Driver for IDS uEye camera
// @see https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html

namespace ueye
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct DeviceSettings
{
    int pixel_clock; // pixel clock in MHz
    std::string color_mode; // ROS color encoding (see sensor_msgs/image_encodings.h)
    std::string frame_id; // frame ID in ROS image message
    double frame_rate; // desired frame rate in Hz
    IS_RECT aoi_rect; // Area Of Interest (AOI) rectangle
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct CaptureSettings
{
    bool external_trigger, hardware_gamma;
    int timeout_ms, master_gain, blacklevel, gamma;
    double exposure;
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class Camera;	// forward declaration

/**
 * @class CameraFrame
 * Holds a camera frame and the corresponding sensor_msgs::Image object
 */

class CameraFrame
{
	const int camera_id_;
	int buffer_id_;
	sensor_msgs::Image::Ptr image_;

public:

	CameraFrame( Camera& camera, const DeviceSettings& device_settings );

	virtual ~CameraFrame();

	ros::Time get_timestamp() const;
	void update_timestamp() { image_->header.stamp = get_timestamp(); }
	sensor_msgs::Image::ConstPtr get_image() const { return image_; }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @class Camera
 * uEye camera object
 */

class Camera
{
	friend class CameraFrame;

	const UEYE_CAMERA_INFO device_info_;
	sensor_msgs::CameraInfo::Ptr camera_info_;
	std::shared_ptr<CameraFrame> frame_;

public:

	/**
	 * Constructor
	 * @param[in] camera_id ID in [1-254] of camera to connect to (0: first available camera)
	 * @param[in] frame_rate desired frame rate
	 * @param[in] encoding ROS color encoding (see sensor_msgs/image_encodings.h)
	 * @param[in] pixel_clock pixel clock in MHz
	 * @param[in] aoi_rect Area Of Interest (AOI) rectangle
	 * @throws std::exception if connection to camera fails
	 */
	Camera( const UEYE_CAMERA_INFO& device_info, const DeviceSettings& device_settings );
			
	virtual ~Camera();

    sensor_msgs::CameraInfo::ConstPtr get_info() const { return camera_info_; }
	bool set_master_gain( uint8_t master_gain ); // in [0,100]
	bool set_exposure( double exposure );
	bool set_blacklevel( int blacklevel );
	bool set_gamma( int gamma );
	bool set_hardware_gamma();
	
	void start_capture( bool external_trigger );
    void start_capture( const CaptureSettings& settings );
    
	/**
	 * Poll next frame
	 * @param[in] timeout_ms time in milliseconds to wait for a frame event
	 * @returns pointer to the new frame or NULL if no frame was retrieved
	 * @throws std::exception if error occurs
	 */
	const CameraFrame* get_frame( int timeout_ms );
	
	/*
	 * Get information about all connected uEye cameras
	 * https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual_4.80.2/is_getcameralist.html
	 */
	static std::vector<UEYE_CAMERA_INFO> get_camera_list();
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
}	// namespace ueye

#endif	// UEYE_DRIVER_CAMERA_H
