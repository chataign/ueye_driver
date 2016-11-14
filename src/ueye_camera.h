#ifndef UEYE_DRIVER_CAMERA_H
#define UEYE_DRIVER_CAMERA_H

#include <string>
#include <memory>
#include <cstdint>
#include <mutex>

#include <ros/ros.h>
#include <opencv/cv.h>
#include <ros/time.h>
#include <ueye.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ueye_driver/CaptureConfig.h>
#include <dynamic_reconfigure/server.h>

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
    typedef dynamic_reconfigure::Server<CaptureConfig> reconfigure_server_t;
    
	const UEYE_CAMERA_INFO device_info_;
	sensor_msgs::CameraInfo::Ptr camera_info_;
    shared_ptr<reconfigure_server_t> reconfigure_server_;
	std::shared_ptr<CameraFrame> frame_;
	bool configured_;
	int timeout_ms_;
	std::mutex mutex_;

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

    void enable_reconfigure( ros::NodeHandle& reconfigure_nh );
    void reconfigure_callback( CaptureConfig &config, uint32_t level ) { apply_config(config); }
    
	void start_capture();
    void apply_config( const CaptureConfig& config );
    
	/**
	 * Poll next frame
	 * @returns pointer to the new frame or NULL if no frame was retrieved
	 * @throws std::exception if error occurs
	 */
	const CameraFrame* get_frame();
	
	static bool set_master_gain( DWORD camera_id, uint8_t master_gain ); // in [0,100]
	static bool set_exposure( DWORD camera_id, double exposure );
	static bool set_blacklevel( DWORD camera_id, int blacklevel );
	static bool set_gamma( DWORD camera_id, int gamma );
	static bool set_hardware_gamma( DWORD camera_id );
	
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
