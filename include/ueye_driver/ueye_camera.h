#ifndef UEYE_DRIVER_CAMERA_H
#define UEYE_DRIVER_CAMERA_H

#include <string>
#include <memory>
#include <cstdint>
#include <mutex>

#include <ros/ros.h>
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
    std::string color_mode; // ROS color encoding (see sensor_msgs/image_encodings.h)
    std::string frame_id;   // frame ID in ROS image message
    IS_RECT aoi_rect;       // Area Of Interest (AOI) rectangle
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
	const int camera_id_; // camera ID from UEYE_CAMERA_INFO
	int buffer_id_; // image buffer ID
	sensor_msgs::Image::Ptr image_; // image buffer

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
	 * @throws std::exception if connection to camera fails
	 */
	Camera( const UEYE_CAMERA_INFO& device_info, const DeviceSettings& device_settings );
			
	virtual ~Camera();

    sensor_msgs::CameraInfo::ConstPtr get_info() const { return camera_info_; }

    // enable dynamic_reconfigure server, to allow live 
    // configuration of the camera settings using rqt_reconfigure
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
	
	IS_RECT get_aoi() const;
    double get_exposure() const;
    int get_blacklevel() const;
    INT get_gamma() const;
    double get_framerate() const;
    bool get_gain_boost() const;
    UINT get_pixelclock() const;
    UINT get_hardware_gain() const;

    IS_RANGE_F64 get_exposure_range() const;
    IS_RANGE_S32 get_blacklevel_range() const;
    IS_RANGE_F64 get_framerate_range() const;
    IS_RANGE_S32 get_pixelclock_range() const;
    	
	bool set_aoi( const IS_RECT& aoi );
	bool set_exposure( double exposure ); // in [1,1000]
	bool set_hardware_gain( uint8_t hardware_gain ); // in [0,100]
	bool set_blacklevel( int blacklevel );
	bool set_gamma( INT gamma );
	bool set_hardware_gamma( bool enable );
	bool set_framerate( double framerate ); // Hz
	bool set_pixelclock( UINT pixelclock ); // Mhz
    bool set_gain_boost( bool enable );
	bool set_color_mode( const std::string& color_mode );
	
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
