#ifndef UEYE_DRIVER_CAMERA_H
#define UEYE_DRIVER_CAMERA_H

#include <string>
#include <memory>
#include <cstdint>

#include <ros/time.h>

#include <sensor_msgs/Image.h>

// Driver for IDS uEye camera
// @see https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual/index.html

namespace ueye
{

class Camera; // forward declaration

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @class CameraFrame
 * Holds a camera frame and the corresponding sensor_msgs::Image object
 */
 
class CameraFrame
{
    Camera& camera_;
    int buffer_id_;
    sensor_msgs::Image image_;
    
public:
    
    CameraFrame( Camera& camera, int image_width, int image_height, const std::string& color_mode );
    virtual ~CameraFrame();

    ros::Time get_timestamp() const;
    void update_timestamp() { image_.header.stamp = get_timestamp(); }
    const sensor_msgs::Image& get_image() const { return image_; }
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

    int camera_id_;
    std::shared_ptr<CameraFrame> frame_;
    
public:
    
    /**
     * Constructor
     * @param[in] camera_id ID in [1-254] of camera to connect to (0: first available camera)
     * @param[in] format_id https://en.ids-imaging.com/manuals/uEye_SDK/EN/uEye_Manual_4.80.2/is_imageformat.html
     * @param[in] frame_rate desired frame rate
     * @param[in] encoding ROS color encoding (see sensor_msgs/image_encodings.h)
     * @throws std::exception if error occurs
     */
    Camera( int camera_id, int32_t format_id, float frame_rate, const std::string& color_mode );
                
    virtual ~Camera();
    
    bool set_master_gain( uint8_t master_gain );
    void start_capture( bool external_trigger );
    
    /**
     * Poll next frame
     * @param[in] timeout_ms time in milliseconds to wait for a frame event
     * @returns pointer to the new frame or NULL if no frame was retrieved
     * @throws std::exception if error occurs
     */
    const CameraFrame* get_frame( int timeout_ms );
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

} // namespace ueye

#endif // UEYE_DRIVER_CAMERA_H
