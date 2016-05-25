#ifndef UEYE_DRIVER_CAMERA_H
#define UEYE_DRIVER_CAMERA_H

#include <string>
#include <memory>
#include <cstdint>

#include <ros/time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

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
    
    CameraFrame( Camera& camera, int cols, int rows, const std::string& encoding );
    virtual ~CameraFrame();

    ros::Time get_timestamp() const;
    void update_timestamp() { image_.header.stamp = get_timestamp(); }
    const sensor_msgs::Image& get_image() const { return image_; }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

enum RunMode { RunMode_FreeRun, 
               RunMode_ExternalTrigger };

enum BinningMode { BinningMode_Off=0, 
                    BinningMode_2X,
                    BinningMode_4X,
                    BinningMode_8X,
                    BinningMode_16X,
                    BinningMode_End };

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/**
 * @class Camera
 * uEye camera object
 */

class Camera
{
    friend class CameraFrame;

    int handle_;
    std::shared_ptr<CameraFrame> frame_;
    sensor_msgs::CameraInfo info_;

protected:

    uint16_t get_binning_rate() const;
    uint32_t get_buffer_pitch() const;
    float get_sensor_scaling() const;
    
public:

    /**
     * Constructor
     * @param[in] camera_id ID in [1-254] of camera to connect to (0: first available camera)
     * @param[in] aoi_width width in pixels of the requested Area Of Interest (AOI)
     * @param[in] aoi_height height in pixels of the requested Area Of Interest (AOI)
     * @param[in] encoding ROS color encoding (see sensor_msgs/image_encodings.h)
     * @param[in] frame_rate desired frame rate
     * @param[in] binning_mode level of binning (to reduce image size without scaling down AOI)
     * @throws std::exception if error occurs
     */
    Camera( int camera_id, uint16_t aoi_width, uint16_t aoi_height, 
                const std::string& encoding, float frame_rate, BinningMode binning_mode );
                
    virtual ~Camera();
    
    void start_capture( RunMode mode );
    bool is_capturing() const;
    bool set_master_gain( uint8_t master_gain );
    
    const sensor_msgs::CameraInfo& get_info() const { return info_; }
    
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
