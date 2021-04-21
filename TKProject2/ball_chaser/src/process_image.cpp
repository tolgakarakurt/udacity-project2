#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget req;
    
    //ROS_INFO_STREAM("Moving the arm to the center");

    req.request.linear_x = lin_x;
    req.request.angular_z = ang_z;
    // Call the safe_move service and pass the requested joint angles
    if (!client.call(req))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    enum Pos {LEFT, MIDDLE, RIGHT, NOTSEEN};
    int white_pixel = 255;
    int scanned_pixel = 0;
    int detected_pixel = 0;
    int ball_volume = 0;
    int ball_position = 0;

    int left_marker = img.width / 3;    
    int forward_marker = 2 * img.width / 3;
    int right_marker = img.width;

    bool ball_detected = false;
    
    float linear_x;
    float angular_z;
    float go_nowhere = 0.0;
    float go_slow = 0.1;
    float go_fast = 0.5;

    int ball_refpoints [3];         // [top_edge, center, bottom_edge]
    int ball_ref_axis = 0;          // ball centerline vertical
    int hysteresis = 10;
/* 
                       col
     ___________________._______________________________________________...
    |                   .
    |                   .
    |                   .
row |------------------_-_--------------------top_edge
    |               *       *
    |             *           *
    |            *  white_ball *
    |            *      +------*--------------center
    |            *             *
    |             *           *
    |               *  _ _  *
    |------------------ - --------------------bottom_edge
    |
    .
    .
    .
*/

    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height; i++)
    {
        ball_volume = detected_pixel;
        for (int j = 0; j < img.step; ++j)
        {   
            scanned_pixel = (i*img.step)+j;
            if (img.data[scanned_pixel] == white_pixel)
            {   
                detected_pixel++;                   // number of detected white_pixels   
                if (detected_pixel == 1)
                {
                    ball_refpoints[0] = i;          // ball top_edge row
                    ball_ref_axis = j/3;            // ball top_edge col
                }
                ball_detected = true;
                break;
            }
		}
        if (detected_pixel > 0 && ball_volume == detected_pixel){
            ball_refpoints[2] = i;                  // ball bottom_edge row
            ball_refpoints[1] = ball_refpoints[0] + (int)((ball_refpoints[2] - ball_refpoints[0]) / 2); // ball center row
            ball_detected = true;
            ball_volume = detected_pixel;           // ball volume
            break;
        }
    }

    // Then, identify if this pixel falls in the left, mid, or right side of the image
    if (ball_detected)
    {
        if (ball_ref_axis < left_marker)
        {
            ball_position = LEFT;
        }  
        else if (ball_ref_axis >= left_marker && ball_ref_axis < forward_marker)
        {
            ball_position = MIDDLE;
        }
        else if (ball_ref_axis >= forward_marker)
        {
            ball_position = RIGHT;
        }
    }
  	else 
    {
        // Stop if the ball is not presented to the camera
        ball_position = NOTSEEN; 
        //ROS_INFO("NOTSEEN");
    }

    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    switch (ball_position)
    {
        case LEFT:
            linear_x = go_nowhere;
            angular_z = go_fast;
            break;
        case MIDDLE:
            if(ball_ref_axis >= img.width/2 - hysteresis && ball_ref_axis <= img.width/2 + hysteresis)
            {
                linear_x = go_fast; 
                angular_z = go_nowhere;
                //ROS_INFO("IN_HYSTERESIS");
            }
            else if (ball_ref_axis < img.width/2 - hysteresis)
            {
                linear_x = go_nowhere;
                angular_z = go_slow;
                //ROS_INFO("OUT_HYSTERESIS - LEFT");
            }
            else if (ball_ref_axis > img.width/2 + hysteresis)
            {
                linear_x = go_nowhere;
                angular_z = -1 * go_slow;
                //ROS_INFO("OUT_HYSTERESIS - RIGHT");
            }
            break;
        case RIGHT:
            linear_x = go_nowhere;
            angular_z = -1 * go_fast;
            break;
        // Request a stop when there's no white ball seen by the camera
        default: //NOTSEEN
            linear_x = go_nowhere;
            angular_z = go_nowhere;
            break;
    }   
    drive_robot(linear_x, angular_z);	
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

