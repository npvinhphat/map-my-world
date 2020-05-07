#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <utility>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO("Driving the robot with argument lin_x:%1.2f, ang_z:%1.2f", (float)lin_x, (float)ang_z);
    
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service command_robot");
    }
}

void move_left()
{
    ROS_INFO("Moving left");
    drive_robot(0.0f, 0.1f);
}

void move_right()
{
    ROS_INFO("Moving right");
    drive_robot(0.0f, -0.1f);
}

void move_forward()
{
    ROS_INFO("Moving forward");
    drive_robot(0.1f, 0.0f);
}

void stop()
{
    ROS_INFO("Stopping");
    drive_robot(0.0f, 0.0f);
}

// Find the middle white pixel in an image, done by calculate the rolling average
// of white pixels inside the image. If there is no such white pixels, return
// a pair of [-1, -1]
std::pair<int, int> find_middle_white_pixel(const sensor_msgs::Image &img)
{
    int white_pixel = 255;
    bool contain_white_pixel = false;
    int width_sum = 0;
    int height_sum = 0;
    int count = 0;
    for (int i = 0; i < img.height; i++) {
	for (int j = 0; j < img.width; j++) {
	    if (img.data[i * img.width + j] == white_pixel) {
		contain_white_pixel = true;
		width_sum += j;
		height_sum += i;
		count++;
	    }	    
	}
    }

    if (!contain_white_pixel) return std::make_pair(-1, -1);
    return std::make_pair(height_sum / count, width_sum / count);
}

// Return -1 if pixel is left, 1 if pixel is right, otherwise 0
int is_pixel_left_or_right(const std::pair<int, int> &pixel, const int image_width)
{
    if (pixel.second < image_width / 3) return -1;
    if (pixel.second > image_width * 2 / 3) return 1;
    return 0;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // Loop through each pixel in the image and check if there's a bright white one
    std::pair<int, int> middle_white_pixel = find_middle_white_pixel(img);
    ROS_INFO("Position of middle white pixel: h=%d, w=%d", middle_white_pixel.first, middle_white_pixel.second);

    // Then, identify if this pixel falls in the left, mid, or right side of the image
    if (middle_white_pixel.first != -1 && middle_white_pixel.second != -1) {
	// Depending on the white ball position, call the drive_bot function and pass velocities to it
	switch (is_pixel_left_or_right(middle_white_pixel, img.width)) {
	    case -1: // LEFT
		move_left();
		break;
	    case 1: // RIGHT
		move_right();
		break;
            default: // MIDDLE
		move_forward();
		break;		
	}
    } else {
	// Request a stop when there's no white ball seen by the camera
	stop();
    }
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
