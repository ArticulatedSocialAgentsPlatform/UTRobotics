// ROS includes
#include <ros/ros.h>
#include <signal.h>

// Dynamic reconfiguration includes
#include <dynamic_reconfigure/server.h>
#include <ram_motion_detection/RamMotionDetectionConfig.h>

// Includes for raspicam
#include <raspicam/raspicam_cv.h>

// OpenCV includes
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Messages
#include <ram_input_msgs/Saliency.h>

// Defines used in the code
#define _ROS_RATE 15
#define _CAMERA_HEIGHT 240
#define _CAMERA_WIDTH  320
#define _CAMERA_AREA ((_CAMERA_HEIGHT * _CAMERA_WIDTH) / 10000)
#define _CAMERA_EXPOSURE 50
#define _SALIENCY_ID "motion"

//#define SHOW_FPS

double REFERENCE_WEIGHT = 0.95; // Weight of current reference frame
double FRAME_WEIGHT = 1.0 - REFERENCE_WEIGHT; // Weight of new image
int GAUSSIAN_SIZE = 9;
int MIN_AREA = 500;

// Show image output or not
bool SHOW_IMAGES = false;
bool SHOW_MOTION = false;
bool SHOW_CONTOURS = false;

int morph_operator = 2;
int morph_elem = 0;
int morph_size = 7;
int const max_operator = 4;
int const max_elem = 2;
int const max_kernel_size = 21;

int operation;
cv::Mat *kernel = new cv::Mat();

void Morphology_Operations(int, void *);
float scaleSize(double size, int scaleFactor, double multiplier);

const char *camera_window = "Camera";
const char *background_window = "Background";
const char *motion_window = "Motion";
const char *contours_window = "Contours";

/**
 * Sigint handler to close all current windows
 */
void mySigintHandler(int sig) {
    // All the default sigint handler does is call shutdown()
    ros::shutdown();

    // Close all windows
    cv::destroyAllWindows();
    cv::waitKey(1);
}

/**
 * Reconfiguration callback
 */
void _reconfigureCallback(ram_motion_detection::RamMotionDetectionConfig &config, uint32_t level) {
    REFERENCE_WEIGHT = config.reference_weight;
    FRAME_WEIGHT = 1.0 - config.reference_weight;
    GAUSSIAN_SIZE = config.gaussian_size;
    MIN_AREA = config.min_area;

    if (!SHOW_IMAGES && config.show_images) {
        // Open image previes
        cv::namedWindow(camera_window, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(background_window, cv::WINDOW_AUTOSIZE);
    }

    if (!SHOW_CONTOURS && config.show_contours) {
        cv::namedWindow(contours_window, cv::WINDOW_AUTOSIZE);
    }

    if (!SHOW_MOTION && config.show_motion) {
        /// Create window
        cv::namedWindow(motion_window, cv::WINDOW_AUTOSIZE);
        /// Create Trackbar to select Morphology operation
        cv::createTrackbar("Operator:\n 0: Opening - 1: Closing \n 2: Gradient - 3: Top Hat \n 4: Black Hat", motion_window,
                           &morph_operator, max_operator, Morphology_Operations);
        /// Create Trackbar to select kernel type
        cv::createTrackbar("Element:\n 0: Rect - 1: Cross - 2: Ellipse", motion_window,
                           &morph_elem, max_elem, Morphology_Operations);
        /// Create Trackbar to choose kernel size
        cv::createTrackbar("Kernel size:\n 2n +1", motion_window,
                           &morph_size, max_kernel_size, Morphology_Operations);
    }

    SHOW_IMAGES = config.show_images;
    SHOW_CONTOURS = config.show_contours;
    SHOW_MOTION = config.show_motion;
}

/**
 * Main method, started on run
 */
int main(int argc, char **argv) {
    // Initialize the ROS system.
    ros::init(argc, argv, "ram_motion_detection_node");

    // Establish this program as a ROS node.
    ros::NodeHandle nh;

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    // Set loop rate
    ros::Rate loop_rate(_ROS_RATE);

    // Configure opencv windows
//    cv::startWindowThread();

    // Configure reconfigure server
    dynamic_reconfigure::Server <ram_motion_detection::RamMotionDetectionConfig> reconfigureServer;
    dynamic_reconfigure::Server<ram_motion_detection::RamMotionDetectionConfig>::CallbackType reconfigureCallback;
    reconfigureCallback = boost::bind(&_reconfigureCallback, _1, _2);
    reconfigureServer.setCallback(reconfigureCallback);

    // Create publisher
    ros::Publisher SaliencyPublisher = nh.advertise<ram_input_msgs::Saliency>("/ram/animation/hmmm/saliency", _ROS_RATE);
    std::vector <float> locationsX, locationsY;
    std::vector <uint16_t> weights;

    // Initialize camera
    raspicam::RaspiCam_Cv rpiCam;
    rpiCam.set(CV_CAP_PROP_FRAME_WIDTH, _CAMERA_WIDTH);
    rpiCam.set(CV_CAP_PROP_FRAME_HEIGHT, _CAMERA_HEIGHT);
    rpiCam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    rpiCam.set(CV_CAP_PROP_EXPOSURE, _CAMERA_EXPOSURE);
    cv::Mat image;

    // Initialize motion detection + background mixing
    cv::Mat referenceFrame, blurredFrame, motion, diff, threshold, contoursFrame;
    std::vector <std::vector<cv::Point>> contours;
    cv::Rect boundingRect;
    cv::Moments moments;
    cv::Mat *nullMat = new cv::Mat();
    cv::Point *point = new cv::Point(-1, -1);
    cv::Size *blurSize = new cv::Size(GAUSSIAN_SIZE, GAUSSIAN_SIZE);
    int contourArea;
    int x = -1, y = -1, i;

    // Open camera
    ROS_DEBUG("Opening camera...");
    if (!rpiCam.open()) {
        ROS_FATAL("Error opening the camera!");
        return -1;
    }

    // Init the morphology
    Morphology_Operations(0, 0);

    // Get reference frame
    rpiCam.grab();
    rpiCam.retrieve(referenceFrame);
    cv::GaussianBlur(referenceFrame, referenceFrame, *blurSize, 0);

    // Init frame to prevent reconfiguration errors
    contoursFrame = referenceFrame.clone();

    // Run as long a node is running
    int count = 0;
    time_t newTime, t = time(0);
    while (ros::ok()) {
        // Grab new image from camera
        rpiCam.grab();
        rpiCam.retrieve(image);
        cv::GaussianBlur(image, blurredFrame, *blurSize, 0);

        // Compute the absolute difference between the current frame and first frame
        cv::absdiff(referenceFrame, blurredFrame, diff);
        cv::threshold(diff, threshold, 25, 255, cv::THRESH_BINARY);

        // Morphology transfer the motion
#ifdef USE_DILATE
        cv::dilate(threshold, motion, *nullMat, *point, 2);
#else
        cv::morphologyEx(threshold, motion, operation, *kernel);
#endif

        // Calculate contours
        cv::findContours(motion.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (SHOW_CONTOURS) {
            contoursFrame = image.clone();
        }

        // Reset vectors en loop contours
        locationsX.clear();
        locationsY.clear();
        weights.clear();
        for (i = 0; i < contours.size(); i++) {
            contourArea = cv::contourArea(contours[i]);
            if (contourArea < MIN_AREA) {
                continue;
            }

            // Map to between 0 and 10000
            contourArea = (int) std::round(contourArea / _CAMERA_AREA);

            // Add to message
            moments = cv::moments(contours[i]);
            weights.push_back(contourArea);

            // Scale locations to [-1,1]
            locationsX.push_back(scaleSize(moments.m10 / moments.m00, _CAMERA_WIDTH, 0.75));
            locationsY.push_back(scaleSize(moments.m01 / moments.m00, _CAMERA_HEIGHT, 1));

            if (SHOW_CONTOURS) {
                boundingRect = cv::boundingRect(contours[i]);
                cv::rectangle(contoursFrame, boundingRect, 255, 2);
            }
        }

        if (SHOW_CONTOURS) {
            cv::imshow(contours_window, contoursFrame);
            cv::waitKey(1);
        } else {
            cv::destroyWindow(contours_window);
            cv::waitKey(1);
        }

        ram_input_msgs::Saliency SaliencyMessage;
        SaliencyMessage.locationsX = locationsX;
        SaliencyMessage.locationsY = locationsY;
        SaliencyMessage.weights = weights;
        SaliencyMessage.inputId = _SALIENCY_ID;
        SaliencyMessage.deliberate = 0;
        SaliencyPublisher.publish(SaliencyMessage);

        // Update reference frame
        cv::addWeighted(referenceFrame, REFERENCE_WEIGHT, blurredFrame, FRAME_WEIGHT, 0.0, referenceFrame);

#ifdef SHOW_FPS
        // FPS detection
        newTime = time(0);
        if (t == newTime) {
            count++;
        } else if ((t + 1) == newTime) {
            std::cout<<'\r'<<std::setw(2)<<std::setfill(' ')<<count<<" FPS. L: "
            <<std::setw(3)<<std::setfill(' ')<<x<<", "
            <<std::setw(3)<<std::setfill(' ')<<y<<std::flush;
            count = 1;
            t = newTime;
        } else {
            std::cout<<'\r';
            t = newTime;
        }
#endif

        if (SHOW_IMAGES) {
            // Show results
            cv::imshow(camera_window, image);
            cv::waitKey(1);
            cv::imshow(background_window, referenceFrame);
            cv::waitKey(1);
        } else {
            cv::destroyWindow(camera_window);
            cv::destroyWindow(background_window);
            cv::waitKey(1);
        }

        if (SHOW_MOTION) {
            cv::imshow(motion_window, motion);
            cv::waitKey(1);
        } else {
            cv::destroyWindow(motion_window);
            cv::waitKey(1);
        }

        // Ros rate
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Graceful shutdown of camera
    ROS_DEBUG("Stopping camera...");
    rpiCam.release();
}

/**
 * Scale the resolution down to a [-1, 1]-range
 * @param size
 * @param scaleFactor
 * @param multiplier
 * @return
 */
float scaleSize(double size, int scaleFactor, double multiplier){
    float scaling = (float)(scaleFactor / 2);
    return ((size - scaling) / scaling) * multiplier;
}

/**
 * Create the morph operation
 */
void Morphology_Operations(int, void *) {
    // Since MORPH_X : 2,3,4,5 and 6
    operation = morph_operator + 2;
    *kernel = cv::getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                        cv::Point(morph_size, morph_size));
}
