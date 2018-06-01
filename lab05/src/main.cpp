#include "ardrone/ardrone.h"
#include <opencv2/aruco.hpp>
#include <string>

// Parameter for calibration pattern
#define PAT_ROWS (6)      // Rows of pattern
#define PAT_COLS (9)      // Columns of pattern
#define CHESS_SIZE (22.0) // Size of a pattern [mm]

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open())
    {
        std::cout << "Failed to initialize." << std::endl;
        return -1;
    }

    // Battery
    std::cout << "Battery = " << ardrone.getBatteryPercentage() << "[%]" << std::endl;

    // Instructions
    std::cout << "***************************************" << std::endl;
    std::cout << "*       CV Drone sample program       *" << std::endl;
    std::cout << "*           - How to play -           *" << std::endl;
    std::cout << "***************************************" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Controls -                        *" << std::endl;
    std::cout << "*    'Space' -- Takeoff/Landing       *" << std::endl;
    std::cout << "*    'Up'    -- Move forward          *" << std::endl;
    std::cout << "*    'Down'  -- Move backward         *" << std::endl;
    std::cout << "*    'Left'  -- Turn left             *" << std::endl;
    std::cout << "*    'Right' -- Turn right            *" << std::endl;
    std::cout << "*    'Q'     -- Move upward           *" << std::endl;
    std::cout << "*    'A'     -- Move downward         *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "* - Others -                          *" << std::endl;
    std::cout << "*    'C'     -- Change camera         *" << std::endl;
    std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
    std::cout << "*                                     *" << std::endl;
    std::cout << "***************************************" << std::endl;

    // Calibration loading
    // Open XML file
    std::string filename("/Users/Henry/Downloads/camera.xml");
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    // Load camera parameters
    cv::Mat cameraMatrix, distCoeffs;
    fs["intrinsic"] >> cameraMatrix;
    fs["distortion"] >> distCoeffs;

    // Create undistort map
    cv::Mat mapx, mapy;
    cv::Mat frame = ardrone.getImage();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);

    // Marker detection variable
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    std::vector<int> ids;
    std::vector< std::vector< cv::Point2f > > corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
    float markerLength = 7.05;

    

    while (1)
    {
        // Key input
        int key = cv::waitKey(30);
        if (key == 0x1b)
            break;

        // Get an image
        cv::Mat image = ardrone.getImage();

        // Take off / Landing
        if (key == ' ')
        {
            if (ardrone.onGround())
                ardrone.takeoff();
            else
                ardrone.landing();
        }

        // Move
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
        if (key == 'i' || key == CV_VK_UP)
            vx = 1.0;
        if (key == 'k' || key == CV_VK_DOWN)
            vx = -1.0;
        if (key == 'u' || key == CV_VK_LEFT)
            vr = 1.0;
        if (key == 'o' || key == CV_VK_RIGHT)
            vr = -1.0;
        if (key == 'j')
            vy = 1.0;
        if (key == 'l')
            vy = -1.0;
        if (key == 'q')
            vz = 1.0;
        if (key == 'a')
            vz = -1.0;
        ardrone.move3D(vx, vy, vz, vr);

        // Change camera
        static int mode = 0;
        if (key == 'c')
            ardrone.setCamera(++mode % 4);

        // Undistort
        cv::remap(image, image, mapx, mapy, cv::INTER_LINEAR);
        // Detect marker
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        if (ids.size() > 0)
        {
            // Draw out the boarder of marker
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            // Pose estimation
            cv::aruco::estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
            // Draw
            for (int i = 0; i < ids.size(); i++)
            {
                cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 7.5);
                char str[100];
                sprintf(str, "%g %g %g", tvecs[i][0], tvecs[i][1], tvecs[i][2]);
                cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
            }
        }

        // Display the image
        cv::imshow("camera", image);
    }

    // See you
    ardrone.close();

    return 0;
}