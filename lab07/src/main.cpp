#include "ardrone/ardrone.h"
#include <opencv2/aruco.hpp>

#include "pid.hpp"
#include <cstring>

// Parameter for calibration pattern
#define PAT_ROWS (6)	  // Rows of pattern
#define PAT_COLS (9)	  // Columns of pattern
#define CHESS_SIZE (22.0) // Size of a pattern [mm]

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	// AR.Drone class
	PIDManager PID("pid.yaml");

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
	std::string filename("camera.xml");
	std::string filename_2("camera_down.xml");
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	cv::FileStorage fs_2(filename_2, cv::FileStorage::READ);

	// Load camera parameters
	cv::Mat cameraMatrix, distCoeffs;
	fs["intrinsic"] >> cameraMatrix;
	fs["distortion"] >> distCoeffs;

	// Create undistort map
	cv::Mat mapx, mapy;
	cv::Mat frame = ardrone.getImage();
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);

	// Marker detection variable
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners;
	std::vector<cv::Vec3d> rvecs, tvecs;
	float markerLength = 9.4;

	//autopolit
	Mat error = Mat(4, 1, CV_64F);
	Mat output = Mat(4, 1, CV_64F);
	//chack if steady enough to go to next id
	const int stady_const[5] = {250, 25, 5, 25, 350};
	int stady_count[5] = {0};
	bool setp_finding = true;
	bool step_upfront = false;
	bool step_id_1 = false;
	bool step_id_2 = false;
	bool step_id_3 = false;
	bool step_id_5 = false;
	bool landing = false;
	int ltcantfi = 200;
	int counter = 0;

	while (1)
	{

		// Key input
		int key = cv::waitKey(33);
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

		// Change camera
		static int mode = 0;
		if (key == 'c')
		{
			ardrone.setCamera(++mode % 4);
			// //down_camera_calibration
			// fs_2["intrinsic"] >> cameraMatrix;
			// fs_2["distortion"] >> distCoeffs;
			// // calculate undistort map
			// cv::Mat frame = ardrone.getImage();
			// cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
		}

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
			// for (int i = 0; i < ids.size(); i++)
			// {
			// 	cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 12.3);
			// 	// char str[100];
			// 	// sprintf(str, "%g %g %g", tvecs[i][0], tvecs[i][1], tvecs[i][2]);
			// 	// cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
			// }
		}

		// tvecs[i][2] -> deep
		// tvecs[i][0] -> left hand +
		// tvecs[i][1] -> height +

		// rvecs[i][2] -> rotate
		// rvecs[i][0] -> up and down
		// rvecs[i][1] -> slide

		//vz -> up and down
		//vy -> left and right slide
		//vx -> proceed and backward
		//vr -> rotate left + right -

		if (key == -1)
		{
			//flying and finding
			if (setp_finding == true && ids.size() == 0)
			{
				vr = -0.1;
			}
			else if (setp_finding == true)
			{
				if (rvecs[0][0] < 0.0)
				{
					rvecs[0][2] = -rvecs[0][2];
				}

				if (ids[0] == 1) // not only see id1, but also id1 is near image center
				{
					setp_finding = false;
					step_upfront = true;
					std::cout << "first time find marker1" << std::endl;
					vr = -2;
					counter = 0;
				}
				else
				{
					vr = -0.1; // not id1, or id1 is not near image center, keep turning
				}
			}

			counter++;
			if (!setp_finding && counter >= ltcantfi)
			{
				vx = 0;
				vy = 0;
				vz = 0;
				vr = 0;
				if (!landing)
					vr = -0.1;
				else
					vz = 0.05;
				std::cout << "I am finding next marker now\n";
			}

			//find ids
			for (int i = 0; i < ids.size(); i++)
			{
				// move forward
				if (step_upfront == true && ids[i] == 1)
				{
					counter = 0;

					std::cout << "in 1" << std::endl;
					error.at<double>(0, 0) = tvecs[i][2] - 80;
					error.at<double>(1, 0) = tvecs[i][0] + 12;
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					} else {
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					else
					{
						step_upfront = false;
						step_id_1 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = 0;
						std::cout << "ok fuck bye 1 (dis)\n";
					}

					if (stady_count[0] == stady_const[0])
					{
						step_upfront = false;
						step_id_1 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = 0;
						std::cout << "ok fuck bye 1(const)\n";
					}

					stady_count[0]++;
				}

				// id_1 and id_2 flying(using else to boost some speed)
				else if (step_id_1 == true && ids[i] == 2)
				{
					counter = 0;

					std::cout << "in 2" << std::endl;
					error.at<double>(0, 0) = tvecs[i][2] - 100;
					error.at<double>(1, 0) = tvecs[i][0];
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(3, 0)) > 0.08 ) 
					{
						vr = 1.6 * output.at<double>(3, 0);
					}
					else
					{
						vr = 0;
					}

					if (abs(error.at<double>(0, 0)) > 7 )
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					else 
					{
						stady_count[1]++;
						vx = 0;
						// step_id_1 = false;
						// step_id_3 = true;
						// vx = 0;
						// vy = 0;
						// vz = 0;
						// vr = 0;
						// std::cout << "ok fuck bye 1 (dis)\n";
					}

					

					if (stady_count[1] == stady_const[1])
					{
						step_id_1 = false;
						step_id_3 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -1;
						std::cout << "ok fuck bye 2 (dis)\n";
					}

					
				}

				else if (step_id_3 == true && ids[i] == 3)
				{

					counter = 0;
					std::cout << "in 3\n";
					error.at<double>(0, 0) = tvecs[i][2] - 100;
					error.at<double>(1, 0) = tvecs[i][0];
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(3, 0)) > 0.08)
					{
						vr = 1.6 * output.at<double>(3, 0);
					}
					else
					{
						vr = 0;
					}

					if (abs(error.at<double>(0, 0)) > 7 )
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					else
					{
						stady_count[2]++;
						vx = 0;
					}

					

					//if stady enought, then go to next step
					if (stady_count[2] == stady_const[2])
					{
						step_id_3 = false;
						step_id_5 = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = -1;
						std::cout << "ok fuck bye 3 (dis)\n";
					}

					
				}

				//flying to marker 4
				else if (step_id_5 == true && ids[i] == 4)
				{
					counter = 0;
					std::cout << "in 4\n";
					error.at<double>(0, 0) = tvecs[i][2] - 90;
					error.at<double>(1, 0) = tvecs[i][0];
					error.at<double>(2, 0) = tvecs[i][1];
					if (rvecs[i][0] < 0.0)
					{
						error.at<double>(3, 0) = -rvecs[i][2];
					}
					else
					{
						error.at<double>(3, 0) = rvecs[i][2];
					}

					PID.getCommand(error, output);

					if (abs(error.at<double>(3, 0)) > 0.08 )
					{
						vr = 1.6 * output.at<double>(3, 0);
					}
					else
					{
						vr = 0;
					}

					if (abs(error.at<double>(0, 0)) > 7)
					{
						vx = 0.01 * output.at<double>(0, 0);
					}
					else
					{
						stady_count[3]++;
						vx = 0;
					}

					

					//if stady enought, then go to next step
					if (stady_count[3] == stady_const[3])
					{
						step_id_5 = false;
						landing = true;
						vx = 0;
						vy = 0;
						vz = 0;
						vr = 0;
						ardrone.setCamera(++mode % 4);
						// //down_camera_calibration
						// fs_2["intrinsic"] >> cameraMatrix;
						// fs_2["distortion"] >> distCoeffs;
						// // calculate undistort map
						// cv::Mat frame = ardrone.getImage();
						// cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, frame.size(), CV_32FC1, mapx, mapy);
						// std::cout << "ok fuck bye 4 (dis)\n";
					}

					
				}

				// landing
				else if (landing == true && ids[i] == 5)
				{

					std::cout << "begin landing\n";
					error.at<double>(0, 0) = tvecs[i][1] + 4; // 2
					error.at<double>(1, 0) = tvecs[i][0];
					error.at<double>(2, 0) = tvecs[i][2]; // 1
					// if (rvecs[0][0] < 0.0)
					// {
					// 	error.at<double>(3, 0) = -rvecs[i][2];
					// }

					PID.getCommand(error, output);

					//if drone on the right position, then doing landing
					if (abs(tvecs[i][2] - 73) < 2)
					{
						ardrone.landing();
						landing = false;
					}

					if (abs(error.at<double>(0, 0)) > 2)
					{



						
						vx = -0.01 * output.at<double>(0, 0);
						vz = -0.008; // -0.07
					}
					else
					{
						vx = 0;
						vz = -0.1; // -0.08
					}

					vy = 0;
					vr = 0;
				}

				//vr = 0;
				//vr = -0.02 * output.at<double>(3, 0);
				char str[100];
				sprintf(str, "%5g %5g %5g", vr, tvecs[i][2], vx); //,// vz);
				cv::putText(image, str, cv::Point(20, 40), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
				//vr = -0.01 * output.at<double>(3, 0);
			}
		}
		ardrone.move3D(vx, vy, vz, vr);
		ardrone.move3D(vx, vy, vz, vr);
		ardrone.move3D(vx, vy, vz, vr);

		// Display the image
		cv::imshow("camera", image);
	}

	// See you
	ardrone.close();

	return 0;
}
