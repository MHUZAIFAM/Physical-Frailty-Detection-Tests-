#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <cmath>

using namespace std;
using namespace std::chrono;

// Declare global variables
IKinectSensor* sensor = nullptr;
IBodyFrameReader* bodyFrameReader = nullptr;
IColorFrameReader* colorFrameReader = nullptr;
ICoordinateMapper* coordinateMapper = nullptr;

HRESULT hr; // Declare hr to handle HRESULT values

// Threshold for foot raise detection
const float FOOT_RAISE_THRESHOLD_Z = 0.1f; // Depth difference
const float FOOT_RAISE_THRESHOLD_Y = 0.01f; // Height difference
const float FOOT_TOUCH_THRESHOLD = 0.05f; // Adjust this value as needed

// Timer variables
steady_clock::time_point footRaiseStart;
bool rightFootTimerActive = false;
bool leftFootTimerActive = false;
float elapsedTimeRight = 0.0f;
float elapsedTimeLeft = 0.0f;

// Function to draw text on the image
void drawText(cv::Mat& frame, const string& text, cv::Point position, cv::Scalar color, double scale = 1.0) {
    cv::putText(frame, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2);
}

// Function to stop the timer if the raised foot touches the ground
void stopTimerIfFootTouchesGround(float raisedFootY, bool& timerActive, steady_clock::time_point& startTime, float& elapsedTime) {
    // Check if the raised foot touches the ground (based on Y value)
    if (raisedFootY < 0.1f) { // Raised foot touches the ground
        if (timerActive) {
            auto footRaiseEnd = steady_clock::now();
            elapsedTime = duration_cast<seconds>(footRaiseEnd - startTime).count();
            timerActive = false;
        }
    }
}

int main() {
    // Initialize Kinect
    hr = GetDefaultKinectSensor(&sensor);
    if (FAILED(hr) || !sensor) {
        cerr << "Error: Kinect sensor not found. HRESULT: " << hr << endl;
        return -1;
    }

    hr = sensor->Open();
    if (FAILED(hr)) {
        cerr << "Error: Could not open Kinect sensor. HRESULT: " << hr << endl;
        return -1;
    }

    // Initialize coordinate mapper
    hr = sensor->get_CoordinateMapper(&coordinateMapper);
    if (FAILED(hr)) {
        cerr << "Error: Could not get coordinate mapper. HRESULT: " << hr << endl;
        sensor->Close();
        sensor->Release();
        return -1;
    }

    // Initialize body frame reader
    IBodyFrameSource* bodySource = nullptr;
    hr = sensor->get_BodyFrameSource(&bodySource);
    if (FAILED(hr) || !bodySource) {
        cerr << "Error: Body frame source not found. HRESULT: " << hr << endl;
        sensor->Close();
        sensor->Release();
        return -1;
    }

    hr = bodySource->OpenReader(&bodyFrameReader);
    if (FAILED(hr)) {
        cerr << "Error: Could not open body frame reader. HRESULT: " << hr << endl;
        bodySource->Release();
        sensor->Close();
        sensor->Release();
        return -1;
    }

    bodySource->Release();

    // Initialize color frame reader
    IColorFrameSource* colorFrameSource = nullptr;
    hr = sensor->get_ColorFrameSource(&colorFrameSource);
    if (FAILED(hr)) {
        cerr << "Error: Unable to get color frame source." << endl;
        bodyFrameReader->Release();
        sensor->Close();
        sensor->Release();
        return -1;
    }

    hr = colorFrameSource->OpenReader(&colorFrameReader);
    if (FAILED(hr)) {
        cerr << "Error: Unable to open color frame reader." << endl;
        colorFrameSource->Release();
        bodyFrameReader->Release();
        sensor->Close();
        sensor->Release();
        return -1;
    }

    colorFrameSource->Release();

    // OpenCV window
    cv::namedWindow("Kinect Feed", cv::WINDOW_AUTOSIZE);

    while (true) {
        // Get the body frame
        IBodyFrame* bodyFrame = nullptr;
        hr = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

        if (SUCCEEDED(hr) && bodyFrame) {
            IBody* bodies[BODY_COUNT] = { 0 };
            bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

            // Get color frame
            IColorFrame* colorFrame = nullptr;
            hr = colorFrameReader->AcquireLatestFrame(&colorFrame);
            if (SUCCEEDED(hr) && colorFrame) {
                int width = 1920; // Kinect color frame width
                int height = 1080; // Kinect color frame height
                cv::Mat colorImage(height, width, CV_8UC4); // 4 channels (BGRA)

                hr = colorFrame->CopyConvertedFrameDataToArray(width * height * 4, (BYTE*)colorImage.data, ColorImageFormat_Bgra);
                if (SUCCEEDED(hr)) {
                    // Display "Test Ready" initially
                    string status = "Test Ready";
                    string instruction = "Please Raise your Right foot";

                    // Display text instructions
                    drawText(colorImage, status, cv::Point(50, 50), cv::Scalar(0, 0 , 255));
                    drawText(colorImage, instruction, cv::Point(50, 100), cv::Scalar(0, 0, 255));

                    for (int i = 0; i < BODY_COUNT; ++i) {
                        IBody* body = bodies[i];
                        if (body) {
                            BOOLEAN isTracked = false;
                            body->get_IsTracked(&isTracked);

                            if (isTracked) {
                                Joint joints[JointType_Count];
                                body->GetJoints(_countof(joints), joints);

                                Joint leftFoot = joints[JointType_FootLeft];
                                Joint rightFoot = joints[JointType_FootRight];

                                // Ensure joints are tracked
                                if (leftFoot.TrackingState == TrackingState_Tracked &&
                                    rightFoot.TrackingState == TrackingState_Tracked) {

                                    float leftZ = leftFoot.Position.Z;
                                    float rightZ = rightFoot.Position.Z;
                                    float leftY = leftFoot.Position.Y;
                                    float rightY = rightFoot.Position.Y;


                                    // Check if the right foot is raised
                                    if (fabs(leftZ - rightZ) > FOOT_RAISE_THRESHOLD_Z ||
                                        fabs(leftY - rightY) > FOOT_RAISE_THRESHOLD_Y) {

                                        if (rightY > leftY) {
                                            //cout << "Right foot raised" << endl;
                                            instruction = "Right foot raised";
                                            if (!rightFootTimerActive) {
                                                footRaiseStart = steady_clock::now();
                                                rightFootTimerActive = true;
                                            }
                                        }

                                    }
                                    else if (rightFootTimerActive) {
                                        auto footRaiseEnd = steady_clock::now();
                                        elapsedTimeRight = duration_cast<seconds>(footRaiseEnd - footRaiseStart).count();
                                        rightFootTimerActive = false;

                                        // Print final time for right foot
                                        cout << "Final Right Foot Time: " << elapsedTimeRight << " seconds" << endl;
                                    }

                                    // If right foot timer is active, display the elapsed time
                                    if (rightFootTimerActive) {
                                        auto currentTime = steady_clock::now();
                                        float liveElapsedTimeRight = duration_cast<seconds>(currentTime - footRaiseStart).count();
                                        drawText(colorImage, "Timer: " + to_string(liveElapsedTimeRight) + "s", cv::Point(50, 150), cv::Scalar(0, 255, 255));
                                    }
                                    else if (elapsedTimeRight > 0) {
                                        drawText(colorImage, "Timer: " + to_string(elapsedTimeRight) + "s", cv::Point(50, 150), cv::Scalar(0, 255, 255));

                                        // If timer exceeds 70 seconds, display message and skip left foot test
                                        if (elapsedTimeRight > 70) {
                                            instruction = "Timer: " + to_string(elapsedTimeRight) + "s greater than 70 seconds, Left Foot Test not Needed";
                                            drawText(colorImage, instruction, cv::Point(50, 200), cv::Scalar(0, 0, 255));
                                        }
                                    }

                                    // Skip the left foot test if the right foot timer exceeds 60 seconds
                                    if (elapsedTimeRight > 60) {
                                        continue; // Skip the left foot test
                                    }

                                    // Now ask for the left foot
                                    instruction = "Please Raise your Left foot";
                                    drawText(colorImage, instruction, cv::Point(50, 200), cv::Scalar(0, 0, 255));

                                    // Check if the left foot is raised
                                    if (fabs(leftZ - rightZ) > FOOT_RAISE_THRESHOLD_Z ||
                                        fabs(leftY - rightY) > FOOT_RAISE_THRESHOLD_Y) {

                                        if (leftY > rightY) {
                                            instruction = "Left foot raised";
                                            if (!leftFootTimerActive) {
                                                footRaiseStart = steady_clock::now();
                                                leftFootTimerActive = true;
                                            }
                                        }

                                    }
                                    else if (leftFootTimerActive) {
                                        auto footRaiseEnd = steady_clock::now();
                                        elapsedTimeLeft = duration_cast<seconds>(footRaiseEnd - footRaiseStart).count();
                                        leftFootTimerActive = false;

                                        // Print final time for left foot
                                        cout << "Final Left Foot Time: " << elapsedTimeLeft << " seconds" << endl;
                                    }

                                    // If left foot timer is active, display the elapsed time
                                    if (leftFootTimerActive) {
                                        auto currentTime = steady_clock::now();
                                        float liveElapsedTimeLeft = duration_cast<seconds>(currentTime - footRaiseStart).count();
                                        drawText(colorImage, "Timer: " + to_string(liveElapsedTimeLeft) + "s", cv::Point(50, 250), cv::Scalar(0, 255, 255));
                                    }
                                    else if (elapsedTimeLeft > 0) {
                                        drawText(colorImage, "Timer: " + to_string(elapsedTimeLeft) + "s", cv::Point(50, 250), cv::Scalar(0, 255, 255));
                                    }
                                }
                            }
                        }
                    }

                    // Display the live color feed with the overlayed text
                    imshow("Kinect Feed", colorImage);
                }
                colorFrame->Release();
            }
            bodyFrame->Release();
        }

        if (cv::waitKey(30) == 13) break; // Press Enter to exit
    }

    // Cleanup
    colorFrameReader->Release();
    bodyFrameReader->Release();
    coordinateMapper->Release();
    sensor->Close();
    sensor->Release();

    return 0;
}
