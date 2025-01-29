#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <deque> // For stability history tracking
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
using namespace std;

// Constants for stability detection
const int stabilityFramesThreshold = 20; // Number of frames to check for stability
const float stabilityYThreshold = 0.02f; // Y-coordinate fluctuation threshold for stability

// Initial Z-coordinate values for left hand, mid spine, and base spine (assuming -1 is invalid/uninitialized)
float initialLeftHandZ = -1.0f, initialMidSpineZ = -1.0f, initialBaseSpineZ = -1.0f;

// Deques to store Y-coordinate history for stability detection
std::deque<float> leftHandYHistory, midSpineYHistory, baseSpineYHistory;

// Variables to track Y-coordinates of joints in previous frames
float lastLeftHandY = -1.0f, lastMidSpineY = -1.0f, lastBaseSpineY = -1.0f;

int stabilityFrames = 0; // To track how many frames the joints are stable

// Function to check stability
bool isStable(const std::deque<float>& history, float threshold) {
    if (history.size() < stabilityFramesThreshold) return false;
    float minVal = *std::min_element(history.begin(), history.end());
    float maxVal = *std::max_element(history.begin(), history.end());
    return (maxVal - minVal) <= threshold;
}


int main() {
    // Initialize Kinect Sensor, readers, and coordinate mapper
    IKinectSensor* sensor = nullptr;
    IColorFrameReader* colorFrameReader = nullptr;
    IBodyFrameReader* bodyFrameReader = nullptr;
    ICoordinateMapper* coordinateMapper = nullptr;

    if (FAILED(GetDefaultKinectSensor(&sensor)) || !sensor) {
        std::cerr << "Kinect sensor not found!" << std::endl;
        return -1;
    }
    sensor->Open();
    sensor->get_CoordinateMapper(&coordinateMapper);

    IColorFrameSource* colorSource = nullptr;
    sensor->get_ColorFrameSource(&colorSource);
    colorSource->OpenReader(&colorFrameReader);

    IBodyFrameSource* bodySource = nullptr;
    sensor->get_BodyFrameSource(&bodySource);
    bodySource->OpenReader(&bodyFrameReader);

    cv::namedWindow("Joint Labels", cv::WINDOW_AUTOSIZE);

    // Frame loop
    while (true) {
        IColorFrame* colorFrame = nullptr;
        HRESULT hrColor = colorFrameReader->AcquireLatestFrame(&colorFrame);

        if (SUCCEEDED(hrColor)) {
            IFrameDescription* frameDescription = nullptr;
            colorFrame->get_FrameDescription(&frameDescription);

            int width, height;
            frameDescription->get_Width(&width);
            frameDescription->get_Height(&height);

            UINT bufferSize = width * height * 4;
            BYTE* colorBuffer = new BYTE[bufferSize];
            hrColor = colorFrame->CopyConvertedFrameDataToArray(bufferSize, colorBuffer, ColorImageFormat_Bgra);

            if (SUCCEEDED(hrColor)) {
                cv::Mat colorMat(height, width, CV_8UC4, colorBuffer);
                cv::Mat bgrMat;
                cv::cvtColor(colorMat, bgrMat, cv::COLOR_BGRA2BGR);

                IBodyFrame* bodyFrame = nullptr;
                HRESULT hrBody = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

                if (SUCCEEDED(hrBody)) {
                    IBody* bodies[BODY_COUNT] = { 0 };
                    hrBody = bodyFrame->GetAndRefreshBodyData(_countof(bodies), bodies);

                    for (int i = 0; i < BODY_COUNT; ++i) {
                        IBody* body = bodies[i];
                        if (body) {
                            BOOLEAN isTracked = false;
                            body->get_IsTracked(&isTracked);

                            if (isTracked) {
                                Joint joints[JointType_Count];
                                body->GetJoints(_countof(joints), joints);

                                // List of joints to label
                                const int jointsToLabel[] = {
                                    JointType_HandLeft,
                                    JointType_ElbowLeft,
                                    JointType_SpineMid,
                                    JointType_SpineBase
                                };

                                const char* jointLabels[] = {
                                    "Left Hand",
                                    "Left Elbow",
                                    "Mid Spine",
                                    "Base Spine"
                                };

                                // Draw and label the selected joints
                                for (size_t j = 0; j < sizeof(jointsToLabel) / sizeof(jointsToLabel[0]); j++) {
                                    int jointType = jointsToLabel[j];

                                    if (joints[jointType].TrackingState == TrackingState_Tracked) {
                                        // Use the raw camera space coordinates (meters)
                                        float x = joints[jointType].Position.X;
                                        float y = joints[jointType].Position.Y;
                                        float z = joints[jointType].Position.Z;

                                        // Convert camera space to color space for visualization
                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[jointType].Position, &colorPoint);
                                        int cx = static_cast<int>(colorPoint.X);
                                        int cy = static_cast<int>(colorPoint.Y);

                                        // Ensure pixel coordinates are within bounds
                                        if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                                            cv::circle(bgrMat, cv::Point(cx, cy), 10, cv::Scalar(255, 0, 0), -1); // Draw a circle
                                            cv::putText(bgrMat, jointLabels[j], cv::Point(cx + 10, cy), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

                                            // Display the decimal camera space coordinates
                                            cv::putText(bgrMat, "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Z: " + std::to_string(z),
                                                cv::Point(cx + 10, cy + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                                        }
                                    }
                                }

                            }
                        }
                    }

                    bodyFrame->Release();
                }
                cv::imshow("Joint Labels", bgrMat);
            }

            delete[] colorBuffer;
            colorFrame->Release();
            frameDescription->Release();
        }

        if (cv::waitKey(30) == 13) break;
    }

    colorFrameReader->Release();
    bodyFrameReader->Release();
    coordinateMapper->Release();
    colorSource->Release();
    sensor->Close();
    sensor->Release();

    return 0;
}
