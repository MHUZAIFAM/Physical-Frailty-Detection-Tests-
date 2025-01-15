//This code will print the Coordiantes of both arms and elbows once the arms are raised and are in line with elbows with some considerable threshold 
//output would be as 
/*Test Ready
Please Raise both of your arms
Arms Stable!
Right Hand Coordinates | x: 0.201648 | y: 0.201648 | z: 0.201648 |
Right Elbow Coordinates| x: 0.154241 | y: 0.154241 | z: 0.154241 |
Left Hand Coordinates  | x: 0.178535 | y: 0.178535 | z: 0.178535 |
Left Elbow Coordinates | x: 0.140473 | y: 0.140473 | z: 0.140473 |
*/
#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

float lastLeftHandY = -1.0f, lastRightHandY = -1.0f;
float lastLeftElbowY = -1.0f, lastRightElbowY = -1.0f;
bool armsStable = false;
int stabilityFrames = 0; // To track how many frames hands are stable
bool armsStablePrinted = false; // Flag to print once when arms are stable

// Function to check if hands are stable (based on distance to elbows)
bool areHandsStable(float leftHandY, float rightHandY, float leftElbowY, float rightElbowY) {
    float threshold = 0.1f;  // Set a small threshold for hand-elbow distance detection (10 cm)

    if (lastLeftHandY == -1.0f || lastRightHandY == -1.0f || lastLeftElbowY == -1.0f || lastRightElbowY == -1.0f) {
        // Initial frame
        lastLeftHandY = leftHandY;
        lastRightHandY = rightHandY;
        lastLeftElbowY = leftElbowY;
        lastRightElbowY = rightElbowY;
        return false; // Arms haven't been stable yet
    }

    // Calculate the distance between the hands and elbows (Y-axis)
    float leftHandElbowDist = std::abs(leftHandY - leftElbowY);
    float rightHandElbowDist = std::abs(rightHandY - rightElbowY);

    // Check if the distances are below the threshold
    if (leftHandElbowDist < threshold && rightHandElbowDist < threshold) {
        stabilityFrames++;
        if (stabilityFrames > 10) { // After 10 stable frames, consider it stable
            return true;
        }
    }
    else {
        stabilityFrames = 0; // Reset if there's any movement
    }

    lastLeftHandY = leftHandY;
    lastRightHandY = rightHandY;
    lastLeftElbowY = leftElbowY;
    lastRightElbowY = rightElbowY;

    return false;
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

    cv::namedWindow("Kinect Skeleton", cv::WINDOW_AUTOSIZE);

    bool messagePrinted = false; // Flag to track if message has been printed

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
                                // Print messages only once when a person is detected for the first time
                                if (!messagePrinted) {
                                    std::cout << "Test Ready" << std::endl;
                                    std::cout << "Please Raise both of your arms" << std::endl;
                                    messagePrinted = true; // Set the flag to true to prevent repeated printing
                                }

                                Joint joints[JointType_Count];
                                body->GetJoints(_countof(joints), joints);

                                float leftHandY = 0, rightHandY = 0, leftElbowY = 0, rightElbowY = 0;

                                // Only draw circles for the left and right hand joints
                                for (int j = 0; j < JointType_Count; j++) {
                                    if (joints[j].TrackingState == TrackingState_Tracked &&
                                        (j == JointType_HandLeft || j == JointType_HandRight || j == JointType_ElbowLeft || j == JointType_ElbowRight)) {

                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);

                                        int x = static_cast<int>(colorPoint.X);
                                        int y = static_cast<int>(colorPoint.Y);
                                        float z = joints[j].Position.Z; // This gives the Z coordinate of the hand or elbow

                                        if (j == JointType_HandLeft) {
                                            leftHandY = joints[j].Position.Y;
                                        }
                                        else if (j == JointType_HandRight) {
                                            rightHandY = joints[j].Position.Y;
                                        }
                                        else if (j == JointType_ElbowLeft) {
                                            leftElbowY = joints[j].Position.Y;
                                        }
                                        else if (j == JointType_ElbowRight) {
                                            rightElbowY = joints[j].Position.Y;
                                        }

                                        if (x >= 0 && x < width && y >= 0 && y < height) {
                                            cv::circle(bgrMat, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1); // Draw a circle with radius 10

                                            // Add text label next to the hand joints
                                            if (j == JointType_HandLeft) {
                                                cv::putText(bgrMat, "Left Hand", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_HandRight) {
                                                cv::putText(bgrMat, "Right Hand", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }

                                            // Display the coordinates next to the joint (X, Y, Z values)
                                            cv::putText(bgrMat, "X: " + std::to_string(x) + " Y: " + std::to_string(y) + " Z: " + std::to_string(z),
                                                cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                                        }
                                    }
                                }

                                // Check if hands are stable (in line with elbows within threshold)
                                if (areHandsStable(leftHandY, rightHandY, leftElbowY, rightElbowY) && !armsStablePrinted) {
                                    // Print the coordinates only once
                                    std::cout << "Right Hand Coordinates | x: " << joints[JointType_HandRight].Position.X << " | y: " << rightHandY << " | z: " << joints[JointType_HandRight].Position.Z << " |" << std::endl;
                                    std::cout << "Right Elbow Coordinates| x: " << joints[JointType_ElbowRight].Position.X << " | y: " << rightElbowY << " | z: " << joints[JointType_ElbowRight].Position.Z << " |" << std::endl;
                                    std::cout << "Left Hand Coordinates  | x: " << joints[JointType_HandLeft].Position.X << " | y: " << leftHandY << " | z: " << joints[JointType_HandLeft].Position.Z << " |" << std::endl;
                                    std::cout << "Left Elbow Coordinates | x: " << joints[JointType_ElbowLeft].Position.X << " | y: " << leftElbowY << " | z: " << joints[JointType_ElbowLeft].Position.Z << " |" << std::endl;

                                    armsStablePrinted = true; // Prevent printing again
                                }
                            }
                        }
                    }

                    bodyFrame->Release();
                }
                cv::imshow("Kinect Skeleton", bgrMat);
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
