#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

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

                                // Draw circles for tracked joints (Hands, Knees, and Spine)
                                for (int j = 0; j < JointType_Count; j++) {
                                    if (joints[j].TrackingState == TrackingState_Tracked &&
                                        (j == JointType_HandLeft || j == JointType_HandRight ||
                                            j == JointType_KneeLeft || j == JointType_KneeRight ||
                                            j == JointType_SpineBase || j == JointType_SpineMid)) {

                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);

                                        int x = static_cast<int>(colorPoint.X);
                                        int y = static_cast<int>(colorPoint.Y);

                                        if (x >= 0 && x < width && y >= 0 && y < height) {
                                            cv::circle(bgrMat, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1); // Draw a circle with radius 10

                                            // Add text labels for different joints
                                            if (j == JointType_HandLeft) {
                                                cv::putText(bgrMat, "Left Hand", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                                                cv::putText(bgrMat, "X: " + std::to_string(joints[j].Position.X) + " Y: " + std::to_string(joints[j].Position.Y) + " Z: " + std::to_string(joints[j].Position.Z), cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_HandRight) {
                                                cv::putText(bgrMat, "Right Hand", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                                                cv::putText(bgrMat, "X: " + std::to_string(joints[j].Position.X) + " Y: " + std::to_string(joints[j].Position.Y) + " Z: " + std::to_string(joints[j].Position.Z), cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_KneeLeft) {
                                                cv::putText(bgrMat, "Left Knee", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                                                cv::putText(bgrMat, "X: " + std::to_string(joints[j].Position.X) + " Y: " + std::to_string(joints[j].Position.Y) + " Z: " + std::to_string(joints[j].Position.Z), cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_KneeRight) {
                                                cv::putText(bgrMat, "Right Knee", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                                                cv::putText(bgrMat, "X: " + std::to_string(joints[j].Position.X) + " Y: " + std::to_string(joints[j].Position.Y) + " Z: " + std::to_string(joints[j].Position.Z), cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_SpineBase) {
                                                cv::putText(bgrMat, "Base Spine", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                                                cv::putText(bgrMat, "X: " + std::to_string(joints[j].Position.X) + " Y: " + std::to_string(joints[j].Position.Y) + " Z: " + std::to_string(joints[j].Position.Z), cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                            else if (j == JointType_SpineMid) {
                                                cv::putText(bgrMat, "Mid Spine", cv::Point(x + 10, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                                                cv::putText(bgrMat, "X: " + std::to_string(joints[j].Position.X) + " Y: " + std::to_string(joints[j].Position.Y) + " Z: " + std::to_string(joints[j].Position.Z), cv::Point(x + 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                                            }
                                        }
                                    }
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
