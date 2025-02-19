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
                                Joint joints[JointType_Count];
                                body->GetJoints(JointType_Count, joints);  // Get all joints first

                                std::vector<cv::Point> jointPoints;  // Store valid joint positions

                                for (int j = 0; j < JointType_Count; ++j) {
                                    if (joints[j].TrackingState == TrackingState_Tracked) {
                                        ColorSpacePoint colorPoint;
                                        coordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorPoint);

                                        int x = static_cast<int>(colorPoint.X);
                                        int y = static_cast<int>(colorPoint.Y);

                                        // Debugging output
                                        std::cout << "Joint " << j << " -> X: " << x << ", Y: " << y << std::endl;

                                        if (x > 0 && x < width && y > 0 && y < height) {
                                            jointPoints.push_back(cv::Point(x, y));
                                        }
                                    }
                                }

                                // If we have valid joint points, draw bounding box
                                if (!jointPoints.empty()) {
                                    cv::Rect boundingRect = cv::boundingRect(jointPoints);
                                    cv::rectangle(bgrMat, boundingRect, cv::Scalar(0, 255, 0), 2);

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
