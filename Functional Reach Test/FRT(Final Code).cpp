#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

float lastLeftHandY = -1.0f, lastRightHandY = -1.0f;
float lastLeftElbowY = -1.0f, lastRightElbowY = -1.0f;
float initialLeftHandZ = -1.0f, initialRightHandZ = -1.0f;  // To store initial Z coordinates of the hands
float initialLeftElbowZ = -1.0f, initialRightElbowZ = -1.0f;  // To store initial Z coordinates of the elbows
bool armsStable = false;

bool FinalMaximumDistance = false;
int stabilityFrames = 0; // To track how many frames hands are stable
bool armsStablePrinted = false; // Flag to print once when arms are stable
float currentRightHandZ = 0.0f, currentLeftHandZ = 0.0f; // To store the current Z coordinates of the hands
float DistanceRightHand = 0.0f;
float DistanceLeftHand = 0.0f;
float MaximumRightHandDistance = 0.0f;
float MaximumLeftHandDistance = 0.0f;

// Z axis right and left hand threshold
float ThresholdZ = 0.05f;
 
 
//Right hand threshold
float RightHandThreshold = 0.05f;

//left Hand Threshold
float LeftHandThreshold = 0.05f;

//initial x and y coordinate variable for right hand
float initialRightHandX = 0.0f;
float initialRightHandY = 0.0f;

//initial x and y coordinate variable for left hand
float initialLeftHandX = 0.0f;
float initialLeftHandY = 0.0f;

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
									cv::putText(bgrMat, "Test Ready", cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
									cv::putText(bgrMat, "Please Raise both of your arms", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
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
                                            // Save the initial Z coordinate for the left hand
                                            if (initialLeftHandZ == -1.0f) {
                                                initialLeftHandZ = z;
                                            }
                                        }
                                        else if (j == JointType_HandRight) {
                                            rightHandY = joints[j].Position.Y;
                                            // Save the initial Z coordinate for the right hand
                                            if (initialRightHandZ == -1.0f) {
                                                initialRightHandZ = z;
                                            }
                                        }
                                        else if (j == JointType_ElbowLeft) {
                                            leftElbowY = joints[j].Position.Y;
                                            // Save the initial Z coordinate for the left elbow
                                            if (initialLeftElbowZ == -1.0f) {
                                                initialLeftElbowZ = z;
                                            }
                                        }
                                        else if (j == JointType_ElbowRight) {
                                            rightElbowY = joints[j].Position.Y;
                                            // Save the initial Z coordinate for the right elbow
                                            if (initialRightElbowZ == -1.0f) {
                                                initialRightElbowZ = z;
                                            }
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
                                    
                                    initialRightHandZ = joints[JointType_HandRight].Position.Z;
									initialRightHandY = joints[JointType_HandRight].Position.Y;
									initialRightHandX = joints[JointType_HandRight].Position.X;

                                    std::cout << "Right Elbow Coordinates| x: " << joints[JointType_ElbowRight].Position.X << " | y: " << rightElbowY << " | z: " << joints[JointType_ElbowRight].Position.Z << " |" << std::endl;
                                    std::cout << "Left Hand Coordinates  | x: " << joints[JointType_HandLeft].Position.X << " | y: " << leftHandY << " | z: " << joints[JointType_HandLeft].Position.Z << " |" << std::endl;
                                    
                                    initialLeftHandZ = joints[JointType_HandLeft].Position.Z;
									initialLeftHandY = joints[JointType_HandLeft].Position.Y;
									initialLeftHandX = joints[JointType_HandLeft].Position.X;
                                    
                                    std::cout << "Left Elbow Coordinates | x: " << joints[JointType_ElbowLeft].Position.X << " | y: " << leftElbowY << " | z: " << joints[JointType_ElbowLeft].Position.Z << " |" << std::endl;

                                    armsStablePrinted = true; // Prevent printing again
                                    std::cout << "Initial Right Hand Z Coordinates are: " << initialRightHandZ << std::endl;
                                    std::cout << "Initial Left Hand Z Coordinates are: " << initialLeftHandZ << std::endl;

                                    //print message to bend forawrd
									std::cout << "Please bend forward" << std::endl;
                                    //display message on screen in yellow to Bend Forward
									cv::putText(bgrMat, "Please bend forward", cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

									

                                }
								//check if bend forward i.e Z axis changes but X and Y axis remains same
                               //if x and y of both hands change within 5 cm threshold , and z decreases then start measuring the distance, intiial - final value
                                
                                //check if current z - initial z  > threshold
                                if ((initialRightHandZ - joints[JointType_HandRight].Position.Z > ThresholdZ) &&
                                    (initialLeftHandZ - joints[JointType_HandLeft].Position.Z > ThresholdZ) &&
                                    (std::abs(joints[JointType_HandRight].Position.X - initialRightHandX) < RightHandThreshold) &&
                                    (std::abs(joints[JointType_HandRight].Position.Y - initialRightHandY) < RightHandThreshold) &&
                                    (std::abs(joints[JointType_HandLeft].Position.X - initialLeftHandX) < LeftHandThreshold) &&
                                    (std::abs(joints[JointType_HandLeft].Position.Y - initialLeftHandY) < LeftHandThreshold))
                                {
                                    std::cout << "You have bent forward" << std::endl;

                                    // Calculate the distance of the hands from their initial positions
                                    currentRightHandZ = joints[JointType_HandRight].Position.Z;
                                    currentLeftHandZ = joints[JointType_HandLeft].Position.Z;
                                    DistanceRightHand = std::abs(initialRightHandZ - currentRightHandZ);
                                    DistanceLeftHand = std::abs(initialLeftHandZ - currentLeftHandZ);

                                    std::cout << "Right Hand Distance: " << (DistanceRightHand) * 100.0f << " centimeters" << std::endl;
                                    std::cout << "Left Hand Distance: " << DistanceLeftHand * 100.0f << " centimeters" << std::endl;
                                    //display Right Hand Distance on Live Feed
									cv::putText(bgrMat, "Right Hand Distance: " + std::to_string(DistanceRightHand * 100.0f) + " cm",
										cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                    //display left Hand Distance on Live Feed
									cv::putText(bgrMat, "Left Hand Distance: " + std::to_string(DistanceLeftHand * 100.0f) + " cm",
										cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

                                    if (MaximumRightHandDistance < DistanceRightHand)
                                    {
                                        MaximumRightHandDistance = DistanceRightHand;
                                    }
                                    if (MaximumLeftHandDistance < DistanceLeftHand)
                                    {
                                        MaximumLeftHandDistance = DistanceLeftHand;
                                    }

									if (MaximumRightHandDistance - DistanceRightHand >= 0.0f && MaximumLeftHandDistance - DistanceLeftHand >= 0.0f)
									{
										FinalMaximumDistance = true;
                                        break;
									}
                                    // Print maximum distance covered so far
                                    std::cout << "Maximum Right Hand Distance Covered: " << MaximumRightHandDistance * 100.0f << " centimeters" << std::endl;
                                    std::cout << "Maximum Left Hand Distance Covered: " << MaximumLeftHandDistance * 100.0f << " centimeters" << std::endl;
                                   
                                    
                                    //display maximum Right Hand Distance on Live feed
									//cv::putText(bgrMat, "Maximum Right Hand Distance: " + std::to_string(MaximumRightHandDistance * 100.0f) + " cm",
									//	cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
									
                                    //display maximum Left Hand Distance on Live feed
									//cv::putText(bgrMat, "Maximum Left Hand Distance: " + std::to_string(MaximumLeftHandDistance * 100.0f) + " cm",
									//	cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);
                                    
                                    }
                                

                            }
                        }
						std::cout << "Test Completed" << std::endl;
						std::cout << "Maximum Right Hand Distance Covered: " << MaximumRightHandDistance * 100.0f << " centimeters" << std::endl;
						std::cout << "Maximum Left Hand Distance Covered: " << MaximumLeftHandDistance * 100.0f << " centimeters" << std::endl;
                        //display maximum Right Hand Distance on Live feed
                        cv::putText(bgrMat, "Maximum Right Hand Distance: " + std::to_string(MaximumRightHandDistance * 100.0f) + " cm",
                            cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

                        //display maximum Left Hand Distance on Live feed
                        cv::putText(bgrMat, "Maximum Left Hand Distance: " + std::to_string(MaximumLeftHandDistance * 100.0f) + " cm",
                            cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 2);

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
