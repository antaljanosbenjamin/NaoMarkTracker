#define _USE_MATH_DEFINES
#include "landmarktracker.h"
#include <iostream>
#include <alcommon/albroker.h>
#include <alproxies/allandmarkdetectionproxy.h>
#include <qi/log.hpp>
#include <althread/alcriticalsection.h>

#include <almath/types/altransform.h>

const std::string LandMarkTracker::cameraName("CameraTop");


LandMarkTracker::LandMarkTracker(boost::shared_ptr<AL::ALBroker> broker,
                                 const std::string& name)
    : AL::ALModule(broker, name), memoryProxy(broker), videoProxy(broker), memValue("LandmarkDetected"), trackedMarkRadius(0.065f),
      fCallbackMutex(AL::ALMutex::createALMutex()), trackedMarkId(0), newData(false), subscribed(false)
{
    // Describe the module here. This will appear on the webpage
    setModuleDescription("LandMarkTracker module.");

    /**
         * addParam(<attribut_name>, <attribut_descrption>);
         * This enables to document the parameters of the method.
         * It is not compulsory to write this line.
         */

    functionName("startTracker", getName(), "Start the tracker");
    BIND_METHOD(LandMarkTracker::startTracker);

    functionName("stopTracker", getName(), "Stop the tracker.");
    BIND_METHOD(LandMarkTracker::stopTracker);

    functionName("isNewData", getName(), "Return true, if there was at leat one detection from the last getPosition.");
    BIND_METHOD(LandMarkTracker::isNewData);

    functionName("isActive", getName(), "Return true, if the tracker is subscribed to LandmarkDetected event");
    BIND_METHOD(LandMarkTracker::isActive);

    functionName("getPosition", getName(), "Return the last detected naomark position.");
    BIND_METHOD(LandMarkTracker::getPosition);

    functionName("landmarkDetected", getName(), "Landmark detected event handler");
    BIND_METHOD(LandMarkTracker::landmarkDetected);

    functionName("setWholeBodyOn", getName(), "Do nothing.");
    addParam("donothing", "Do nothing.");
    BIND_METHOD(LandMarkTracker::setWholeBodyOn);

    functionName("setLandmarkRadius", getName(), "Set the tracked landmark radius to the given.");
    addParam("radius", "The radius in meter to be set.");
    BIND_METHOD(LandMarkTracker::setLandmarkRadius);

    functionName("setLandmarkId", getName(), "Set the tracked landmark id to the given.");
    addParam("markId", "The id to be set.");
    BIND_METHOD(LandMarkTracker::setLandmarkId);

}

LandMarkTracker::~LandMarkTracker()
{

}

std::vector<float> LandMarkTracker::getPosition()
{
    //AL::ALCriticalSection section(fCallbackMutex);
    if (newData){
        AL::ALValue ownDetectInfo = detectInfo;
        for(int i = 0; i< ownDetectInfo[1].getSize(); i++){
            // if trackedMarkId == 0, then we want to track the first mark
            int markId = ownDetectInfo[1][i][1][0];
            if (trackedMarkId == 0 || markId == trackedMarkId){
                lastDetectedMarkInfo = ownDetectInfo[1][i];
                lastDetectedCameraPos = ownDetectInfo[2];
                calculateMarkPosition();

                newData = false;
                return createFloatVectorFromMarkPosition();
            }
        }

        std::cout << "LMT: Can not find the tracked landmark in detectinfo, so the last saved one will be used!" << std::endl;
        if (lastDetectedMarkInfo.getSize() > 0){
            calculateMarkPosition();
            newData = false;
            return createFloatVectorFromMarkPosition();
        }
    }

    std::cout << "LMT: There is no detected info, so [0,0,0] will be returned!" << std::endl;
    static const float arr[] = {0,0,0 };
    return std::vector<float>(arr, arr + sizeof(arr) / sizeof(arr[0]) );
}

std::vector<float> LandMarkTracker::createFloatVectorFromMarkPosition(){
    Vector savedPosition = markPositionInNaoSpace;
    std::vector<float> returnVector;
    returnVector.push_back(savedPosition.x);
    returnVector.push_back(savedPosition.y);
    returnVector.push_back(savedPosition.z);
    return returnVector;
}

bool LandMarkTracker::isActive()
{
    return subscribed;
}

bool LandMarkTracker::isNewData()
{
    if (!newData)
        return false;

    AL::ALValue ownDetectInfo = detectInfo;
    for(int i = 0; i< ownDetectInfo[1].getSize(); i++){
        // if trackedMarkId == 0, then we want to track the first mark
        int markId = ownDetectInfo[1][i][1][0];
        if (trackedMarkId == 0 || markId == trackedMarkId){
            lastDetectedMarkInfo = ownDetectInfo[1][i];
            lastDetectedCameraPos = ownDetectInfo[2];
            return true;
        }
    }

    return false;
}

void LandMarkTracker::setWholeBodyOn(const bool& pWholeBodyOn)
{

}

void LandMarkTracker::startTracker()
{
    subscribed = true;
    std::cout << "LMT: Subscribed!" << std::endl;
    memoryProxy.subscribeToEvent("LandmarkDetected", "LandMarkTracker", "landmarkDetected");

}

void LandMarkTracker::stopTracker()
{
    subscribed = false;
    std::cout << "LMT: Unsubscribed!" << std::endl;
    memoryProxy.unsubscribeToEvent("LandmarkDetected", "LandMarkTracker");
}

void LandMarkTracker::landmarkDetected()
{

    std::cout << "LMT: LandmarkDetected event raised!" << std::endl;
    //AL::ALCriticalSection section(fCallbackMutex);
    AL::ALValue localDetectInfo = memoryProxy.getData(memValue,0);
    std::cout << "LMT: detectinfo size:" << localDetectInfo.getSize() <<  std::endl;
    if (localDetectInfo.getSize() < 2){
        std::cout << "LMT: No landmark detected!" << std::endl;
        return;
    }

    detectInfo = localDetectInfo;
    newData = true;
}

void LandMarkTracker::calculateMarkPosition()
{
    AL::ALValue shapeInfo = lastDetectedMarkInfo[0];
    float horizontalAngle = shapeInfo[1];
    float verticalAngle = shapeInfo[2];
    float widthInAngle = shapeInfo[3];

    float distance = trackedMarkRadius / tan(widthInAngle / 2.0);

    //Get the transform that describes the camera's position
    std::vector<float>  transform = lastDetectedCameraPos;
    AL::Math::Transform robotToCamera = AL::Math::Transform::fromPosition(transform[0], transform[1], transform[2], transform[3], transform[4], transform[5]);

    //Compute the rotation to point towards the landmark.
    AL::Math::Transform cameraToLandmarkRotationTransform = AL::Math::Transform::from3DRotation(0, verticalAngle, horizontalAngle);

    //Compute the translation to reach the landmark.
    AL::Math::Transform cameraToLandmarkTranslationTransform = AL::Math::Transform(distance, 0, 0);

    //Combine all transformations to get the landmark position in NAO space.
    AL::Math::Transform robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform *cameraToLandmarkTranslationTransform;

    std::cout << "LMT: x " << robotToLandmark.r1_c4 << " (in meters)" << std::endl;
    std::cout << "LMT: y " << robotToLandmark.r2_c4 << " (in meters)" << std::endl;
    std::cout << "LMT: z " << robotToLandmark.r3_c4 << " (in meters)" << std::endl;
    markPositionInNaoSpace = Vector(robotToLandmark.r1_c4, robotToLandmark.r2_c4, robotToLandmark.r3_c4);
}

void LandMarkTracker::init()
{
    std::cout << "LandmarkTracker initialized" << std::endl;
}

void LandMarkTracker::setLandmarkRadius(const float& markRadius){
    trackedMarkRadius = markRadius;
    newData = false;
    lastDetectedMarkInfo = AL::ALValue();
    std::cout << "LMT: radius was set to: " << markRadius << std::endl;
}

void LandMarkTracker::setLandmarkId(const int& markId){
    trackedMarkId = markId;
    newData = false;
    lastDetectedMarkInfo = AL::ALValue();
    std::cout << "LMT: id was set to: " << markId << std::endl;
}


