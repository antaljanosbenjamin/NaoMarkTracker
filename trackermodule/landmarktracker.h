#ifndef MY_MODULE_H
#define MY_MODULE_H

#include <iostream>
#include <alcommon/almodule.h>
#include <alcommon/alproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "myvector.h"
#include <boost/shared_ptr.hpp>
#include <althread/almutex.h>

namespace AL
{
// This is a forward declaration of AL:ALBroker which
// avoids including <alcommon/albroker.h> in this header
class ALBroker;
}

struct Vector;
struct Matrix;

/**
 * This class inherits AL::ALModule. This allows it to bind methods
 * and be run as a remote executable within NAOqi
 */
class LandMarkTracker : public AL::ALModule
{
private:

    const static std::string cameraName;
    AL::ALMemoryProxy memoryProxy;
    AL::ALVideoDeviceProxy videoProxy;
    AL::ALMotionProxy motionProxy;
    std::string memValue;
    boost::shared_ptr<AL::ALMutex> fCallbackMutex;
    Vector markPositionInNaoSpace;
    float trackedMarkRadius;
    bool newData;
    bool subscribed;
    int trackedMarkId;
    AL::ALValue detectInfo;
    AL::ALValue lastDetectedMarkInfo;
    AL::ALValue lastDetectedCameraPos;

    void calculateMarkPosition();
    std::vector<float> createFloatVectorFromMarkPosition();


public:

    LandMarkTracker(boost::shared_ptr<AL::ALBroker> broker,
                    const std::string &name);

    virtual ~LandMarkTracker();

    /**
         * Overloading ALModule::init().
         * This is called right after the module has been loaded
         */
    virtual void init();

    // After that you may add all your bind method.

    // Returns the [x, y, z] position of the landmark in FRAME_TORSO. This is done assuming an average face size, so it might not be very accurate. This invalidates the isNewData field of the tracker. See LandMarkTracker::isNewData() for more details.
    std::vector<float> getPosition();

    // Return true if the face Tracker is running.
    bool isActive();

    // Return true if a new landmark was detected since the last getPosition()
    bool isNewData();

    // if true, the tracking will be through a Whole Body Process.
    void setWholeBodyOn(const bool& pWholeBodyOn);

    // Start the tracker by Subscribing to Event LandmarkDetected from ALLandmarkDetection module. Then Wait Event LandmarkDetected from ALLandmarkDetection module. And finally send information to motion for head tracking. Note: Stiffness of Head must be set to 1.0 to move!
    void startTracker();

    //Stop the tracker by Unsubscribing to Event LandmarkDetected from ALLandmarkDetection module
    void stopTracker();

    // callback for LandmarkDetected event
    void landmarkDetected();

    // Function which prints the word given on parameters
    void setLandmarkRadius(const float& markRadius);

    // Function which prints the word given on parameters
    void setLandmarkId(const int& markId);

};
#endif // MY_MODULE_H
