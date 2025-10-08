#ifndef __ODOMETRY_H_INCLUDED__
#define __ODOMETRY_H_INCLUDED__

#ifdef __TESTING__
#include "../../tests/mock_vex.h"
#include "../util/pose.h"
#else
#include "vex.h"
#include "util/pose.h"
#endif

/**
 * Calculates position on field using 3 encoders.
 *
 * - All units are in inches
 * - two encoders on the sides and one encoder in the back
 * - Coordinate system used is based on right hand rule:
 *      X is forward, Y is sideways, Counter Clock Wise (CCW) is positive
 *      https://en.wikipedia.org/wiki/Right-hand_rule#Coordinates
 * - Good Resources on Odometry:
 *     explains the math:
 *         https://www.youtube.com/watch?v=vxSK2NYtYJQ&t=35s
 *     more specifics on implementation:
 *         http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
 *     OkapiLib odometry implementation (math is almost the same):
 *         https://github.com/purduesigbots/OkapiLib/blob/master/src/api/odometry/threeEncoderOdometry.cpp
 * */

class Odometry
{
private:
    const double DIST_CENTER_TO_RIGHT_WHEEL = 7.25;
    const double DIST_CENTER_TO_LEFT_WHEEL = 7.25;
    const double DIST_CENTER_TO_BOT_WHEEL = 7.75;

    vex::encoder &leftEncoder;
    vex::encoder &rightEncoder;
    vex::encoder &backEncoder;

    // distances based on encoders
    double leftDist = 0;
    double rightDist = 0;
    double backDist = 0;
    // deltas
    double dLeftDist = 0;
    double dRightDist = 0;
    double dBackDist = 0;

    Pose pose{0, 0, 0};

    vex::thread worker;
    vex::mutex mutex;
    bool workerRunning;

    // vex threads uses c style threads rather than c++
    // don't know exaclty whats going on but ok
    static int vexThreadWrapper(void *param)
    {
        return static_cast<Odometry *>(param)->threadLoop();
    }

    int threadLoop()
    {
        while (workerRunning)
        {
            update();
            printf("hi!");
            vex::this_thread::sleep_for(10);
        }
        return 0;
    }

    /**
     * Updates odometry by reading encoder distances and calculating pose.
     * Called every loop/tick by the worker thread
     * */
    void update();
    void updateEncoderDistances();
    void updatePose();

    /* added to simplify testing */
    void setNewEncoderDistances(double leftDist, double rightDist, double backDist);

public:
    static constexpr double WHEEL_RADIUS_INCHES = 2.0;
    // TODO: will remove later, but for testing the back wheel has a different radius
    static constexpr double BACK_WHEEL_RADIUS_INCHES = 2.0;

    Odometry(vex::encoder &leftEncoder, vex::encoder &rightEncoder, vex::encoder &backEncoder)
        : leftEncoder(leftEncoder), rightEncoder(rightEncoder), backEncoder(backEncoder)
    {
        workerRunning = true;
        worker = vex::thread(Odometry::vexThreadWrapper, this);
    };

    /* returns calculated pose */
    Pose getPose()
    {
        mutex.lock();
        Pose newPose = pose;
        mutex.unlock();
        return newPose;
    }

    void resetOdometry(double x, double y, double rad)
    {
        mutex.lock();
        pose = Pose{x, y, rad};
        mutex.unlock();
        setNewEncoderDistances(0, 0, 0);
    }

    ~Odometry()
    {
        // stop worker thread
        workerRunning = false;
        worker.join();
    }
};

#endif