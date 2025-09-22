#include <cstdio>

#include "mock_vex.h"
#include "../include/subsystems/odometry.h"

/** Command in order to run:
 * g++ --std=c++11 odom_tests.cpp ../src/subsystems/odometry.cpp -D__TESTING__ && ./a.out
 */

// If parameter is not true, test fails
// This check function would be provided by the test framework
// P.S. I don't really use this right now, i should
#define IS_TRUE(x)                                                   \
    {                                                                \
        if (!(x))                                                    \
            printf("%s failed on line %d\n", __FUNCTION__, __LINE__) \
    }

class OdometryTest
{
private:
    vex::encoder leftEncoder{0}, rightEncoder{0}, backEncoder{0};
    Odometry odomTest{leftEncoder, rightEncoder, backEncoder};

    void setMockEncoderDistInches(vex::encoder &leftEncoder, vex::encoder &rightEncoder, vex::encoder &backEncoder,
                                  double leftDist, double rightDist, double backDist)
    {
        leftEncoder.setMockPosition(encoderDistToRevs(leftDist));
        rightEncoder.setMockPosition(encoderDistToRevs(rightDist));
        backEncoder.setMockPosition(encoderDistToRevs(backDist));
    }

    double encoderDistToRevs(double encoderDistInches)
    {
        return encoderDistInches / (2 * M_PI * Odometry::WHEEL_RADIUS_INCHES);
    }

public:
    /* Updates odometry object with mock encoder values and prints the resulting pose */
    void testWithEncoderDistance(double leftDist, double rightDist, double backDist)
    {
        setMockEncoderDistInches(leftEncoder, rightEncoder, backEncoder,
                                 leftDist, rightDist, backDist);
        odomTest.update();
        printf("  ");
        odomTest.getPose().print();
    }

    void reset()
    {
        odomTest.resetOdometry(0, 0, 0);
    }
};

int main()
{
    OdometryTest odomTest{};

    printf("\nGOING STRAIGHT:\n");
    odomTest.testWithEncoderDistance(1.0, 1.0, 0.0);
    odomTest.reset();

    printf("\nGOING RIGHT ANGELED:\n");
    odomTest.testWithEncoderDistance(17.606, 13.810, 2.03);
    odomTest.reset();

    printf("\nGOING LEFT ANGELED:\n");
    odomTest.testWithEncoderDistance(13.810, 17.606, -2.03);
    odomTest.reset();

    printf("\nGOING RIGHT ANGELED 3 TIMES:\n");
    double leftDist = 17.606, rightDist = 13.810, backDist = 2.03;
    for (int i = 0; i < 3; i++)
    {
        odomTest.testWithEncoderDistance(leftDist, rightDist, backDist);
        leftDist += 17.606;
        rightDist += 13.810;
        backDist += 2.03;
    }
    odomTest.reset();

    printf("\nGOING HARD RIGHT ANGELED 3 TIMES:\n");
    leftDist = 16.0, rightDist = -12.0, backDist = 14.97;
    for (int i = 0; i < 3; i++)
    {
        odomTest.testWithEncoderDistance(
            leftDist, rightDist, backDist);
        leftDist += 16.0;
        rightDist += -12.0;
        backDist += 14.97;
    }
    odomTest.reset();

    return 0;
}