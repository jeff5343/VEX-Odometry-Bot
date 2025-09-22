#ifndef __MOCK_VEX_H_INCLUDED__
#define __MOCK_VEX_H_INCLUDED__

namespace vex
{
    enum rotationUnits
    {
        rev
    };

    class encoder
    {
    private:
        double mockRevs;

    public:
        encoder(double revs) : mockRevs(revs) {};

        double position(rotationUnits rotUnits) { return mockRevs; }

        void setMockPosition(double revs)
        {
            mockRevs = revs;
        }
    };
}

#endif