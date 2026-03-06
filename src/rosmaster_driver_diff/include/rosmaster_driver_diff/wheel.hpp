#ifndef DIFFDRIVE_ROSMASTER_WHEEL_HPP
#define DIFFDRIVE_ROSMASTER_WHEEL_HPP

#include <string>
#include <cmath>


class Wheel
{
    public:

    std::string name = "";
    long enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;
    double rads_per_count = 0;
    long enc_offset = 0;
    bool update_offset = false;
    double imu_z = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
        setup(wheel_name, counts_per_rev);
    }
    
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
        name = wheel_name;
        rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calcEncAngle(long offset)
    {
        return ((enc + offset) * rads_per_count);
    }


};


#endif // DIFFDRIVE_ROSMASTER_WHEEL_HPP