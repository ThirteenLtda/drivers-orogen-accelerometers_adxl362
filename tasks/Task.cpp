/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <iostream>
#include <iomanip>

using namespace std;
using namespace accelerometers_adxl362;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    mDriver.open(_device.get());
    mFilterFactor = _filter_factor.get();
    Driver::DeviceID device_id = mDriver.readID();
    cout << "connected to Analog Device device " << hex
        << (int)device_id.ad_device_id << ":" << (int)device_id.mems_device_id << " "
        << "part " << (int)device_id.part_id << " rev " << (int)device_id.rev_id << endl;

    mDriver.standby();
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    mDriver.measure();
    if (_wait_filter_initialization.get())
        mInitializationCount = floor(log(0.001) / log(mFilterFactor));
    else
        mInitializationCount = 0;
    return true;
}
void Task::updateHook()
{
    Driver::RawData raw = mDriver.readRawData();
    Driver::Data data = mDriver.convertRawData(raw);

    base::samples::IMUSensors sample;
    sample.time = base::Time::now();
    sample.acc  = Eigen::Vector3d(data.x, data.y, data.z);
    sample.gyro = Eigen::Vector3d(base::unknown<double>(), base::unknown<double>(), base::unknown<double>());
    sample.mag  = Eigen::Vector3d(base::unknown<double>(), base::unknown<double>(), base::unknown<double>());
    _raw_sensors_samples.write(sample);

    mFilteredAccelerometers = mFilterFactor * sample.acc + (1 - mFilterFactor) * mFilteredAccelerometers;
    if (mInitializationCount == 0)
    {
        sample.acc = mFilteredAccelerometers;
        _sensors_samples.write(sample);
        double pitch = atan2(sample.acc.x(), sample.acc.z());
        double roll  = atan2(sample.acc.y(), sample.acc.z());
        Eigen::Quaterniond q =
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        base::samples::RigidBodyState rbs = base::samples::RigidBodyState::invalid();
        rbs.orientation = q;
        _inclination_samples.write(rbs);
    }
    else
        mInitializationCount--;

    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    mDriver.standby();
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    mDriver.close();
    TaskBase::cleanupHook();
}
