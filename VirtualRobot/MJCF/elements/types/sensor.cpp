#include "sensor.h"

//#include "../Document.h"


namespace mjcf
{
    const std::string SensorSection::tag = "sensor";

    const std::string TouchSensor::tag = "touch";
    const std::string AccelerometerSensor::tag = "accelerometer";
    const std::string VelocimeterSensor::tag = "velocimeter";
    const std::string GyroSensor::tag = "gyro";
    const std::string ForceSensor::tag = "force";
    const std::string TorqueSensor::tag = "torque";
    const std::string MagnetometerSensor::tag = "magnetometer";
    const std::string RangeFinderSensor::tag = "rangefinder";
    const std::string JointPositionSensor::tag = "jointpos";
    const std::string JointVelocitySensor::tag = "jointvel";
    const std::string TendonPositionSensor::tag = "tendonpos";
    const std::string TendonVelocitySensor::tag = "tendonvel";
    const std::string ActuatorPositionSensor::tag = "actuatorpos";
    const std::string ActuatorVelocitySensor::tag = "actuatorvel";
    const std::string ActuatorForceSensor::tag = "actuatorfrc";
    const std::string BallJointValueSensor::tag = "ballquat";
    const std::string BallAngularVelocitySensor::tag = "ballangvel";
    const std::string JointLimitPositionSensor::tag = "jointlimitpos";
    const std::string JointLimitVelocitySensor::tag = "jointlimitvel";
    const std::string JointLimitForceSensor::tag = "jointlimitfrc";
    const std::string TendonLimitPositionSensor::tag = "tendonlimitpos";
    const std::string TendonLimitVelocitySensor::tag = "tendonlimitvel";
    const std::string TendonLimitForceSensor::tag = "tendonlimitfrc";
    const std::string FramePositionSensor::tag = "framepos";
    const std::string FrameOrientationSensor::tag = "framequat";
    const std::string FrameXAxisSensor::tag = "framexaxis";
    const std::string FrameYAxisSensor::tag = "frameyaxis";
    const std::string FrameZAxisSensor::tag = "framezaxis";
    const std::string FrameLinearVelocitySensor::tag = "framelinvel";
    const std::string FrameAngularVelocitySensor::tag = "frameangvel";
    const std::string FrameLinearAccelerationSensor::tag = "framelinacc";
    const std::string FrameAngularAccelerationSensor::tag = "frameangacc";
    const std::string SubtreeCenterOfMassSensor::tag = "subtreecom";
    const std::string SubtreeLinearVelocitySensor::tag = "subtreelinvel";
    const std::string SubtreeAngularMomentumSensor::tag = "subtreeangmom";
} // namespace mjcf
