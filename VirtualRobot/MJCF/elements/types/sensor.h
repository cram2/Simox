#pragma once

#include "../core/Attribute.h"

namespace mjcf
{

    /// Sensor section.
    /// @see http://www.mujoco.org/book/XMLreference.html#sensor
    struct SensorSection : public Element<SensorSection>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(SensorSection)
    };

#define mjcf_SensorAttributes(Derived)                                                             \
    mjcf_NameAttribute(Derived);                                                                   \
    mjcf_FloatAttributeDef(Derived, noise, 0);                                                     \
    mjcf_FloatAttributeDef(Derived, cutoff, 0);

    // noise:
    /**
     * When this value is positive, it limits the absolute value of the
     * sensor output. It is also used to normalize the sensor output in
     * the sensor data plots in HAPTIX and simulate.cpp.
     */

    // cutoff
    /**
     * The standard deviation of zero-mean Gaussian noise added to the
     * sensor output, when the sensornoise attribute of flag is enabled.
     * Sensor noise respects the sensor data type: quaternions and unit
     * vectors remain normalized, non-negative quantities remain non-negative.
     */


    /**
     * @brief A TouchSensor element.
     *
     * "This element creates a touch sensor. The active sensor zone is defined
     * by a site. If a contact point falls within the site's volume, and
     * involves a geom attached to the same body as the site, the corresponding
     * contact force is included in the sensor reading. If a contact point falls
     * outside the sensor zone, but the normal ray intersects the sensor zone,
     * it is also included. This re-projection feature is needed because,
     * without it, the contact point may leave the sensor zone from the back
     * (due to soft contacts) and cause an erroneous force reading. The output
     * of this sensor is non-negative scalar. It is computed by adding up the
     * (scalar) normal forces from all included contacts. An example of touch
     * sensor zones for a robotic hand can be found in the Sensors section in
     * the MuJoCo HATPIX chapter."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-touch
     */
    struct TouchSensor : public Element<TouchSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(TouchSensor)

            mjcf_SensorAttributes(TouchSensor)

            /**
         * @brief Site defining the active sensor zone.
         */
            mjcf_StringAttributeReq(TouchSensor, site);
    };

    /**
     * @brief A AccelerometerSensor element.
     *
     * "This element creates a 3-axis accelerometer. The sensor is mounted at a
     * site, and has the same position and orientation as the site frame. This
     * sensor outputs three numbers, which are the linear acceleration of the
     * site (including gravity) in local coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-accelerometer
     */
    struct AccelerometerSensor : public Element<AccelerometerSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(AccelerometerSensor)

            mjcf_SensorAttributes(AccelerometerSensor)

            /**
         * @brief Site where the sensor is mounted.
         *
         * The accelerometer is centered and aligned with the site local frame.
         */
            mjcf_StringAttributeReq(AccelerometerSensor, site);
    };

    /**
     * @brief A VelocimeterSensor element.
     *
     * "This element creates a 3-axis velocimeter. The sensor is mounted at a
     * site, and has the same position and orientation as the site frame. This
     * sensor outputs three numbers, which are the linear velocity of the site
     * in local coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-velocimeter
     */
    struct VelocimeterSensor : public Element<VelocimeterSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(VelocimeterSensor)

            mjcf_SensorAttributes(VelocimeterSensor)

            /**
         * @brief Site where the sensor is mounted.
         *
         * The velocimeter is centered and aligned with the site local frame.
         */
            mjcf_StringAttributeReq(VelocimeterSensor, site);
    };

    /**
     * @brief A GyroSensor element.
     *
     * "This element creates a 3-axis gyroscope. The sensor is mounted at a
     * site, and has the same position and orientation as the site frame. This
     * sensor outputs three numbers, which are the angular velocity of the site
     * in local coordinates. This sensor is often used in conjunction with an
     * accelerometer mounted at the same site, to simulate an inertial
     * measurement unit (IMU)."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-gyro
     */
    struct GyroSensor : public Element<GyroSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(GyroSensor)

            mjcf_SensorAttributes(GyroSensor)

            /**
         * @brief Site where the sensor is mounted.
         *
         * The gyroscope is centered and aligned with the site local frame.
         */
            mjcf_StringAttributeReq(GyroSensor, site);
    };

    /**
     * @brief A ForceSensor element.
     *
     * "This element creates a 3-axis force sensor. The sensor outputs three
     * numbers, which are the interaction force between a child and a parent
     * body, expressed in the site frame defining the sensor. The convention is
     * that the site is attached to the child body, and the force points from
     * the child towards the parent. To change the sign of the sensor reading,
     * use the scale attribute. The computation here takes into account all
     * forces acting on the system, including contacts as well as external
     * perturbations. Using this sensor often requires creating a dummy body
     * welded to its parent (i.e. having no joint elements)."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-force
     */
    struct ForceSensor : public Element<ForceSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(ForceSensor)

            mjcf_SensorAttributes(ForceSensor)

            /**
         * @brief Site where the sensor is mounted.
         *
         * The measured interaction force is between the body where the site is
         * defined and its parent body, and points from the child towards the
         * parent. The physical sensor being modeled could of course be attached
         * to the parent body, in which case the sensor data would have the
         * opposite sign. Note that each body has a unique parent but can have
         * multiple children, which is why we define this sensor through the
         * child rather than the parent body in the pair.
         */
            mjcf_StringAttributeReq(ForceSensor, site);
    };

    /**
     * @brief A TorqueSensor element.
     *
     * "This element creates a 3-axis torque sensor. This is similar to the
     * force sensor above, but measures torque rather than force."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-torque
     */
    struct TorqueSensor : public Element<TorqueSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(TorqueSensor)

            mjcf_SensorAttributes(TorqueSensor)

            /**
         * @brief Site where the sensor is mounted.
         *
         * The measured interaction torque is between the body where the site is
         * defined and its parent body.
         */
            mjcf_StringAttributeReq(TorqueSensor, site);
    };

    /**
     * @brief A MagnetometerSensor element.
     *
     * "This element creates a magnetometer. It measures the magnetic flux at
     * the sensor site position, expressed in the sensor site frame. The output
     * is a 3D vector."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-magnetometer
     */
    struct MagnetometerSensor : public Element<MagnetometerSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(MagnetometerSensor)

            mjcf_SensorAttributes(MagnetometerSensor)

            /**
         * @brief The site where the sensor is attached.
         */
            mjcf_StringAttributeReq(MagnetometerSensor, site);
    };

    /**
     * @brief A RangeFinderSensor element.
     *
     * "This element creates a rangefinder. It measures the distance to the
     * nearest geom surface, along the ray defined by the positive Z-axis of the
     * sensor site. If the ray does not intersect any geom surface, the sensor
     * output is -1. If the origin of the ray is inside a geom, the surface is
     * still sensed (but not the inner volume). Geoms attached to the same body
     * as the sensor site are excluded. Invisible geoms, defined as geoms whose
     * rgba (or whose material rgba) has alpha=0, are also excluded. Note
     * however that geoms made invisible in the visualizer by disabling their
     * geom group are not excluded; this is because sensor calculations are
     * independent of the visualizer."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-rangefinder
     */
    struct RangeFinderSensor : public Element<RangeFinderSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(RangeFinderSensor)

            mjcf_SensorAttributes(RangeFinderSensor)

            /**
         * @brief The site where the sensor is attached.
         */
            mjcf_StringAttributeReq(RangeFinderSensor, site);
    };

    /**
     * @brief A JointPositionSensor element.
     *
     * "This and the remaining sensor elements do not involve sensor-specific
     * computations. Instead they copy into the array mjData.sensordata
     * quantities that are already computed. This element creates a joint
     * position or angle sensor. It can be attached to scalar joints (slide or
     * hinge). Its output is scalar."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-jointpos
     */
    struct JointPositionSensor : public Element<JointPositionSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(JointPositionSensor)

            mjcf_SensorAttributes(JointPositionSensor)

            /**
         * @brief The joint whose position or angle will be sensed.
         *
         * Only scalar joints can be referenced here. The sensor output is copied
         * from mjData.qpos.
         */
            mjcf_StringAttributeReq(JointPositionSensor, joint);
    };

    /**
     * @brief A JointVelocitySensor element.
     *
     * "This element creates a joint velocity sensor. It can be attached to
     * scalar joints (slide or hinge). Its output is scalar."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-jointvel
     */
    struct JointVelocitySensor : public Element<JointVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(JointVelocitySensor)

            mjcf_SensorAttributes(JointVelocitySensor)

            /**
         * @brief The joint whose velocity will be sensed.
         *
         * Only scalar joints can be referenced here. The sensor output is copied
         * from mjData.qvel.
         */
            mjcf_StringAttributeReq(JointVelocitySensor, joint);
    };

    /**
     * @brief A TendonPositionSensor element.
     *
     * "This element creates a tendon length sensor. It can be attached to both
     * spatial and fixed tendons. Its output is scalar."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-tendonpos
     */
    struct TendonPositionSensor : public Element<TendonPositionSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(TendonPositionSensor)

            mjcf_SensorAttributes(TendonPositionSensor)

            /**
         * @brief The tendon whose length will be sensed.
         *
         * The sensor output is copied from mjData.ten_length.
         */
            mjcf_StringAttributeReq(TendonPositionSensor, tendon);
    };

    /**
     * @brief A TendonVelocitySensor element.
     *
     * "This element creates a tendon velocity sensor. It can be attached to
     * both spatial and fixed tendons. Its output is scalar."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-tendonvel
     */
    struct TendonVelocitySensor : public Element<TendonVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(TendonVelocitySensor)

            mjcf_SensorAttributes(TendonVelocitySensor)

            /**
         * @brief The tendon whose velocity will be sensed.
         *
         * The sensor output is copied from mjData.ten_velocity.
         */
            mjcf_StringAttributeReq(TendonVelocitySensor, tendon);
    };

    /**
     * @brief A ActuatorPositionSensor element.
     *
     * "This element creates an actuator length sensor. Recall that each
     * actuator has a transmission which has length. This sensor can be attached
     * to any actuator. Its output is scalar."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-actuatorpos
     */
    struct ActuatorPositionSensor : public Element<ActuatorPositionSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(ActuatorPositionSensor)

            mjcf_SensorAttributes(ActuatorPositionSensor)

            /**
         * @brief The actuator whose transmission's length will be sensed.
         *
         * The sensor output is copied from mjData.actuator_length.
         */
            mjcf_StringAttributeReq(ActuatorPositionSensor, actuator);
    };

    /**
     * @brief A ActuatorVelocitySensor element.
     *
     * "This element creates an actuator velocity sensor. This sensor can be
     * attached to any actuator. Its output is scalar."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-actuatorvel
     */
    struct ActuatorVelocitySensor : public Element<ActuatorVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(ActuatorVelocitySensor)

            mjcf_SensorAttributes(ActuatorVelocitySensor)

            /**
         * @brief The actuator whose transmission's velocity will be sensed.
         *
         * The sensor output is copied from mjData.actuator_velocity.
         */
            mjcf_StringAttributeReq(ActuatorVelocitySensor, actuator);
    };

    /**
     * @brief A ActuatorForceSensor element.
     *
     * "This element creates an actuator force sensor. The quantity being sensed
     * is the scalar actuator force, not the generalized force contributed by
     * the actuator (the latter is the product of the scalar force and the
     * vector of moment arms determined by the transmission). This sensor can be
     * attached to any actuator. Its output is scalar."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-actuatorfrc
     */
    struct ActuatorForceSensor : public Element<ActuatorForceSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(ActuatorForceSensor)

            mjcf_SensorAttributes(ActuatorForceSensor)

            /**
         * @brief The actuator whose scalar force output will be sensed.
         *
         * The sensor output is copied from mjData.actuator_force.
         */
            mjcf_StringAttributeReq(ActuatorForceSensor, actuator);
    };

    /**
     * @brief A BallJointValueSensor element.
     *
     * "This element creates a quaternion sensor for a ball joints. It outputs 4
     * numbers corresponding to a unit quaternion."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-ballquat
     */
    struct BallJointValueSensor : public Element<BallJointValueSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(BallJointValueSensor)

            mjcf_SensorAttributes(BallJointValueSensor)

            /**
         * @brief The ball joint whose quaternion is sensed.
         *
         * The sensor output is copied from mjData.qpos.
         */
            mjcf_StringAttributeReq(BallJointValueSensor, joint);
    };

    /**
     * @brief A BallAngularVelocitySensor element.
     *
     * "This element creates a ball joint angular velocity sensor. It outputs 3
     * numbers corresponding to the angular velocity of the joint. The norm of
     * that vector is the rotation speed in rad/s and the direction is the axis
     * around which the rotation takes place."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-ballangvel
     */
    struct BallAngularVelocitySensor : public Element<BallAngularVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(BallAngularVelocitySensor)

            mjcf_SensorAttributes(BallAngularVelocitySensor)

            /**
         * @brief The ball joint whose angular velocity is sensed.
         *
         * The sensor output is copied from mjData.qvel.
         */
            mjcf_StringAttributeReq(BallAngularVelocitySensor, joint);
    };

    /**
     * @brief A JointLimitPositionSensor element.
     *
     * "This element creates a joint limit sensor for position."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-jointlimitpos
     */
    struct JointLimitPositionSensor : public Element<JointLimitPositionSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(JointLimitPositionSensor)

            mjcf_SensorAttributes(JointLimitPositionSensor)

            /**
         * @brief The joint whose limit is sensed.
         *
         * The sensor output equals mjData.efc_pos - mjData.efc_margin for the
         * corresponding limit constraint. Note that the result is negative if
         * the limit is violated, regardless of which side of the limit is
         * violated. If both sides of the limit are violated simultaneously, only
         * the first component is returned. If there is no violation, the result
         * is 0.
         */
            mjcf_StringAttributeReq(JointLimitPositionSensor, joint);
    };

    /**
     * @brief A JointLimitVelocitySensor element.
     *
     * "This element creates a joint limit sensor for velocity."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-jointlimitvel
     */
    struct JointLimitVelocitySensor : public Element<JointLimitVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(JointLimitVelocitySensor)

            mjcf_SensorAttributes(JointLimitVelocitySensor)

            /**
         * @brief The joint whose limit is sensed.
         *
         * The sensor output is copied from mjData.efc_vel. If the joint limit is
         * not violated, the result is 0.
         */
            mjcf_StringAttributeReq(JointLimitVelocitySensor, joint);
    };

    /**
     * @brief A JointLimitForceSensor element.
     *
     * "This element creates a joint limit sensor for constraint force."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-jointlimitfrc
     */
    struct JointLimitForceSensor : public Element<JointLimitForceSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(JointLimitForceSensor)

            mjcf_SensorAttributes(JointLimitForceSensor)

            /**
         * @brief The joint whose limit is sensed.
         *
         * The sensor output is copied from mjData.efc_force. If the joint limit
         * is not violated, the result is 0.
         */
            mjcf_StringAttributeReq(JointLimitForceSensor, joint);
    };

    /**
     * @brief A TendonLimitPositionSensor element.
     *
     * "This element creates a tendon limit sensor for position."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-tendonlimitpos
     */
    struct TendonLimitPositionSensor : public Element<TendonLimitPositionSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(TendonLimitPositionSensor)

            mjcf_SensorAttributes(TendonLimitPositionSensor)

            /**
         * @brief The tendon whose limit is sensed.
         *
         * The sensor output equals mjData.efc_pos - mjData.efc_margin for the
         * corresponding limit constraint. If the tendon limit is not violated,
         * the result is 0.
         */
            mjcf_StringAttributeReq(TendonLimitPositionSensor, tendon);
    };

    /**
     * @brief A TendonLimitVelocitySensor element.
     *
     * "This element creates a tendon limit sensor for velocity."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-tendonlimitvel
     */
    struct TendonLimitVelocitySensor : public Element<TendonLimitVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(TendonLimitVelocitySensor)

            mjcf_SensorAttributes(TendonLimitVelocitySensor)

            /**
         * @brief The tendon whose limit is sensed.
         *
         * The sensor output is copied from mjData.efc_vel. If the tendon limit
         * is not violated, the result is 0.
         */
            mjcf_StringAttributeReq(TendonLimitVelocitySensor, tendon);
    };

    /**
     * @brief A TendonLimitForceSensor element.
     *
     * "This element creates a tendon limit sensor for constraint force."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-tendonlimitfrc
     */
    struct TendonLimitForceSensor : public Element<TendonLimitForceSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(TendonLimitForceSensor)

            mjcf_SensorAttributes(TendonLimitForceSensor)

            /**
         * @brief The tendon whose limit is sensed.
         *
         * The sensor output is copied from mjData.efc_force. If the tendon limit
         * is not violated, the result is 0.
         */
            mjcf_StringAttributeReq(TendonLimitForceSensor, tendon);
    };

    /**
     * @brief A FramePositionSensor element.
     *
     * "This element creates a sensor that returns the 3D position of the
     * spatial frame of the object, in global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-framepos
     */
    struct FramePositionSensor : public Element<FramePositionSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FramePositionSensor)

            mjcf_SensorAttributes(FramePositionSensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FramePositionSensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FramePositionSensor, objname);
    };

    /**
     * @brief A FrameOrientationSensor element.
     *
     * "This element creates a sensor that returns the unit quaternion
     * specifying the orientation of the spatial frame of the object, in global
     * coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-framequat
     */
    struct FrameOrientationSensor : public Element<FrameOrientationSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameOrientationSensor)

            mjcf_SensorAttributes(FrameOrientationSensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameOrientationSensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameOrientationSensor, objname);
    };

    /**
     * @brief A FrameXAxisSensor element.
     *
     * "This element creates a sensor that returns the 3D unit vector
     * corresponding to the X-axis of the spatial frame of the object, in global
     * coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-framexaxis
     */
    struct FrameXAxisSensor : public Element<FrameXAxisSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameXAxisSensor)

            mjcf_SensorAttributes(FrameXAxisSensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameXAxisSensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameXAxisSensor, objname);
    };

    /**
     * @brief A FrameYAxisSensor element.
     *
     * "This element creates a sensor that returns the 3D unit vector
     * corresponding to the Y-axis of the spatial frame of the object, in global
     * coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-frameyaxis
     */
    struct FrameYAxisSensor : public Element<FrameYAxisSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameYAxisSensor)

            mjcf_SensorAttributes(FrameYAxisSensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameYAxisSensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameYAxisSensor, objname);
    };

    /**
     * @brief A FrameZAxisSensor element.
     *
     * "This element creates a sensor that returns the 3D unit vector
     * corresponding to the Z-axis of the spatial frame of the object, in global
     * coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-framezaxis
     */
    struct FrameZAxisSensor : public Element<FrameZAxisSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameZAxisSensor)

            mjcf_SensorAttributes(FrameZAxisSensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameZAxisSensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameZAxisSensor, objname);
    };

    /**
     * @brief A FrameLinearVelocitySensor element.
     *
     * "This element creates a sensor that returns the 3D linear velocity of the
     * spatial frame of the object, in global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-framelinvel
     */
    struct FrameLinearVelocitySensor : public Element<FrameLinearVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameLinearVelocitySensor)

            mjcf_SensorAttributes(FrameLinearVelocitySensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameLinearVelocitySensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameLinearVelocitySensor, objname);
    };

    /**
     * @brief A FrameAngularVelocitySensor element.
     *
     * "This element creates a sensor that returns the 3D angular velocity of
     * the spatial frame of the object, in global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-frameangvel
     */
    struct FrameAngularVelocitySensor : public Element<FrameAngularVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameAngularVelocitySensor)

            mjcf_SensorAttributes(FrameAngularVelocitySensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameAngularVelocitySensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameAngularVelocitySensor, objname);
    };

    /**
     * @brief A FrameLinearAccelerationSensor element.
     *
     * "This element creates a sensor that returns the 3D linear acceleration of
     * the spatial frame of the object, in global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-framelinacc
     */
    struct FrameLinearAccelerationSensor : public Element<FrameLinearAccelerationSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameLinearAccelerationSensor)

            mjcf_SensorAttributes(FrameLinearAccelerationSensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameLinearAccelerationSensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameLinearAccelerationSensor, objname);
    };

    /**
     * @brief A FrameAngularAccelerationSensor element.
     *
     * "This element creates a sensor that returns the 3D angular acceleration
     * of the spatial frame of the object, in global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-frameangacc
     */
    struct FrameAngularAccelerationSensor : public Element<FrameAngularAccelerationSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(FrameAngularAccelerationSensor)

            mjcf_SensorAttributes(FrameAngularAccelerationSensor)

            /**
         * @brief The type of object to which the sensor is attached.
         *
         * This must be an object type that has a spatial frame. "body" refers to
         * the inertial frame of the body, while "xbody" refers to the regular
         * frame of the body (usually centered at the joint with the parent
         * body).
         */
            mjcf_StringAttributeReq(FrameAngularAccelerationSensor, objtype);

        /**
         * @brief The name of the object to which the sensor is attached.
         */
        mjcf_StringAttributeReq(FrameAngularAccelerationSensor, objname);
    };

    /**
     * @brief A SubtreeCenterOfMassSensor element.
     *
     * "This element creates sensor that returns the center of mass of the
     * kinematic subtree rooted at a specified body, in global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-subtreecom
     */
    struct SubtreeCenterOfMassSensor : public Element<SubtreeCenterOfMassSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(SubtreeCenterOfMassSensor)

            mjcf_SensorAttributes(SubtreeCenterOfMassSensor)

            /**
         * @brief Name of the body where the kinematic subtree is rooted.
         */
            mjcf_StringAttributeReq(SubtreeCenterOfMassSensor, body);
    };

    /**
     * @brief A SubtreeLinearVelocitySensor element.
     *
     * "This element creates sensor that returns the linear velocity of the
     * center of mass of the kinematic subtree rooted at a specified body, in
     * global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-subtreelinvel
     */
    struct SubtreeLinearVelocitySensor : public Element<SubtreeLinearVelocitySensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(SubtreeLinearVelocitySensor)

            mjcf_SensorAttributes(SubtreeLinearVelocitySensor)

            /**
         * @brief Name of the body where the kinematic subtree is rooted.
         */
            mjcf_StringAttributeReq(SubtreeLinearVelocitySensor, body);
    };

    /**
     * @brief A SubtreeAngularMomentumSensor element.
     *
     * "This element creates sensor that returns the angular momentum around the
     * center of mass of the kinematic subtree rooted at a specified body, in
     * global coordinates."
     *
     * @see http://www.mujoco.org/book/XMLreference.html#sensor-subtreeangmom
     */
    struct SubtreeAngularMomentumSensor : public Element<SubtreeAngularMomentumSensor>
    {
        static const std::string tag;
        mjcf_ElementDerivedConstructors(SubtreeAngularMomentumSensor)

            mjcf_SensorAttributes(SubtreeAngularMomentumSensor)

            /**
         * @brief Name of the body where the kinematic subtree is rooted.
         */
            mjcf_StringAttributeReq(SubtreeAngularMomentumSensor, body);
    };

#undef mjcf_SensorAttributes
} // namespace mjcf
