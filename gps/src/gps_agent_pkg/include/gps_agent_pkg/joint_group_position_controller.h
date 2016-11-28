
#pragma once

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>
#include <vector>
#include <string>
#include <iostream>

namespace gps_control
{

        class JointGroupPositionController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
        {
public:

                JointGroupPositionController();
                ~JointGroupPositionController();


                bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

                void setCommand(double pos_target);

                void setCommand(double pos_target, double vel_target);

                void starting(const ros::Time& time);

                void update(const ros::Time& time, const ros::Duration& period);

                std::vector<hardware_interface::JointHandle> joint_;
                std::vector<boost::shared_ptr<const urdf::Joint> > joint_urdf_;

private:
                int loop_count_;
                control_toolbox::Pid pid_controller_;
                std::vector<control_toolbox::Pid> pid_controller_group; /**< Internal PID controller. */
                std::vector<double> command_position;

        };

} // namespace
