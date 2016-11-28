
#include <string>
#include <sstream>


template < typename T > std::string to_string( const T& n )
{
        std::ostringstream stm;
        stm << n;
        return stm.str();
}


#include "gps_agent_pkg/joint_group_position_controller.h"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace gps_control {

JointGroupPositionController::JointGroupPositionController()
        : loop_count_(0)
{
}

JointGroupPositionController::~JointGroupPositionController()
{

}

bool JointGroupPositionController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
        urdf::Model urdf;

        if (!urdf.initParam("robot_description"))
        {
                ROS_ERROR("Failed to parse urdf file");
                return false;
        }

        std::vector<std::string> joint_names;
        int joint_count = 0;
        while(true)
        {
                std::string joint_name;
                std::string param_name = std::string("joint_" + to_string(joint_count));
                if (n.getParam(param_name.c_str(), joint_name))
                {
                        joint_names.push_back(joint_name);
                }
                else
                {
                        std::cout<<"num of joints:"<<joint_count<<std::endl;
                        break;
                }
                joint_count++;
        }

        for(std::vector<std::string>::iterator iter = joint_names.begin(); iter!=joint_names.end(); iter++)
        {
                // Get joint handle from hardware interface
                joint_.push_back(robot->getHandle(*iter));

                if (!urdf.getJoint(*iter))
                {
                        break;
                }

                joint_urdf_.push_back(urdf.getJoint(*iter));//???
        }
        int pid_count = 0;

        while(true)
        {
                // Load PID Controller using gains set on parameter server
                if (!pid_controller_.init(ros::NodeHandle(n, std::string("pid_"+to_string(pid_count)))))
                {
                        std::cout<<"num of pid_controller:"<<pid_count<<std::endl;
                        break;
                }
                pid_controller_group.push_back(pid_controller_);
                pid_count++;
        }

        std::cout<<"init succeed!"<<std::endl;
        return true;
}

void JointGroupPositionController::starting(const ros::Time& time)
{
        std::vector<double> pos_command;
        for(std::vector<hardware_interface::JointHandle>::iterator iter = joint_.begin(); iter!=joint_.end(); iter++)
        {
                pos_command.push_back((*iter).getPosition());
        }
        // Make sure joint is within limits if applicable
        // enforceJointLimits(pos_command);
        for(std::vector<control_toolbox::Pid>::iterator iter = pid_controller_group.begin();
            iter!=pid_controller_group.end(); iter++)
        {
                (*iter).reset();
        }

        std::cout<<"starting succeed!"<<std::endl;
}

void JointGroupPositionController::update(const ros::Time& time, const ros::Duration& period)
{
        command_position.push_back(0.25);
        command_position.push_back(0.25);
        command_position.push_back(-0.2);
        command_position.push_back(-0.2);
        command_position.push_back(-0.5);
        command_position.push_back(-0.5);
        command_position.push_back(0.6);
        command_position.push_back(0.6);

        // double command_velocity[] = {};

        double error,vel_error;
        std::vector<double> commanded_effort;

        std::vector<double> current_position;

        for(std::vector<hardware_interface::JointHandle>::iterator iter = joint_.begin(); iter!=joint_.end(); iter++)
        {
                current_position.push_back((*iter).getPosition());
        }

        int itertor_count = 0;
        for(std::vector<boost::shared_ptr<const urdf::Joint> >::iterator iter = joint_urdf_.begin(); iter != joint_urdf_.end(); iter++)
        {
                // Compute position error
                // std::cout<<"Joint:"<<joint_[itertor_count].getName()<<"initial angle"<<current_position[itertor_count]<<std::endl;
                angles::shortest_angular_distance_with_limits(
                        current_position[itertor_count],
                        command_position[itertor_count],
                        (*iter)->limits->lower,
                        (*iter)->limits->upper,
                        error);
                // Set the PID error and compute the PID command with nonuniform
                // time step size.
                vel_error = 0 - joint_[itertor_count].getVelocity();
                commanded_effort.push_back(pid_controller_group[itertor_count].computeCommand(error, vel_error, period));

                joint_[itertor_count].setCommand(commanded_effort[itertor_count]);
                // std::cout<<joint_[itertor_count].getName();
                // std::cout<<commanded_effort[itertor_count]<<"  ";
                std::cout<<error<<"   ";
                itertor_count++;
        }
        std::cout<<std::endl;
}


} // namespace
PLUGINLIB_DECLARE_CLASS(gps_agent_pkg, JointGroupPositionController,
                        gps_control::JointGroupPositionController,
                        controller_interface::ControllerBase)

// PLUGINLIB_EXPORT_CLASS( gps_control::JointGroupPositionController, controller_interface::ControllerBase)
