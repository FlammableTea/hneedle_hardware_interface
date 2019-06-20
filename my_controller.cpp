#include <controller_interface/controller.h>
#include <hardware_interface/actuator_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <roboticscape.h> 

namespace my_controller_ns
{
    class MyPositionController : public controller_interface::Controller<hardware_interface::PositionActuatorInterface>
    {
        bool init(hardware_interface::PositionActuatorInterface* hw, ros::NodeHandle &n)
        {
            std::string my_actuator;
            if (!n.getParam("actuator", my_actuator)){
                ROS_ERROR("Could not find actuator name");
                return false; 
            }
            actuator_ = hw->getHandle(my_actuator); //throws on failure
            command_ = actuator.getPosition(); //set the current actuator goal to the current actuator position

            //Load gain using gains set on parameter server
            if(!n.getParam("gain", gain_))
            {
                ROS_ERROR("Could not find the gain parameter value");
                return false;
            }
            // Start command subscriber
            sub_command_ = n.subsrcribe<std_msgs::Float64>("command", 1, &MyPositionController::setCommandCB, this);

            return true;
        }
        void update(const ros::Time& time, const ros::Duration& period)
        {
            double error = command_ - actuator_.getPosition();
            double commanded_pos = error*gain_;
            actuator_.setCommand(commanded_pos);
        }

        void setCommandCB(const std_msgs::Float64ConstPtr& msg) //TODO: See if this needs to be replaced with falconPos msg 
        {
            command_ = msg->data; 
        }
    };

}

