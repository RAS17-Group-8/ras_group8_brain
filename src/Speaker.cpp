#include <ras_group8_brain/Brain.hpp>

namespace ras_group8_brain {


bool Brain::Speak(std_msgs::String msg)
{
    std_msgs::String speaker_message;

    speaker_message.data="data: '"+ msg.data +"'";
    speaker_publisher_.publish(speaker_message);
    //ROS_INFO("%s",speaker_message.data.c_str());
    return true;
}

} /* namespace */
