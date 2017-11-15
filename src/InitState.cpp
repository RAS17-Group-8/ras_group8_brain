#include <ras_group8_brain/Brain.hpp>

// STD
#include <string>

namespace ras_group8_brain {


int  Brain::initState()
{
   planned_element=-1;
   picked_up_element=-1;

  std_msgs::String greeting;
  greeting.data="Hello I'm a robot";
  Brain::Speak(greeting);

}




} /* namespace */
