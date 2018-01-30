/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OptimalHeadingController.hpp"
#include <base/Angle.hpp>

using namespace auv_control;

OptimalHeadingController::OptimalHeadingController(std::string const& name)
    : OptimalHeadingControllerBase(name)
{
}

OptimalHeadingController::OptimalHeadingController(std::string const& name, RTT::ExecutionEngine* engine)
    : OptimalHeadingControllerBase(name, engine)
{
}

OptimalHeadingController::~OptimalHeadingController()
{
}

bool OptimalHeadingController::disable()
{
    do_optimal_heading = false; 
    std::cout << "OPTIMAL False" << std::endl;
    return false;
}

bool OptimalHeadingController::enable()
{
    do_optimal_heading = true;
    std::cout << "OPTIMAL TRUE" << std::endl;

    return true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OptimalHeadingController.hpp for more detailed
// documentation about them.

bool OptimalHeadingController::configureHook()
{
    if (! OptimalHeadingControllerBase::configureHook())
        return false;
    return true;
}
bool OptimalHeadingController::startHook()
{
    if (! OptimalHeadingControllerBase::startHook())
        return false;
    do_optimal_heading = false;
    return true;
}
void OptimalHeadingController::updateHook()
{
    OptimalHeadingControllerBase::updateHook();
}

void OptimalHeadingController::errorHook()
{
    OptimalHeadingControllerBase::errorHook();
}

void OptimalHeadingController::keep(){
}

bool OptimalHeadingController::calcOutput(const LinearAngular6DCommandStatus &merged_command){
    base::LinearAngular6DCommand output_command;
    double opt_heading;
    double opt_distance;

    opt_heading = _optimal_heading.get();
    opt_distance = _optimal_heading_distance.get();

    output_command = merged_command.command;
    linear = merged_command.command.linear;

    if(do_optimal_heading && !(base::isUnset(linear[0]) || base::isUnset(linear[1]))){
        //Set z to 0, to use only x and y fpr the distance

        linear[2] = 0.0;


        
        if(_always_forward.get()){
            output_command.x() = linear.norm();
        }
        output_command.y() = 0;
        output_command.yaw() = base::Angle::normalizeRad(atan2(merged_command.command.linear(1), 
                                                               merged_command.command.linear(0))
                                                         + opt_heading);

        if(linear.norm() < opt_distance){
            std::cout << "ERREICHT!" << std::endl;
            do_optimal_heading = false;
        }

    }
   
    _do_optimal_heading.write(do_optimal_heading);
    //write the command
    _cmd_out.write(output_command);

    return true;
}
