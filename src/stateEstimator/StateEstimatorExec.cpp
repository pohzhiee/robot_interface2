#include "robot_interface2/stateEstimator/StateEstimator.hpp"
#include <ecal/ecal.h>
#include <iostream>

int main()
{
    // initialize eCAL API
    eCAL::Initialize({}, "Robot1 State Estimator");

    // set process state
    eCAL::Process::SetState(proc_sev_healthy, proc_sev_level1, "Robot1 State Estimator");

    auto a = robot_interface2::StateEstimator();
    while (eCAL::Ok())
    {
        eCAL::Process::SleepMS(100);
    }

    std::cout << "FINISHED" << std::endl;
    eCAL::Finalize();
}