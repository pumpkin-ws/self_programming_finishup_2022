#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface_doc.h>
#include <vector>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

class URMover
{
public:
    //constructor

    ~URMover(){
        delete rtde_control;
    }

    explicit URMover(std::string _UR_IP) : UR_IP(_UR_IP)
    {
        this->rtde_control = new RTDEControlInterface(UR_IP);
    }


    void SpeedJ(std::vector<double> joint_pose
                                    ,double acceleration=0.5
                                    ,double time = 0.0)
    {
        for (unsigned int i=0; i<10; i++)
        {
            rtde_control->speedJ(joint_pose, acceleration, time);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        rtde_control->speedStop();
        
    }

    void SpeedL(std::vector<double> tool_speed
                                    ,double acceleration=0.25
                                    ,double time = 0.0)
    {
        for (unsigned int i=0; i<10; i++)
        {
            rtde_control->speedL(tool_speed, acceleration, time);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        rtde_control->speedStop();
    }

    void MoveJ(std::vector<double> position
                                    ,double speed = 1.05
                                    ,double acceleration=1.4)
    {
        rtde_control->moveJ(position, speed, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        rtde_control->stopJ(0.5);
    }

    void MoveJ_Blend(std::vector<double> position
                                            ,double speed = 1.05
                                            ,double acceleration = 1.4
                                            ,double blend = 0)
    {
        position.push_back(speed);
        position.push_back(acceleration);
        position.push_back(blend);
        std::vector<std::vector<double>> path;
        path.push_back(position);
        rtde_control->moveJ(path);
        rtde_control->stopScript();
    }                                        

    void MoveL(std::vector<double> pose
                                    ,double speed=0.25
                                    ,double acceleration=1.2)
    {
        rtde_control->moveL(pose,speed,acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        rtde_control->stopL(0.5);
    }

private:
    std::string UR_IP = "192.168.1.11";
    RTDEControlInterface* rtde_control;
};
