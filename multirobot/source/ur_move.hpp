#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface_doc.h>
#include <vector>
#include <thread>
#include <chrono>

#include "Eigen/Core"
#include "Eigen/Dense"

using namespace ur_rtde;
using namespace std::chrono;

class URMover
{
public:
    //constructor
    struct JointPose {
        double j1{0}, j2{0}, j3{0}, j4{0}, j5{0}, j6{0};
    };
    struct ToolPose{
        double x{0}, y{0}, z{0};
        double qw{0}, qx{0}, qy{0}, qz{0};
    };

    ~URMover(){
        delete rtde_control;
    }

    explicit URMover(std::string _UR_IP) : UR_IP(_UR_IP)
    {
        this->rtde_control = new RTDEControlInterface(UR_IP);
        this->rtde_receive = new RTDEReceiveInterface(UR_IP);
    }

    /**
     * @brief Get the current joint angles, RT stands for real time
     * The angle units are radians
     * 
     */
    JointPose getRTJointPose() {
        std::vector<double> joints = rtde_receive->getActualQ();
        JointPose jp;
        jp.j1 = joints[0];
        jp.j2 = joints[1];
        jp.j3 = joints[2];
        jp.j4 = joints[3];
        jp.j5 = joints[4];
        jp.j6 = joints[5];
        return jp;
    }

    /**
     * @brief Get the cartesian pose of the tool pose, 
     * 
     * @return CartPose 
     */
    ToolPose getRTCartPose() {
        std::vector<double> cart_pos = rtde_receive->getActualTCPPose();
        ToolPose tp;
        tp.x = cart_pos[0];
        tp.y = cart_pos[1];
        tp.z = cart_pos[2];

        Eigen::Vector3d rvec{cart_pos[3], cart_pos[4], cart_pos[5]};
        double theta = rvec.norm();
        rvec.normalized();
        Eigen::AngleAxisd aa_rvec{rvec.norm(), rvec};
        Eigen::Quaterniond quat(aa_rvec);
        tp.qw = quat.w();
        tp.qx = quat.x();
        tp.qy = quat.y();
        tp.qz = quat.z();
        return tp;
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
        rtde_control->moveL(pose, speed, acceleration);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        rtde_control->stopL(0.5);
    }

private:
    std::string UR_IP = "192.168.1.11";
    RTDEControlInterface* rtde_control;
    RTDEReceiveInterface* rtde_receive;
};
