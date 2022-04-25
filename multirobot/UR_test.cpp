#include "source/ur_move.hpp"

int main(int argc, char** argv) {
    URMover ur("192.168.1.1");
    URMover::JointPose current_jp = ur.getRTJointPose();
    URMover::ToolPose current_tp = ur.getRTCartPose();

    return EXIT_SUCCESS;
}