#include "pub_connection.h"
using namespace ForceMasterPub;

void PubConnection::handlePubMessage(char* buffer, int &data_len)
{
    JointPosition joint;
    joint.set_joint_num(6);
    joint.set_joint(0, 0.1);
    joint.set_joint(1, 0.2);
    joint.set_joint(2, 0.3);
    joint.set_joint(3, 0.4);
    joint.set_joint(4, 0.5);
    joint.set_joint(5, 0.6);

    CartPose pose;
    pose.set_x(0.11);
    pose.set_y(0.12);
    pose.set_z(0.13);
    pose.set_a(0.14);
    pose.set_b(0.15);
    pose.set_c(0.16);

    PublishData publish_data;
    publish_data.set_allocated_joint(&joint);
    publish_data.set_allocated_pose(&pose);

    publish_data.SerializeToArray(buffer, publish_data.ByteSize());
    data_len += publish_data.ByteSize();

    publish_data.release_joint();
    publish_data.release_pose();
}