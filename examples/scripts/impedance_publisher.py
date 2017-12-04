from XCM import msg
import rospy

if __name__ == '__main__':

    rospy.init_node('impedance_pub')

# replace 'centauro' with the actual robot name ('bigman' for WALKMAN)
    pub = rospy.Publisher('/xbotcore/bigman/command', msg.CommandAdvr, queue_size='1')
    sub = rospy.Subscriber('/xbotcore/bigman/joint_states', msg.JointStateAdvr)

# read a joint state
    joint_state = msg.JointStateAdvr()
    joint_state = rospy.wait_for_message('/xbotcore/bigman/joint_states', msg.JointStateAdvr, 1.0)

    print(joint_state)

    m = msg.CommandAdvr()

# define a list of joint names to which you want to send commands
    m.name.append('j_arm1_4')
    m.name.append('j_arm2_4')

# choose the stiffness level to command
    m.stiffness.append(joint_state.stiffness[joint_state.name.index('j_arm1_4')])
    m.stiffness.append(4000)

# choose the stiffness level to command
    m.damping.append(10)
    m.damping.append(10)

# choose the joint position to command
    m.position.append(-1.0)
    m.position.append(1.0)

    while not rospy.is_shutdown():
        pub.publish(m)


