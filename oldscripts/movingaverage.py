import rospy
import geometry_msgs.msg
history = [[],[],[],[],[],[]]
average = [0,0,0,0,0,0]
def callback(msg):
    global history
    history[0].append(msg.wrench.force.x)
    history[1].append(msg.wrench.force.y)
    history[2].append(msg.wrench.force.z)
    history[3].append(msg.wrench.torque.x)
    history[4].append(msg.wrench.torque.y)
    history[5].append(msg.wrench.torque.z)
    # print(history)
    for i, value in enumerate(history):
       if len(history[i]) > 60:
        history[i] = history[i][-60:]
        
        average[i] = round(sum(history[i]) / float(len(history[i])), 1)
        print(average)
        #rospy.loginfo('Average of most recent {} samples: {}'.format(len(history[i])), average[i])


n = rospy.init_node('moving_average')
bota_listener = rospy.Subscriber("/ft_sensor/ft_compensated", geometry_msgs.msg.WrenchStamped, callback)

rospy.spin()


