
# #!/usr/bin/env python3
# import rospy
# import sys
# import select
# from std_msgs.msg import String

# def send_motor_command(pub, command):
#     rospy.loginfo(f"Publishing command: {command}")
#     pub.publish(command)
#     rospy.sleep(1)  # Give time for message to be received

# if __name__ == '__main__':
#     try:
#         rospy.init_node('test_motor_command_publisher', anonymous=True)
#         pub = rospy.Publisher('crth_handler/mcu_command', String, queue_size=10)
        
#         rospy.sleep(1)  # Allow publisher to register with ROS

#         print("Press ENTER to stop...")

#         while not rospy.is_shutdown():
#             # send_motor_command(pub, "MX:3000,20,800,")
#             # send_motor_command(pub, "MA:2000,26,540,4000,27,600,5000,29,700,200,2,0,")
#             # send_motor_command(pub, "MA:3000,25,1000,5000,25,1000,1000,25,1000,200,2,0,")
#             # send_motor_command(pub, "MY:5000,20,1000,")
#             send_motor_command(pub, "MZ:1000,20,1200,")
#             # send_motor_command(pub, "CS")


#             # Check if Enter is pressed (non-blocking)
#             if select.select([sys.stdin], [], [], 0)[0]:  
#                 sys.stdin.read(1)  # Read the Enter key input
#                 print("\nStopping...")
#                 break  # Exit loop

#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3
import rospy
import sys
import select
from std_msgs.msg import String

def send_motor_command(pub, value_x, value_y, value_z):
    # command = f"MA:{value_x},20,800,{value_y},20,800,{value_z},20,800,0,2,0,"
    command = f"MX:{value_x},20,800,"
    rospy.loginfo(f"Publishing command: {command}")
    pub.publish(command)
    rospy.sleep(1)  # Give time for message to be processed

if __name__ == '__main__':
    try:
        rospy.init_node('test_motor_command_publisher', anonymous=True)
        pub = rospy.Publisher('crth_handler/mcu_command', String, queue_size=10)
        
        rospy.sleep(1.5)  # Allow publisher to register with ROS

        mx_value = 1000  # Start value
        my_value = 1000
        mz_value = 1000
        print("Press ENTER to stop...")

        while not rospy.is_shutdown():
            send_motor_command(pub, mx_value, my_value, mz_value)
            mx_value -= 3000  # Increase first value by 1000
            my_value += 2000
            mz_value += 2000

            # Check if Enter is pressed (non-blocking)
            if select.select([sys.stdin], [], [], 0)[0]:  
                sys.stdin.read(1)  # Read Enter key input
                print("\nStopping...")
                break  # Exit loop

    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python3
# import rospy
# from std_msgs.msg import String

# def send_motor_command():
#     rospy.init_node('test_motor_command_publisher', anonymous=True)
#     pub = rospy.Publisher('crth_handler/mcu_command', String, queue_size=10)
#     rate = rospy.Rate(0.2)  # 0.2 Hz

#     base_command = [1000,26,200,1000,27,200,1000,29,200,400,2,0]
#     increment_value = 1000
#     increment_s_value = 2
#     increment_sd_value = 3
#     iterations = 5

#     for i in range(2 * iterations):  # 5 increment + 5 decrement iterations
#         # Determine whether to increment or decrement
#         is_increment = i < iterations
#         modifier = increment_value if is_increment else -increment_value
#         modifier_s = increment_s_value if is_increment else -increment_s_value
#         modifier_sd = increment_sd_value if is_increment else -increment_sd_value

#         # Modify the required elements (1st, 4th, 7th)
#         base_command[0] += modifier
#         base_command[1] += modifier_s
#         base_command[2] += modifier_sd
#         base_command[3] += modifier
#         base_command[4] += modifier_s
#         base_command[5] += modifier_sd
#         base_command[6] += modifier
#         base_command[7] += modifier_s
#         base_command[8] += modifier_sd

#         # Format the command as a string
#         command_str = "MA:" + ",".join(map(str, base_command)) + ","

#         # Publish the command
#         rospy.loginfo(f"Publishing command: {command_str}")
#         pub.publish(command_str)

#         # Wait for the next iteration
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         send_motor_command()
#     except rospy.ROSInterruptException:
#         pass
