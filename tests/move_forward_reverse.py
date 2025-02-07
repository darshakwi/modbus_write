import rospy
from std_msgs.msg import String

def send_commands(iterations):
    # Set the topic name
    topic = "crth_handler/mcu_command"

    # Initialize ROS node and publisher
    rospy.init_node('command_sender', anonymous=True)
    publisher = rospy.Publisher(topic, String, queue_size=10)

    # Define the two commands
    # command1 = "MA:2000,26,300,2000,27,300,2000,29,300,0,2,0,"
    # command2 = "MA:0,26,540,0,27,600,0,29,700,0,2,0,"

    command1 = "MX:2000,26,800,"
    command2 = "MX:0,26,800,"

    rate = rospy.Rate(0.4)  # Sending rate (1 Hz)

    for i in range(iterations):
        if rospy.is_shutdown():
            break

        rospy.loginfo(f"Iteration {i + 1}:")

        rate.sleep()
        # Send the first command
        rospy.loginfo(f"Sending command: {command1}")
        publisher.publish(command1)
        
        rate.sleep()
        # Send the second command
        rospy.loginfo(f"Sending command: {command2}")
        publisher.publish(command2)
        

if __name__ == "__main__":
    try:
        # Ask the user for the number of iterations
        iterations = int(input("Enter the number of iterations: "))

        if iterations <= 0:
            print("Please enter a positive number for iterations.")
        else:
            send_commands(iterations)
    except ValueError:
        print("Invalid input. Please enter a valid number for iterations.")
    except rospy.ROSInterruptException:
        pass