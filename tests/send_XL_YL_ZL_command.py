import rospy
from std_msgs.msg import String  # Replace 'String' with the appropriate message type for your topic

def publish_command():
    rospy.init_node('command_publisher', anonymous=True)
    topic_name = '/mcu_read/calib_feedback'  # Replace with your ROS topic name
    publisher = rospy.Publisher(topic_name, String, queue_size=10)

    commands = ["XL:", "YL:", "ZL:"]
    print("ROS Command Publisher initialized.")
    print("Available commands:")
    for i, cmd in enumerate(commands, 1):
        print(f"{i}: {cmd}")

    try:
        while not rospy.is_shutdown():
            print("\nPress Enter to send a command.")
            input("")
            print("Select a command to send:")

            for i, cmd in enumerate(commands, 1):
                print(f"{i}: {cmd}")

            choice = input("Enter the command number (1-3): ")

            try:
                choice = int(choice)
                if 1 <= choice <= len(commands):
                    command = commands[choice - 1]
                    publisher.publish(command)
                    print(f"Published: {command}")
                else:
                    print("Invalid choice. Please select a number between 1 and 3.")
            except ValueError:
                print("Invalid input. Please enter a number between 1 and 3.")
    except rospy.ROSInterruptException:
        print("ROS Node interrupted.")

if __name__ == '__main__':
    publish_command()
