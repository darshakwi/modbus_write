/**
 * @file   modbus_write_node.cpp
 * @brief  Initializes ROS cpp Modbus Write Node
 * @author Darshak Vaghela
 * @date   2025-01-02
 * *****************************************************************/

#include "modbus_write/modbus_write.h"

/// @brief Main function to handle ROS Node working
int main (int argc, char** argv) {
    // Set this node's logger level here ('Debug', 'Info', 'Warn', 'Error', 'Fatal')
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug/*Set Level Here*/))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    /// Initialize ROS Node with name remapping provision
    ros::init(argc, argv, "modbus_write_node");

    // Initialize NodeHandle
    ros::NodeHandle nh;

    // Instantiate ModbusWrite class object
    ModbusWrite modbus_write_node (&nh);

    /// Loop for calling message callbacks
    ros::spin();

    return 0;
}