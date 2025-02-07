/**
 * @file   modbus_write_node.cpp
 * @brief  Source file containing ModbusWrite Class definitions.
 * @author Darshak Vaghela
 * @date   2025-01-02
 * *****************************************************************/

#include "modbus_write/modbus_write.h"

ModbusWrite::ModbusWrite(ros::NodeHandle *nodehandle)
    // : nh_(*nodehandle), io_(new boost::asio::io_service), my_port(*io_) {
        : nh_(*nodehandle), io_(std::make_unique<boost::asio::io_context>()), my_port(std::make_unique<boost::asio::serial_port>(*io_)) {
    // Load Namespace
    namespace_str = nh_.getNamespace();
    ROS_DEBUG("Modbus Write : Namespace : %s", namespace_str.c_str());

    size_t found = nh_.getNamespace().find("robot");

    if (found != std::string::npos) {
        robot_num = std::stoi(nh_.getNamespace().substr(6));
    } else {
        robot_num = -1;
    }

    ROS_INFO_STREAM("Modbus Write : Robot Number : " << robot_num);

    initializePublishers();
    initializeSerialPort();
    initializeParameters();
    initializeSubscribers();
    initializeTimers();
    // startParameterReadingTimer();

    ROS_INFO("Node has Started.");
}

// ModbusWrite Class Destructer
ModbusWrite::~ModbusWrite()
{
        // Close the serial port if it's open
    if (my_port && my_port->is_open()) {
        my_port->close();
        ROS_INFO("MCU Feedback : Serial port closed.");
    }else{
        ROS_INFO("MCU Feedback : Serial port closed in previous state");

    }
    // delete io_;
}

// Initializing ROS Publishers
void ModbusWrite::initializePublishers()
{
    /* Publihsers for ZC time comparison */
    MA_time_pub = nh_.advertise<hope_msgs::TimeComparison>("modbus_write/ma_time", 100);
    MZ_time_pub = nh_.advertise<hope_msgs::TimeComparison>("modbus_write/mz_time", 100);
    bi_time_pub = nh_.advertise<hope_msgs::TimeComparison>("modbus_write/bi_time", 100);
    ti_time_pub = nh_.advertise<hope_msgs::TimeComparison>("modbus_write/ti_time", 100);
    bi_delay_pub = nh_.advertise<std_msgs::Int64>("modbus_write/bi_delay", 100);
    ti_delay_pub = nh_.advertise<std_msgs::Int64>("modbus_write/ti_delay", 100);
    motion_complete_pub = nh_.advertise<std_msgs::Int64>("modbus_write/motion_complete", 100);
    modbus_running_data_pub = nh_.advertise<hope_msgs::ModbusRobotFeedback>("modbus_write/modbus_running_data_feedback", 10);
    write_active_pub = nh_.advertise<std_msgs::Bool>("modbus_write/modbus_write_active", 10);

    /* Publisher for error codes*/
    crt_ros_info_pub_ = nh_.advertise<std_msgs::String>("crth_handler/crt_ros_gui_info", 10);

}

// Initializing the port
void ModbusWrite::initializeSerialPort() {

    /* Getting parameters from param server */
    ROS_WARN ("Mcu Write : Using UART protocol for MCU communication");
    getParameter("uart/port", port, "/dev/ttyACM1");
    getParameter<int>("uart/baudrate", baudrate, 115200);

    ROS_DEBUG_STREAM("Mcu Write : Baudrate : " << baudrate);
    ROS_DEBUG_STREAM("Mcu Write : Port     : " << port);

    try {
        /* Open the port */
        my_port->open(port);

        /* Setting options (baudrate , character size , stopbits and parity)*/
        my_port->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        my_port->set_option(boost::asio::serial_port_base::character_size(8));
        my_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        my_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

        /* Serial Communication Initialised */
        ROS_INFO("Mcu Write : Serial Communication of mcu_write Initialised");
    }

    catch (const std::exception &e) {
        crt_ros_gui_info_publish("UNPORT");
        ROS_ERROR_STREAM("Mcu Write : Unable to open serial Port " << e.what());
        // Handle errors by closing and reopening the port
        if (my_port->is_open()) {
        my_port->close();
        ROS_ERROR_STREAM("Mcu Writer : Closing Serial port");
        }
    }
}

// Initializing ROS Parameters
void ModbusWrite::initializeParameters()
{
    ROS_INFO("Initializing Parameters");
    getParameter("modbus/port", modbus_port, "/dev/ttyACM0");
    getParameter<int>("modbus/baudrate", modbus_baudrate, 115200);
    getParameter("modbus/parity", modbus_parity, "N");
    getParameter<int>("modbus/stopbits", modbus_stopbits, 1);
    getParameter<int>("modbus/bytesize", modbus_bytesize, 8);
    getParameter<int>("modbus/timeout", modbus_timeout, 5);
    getParameter<int>("parameters/pulse_per_rotation", pulse_per_rotation, 1000);
    getParameter<int>("parameters/position_target_low_regr", position_target_low_regr, 460);
    getParameter<int>("parameters/position_target_high_regr", position_target_high_regr, 461);
    getParameter<int>("parameters/motor_start_stop_regr", motor_start_stop_regr, 1210);
    getParameter<int>("parameters/motion_rotation_dir_regr", motion_rotation_dir_regr, 101);
    getParameter<int>("parameters/position_acceleration_time_regr", position_acceleration_time_regr, 463);
    getParameter<int>("parameters/position_deceleration_time_regr", position_deceleration_time_regr, 464);
    getParameter<int>("parameters/position_speed_rpm_regr", position_speed_rpm_regr, 462);
    getParameter<int>("parameters/motor_speed", motor_speed, 1301);
    getParameter<int>("read_parameters/position_feedback_counter", position_feedback_counter, 1311);
    getParameter<double>("encoder_value/x/gear_box_driver", gear_box_driver_x, 1);
    getParameter<double>("encoder_value/x/gear_box_driven", gear_box_driven_x, 1);
    getParameter<double>("encoder_value/x/pulley_pitch", pulley_pitch_x, 8);
    getParameter<double>("encoder_value/x/pulley_no_of_teeth", pulley_no_of_teeth_x, 18);
    getParameter<double>("encoder_value/x/encoder_resolution", encoder_resolution_x, 131072);
    getParameter<double>("encoder_value/y/gear_box_driver", gear_box_driver_y, 1);
    getParameter<double>("encoder_value/y/gear_box_driven", gear_box_driven_y, 1);
    getParameter<double>("encoder_value/y/pulley_pitch", pulley_pitch_y, 8);
    getParameter<double>("encoder_value/y/pulley_no_of_teeth", pulley_no_of_teeth_y, 18);
    getParameter<double>("encoder_value/y/encoder_resolution", encoder_resolution_y, 131072);
    getParameter<double>("encoder_value/z/gear_box_driver", gear_box_driver_z, 1);
    getParameter<double>("encoder_value/z/gear_box_driven", gear_box_driven_z, 1);
    getParameter<double>("encoder_value/z/pulley_pitch", pulley_pitch_z, 8);
    getParameter<double>("encoder_value/z/pulley_no_of_teeth", pulley_no_of_teeth_z, 18);
    getParameter<double>("encoder_value/z/encoder_resolution", encoder_resolution_z, 131072);

    if (!nh_.getParam("modbus/unit_ids", unit_ids_)) {
      ROS_WARN("Parameter 'modbus/unit_ids' not found, using default [1].");
      unit_ids_ = {1};  // Default value
    }

    ROS_INFO("Modbus configuration loaded successfully:");
    ROS_INFO("Port: %s, Baudrate: %d, Parity: %s, Stopbits: %d, Bytesize: %d, Timeout: %d",
             modbus_port.c_str(), modbus_baudrate, modbus_parity.c_str(),
             modbus_stopbits, modbus_bytesize, modbus_timeout);

    ROS_INFO("Unit IDs: ");
    for (int id : unit_ids_) {
        ROS_INFO("%d", id);
    }

    ROS_INFO("Motor registers initialized.");

    // Initialize Modbus contexts
    driver_ctxs.resize(unit_ids_.size());
    for (size_t i = 0; i < unit_ids_.size(); ++i) {
        ROS_INFO("Initializing Modbus context for motor %d on port: %s", unit_ids_[i], modbus_port.c_str());

        // Create and configure the Modbus RTU context
        modbus_t* ctx = modbus_new_rtu(modbus_port.c_str(), modbus_baudrate, modbus_parity[0], modbus_bytesize, modbus_stopbits);
        if (ctx == nullptr) {
            ROS_ERROR("Failed to allocate Modbus context for motor %d", unit_ids_[i]);
            cleanupContexts(); // Clean up any previously created contexts
            ros::shutdown();
        }

        // Set response timeout
        modbus_set_response_timeout(ctx, modbus_timeout / 1000, (modbus_timeout % 1000) * 1000);

        // Set the Modbus slave ID
        if (modbus_set_slave(ctx, unit_ids_[i]) == -1) {
            ROS_ERROR("Failed to set slave ID %d: %s", unit_ids_[i], modbus_strerror(errno));
            modbus_free(ctx);
            cleanupContexts();
            ros::shutdown();
        }

        // Connect to the Modbus slave
        if (modbus_connect(ctx) == -1) {
            ROS_ERROR("Failed to connect to motor %d: %s", unit_ids_[i], modbus_strerror(errno));
            modbus_free(ctx);
            cleanupContexts();
            ros::shutdown();
        }

        // Store the context
        driver_ctxs[i] = ctx;
        ROS_INFO("Modbus context initialized for motor %d", unit_ids_[i]);
    }

    // motor_registers_value_.resize(3);

    ROS_INFO("All Modbus contexts initialized successfully.");
}

// Initialize subscribers
void ModbusWrite::initializeSubscribers() {
    /* Subscribers */
    handware_handler_sub = nh_.subscribe<std_msgs::String>(
      "crth_handler/mcu_command", 10, &ModbusWrite::mcu_command_cb, this,
      ros::TransportHints().tcpNoDelay().maxDatagramSize(100)); // Topic change

    mcu_feedback_sub = nh_.subscribe("mcu_read/calib_feedback", 10, &ModbusWrite::mcu_read_data, this);
}

void ModbusWrite::initializeTimers() {
    resume_timer = nh_.createTimer(ros::Duration(0), &ModbusWrite::resumeCallback, this, true, false);
}

void ModbusWrite::startParameterReadingTimer() {
    // Default frequency
    double frequency; // = 50;  // Default: 0.2 Hz

    // Load frequency from the parameter server
    nh_.param<double>("modbus/parameter_reading_frequency", frequency, 100);
    
    // Validate frequency
    if (frequency <= 0.0) {
        ROS_ERROR("Invalid frequency value: %f. Frequency must be greater than 0. Using default: 0.2 Hz", frequency);
        frequency = 0.2; // Reset to default
    }

    // Calculate timer interval
    ros::Duration interval(1.0 / frequency);

    // Start the timer
    motor_status_ = nh_.createTimer(interval, &ModbusWrite::motorStatusReadCallback, this);
    ROS_INFO("Parameters reading timer started with a frequency of %f Hz (interval: %f seconds).", frequency, interval.toSec());
}

void ModbusWrite::cleanupContexts() {

    // Join the thread if it exists and is joinable
    if (motor3_thread.joinable()) {
        motor3_thread.join();
    }

    for (auto& ctx : driver_ctxs) {
        if (ctx) {
            modbus_close(ctx);
            modbus_free(ctx);
        }
    }
    driver_ctxs.clear();
}

void ModbusWrite::resumeCallback(const ros::TimerEvent&) {
    std_msgs::Bool msg;
    msg.data = false;
    write_active_pub.publish(msg);
    ROS_INFO("Sent 'false' -> Resuming modbus_read");
}

void ModbusWrite::reconnectSerialPort() {
  try {
    // Ensure the port is closed
    if (my_port->is_open()) {
        my_port->close();
        ROS_WARN("Mcu Write : Closed MCU UART Connection");
    }

    // Add a short delay to allow the system to release resources
    ros::Duration(1).sleep();

    // Reset the serial port object
    my_port.reset(new boost::asio::serial_port(*io_));

    // Attempt to reopen the port
    initializeSerialPort();
  }
  catch (const std::exception &e) {
    ROS_ERROR_STREAM("Mcu Write : Error during reconnection: " << e.what());
    ros::Duration(1).sleep(); // Wait and retry later
  }
}

bool ModbusWrite::crt_ros_gui_info_publish(std::string msg) {
  try {
    std_msgs::String pub_msg;
    pub_msg.data = "/robot" + std::to_string(robot_num) + ":" + msg;
    crt_ros_info_pub_.publish(pub_msg);
    ROS_DEBUG("Mcu Writer : %s", pub_msg.data.c_str());

    return true;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return false;
  }
}

int ModbusWrite::from_string_to_int_msg() {
  try {
    size_t colon_pos = command.find(':'); // find the position of the colon
    std::string delay_str = command.substr(colon_pos + 1);
    return std::stoi(delay_str);
  }

  catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
}

double ModbusWrite::calculateMotionTime(int target_position, int mini_speed_rpm, int max_speed_rpm, int accel_time_ms, int decel_time_ms) {
    double initialSPS = rpmToSPS(mini_speed_rpm, pulse_per_rotation);
    double maxSPS = rpmToSPS(max_speed_rpm, pulse_per_rotation);

    // Convert acceleration and deceleration times to seconds
    double accel_time_sec = accel_time_ms / 1000.0;
    double decel_time_sec = decel_time_ms / 1000.0;

    // Acceleration rate (steps per second²)
    double accel_rate = (maxSPS - initialSPS) / accel_time_sec;
    double decel_rate = (maxSPS - initialSPS) / decel_time_sec;

    // Distance covered during acceleration
    double d_accel = (initialSPS * accel_time_sec) + (0.5 * accel_rate * accel_time_sec * accel_time_sec);
    
    // Distance covered during deceleration
    double d_decel = (maxSPS * decel_time_sec) - (0.5 * decel_rate * decel_time_sec * decel_time_sec);

    // Ensure distance covered in accel + decel is not greater than target
    double d_constant = target_position - (d_accel + d_decel);
    if (d_constant < 0) d_constant = 0; // Avoid negative values

    // Time at max speed
    double t_constant = (d_constant > 0) ? (d_constant / maxSPS) : 0;

    // Total motion time (in seconds)
    double total_time_sec = accel_time_sec + t_constant + decel_time_sec;

    // Convert to milliseconds
    return total_time_sec * 1000.0; // Returns time in milliseconds
}


// Function to convert RPM to Steps Per Second (SPS)
double ModbusWrite::rpmToSPS(double rpm, int ppr) {
    return (rpm / 60.0) * ppr;
}

// Function to handle motor control loop
void ModbusWrite::motorControlLoop(modbus_t *ctx, int target_position, int mini_speed_rpm, int max_speed_rpm, int direction) {
    target_low = target_position & 0xFFFF;
    target_high = (target_position >> 16) & 0xFFFF;

    std::cout << "Position: " << target_position << std::endl;
    std::cout << "Minimum speed: " << mini_speed_rpm << std::endl;
    std::cout << "Maximum speed: " << max_speed_rpm << std::endl;
    std::cout << "Direction: " << direction << std::endl;

    // Convert RPM to SPS
    double initialSPS = rpmToSPS(mini_speed_rpm, pulse_per_rotation);
    double maxSPS = rpmToSPS(max_speed_rpm, pulse_per_rotation);

    // Total time to complete acceleration/deceleration
    double totalTime = (target_position / ((initialSPS + maxSPS) / 2)) * 1000;

    // Split the time into acceleration and deceleration
    accel_time = totalTime / 6;
    decel_time = totalTime / 6;


    // Print the results
    std::cout << "Initial Speed (SPS): " << initialSPS << " steps/sec" << std::endl;
    std::cout << "Maximum Speed (SPS): " << maxSPS << " steps/sec" << std::endl;
    std::cout << "Total Time: " << totalTime << " ms" << std::endl;
    std::cout << "Acceleration Time: " << accel_time << " ms" << std::endl;
    std::cout << "Deceleration Time: " << decel_time << " ms" << std::endl;

    double time_needed = calculateMotionTime(target_position, mini_speed_rpm, max_speed_rpm, accel_time, decel_time);
    std::cout << "Total motion time: " << time_needed << " ms" << std::endl;

    modbus_flush(ctx);

    // Write motor commands
    if (modbus_write_register(ctx, position_target_low_regr, target_low) == -1 ||
        modbus_write_register(ctx, position_target_high_regr, target_high) == -1 ||
        modbus_write_register(ctx, position_speed_rpm_regr, max_speed_rpm) == -1 ||
        modbus_write_register(ctx, motion_rotation_dir_regr, direction) == -1 ||
        modbus_write_register(ctx, position_acceleration_time_regr, static_cast<int>(accel_time)) == -1 ||
        modbus_write_register(ctx, position_deceleration_time_regr, static_cast<int>(decel_time)) == -1) {
        std::cerr << "Failed to write motor control registers: " << modbus_strerror(errno) << std::endl;
        return;
    }

    // Start the motor
    if (modbus_write_register(ctx, motor_start_stop_regr, 1) == -1) {
        std::cerr << "Failed to start motor: " << modbus_strerror(errno) << std::endl;
        return;
    }
}

void ModbusWrite::motorStatusReadCallback(const ros::TimerEvent&){
    if (tcp_active) {
        ROS_WARN("Skipping Modbus read: TCP command in progress");
        return;
    }

    // std::vector<int> motor_ids = {1, 2, 3};  // Define motor IDs
    for (int id : unit_ids_) {
        uint16_t value16;
        if (readData(id, motor_speed, &value16)) {
            // feedback_msg.motor_speed_value = value16;
            ROS_INFO("Motor ID %d: Motor speed Value = %d", id, value16);

            if (value16 == 0) { 
                // Motor just stopped, send message once
                if (motor_was_running[id]) {
                    ROS_WARN("Motor ID %d has stopped!", id);
                    // Publish stop message here
                    std_msgs::String stop_msg;
                    stop_msg.data = "Motor " + std::to_string(id) + " has stopped.";
                    // motor_stop_pub.publish(stop_msg);

                    motor_was_running[id] = false;  // Mark motor as stopped
                }
            } 
            else {
                motor_was_running[id] = true;  // Motor is running
            }
        } 
        else {
            ROS_WARN("Failed to read motor speed value for Motor ID %d", id);
        }
    }
}

void ModbusWrite::mcu_read_data(const std_msgs::String::ConstPtr& msg) {

    tcp_active = true;  // Block Modbus read while processing TCP command

    ROS_DEBUG("Received Feedback: %s", msg->data.c_str());
    std::string calib_code = msg->data;

    uint32_t value32 = 0;

    if(calib_code.find("XL:") == 0) {
        readData(driver_id_x, position_feedback_counter, &value32, true);

        mm_per_encoder_counts_x = (gear_box_driver_x/gear_box_driven_x) * (pulley_pitch_x*pulley_no_of_teeth_x) / encoder_resolution_x;
        position_mm_x = (value32) * mm_per_encoder_counts_x;
        encoder_x_starting_value = position_mm_x;
        position_mm_x = 0;

        ROS_DEBUG("encoder_x_starting_value : %u", encoder_x_starting_value);
        ROS_DEBUG("encoder x value : %u", value32);
    }
    else if(calib_code.find("YL:") == 0) {
        readData(driver_id_y, position_feedback_counter, &value32, true);

        mm_per_encoder_counts_y = (gear_box_driver_y/gear_box_driven_y) * (pulley_pitch_y*pulley_no_of_teeth_y) / encoder_resolution_y;
        position_mm_y = (value32) * mm_per_encoder_counts_y;
        encoder_y_starting_value = position_mm_y;
        position_mm_y = 0;

        ROS_DEBUG("encoder_y_starting_value : %u", encoder_y_starting_value);
        ROS_DEBUG("encoder y value : %u", value32);
    }
    else if(calib_code.find("ZL:") == 0) {
        readData(driver_id_z, position_feedback_counter, &value32, true);

        mm_per_encoder_counts_z = (gear_box_driver_z/gear_box_driven_z) * (pulley_pitch_z * pulley_no_of_teeth_z) / encoder_resolution_z;
        position_mm_z = (value32) * mm_per_encoder_counts_z;
        encoder_z_starting_value = position_mm_z;
        position_mm_z = 0;

        ROS_DEBUG("encoder_z_starting_value : %u", encoder_z_starting_value);
        ROS_DEBUG("encoder z value : %u", value32);
    }
    tcp_active = false;  // Allow Modbus reads again
    ROS_INFO("TCP command processing complete. Modbus read re-enabled.");
}

// Function to calculate the fractional rotation value
double ModbusWrite::getRotationFraction(uint32_t encoderValue, int isForward) {
    // Handle the case when the encoder value is greater than 2^17, for multiple rotations

    ROS_DEBUG("directoion2 : %d", isForward);
    double rotationFraction = (double)encoderValue / 131072;

    // Adjust rotation based on direction
    if (!isForward) {
        rotationFraction = 131072 - rotationFraction; // Reverse direction
    }

    return rotationFraction;
}

// Function to get the actual rotation count (in terms of full rotations)
double ModbusWrite::getActualRotation(uint32_t encoderValue, int isForward) {

    ROS_DEBUG("directoion1 : %d", isForward);
    double rotationFraction = getRotationFraction(encoderValue, isForward);
    // Calculate the total number of full rotations (integer part) and the fractional part
    double actualRotation = floor(rotationFraction);
    double fractionalPart = rotationFraction - actualRotation;

    return actualRotation + fractionalPart;
}

//
int32_t ModbusWrite::updateEncoder(uint32_t current_encoder, uint32_t last_encoder_value, int direction, int32_t adjusted_encoder){

        // Calculate the difference
        int32_t difference = static_cast<int32_t>(current_encoder - last_encoder_value);

        // Adjust for direction (reverse if direction is 0)
        if (direction == 1) {
            difference *= -1;
        }

        // Update the adjusted encoder value
        adjusted_encoder += difference;

        return adjusted_encoder;
    
}
//

void ModbusWrite::publishMotorFeedback(int motor_id, hope_msgs::ModbusRunningParams& feedback_msg) {
    feedback_msg.header.stamp = ros::Time::now();
    feedback_msg.slave_num = motor_id;

    uint16_t value16 = 0;
    uint32_t value32 = 0;

    // Read dynamic parameters
    if (readData(motor_id, position_feedback_counter, &value32, true)) {
        if(motor_id == 1)
        {
            if (value32 > 4294967296) {
                std::cerr << "X Encoder value exceeds the maximum limit of 2^32." << std::endl;
                return;
            }

            mm_per_encoder_counts_x = (gear_box_driver_x/gear_box_driven_x) * (pulley_pitch_x * pulley_no_of_teeth_x) / encoder_resolution_x;

            if(direction_0 == 0){            
                // position_mm_x += (value32 - encoder_x_starting_value) * mm_per_encoder_counts_x;
                position_mm_x = (value32) * mm_per_encoder_counts_x - encoder_x_starting_value;
            }
            else if(direction_0 == 1){
                // position_mm_x -= (value32 - encoder_x_starting_value) * mm_per_encoder_counts_x;
                position_mm_x -= (value32) * mm_per_encoder_counts_x - encoder_x_starting_value;
            }

            feedback_msg.position_feedback_counter_value = position_mm_x;
            ROS_INFO("Motor ID %d: position_feedback_counter_x in mm = %f", motor_id, position_mm_x);
        }
        else if(motor_id == 2)
        {
            if (value32 > 4294967296) {
                std::cerr << "Y Encoder value exceeds the maximum limit of 2^32." << std::endl;
                return;
            }

            mm_per_encoder_counts_y = (gear_box_driver_y/gear_box_driven_y) * (pulley_pitch_y*pulley_no_of_teeth_y) / encoder_resolution_y;
            
            if(direction_3 == 1){            
                // position_mm_y = (value32 - encoder_y_starting_value) * mm_per_encoder_counts_y;
                position_mm_y = (value32) * mm_per_encoder_counts_y - encoder_y_starting_value;
            }
            else if(direction_3 == 0){
                // position_mm_y-= (value32 - encoder_y_starting_value) * mm_per_encoder_counts_y;
                position_mm_y -= (value32) * mm_per_encoder_counts_y - encoder_y_starting_value;
            }

            feedback_msg.position_feedback_counter_value = position_mm_y;
            ROS_INFO("Motor ID %d: position_feedback_counter_y in mm = %f", motor_id, position_mm_y);
        }
        else if(motor_id == 3)
        {
            if (value32 > 4294967296) {
                std::cerr << "Z Encoder value exceeds the maximum limit of 2^32." << std::endl;
                return;
            } 

            mm_per_encoder_counts_z = (gear_box_driver_z/gear_box_driven_z) * (pulley_pitch_z*pulley_no_of_teeth_z) / encoder_resolution_z;
            
            if(direction_6 == 1){            
                // position_mm_z = (value32 - encoder_z_starting_value) * mm_per_encoder_counts_z;
                position_mm_y = (value32) * mm_per_encoder_counts_y - encoder_y_starting_value;
            }
            else if(direction_6 == 0){
                // position_mm_z -= (value32 - encoder_z_starting_value) * mm_per_encoder_counts_z;
                position_mm_z -= (value32) * mm_per_encoder_counts_z - encoder_z_starting_value;
            }

            feedback_msg.position_feedback_counter_value = (position_mm_z);
            ROS_INFO("Motor ID %d: position_feedback_counter_z in mm = %f", motor_id, position_mm_z);
        }
        ROS_INFO("Motor ID %d: Encoder Value = %u", motor_id, value32);
    } else {
        ROS_WARN("Failed to read encoder value for Motor ID %d", motor_id);
    }
}

bool ModbusWrite::validateDriverIndex(int driver_index) {
    return driver_index > 0 && driver_index <= unit_ids_.size();
}

bool ModbusWrite::readData(int driver_index, uint16_t reg_address, uint16_t* value) {

    // Retrieve the Modbus context for the given driver
    modbus_t* context = driver_ctxs[driver_index - 1];

    ROS_INFO("Reading from unit ID: %d, Register: %d", driver_index, reg_address);

    // Perform the Modbus read operation
    if (modbus_read_registers(context, reg_address, 1, value) == -1) {
        ROS_ERROR("Failed to read 16-bit data from driver with unit ID %d at register %d", driver_index, reg_address);
        return false;
    }
    return true;
}

bool ModbusWrite::readData(int driver_index, uint16_t reg_address, uint32_t* value, bool is_32bit) {

    // // Validate the driver index
    // if (!validateDriverIndex(driver_index)) {
    //     ROS_ERROR("Invalid driver index: %d", driver_index);
    //     return false;
    // }

    // Retrieve the Modbus context for the given driver
    context = driver_ctxs[driver_index - 1];

    ROS_INFO("Reading 32-bit data from unit ID: %d, Starting Register: %d", driver_index, reg_address);

    // Read two consecutive registers (low and high word)
    uint16_t registers[2] = {0};
    if (modbus_read_registers(context, reg_address, 2, registers) == -1) {
        ROS_ERROR("Failed to read 32-bit data from driver with unit ID %d at register %d", driver_index, reg_address);
        return false;
    }

    // Combine the two 16-bit registers into a 32-bit value
    *value = (static_cast<uint32_t>(registers[1]) << 16) | registers[0];

    return true;
}

void ModbusWrite::mcu_command_cb(const std_msgs::String::ConstPtr &msg) {

    //To stop 'modbus_read' node
    std_msgs::Bool active_msg;
    active_msg.data = true;
    write_active_pub.publish(active_msg);  // Signal to stop reading

    tcp_active = true;  // Block Modbus read while processing TCP command
    /* Command to be written */
    command = msg->data;
    // parseAndProcessCommand(ctx, msg->data);

    ROS_DEBUG_STREAM("Serial Command : " << command);

    // UART WRITE
    if ((command.substr(0, 3) != "MA:") && (command.substr(0, 3) != "MX:") && (command.substr(0, 3) != "MY:") && (command.substr(0, 3) != "MZ:")){
        try {
            if (!my_port->is_open())
            {
                ROS_WARN("Mcu Write : Port not open, attempting to reconnect");
                reconnectSerialPort();
            }

            // Write to the serial port
            boost::asio::write(*my_port, boost::asio::buffer(command));
            ROS_DEBUG_STREAM("Mcu Write : Serial Command " << command << " Sent");

        }
        catch (const std::exception &e) {
            ROS_ERROR_STREAM("Mcu Write : Unable to send the command " << command << " to MCU "<< e.what());
            ROS_WARN("Mcu Write : Trying to Reconnect MCU");

            crt_ros_gui_info_publish("UNSEND");

            // Handle errors by closing and reopening the port
            reconnectSerialPort();

            // Send a dummy message
            try{
                boost::asio::write(*my_port, boost::asio::buffer("RECONNECTED"));
                ROS_WARN("Mcu Write : Serial Command RECONNECTED Sent");
            }
            catch (const std::exception &e) {
                ROS_WARN_STREAM("Mcu Write : Error sending Dummy message " << e.what());
            }
        }
    }
    else if(command.substr(0, 3) == "MA:"){
        // Remove the prefix and split the remaining string by commas
        data = command.substr(3);
        ss.clear();  // Clear any error flags
        ss.str("");  // Reset the contents of the stringstream
        ss.str(data);
        values.clear();  // Clear the previous values from the vector

        while (getline(ss, token, ',')) {
            if (!token.empty()) {
                values.push_back(stoi(token));
            }
        }

        // Clear the command after processing
        command.clear();

        // Ensure there are at least 12 elements (3 sets of motor commands + delay + 2 additional values)
        if (values.size() != 12) {
            std::cerr << "Insufficient values in command!" << std::endl;
            return;
        }

        // Extract motor X, Y, and Z parameters
        current_value_0 = values[0];
        mini_speed_x = values[1];
        max_speed_x = values[2];
        current_value_3 = values[3];
        mini_speed_y = values[4];
        max_speed_y = values[5];
        current_value_6 = values[6];
        mini_speed_z = values[7];
        max_speed_z = values[8];

        // Extract the delay for the Z motor
        z_delay_ms = values[9];

        // Extract the last two values for future use
        extra_value1 = values[10];
        extra_value2 = values[11];

        ROS_WARN("current_value_0 : %d", values[0]);
        ROS_WARN("mini_speed_x : %d", values[1]);
        ROS_WARN("max_speed_x : %d", values[2]);
        ROS_WARN("current_value_3 : %d", values[3]);
        ROS_WARN("mini_speed_y : %d", values[4]);
        ROS_WARN("max_speed_y : %d", values[5]);
        ROS_WARN("current_value_6 : %d", values[6]);
        ROS_WARN("mini_speed_z : %d", values[7]);
        ROS_WARN("max_speed_z : %d", values[8]);
        ROS_WARN("z_delay_ms : %d", values[9]);
        ROS_INFO("Extra Value 1: %d", values[10]);
        ROS_INFO("Extra Value 2: %d", values[11]);

        // Compare with previous values and determine directions

        target_x = current_value_0 - prev_value_0;
        target_y = current_value_3 - prev_value_3;
        target_z = current_value_6 - prev_value_6;

        direction_0 = (target_x) < 0 ? 1 : 0;
        direction_3 = (target_y) < 0 ? 1 : 0;
        direction_6 = (target_z) < 0 ? 1 : 0;

        if(direction_0)
        {
            target_x = -target_x;
        }
        if(direction_3)
        {
            target_y = -target_y;
        }
        if(direction_6)
        {
            target_z = -target_z;
        }

        ROS_WARN("current_value_0 : %d", current_value_0);
        ROS_WARN("current_value_3 : %d", current_value_3);
        ROS_WARN("current_value_6 : %d", current_value_6);
        ROS_WARN("direction_0 : %d", direction_0);
        ROS_WARN("direction_3 : %d", direction_3);
        ROS_WARN("direction_6 : %d", direction_6);
        ROS_WARN("prev_value_0 : %d", prev_value_0);
        ROS_WARN("prev_value_3 : %d", prev_value_3);
        ROS_WARN("prev_value_6 : %d", prev_value_6);

        // Update previous values
        prev_value_0 = current_value_0;
        prev_value_3 = current_value_3;
        prev_value_6 = current_value_6;

        hope_msgs::ModbusRobotFeedback modbus_robot_feedback_msg;
        modbus_robot_feedback_msg.header.stamp = ros::Time::now();
        modbus_robot_feedback_msg.header.frame_id = "Running Parameters";
        // Publish feedback for each motor
        if (unit_ids_.size() > 0) {
            hope_msgs::ModbusRunningParams modbus_x_motor_feedback_msg;
            publishMotorFeedback(driver_id_x, modbus_x_motor_feedback_msg);
            modbus_x_motor_feedback_msg.header.frame_id = 'x';
            modbus_robot_feedback_msg.x = modbus_x_motor_feedback_msg;
            // modbus_running_data_pub.publish(modbus_robot_feedback_msg.x);
        }
        if (unit_ids_.size() > 1) {
            hope_msgs::ModbusRunningParams modbus_y_motor_feedback_msg;
            publishMotorFeedback(driver_id_y, modbus_y_motor_feedback_msg);
            modbus_y_motor_feedback_msg.header.frame_id = 'y';
            modbus_robot_feedback_msg.y = modbus_y_motor_feedback_msg;
            // modbus_running_data_pub.publish(modbus_robot_feedback_msg.y);
        }
        if (unit_ids_.size() > 2) {
            hope_msgs::ModbusRunningParams modbus_z_motor_feedback_msg;
            publishMotorFeedback(driver_id_z, modbus_z_motor_feedback_msg);
            modbus_z_motor_feedback_msg.header.frame_id = 'z';
            modbus_robot_feedback_msg.z = modbus_z_motor_feedback_msg;
            // modbus_running_data_pub.publish(modbus_robot_feedback_msg.z);
        }

        modbus_running_data_pub.publish(modbus_robot_feedback_msg);

        // Process motor X command
        context = driver_ctxs[driver_id_x - 1];
        motorControlLoop(context, target_x, mini_speed_x, max_speed_x, direction_0);

        // Process motor Y command
        context = driver_ctxs[driver_id_y - 1];
        motorControlLoop(context, target_y, mini_speed_y, max_speed_y, direction_3);

        // Process motor Z command with delay
        context = driver_ctxs[driver_id_z - 1];
        ros::Duration(z_delay_ms / 1000.0).sleep();  // Custom delay for Z motor
        motorControlLoop(context, target_z, mini_speed_z, max_speed_z, direction_6);

    }
    else if(command.substr(0, 3) == "MX:"){
        // Remove the prefix and split the remaining string by commas
        data = command.substr(3);
        ss.clear();  // Clear any error flags
        ss.str("");  // Reset the contents of the stringstream
        ss.str(data);
        values.clear();  // Clear the previous values from the vector

        while (getline(ss, token, ',')) {
            if (!token.empty()) {
                values.push_back(stoi(token));
            }
        }

        // Clear the command after processing
        command.clear();

        // Ensure there are at least 12 elements (3 sets of motor commands + delay + 2 additional values)
        if (values.size() != 3) {
            ROS_WARN("Insufficient values in command MX!");
            return;
        }

        current_value_0 = values[0];

        // Extract motor X, Y, and Z parameters
        target_x = current_value_0 - prev_value_0;
        mini_speed_x = values[1];
        max_speed_x = values[2];

        direction_0 = (target_x) < 0 ? 1 : 0;

        if(direction_0)
        {
            target_x = -target_x;
        }

        prev_value_0 = current_value_0;

        hope_msgs::ModbusRobotFeedback modbus_robot_feedback_msg;
        modbus_robot_feedback_msg.header.stamp = ros::Time::now();
        modbus_robot_feedback_msg.header.frame_id = "X Encoder Value : ";
        // Publish feedback for each motor
        if (unit_ids_.size() > 0) {
            hope_msgs::ModbusRunningParams modbus_x_motor_feedback_msg;
            publishMotorFeedback(driver_id_x, modbus_x_motor_feedback_msg);
            modbus_x_motor_feedback_msg.header.frame_id = 'x';
            modbus_robot_feedback_msg.x = modbus_x_motor_feedback_msg;
        }

        modbus_running_data_pub.publish(modbus_robot_feedback_msg);

        // Process motor X command
        context = driver_ctxs[driver_id_x - 1];
        motorControlLoop(context, target_x, mini_speed_x, max_speed_x, direction_0);

        // Print the extra values for future use

        ROS_INFO("Pos1: %d", target_x);
        ROS_INFO("Min Speed 1: %d", mini_speed_x);
        ROS_INFO("Max Speed 1: %d", max_speed_x);
    }
    else if(command.substr(0, 3) == "MY:"){
        // Remove the prefix and split the remaining string by commas
        data = command.substr(3);
        ss.clear();  // Clear any error flags
        ss.str("");  // Reset the contents of the stringstream
        ss.str(data);
        values.clear();  // Clear the previous values from the vector

        while (getline(ss, token, ',')) {
            if (!token.empty()) {
                values.push_back(stoi(token));
            }
        }
        // Clear the command after processing
        command.clear();

        // Ensure there are at least 12 elements (3 sets of motor commands + delay + 2 additional values)
        if (values.size() != 3) {
            ROS_WARN("Insufficient values in command MY!");
            return;
        }

        current_value_3 = values[0];

        // Extract motor X, Y, and Z parameters
        target_y = current_value_3 - prev_value_3;
        mini_speed_y = values[1];
        max_speed_y = values[2];

        direction_3 = (target_y) < 0 ? 1 : 0;

        if(direction_3)
        {
            target_y = -target_y;
        }

        prev_value_3 = current_value_3;

        hope_msgs::ModbusRobotFeedback modbus_robot_feedback_msg;
        modbus_robot_feedback_msg.header.stamp = ros::Time::now();
        modbus_robot_feedback_msg.header.frame_id = "Y Encoder value : ";

        if (unit_ids_.size() > 1) {
            hope_msgs::ModbusRunningParams modbus_y_motor_feedback_msg;
            publishMotorFeedback(driver_id_y, modbus_y_motor_feedback_msg);
            modbus_y_motor_feedback_msg.header.frame_id = 'y';
            modbus_robot_feedback_msg.y = modbus_y_motor_feedback_msg;
        }

        modbus_running_data_pub.publish(modbus_robot_feedback_msg);

        // Process motor Y command
        context = driver_ctxs[driver_id_y - 1];
        motorControlLoop(context, target_y, mini_speed_y, max_speed_y, direction_3);

        // Print the extra values for future use

        ROS_INFO("Pos2: %d", target_y);
        ROS_INFO("Min Speed 2: %d", mini_speed_y);
        ROS_INFO("Max Speed 2: %d", max_speed_y);
    }
    else if(command.substr(0, 3) == "MZ:"){
        // Remove the prefix and split the remaining string by commas
        data = command.substr(3);
        ss.clear();  // Clear any error flags
        ss.str("");  // Reset the contents of the stringstream
        ss.str(data);
        values.clear();  // Clear the previous values from the vector

        while (getline(ss, token, ',')) {
            if (!token.empty()) {
                values.push_back(stoi(token));
            }
        }
        // Clear the command after processing
        command.clear();

        // Ensure there are at least 12 elements (3 sets of motor commands + delay + 2 additional values)
        if (values.size() != 3) {
            ROS_WARN("Insufficient values in command MZ!");
            return;
        }

        current_value_6 = values[0];

        // Extract motor X, Y, and Z parameters
        target_z = current_value_6 - prev_value_6;
        mini_speed_z = values[1];
        max_speed_z = values[2];

        direction_6 = (target_z) < 0 ? 1 : 0;

        if(direction_6)
        {
            target_z = -target_z;
        }

        prev_value_6 = current_value_6;

        hope_msgs::ModbusRobotFeedback modbus_robot_feedback_msg;
        modbus_robot_feedback_msg.header.stamp = ros::Time::now();
        modbus_robot_feedback_msg.header.frame_id = "Z Encoder Value : ";
        if (unit_ids_.size() > 2) {
            hope_msgs::ModbusRunningParams modbus_z_motor_feedback_msg;
            publishMotorFeedback(driver_id_z, modbus_z_motor_feedback_msg);
            modbus_z_motor_feedback_msg.header.frame_id = 'z';
            modbus_robot_feedback_msg.z = modbus_z_motor_feedback_msg;
        }

        modbus_running_data_pub.publish(modbus_robot_feedback_msg);

        // Process motor Z command
        context = driver_ctxs[driver_id_z - 1];
        motorControlLoop(context, target_z, mini_speed_z, max_speed_z, direction_6);

        // Print the extra values for future use

        ROS_INFO("Pos3: %d", target_z);
        ROS_INFO("Min Speed 3: %d", mini_speed_z);
        ROS_INFO("Max Speed 3: %d", max_speed_z);

    }

    /* Storing the Time of Every Command */
    ros::Time command_time =
        ros::Time::now(); /* But Will only publish for MA: and MZ:*/

        /* Warning the user that command was sent */
    int i = 0;

    /* Switch case only to Recognise MA and MZ command */
    switch (command[i]) {

        case 'M':
        {
            i++;
            switch (command[i]) {
            case 'A':
                ma_time_msg.header.stamp = command_time;
                MA_time_pub.publish(ma_time_msg);
                break;
            case 'Z':
                mz_time_msg.header.stamp = command_time;
                MZ_time_pub.publish(mz_time_msg);
                break;
            default:
                break;
            }
            break;
        }
        case 'B':
        {
            // Ensure the command starts with "BI:"
            if (command.substr(0, 3) != "BI:") {
                std::cerr << "Invalid command format of BI!" << std::endl;
                return;
            }

            bi_time_msg.header.stamp = command_time;
            bi_time_pub.publish(bi_time_msg);

            bi_delay_int_msg.data = from_string_to_int_msg();
            bi_delay_pub.publish(bi_delay_int_msg);

            break;
        }
        case 'T':
        {
            // Ensure the command starts with "TI:"
            if (command.substr(0, 3) != "TI:") {
                std::cerr << "Invalid command format of TI!" << std::endl;
                return;
            }
            ti_time_msg.header.stamp = command_time;
            ti_time_pub.publish(ti_time_msg);

            ti_delay_int_msg.data = from_string_to_int_msg();
            ti_delay_pub.publish(ti_delay_int_msg);

            break;
        }
        default:
        {
            break;
        }
    }
    tcp_active = false;  // Allow Modbus reads again
    ROS_INFO("TCP command processing complete. Modbus read re-enabled.");

    //To start 'modbus_read' node
    //Start a **non-blocking timer** to resume after 3 seconds
    resume_timer.setPeriod(ros::Duration(1), true);  // Set timer duration (0.5 sec)
    resume_timer.start();  // Start the timer
    // active_msg.data = false;
    // write_active_pub.publish(active_msg);  // Signal that writing is done
}

void ModbusWrite::startThirdMotor(modbus_t *mb, int target, int mini_speed, int max_speed, int direction, int z_delay_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(z_delay_ms));  // Non-blocking delay

    ROS_DEBUG("Start : %d", z_delay_ms);

    motorControlLoop(mb, target, mini_speed, max_speed, direction);
}

template<typename T>
void ModbusWrite::getParameter(const std::string& param_name, T& param_value, const T& default_value) {
    // Try to get the parameter from the parameter server
    nh_.getParam(param_name, param_value);
    // If the parameter is not found, use the default value
    if (!nh_.hasParam(param_name)) {
        param_value = default_value;
        ROS_WARN_STREAM("ModbusWrite : Unable to load '" << param_name << "' defaulting to : " << param_value);
    }
}

template<typename T>
void ModbusWrite::getParameter(const std::string& param_name, T& param_value, const std::string& default_value) {
    // Try to get the parameter from the parameter server
    nh_.getParam(param_name, param_value);
    // If the parameter is not found, use the default value
    if (!nh_.hasParam(param_name)) {
        param_value = default_value;
        std::ostringstream oss;
        for (const auto& element : param_value) {
            oss << element << " ";
        }
        ROS_WARN_STREAM("modbus_write" << " : Unable to load '" << param_name << "' defaulting to : " << oss.str());
    }
}

