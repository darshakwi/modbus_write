/**
 * @file   modbus_write.h
 * @brief  Header containing ModbusWrite Class declaration.
 * @author Darshak Vaghela
 * @date   2025-01-02
 * *****************************************************************/

// Core C++ Header Files
#include <vector>
#include <variant>
#include <string>
#include <sstream>
#include <unordered_map>
#include <iostream>
#include <modbus/modbus.h>
#include <yaml-cpp/yaml.h>
#include <sys/select.h>  // For select-based timeout
#include <thread>
#include <chrono>
#include <unordered_map>

// ROS Core Header Files
#include <ros/ros.h>
#include <ros/console.h>

// ROS Built-in Message Header Files
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

// Boost Asio Header
#include <boost/asio.hpp>

#include <hope_msgs/ModbusFeedback.h>
#include <hope_msgs/ModbusRobotFeedback.h>
#include <hope_msgs/ModbusRunningParams.h>
#include <hope_msgs/TimeComparison.h>


class ModbusWrite {
public:
    /// @brief Instantiate a ROS nodehandle (user defined).
    ///
    /// @pre   ROS node should be initialized.
    ///
    /// @param nodehandle Pointer to the ROS NodeHandle.
    ///
    ModbusWrite(ros::NodeHandle* nodehandle);
    
    /// @brief ModbusWrite destructor utilized
    /// to release unqiue pointer
    ///
    ~ModbusWrite();

private:
    /// @brief Unique Pointer handling modbus_write library calls
    ///
    std::unique_ptr<ModbusWrite> modbus_write_;

    ros::NodeHandle nh_;    ///< Public nodehandle for ROS param
    // ros::NodeHandle p_nh_;  ///< Private nodehandle for ROS communicators
                            ///< other than ROS param

    ros::Timer motor_status_;
    ros::Timer resume_timer;

    // Serial UART communication
    std::string port ; ///< Store value from param_server
    int baudrate ;  ///< Store value from param_server

    std::string command{};

    // /* Declaring Boost asio */
    // boost::asio::io_context* io_ = new boost::asio::io_context; ///< Required to open the serial port
    // boost::asio::serial_port my_port; ///< Serial Port Declaration

    std::unique_ptr<boost::asio::io_context> io_;
    std::unique_ptr<boost::asio::serial_port> my_port;

    std::unordered_map<int, bool> motor_was_running;  // Stores motor state

    std::string namespace_str{};
    std::string data;
    std::string token;
    std::vector<int> values;
    std::istringstream ss;

    modbus_t* context;

    bool tcp_active = false; // Flag to indicate TCP activity

    int robot_num;

    // Modbus connection settings
    std::string modbus_port;     ///< Port for Modbus communication
    int modbus_baudrate;         ///< Baud rate for Modbus communication
    std::string modbus_parity;   ///< Parity setting ('N', 'E', 'O')
    int modbus_stopbits;         ///< Number of stop bits (1 or 2)
    int modbus_bytesize;         ///< Byte size (typically 8)
    int modbus_timeout;          ///< Timeout for Modbus responses in milliseconds

    int pulse_per_rotation;
    int position_target_low_regr;
    int position_target_high_regr;
    int motor_start_stop_regr;
    int motion_rotation_dir_regr;
    int position_acceleration_time_regr;
    int position_deceleration_time_regr;
    int position_speed_rpm_regr;
    int motor_speed;

    int position_feedback_counter;

    std::vector<int> unit_ids_;                     ///< List of motor unit IDs
    
    std::vector<modbus_t*> driver_ctxs; ///< Vector of Modbus contexts (one for each motor)
    std::thread motor3_thread;  // Thread for the third motor

    uint16_t target_low;
    uint16_t target_high;
    int direction;
    int target_x;
    int target_y;
    int target_z;
    int mini_speed_x;
    int mini_speed_y;
    int mini_speed_z;
    int max_speed_x;
    int max_speed_y;
    int max_speed_z;
    double accel_time;
    double decel_time;
    int z_delay_ms;
    int extra_value1;
    int extra_value2;
    int driver_id_x = 1;
    int driver_id_y = 2;
    int driver_id_z = 3;

    int encoder_x_starting_value;
    int encoder_y_starting_value;
    int encoder_z_starting_value;

    double gear_box_driver_x;
    double gear_box_driven_x;
    double pulley_pitch_x;
    double pulley_no_of_teeth_x;
    double encoder_resolution_x;
    double mm_per_encoder_counts_x;
    double position_mm_x;
    double pre_position_mm_x;
    double position_init_value_x;
    
    double gear_box_driver_y;
    double gear_box_driven_y;
    double pulley_pitch_y;
    double pulley_no_of_teeth_y;
    double encoder_resolution_y;
    double mm_per_encoder_counts_y;
    double position_mm_y;
    double position_init_value_y;
    
    double gear_box_driver_z;
    double gear_box_driven_z;
    double pulley_pitch_z;
    double pulley_no_of_teeth_z;
    double encoder_resolution_z;
    double mm_per_encoder_counts_z;
    double position_mm_z;
    double position_init_value_z;

    int current_value_0 = 0;
    int current_value_3 = 0;
    int current_value_6 = 0;

    // Variables to store previous values
    int prev_value_0 = 0;
    int prev_value_3 = 0;
    int prev_value_6 = 0;

    // Direction variables
    int direction_0 = -1;
    int direction_3 = -1;
    int direction_6 = -1;

    uint32_t last_encoder_value_x = 0;
    uint32_t last_encoder_value_y = 0;
    uint32_t last_encoder_value_z = 0;
    int32_t adjusted_encoder = 0;


    /********************************************************************
     * ROS Subscribers with callback and message declaration
     *******************************************************************/

    /// @brief Initializes ROS Subscribers.
    ///
    /// @pre ROS node and nodehandle should be initialized
    /// @pre Declaration of ros::Subscriber calls objects
    /// @pre Declaration of subscriber callback function
    ///
    /// @return Void
    ///
    void initializeSubscribers() ;

    /// @brief Mqtt Rpm Data Subscriber
    /// @param Topic "/conveyor_speed"
    ///
    ros::Subscriber mqtt_rpm_data_sub_;

    /*******************************************************************
     * ROS Publishers declaration
     ******************************************************************/

    /// @brief Initializes ROS Publishers.
    ///
    /// @pre ROS node and nodehandle should be initialized
    /// @pre Declaration of ros::Publisher class objects
    ///
    /// @return Void
    ///
    void initializePublishers();

    ros::Publisher MA_time_pub;
    ros::Publisher MZ_time_pub;
    ros::Publisher bi_time_pub;
    ros::Publisher ti_time_pub;
    ros::Publisher bi_delay_pub;
    ros::Publisher ti_delay_pub;
    ros::Publisher motion_complete_pub;
    ros::Publisher crt_ros_info_pub_;
    ros::Publisher modbus_running_data_pub; ///< ROS publisher for Modbus data
    ros::Publisher write_active_pub;

    hope_msgs::TimeComparison ma_time_msg{}; ///< Msg to publish
    hope_msgs::TimeComparison mz_time_msg{}; ///< Msg to publish
    hope_msgs::TimeComparison bi_time_msg{}; ///< Msg to publish
    hope_msgs::TimeComparison ti_time_msg{}; ///< Msg to publish

    // std::vector<MotorRegistersValue> motor_registers_value_;
    hope_msgs::ModbusRobotFeedback modbus_robot_feedback_msg; ///< Message for publishing serial data

    std_msgs::Int64 bi_delay_int_msg;
    std_msgs::Int64 ti_delay_int_msg;

    int from_string_to_int_msg();

    /*******************************************************************
     * ROS Parameter Interface for initialization and handling of parameters
     ******************************************************************/

    /// @brief Initializes the ROS parameters for this node.
    ///
    /// @pre ROS node and nodehandle should be initialized.
    ///
    /// @return void
    void initializeParameters();
    void initializeSerialPort();
    void startParameterReadingTimer();
    void initializeTimers();

    /**
     * @brief Cleans up all Modbus contexts.
     */
    void cleanupContexts();

    /*******************************************************************
     * ROS Private Functions
     ******************************************************************/

    void mcu_command_cb(const std_msgs::String::ConstPtr &msg);

    void motorControlLoop(modbus_t *ctx, int target_position, int mini_speed_rpm, int max_speed_rpm, int direction);
    void parseAndProcessCommand(modbus_t *ctx, const std::string &command);
    int convertRpmToTime(int min_rpm, int max_rpm, int ppr);

    void sendModbusWriteRequest(uint8_t slave_id, uint16_t register_address, uint16_t value1, uint16_t value2);
    double rpmToSPS(double rpm, int ppr);
    uint16_t calculateCRC(const std::vector<uint8_t> &data);
    bool crt_ros_gui_info_publish(std::string msg);
    void reconnectSerialPort();
    void mcu_read_data(const std_msgs::String::ConstPtr& msg);

    /**
     * @brief Validates the driver index to avoid out-of-bounds access.
     * @param driver_index Index of the motor to validate.
     * @return True if the index is valid, false otherwise.
     */
    bool validateDriverIndex(int driver_index);

    /*--------------------------------Template Functions---------------------------*/

    /**
     * @brief Gets a parameter from the ROS parameter server and stores it in the given variable. 
     *        If the parameter is not found, the default value is used instead.
     * 
     * @tparam T The type of the parameter value.
     * @param param_name The name of the parameter to get.
     * @param param_value The variable to store the parameter value in.
     * @param default_value The default value to use if the parameter is not found.
     * 
     * @note This function assumes that a `ros::NodeHandle` object has already been created and is accessible.
     * 
     * @return None.
     */
    template<typename T>
    void getParameter(const std::string& param_name, T& param_value, const T& default_value);
    
    ros::Subscriber handware_handler_sub;
    ros::Subscriber mcu_feedback_sub;

    /**
     * @brief Retrieves a parameter value from the parameter server or assigns a default value if not found.
     *
     * This function attempts to retrieve a parameter value of type std::vector<T> from the ROS parameter server using the specified parameter name. If the parameter is found, its value is stored in the param_value argument. If the parameter is not found, the param_value argument is assigned the provided default_value.
     *
     * @tparam T The type of elements in the vector.
     * @param param_name The name of the parameter to retrieve.
     * @param param_value [in, out] The variable to store the retrieved parameter value.
     * @param default_value The default value to assign to param_value if the parameter is not found.
     *
     * @note If the parameter is found, but its value is not a list (not a vector), the default_value will be assigned to param_value.
     *
     * @see nh_.getParam(const std::string&, T&) - ROS function used to retrieve the parameter value.
     * @see nh_.hasParam(const std::string&) - ROS function used to check if the parameter exists on the parameter server.
     */
    template<typename T>
    void getParameter(const std::string& param_name, T& param_value, const std::string& default_value);   
    void publishMotorFeedback(int motor_id, hope_msgs::ModbusRunningParams& feedback_msg);
    bool readData(int driver_index, uint16_t reg_address, uint32_t* value, bool is_32bit);
    double getActualRotation(uint32_t encoderValue, int isForward);
    double getRotationFraction(uint32_t encoderValue, int isForward);
    int32_t updateEncoder(uint32_t current_encoder, uint32_t last_encoder_value, int direction, int32_t adjusted_encoder);
    void startThirdMotor(modbus_t *mb, int target, int mini_speed, int max_speed, int direction, int z_delay_ms);
    void motorStatusReadCallback(const ros::TimerEvent&);
    bool readData(int driver_index, uint16_t reg_address, uint16_t* value);
    double calculateMotionTime(int target_position, int mini_speed_rpm, int max_speed_rpm, int accel_time_ms, int decel_time_ms);
    void resumeCallback(const ros::TimerEvent&);

};

