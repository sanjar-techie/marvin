#include "rclcpp/rclcpp.hpp"
#include "motor_test/global.hpp"
#include "motor_test/main.hpp"
#include "motor_test/com.hpp"
#include "motor_msgs/msg/mdrobotmsg1.hpp"
#include "motor_msgs/msg/mdrobotmsg2.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

motor_msgs::msg::Mdrobotmsg1 md_robot_message1;
motor_msgs::msg::Mdrobotmsg2 md_robot_message2;

ROBOT_PARAMETER_t robotParamData; //wheel_radius, distance between wheels, max rpm, PID gain etc..
static geometry_msgs::msg::Twist old_vel_cmd; // to record old velocity

SETTINNG_PARAM_STEP_t byCntInitStep; // stop to set the motor parameter
// volatile uint16_t appTick;
// volatile uint16_t req_tick;
uint16_t byCntComStep;
uint32_t velCmdUpdateCount;
uint32_t velCmdRcvCount;

INIT_SETTING_STATE_t fgInitsetting; // None=0, OK=1, Error=2

double goal_cmd_speed;             // m/sec
double goal_cmd_ang_speed;         // radian/sec
bool reset_pos_flag;               // reset position flag
bool reset_alarm_flag;             // reset alarm flag

extern PID_PNT_MAIN_DATA_t curr_pid_pnt_main_data;
extern PID_ROBOT_MONITOR2_t curr_pid_robot_monitor2;
extern PID_PNT_IO_MONITOR_t curr_pid_pnt_io_monitor;

extern int InitSerialComm(void);    // Serial Communication
extern int16_t * RobotSpeedToRPMSpeed(double linear, double angular); // convert m/s to rpm

void InitMotorParameter(void) // set the motor parameter
{
    switch(byCntInitStep)
    {
        case SETTING_PARAM_STEP_PID_PNT_VEL_CMD:
        {
            PID_PNT_VEL_CMD_t cmd_data, *p;

#if 0
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SET_PID_PNT_VEL_CMD(%d)", PID_PNT_VEL_CMD);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size of PID_PNT_VEL_CMD_t: %ld", sizeof(PID_PNT_VEL_CMD_t));
#endif

            p = &cmd_data;
            p->enable_id1 = 1;
            p->rpm_id1 = 0;
            p->enable_id2 = 1;
            p->rpm_id2 = 0;
            p->req_monitor_id = REQUEST_PNT_MAIN_DATA; // set PID_PNT_VEL_CMD_t parameter
            PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data)); // 207

            byCntInitStep = SETTING_PARAM_STEP_PID_ROBOT_PARAM; // next step
            break;
        }

        case SETTING_PARAM_STEP_PID_ROBOT_PARAM:
        {
            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                if(robotParamData.nRMID == robotParamData.nIDMDUI) 
                {
                    PID_ROBOT_PARAM_t cmd_data, *p;

    #if 0
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SETTING_PARAM_STEP_PID_ROBOT_PARAM(%d)", SETTING_PARAM_STEP_PID_ROBOT_PARAM);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size of PID_ROBOT_PARAM_t: %ld", sizeof(c));
    #endif
                    p = &cmd_data;
                    p->nDiameter = (uint16_t)robotParamData.nDiameter;
                    p->nWheelLength = (uint16_t)robotParamData.nWheelLength * 1000;                 // m unit --> mm unit
                    p->nGearRatio = (uint16_t)robotParamData.nGearRatio;
                    PutMdData(PID_ROBOT_PARAM, MID_MDUI, (const uint8_t *)p, sizeof(cmd_data));     // 247

                    byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
                }
                else {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "err.mismatch ID(RMID(%d), MDUI(%d))", robotParamData.nRMID, robotParamData.nIDMDUI);
                    fgInitsetting = INIT_SETTING_STATE_ERROR;
                }
            }
            else {
                byCntInitStep = SETTING_PARAM_STEP_PID_POSI_RESET;
            }
            break;
        }

        case SETTING_PARAM_STEP_PID_POSI_RESET: // to reset pid_posi
        {
            uint8_t dummy;

#if 0
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_POSI_RESET(%d)", PID_POSI_RESET);
#endif

            dummy = 0;
            PutMdData(PID_POSI_RESET, robotParamData.nRMID, (const uint8_t *)&dummy, sizeof(dummy));

            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_START;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_START:
        {
            PID_SLOW_START_t cmd_data, *p;

#if 0
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_SLOW_START(%d)", PID_SLOW_START);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size of PID_SLOW_START_t: %ld", sizeof(PID_SLOW_START_t));
#endif

            p = &cmd_data;
            p->value = robotParamData.nSlowstart; //setting slowstart velocity value

            PutMdData(PID_SLOW_START, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_SLOW_DOWN;
            break;
        }

        case SETTING_PARAM_STEP_PID_SLOW_DOWN:
        {
            PID_SLOW_DOWN_t cmd_data, *p;

#if 0
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_SLOW_DOWN(%d)", PID_SLOW_DOWN);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size of PID_SLOW_DOWN_t: %ld", sizeof(PID_SLOW_DOWN_t));
#endif

            p = &cmd_data;
            p->value = robotParamData.nSlowdown; // setting slowdown velocity value

            PutMdData(PID_SLOW_DOWN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_GAIN;
            break;
        }

        case SETTING_PARAM_STEP_PID_GAIN:
        {
            PID_GAIN_t cmd_data, *p;

#if 0
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_GAIN(%d)", PID_GAIN);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "size of PID_GAIN: %ld", sizeof(PID_GAIN_t));
#endif

            p = &cmd_data;
            p->position_proportion_gain = robotParamData.position_proportion_gain; // setting proportion gain
            p->speed_proportion_gain = robotParamData.speed_proportion_gain;       // setting differential gain
            p->integral_gain = robotParamData.integral_gain;                       // setting integral gain

            PutMdData(PID_GAIN, robotParamData.nRMID, (const uint8_t *)p, sizeof(cmd_data));

            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD:       // Left motor
        {
            uint8_t cmd_data;

#if 1
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_INV_SIGN_CMD(%d)", PID_INV_SIGN_CMD);
#endif

            if(robotParamData.reverse_direction == 0) {
                cmd_data = 1;
            }
            else {
                cmd_data = 0;
            }

            PutMdData(PID_INV_SIGN_CMD, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_INV_SIGH_CMD2;
            break;
        }

        case SETTING_PARAM_STEP_PID_INV_SIGH_CMD2:      // Right motor
        {
            uint8_t cmd_data;

#if 1
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_INV_SIGN_CMD2(%d)", PID_INV_SIGN_CMD2);
#endif

            if(robotParamData.reverse_direction == 0) {
                cmd_data = 0;
            }
            else {
                cmd_data = 1;
            }

            PutMdData(PID_INV_SIGN_CMD2, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1);

            byCntInitStep = SETTING_PARAM_STEP_PID_USE_EPOSI;
            break;
        }

        case SETTING_PARAM_STEP_PID_USE_EPOSI:
        {
            uint8_t cmd_data;

#if 1
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_USE_POSI(%d)", PID_USE_POSI);
#endif

            if(robotParamData.motor_position_type == 0) {
                cmd_data = 0;               // hall sensor
            }
            else {
                cmd_data = 1;               // encoder
            }

            PutMdData(PID_USE_POSI, robotParamData.nRMID, (const uint8_t *)&cmd_data, 1); // setting parameter which sensor do you use hall sensor or encoder

            byCntInitStep = SETTING_PARAM_STEP_PID_PPR;
            break;
        }

        case SETTING_PARAM_STEP_PID_PPR:
        {
            PID_PPR_t cmd_data, *p; 

#if 1
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PID_PPR(%d)", PID_PPR);
#endif
            p = &cmd_data;

            p->PPR = robotParamData.encoder_PPR;

            PutMdData(PID_PPR, robotParamData.nRMID, (const uint8_t *)&cmd_data, sizeof(PID_PPR_t)); // setting parameter PPR(Pulse Per Rotation)

            byCntInitStep = SETTING_PARAM_STEP_DONE;

            fgInitsetting = INIT_SETTING_STATE_OK;
            break;
        }

        default:
            break;
    }
}

void RequestRobotStatusTask(void)
{
    int nArray[5];
    uint8_t req_pid;
    int16_t *pGoalRPMSpeed;

    switch(byCntComStep)
    {
        case 0:
        {
            req_pid = PID_PNT_MAIN_DATA;            //PID 210
            PutMdData(PID_REQ_PID_DATA, robotParamData.nRMID, (const uint8_t *)&req_pid, 1);

            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                byCntComStep = 1;
            }
            else {
                byCntComStep = 3;
            }
            break;
        }

        case 1:
        {
            if(robotParamData.use_MDUI == 1) {  // If using MDUI
                // req_pid = PID_ROBOT_MONITOR2;            // PID 224, Only MDUI
                req_pid = PID_ROBOT_MONITOR;            // PID 253, Only MDUI
                PutMdData(PID_REQ_PID_DATA, MID_MDUI, (const uint8_t *)&req_pid, 1);
            }

            byCntComStep = 3;
            break;
        }

        case 3:
        {
            if(curr_pid_robot_monitor2.byPlatStatus.bits.bEmerSW == 1)
            {
                PID_PNT_TQ_OFF_t pid_pnt_tq_off, *p;

                pid_pnt_tq_off.enable_id1 = 1;
                pid_pnt_tq_off.enable_id2 = 1;
                pid_pnt_tq_off.req_monitor_id = REQUEST_PNT_MAIN_DATA;
                PutMdData(PID_PNT_TQ_OFF, robotParamData.nRMID, (const uint8_t *)&pid_pnt_tq_off, sizeof(pid_pnt_tq_off));
            }
            else if(reset_pos_flag == true) {
                uint8_t dummy;  
                
                reset_pos_flag = false;

                dummy = 0;
                PutMdData(PID_POSI_RESET, robotParamData.nRMID, (const uint8_t *)&dummy, sizeof(dummy));
            }
            else if(reset_alarm_flag == true) {
                uint8_t cmd_pid;

                reset_alarm_flag = false;

                cmd_pid = CMD_ALARM_RESET;
                PutMdData(PID_COMMAND, robotParamData.nRMID, (const uint8_t *)&cmd_pid, 1);
            }
            else {
                if(velCmdUpdateCount > 0) {
                    velCmdUpdateCount = 0;

                    PID_PNT_VEL_CMD_t pid_pnt_vel_cmd, *p;

                    pGoalRPMSpeed = RobotSpeedToRPMSpeed(goal_cmd_speed, goal_cmd_ang_speed);

        #if 0
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal RPM L:%d, R:%d", pGoalRPMSpeed[0], pGoalRPMSpeed[1]);
        #endif

                    p = &pid_pnt_vel_cmd;
                    p->enable_id1 = 1;
                    p->rpm_id1 = pGoalRPMSpeed[0];
                    p->enable_id2 = 1;
                    p->rpm_id2 = pGoalRPMSpeed[1];
                    p->req_monitor_id = REQUEST_PNT_MAIN_DATA;

                    PutMdData(PID_PNT_VEL_CMD, robotParamData.nRMID, (const uint8_t *)&pid_pnt_vel_cmd, sizeof(pid_pnt_vel_cmd));
                }
            }

            byCntComStep = 0;
            break;
        }

        default:
            byCntComStep = 0;
            break;
    }
}

class MDRobotNode : public rclcpp::Node
{
public:
    MDRobotNode()
        : Node("md_robot_node")
    {
        // ROS 2 Publishers
        md_robot_message1_pub_ = this->create_publisher<motor_msgs::msg::Mdrobotmsg1>("md_robot_message1", 10);
        md_robot_message2_pub_ = this->create_publisher<motor_msgs::msg::Mdrobotmsg2>("md_robot_message2", 10);

        // ROS 2 Subscribers
        keyboard_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MDRobotNode::cmdVelCallback, this, std::placeholders::_1));
        reset_position_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "reset_position", 10, std::bind(&MDRobotNode::resetPositionCallback, this, std::placeholders::_1));
        reset_alarm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "reset_alarm", 10, std::bind(&MDRobotNode::resetAlarmCallback, this, std::placeholders::_1));

        // Initialize other variables and parameters
        // ...

        // Create ROS 2 Timers
        // app_tick_timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
        //                                           std::bind(&MDRobotNode::appTickTimerCallback, this));
        // vel_cmd_rcv_timeout_timer_ = this->create_timer(std::chrono::seconds(2),
        //                                                 std::bind(&MDRobotNode::velCmdRcvTimeoutCallback, this));
        vel_cmd_rcv_timeout_timer_ =this->create_wall_timer(
            2000ms, std::bind(&MDRobotNode::velCmdRcvTimeoutCallback,this));
        // rcv_timer_ = this->create_timer(std::chrono::milliseconds(1),
        //                                 std::bind(&MDRobotNode::rcvTickTimerCallback, this));
        rcv_timer_ = this->create_wall_timer(
            1ms, std::bind(&MDRobotNode::rcvTickTimerCallback, this));
        // req_timer_ = this->create_timer(std::chrono::milliseconds(30),
        //                                 std::bind(&MDRobotNode::reqTickTimerCallback, this));
        req_timer = this->create_wall_timer(30ms, std::bind(&MDRobotNode::reqTickTimerCallback, this));
        
        pub_timer_1 = this->create_wall_timer(500ms, std::bind(&MDRobotNode::PubMessage1Callback, this));

        pub_timer_2 = this->create_wall_timer(500ms, std::bind(&MDRobotNode::PubMessage2Callback, this));
    }
    rclcpp::Publisher<motor_msgs::msg::Mdrobotmsg1>::SharedPtr md_robot_message1_pub_; // have to convert global variable
    rclcpp::Publisher<motor_msgs::msg::Mdrobotmsg2>::SharedPtr md_robot_message2_pub_; // have to convert global variable

private:
    // void appTickTimerCallback()
    // {
    //     appTick++;
    //     // Implement your logic here
    // }
    void velCmdRcvTimeoutCallback()
    {
        static uint32_t old_velCmdRcvCount;

        if (velCmdRcvCount == old_velCmdRcvCount)
        {
            goal_cmd_speed = 0;
            goal_cmd_ang_speed = 0;

            velCmdUpdateCount++;

            old_velCmdRcvCount = velCmdRcvCount;
        }
    }

    void rcvTickTimerCallback()
    {
        ReceiveDataFromController();
    }

    void reqTickTimerCallback()
    {
        RequestRobotStatusTask();
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr keyVel)
    {
        if (fgInitsetting == INIT_SETTING_STATE_OK)
        {
            velCmdRcvCount++;

            if (old_vel_cmd.linear.x != keyVel->linear.x || old_vel_cmd.angular.z != keyVel->angular.z)
            {
                old_vel_cmd.linear.x = keyVel->linear.x;
                old_vel_cmd.angular.z = keyVel->angular.z;

                velCmdUpdateCount++;

                goal_cmd_speed = keyVel->linear.x;
                goal_cmd_ang_speed = keyVel->angular.z;

                // Implement your logic here
            }
        }
    }

    void resetPositionCallback(const std_msgs::msg::Bool::SharedPtr reset_position_msg)
    {
        if (reset_position_msg->data == true)
        {
            reset_pos_flag = true;
        }
    }

    void resetAlarmCallback(const std_msgs::msg::Bool::SharedPtr reset_alarm_msg)
    {
        if (reset_alarm_msg->data == true)
        {
            reset_alarm_flag = true;
        }
    }

    void PubMessage1Callback()
    {
        md_robot_message1_pub_->publish(md_robot_message1);
    }
    
    void PubMessage2Callback()
    {
        md_robot_message2_pub_->publish(md_robot_message2);
    }

    // Declare class member variables here

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_alarm_sub_;

    // rclcpp::TimerBase::SharedPtr app_tick_timer_;
    rclcpp::TimerBase::SharedPtr vel_cmd_rcv_timeout_timer_;
    rclcpp::TimerBase::SharedPtr rcv_timer_;
    rclcpp::TimerBase::SharedPtr req_timer;
    rclcpp::TimerBase::SharedPtr pub_timer_1;
    rclcpp::TimerBase::SharedPtr pub_timer_2;
    // rclcpp::TimerBase::SharedPtr req_timer_;
};

int main(int argc, char **argv)
{   
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("motor_node");

    int16_t *pGoalRPMSpeed;

    reset_pos_flag = false;
    reset_alarm_flag = false;

    fgInitsetting = INIT_SETTING_STATE_NONE;

    node->get_parameter("md_robot_node/use_MDUI", robotParamData.use_MDUI);         //@@ need to check launch file
    node->get_parameter("md_robot_node/wheel_radius", robotParamData.wheel_radius);               // m unit
    node->get_parameter("md_robot_node/wheel_length", robotParamData.nWheelLength);                // m unit
    node->get_parameter("md_robot_node/reduction", robotParamData.nGearRatio);
    node->get_parameter("md_robot_node/reverse_direction", robotParamData.reverse_direction);
    node->get_parameter("md_robot_node/maxrpm", robotParamData.nMaxRPM);
    node->get_parameter("md_robot_node/position_type", robotParamData.motor_position_type);
    node->get_parameter("md_robot_node/encoder_PPR", robotParamData.encoder_PPR);
    node->get_parameter("md_robot_node/position_proportion_gain", robotParamData.position_proportion_gain);
    node->get_parameter("md_robot_node/speed_proportion_gain", robotParamData.speed_proportion_gain);
    node->get_parameter("md_robot_node/integral_gain", robotParamData.integral_gain);
    node->get_parameter("md_robot_node/slow_start", robotParamData.nSlowstart);
    node->get_parameter("md_robot_node/slow_down", robotParamData.nSlowdown);

    robotParamData.nIDPC = MID_PC;         // Platform mini-PC ID -> PC ID 설정
    robotParamData.nIDMDUI = MID_MDUI;       // MDUI ID -> MDUI 모터 드라이버와 통신 모듈 
    robotParamData.nIDMDT = MID_MDT;        // MD750T, MD400T, MD200T ID -> 모터드라이버 선택

    if(robotParamData.use_MDUI == 1) {  // If using MDUI
        robotParamData.nRMID = robotParamData.nIDMDUI;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Using MDUI");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------");
    }
    else {
        robotParamData.nRMID = robotParamData.nIDMDT;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Not using MDUI");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "----------------------------------");
    }

    robotParamData.nBaudrate = 57600;
    robotParamData.nDiameter = (int)(robotParamData.wheel_radius * 2.0 * 1000.0);

    if(InitSerialComm() == -1) {     //communication initialization in com.cpp 
        return 1;
    }

    byCntInitStep = SETTING_PARAM_STEP_PID_PNT_VEL_CMD;

    ReceiveDataFromController();
    InitMotorParameter();               //init motor setting
    if(fgInitsetting == INIT_SETTING_STATE_OK)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor init done");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor init error");
    }

    byCntComStep = 0;
    // ReceiveDataFromController();
    // RequestRobotStatusTask();           //init serial communication
    rclcpp::spin(std::make_shared<MDRobotNode>());
    
    rclcpp::shutdown();
    return 0;
}

// void PubMDRobotMessage1()
// {
//     MDRobotNode::md_robot_message1_pub_->publish(md_robot_message1);
// }

// void PubMDRobotMessage2()
// {
//     md_robot_message2_pub_->publish(md_robot_message2);
// }