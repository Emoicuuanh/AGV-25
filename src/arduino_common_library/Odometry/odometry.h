#ifndef AGV_ODOM_
#define AGV_ODOM_

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include "encoder_define.h"

/* DEFINE ODOMETRY*/
#define NAME                             "AGV"

// Function Odometry
void initOdom();
void updateOdometry();
void Pub_Drive_Information();
bool calcOdometry(double diff_time);
void updateTime();
void initJointStates();
void updateVariable(bool isConnected);


// Function update
void odom_init();
void odom_loop();
void updateTF(geometry_msgs::TransformStamped& odom_tf);
void updateTFPrefix(bool isConnected);
void updateMotorInfo(int32_t left_tick, int32_t right_tick);
void updateJointStates(void);


/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
// ros::NodeHandle nh;
ros::Time current_time;
ros::Time rosNow(void);
uint32_t current_offset;

// Odometry of AGV
nav_msgs::Odometry odom;
ros::Publisher odom_pub("/odom", &odom);

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
bool init_encoder = true;
int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("/joint_states", &joint_states);

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
double odom_vel[3];

/*******************************************************************************
* SoftwareTimer of AGV
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
* ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

// char imu_frame_id[30];
// char mag_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
* Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;


void odom_setup()
{   
    nh.advertise(odom_pub);
    nh.advertise(joint_states_pub);

    tf_broadcaster.init(nh);

    // Setting for SLAM and navigation (odometry, joint states, TF)
    initOdom();
    initJointStates();
    prev_update_time = millis();
}

void odom_loop()
{
    uint32_t t = millis();
    updateTime();
    // updateVariable(nh.connected());
    updateTFPrefix(nh.connected());

    if ((t - tTime[2]) >= (1000 / 30))
    {
        updateMotorInfo(EncoderValue1, EncoderValue2);
        Pub_Drive_Information();
        tTime[2] = t;
    }
}

void Pub_Drive_Information()
{
    unsigned long time_now = millis();
    unsigned long step_time = time_now - prev_update_time;

    prev_update_time = time_now;
    ros::Time stamp_now = rosNow();

    // calculate odometry
    calcOdometry((double)(step_time * 0.001));

    // odometry
    updateOdometry();
    odom.header.stamp = stamp_now;
    odom_pub.publish(&odom);

    // odometry tf
    updateTF(odom_tf);
    odom_tf.header.stamp = stamp_now;
    tf_broadcaster.sendTransform(odom_tf);

    // joint states
    updateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
    static bool isChecked = false;
    char log_msg[50];
    if (isConnected)
    {
        if (isChecked == false)
        {
            nh.getParam("~tf_prefix", &get_tf_prefix);

            if (!strcmp(get_tf_prefix, ""))
            {
                sprintf(odom_header_frame_id, "odom");
                sprintf(odom_child_frame_id, "base_footprint");

                // sprintf(imu_frame_id, "imu_link");
                // sprintf(mag_frame_id, "mag_link");
                sprintf(joint_state_header_frame_id, "base_link");
            }
            else
            {
                strcpy(odom_header_frame_id, get_tf_prefix);
                strcpy(odom_child_frame_id, get_tf_prefix);

                // strcpy(imu_frame_id, get_tf_prefix);
                // strcpy(mag_frame_id, get_tf_prefix);
                strcpy(joint_state_header_frame_id, get_tf_prefix);

                strcat(odom_header_frame_id, "/odom");
                strcat(odom_child_frame_id, "/base_footprint");

                // strcat(imu_frame_id, "/imu_link");
                // strcat(mag_frame_id, "/mag_link");
                strcat(joint_state_header_frame_id, "/base_link");
            }

            sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
            nh.loginfo(log_msg);

            // sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
            // nh.loginfo(log_msg);

            // sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
            // nh.loginfo(log_msg);

            sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
            nh.loginfo(log_msg);

            isChecked = true;
        }
    }
    else
    {
        isChecked = false;
    }
}

void updateOdometry()
{
    // strcat(odom_header_frame_id, "/odom");
    // strcat(odom_child_frame_id, "/base_footprint");
    odom.header.frame_id = odom_header_frame_id;
    odom.child_frame_id = odom_child_frame_id;

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];
}
/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
    static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
    static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
    //static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};

    joint_states_pos[LEFT] = last_rad[LEFT];
    joint_states_pos[RIGHT] = last_rad[RIGHT];

    joint_states_vel[LEFT] = last_velocity[LEFT];
    joint_states_vel[RIGHT] = last_velocity[RIGHT];

    joint_states.position = joint_states_pos;
    joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped &odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{

    int32_t current_tick = 0;
    static int32_t last_tick[WHEEL_NUM] = {0, 0};

    if (init_encoder)
    {
        for (int index = 0; index < WHEEL_NUM; index++)
        {
            last_diff_tick[index] = 0;
            last_tick[index] = 0;
            last_rad[index] = 0.0;

            last_velocity[index] = 0.0;
        }

        last_tick[LEFT] = left_tick;
        last_tick[RIGHT] = right_tick;

        init_encoder = false;
        return;
    }

    current_tick = left_tick;

    last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
    last_tick[LEFT] = current_tick;
    last_rad[LEFT] += TICK2RAD * (double)last_diff_tick[LEFT];

    current_tick = right_tick;

    last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
    last_tick[RIGHT] = current_tick;
    last_rad[RIGHT] += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/

bool calcOdometry(double diff_time)
{
    float *orientation;
    double wheel_l, wheel_r; // rotation value of wheel [rad]
    double delta_s, theta, delta_theta;
    static double last_theta = 0.0;
    double v, w; // v = translational velocity [m/s], w = rotational velocity [rad/s]
    double step_time;

    wheel_l = wheel_r = 0.0;
    delta_s = delta_theta = theta = 0.0;
    v = w = 0.0;
    step_time = 0.0;

    step_time = diff_time;

    if (step_time == 0)
    {
        return false;
    }

    wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
    wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

    if (isnan(wheel_l))
    {
        wheel_l = 0.0;
    }
    if (isnan(wheel_r))
    {
        wheel_r = 0.0;
    }

    delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
    // orientation = sensors.getOrientation();
    theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;

    delta_theta = theta - last_theta;
    // compute odometric pose
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

    // compute odometric instantaneouse velocity
    v = delta_s / step_time;
    w = delta_theta / step_time;

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;

    last_velocity[LEFT] = wheel_l / step_time;
    last_velocity[RIGHT] = wheel_r / step_time;
    last_theta = theta;

    return true;
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
    current_offset = millis();
    current_time = nh.now();
}
/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
    return nh.now();
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom()
{
    init_encoder = true;
    for (int index = 0; index < 3; index++)
    {
        odom_pose[index] = 0.0;
        odom_vel[index] = 0.0;
    }

    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;
    odom.pose.pose.orientation.w = 0.0;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
    static char *joint_states_name[] = {(char *)"wheel_left_joint", (char *)"wheel_right_joint"};

    joint_states.header.frame_id = joint_state_header_frame_id;
    joint_states.name = joint_states_name;

    joint_states.name_length = WHEEL_NUM;
    joint_states.position_length = WHEEL_NUM;
    joint_states.velocity_length = WHEEL_NUM;
    joint_states.effort_length = WHEEL_NUM;
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
    static bool variable_flag = false;

    if (isConnected)
    {
        if (variable_flag == false)
        {
            // sensors.initIMU();
            initOdom();

            variable_flag = true;
        }
    }
    else
    {
        variable_flag = false;
    }
}
#endif