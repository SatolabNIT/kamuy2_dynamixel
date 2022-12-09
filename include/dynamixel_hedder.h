#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <memory>
#include <vector>
#include "dynamixel_controller/wheelstatus.h"



namespace std{};
class MultiDynamixelControl{
private:

class SingleDynamixelControl:public DynamixelWorkbench{
     public:
     SingleDynamixelControl(){}
     ~SingleDynamixelControl(){}

       uint8_t dxl_id;
       uint16_t model_number;
       uint32_t baud_rate;
       uint32_t p_gain;
       uint32_t i_gain;
       uint32_t d_gain;
       double initial_pos;
       std::string dxl_name_;
       std::string mode_;
       const char *log;
       const char *port_name;
       const char *dxl_name;
       const char *mode;
      const char* position="position";
      const char* current="current";
      const char* velocity="velocity";
       float now_radian;//motor
       void init();
       bool ping();
       void changemode();
       float getposition();
       float getvelocity();
       void MotorControl();
       bool ItemWrite(const char *item_name,double value);

      

      DynamixelWorkbench dxl_wb;
      
      private:
      ros::NodeHandle node;                                               
    };



public:

class WheelCar{
  public:
    dynamixel_controller::wheelstatus target,present;
    bool CalcTarget(double Velocity,double Curvature);
    bool MakePresentStatus();
    //bool ConvertRotationSpeedtoVelocity();
    void TimeUpdate();
    double wheels_width=0.287;//0.287[m] 
    double wheel_r=0.033;//0.033[m]
    int *left_rpm,*right_rpm;
    WheelCar(){
      left_rpm=new int(0);
      right_rpm=new int(0);
    }
    ~WheelCar(){
      delete left_rpm;
      delete right_rpm;
    }
};

MultiDynamixelControl(int dxl_quantity);
~MultiDynamixelControl(){
if(dxl != NULL){
  for(int i=0;i<Dynamixel_Quantity;i++){
    dxl[i].dxl_wb.torqueOff(dxl[i].dxl_id,&dxl[i].log);//when program shut down ,dynamixel is troque off
  }
  delete [] dxl;}
}

void PublishJointState();
int Dynamixel_Quantity;
bool USE_INITIAL_POSE=true;
SingleDynamixelControl* dxl;
std::vector<SingleDynamixelControl> singledynamixel;
std::vector<bool> ping_result;
WheelCar waffle;
ros::NodeHandle node;
ros::Timer timer;
ros::Subscriber sub_target_twist,sub_target_jointstate,sub_fl,sub_fr,sub_bl,sub_br;
ros::Publisher pub_jointstate,pub_velocitystatus,pub_velocitytarget,now_angle_pub;//0902now_angle_pub追加
sensor_msgs::JointState jointstate;
std_msgs::Float32MultiArray initial_rad,now_rad,target_rad,target_vel,control_dxl_rad,now_angle_array;//0902now_angle_array追加
int count=0;
std::map<std::string,int> joint_name;
//dynamixel_controller::wheelstatus present_status,target_status;

bool change_target=false;

void TimerCallback(const ros::TimerEvent& a);

bool Reboot();
bool ConnectAll();
bool SetupMode();
bool SetupPubSub();
void SetPortSetting(int num,const char* dxl_name,const char* port_name,int* baud_rate,int* dxl_id,
                    int* model_number,const char* mode,int* p_gain,int* i_gain,int* d_gain,double* initial_pos);
void CallbackTargetTwist(const geometry_msgs::Twist &target_twist);
void CallbackTargetJointState(const sensor_msgs::JointState &target_jointstate);
void CallbackTargetJointState_(const sensor_msgs::JointState &target_jointstate);

void notifyChangeTarget();
void CalcTargetRad();
void GetPosition();
void GetVelocity(double *left,double *right);
bool MoveTargetPosition();
bool MoveTargetSpeed(int id);
void MakePresentStatus();
void MoveTargetPositionSingle(int id);



};