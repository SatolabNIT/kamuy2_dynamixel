#include "dynamixel_hedder.h"


bool MultiDynamixelControl::WheelCar::CalcTarget(double velocity,double angular_velocity){

   // target.curvature =curvature;//1/r[1/m]
  //  target.turning_radius= 1/curvature;
    
    target.velocity_left=velocity+(angular_velocity*wheels_width/(2));
    target.velocity_right=velocity-(angular_velocity*wheels_width/(2));

   *left_rpm=30/(M_PI* wheel_r*0.229)*target.velocity_left;
   *right_rpm=-30/(M_PI*wheel_r*0.229)*target.velocity_right;

   target.stamp=ros::Time::now();
   
  //  ROS_INFO("left=%lf[m/s],%d[rpm]   right=%lf[m/s],%d[rpm]   kv=%lf  ang_v=%lf[rad/s]  curv=%lf[1/m]",
  //  target.velocity_left,*left_rpm,target.velocity_right,*right_rpm,target.velocity_ratio,target.angular_velocity,target.curvature);  
    
   return true;
}

void MultiDynamixelControl::WheelCar::TimeUpdate(){
    target.stamp=ros::Time::now();
   
}

bool MultiDynamixelControl::WheelCar::MakePresentStatus(){
    //present.rotation_speed_left,right is already knew
    present.stamp=ros::Time::now();
    
    present.velocity_left=wheel_r*present.rotation_speed_left;
    present.velocity_right=wheel_r*present.rotation_speed_right;
    
    present.velocity=(present.velocity_left+present.velocity_right)/2;
    present.velocity_ratio=present.velocity_right/present.velocity_left;

    present.turning_radius=(wheels_width/2)*(present.velocity_left+present.velocity_right)/(present.velocity_left-present.velocity_right);
    present.curvature=1/present.turning_radius;
    present.angular_velocity=present.curvature*present.velocity;

return true;
}
/////////////////////////////////////////////////////////////////////
MultiDynamixelControl::MultiDynamixelControl(int dxl_quantity){
  
  singledynamixel.reserve(dxl_quantity);
  ping_result.reserve(dxl_quantity);
  for(int i=0;i<dxl_quantity;i++){
    singledynamixel.emplace_back();
    ping_result.emplace_back();
  }
  //SingleDynamixelControl* singledynamixel[dxl_quantity];
  dxl=&singledynamixel[0];
  Dynamixel_Quantity=dxl_quantity;
  target_rad.data.resize(Dynamixel_Quantity);
  initial_rad.data.resize(Dynamixel_Quantity);
  control_dxl_rad.data.resize(Dynamixel_Quantity);
  now_rad.data.resize(Dynamixel_Quantity);
  jointstate.position.resize(Dynamixel_Quantity);
  target_vel.data.resize(Dynamixel_Quantity);
 // now_angle_array.data.resize(12);//0902追加

  for(int i=0;i<Dynamixel_Quantity;i++){
    target_rad.data[i]=0;
    initial_rad.data[i]=0;
    control_dxl_rad.data[i]=0;
    target_vel.data[i]=0;

  }

 timer=node.createTimer(ros::Duration(0.1),&MultiDynamixelControl::TimerCallback,this);
/*
ros::NodeHandle pnh("~");
std::string get_portname;
int get_id=0;
int get_baud_rate=0;
int get_model_number=0;
//if you dont know dxl id etc
pnh.getParam("dxl_port_name_roll",get_portname);
pnh.getParam("dxl_id_roll",get_id);
pnh.getParam("dxl_baud_rate_roll",get_baud_rate);
pnh.getParam("dxl_model_number_roll",get_model_number);
dxl_roll.port_name=get_portname.c_str();
dxl_roll.dxl_id=get_id;
dxl_roll.baud_rate=get_baud_rate;
dxl_roll.model_number=get_model_number;
*/
}

void MultiDynamixelControl::SetPortSetting(int num,const char* dxl_name,const char* port_name,
    int* baud_rate,int* dxl_id,int* model_number,const char* mode,int* p_gain,int* i_gain,int* d_gain,double* initial_pos){
  singledynamixel.at(num).port_name = port_name;
  singledynamixel.at(num).dxl_name = dxl_name;
  singledynamixel.at(num).baud_rate = *baud_rate;
  singledynamixel.at(num).dxl_id = *dxl_id;
  singledynamixel.at(num).model_number = *model_number;
  singledynamixel.at(num).mode = mode;
  singledynamixel.at(num).p_gain=*p_gain;
   singledynamixel.at(num).i_gain=*i_gain;
    singledynamixel.at(num).d_gain=*d_gain;
  singledynamixel.at(num).dxl_name_=dxl_name;
  singledynamixel.at(num).mode_=mode;
  singledynamixel.at(num).initial_pos= *initial_pos;
  std::string name= dxl_name;

  ROS_INFO("%s",name.c_str());
  joint_name[name]=num;
  ROS_INFO("Completed to setting in port id:%d,%s",singledynamixel.at(num).dxl_id,singledynamixel.at(num).dxl_name);
}


bool MultiDynamixelControl::ConnectAll(){
  //init and ping and set first joint mode
      for(auto itr = joint_name.begin(); itr != joint_name.end(); ++itr) {
        std::cout << "key = " << itr->first           // キーを表示
                        << ", val = " << itr->second << "\n";    // 値を表示
    }
  for(int i=0;i<Dynamixel_Quantity;i++){
      singledynamixel.at(i).init();
    if(singledynamixel.at(i).ping()==false){
      ROS_WARN("could not ping dxl_id=%d ,name=%s",singledynamixel.at(i).dxl_id,singledynamixel.at(i).dxl_name_.c_str());
      ping_result[i]=false;
      }
    else{
      ROS_INFO("Succeed ping dxl_id=%d ,name=%s",singledynamixel.at(i).dxl_id,singledynamixel.at(i).dxl_name_.c_str());
      ping_result[i]=true;
        if(strcmp(dxl[i].mode_.c_str(),dxl[i].position)==0){

      if(USE_INITIAL_POSE){
          initial_rad.data[i]=singledynamixel.at(i).initial_pos;
                
      }
      else{
            initial_rad.data[i]=singledynamixel.at(i).getposition();
      }
        }

    }
  }
  return true;
}

bool MultiDynamixelControl::SetupMode(){
   //change mode 0=current,1=velocity,2=position
  for(int i=0;i<Dynamixel_Quantity;i++){
    if(ping_result.at(i)){
      singledynamixel.at(i).changemode();
      singledynamixel.at(i).dxl_wb.torqueOff(singledynamixel.at(i).dxl_id);

      const char* change_item_name="Position_P_Gain";
      singledynamixel.at(i).dxl_wb.itemWrite(singledynamixel.at(i).dxl_id,change_item_name,singledynamixel.at(i).p_gain);
      int32_t* data;
      if(singledynamixel.at(i).dxl_wb.itemRead(singledynamixel.at(i).dxl_id,change_item_name,data)){
        ROS_WARN("%d",*data);
        }
        /*
      const char* change_item_nameI="Position_I_Gain";
      singledynamixel.at(i).dxl_wb.itemWrite(singledynamixel.at(i).dxl_id,change_item_nameI,singledynamixel.at(i).i_gain);
     
      const char* change_item_nameD="Position_D_Gain";
      singledynamixel.at(i).dxl_wb.itemWrite(singledynamixel.at(i).dxl_id,change_item_nameD,singledynamixel.at(i).d_gain);
      */

      singledynamixel.at(i).dxl_wb.torqueOn(singledynamixel.at(i).dxl_id);
      // singledynamixel.at(i).dxl_wb.torqueOff(singledynamixel.at(i).dxl_id);
  }
  }
  return true;
}

bool MultiDynamixelControl::SetupPubSub(){
  pub_jointstate=node.advertise<sensor_msgs::JointState>("dxl_jointstate",1);
  pub_velocitystatus=node.advertise<kamuy2_dynamixel::wheelstatus>("present_velocity_status",1);
  pub_velocitytarget=node.advertise<kamuy2_dynamixel::wheelstatus>("target_velocity",1);
  sub_target_twist=node.subscribe("/cmd_vel",1,&MultiDynamixelControl::CallbackTargetTwist,this);
  sub_target_jointstate=node.subscribe("/dxl_target_rad",1,&MultiDynamixelControl::CallbackTargetJointState_,this);
 // now_angle_pub=node.advertise<std_msgs::Float32MultiArray>("/now_angle",10);//0902追加、最初の角度を取得しpub
 sub_fl=node.subscribe("/dxl_target_rad_fl",1,&MultiDynamixelControl::CallbackTargetJointState_,this);
 sub_fr=node.subscribe("/dxl_target_rad_fr",1,&MultiDynamixelControl::CallbackTargetJointState_,this);
 sub_bl=node.subscribe("/dxl_target_rad_bl",1,&MultiDynamixelControl::CallbackTargetJointState_,this);
 sub_br=node.subscribe("/dxl_target_rad_br",1,&MultiDynamixelControl::CallbackTargetJointState_,this);
  return true;
}

bool MultiDynamixelControl::Reboot(){
  for(int i=0;i<Dynamixel_Quantity;i++){
    if(ping_result[i]==true){
      singledynamixel.at(i).dxl_wb.reboot(singledynamixel.at(i).dxl_id);
    }
  }
 return true;
}

void MultiDynamixelControl::CallbackTargetJointState(const sensor_msgs::JointState& target_msg_jointstate){
     ROS_INFO("%d",(int)target_msg_jointstate.name.size());
  for(int i=0;i<(int)target_msg_jointstate.name.size();i++){
  // ROS_INFO("subscribed %s",target_msg_jointstate.name[i].c_str());
   std::string command_name=target_msg_jointstate.name[i];
   /*
   auto itr=joint_name.find(command_name);
    if(itr!=joint_name.end()){
      int target_joint_number=itr->second;
      
     if(strcmp(dxl[target_joint_number].mode_.c_str(),dxl[target_joint_number].position)==0){//position mode
       target_rad.data[target_joint_number]=target_msg_jointstate.position[i];

     }
    else if(strcmp(dxl[target_joint_number].mode_.c_str(),dxl[target_joint_number].velocity)==0){//velocitymode
      target_vel.data[target_joint_number]=target_msg_jointstate.velocity[i];
    }

      */

          if(strcmp(dxl[i].mode_.c_str(),dxl[i].position)==0){//position mode
       target_rad.data[i]=target_msg_jointstate.position[i];
       MoveTargetPositionSingle(i);
     }
    else if(strcmp(dxl[i].mode_.c_str(),dxl[i].velocity)==0){//velocitymode
      target_vel.data[i]=target_msg_jointstate.velocity[i];
      MoveTargetSpeed(i);
    }

  
    
  }
   
 //MoveTargetPosition();
   /* }else{
    ROS_WARN("There is no dynamixel with joint name %s in the target message ",target_msg_jointstate.name[i].c_str());
   }
 */
 

}

void MultiDynamixelControl::CallbackTargetJointState_(const sensor_msgs::JointState& target_msg_jointstate){
     ROS_INFO("%d",(int)target_msg_jointstate.name.size());
  for(int i=0;i<(int)target_msg_jointstate.name.size();i++){
   ROS_INFO("subscribed %s",target_msg_jointstate.name[i].c_str());
   std::string command_name=target_msg_jointstate.name[i];
   auto itr=joint_name.find(command_name);
    if(itr!=joint_name.end()){
      int target_joint_number=itr->second;
      
     if(strcmp(dxl[target_joint_number].mode_.c_str(),dxl[target_joint_number].position)==0){//position mode
       target_rad.data[target_joint_number]=target_msg_jointstate.position[i];
       MoveTargetPositionSingle(target_joint_number);
     }
    else if(strcmp(dxl[target_joint_number].mode_.c_str(),dxl[target_joint_number].velocity)==0){//velocitymode
      target_vel.data[target_joint_number]=target_msg_jointstate.velocity[i];
       MoveTargetSpeed(target_joint_number);
    }
    
  }
}
}

void MultiDynamixelControl::CallbackTargetTwist(const geometry_msgs::Twist &target_twist){

    waffle.CalcTarget(target_twist.linear.x,target_twist.angular.z);

    dxl[0].dxl_wb.goalVelocity(dxl[0].dxl_id,*waffle.left_rpm,&dxl[0].log);
    dxl[1].dxl_wb.goalVelocity(dxl[1].dxl_id,*waffle.right_rpm,&dxl[1].log);

    //ROS_INFO("left_rpm=%d ,right_rpm=%d",*waffle.left_rpm,*waffle.right_rpm);
    //MakePresentStatus();

    //velocity[rpm]=(int Value)*0.299[rpm]
}

bool MultiDynamixelControl::MoveTargetSpeed(int i){
    if(ping_result[i]==true){
      if(strcmp(dxl[i].mode_.c_str(),dxl[i].velocity)==0){
         dxl[i].dxl_wb.goalVelocity(dxl[i].dxl_id,target_vel.data[i],&dxl[i].log);
         ROS_INFO("%d ,,,,%lf",i,target_vel.data[i]);
      }
    }
 return true;
}

void MultiDynamixelControl::TimerCallback(const ros::TimerEvent &a){
  PublishJointState();

}
void MultiDynamixelControl::PublishJointState(){
  GetPosition();
  jointstate.header.stamp = ros::Time::now();
  for(int i=0;i<Dynamixel_Quantity;i++){
    if(ping_result[i]==true){
      if(strcmp(dxl[i].mode_.c_str(),dxl[i].position)==0){
       // jointstate.position.at(i)=180*(now_rad.data[i]-initial_rad.data[i])/M_PI;
        jointstate.position.at(i)=(now_rad.data[i]-initial_rad.data[i])/M_PI;
      }
  }       
  }
  pub_jointstate.publish(jointstate);
}


void MultiDynamixelControl::GetPosition(){
  for(int i=0;i<Dynamixel_Quantity;i++){
    if(ping_result[i]==true){
      if(strcmp(dxl[i].mode_.c_str(),dxl[i].position)==0){
      ROS_INFO("%d->",singledynamixel.at(i).dxl_id);now_rad.data[i]=singledynamixel.at(i).getposition();
    } 
    }
  }
}

void MultiDynamixelControl::CalcTargetRad(){
  for(int i=0;i<Dynamixel_Quantity;i++){
    if(ping_result[i]==true){
      if(strcmp(dxl[i].mode_.c_str(),dxl[i].position)==0){
       control_dxl_rad.data[i]=target_rad.data[i]+initial_rad.data[i];
      
      }
    }
  }
}
void MultiDynamixelControl::MoveTargetPositionSingle(int i){
    if(ping_result[i]==true){
      if(strcmp(dxl[i].mode_.c_str(),dxl[i].position)==0){
      ROS_INFO("%d->",singledynamixel.at(i).dxl_id);
      now_rad.data[i]=singledynamixel.at(i).getposition();
      control_dxl_rad.data[i]=target_rad.data[i]+initial_rad.data[i];
       singledynamixel.at(i).dxl_wb.goalPosition(singledynamixel.at(i).dxl_id,control_dxl_rad.data[i],&singledynamixel.at(i).log);
    } 
    }
}

bool MultiDynamixelControl::MoveTargetPosition(){
  GetPosition();
  CalcTargetRad();
  for(int i=0;i<Dynamixel_Quantity;i++){
    if(ping_result[i]==true){
      if(strcmp(dxl[i].mode_.c_str(),dxl[i].position)==0){
      singledynamixel.at(i).dxl_wb.goalPosition(singledynamixel.at(i).dxl_id,control_dxl_rad.data[i],&singledynamixel.at(i).log);
    //   ROS_WARN("%d is moveded",i);
    }
    }
  }
  return true;
}



void MultiDynamixelControl::MakePresentStatus(){
  GetVelocity(&waffle.present.rotation_speed_left,&waffle.present.rotation_speed_right);
 
  if(waffle.MakePresentStatus()){
  pub_velocitystatus.publish(waffle.present);
  pub_velocitytarget.publish(waffle.target);
  }

}

void MultiDynamixelControl::GetVelocity(double *left_velocity,double *right_velocity){//rad/s

   *left_velocity=dxl[0].getvelocity();
   *right_velocity=-dxl[1].getvelocity();

    double v_left=waffle.wheel_r**left_velocity;
   double v_right=waffle.wheel_r**right_velocity;
   // double v_left=*left_velocity/(60*2*M_PI*waffle.wheel_r);
    //double v_right=*right_velocity/(60*2*M_PI*waffle.wheel_r);
    double velocity_robot=(v_left+v_right)/2;
    double angular_velocity_z=(v_right/waffle.wheels_width)-(v_left/waffle.wheels_width);

    //ROS_INFO("velocity =%lf  angular_velocity =%lf",*left_velocity,*right_velocity);
    ROS_INFO("velocity =%lf  angular_velocity =%lf",velocity_robot,angular_velocity_z);
}




////////////////////////////////////////////////////////////////////////////////
void MultiDynamixelControl::SingleDynamixelControl::init(){
 bool result = dxl_wb.init(port_name, baud_rate, &log);

if (result == false){

  }
  else{//ROS_INFO("Succeeded to init(%d)\n", baud_rate);
  }  
}



bool MultiDynamixelControl::SingleDynamixelControl::ping(){
    bool result = dxl_wb.ping(dxl_id,&model_number,&log);
  if (result == false)
  {

    return false;
  }
  else
  {
    ROS_INFO("Succeeded to ping\n");
    ROS_INFO("id : %d, model_number : %d\n", dxl_id, model_number);
   return true;
  }
}

float MultiDynamixelControl::SingleDynamixelControl::getposition()
{ 
  //dxl_wb.torqueOff(dxl_id, &log); 
   float radian=0;
   bool result=dxl_wb.getRadian(dxl_id, &radian, &log);
    if(result==false){ROS_INFO("could not get posision");}
    float degree=radian*180/M_PI;
    ROS_INFO("Radian: %f,Degree: %f", radian,degree);
    
    // now_angle_array.data[count]=radian;//0902追加
    // count++;//0902追加
    return radian;
}

float MultiDynamixelControl::SingleDynamixelControl::getvelocity()
{ 
  //dxl_wb.torqueOff(dxl_id, &log); 
   float angle_velocity=0;
   if(!dxl_wb.getVelocity(dxl_id, &angle_velocity, &log))
   {
     ROS_WARN("Error in get velocity");
   }

    return angle_velocity;
}

void MultiDynamixelControl::SingleDynamixelControl::changemode(){
  bool result=false;
  int mode_number=10;
  dxl_wb.torqueOff(dxl_id, &log);

if(strcmp(mode_.c_str(),current)==0){
mode_number=0;
}
else if(strcmp(mode_.c_str(),velocity)==0){
mode_number=1;

}
else if(strcmp(mode_.c_str(),position)==0){
mode_number=3;
}
else{
  ROS_INFO("mode name didn't matching");
}

switch (mode_number)
  {   
    case 0:
      result = dxl_wb.setCurrentControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
       // printf("Failed to set mode\n");
      }
      else
      {
       // printf("Succeeded to set mode\n");
        dxl_wb.torqueOn(dxl_id, &log);
      }

     break;
     
    case 1:
      result = dxl_wb.setVelocityControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
      //  printf("Failed to set mode\n");
      }
      else
      {
       // printf("Succeeded to set Velocity mode\n");
        dxl_wb.torqueOn(dxl_id, &log);
      }
     break;

    case 2:
      result = dxl_wb.setPositionControlMode(dxl_id, &log);
      if (result == false)
      {
       // printf("%s\n", log);
        //printf("Failed to set mode\n");
      }
      else
      {
       // printf("Succeeded to set mode\n");
        dxl_wb.torqueOn(dxl_id, &log);
      }
     break;

    case 3:
      result = dxl_wb.setExtendedPositionControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
      //  printf("Failed to set mode\n");
      }
      else
      {
      //  printf("Succeeded to set mode\n");
      }
     break;

    case 4:
      result = dxl_wb.setCurrentBasedPositionControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
      //  printf("Failed to set mode\n");
      }
      else
      {
       // printf("Succeeded to set mode\n");
      }
     break;

    case 5:
      result = dxl_wb.setPWMControlMode(dxl_id, &log);
      if (result == false)
      {
        printf("%s\n", log);
      //  printf("Failed to set mode\n");
      }
      else
      {
      //  printf("Succeeded to set mode\n");
      }
     break;
      case 6:
      result = dxl_wb.jointMode(dxl_id,0,0,&log);
      if (result == false)
      {
      //  printf("%s\n", log);
      //  printf("Failed to set mode\n");
      }
      else
      {
      //  printf("Succeeded to set mode\n");
        //dxl_wb.goalPosition(dxl_id,1024,&log);
        //sleep(2);

      }
     break;
    default:
      result = dxl_wb.setPositionControlMode(dxl_id, &log);
      if (result == false)
      {
       // printf("Failed to set mode\n");
      }
      else
      {
      //  printf("Succeeded to set mode\n");
      }
     break;
  }
  

}


void MultiDynamixelControl::SingleDynamixelControl::MotorControl(){//for testing dxl rotate

for(int i=0;i<2;i++){
if(i==1){dxl_wb.setReverseDirection(dxl_id,&log);ROS_INFO("reverse");}
else{dxl_wb.setNormalDirection(dxl_id,&log);ROS_INFO("normal");}
dxl_wb.goalPosition(dxl_id,(float)M_PI,&log);
sleep(2);

dxl_wb.goalPosition(dxl_id,(float)-M_PI*(3/4),&log);
sleep(2);
getposition();

dxl_wb.goalPosition(dxl_id,(float)M_PI*(3/4),&log);
sleep(2);
getposition();

dxl_wb.goalPosition(dxl_id,(float)M_PI,&log);
sleep(1);
  } 
}

bool MultiDynamixelControl::SingleDynamixelControl::ItemWrite(const char *item_name,double value)
{/*
  const char *item_name="Position_P_Gain";
  int *p_gain, *opemode,*offset;
  const char** log;
  dxl_wb.itemWrite(dxl_id,item_name,value);
  */
  return true;
}