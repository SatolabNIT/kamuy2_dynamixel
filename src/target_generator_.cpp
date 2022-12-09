#include "dog_target_generator.h"
using namespace std;

void timer_callback(const ros::TimerEvent& e)
{
  std_msgs::String msg;
  msg.data = "hello world!";
  ROS_INFO("publish: %s", msg.data.c_str());
 // chatter_pub.publish(msg);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"listener");
    XmlRpc::XmlRpcValue member_list; //read config file
    ros::NodeHandle pnh("~");
  pnh.getParam("member_list", member_list);
  ROS_ASSERT(member_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO("member size: %i", (int)member_list.size());

    dog_robot_class dog((int)member_list.size());
  for (int32_t i = 0; i < member_list.size(); ++i)
  {
    ROS_INFO("read [%i]", i);
    int id = 0;
    std::string dxl_name = "";
    std::string port_name = "";
    std::string mode = "";
    int baud_rate=0;
    int model_number;
    if (!member_list[i]["dxl_id"].valid() || !member_list[i]["dxl_name"].valid())
    {
      ROS_WARN("No id or name");
      continue;
    }

    if (member_list[i]["dxl_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
      dxl_name = static_cast<std::string>(member_list[i]["dxl_name"]);

    ROS_INFO("dxl_name: %s",dxl_name.c_str());
    dog.joint_state(i,dxl_name);
  }
    
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timer_callback);
    ros::spin();
}



void dog_robot_class::angle_publish(const geometry_msgs::Twist& msg){
    //twist_msg
    float vel_x=msg.linear.x;
    float vel_y=msg.linear.y;
    float vel_z=msg.linear.z;
    float ang_x=msg.angular.x;
    float ang_y=msg.angular.y;
    float ang_z=msg.angular.z;
    if(robot_active){
      make_orbit(vel_x,ang_x);
    }
    
    
}

// void angle_culculate(){

// }

float dog_robot_class::inverce_kinetic_theta1(float y){
    theta1=atan2(y,height);

    return theta1;
}
float dog_robot_class::inverce_kinetic_theta2(float x,float y,float z){
    float v31=sqrt(pow(x,2)+pow(y,2));//原点の分だけずらす
    float z31=z;
    float theta2=acos((pow(v31,2)+pow(z31,2)+pow(L2,2)-pow(L3,2))/(2*L2*sqrt(pow(v31,2)+pow(z31,2))))+atan(z31/v31);
    return theta2;
}
float dog_robot_class::inverce_kinetic_theta3(float x,float y,float z,float theta2){
    float v31=sqrt(pow(x,2)+pow(y,2));//原点の分だけずらす
    float z31=z;
    float theta3=atan2((z31-L2*sin(theta2)),(v31-L2*cos(theta2)));
    return theta3;
}


void dog_robot_class::orbit_list(int judge,std::vector<float> theta_front){

    if (judge>0){//左後ろと右前の角度の配列作成
        for(int i=0;i<7;i++){
          // fr1.insert(cend(fr1),start_theta1);
          // fr2.insert(cend(fr2),-1*start_theta2);
          // fr3.insert(cend(fr3),-1*start_theta3);
          // bl1.insert(cend(bl1),start_theta1);
          // bl2.insert(cend(bl2),start_theta2);
          // bl3.insert(cend(bl3),start_theta3);
        }
        for (int i=1;i<7;i++){
          
          fr1.insert(cend(fr1),theta_front[i*3]);
          fr2.insert(cend(fr2),-1*theta_front[i*3+2]);
          fr3.insert(cend(fr3),-1*theta_front[i*3+3]);
          bl1.insert(cend(bl1),theta_front[i*3]);
          bl2.insert(cend(bl2),theta_front[i*3+2]);
          bl3.insert(cend(bl3),theta_front[i*3+3]);
          
        }
    }
   else{//左前と右後ろの角度の配列作成
        for(int i=0;i<7;i++){
          fl1.insert(cend(fl1),theta_front[i*3]);
          fl2.insert(cend(fl2),theta_front[i*3+2]);
          fl3.insert(cend(fl3),theta_front[i*3+3]);
          br1.insert(cend(br1),theta_front[i*3]);
          br2.insert(cend(br2),-1*theta_front[i*3+2]);
          br3.insert(cend(br3),-1*theta_front[i*3+3]);
        }
        for (int i=1;i<7;i++){
          fl1.insert(cend(fl1),start_theta1);
          fl2.insert(cend(fl2),start_theta2);
          fl3.insert(cend(fl3),start_theta3);
          br1.insert(cend(br1),start_theta1);
          br2.insert(cend(br2),-1*start_theta2);
          br3.insert(cend(br3),-1*start_theta3);
        }
    }

}

void dog_robot_class::make_orbit(float vel_x,float ang_x){//軌道作成
    // float a=vel_x/0.5*0.15;//進行速度を長軸の半分に
    float a=0.05;
    std::cout<<a<<std::endl;
    float b=0.05;//足をどれだけ上げるかは固定(m)
    float thetaxy=0;//joyをどの角度に傾けるかを用いる、それが進行方向に
    float origin_=0;//原点を足の接地点とするため、楕円は長軸の半分だけずれる
    height=sqrt(pow(L2,2)+pow(L3,2)-2*L2*L3*cos(start_theta3));
    //理想的な足の配置、インデックス0から順番に７点を指定し、それをたどるようにする、
    float orbit_magnitude[][3]={{(-a+origin_+0.00000001f)*cos(thetaxy),(-a+origin_+0.00000001f)*sin(thetaxy),0},{(-sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(-sqrt(3.0f)/2*a+origin_)*sin(thetaxy),1/2*b},{(-1/2*a+origin_)*cos(thetaxy),(-1/2*a+origin_)*sin(thetaxy),sqrt(3.0f)/2*b},{(0+origin_)*cos(thetaxy),(0+origin_)*sin(thetaxy),b},{(1/2*a+origin_)*cos(thetaxy),(1/2*a+origin_)*sin(thetaxy),sqrt(3.0f)/2*b},{(sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(sqrt(3.0f)/2*a+origin_)*sin(thetaxy),1/2*b},{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0}};
    float orb_mag_rotate[][3]={{-1*orbit_magnitude[0][2]+height,orbit_magnitude[0][1],orbit_magnitude[0][0]},{-1*orbit_magnitude[1][2]+height,orbit_magnitude[1][1],orbit_magnitude[1][0]},{-1*orbit_magnitude[2][2]+height,orbit_magnitude[2][1],orbit_magnitude[2][0]},{-1*orbit_magnitude[3][2]+height,orbit_magnitude[3][1],orbit_magnitude[3][0]},{-1*orbit_magnitude[4][2]+height,orbit_magnitude[4][1],orbit_magnitude[4][0]},{-1*orbit_magnitude[5][2]+height,orbit_magnitude[5][1],orbit_magnitude[5][0]},{-1*orbit_magnitude[6][2]+height,orbit_magnitude[6][1],orbit_magnitude[6][0]},};
    // float orb_mag_rotate[][3]={{0,0,0.1},{0,0,0.1},{0,0,0.1},{0,0,0.1},{0,0,0.1},{0,0,0.1},{0,0,0.1}};
    vector<float> theta_front {};
    for (int i=0;i<7;i++){
        float theta1=inverce_kinetic_theta1(orb_mag_rotate[i][1]);
        float theta2=inverce_kinetic_theta2(orb_mag_rotate[i][0],orb_mag_rotate[i][1],orb_mag_rotate[i][2]);
        float theta3=inverce_kinetic_theta3(orb_mag_rotate[i][0],orb_mag_rotate[i][1],orb_mag_rotate[i][2],theta2);
        theta_front.insert(cend(theta_front),{theta1,3.14f/2-theta2,theta3}); //iはtheta1,2*iはtheta2,3*iはtheta3   
    }
    for (int i=0;i<=1;i++){
      orbit_list(i,theta_front);
    }

    // std::size_t size=fl1.size();
    // std::cout<<size<<std::endl;
    // joint_rad.position[i]=teee;
    for (int i=0;i<=12;i++){
      std::this_thread::sleep_for(std::chrono::milliseconds(5));//5ミリ秒止める
      joint_rad.position[0] =fl1[i];
      joint_rad.position[1] =fl2[i];
      joint_rad.position[2] =fl3[i];
      joint_rad.position[3] =fr1[i];
      joint_rad.position[4] =fr2[i];
      joint_rad.position[5] =fr3[i];
      joint_rad.position[6] =bl1[i];
      joint_rad.position[7] =bl2[i];
      joint_rad.position[8] =bl3[i];
      joint_rad.position[9] =br1[i];
      joint_rad.position[10]=br2[i];
      joint_rad.position[11]=br3[i];
      angle_pub.publish(joint_rad);
    }
    // ROS_INFO("5");
    // publishする
    
    //theta_front、足の軌道をリセット
    vector<float>().swap(theta_front);
    vector<float>().swap(fl1);
    vector<float>().swap(fl2);
    vector<float>().swap(fl3);
    vector<float>().swap(fr1);
    vector<float>().swap(fr2);
    vector<float>().swap(fr3);
    vector<float>().swap(bl1);
    vector<float>().swap(bl2);
    vector<float>().swap(bl3);
    vector<float>().swap(br1);
    vector<float>().swap(br2);
    vector<float>().swap(br3);
    // std::size_t size=fl1.size();
    // std::cout<<size<<std::endl;
    // joint_rad.position[i]=teee;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));//5ミリ秒止める

}

void dog_robot_class::joint_state(int num,std::string name_joint){
    
    joint_rad.name.at(num)=name_joint;

}

void dog_robot_class::button_push(const std_msgs::Float32MultiArray& msg_push){
    if(msg_push.data[0]>0){//L1
      
      for (int i=0;i<4;i++){
        joint_rad.position[0]=start_theta1;
        joint_rad.position[1]=start_theta2;
        joint_rad.position[2]=start_theta3;
        joint_rad.position[3]=start_theta1*-1;
        joint_rad.position[4]=-1*start_theta2;
        joint_rad.position[5]=-1*start_theta3;
        joint_rad.position[6]=start_theta1*-1;
        joint_rad.position[7]=start_theta2;
        joint_rad.position[8]=start_theta3;
        joint_rad.position[9]=start_theta1;
        joint_rad.position[10]=-1*start_theta2;
        joint_rad.position[11]=-1*start_theta3;
        angle_pub.publish(joint_rad);
        }
      
      
    }

    if(msg_push.data[1]>0){//R1
      
    }

    if(msg_push.data[2]>0){//cross
      robot_active=true;
    }

    if(msg_push.data[3]>0){//Triangle
      robot_active=false;
    }
}