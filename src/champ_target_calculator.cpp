#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>


class champ_target_calc
{
    public:
        ros::NodeHandle nh;
        sensor_msgs::JointState pub_joint,pub_fl,pub_fr,pub_bl,pub_br;
        ros::Subscriber sub;
        ros::Publisher pub=nh.advertise<sensor_msgs::JointState>("dxl_target_rad",1);
        ros::Publisher bl=nh.advertise<sensor_msgs::JointState>("dxl_target_rad_bl",10);
        ros::Publisher br=nh.advertise<sensor_msgs::JointState>("dxl_target_rad_br",10);
        ros::Publisher fl=nh.advertise<sensor_msgs::JointState>("dxl_target_rad_fl",10);
        ros::Publisher fr=nh.advertise<sensor_msgs::JointState>("dxl_target_rad_fr",10);

        champ_target_calc(){

            sub=nh.subscribe("/joint_states",1,&champ_target_calc::sub_target,this);
            pub_joint.name.resize(12);
            pub_joint.position.resize(12);
            pub_bl.name.resize(3);
            pub_bl.position.resize(3);
          pub_br.name.resize(3);
            pub_br.position.resize(3);
          pub_fl.name.resize(3);
            pub_fl.position.resize(3);
          pub_fr.name.resize(3);
            pub_fr.position.resize(3);


            }
    void sub_target(const sensor_msgs::JointState target){
    /*
 0 - front_left_link1
  1- front_left_link2
  2- front_left_link3
  3- front_right_link1
  4- front_right_link2
  5- front_right_link3
  6- back_left_link1
  7- back_left_link2
  8- back_left_link3
  9- back_right_link1
  10- back_right_link2
  11- back_right_link3

  */
 /*
    for(int i=0;i<12;i++){
    switch (i)
    {
    case 2:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=-target.position[i];
    break;

    case 3:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=target.position[i];
    break;

    case 4:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=-target.position[i];
    break;

     case 5:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=target.position[i];
    break;

     case 6:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=-target.position[i];
    break;

    case 7:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=target.position[i];
    break;

    case 8:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=-target.position[i];
    break;

     case 9:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=-target.position[i];
    break;

    case 10:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=-target.position[i];
    break;

    case 11:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=target.position[i];
    break;

    

    default:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=target.position[i];
    break;
    }

    }
    pub_joint.header=target.header;
    pub.publish(pub_joint);
    */
 for(int i=0;i<12;i++){
    switch (i)
    {
     case 0:pub_fl.name[0]=target.name[i];
         pub_fl.position[0]=target.position[i];
    break;
     case 1:pub_fl.name[1]=target.name[i];
         pub_fl.position[1]=target.position[i];
    break;
    case 2:pub_fl.name[2]=target.name[i];
         pub_fl.position[2]=-target.position[i];
         
    break;

    case 3:pub_fr.name[0]=target.name[i];
         pub_fr.position[0]=target.position[i];
    break;

    case 4:pub_fr.name[1]=target.name[i];
         pub_fr.position[1]=-target.position[i];
    break;

     case 5:pub_fr.name[2]=target.name[i];
         pub_fr.position[2]=target.position[i];
         
    break;

     case 6:pub_bl.name[0]=target.name[i];
         pub_bl.position[0]=-target.position[i];
    break;

    case 7:pub_bl.name[1]=target.name[i];
         pub_bl.position[1]=target.position[i];
    break;

    case 8:pub_bl.name[2]=target.name[i];
         pub_bl.position[2]=-target.position[i];
         bl.publish(pub_bl);
    break;

     case 9:pub_br.name[0]=target.name[i];
         pub_br.position[0]=-target.position[i];
    break;

    case 10:pub_br.name[1]=target.name[i];
         pub_br.position[1]=-target.position[i];
    break;

    case 11:pub_br.name[2]=target.name[i];
         pub_br.position[2]=target.position[i];
         
    break;

    

    default:pub_joint.name[i]=target.name[i];
         pub_joint.position[i]=target.position[i];
    break;
    }

    }
    fl.publish(pub_fl);
    fr.publish(pub_fr);
    bl.publish(pub_bl);
     br.publish(pub_br);
    }
    private:

};


int main(int argc, char *argv[]) 
{
    ros::init(argc,argv,"champ_target_converter");
    
    champ_target_calc converter;

    while(ros::ok()){
            ros::spin();   

    }

 


  return 0;
}
