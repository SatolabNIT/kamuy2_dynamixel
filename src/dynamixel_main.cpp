#include "dynamixel_hedder.h"


int main(int argc,char *argv[]){
ros::init(argc,argv,"dynamixel");
ros::NodeHandle nh;
ros::NodeHandle pnh("~");

  // MultiDynamixelControl::SingleDynamixelControl ms;//0902追加
  XmlRpc::XmlRpcValue member_list; //read config file
  pnh.getParam("member_list", member_list);
  ROS_ASSERT(member_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  ROS_INFO("member size: %i", (int)member_list.size());
std::vector<std::string> port_name{};
port_name.resize((int)member_list.size());
  MultiDynamixelControl multidynamixel((int)member_list.size());
  for (int32_t i = 0; i < member_list.size(); ++i)
  {
    ROS_INFO("read [%i]", i);
    int id = 0;
    std::string dxl_name = "";
    
    std::string mode = "";
    int baud_rate=0;
    int model_number;
    int p_gain,i_gain,d_gain;
    double initial_pos;
    if (!member_list[i]["dxl_id"].valid() || !member_list[i]["dxl_name"].valid())
    {
      ROS_WARN("No id or name");
      continue;
    }
    if (member_list[i]["dxl_id"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      id = static_cast<int>(member_list[i]["dxl_id"]);
    if (member_list[i]["dxl_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
      dxl_name = static_cast<std::string>(member_list[i]["dxl_name"]);
    if (member_list[i]["port_name"].getType() == XmlRpc::XmlRpcValue::TypeString)
      port_name[i] = static_cast<std::string>(member_list[i]["port_name"]);
    if (member_list[i]["baud_rate"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      baud_rate = static_cast<int>(member_list[i]["baud_rate"]);
    if (member_list[i]["model_number"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      model_number = static_cast<int>(member_list[i]["model_number"]);
    if (member_list[i]["mode"].getType() == XmlRpc::XmlRpcValue::TypeString)
      mode = static_cast<std::string>(member_list[i]["mode"]);
    if (member_list[i]["p_gain"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      p_gain = static_cast<int>(member_list[i]["p_gain"]);
          if (member_list[i]["i_gain"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      p_gain = static_cast<int>(member_list[i]["i_gain"]);
          if (member_list[i]["d_gain"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      p_gain = static_cast<int>(member_list[i]["d_gain"]);
    if (member_list[i]["initial_pos"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      initial_pos = static_cast<double>(member_list[i]["initial_pos"]);
    ROS_INFO("[%i] id: %i, dxl_name: %s, port_name: %s, baud_rate:%d, model_number:%d,mode: %s",
               i, id, dxl_name.c_str(), port_name[i].c_str(),baud_rate,model_number,mode.c_str());
    multidynamixel.SetPortSetting(i,dxl_name.c_str(), port_name[i].c_str(),
    &baud_rate,&id,&model_number,mode.c_str(),&p_gain,&i_gain,&d_gain,&initial_pos);
  }

multidynamixel.ConnectAll();

bool continue_without_a_part_of_dxl=true;

if(!continue_without_a_part_of_dxl){
  for(int i=0;i<(int)member_list.size();i++){
    if(multidynamixel.ping_result[i]==false){
      return 0;
    }
  }
  /*
  int32_t* data;
  ROS_INFO("%d",multidynamixel.dxl[0].dxl_id);
  const char *item_name="Velocity_Limit";
  //multidynamixel.dxl[0].dxl_wb.itemWrite(multidynamixel.dxl[0].dxl_id,item_name,1023);
  //multidynamixel.dxl[1].dxl_wb.itemWrite(multidynamixel.dxl[1].dxl_id,item_name,1023);
  //multidynamixel.dxl[0].dxl_wb.itemRead(multidynamixel.dxl[0].dxl_id,item_name,data,&multidynamixel.dxl[0].log);
  ROS_WARN("%d",*data);
  */
  multidynamixel.SetupMode();

  multidynamixel.SetupPubSub();
  ros::Rate loop_rate(120);
  ros::spin();
  /*
  ros::AsyncSpinner spinner(4);
  spinner.start();

 // multidynamixel.now_angle_pub.publish(multidynamixel.now_rad);//0902追加
  ROS_INFO("waiting topics...");

  while(ros::ok()){
 loop_rate.sleep();
}


  spinner.stop();
  */
}
else{
/*
int32_t* data;
ROS_INFO("%d",multidynamixel.dxl[0].dxl_id);
const char *item_name="Velocity_Limit";
//multidynamixel.dxl[0].dxl_wb.itemWrite(multidynamixel.dxl[0].dxl_id,item_name,1023);
//multidynamixel.dxl[1].dxl_wb.itemWrite(multidynamixel.dxl[1].dxl_id,item_name,1023);
//multidynamixel.dxl[0].dxl_wb.itemRead(multidynamixel.dxl[0].dxl_id,item_name,data,&multidynamixel.dxl[0].log);
ROS_WARN("%d",*data);
*/
multidynamixel.SetupMode();
multidynamixel.SetupPubSub();
ros::Rate loop_rate(200);
//ros::MultiThreadedSpinner spinner(4);
//spinner.spin();
ros::spin();
//ros::AsyncSpinner spinner(4);
  //spinner.start();
  //ros::waitForShutdown();

while (ros::ok())
{ 

  loop_rate.sleep();

}
//spinner.stop();

/*



ros::AsyncSpinner spinner(12);
ROS_INFO("waiting topics...");
  spinner.start();
  while(ros::ok()){
 loop_rate.sleep();
}
spinner.stop();

}
*/



}
return 0;}

