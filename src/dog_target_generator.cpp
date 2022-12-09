#include "dog_target_generator.h"
using namespace std;

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

    ros::spin();
}


void dog_robot_class::angle_publish(const geometry_msgs::Twist& msg){
    //twist_msg
  //ps5コントローラーから受け取る値
    float vel_x=msg.linear.x;//LStickの左右の倒し具合、角度関係:右-1.0~1.0左
    float vel_y=msg.linear.y;//LStickの上下の倒し具合、角度関係:下-1.0~1.0上
    float vel_z=msg.linear.z;//十字キー上下、上1.0、下-1.0
    float ang_x=msg.angular.x;//十字キー左右、左1.0、右-1.0
    float ang_y=msg.angular.y;//RStickの左右の倒し具合、角度関係:右-1.0~1.0左
    float ang_z=msg.angular.z;//RStickの上下の倒し具合、角度関係:下-1.0~1.0上

    // stop_roll();//imuの角度がthreshold_roll値以上のときに起動、常に動かす
    if(vel_z!=0){
      side_leg_updown(vel_z);
    }
    if(R3_switch||L3_switch){
      vel_x=0;
      vel_y=0;
      ang_y=0;
      ang_z=0;
      
    }
    // std::cout<<vel_x<<endl;
    // std::cout<<ang_z<<endl;
    //手動姿勢制御
    // if(robot_mode&&robot_move&&end_pub){//crossボタンで動く、動きを制御
    if((robot_mode&&robot_move)||crawl_mode){//手動姿勢制御
    //・
      // end_pub=false;//次が入ってこないようにする
      if(vel_x!=0||vel_y!=0){//左スティックの入力値があれば、歩きを許可
        robot_walk=true;
      }
      if(ang_y!=0||ang_z!=0){//右スティックの入力値があれば、体の回転を許可
        robot_rotate=true;
      }
      if(ang_x!=0){//L2のみを入力左旋回、R2のみを入力右旋回、旋回を許可
        robot_senkai=true;
      }

      if(robot_walk){//歩き許可されてるなら
        if(robot_rotate&&!crawl_mode){//体の回転も許可されてるなら
          // rotate_body(ang_y,ang_z,vel_z);
          rotate_body_by_height(ang_y,ang_z);
          std::cout<<"回転かつ歩き"<<endl;
        }
        make_orbit(vel_x,vel_y,ang_x,vel_z);
        std::cout<<"歩きのみ"<<endl;
      }
      else if(robot_senkai){//旋回許可されてるなら
        if(robot_rotate&&!crawl_mode){//体の回転も許可されてるなら
          // rotate_body(ang_y,ang_z,vel_z);
          rotate_body_by_height(ang_y,ang_z);
          std::cout<<"回転かつ旋回"<<endl;
        }
        senkai(ang_x,vel_z);
        std::cout<<"旋回のみ"<<endl;
      }
      else if(robot_rotate&&!crawl_mode){//体の回転許可されてるなら
        // rotate_body(ang_y,ang_z,vel_z);
        rotate_body_by_height(ang_y,ang_z);
        std::cout<<"回転のみ"<<endl;
      }
      else{//何も入力値がなかった。
        // start_position();cd
        zero_velocity();
        std::cout<<"入力なし"<<endl;
      }
      //リセット
      robot_walk=false;
      robot_senkai=false;
      robot_rotate=false;
      end_pub=true;
    }

    // 自動姿勢制御
    if(robot_mode&&robot_keep){//PSButton+SHAREで起動
      
      if(vel_x!=0||vel_y!=0){//左スティックの入力値があれば、歩きを許可
        robot_walk=true;
      }
      if(ang_x!=0){//十字左右キーの入力値があれば、旋回を許可
        robot_senkai=true;
      }

      if(robot_walk){//歩き許可されてるなら
        rotate_body(0,0,0);//imuで制御
        make_orbit(vel_x,vel_y,ang_x,vel_z);
        std::cout<<"imuの歩き"<<endl;
      }
      else if(robot_senkai){//旋回許可されてるなら
        rotate_body(0,0,0);//imuで制御
        senkai(ang_x,vel_z);
        std::cout<<"imuの旋回"<<endl;
      }
      
      else{//何も入力値がなかった。
        // start_position();
        rotate_body(0,0,0);//imuで制御
        std::cout<<"imu入力なし"<<endl;
      }

      if(std::abs(ang_y)>0.01f||std::abs(ang_z)>0.01){//右スティックでクローラ動く

        crwlr_move(ang_y,ang_z);//クローラを動かす
        std::cout<<"クローラ動く"<<endl;
        }
      else{
        crwlr_value_reset();//velocityを０にする
      }
      //リセット
      robot_walk=false;
      robot_senkai=false;
    }
//車モード//imu使用する、roll角のみ使い、地面に脚が着くようにする


    if(robot_mode&&car_mode){//Triangleボタンで動く、動きを制御
      
      if(vel_x!=0||vel_y!=0){//左スティックの入力値があれば、歩きを許可
        robot_walk=true;
      }
      if(ang_x!=0){//十字キーの入力値があれば、旋回を許可
        robot_senkai=true;
      }
      if(robot_walk){//歩き許可されてるなら
        rotate_body(0,0,vel_z);
        make_orbit(vel_x,vel_y,ang_x,vel_z);
        std::cout<<"imuの歩き"<<endl;
      }
      else if(robot_senkai){//旋回許可されてるなら
        rotate_body(0,0,vel_z);
        senkai(ang_x,vel_z);
        std::cout<<"imuの旋回"<<endl;
      }

      else{//何も入力値がなかった。
        // start_position();
        rotate_body(0,0,vel_z);
        std::cout<<"imu入力なし"<<endl;
      }
      if(std::abs(ang_y)>0.01f||std::abs(ang_z)>0.01){//右スティックでタイヤ動く

        crwlr_move(ang_y,ang_z);
        std::cout<<"クローラ動く"<<endl;
        }
      else{//velocity値を全部０にする
        crwlr_value_reset();
      }
      //リセット
      robot_walk=false;
      robot_senkai=false;
    }

    //クローラモードここから
    // if(crawlr_mode){
    //   // if(robot_keep){
    //   //   rotate_body(0,0,0);
    //   // }
    //   if(ang_y!=0||ang_z!=0){
    //     crwlr_move(ang_y,ang_z);
    //     std::cout<<"クローラ動く"<<endl;
    //     }
    //   else{
    //     crwlr_value_reset();
    //   }
    //   if((vel_z!=0)){//十字キー上下
    //     if(L1_switch){
    //       leg_height_th2(vel_z,0);
    //       // std::cout<<"L1flipper_th2"<<endl;
    //     }
    //     if(R1_switch){
    //       leg_height_th2(vel_z,1);
    //       // std::cout<<"R1flipper_th2"<<endl;
    //     }
    //     if(L2_switch){
    //       leg_height_th2(vel_z,2);
    //       // std::cout<<"L2flipper_th2"<<endl;
    //     }
    //     if(R2_switch){
    //       leg_height_th2(vel_z,3);
    //       // std::cout<<"R2flipper_th2"<<endl;
    //     }
        
    //   }
    //   if((ang_x!=0)){//十字キー左右
    //     if(L1_switch){
    //       leg_height_th3(ang_x,0);
    //       // std::cout<<"L1flipper_th3333"<<endl;
    //     }
    //     if(R1_switch){
    //       leg_height_th3(ang_x,1);
    //       // std::cout<<"R1flipper_th3333"<<endl;
    //     }
    //     if(L2_switch){
    //       leg_height_th3(ang_x,2);
    //       // std::cout<<"L2flipper_th3333"<<endl;
    //     }
    //     if(R2_switch){
    //       leg_height_th3(ang_x,3);
    //       // std::cout<<"R2flipper_th3333"<<endl;
    //     }
        
    //   }

    // }//クローラモードここまで
    if(kick_car_mode){//Triangleで起動、前は脚、後ろはクローラ
      if(ang_y!=0||ang_z!=0||vel_x!=0||vel_y!=0){
        std::cout<<"kick_car動く"<<endl;
        kick_car_move=true;
        rotate_body(0,0,0);//imu
        crwlr_move(ang_y,ang_z);//クローラ動かす
        kick_car(vel_x,vel_y,ang_y,ang_z);//このモードの脚のpositionをpubする
      }
      else{
        kick_car_move=false;
        start_kick_car();//kick_carモードの初期姿勢になる
        crwlr_value_reset();//クローラの速度をすべて０にする
        std::cout<<"kick_car入力なし"<<endl;
      }
    }

    // if(crawl_mode){//Triangleで起動、前は脚、後ろはクローラ
    //   if(ang_y!=0||ang_z!=0||vel_x!=0||vel_y!=0){
    //     std::cout<<"crawl_walk動く"<<endl;
    //     crawl_walk_2(vel_x,vel_y,ang_x,vel_z);
    //   }
    //   else{
    //     start_position();//初期姿勢になる
    //     std::cout<<"crawl_walk入力なし"<<endl;
    //   }
    // }

    if(arm_mode){//Triangleで起動、前は脚、後ろはクローラ
      if(ang_y!=0||ang_z!=0){
        // std::cout<<"arm_mode動く"<<endl;
        rotate_body_by_height(ang_y,ang_z);
      }
      // else{
      //    start_position();//初期姿勢になる
      // //   std::cout<<"crawl_walk入力なし"<<endl;
      // }
    }
    if(spider_mode){
      if(vel_x!=0||vel_y!=0||ang_y!=0||ang_z!=0){
        crwlr_move(ang_y,ang_z);
        spider_orb(vel_x,vel_y,ang_y,ang_z);
      }
      else{
        crwlr_value_reset();
      }
    }
    }  
    

std::vector<float> dog_robot_class::rotate_pitch_roll(std::vector<float> transform,float pitch,float roll){//座標を渡し、付け根、接地点の座標を更新、角度も算出//transform(接地点)、体の回転する目標角度pitch,roll
  
  //接地点の座標
  //pitch回転（x軸回転をする）させた後のxyz座標
  float body_ang_x_pitch=transform[0];
  float body_ang_y_pitch=transform[1]*cos(pitch)-transform[2]*sin(pitch);
  float body_ang_z_pitch=transform[2]*cos(pitch)+transform[1]*sin(pitch);
  //roll回転（y軸回転をする）させた後のxyz座標
  float body_ang_x_roll=body_ang_x_pitch*cos(roll)+body_ang_z_pitch*sin(roll);
  float body_ang_y_roll=body_ang_y_pitch;
  float body_ang_z_roll=-1*body_ang_x_pitch*sin(roll)+body_ang_z_pitch*cos(roll);


  // float theta1=inverce_kinetic_theta1(body_ang_y_roll,transform[1],height);
  float theta1=pitch;
  float theta2=inverce_kinetic_theta2(body_ang_x_roll,body_ang_y_roll,body_ang_z_roll,transform[0],transform[1],height);
  float theta3=inverce_kinetic_theta3(body_ang_x_roll,body_ang_y_roll,body_ang_z_roll,theta2,transform[0],transform[1],height);

  // count_t+=1;
  // if(count_t<=2){
  //   std::cout<<"ピッチ"<<body_ang_x_pitch<<","<<body_ang_y_pitch<<","<<body_ang_z_pitch<<endl;
  // }

  transform_leg[0]=theta1;
  transform_leg[1]=theta2;
  transform_leg[2]=theta3;
  transform_leg[3]=height-body_ang_z_roll;//現在の足の接地点、付け根間の距離

  // std::cout<<theta1<<std::endl;
  return transform_leg;
}

void dog_robot_class::rotate_body(float ang_y,float ang_z,float vel_z){//体の回転に使用
  float pitch = ang_y*body_rotate_pitch;//y軸周り
  float roll = ang_z*-1.0f*body_rotate_roll;//x軸周り

  if(robot_keep){
    float angle_pgain=-0.5;
    pitch=angle_pgain*imu_pitch_y;
    roll=angle_pgain*imu_roll_x;
  }
  if(car_mode){//roll角のみを扱うため
    pitch=0.0f;
  }

  if(vel_z>0){
  //     cross_height=-0.02;
  //     std::this_thread::sleep_for(chrono::milliseconds(1));
  // }
  // else if(vel_z<0){
  //     cross_height=0.02f;
  //     std::this_thread::sleep_for(chrono::milliseconds(1));
  }
  std::vector<float> fl_xyz = {length/2.0f+origin_culculate,1.0f*width/2.0f,0};//接地点の座標
  std::vector<float> fr_xyz = {1.0f*length/2.0f+origin_culculate,-1.0f*width/2.0f,0};
  std::vector<float> bl_xyz = {-1.0f*length/2.0f-origin_culculate,1.0f*width/2.0f,0};
  std::vector<float> br_xyz = {-1.0f*length/2.0f-origin_culculate,-1.0f*width/2.0f,cross_height};

  rotate_pitch_roll(fl_xyz,pitch,roll);
  fl_r=transform_leg;
  now_height_fl=transform_leg[3];

  rotate_pitch_roll(fr_xyz,pitch,roll);
  fr_r=transform_leg;
  now_height_fr=transform_leg[3];

  rotate_pitch_roll(bl_xyz,pitch,roll);
  bl_r=transform_leg;
  now_height_bl=transform_leg[3];

  rotate_pitch_roll(br_xyz,pitch,roll);
  br_r=transform_leg;
  now_height_br=transform_leg[3];

  // std::cout<<"前左::"<<now_height_fl<<"::前右"<<now_height_fr<<"::後左::"<<now_height_bl<<"::後右::"<<now_height_br<<endl;
  if(!robot_walk&&!robot_senkai&&!crawlr_mode&&!kick_car_move&&!crawl_mode){//歩き、旋回、クローラ、前後モード(入力あり)ではないとき、回転かつのときに、使いたくない
    joint_rad.position[0]=fl_r[0]+always_theta1;
    joint_rad.position[1]=(fl_r[1]+p/2.0f);
    joint_rad.position[2]=-1.0f*fl_r[2];
    joint_rad.position[3]=fr_r[0]-always_theta1;
    joint_rad.position[4]=-1.0f*(fr_r[1]+p/2.0f);
    joint_rad.position[5]=fr_r[2];
    joint_rad.position[6]=-1.0f*bl_r[0]-always_theta1;
    joint_rad.position[7]=-1.0f*(bl_r[1]+p/2.0f);
    joint_rad.position[8]=bl_r[2];
    joint_rad.position[9]=-1.0f*br_r[0]+always_theta1;
    joint_rad.position[10]=(br_r[1]+p/2.0f);
    joint_rad.position[11]=-1.0f*br_r[2];
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
    // pub(joint_rad);
    value_reset();
  }
  
}

void dog_robot_class::rotate_body_by_height(float ang_y,float ang_z){
  float pitch = 0.0f*body_rotate_pitch;//y軸周り
  float roll = 0.0f*body_rotate_roll;//x軸周り

  float thetaxy=atan2(ang_y,ang_z);
  float magnitude=sqrt(pow(ang_y,2)+pow(ang_z,2));

  float fl_height=(((ang_y)+(ang_z))+1.52f)/3.0f*max_rb_height;
  float fr_height=((-1.0f*(ang_y)+(ang_z))+1.52f)/3.0f*max_rb_height;
  float bl_height=(((ang_y)-(ang_z))+1.52f)/3.0f*max_rb_height;
  float br_height=((-1.0f*(ang_y)-(ang_z))+1.52f)/3.0f*max_rb_height;
  if(body_height!=0.0f){
    fl_height=body_height;
    fr_height=body_height;
    bl_height=body_height;
    br_height=body_height;
  }
  // std::cout<<"fl_height::"<<fl_height<<endl;
  // std::cout<<"fr_height::"<<fr_height<<endl;
  // std::cout<<"bl_height::"<<bl_height<<endl;
  // std::cout<<"br_height::"<<br_height<<endl;
  // std::cout<<""<<endl;
  
  std::vector<float> fl_xyz = {length/2.0f+origin_culculate,1.0f*width/2.0f,fl_height};//接地点の座標
  std::vector<float> fr_xyz = {1.0f*length/2.0f+origin_culculate,-1.0f*width/2.0f,fr_height};
  std::vector<float> bl_xyz = {-1.0f*length/2.0f-origin_culculate,1.0f*width/2.0f,bl_height};
  std::vector<float> br_xyz = {-1.0f*length/2.0f-origin_culculate,-1.0f*width/2.0f,br_height};

  rotate_pitch_roll(fl_xyz,pitch,roll);
  fl_r=transform_leg;
  now_height_fl=transform_leg[3];
std::cout<<"fl::"<<now_height_fl<<endl;
  rotate_pitch_roll(fr_xyz,pitch,roll);
  fr_r=transform_leg;
  now_height_fr=transform_leg[3];
std::cout<<"fr::"<<now_height_fr<<endl;
  rotate_pitch_roll(bl_xyz,pitch,roll);
  bl_r=transform_leg;
  now_height_bl=transform_leg[3];
std::cout<<"bl::"<<now_height_bl<<endl;
  rotate_pitch_roll(br_xyz,pitch,roll);
  br_r=transform_leg;
  now_height_br=transform_leg[3];
std::cout<<"br::"<<now_height_br<<endl;
  // std::cout<<"前左::"<<now_height_fl<<"::前右"<<now_height_fr<<"::後左::"<<now_height_bl<<"::後右::"<<now_height_br<<endl;
  //歩き、旋回、クローラ、前後モード(入力あり)ではないとき、回転かつのときに、使いたくない
if(robot_rotate&&!crawl_mode&&!robot_senkai&&!robot_walk){
  joint_rad.position[0]=fl_r[0]+always_theta1;
  joint_rad.position[1]=(fl_r[1]+p/2.0f);
  joint_rad.position[2]=-1.0f*fl_r[2];
  joint_rad.position[3]=fr_r[0]+always_theta1;
  joint_rad.position[4]=-1.0f*(fr_r[1]+p/2.0f);
  joint_rad.position[5]=fr_r[2];
  joint_rad.position[6]=-1.0f*bl_r[0]-always_theta1;
  joint_rad.position[7]=-1.0f*(bl_r[1]+p/2.0f);
  joint_rad.position[8]=bl_r[2];
  joint_rad.position[9]=-1.0f*br_r[0]-always_theta1;
  joint_rad.position[10]=(br_r[1]+p/2.0f);
  joint_rad.position[11]=-1.0f*br_r[2];
  
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
    // pub(joint_rad);
    // value_reset();
}
    
  
  
}

float dog_robot_class::senkai_culculate(std::vector<float> transform){//旋回時のxyz座標を計算
  float x=transform[0]*cos(senkai_theta)-transform[1]*sin(senkai_theta);//z軸を回転軸にして回転
  float y=transform[0]*sin(senkai_theta)+transform[1]*cos(senkai_theta);//z軸を回転軸にして回転
  float z=transform[2];//z軸を回転軸にして回転
  // std::cout<<x<<endl;
  
  float next_distance=sqrt(pow((transform[0]-x),2)+pow((transform[1]-y),2));//次に脚を置く位置と現在の距離
  
  return next_distance;
}

void dog_robot_class::senkai(float ang_x,float vel_z){//旋回時に使用
  //左回転>0、右回転<0

  //右回りが初期値
  float fl_v=-1.0f*senkai_theta;
  float fr_v=senkai_theta-p;
  float bl_v=senkai_theta;
  float br_v=p-senkai_theta;

  car_speed=1.0f;//算出
  // car_speed=1.84f;
  std::cout<<"car_speed::::"<<car_speed<<endl;
  car_right_speed=-1.0f*car_speed;
  car_left_speed=-1.0f*car_speed;

  if(ang_x>0){//左回りなら
    fl_v=p-senkai_theta;
    fr_v=senkai_theta;
    bl_v=senkai_theta-p;
    br_v=-1.0f*senkai_theta;
    car_right_speed=car_speed;
    car_left_speed=car_speed;
  }
  float fl_a=senkai_culculate(fl_start);
  float fr_a=senkai_culculate(fr_start);
  float bl_a=senkai_culculate(bl_start);
  float br_a=senkai_culculate(br_start);

//歩幅と、傾きの決定

if(step_num==8){//普通の楕円、pull_push
  float origin_f=-1.0f*origin_culculate;
  float add_front=0.05f;//前から接地するために使用
  if(ang_x>0){
    float origin_f=1.0f*origin_culculate;
  }
  float origin_b=origin_culculate;
  float a=fl_a;
  float thetaxy=fl_v;
  // float origin_fl=-1.0f*origin_culculate;
  std::vector<std::vector<float>> fl_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),b},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
  a=fr_a;
  thetaxy=fr_v;
  std::vector<std::vector<float>> fr_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),b},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
  a=bl_a;
  thetaxy=bl_v;
  std::vector<std::vector<float>> bl_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*1.0f/sqrt(2.0f)},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),b_b},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*1.0f/sqrt(2.0f)},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};
  a=br_a;
  thetaxy=br_v;
  std::vector<std::vector<float>> br_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*1.0f/sqrt(2.0f)},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),b_b},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*1.0f/sqrt(2.0f)},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};

  
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(fl_orb,now_height_fl,fl_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(fr_orb,now_height_fr,fr_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(bl_orb,now_height_bl,bl_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(br_orb,now_height_br,br_a,0.0f);

  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,-1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,-1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
  // std::cout<<"bl_theta_list2"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<bl_theta_list[3*i+1]<<endl;
  // }
  // std::cout<<"bl_theta_list3"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<bl_theta_list[3*i+2]<<endl;
  // }
  orbit_list();
  pull_push_pub();
}

if(step_num==10){//地面蹴るモーションがある軌道、kick_ground_pub
  float origin_f=-1.0f*origin_culculate;
  float add_front=0.05f;//前から接地するために使用
  if(ang_x>0){
    float origin_f=1.0f*origin_culculate;
  }
  float origin_b=origin_culculate;
  float a=fl_a;
  float thetaxy=fl_v;
  std::vector<std::vector<float>> fl_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b_under},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),b*sin(p/4.0f)},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*sin(p/8.0f)},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b_under},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0}};
   a=fr_a;
  thetaxy=fr_v;
  std::vector<std::vector<float>> fr_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b_under},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),b*sin(p/4.0f)},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*sin(p/8.0f)},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b_under},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0}};
    a=bl_a;
  thetaxy=bl_v;
  std::vector<std::vector<float>> bl_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_under},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*sin(p/8.0f)},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),b_b*sin(p/4.0f)},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_under},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0}};
  a=br_a;
  thetaxy=br_v;
  std::vector<std::vector<float>> br_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_under},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*sin(p/8.0f)},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),b_b*sin(p/4.0f)},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_under},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0}};
  
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(fl_orb,now_height_fl,fl_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(fr_orb,now_height_fr,fr_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(bl_orb,now_height_bl,bl_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(br_orb,now_height_br,br_a,0.0f);

  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
  // std::cout<<"bl_theta_list2"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<bl_theta_list[3*i+1]<<endl;
  // }
  // std::cout<<"bl_theta_list3"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<bl_theta_list[3*i+2]<<endl;
  // }
  orbit_list();
  kick_ground_pub();
}


if(step_num==4){//地面蹴るモーションがある軌道、kick_ground_pub
  float origin_f=-1.0f*origin_culculate;
  float add_front=0.05f;//前から接地するために使用
  if(ang_x>0){
    float origin_f=1.0f*origin_culculate;
  }
  float origin_b=origin_culculate;
  float a=fl_a;
  float thetaxy=fl_v;
  std::vector<std::vector<float>> fl_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f*a+origin_f)*cos(thetaxy),(1.0f*a+origin_f)*sin(thetaxy),0},{(0.0f*a+origin_f)*cos(thetaxy),(0.0f*a+origin_f)*sin(thetaxy),b},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0}};
  a=fr_a;
  thetaxy=fr_v;
  std::vector<std::vector<float>> fr_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f*a+origin_f)*cos(thetaxy),(1.0f*a+origin_f)*sin(thetaxy),0},{(0.0f*a+origin_f)*cos(thetaxy),(0.0f*a+origin_f)*sin(thetaxy),b},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0}};
  a=bl_a;
  thetaxy=bl_v;
  std::vector<std::vector<float>> bl_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(0.0f*a+origin_b)*cos(thetaxy),(0.0f*a+origin_b)*sin(thetaxy),b_b},{(1.0f*a+origin_b)*cos(thetaxy),(1.0f*a+origin_b)*sin(thetaxy),0}};
  a=br_a;
  thetaxy=br_v;
  std::vector<std::vector<float>> br_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(0.0f*a+origin_b)*cos(thetaxy),(0.0f*a+origin_b)*sin(thetaxy),b_b},{(1.0f*a+origin_b)*cos(thetaxy),(1.0f*a+origin_b)*sin(thetaxy),0}};
  
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(fl_orb,now_height_fl,fl_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(fr_orb,now_height_fr,fr_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(bl_orb,now_height_bl,bl_a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(br_orb,now_height_br,br_a,0.0f);

  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
  // std::cout<<"bl_theta_list2"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<bl_theta_list[3*i+1]<<endl;
  // }
  // std::cout<<"bl_theta_list3"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<bl_theta_list[3*i+2]<<endl;
  // }
  orbit_list();
  triangle_pub();
}

if(step_num==7){
  float origin_fl=1.0f*origin_culculate;
  float origin_bl=1.0f*origin_culculate;
  float origin_fr=-1.0f*origin_culculate;
  float origin_br=-1.0f*origin_culculate;
  float add_front=0.05f;//前から接地するために使用
  if(ang_x>0){
    origin_fl=-1.0f*origin_culculate;
    origin_bl=-1.0f*origin_culculate;
    origin_fr=1.0f*origin_culculate;
    origin_br=1.0f*origin_culculate;
  }

  float a=fl_a;
  float thetaxy=fl_v;
//初期姿勢からスタートさせるのに必要、excelより求めるしかないかも
  //軌道生成、脚を上げる量を変えたいので、前と後ろどっちも作成している
  std::vector<std::vector<float>> fl_orb={{origin_fl*cos(thetaxy),origin_fl*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_fl)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_fl)*sin(thetaxy),0},{(a+origin_fl)*cos(thetaxy),(a+origin_fl)*sin(thetaxy),0},{(origin_fl+a*cos(135/180.0f*p))*cos(thetaxy),(origin_fl+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_fl-add_front)*cos(thetaxy),(-1.0f*a+origin_fl-add_front)*sin(thetaxy),b/2.0f},{(-1.0f*a+origin_fl)*cos(thetaxy),(-1.0f*a+origin_fl)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_fl)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_fl)*sin(thetaxy),0}};
  a=fr_a;
  thetaxy=fr_v;
  std::vector<std::vector<float>> fr_orb={{origin_fr*cos(thetaxy),origin_fr*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_fr)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_fr)*sin(thetaxy),0},{(a+origin_fr)*cos(thetaxy),(a+origin_fr)*sin(thetaxy),0},{(origin_fr+a*cos(135/180.0f*p))*cos(thetaxy),(origin_fr+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_fr-add_front)*cos(thetaxy),(-1.0f*a+origin_fr-add_front)*sin(thetaxy),b/2.0f},{(-1.0f*a+origin_fr)*cos(thetaxy),(-1.0f*a+origin_fr)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_fr)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_fr)*sin(thetaxy),0}};
  //後ろの軌道
  a=bl_a;
  thetaxy=bl_v;
  std::vector<std::vector<float>> bl_orb={{origin_bl*cos(thetaxy),origin_bl*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_bl)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_bl)*sin(thetaxy),0},{(-1.0f*a+origin_bl)*cos(thetaxy),(-1.0f*a+origin_bl)*sin(thetaxy),0},{(origin_bl+a*cos(45/180.0f*p))*cos(thetaxy),(origin_bl+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_bl+add_front)*cos(thetaxy),(a+origin_bl+add_front)*sin(thetaxy),b_b/2.0f},{(a+origin_bl)*cos(thetaxy),(a+origin_bl)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_bl)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_bl)*sin(thetaxy),0}};
  a=br_a;
  thetaxy=br_v;
  std::vector<std::vector<float>> br_orb={{origin_br*cos(thetaxy),origin_br*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_br)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_br)*sin(thetaxy),0},{(-1.0f*a+origin_br)*cos(thetaxy),(-1.0f*a+origin_br)*sin(thetaxy),0},{(origin_br+a*cos(45/180.0f*p))*cos(thetaxy),(origin_br+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_br+add_front)*cos(thetaxy),(a+origin_br+add_front)*sin(thetaxy),b_b/2.0f},{(a+origin_br)*cos(thetaxy),(a+origin_br)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_br)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_br)*sin(thetaxy),0}};

//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(fl_orb,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(fr_orb,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(bl_orb,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(br_orb,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);

  orbit_list();

  if(crawl_mode){
    // crawl_walk_pub_7();//press_yes,noあり
    // crawl_walk_pub_8();//pressなし
    crawl_walk_pub_9();//press_noあり
  }
  else{
    add_front_pub();
  }

}

if(step_num==6){
  float origin_f=-1.0f*origin_culculate;
  float add_front=0.05f;//前から接地するために使用
  if(ang_x>0){
    float origin_f=1.0f*origin_culculate;
  }
  float origin_b=origin_culculate;
  float a=fl_a;
  float thetaxy=fl_v;
//初期姿勢からスタートさせるのに必要、excelより求めるしかないかも
  // float from=0.05f;
  //軌道生成、脚を上げる量を変えたいので、前と後ろどっちも作成している
  std::vector<std::vector<float>> fl_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
  a=fr_a;
  thetaxy=fr_v;
  std::vector<std::vector<float>> fr_orb={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
 //後ろの軌道
  a=bl_a;
  thetaxy=bl_v;
  std::vector<std::vector<float>> bl_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};
  a=br_a;
  thetaxy=br_v;
  std::vector<std::vector<float>> br_orb={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};

//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(fl_orb,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(fr_orb,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(bl_orb,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(br_orb,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);

  orbit_list();
  if(crawl_mode){
    crawl_walk_pub_2();
  }
  else{
    one_point_pub();
  }
  
}

  vector<float>().swap(fl_theta_list);
  vector<float>().swap(fr_theta_list);
  vector<float>().swap(bl_theta_list);
  vector<float>().swap(br_theta_list);

  value_reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
}

float dog_robot_class::inverce_kinetic_theta1(float y,float diff_y,float now){//theta1の逆運動学
    float theta1=atan2(y-diff_y,now);

    return theta1;
}
float dog_robot_class::inverce_kinetic_theta2(float x,float y,float z,float diff_x,float diff_y,float diff_z){//theta2の逆運動学
    float v31=sqrt(pow(x-diff_x,2)+pow(y-diff_y,2))+0.00001f;//原点の分だけずらす
    float z31=z-diff_z;
    float theta2=-1*acos((pow(v31,2)+pow(z31,2)+pow(L2,2)-pow(L3,2))/(2.0f*L2*sqrt(pow(v31,2)+pow(z31,2))))+atan2(z31,v31);
    count++;
    return theta2;
}
float dog_robot_class::inverce_kinetic_theta3(float x,float y,float z,float theta2,float diff_x,float diff_y,float diff_z){//theta3の逆運動学
    float v31=sqrt(pow(x-diff_x,2)+pow(y-diff_y,2))+0.00001f;//原点の分だけずらす
    float z31=z-diff_z;
    float theta3=atan2((z31-L2*sin(theta2)),(v31-L2*cos(theta2)))-theta2;
    
    return theta3;
}


void dog_robot_class::orbit_list(){//軌道のtheta値を入れていく

  for(int i=0;i<step_num;i++){          
    fl1[i]=fl_theta_list[i*3];
    // std::cout<<fl_theta_list[i*3+1]<<endl;
    fl2[i]=fl_theta_list[i*3+1];
    fl3[i]=fl_theta_list[i*3+2];

    fr1[i]=fr_theta_list[i*3];
    fr2[i]=fr_theta_list[i*3+1];
    fr3[i]=fr_theta_list[i*3+2];

    bl1[i]=bl_theta_list[i*3];
    bl2[i]=bl_theta_list[i*3+1];
    bl3[i]=bl_theta_list[i*3+2];

    br1[i]=br_theta_list[i*3];
    br2[i]=br_theta_list[i*3+1];
    br3[i]=br_theta_list[i*3+2];
    }

}

void dog_robot_class::start_position(){//初期位置にする、hファイルで変更可能
//付け根の座標、接地点の座標の初期化
    // height=L2*sin(height_theta2)+L3*sin(height_theta2-height_theta3);
    height=L2*cos(start_theta2)+L3*cos(-1.0f*start_theta2-1.0f*start_theta3);//初期height値の計算
    origin_culculate=L2*sin(-1.0f*start_theta2)-L3*sin(start_theta2+start_theta3);
    max_speed=(sqrt(pow((L2+L3),2)-pow(height,2))+origin_culculate)*2.0f;
    if(max_speed>0.18f){
      max_speed=0.18f;
    }
    // std::cout<<L2*cos(start_theta2)<<"::::::"<<L3*cos(-1.0f*start_theta2-1.0f*start_theta3)
    // <<endl;
    // std::cout<<"origin::"<<L2*sin(-1.0f*start_theta2)-L3*sin(start_theta2+start_theta3)
    // <<endl;
    std::cout<<"高さ::"<<height<<endl;
    std::cout<<max_speed<<endl;
    //各々でheight値を初期化
    now_height_fl=height;
    now_height_fr=height;
    now_height_bl=height;
    now_height_br=height;
    body_height=0.0f;
    // if(gp_center){//
    //   for(int i=0;i<press_controll;i++){
    //     controll_gravity();
    //   }
    //   gp_center=false;
    // }

    //値を初期化、傾きながら歩く値
    if(crawlr_mode){//クローラモードから4footに戻る
      for(int i=10;i>=0;i--){
      joint_rad.position[0]=change_mode[3*i];
      joint_rad.position[1]=-1.0f*change_mode[3*i+1];
      joint_rad.position[2]=change_mode[3*i+2];
      joint_rad.position[3]=-1.0f*change_mode[3*i];
      joint_rad.position[4]=change_mode[3*i+1];
      joint_rad.position[5]=-1.0f*change_mode[3*i+2];
      // pub(joint_rad);
      // std::this_thread::sleep_for(std::chrono::seconds(1));//前、後ろの順で立つようにするため。
      joint_rad.position[6]=-1.0f*change_mode[3*i];
      joint_rad.position[7]=change_mode[3*i+1];
      joint_rad.position[8]=-1.0f*change_mode[3*i+2];
      joint_rad.position[9]=change_mode[3*i];
      joint_rad.position[10]=-1.0f*change_mode[3*i+1];
      joint_rad.position[11]=change_mode[3*i+2];
      joint_rad.velocity[12]=change_mode_velocity[i]*-1.0f;
      joint_rad.velocity[13]=change_mode_velocity[i];
      joint_rad.velocity[14]=change_mode_velocity[i];
      joint_rad.velocity[15]=change_mode_velocity[i]*-1.0f;
      pub(joint_rad);
  }
    }
    else{//4footの初期位置
      joint_rad.position[0]=start_theta1+always_theta1;
      joint_rad.position[1]=-1.0f*start_theta2;
      joint_rad.position[2]=start_theta3;
      joint_rad.position[3]=-1.0f*start_theta1-always_theta1;
      joint_rad.position[4]=start_theta2;
      joint_rad.position[5]=-1.0f*start_theta3;
      joint_rad.position[6]=-1.0f*start_theta1-always_theta1;
      joint_rad.position[7]=start_theta2;
      joint_rad.position[8]=-1.0f*start_theta3;
      joint_rad.position[9]=start_theta1+always_theta1;
      joint_rad.position[10]=-1.0f*start_theta2;
      joint_rad.position[11]=start_theta3;
      joint_rad.position[12]=0.0f;
      joint_rad.position[13]=-20/180.0f*p;
      pub(joint_rad);
    }
    if(arm_mode){
      arm_x=height;
      arm_y=0;
      arm_z=0;
    }
    
}

void dog_robot_class::value_reset(){//pubするthetaのリスト、回転かつに使う際の傾き値をリセットする
  //配列の行列を指定して、値をいれているため、ここで初期値にリセットする
  for(int i=0;i<step_num;i++){
      fl1[i]=start_theta1;
      fl2[i]=-1.0f*start_theta2;
      fl3[i]=start_theta3;
      fr1[i]=-1.0f*start_theta1;
      fr2[i]=start_theta2;
      fr3[i]=-1.0f*start_theta3;
      bl1[i]=-1.0f*start_theta1;
      bl2[i]=start_theta2;
      bl3[i]=-1.0f*start_theta3;
      br1[i]=start_theta1;
      br2[i]=-1.0f*start_theta2;
      br3[i]=start_theta3;
    }
    //回転かつで使うときに使用、毎回リセットさせる
    fl_r={start_theta1,-1.0f*start_theta2-p/2.0f,-1.0f*start_theta3,0};
    fr_r={-1.0f*start_theta1,-1.0f*start_theta2-p/2.0f,-1.0f*start_theta3,0};
    bl_r={-1.0f*start_theta1,-1.0f*start_theta2-p/2.0f,-1.0f*start_theta3,0};
    br_r={start_theta1,-1.0f*start_theta2-p/2.0f,-1.0f*start_theta3,0};
    cross_height=0.0f;
}
void dog_robot_class::kick_value_reset(){//pubするthetaのリスト、回転かつに使う際の傾き値をリセットする
  //配列の行列を指定して、値をいれているため、ここで初期値にリセットする
  for(int i=0;i<step_num;i++){
      fl1[i]=kcf_st_th1;
      fl2[i]=-1.0f*kcf_st_th2;
      fl3[i]=kcf_st_th3;
      fr1[i]=-1.0f*kcf_st_th1;
      fr2[i]=kcf_st_th2;
      fr3[i]=-1.0f*kcf_st_th3;
      bl1[i]=-1.0f*kcb_st_th1;
      bl2[i]=kcb_st_th2;
      bl3[i]=-1.0f*kcb_st_th3;
      br1[i]=kcb_st_th1;
      br2[i]=-1.0f*kcb_st_th2;
      br3[i]=kcb_st_th3;
    }
    //回転かつで使うときに使用、毎回リセットさせる
    fl_r={kcf_st_th1,-1.0f*kcf_st_th2-p/2.0f,-1.0f*kcf_st_th3,0};
    fr_r={-1.0f*kcf_st_th1,-1.0f*kcf_st_th2-p/2.0f,-1.0f*kcf_st_th3,0};
    bl_r={-1.0f*kcb_st_th1,-1.0f*kcb_st_th2-p/2.0f,-1.0f*kcb_st_th3,0};
    br_r={kcb_st_th1,-1.0f*kcb_st_th2-p/2.0f,-1.0f*kcb_st_th3,0};
}


void dog_robot_class::spot_orb_pub(){//spotの軌道をpubする関数、pubの順番はネットで見たものを再現するようにしている
  
  if(move_dir<0){//前に進む
    // std::cout<<"正"<<endl;
    for(int i=9;i>=4;i--){
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i-4]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i-4]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i-4]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i-4]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i-4]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i-4]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      // std::this_thread::sleep_for(std::chrono::milliseconds(5));
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    for(int i=3;i>=0;i--){
      // std::cout<<fl1[i]i<<std::endl;

      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;

      joint_rad.position[3] =fr1[i+6]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i+6]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i+6]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i+6]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i+6]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i+6]+bl_r[2]+1.0f*start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));//5ミリ秒止める  
    }
  }
  else if(move_dir>0){//後ろに進む
    for(int i=0;i<4;i++){
      // std::cout<<fl1[i]i<<std::endl;
      // std::cout<<"負"<<endl;
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;

      joint_rad.position[3] =fr1[i+6]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i+6]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i+6]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i+6]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i+6]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i+6]+bl_r[2]+1.0f*start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));//5ミリ秒止める  
    }
    for(int i=4;i<10;i++){
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i-4]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i-4]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i-4]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i-4]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i-4]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i-4]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
  }

}

void dog_robot_class::one_leg_pub(){//片足ずつ動かすのに使用//0919確認してない
  for(int i=0;i<=4;i++){//fl 0,1,2,3,4 bl 4,5,6,7,8
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i+4]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i+4]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i+4]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i+4]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i+4]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i+4]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
    }
  for(int i=1;i<=4;i++){//fl 5,6,7,8 bl 1,2,3,4
      joint_rad.position[0] =fl1[i+4]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i+4]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i+4]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i+4]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i+4]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i+4]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
    }

}

void dog_robot_class::kick_ground_pub(){//蹴る動きを追加した軌道をpubする
  
  for(int i=0;i<=3;i++){//fl 0,1,2,3 bl 4,5,6,7
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i+4]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i+4]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i+4]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i+4]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i+4]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i+4]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
      // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  for(int i=0;i<=3;i++){//fl 4,5,6,7 bl 0,1,2,3
      joint_rad.position[0] =fl1[i+4]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i+4]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i+4]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i+4]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i+4]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i+4]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
      // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    for(int i=8;i<10;i++){//fl 8,9 bl 8,9
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);

      std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
    }
}

void dog_robot_class::triangle_pub(){
  for(int i=0;i<2;i++){
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i+2]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i+2]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i+2]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i+2]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i+2]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i+4]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
   for(int i=0;i<2;i++){
      joint_rad.position[0] =fl1[i+2]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i+2]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i+2]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i+4]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i+2]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i+2]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i+2]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
}

void dog_robot_class::pull_push_pub(){//歩けるの確認済み
  for(int i=0;i<=3;i++){//fl 0,1,2,3 bl 4,5,6,7
      joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i+4]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i+4]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i+4]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i+4]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i+4]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i+4]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
      // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  for(int i=0;i<=3;i++){//fl 4,5,6,7 bl 0,1,2,3
      joint_rad.position[0] =fl1[i+4]+fl_r[0]-start_theta1;
      joint_rad.position[1] =fl2[i+4]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[2] =-1.0f*fl3[i+4]-1.0f*fl_r[2]-start_theta3;
      joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;
      joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
      joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
      joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
      joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
      joint_rad.position[9] =-1.0f*br1[i+4]-1.0f*br_r[0]-start_theta1;
      joint_rad.position[10]=br2[i+4]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
      joint_rad.position[11]=-1.0f*br3[i+4]-1.0f*br_r[2]-start_theta3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
      // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void dog_robot_class::one_point_pub(){
  for(int i=0;i<3;i++){//fl 012 bl 345
    joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[i+3]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[i+3]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i+3]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[i+3]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[i+3]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i+3]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  }
  for(int i=0;i<3;i++){//fl 345 bl 012
    joint_rad.position[0] =fl1[i+3]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[i+3]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i+3]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =-1.0f*br1[i+3]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[i+3]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i+3]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  }
}

void dog_robot_class::add_front_pub(){
  //fl 0 bl 3
  joint_rad.position[0] =fl1[0]+always_theta1;
  joint_rad.position[1] =fl2[0];
  joint_rad.position[2] =-1.0f*fl3[0];
  joint_rad.position[3] =fr1[3]-always_theta1;
  joint_rad.position[4] =-1.0f*fr2[3];
  joint_rad.position[5] =fr3[3];
  joint_rad.position[6] =-1.0f*bl1[3]-always_theta1;
  joint_rad.position[7] =-1.0f*bl2[3];
  joint_rad.position[8] =bl3[3];
  joint_rad.position[9] =-1.0f*br1[0]+always_theta1;
  joint_rad.position[10]=br2[0];
  joint_rad.position[11]=-1.0f*br3[0];
  // joint_rad.position[16]=crawl_walk_pub;//左
  // joint_rad.position[17]=crawl_walk_pub;//右
  joint_rad.velocity[15]=car_left_speed;//左
  joint_rad.velocity[14]=car_right_speed;//右
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  for(int i=0;i<3;i++){//fl 0 012 bl 3 456
    joint_rad.position[0] =fl1[i]+always_theta1;
    joint_rad.position[1] =fl2[i];
    joint_rad.position[2] =-1.0f*fl3[i];
    joint_rad.position[3] =fr1[i+4]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[i+4];
    joint_rad.position[5] =fr3[i+4];
    joint_rad.position[6] =-1.0f*bl1[i+4]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[i+4];
    joint_rad.position[8] =bl3[i+4];
    joint_rad.position[9] =-1.0f*br1[i]+always_theta1;
    joint_rad.position[10]=br2[i];
    joint_rad.position[11]=-1.0f*br3[i];
    // joint_rad.position[12]=crawl_walk_pub;
    // joint_rad.position[12]=crawl_walk_pub;
    joint_rad.velocity[15]=car_left_speed;
    joint_rad.velocity[14]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  }
  //fl 3 bl 0
  joint_rad.position[0] =fl1[3]+always_theta1;
  joint_rad.position[1] =fl2[3];
  joint_rad.position[2] =-1.0f*fl3[3];
  joint_rad.position[3] =fr1[0]-always_theta1;
  joint_rad.position[4] =-1.0f*fr2[0];
  joint_rad.position[5] =fr3[0];
  joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
  joint_rad.position[7] =-1.0f*bl2[0];
  joint_rad.position[8] =bl3[0];
  joint_rad.position[9] =-1.0f*br1[3]+always_theta1;
  joint_rad.position[10]=br2[3];
  joint_rad.position[11]=-1.0f*br3[3];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  joint_rad.velocity[15]=car_left_speed;
  joint_rad.velocity[14]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  for(int i=0;i<3;i++){//fl 3 456 bl 0 012
    joint_rad.position[0] =fl1[i+4]+always_theta1;
    joint_rad.position[1] =fl2[i+4];
    joint_rad.position[2] =-1.0f*fl3[i+4];
    joint_rad.position[3] =fr1[i]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[i];
    joint_rad.position[5] =fr3[i];
    joint_rad.position[6] =-1.0f*bl1[i]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[i];
    joint_rad.position[8] =bl3[i];
    joint_rad.position[9] =-1.0f*br1[i+4]+always_theta1;
    joint_rad.position[10]=br2[i+4];
    joint_rad.position[11]=-1.0f*br3[i+4];
    // joint_rad.position[12]=crawl_walk_pub;
    // joint_rad.position[12]=crawl_walk_pub;
    joint_rad.velocity[15]=car_left_speed;
    joint_rad.velocity[14]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  }
}

void dog_robot_class::one_side_pub(){//左上げると、右上げる、左つける
  //fl 00123456 br 34560012
  //fl 0 br 3
  joint_rad.position[0] =fl1[0]+always_theta1;
  joint_rad.position[1] =fl2[0];
  joint_rad.position[2] =-1.0f*fl3[0];
  joint_rad.position[3] =fr1[3]-always_theta1;
  joint_rad.position[4] =-1.0f*fr2[3];
  joint_rad.position[5] =fr3[3];
  joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
  joint_rad.position[7] =-1.0f*bl2[0];
  joint_rad.position[8] =bl3[0];
  joint_rad.position[9] =-1.0f*br1[3]+always_theta1;
  joint_rad.position[10]=br2[3];
  joint_rad.position[11]=-1.0f*br3[3];
  joint_rad.position[12]=-1.0f*side_leg_add/2.0f;//左
  joint_rad.position[13]=one_side_under;//右
  angle_pub.publish(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  for(int i=0;i<3;i++){//fl 012 br 456
    joint_rad.position[0] =fl1[i]+always_theta1;
    joint_rad.position[1] =fl2[i];
    joint_rad.position[2] =-1.0f*fl3[i];
    joint_rad.position[3] =fr1[i+4]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[i+4];
    joint_rad.position[5] =fr3[i+4];
    joint_rad.position[6] =-1.0f*bl1[i]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[i];
    joint_rad.position[8] =bl3[i];
    joint_rad.position[9] =-1.0f*br1[i+4]+always_theta1;
    joint_rad.position[10]=br2[i+4];
    joint_rad.position[11]=-1.0f*br3[i+4];
    joint_rad.position[12]=0.0f;//左
    joint_rad.position[13]=0.0f;//右
    if(i==0){
      joint_rad.position[12]=-1.0f*side_leg_add/2.0f;//左
      joint_rad.position[13]=one_side_under;//右
    }
    angle_pub.publish(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  }
  //fl 3 br 0
  joint_rad.position[0] =fl1[3]+always_theta1;
  joint_rad.position[1] =fl2[3];
  joint_rad.position[2] =-1.0f*fl3[3];
  joint_rad.position[3] =fr1[0]-always_theta1;
  joint_rad.position[4] =-1.0f*fr2[0];
  joint_rad.position[5] =fr3[0];
  joint_rad.position[6] =-1.0f*bl1[3]-always_theta1;
  joint_rad.position[7] =-1.0f*bl2[3];
  joint_rad.position[8] =bl3[3];
  joint_rad.position[9] =-1.0f*br1[0]+always_theta1;
  joint_rad.position[10]=br2[0];
  joint_rad.position[11]=-1.0f*br3[0];
  joint_rad.position[12]=one_side_under;//左
  joint_rad.position[13]=1.0f*side_leg_add/2.0f;//右
  angle_pub.publish(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  //fl 456 br 012
  for(int i=0;i<3;i++){
    joint_rad.position[0] =fl1[i+4]+always_theta1;
    joint_rad.position[1] =fl2[i+4];
    joint_rad.position[2] =-1.0f*fl3[i+4];
    joint_rad.position[3] =fr1[i]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[i];
    joint_rad.position[5] =fr3[i];
    joint_rad.position[6] =-1.0f*bl1[i+4]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[i+4];
    joint_rad.position[8] =bl3[i+4];
    joint_rad.position[9] =-1.0f*br1[i]+always_theta1;
    joint_rad.position[10]=br2[i];
    joint_rad.position[11]=-1.0f*br3[i];
    joint_rad.position[12]=0.0f;//左
    joint_rad.position[13]=0.0f;//右
    if(i==0){
      joint_rad.position[12]=one_side_under;//左
      joint_rad.position[13]=1.0f*side_leg_add/2.0f;//右
    }
    angle_pub.publish(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  }
}

std::vector<std::vector<float>> dog_robot_class::each_make_orb(std::vector<std::vector<float>> orbit_magnitude,float each_h,float a,float diff){//diffは0 or 1で、spot_orbなら1
//xyz値を回転させて、逆運動学を考えやすくするのに使用

  std::vector<std::vector<float>> return_orb(step_num,vector<float>(3));
  for(int i=0;i<step_num;i++){
    return_orb.at(i).at(0)=-1*orbit_magnitude.at(i).at(2)+each_h;
    return_orb.at(i).at(1)=orbit_magnitude.at(i).at(1);
    return_orb.at(i).at(2)=orbit_magnitude.at(i).at(0)-diff*move_dir*a*2.0f/10.0f*i;
  }

  return return_orb;
}

std::vector<float> dog_robot_class::make_theta(std::vector<std::vector<float>> orb_mag_rotate,float each_h,float judge){//逆運動学で求めた値をvectorで返す
  //judge=1は前足、judge=-1は後ろ足
  std::vector<float> theta_list {};
  for (int i=0;i<step_num;i++){
    // std::cout<<"give_x::"<<orb_mag_rotate[i][0]<<",give_y::"<<orb_mag_rotate[i][1]<<",give_z::"<<orb_mag_rotate[i][2]<<endl;
    float theta1=inverce_kinetic_theta1(orb_mag_rotate.at(i).at(1),0.0f,each_h);
    float theta2=inverce_kinetic_theta2(orb_mag_rotate.at(i).at(0),orb_mag_rotate.at(i).at(1),judge*orb_mag_rotate.at(i).at(2),0.0f,0.0f,0.0f);
    float theta3=inverce_kinetic_theta3(orb_mag_rotate.at(i).at(0),orb_mag_rotate.at(i).at(1),judge*orb_mag_rotate.at(i).at(2),theta2,0.0f,0.0f,0.0f);
    theta_list.insert(theta_list.end(),{theta1,theta2,theta3}); //iはtheta1,2*iはtheta2,3*iはtheta3
    // if(i==0){
    //   std::cout<<"theta2::"<<theta2*180/p<<endl;
    //   std::cout<<"theta3::"<<theta3*180/p<<endl;
    // }
    }
    return theta_list;

}

void dog_robot_class::make_orbit(float vel_x,float vel_y,float ang_x,float vel_z){//軌道作成
    float speed = max_speed*sqrt(pow(vel_x,2)+pow(vel_y,2));//歩幅、倒す量0~1.0//max_speedで変更可能
    float a=speed/2;//進行速度を長軸の半分に//-1<=speed<=1//進む距離
    float dir=-1.0f*atan2(vel_x,vel_y);//進む角度
    // car_speed=speed/(gear_r*2*p)/(time/1000.0f*step_num)*60.0f;//回転数(進む距離/タイヤの円周)/時間(sec)
    // std::cout<<"car_speed::::"<<car_speed<<endl;
    car_speed=10.0f*speed;
    if(crawl_mode){//クロール歩行ならタイヤの速度は1/2にする
      car_speed=car_speed/2.0f;
    }
    // std::cout<<"car_speed::::"<<car_speed<<endl;
    car_right_speed=-1.0f*car_speed;//初期値は前進
    car_left_speed=1.0f*car_speed;//
    b=0.05f;//前足の上げる量
    b_b=0.05f;//後ろ脚の上げる量
    if(R1_switch){//押してる間脚上げる量変える。
      b=0.1f;
    }
    else if(R2_switch){
      b=0.2f;
    }
    if(L1_switch){
      b_b=0.1f;
    }
    else if(L2_switch){
      b_b=0.15f;
    }
    // std::cout<<dir*180.0f/p<<endl;
    if(vel_x==0&&vel_y==0){//atan2値が発散しないように使用
      dir=0.0f;
    }

    // std::cout<<"前足の上げる量：："<<b<<endl;
    // std::cout<<"後ろの上げる量：："<<b_b<<endl;

    float thetaxy=-1.0f*dir;//joyをどの角度に傾けるかを用いる、それが進行方向に
    if(thetaxy<p/2&&thetaxy>-1.0f*p/2){
      car_right_speed=1.0f*car_speed;
      car_left_speed=-1.0f*car_speed;
    }
    float origin_f=a;//原点を足の接地点とするため、楕円は長軸の半分だけずれる
    float origin_b=a;
  // std::cout<<"高さ：："<<height<<"傾き：："<<thetaxy<<"歩幅：："<<a<<"::"<<-1.0f*a+origin_<<endl;
    if (std::abs(ang_x)<p/2.0f){//体がどっちに動いているかで、原点をずらすのに必要
      move_dir=1.0f;
    }
    else{
      move_dir=-1.0f;
    }   

  //理想的な足の配置、インデックス0から順番に７点を指定し、それをたどるようにする、

     
    
//step_numが8のときspeedは0.18がmax
if(step_num==9){
  //軌道生成
  std::vector<std::vector<float>> orbit_magnitude={{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{a*cos(thetaxy),a*sin(thetaxy),b},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{a*cos(thetaxy),a*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,-1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,-1.0f);

  std::cout<<"歩幅::"<<a<<"::角度::"<<thetaxy<<endl;
}

if(step_num==8){

  origin_f=-1.0f*origin_culculate;
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_culculate;
//初期姿勢からスタートさせるのに必要、excelより求めるしかないかも

  // float from=0.05f;
  //軌道生成、脚を上げる量を変えたいので、前と後ろどっちも作成している
  // std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),b},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*1.0f/sqrt(2.0f)},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),b_b},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*1.0f/sqrt(2.0f)},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};
  std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),b},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
}

if(step_num==10){//地面蹴る//上とコメントは同様
  origin_f=-1.0f*origin_culculate;
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_f;

  // float from=0.05f;
  std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b_under},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),b*sin(p/4.0f)},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b*sin(p/8.0f)},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),b_under},{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0}};
  
  std::vector<std::vector<float>> orbit_magnitude_b={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_under},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b*sin(p/8.0f)},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),b_b*sin(p/4.0f)},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_b},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),b_under},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0}};

  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);

  // std::cout<<"x"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_bl[i][0]<<endl;
  // }
  // std::cout<<"z"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_bl[i][2]<<endl;
  // }

  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);

}

if(step_num==4){
  origin_f=-1.0f*origin_culculate;
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_f;
//初期姿勢からスタートさせるのに必要、excelより求めるしかないかも
  // float from=0.05f;
  //軌道生成、脚を上げる量を変えたいので、前と後ろどっちも作成している
  std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f*a+origin_f)*cos(thetaxy),(1.0f*a+origin_f)*sin(thetaxy),0},{(0.0f*a+origin_f)*cos(thetaxy),(0.0f*a+origin_f)*sin(thetaxy),b},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0}};
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(0.0f*a+origin_b)*cos(thetaxy),(0.0f*a+origin_b)*sin(thetaxy),b_b},{(1.0f*a+origin_b)*cos(thetaxy),(1.0f*a+origin_b)*sin(thetaxy),0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
}

if(step_num==6){
  origin_f=-1.0f*origin_culculate;
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_f;
//初期姿勢からスタートさせるのに必要、excelより求めるしかないかも

  // float from=0.05f;
  //軌道生成、脚を上げる量を変えたいので、前と後ろどっちも作成している
  std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),0},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
}
if(step_num==7){
  origin_f=-1.0f*origin_culculate;
  float add_front=0.05f;//前から接地するために使用
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_f;
  if(crawl_mode){
    b_under=0.02f;
  }
//初期姿勢からスタートさせるのに必要、excelより求めるしかないかも

  // float from=0.05f;
  //軌道生成、脚を上げる量を変えたいので、前と後ろどっちも作成している
  std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_f-add_front)*cos(thetaxy),(-1.0f*a+origin_f-add_front)*sin(thetaxy),b/2.0f},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),b_under},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),b_under},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_b+add_front)*cos(thetaxy),(a+origin_b+add_front)*sin(thetaxy),b_b/2.0f},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);
  // std::cout<<"flx"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_fl[i][0]<<endl;
  // }
  // std::cout<<"fly"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_bl[i][1]<<endl;
  // }
  // std::cout<<"flz"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_bl[i][2]<<endl;
  // }
  // std::cout<<"blx"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_bl[i][0]<<endl;
  // }
  // std::cout<<"bly"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_bl[i][1]<<endl;
  // }
  // std::cout<<"blz"<<endl;
  // for(int i=0;i<step_num;i++){
  //   std::cout<<orb_rotate_bl[i][2]<<endl;
  // }
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
}
    orbit_list();//fl1,fl2,fl3などに値を入れていく


    
    //spotを模倣したときのやつ
    // else if(step_num==10){
    //   spot_orb_pub();
    // }
    if(step_num==10){
      kick_ground_pub();
    }

    else if(step_num==9){
      one_leg_pub();
    }

    else if(step_num==8){
      pull_push_pub();
    }
    else if(step_num==4){
      triangle_pub();
    }
    else if(step_num==6){
      if(crawl_mode){
        crawl_walk_pub_2();
      }
      else{
        one_point_pub();
      }
    }
    else if(step_num==7){
      if(crawl_mode){
        // crawl_walk_pub_7();
        // crawl_walk_pub_8();
        crawl_walk_pub_9();
        // one_side_pub();
      }
      else if(spider_mode){
        spider_pub();
      }
      else{
        add_front_pub();
      }
      
    }

    //theta_front、足の軌道をリセット
    vector<float>().swap(fl_theta_list);
    vector<float>().swap(fr_theta_list);
    vector<float>().swap(bl_theta_list);
    vector<float>().swap(br_theta_list);

    value_reset();

    // std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
    

}


void dog_robot_class::joint_state(int num,std::string name_joint){
    
    joint_rad.name.at(num)=name_joint;

}

void dog_robot_class::button_push(const std_msgs::Float32MultiArray& msg_push){
    if(msg_push.data[0]>0){//L1ボタン
      L1_switch=true;
      // std::cout<<"L1"<<endl;
    }
    else{
      L1_switch=false;
    }
    if(msg_push.data[1]>0){//R1
      R1_switch=true;
      // std::cout<<"R1"<<endl;
    }
    else{
      R1_switch=false;
    }
    if(msg_push.data[2]>0){//Cross
      Cross_switch=true;
      // std::cout<<"Cross"<<endl;
      if(robot_mode){//歩行、旋回、体の回転、手動制御をonにする
      std::cout<<"手動制御on"<<endl;
        //自動姿勢制御をoff
        robot_keep=false;
        kick_car_mode=false;
        car_mode=false;
        crawl_mode=false;
        arm_mode=false;
        spider_mode=false;
        gp_center=true;
        value_reset();
        start_position();
        
        //歩き、回転、旋回、処理終了をon
        robot_move=true;
        end_pub=true;
      }
      std::this_thread::sleep_for(chrono::seconds(2));
    }
    else{
      Cross_switch=false;
    }
    if(msg_push.data[3]>0){//Square
      Square_switch=true;
      // std::cout<<"Square"<<endl;
      robot_keep=false;
      if(crawlr_mode){
        value_reset();
        std::cout<<"Crawlerを初期姿勢にします"<<endl;
        start_crawler();
      }
      else if(robot_mode||crawl_mode){
        value_reset();
        std::cout<<"4footを初期姿勢にします"<<endl;
        start_position();
      }
      else if(spider_mode){
        value_reset();
        std::cout<<"spiderを初期姿勢にします"<<endl;
        start_spider();
      }
      std::this_thread::sleep_for(chrono::seconds(2));
    }
    else{
      Square_switch=false;
    }
    if(msg_push.data[4]>0){//Circle//spider_modeモード始動
      Circle_switch=true;
      robot_mode=false;
      car_mode=false;
      crawlr_mode=false;
      robot_keep=false;//imu使用やめます
      arm_mode=false;
      kick_car_mode=false;
      crawl_mode=false;
      spider_mode=true;
      
      std::cout<<"spider_modeモード始動"<<endl;
      start_spider();
      std::this_thread::sleep_for(chrono::seconds(2));
    }
    else{
      Circle_switch=false;
    }
    if(msg_push.data[5]>0){//L2
      L2_switch=true;
      // std::cout<<"L2"<<endl;
    }
    else{
      L2_switch=false;

    }
    if(msg_push.data[6]>0){//R2
      R2_switch=true;
          // std::cout<<"R2"<<endl;
    }
    else{
      R2_switch=false;
    }
    if(msg_push.data[7]>0){//SHARE
      SHARE_switch=true;
      // if(robot_keep){//imuがonになっているなら
      //   std::cout<<"imu使用停止"<<endl;
      //   robot_keep=false;
      //   robot_rotate=true;
      //   end_pub=true;
      //   std::this_thread::sleep_for(chrono::seconds(2));
      // }
      // else{//imuがoffになっているなら
      //   robot_keep=true;
      //   end_pub=true;
      //   robot_rotate=false;
      //   std::cout<<"imu使用します"<<endl;
      //   std::this_thread::sleep_for(chrono::seconds(2));
      // }
      // std::cout<<"SHARE"<<endl;
      if(always_theta1==0.0f){
        always_theta1=5/180.0f*p;
        std::cout<<"初期位置を外側にします"<<endl;
      }
      else if(always_theta1!=0.0f){
        always_theta1=0.0f;
        std::cout<<"初期位置を元に戻します"<<endl;
      }
      std::this_thread::sleep_for(chrono::seconds(2));
      
    }
    else{
      SHARE_switch=false;
    }
    if(msg_push.data[8]>0){//OPTION//全部0にする
      OPTION_switch=true;
      // std::cout<<"OPTION"<<endl;
      std::cout<<"全部0にします"<<endl;
      crawlr_mode=false;
      robot_mode=false;
      robot_keep=false;
      car_mode=false;
      crawl_mode=false;
      kick_car_mode=false;
      arm_mode=false;
      gp_center=false;
      spider_mode=false;
      for(int i=0;i<12;i++){
        joint_rad.position[i]=0;
      }
      for(int i=0;i<16;i++){
        joint_rad.velocity[i]=0;
      }
      pub(joint_rad);
      std::this_thread::sleep_for(chrono::seconds(2));
    }
    else{
      OPTION_switch=false;
    }
    if(msg_push.data[9]>0){//PSButton
      PSButton_switch=true;
      if(!crawlr_mode&&robot_mode){//クローラモードに変更
        std::cout<<"Crawler mode on 今は無し"<<endl;
        robot_mode=false;
        kick_car_mode=false;
        car_mode=false;
        arm_mode=false;
        spider_mode=false;
        crawlr_mode=true;
        // start_crawler();
      }
      else if((!robot_mode&&crawlr_mode)||(!robot_mode&&!crawlr_mode)){
        std::cout<<"4foot mode on"<<endl;
        crawlr_mode=false;
        car_mode=false;
        kick_car_mode=false;
        arm_mode=false;
        crawl_mode=false;
        spider_mode=false;
        robot_mode=true;
        start_position();
      }
      std::this_thread::sleep_for(chrono::seconds(2));
    }
    else{
      PSButton_switch=false;
    }
    if(msg_push.data[10]>0){//L3
      L3_switch=true;
      // std::cout<<"L3"<<endl;
      if(body_height<0.15f&&body_height>=-0.05f){
        body_height+=0.05f;
        std::cout<<"体を下げる::"<<body_height<<endl;
        robot_rotate=true;
        rotate_body_by_height(0,0);
        robot_rotate=false;
        std::this_thread::sleep_for(chrono::milliseconds(1000));
      }
    }
    else{
      L3_switch=false;
            
    }
    if(msg_push.data[11]>0){//R3
      R3_switch=true;
      // std::cout<<"R3"<<endl;
      if(body_height<=0.15f&&body_height>-0.05f){
        body_height-=0.05f;
        std::cout<<"体を上げる::"<<body_height<<endl;
        robot_rotate=true;
        rotate_body_by_height(0,0);
        robot_rotate=false;
        std::this_thread::sleep_for(chrono::milliseconds(1000));
      }
      
      
    }
    else{
      R3_switch=false;
    }
    if(msg_push.data[12]>0){//Triangle
      Triangle_switch=true;
      std::cout<<"crawl mode on"<<endl;
      car_mode=false;
      robot_mode=false;
      robot_keep=false;
      crawlr_mode=false;
      arm_mode=false;
      kick_car_mode=false;
      spider_mode=false;
      crawl_mode=true;
      start_position();
      value_reset();
      std::this_thread::sleep_for(chrono::seconds(2));
      // std::cout<<"Triangle"<<endl;
    }
    else{
      Triangle_switch=false;
    }

    
}

void dog_robot_class::position_keep(const geometry_msgs::Vector3& msg){//imuの値を受け取る
  if(count==0){//起動時のpitch、roll角を入手
    if(msg.x>0){
      imu_roll_x_st=p-msg.x;
    }
    else{
      imu_roll_x_st=msg.x+p;
    }
    imu_pitch_y_st = msg.y;
    count++;
    imu_pitch_y_past=imu_pitch_y_st;
    imu_roll_x_past=imu_roll_x_st;
    std::cout<<"基準値設定（position_keep）"<<endl;
  }
  if(robot_keep){//imu使用onなら、
    // imu_roll_x  = msg.x;
    imu_pitch_y = -1.0f*(msg.y-imu_pitch_y_st);//基準からどれだけpitchがずれてるか
    if(msg.x>0){
      imu_roll_x=(p-msg.x-imu_roll_x_st);//基準からどれだけrollがずれてるか
      std::cout<<"正"<<endl;
    }
    else{
      imu_roll_x = -1.0f*(msg.x+p-imu_roll_x_st);
    }
    
    //歩幅(a)=sqrt(pow(height*cos(roll),2)+pow(height*cos(pitch),2))
    //傾き(thetaxy)=atan2(cos(pitch),cos(roll))
    

    // if(std::abs(imu_roll_x-imu_roll_x_past)<0.01f){//偏差が0.01以下なら過去と同じにする
    //   imu_roll_x=imu_roll_x_past;
    // }
    // if(std::abs(imu_roll_x-imu_roll_x_past)<0.01f){
    //   imu_pitch_y=imu_pitch_y_past;
    // }
    // imu_roll_x_past=imu_roll_x;
    // imu_pitch_y_past=imu_pitch_y;


    imu_roll_x=-1.0f*std::floor(imu_roll_x * 100)/100;
    imu_pitch_y=std::floor(imu_pitch_y * 100)/100;


    bool use_lowpass=true;
    if(use_lowpass){
      float ratio_past=0.9;
      imu_roll_x=ratio_past*imu_roll_x_past+(1-ratio_past)*imu_roll_x;
      imu_pitch_y=ratio_past*imu_pitch_y_past+(1-ratio_past)*imu_pitch_y;
      imu_roll_x_past=imu_roll_x;
      imu_pitch_y_past=imu_pitch_y;
    }

    std::cout<<"imu_pitch::"<<imu_pitch_y<<endl;
    std::cout<<"imu_roll::"<<imu_roll_x<<endl;
  }
  // if((imu_pitch_y>body_rotate_pitch||imu_pitch_y<-1.0f*body_rotate_pitch||imu_roll_x>body_rotate_roll||imu_roll_x<-1.0f*body_rotate_roll)){
  // }
}

void dog_robot_class::stop_roll(){//imuがthreshold_roll以上の値になったときに起動し、脚を動かし立て直す
  //棒を手でバランス取るように考えた
  if(std::abs(imu_roll_x)>threshold_roll||std::abs(imu_pitch_y)>threshold_roll){
    std::cout<<"倒れます"<<endl;
    float roll_height=0.25f;
    float a=sqrt(pow(roll_height*cos(imu_roll_x),2)+pow(roll_height*cos(imu_pitch_y),2))/4.0f;//
    float origin_=a;

    float thetaxy=atan2(cos(imu_pitch_y),cos(imu_roll_x));
    float b_l=0.05f;
    float b_r=0.15f;
    int keep=step_num;//現在のstep_numを保存しておく、stop_rollではstep_numを8でのみ使用してるため
    if(imu_pitch_y>threshold_roll){
      b_l=0.15f;
      b_r=0.05f;
    }
    // std::cout<<"歩幅：："<<a<<"：：傾き：："<<thetaxy<<"：：左足：："<<b_l<<"：：右足：："<<b_r<<endl;
    // std::cout<<"歩幅：："<<a<<"：：pitch：："<<imu_pitch_y<<"：：roll_x：："<<imu_roll_x<<"：：高さ：："<<height<<endl;

  //倒れる向きから、左右のどっちの脚をたくさん上げるかを指定するため、左右それぞれの軌道を作成
    std::vector<std::vector<float>> orbit_magnitude_l={{origin_*cos(thetaxy),origin_*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),0},{(-1.0f*a+origin_)*cos(thetaxy),(-1.0f*a+origin_)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b_l*1.0f/sqrt(2.0f)},{origin_*cos(thetaxy),origin_*sin(thetaxy),b_l},{(1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b_l*1.0f/sqrt(2.0f)},{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),0}};
    std::vector<std::vector<float>> orbit_magnitude_r={{origin_*cos(thetaxy),origin_*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),0},{(-1.0f*a+origin_)*cos(thetaxy),(-1.0f*a+origin_)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b_r*1.0f/sqrt(2.0f)},{origin_*cos(thetaxy),origin_*sin(thetaxy),b_r},{(1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b_r*1.0f/sqrt(2.0f)},{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),0}};

    step_num=8;

    std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_l,roll_height,a,0.0f);
    std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_r,roll_height,a,0.0f);
    std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_l,roll_height,a,0.0f);
    std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_r,roll_height,a,0.0f);

    fl_theta_list=make_theta(orb_rotate_fl,roll_height,-1.0f);
    fr_theta_list=make_theta(orb_rotate_fr,roll_height,-1.0f);
    bl_theta_list=make_theta(orb_rotate_bl,roll_height,1.0f);
    br_theta_list=make_theta(orb_rotate_br,roll_height,1.0f);

    orbit_list();
    pull_push_pub();
    vector<float>().swap(fl_theta_list);
    vector<float>().swap(fr_theta_list);
    vector<float>().swap(bl_theta_list);
    vector<float>().swap(br_theta_list);

    value_reset();
    step_num=keep;//step_numをもとに戻す

    }
}

void dog_robot_class::start_crawler(){//crawlerの初期姿勢
  //値リセット
  fl_crwlr_th2=c_st_th2;
  fr_crwlr_th2=c_st_th2;
  bl_crwlr_th2=c_st_th2;
  br_crwlr_th2=c_st_th2;
  fl_crwlr_th3=c_st_th3;
  fr_crwlr_th3=c_st_th3;
  bl_crwlr_th3=c_st_th3;
  br_crwlr_th3=c_st_th3;
  change_mode={start_theta1,start_theta2,start_theta3,start_theta1,start_theta2*4.0f/5.0f,start_theta3*4.0f/5.0f,start_theta1,start_theta2*3.0f/5.0f,start_theta3*3.0f/5.0f,  start_theta1,start_theta2*2.0f/5.0f,start_theta3*2.0f/5.0f,  start_theta1,start_theta2*1.0f/5.0f,start_theta3*1.0f/5.0f,  start_theta1,start_theta2*0.0f/5.0f,start_theta3*0.0f/5.0f,c_st_th1,c_st_th2*1.0f/5.0f,c_st_th3*1.0f/5.0f,c_st_th1,c_st_th2*2.0f/5.0f,c_st_th3*2.0f/5.0f,c_st_th1,c_st_th2*3.0f/5.0f,c_st_th3*3.0f/5.0f,c_st_th1,c_st_th2*4.0f/5.0f,c_st_th3*5.0f/5.0f,c_st_th1,c_st_th2,c_st_th3};
  // for(int i=0;i<9;i++){
  //   std::cout<<change_mode[i]<<endl;
  // }
  change_mode_velocity={0,0,0,0,0,0,1,1,1,1,0};
  // change_mode={c_st_th1,c_st_th2,c_st_th3};
  for(int i=0;i<11;i++){
    joint_rad.position[0]=change_mode[3*i];
    joint_rad.position[1]=-1.0f*change_mode[3*i+1];
    joint_rad.position[2]=change_mode[3*i+2];
    joint_rad.position[3]=-1.0f*change_mode[3*i];
    joint_rad.position[4]=change_mode[3*i+1];
    joint_rad.position[5]=-1.0f*change_mode[3*i+2];
    joint_rad.position[6]=-1.0f*change_mode[3*i];
    joint_rad.position[7]=change_mode[3*i+1];
    joint_rad.position[8]=-1.0f*change_mode[3*i+2];
    joint_rad.position[9]=change_mode[3*i];
    joint_rad.position[10]=-1.0f*change_mode[3*i+1];
    joint_rad.position[11]=change_mode[3*i+2];
    joint_rad.velocity[12]=change_mode_velocity[i];
    joint_rad.velocity[13]=change_mode_velocity[i]*-1.0f;
    joint_rad.velocity[14]=change_mode_velocity[i]*-1.0f;
    joint_rad.velocity[15]=change_mode_velocity[i];
    pub(joint_rad);
    std::this_thread::sleep_for(chrono::milliseconds(5));
  }
  
}  

void dog_robot_class::crwlr_move(float ang_y,float ang_z){//クローラの動きを作成、velocity値のみ
  float speed=sqrt(pow(ang_y,2)+pow(ang_z,2))*max_crwlr_speed;//max_crwlr_speedで最大値指定
  float dir=atan2(ang_y,ang_z);
  if(ang_z==0&&ang_y==0){
    dir=0;
  }
  if(speed>7.0f){
    speed=7.0f;
  }
  float left_speed=0;
  float right_speed=0;

  if(dir<p/4.0f&&dir>-1.0f*p/4.0f){
    left_speed=speed;//前周り
    right_speed=-1.0f*speed;//前周り
  }
  if(dir>p/4.0f&&dir<3.0f*p/4.0f){
    left_speed=speed;
    right_speed=speed;
  }
  if(dir<-p/4.0f&&dir>-3.0f*p/4.0f){
    left_speed=-1.0f*speed;
    right_speed=-1.0f*speed;
  }
  if((dir<p&&dir>3.0f*p/4.0f)||(dir>-1.0f*p&&dir<-3.0f*p/4.0f)){
    left_speed=-1.0f*speed;
    right_speed=speed;
  }
  
  crwlr_move_pub(left_speed,right_speed);
}

void dog_robot_class::leg_height_th2(float vel_z,int leg){//フリッパth2を変更
  if(vel_z>0){
    if(leg==0){
      fl_crwlr_th2=fl_crwlr_th2+flipper_rotate_speed/180.0f*p;
      // std::cout<<"flのth2::"<<fl_crwlr_th2<<endl;
      if(fl_crwlr_th2>max_crwlr_th2){//上限を超えたら
        fl_crwlr_th2=max_crwlr_th2;
  }
    }
    
    if(leg==1){
      fr_crwlr_th2=fr_crwlr_th2+flipper_rotate_speed/180.0f*p;
      // std::cout<<"frのth2::"<<fr_crwlr_th2<<endl;
      if(fr_crwlr_th2>max_crwlr_th2){//上限を超えたら
        fr_crwlr_th2=max_crwlr_th2;
    }
    }
    if(leg==2){
      bl_crwlr_th2=bl_crwlr_th2+flipper_rotate_speed/180.0f*p;
      // std::cout<<"blのth2::"<<bl_crwlr_th2<<endl;
      if(bl_crwlr_th2>max_crwlr_th2){//上限を超えたら
        bl_crwlr_th2=max_crwlr_th2;
    }
    }
    if(leg==3){
      br_crwlr_th2=br_crwlr_th2+flipper_rotate_speed/180.0f*p;
      // std::cout<<"brのth2::"<<br_crwlr_th2<<endl;
      if(br_crwlr_th2>max_crwlr_th2){//上限を超えたら
        br_crwlr_th2=max_crwlr_th2;
    }
    }
  }
  else if(vel_z<0){
    if(leg==0){
      fl_crwlr_th2=fl_crwlr_th2-flipper_rotate_speed/180.0f*p;
      // std::cout<<"flのth2::"<<fl_crwlr_th2<<endl;
      if(fl_crwlr_th2<min_crwlr_th2){//下限を超えたら
        fl_crwlr_th2=min_crwlr_th2;
      }
    }
    if(leg==1){
      fr_crwlr_th2=fr_crwlr_th2-flipper_rotate_speed/180.0f*p;
      // std::cout<<"frのth2::"<<fr_crwlr_th2<<endl;
      if(fr_crwlr_th2<min_crwlr_th2){//下限を超えたら
        fr_crwlr_th2=min_crwlr_th2;
      }
    }
    if(leg==2){
      bl_crwlr_th2=bl_crwlr_th2-flipper_rotate_speed/180.0f*p;
      // std::cout<<"blのth2::"<<bl_crwlr_th2<<endl;
      if(bl_crwlr_th2<min_crwlr_th2){//下限を超えたら
        bl_crwlr_th2=min_crwlr_th2;
      }
    }
    if(leg==3){
      br_crwlr_th2=br_crwlr_th2-flipper_rotate_speed/180.0f*p;
      // std::cout<<"brのth2::"<<br_crwlr_th2<<endl;
      if(br_crwlr_th2<min_crwlr_th2){//下限を超えたら
        br_crwlr_th2=min_crwlr_th2;
      }
    }
  }
  crwlr_flipper_pub();
}
void dog_robot_class::leg_height_th3(float ang_x,int leg){//フリッパth3を変更
  if(ang_x<0){
    if(leg==0){
      fl_crwlr_th3=fl_crwlr_th3+flipper_rotate_speed/180.0f*p;
      // std::cout<<"flのth3::"<<fl_crwlr_th3<<endl;
      
      if(fl_crwlr_th3>max_crwlr_th3){//上限を超えたら
        fl_crwlr_th3=max_crwlr_th3;
  }
    }
    if(leg==1){
      fr_crwlr_th3=fr_crwlr_th3+flipper_rotate_speed/180.0f*p;
      if(fr_crwlr_th3>max_crwlr_th3){//上限を超えたら
        fr_crwlr_th3=max_crwlr_th3;
    }
      // std::cout<<"frのth3::"<<fr_crwlr_th3<<endl;
    }
    if(leg==2){
      bl_crwlr_th3=bl_crwlr_th3+flipper_rotate_speed/180.0f*p;
      if(bl_crwlr_th3>max_crwlr_th3){//上限を超えたら
        bl_crwlr_th3=max_crwlr_th3;
    }
      // std::cout<<"blのth3::"<<bl_crwlr_th3<<endl;     
    }
    if(leg==3){
      br_crwlr_th3=br_crwlr_th3+flipper_rotate_speed/180.0f*p;
      if(br_crwlr_th3>max_crwlr_th3){//上限を超えたら
        br_crwlr_th3=max_crwlr_th3;
    }
      // std::cout<<"brのth3::"<<br_crwlr_th3<<endl;
      
    }
  }
  else if(ang_x>0){
    if(leg==0){
      fl_crwlr_th3=fl_crwlr_th3-flipper_rotate_speed/180.0f*p;
      // std::cout<<"flのth3::"<<fl_crwlr_th3<<endl;
      if(fl_crwlr_th3<min_crwlr_th3){//下限を超えたら
        fl_crwlr_th3=min_crwlr_th3;
      }
    }
    if(leg==1){
      fr_crwlr_th3=fr_crwlr_th3-flipper_rotate_speed/180.0f*p;
      // std::cout<<"frのth3::"<<fr_crwlr_th3<<endl;
      if(fr_crwlr_th3<min_crwlr_th3){//下限を超えたら
        fr_crwlr_th3=min_crwlr_th3;
      }
    }
    if(leg==2){
      bl_crwlr_th3=bl_crwlr_th3-flipper_rotate_speed/180.0f*p;
      // std::cout<<"blのth3:"<<bl_crwlr_th3<<endl;
      if(bl_crwlr_th3<min_crwlr_th3){//下限を超えたら
        bl_crwlr_th3=min_crwlr_th3;
      }
    }
    if(leg==3){
      br_crwlr_th3=br_crwlr_th3-flipper_rotate_speed/180.0f*p;
      // std::cout<<"brのth3::"<<br_crwlr_th3<<endl;
      if(br_crwlr_th3<min_crwlr_th3){//下限を超えたら
        br_crwlr_th3=min_crwlr_th3;
      }
    }
  }
  
  crwlr_flipper_pub();
}



  

void dog_robot_class::crwlr_move_pub(float left_speed,float right_speed){//クローラのvelocity値をpub
  int num=12;
  if(kick_car_mode){
    num=14;
  }
  for(int i=0;i<num;i++){
    joint_rad.velocity[i]=0;
  }
  if(!kick_car_mode){
    joint_rad.velocity[12]=left_speed;
    joint_rad.velocity[13]=right_speed;
  }
  
  joint_rad.velocity[14]=left_speed;
  joint_rad.velocity[15]=right_speed;
  // for(int i=0;i<16;i++){
  //   joint_rad.position[i]=0;
  // }
  
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

}
void dog_robot_class::crwlr_flipper_pub(){//クローラのthetaのposition値をpub
  joint_rad.position[0]=fl_r[0];//imuがrollだけ入るようにするため
  joint_rad.position[1]=-1.0f*fl_crwlr_th2;
  joint_rad.position[2]=fl_crwlr_th3;
  joint_rad.position[3]=fr_r[0];
  joint_rad.position[4]=fr_crwlr_th2;
  joint_rad.position[5]=-1.0f*fr_crwlr_th3;
  joint_rad.position[6]=bl_r[0];
  joint_rad.position[7]=bl_crwlr_th2;
  joint_rad.position[8]=-1.0f*bl_crwlr_th3;
  joint_rad.position[9]=br_r[0];
  joint_rad.position[10]=-1.0f*br_crwlr_th2;
  joint_rad.position[11]=br_crwlr_th3;
  for(int i=12;i<16;i++){
    joint_rad.position[i]=0;
  }
  pub(joint_rad);
}
void dog_robot_class::crwlr_value_reset(){//velocity値を全部0になる
  for(int i=0;i<16;i++){
    joint_rad.velocity[i]=0;
  }
}

void dog_robot_class::kick_car(float vel_x,float vel_y,float ang_y,float ang_z){//前足の歩きのみ、後ろは姿勢キープ
  float speed = max_speed*sqrt(pow(vel_x,2)+pow(vel_y,2));//歩幅、倒す量0~1.0
  float a=speed/2;//進行速度を長軸の半分に//-1<=speed<=1
  float dir=-1.0f*atan2(vel_x,vel_y);
  int step_num_keep=step_num;
  step_num=9;
  b=0.05f;
  b_under=0.0f;
  if(R1_switch){//押してる間脚上げる量変える。
    b_under=-0.03f;
  }
  else if(R2_switch){
    b_under=-0.07f;
  }

  // std::cout<<dir*180.0f/p<<endl;
  if(vel_x==0&&vel_y==0){
      dir=0.0f;
  }




  float thetaxy=-1.0f*dir;//joyをどの角度に傾けるかを用いる、それが進行方向に
  float origin_=a;//原点を足の接地点とするため、楕円は長軸の半分だけずれる

  origin_=0.0f;
  // float from=0.05f;
  

  std::vector<std::vector<float>> orbit_magnitude_f={{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),sin(p/8.0f)*b_under},{origin_*cos(thetaxy),origin_*sin(thetaxy),sin(p/4.0f)*b_under},{(-1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b_under},{(-a+origin_)*cos(thetaxy),(-a+origin_)*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{origin_*cos(thetaxy),origin_*sin(thetaxy),b},{(1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0}};

  

  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);


  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,-1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,-1.0f);
  for(int i=0;i<9;i++){
    bl_theta_list.insert(bl_theta_list.end(),{kcf_st_th1,kcf_st_th2,kcf_st_th3});//後ろの脚は常にキープ
    br_theta_list.insert(br_theta_list.end(),{kcf_st_th1,kcf_st_th2,kcf_st_th3});
  }

  orbit_list();
    
  kick_car_pub();

    //theta_front、足の軌道をリセット
  vector<float>().swap(fl_theta_list);
  vector<float>().swap(fr_theta_list);
  vector<float>().swap(bl_theta_list);
  vector<float>().swap(br_theta_list);

  kick_value_reset();

  step_num=step_num_keep;
  std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
    
}

void dog_robot_class::start_kick_car(){//kick_carの初期姿勢
  height=L2*cos(kcf_st_th2)+L3*cos(-1.0f*kcf_st_th2+-1.0f*kcf_st_th3);
  now_height_fl=height;
  now_height_fr=height;
  now_height_bl=height;
  now_height_br=height;



  joint_rad.position[0]=kcf_st_th1;
  joint_rad.position[1]=-1.0f*kcf_st_th2;
  joint_rad.position[2]=kcf_st_th3;
  joint_rad.position[3]=-1.0f*kcf_st_th1;
  joint_rad.position[4]=kcf_st_th2;
  joint_rad.position[5]=-1.0f*kcf_st_th3;
  joint_rad.position[6]=-1.0f*kcb_st_th1;
  joint_rad.position[7]=kcb_st_th2;
  joint_rad.position[8]=-1.0f*kcb_st_th3;
  joint_rad.position[9]=kcb_st_th1;
  joint_rad.position[10]=-1.0f*kcb_st_th2;
  joint_rad.position[11]=kcb_st_th3;
  pub(joint_rad);

  // change_mode_kcf={start_theta1,start_theta2,start_theta3,start_theta1,start_theta2*4.0f/5.0f,start_theta3*4.0f/5.0f,start_theta1,start_theta2*3.0f/5.0f,start_theta3*3.0f/5.0f,  start_theta1,start_theta2*2.0f/5.0f,start_theta3*2.0f/5.0f,  start_theta1,start_theta2*1.0f/5.0f,start_theta3*1.0f/5.0f,  start_theta1,start_theta2*0.0f/5.0f,start_theta3*0.0f/5.0f,kcf_st_th1,kcf_st_th2*1.0f/5.0f,kcf_st_th3*1.0f/5.0f,kcf_st_th1,kcf_st_th2*2.0f/5.0f,kcf_st_th3*2.0f/5.0f,kcf_st_th1,kcf_st_th2*3.0f/5.0f,kcf_st_th3*3.0f/5.0f,kcf_st_th1,kcf_st_th2*4.0f/5.0f,kcf_st_th3*5.0f/5.0f,kcf_st_th1,kcf_st_th2,kcf_st_th3};
  // change_mode_kcb={start_theta1,start_theta2,start_theta3,start_theta1,start_theta2*4.0f/5.0f,start_theta3*4.0f/5.0f,start_theta1,start_theta2*3.0f/5.0f,start_theta3*3.0f/5.0f,  start_theta1,start_theta2*2.0f/5.0f,start_theta3*2.0f/5.0f,start_theta1,start_theta2*1.0f/5.0f,start_theta3*1.0f/5.0f,start_theta1,start_theta2*0.0f/5.0f,start_theta3*0.0f/5.0f,kcb_st_th1,kcb_st_th2*1.0f/5.0f,kcb_st_th3*1.0f/5.0f,kcb_st_th1,kcb_st_th2*2.0f/5.0f,kcb_st_th3*2.0f/5.0f,kcb_st_th1,kcb_st_th2*3.0f/5.0f,kcb_st_th3*3.0f/5.0f,kcb_st_th1,kcb_st_th2*4.0f/5.0f,kcb_st_th3*5.0f/5.0f,kcb_st_th1,kcb_st_th2,kcb_st_th3};
  // // for(int i=0;i<9;i++){
  // //   std::cout<<change_mode[i]<<endl;
  // // }
  // change_mode_velocity={0,0,0,0,0,0,1,1,1,1,0};
  // // change_mode={c_st_th1,c_st_th2,c_st_th3};
  // for(int i=0;i<11;i++){
  //   joint_rad.position[0]=change_mode_kcf[3*i];
  //   joint_rad.position[1]=-1.0f*change_mode_kcf[3*i+1];
  //   joint_rad.position[2]=change_mode_kcf[3*i+2];
  //   joint_rad.position[3]=-1.0f*change_mode_kcf[3*i];
  //   joint_rad.position[4]=change_mode_kcf[3*i+1];
  //   joint_rad.position[5]=-1.0f*change_mode_kcf[3*i+2];
  //   joint_rad.position[6]=-1.0f*change_mode_kcb[3*i];
  //   joint_rad.position[7]=change_mode_kcb[3*i+1];
  //   joint_rad.position[8]=-1.0f*change_mode_kcb[3*i+2];
  //   joint_rad.position[9]=change_mode_kcb[3*i];
  //   joint_rad.position[10]=-1.0f*change_mode_kcb[3*i+1];
  //   joint_rad.position[11]=change_mode_kcb[3*i+2];
  //   joint_rad.velocity[12]=change_mode_velocity[i];
  //   joint_rad.velocity[13]=change_mode_velocity[i]*-1.0f;
  //   joint_rad.velocity[14]=change_mode_velocity[i]*-1.0f;
  //   joint_rad.velocity[15]=change_mode_velocity[i];
  //   pub(joint_rad);
  //   std::this_thread::sleep_for(chrono::milliseconds(5));
  // }

}
void dog_robot_class::kick_car_pub(){//kick_car時のpub
  for(int i=0;i<=3;i++){//fl 0,1,2,3 bl 4,5,6,7
      joint_rad.position[0] =fl1[i]+fl_r[0]-kcf_st_th1;
      joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*kcf_st_th2;
      joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-kcf_st_th3;
      joint_rad.position[3] =fr1[i+4]+fr_r[0]+1.0f*kcf_st_th1;
      joint_rad.position[4] =-1.0f*fr2[i+4]-1.0f*(fr_r[1]+p/2.0f)-kcf_st_th2;
      joint_rad.position[5] =fr3[i+4]+fr_r[2]+1.0f*kcf_st_th3;
      joint_rad.position[6] =-1.0f*bl1[i+4]-1.0f*bl_r[0]+1.0f*kcb_st_th1;
      joint_rad.position[7] =-1.0f*bl2[i+4]-1.0f*(bl_r[1]+p/2.0f)-kcb_st_th2;
      joint_rad.position[8] =bl3[i+4]+bl_r[2]+1.0f*kcb_st_th3;
      joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-kcb_st_th1;
      joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*kcb_st_th2;
      joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-kcb_st_th3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
      // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  for(int i=0;i<=3;i++){//fl 4,5,6,7 bl 0,1,2,3
      joint_rad.position[0] =fl1[i+4]+fl_r[0]-kcf_st_th1;
      joint_rad.position[1] =fl2[i+4]+(fl_r[1]+p/2.0f)+1.0f*kcf_st_th2;
      joint_rad.position[2] =-1.0f*fl3[i+4]-1.0f*fl_r[2]-kcf_st_th3;
      joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*kcf_st_th1;
      joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-kcf_st_th2;
      joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*kcf_st_th3;
      joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*kcb_st_th1;
      joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-kcb_st_th2;
      joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*kcb_st_th3;
      joint_rad.position[9] =-1.0f*br1[i+4]-1.0f*br_r[0]-kcb_st_th1;
      joint_rad.position[10]=br2[i+4]+(br_r[1]+p/2.0f)+1.0f*kcb_st_th2;
      joint_rad.position[11]=-1.0f*br3[i+4]-1.0f*br_r[2]-kcb_st_th3;
      pub(joint_rad);
      std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
      // std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  joint_rad.position[0] =fl1[8]+fl_r[0]-kcf_st_th1;
  joint_rad.position[1] =fl2[8]+(fl_r[1]+p/2.0f)+1.0f*kcf_st_th2;
  joint_rad.position[2] =-1.0f*fl3[8]-1.0f*fl_r[2]-kcf_st_th3;
  joint_rad.position[3] =fr1[8]+fr_r[0]+1.0f*kcf_st_th1;
  joint_rad.position[4] =-1.0f*fr2[8]-1.0f*(fr_r[1]+p/2.0f)-kcf_st_th2;
  joint_rad.position[5] =fr3[8]+fr_r[2]+1.0f*kcf_st_th3;
  joint_rad.position[6] =-1.0f*bl1[8]-1.0f*bl_r[0]+1.0f*kcb_st_th1;
  joint_rad.position[7] =-1.0f*bl2[8]-1.0f*(bl_r[1]+p/2.0f)-kcb_st_th2;
  joint_rad.position[8] =bl3[8]+bl_r[2]+1.0f*kcb_st_th3;
  joint_rad.position[9] =-1.0f*br1[8]-1.0f*br_r[0]-kcb_st_th1;
  joint_rad.position[10]=br2[8]+(br_r[1]+p/2.0f)+1.0f*kcb_st_th2;
  joint_rad.position[11]=-1.0f*br3[8]-1.0f*br_r[2]-kcb_st_th3;
  pub(joint_rad);

  std::this_thread::sleep_for(std::chrono::milliseconds(time));

}

void dog_robot_class::arm_move(float ang_z,float vel_y,float ang_x){
//RStick　歩幅、LStick　高さ
  max_x=sqrt(pow((L2+L3)*7/8.0f,2)-pow(arm_z,2));
  max_z=sqrt(pow((L2+L3)*7/8.0f,2)-pow(arm_x,2));
  std::cout<<"max_x::"<<max_x<<endl;
  // std::cout<<"y::"<<arm_y<<endl;
  std::cout<<"max_z::"<<max_z<<endl;
  if(ang_z<0&&arm_x<max_x){//脚上げる高さ//RStick上で脚上げる
    arm_x+=0.01f;
  }
  else if(ang_z>0&&arm_x>-1.0f*max_x){
    arm_x-=0.01f;
  }
  
  if(vel_y>0&&arm_z<max_z){//歩幅調整//LStick上で前に脚出す
    arm_z+=0.01f;
  }
  else if(vel_y<0&&arm_z>-1.0f*max_z){
    arm_z-=0.01f;
  } 
  

  if(ang_x>0&&arm_theta1<90/180.0f*p){
    arm_theta1+=1/180.0f*p;
  }
  else if(ang_x<0&&arm_theta1>-90/180.0f*p){
    arm_theta1-=1/180.0f*p;
  }
  
  

  std::cout<<"x::"<<arm_x<<endl;
  // std::cout<<"y::"<<arm_y<<endl;
  std::cout<<"z::"<<arm_z<<endl;

  float theta2=-1.0f*inverce_kinetic_theta2(arm_x,arm_y,arm_z,0,0,0);
  float theta3=1.0f*inverce_kinetic_theta3(arm_x,arm_y,arm_z,theta2,0,0,0);

  joint_rad.position[0]=arm_theta1;
  joint_rad.position[1]=-1.0f*theta2;
  joint_rad.position[2]=theta3;
  joint_rad.position[3]=-1.0f*start_theta1;
  joint_rad.position[4]=start_theta2;
  joint_rad.position[5]=-1.0f*start_theta3;
  joint_rad.position[6]=-1.0f*start_theta1;
  joint_rad.position[7]=start_theta2;
  joint_rad.position[8]=-1.0f*start_theta3;
  joint_rad.position[9]=start_theta1;
  joint_rad.position[10]=-1.0f*start_theta2;
  joint_rad.position[11]=start_theta3;
  pub(joint_rad);
  std::this_thread::sleep_for(chrono::milliseconds(200));
}


void dog_robot_class::start_spider(){
  height=L2*cos(sp_st_th2)+L3*cos(-1.0f*sp_st_th2-1.0f*sp_st_th3);//初期height値の計算
  origin_culculate=L2*sin(-1.0f*sp_st_th2)-L3*sin(sp_st_th2+sp_st_th3);

  now_height_fl=height;
  now_height_fr=height;
  now_height_bl=height;
  now_height_br=height;

  joint_rad.position[0]=sp_st_th1+always_theta1;
  joint_rad.position[1]=-1.0f*sp_st_th2;
  joint_rad.position[2]=sp_st_th3;
  joint_rad.position[3]=-1.0f*sp_st_th1-always_theta1;
  joint_rad.position[4]=sp_st_th2;
  joint_rad.position[5]=-1.0f*sp_st_th3;
  joint_rad.position[6]=-1.0f*sp_st_th1-always_theta1;
  joint_rad.position[7]=sp_st_th2;
  joint_rad.position[8]=-1.0f*sp_st_th3;
  joint_rad.position[9]=sp_st_th1+always_theta1;
  joint_rad.position[10]=-1.0f*sp_st_th2;
  joint_rad.position[11]=sp_st_th3;
  pub(joint_rad);
}

void dog_robot_class::spider_orb(float vel_x,float vel_y,float ang_y,float ang_z){
  float speed = max_speed*sqrt(pow(vel_x,2)+pow(vel_y,2));//歩幅、倒す量0~1.0//max_speedで変更可能
  float a=speed/2;//進行速度を長軸の半分に//-1<=speed<=1//進む距離
  float dir=-1.0f*atan2(vel_x,vel_y);//進む角度
  b=0.05f;//前足の上げる量
  b_b=0.05f;//後ろ脚の上げる量
  float ground_under_f=0.0f;
  float ground_under_b=0.0f;
  if(R1_switch){//押してる間脚上げる量変える。
    b=0.1f;
  }
  else if(R2_switch){
    ground_under_f=-0.05f;
  }
  if(L1_switch){
    b_b=0.1f;
  }
  else if(L2_switch){
    ground_under_b=-0.05f;
  }
  // std::cout<<dir*180.0f/p<<endl;
  if(vel_x==0&&vel_y==0){//atan2値が発散しないように使用
    dir=0.0f;
  }

  float thetaxy=-1.0f*dir;//joyをどの角度に傾けるかを用いる、それが進行方向に
  float origin_f=a;//原点を足の接地点とするため、楕円は長軸の半分だけずれる
  float origin_b=a;

  origin_f=-1.0f*origin_culculate;
  float add_front=0.05f;//前から接地するために使用
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_f;

//初期姿勢からスタートさせるのに必要、excelより求めるしかないかも

  // float from=0.05f;
  //軌道生成、脚を上げる量を変えたいので、前と後ろどっちも作成している
  std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),ground_under_f},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),ground_under_f},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),ground_under_f},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_f-add_front)*cos(thetaxy),(-1.0f*a+origin_f-add_front)*sin(thetaxy),b/2.0f},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),ground_under_f},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),ground_under_f}};
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),ground_under_b},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),ground_under_b},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),ground_under_b},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_b+add_front)*cos(thetaxy),(a+origin_b+add_front)*sin(thetaxy),b_b/2.0f},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),ground_under_b},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),ground_under_b}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);

//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);

  orbit_list();//fl1,fl2,fl3などに値を入れていく

  spider_pub();

  vector<float>().swap(fl_theta_list);
  vector<float>().swap(fr_theta_list);
  vector<float>().swap(bl_theta_list);
  vector<float>().swap(br_theta_list);

  value_reset();
}

void dog_robot_class::spider_pub(){//全部同時に、同じ動きで動かす
  for(int i=0;i<3;i++){//fl 0 012 bl 3 456
    joint_rad.position[0] =fl1[i]+always_theta1;
    joint_rad.position[1] =fl2[i];
    joint_rad.position[2] =-1.0f*fl3[i];
    joint_rad.position[3] =fr1[i]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[i];
    joint_rad.position[5] =fr3[i+3];
    joint_rad.position[6] =-1.0f*bl1[i]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[i];
    joint_rad.position[8] =bl3[i];
    joint_rad.position[9] =-1.0f*br1[i]+always_theta1;
    joint_rad.position[10]=br2[i];
    joint_rad.position[11]=-1.0f*br3[i];
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));//5ミリ秒止める
  }
}

void dog_robot_class::spider_value_reset(){

}
void dog_robot_class::press_check(const std_msgs::Float32MultiArray& foot_msg){//センサー値をpress_valueに保管
  for(int i=0;i<4;i++){
    press_value[i]=foot_msg.data[i];
    // std::cout<<i<<":::"<<press_value[i]<<endl;
  }
  // std::cout<<"fl:::"<<foot_msg.data[0]<<endl;
  // std::cout<<"fr:::"<<foot_msg.data[1]<<endl;
  // std::cout<<"bl:::"<<foot_msg.data[2]<<endl;
  // std::cout<<"br:::"<<foot_msg.data[3]<<endl;

  float threshhold_fl=0.85;
  float threshhold_fr=0.85;
  float threshhold_bl=0.85;
  float threshhold_br=0.85;

  if(foot_msg.data[0]<threshhold_fl){
    press_fl=0;
  }
  else{
    press_fl=9;
  }
  if(foot_msg.data[1]<threshhold_fr){
    press_fr=3;
  }
  else{
    press_fr=6;
  }
  if(foot_msg.data[2]<threshhold_bl){
    press_bl=6;
  }
  else{
    press_bl=3;
  }
  if(foot_msg.data[3]<threshhold_br){
    press_br=9;
  }
  else{
    press_br=0;
  }  
}

void dog_robot_class::controll_gravity(){
  for(int i=0;i<4;i++){
    std::cout<<i<<"::"<<press_value[i]<<endl;
  }
  for(int i=1;i<4;i++){
      if(press_value[now_max_index]<press_value[i]){
        now_max_index=i;
      }  
  }
  switch (now_max_index){
    case 0://brに傾く
      press_height_fl-=press_height_change;
      press_height_fr-=press_height_change;
      press_height_bl-=press_height_change;
      press_height_br+=press_height_change;
      break;
    case 1://blに傾く
      press_height_fl-=press_height_change;
      press_height_fr-=press_height_change;
      press_height_bl+=press_height_change;
      press_height_br-=press_height_change;
      break;
    case 2://frに傾く
      press_height_fl-=press_height_change;
      press_height_fr+=press_height_change;
      press_height_bl-=press_height_change;
      press_height_br-=press_height_change;
      break;
    case 3://flに傾く
      press_height_fl+=press_height_change;
      press_height_fr-=press_height_change;
      press_height_bl-=press_height_change;
      press_height_br-=press_height_change;
      break;
  }
  
  // rotate_body_by_height(press_height_fl,press_height_fr,press_height_bl,press_height_br);//pitch,roll,vel_z
  now_max_index=0;//リセット
}

void dog_robot_class::now_angle_get(const sensor_msgs::JointState& now_angle){
  now_fl_th1=now_angle.position[0];
  now_fl_th2=now_angle.position[1];
  now_fl_th3=now_angle.position[2];
  now_fr_th1=now_angle.position[3];
  now_fr_th2=now_angle.position[4];
  now_fr_th3=now_angle.position[5];
  now_bl_th1=now_angle.position[6];
  now_bl_th2=now_angle.position[7];
  now_bl_th3=now_angle.position[8];
  now_br_th1=now_angle.position[9];
  now_br_th2=now_angle.position[10];
  now_br_th3=now_angle.position[11];

  now_height_fl=L2*cos(now_fl_th1)*cos(now_fl_th2)+L3*cos(now_fl_th1)*cos(now_fl_th2+now_fl_th3);
  now_height_fr=L2*cos(now_fr_th1)*cos(now_fr_th2)+L3*cos(now_fr_th1)*cos(now_fr_th2+now_fr_th3);
  now_height_bl=L2*cos(now_bl_th1)*cos(now_bl_th2)+L3*cos(now_bl_th1)*cos(now_bl_th2+now_bl_th3);
  now_height_br=L2*cos(now_br_th1)*cos(now_br_th2)+L3*cos(now_br_th1)*cos(now_br_th2+now_br_th3);
}

void dog_robot_class::crawl_walk(float vel_x,float vel_y,float ang_x,float vel_z){//旋回できない
  float speed = max_crawl_speed*sqrt(pow(vel_x,2)+pow(vel_y,2));//歩幅、倒す量0~1.0//max_speedで変更可能
  float a=speed;//進行速度を長軸の半分に//-1<=speed<=1//進む距離
  float dir=-1.0f*atan2(vel_x,vel_y);//進む角度
  int step_num_keep=step_num;

  step_num=8;//8点扱うため
  b=0.05f;//前足の上げる量
  b_b=0.05f;//後ろ脚の上げる量
  if(R1_switch){//押してる間脚上げる量変える。
    b=0.1f;
  }
  else if(R2_switch){
    b=0.15f;
  }
  if(L1_switch){
    b_b=0.1f;
  }
  else if(L2_switch){
    b_b=0.15f;
  }
  // std::cout<<dir*180.0f/p<<endl;
  if(vel_x==0&&vel_y==0){//atan2値が発散しないように使用
    dir=0.0f;
  }
  float thetaxy=-1.0f*dir;//joyをどの角度に傾けるかを用いる、それが進行方向に
  float origin_f=0.1f;
  float origin_b=0.1f;
  origin_f=-1.0f*origin_culculate;
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_culculate;
  std::vector<std::vector<float>> orbit_magnitude_f={{(origin_f+a*cos(90/180.0f*p))*cos(thetaxy),(origin_f+a*cos(90/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*0.0f/4.0f)*cos(thetaxy),(origin_f+a*0.0f/4.0f)*sin(thetaxy),b*0},{(origin_f+a*1.0f/4.0f)*cos(thetaxy),(origin_f+a*1.0f/4.0f)*sin(thetaxy),b*0},{(origin_f+a*2.0f/4.0f)*cos(thetaxy),(origin_f+a*2.0f/4.0f)*sin(thetaxy),b*0},{(origin_f+a*3.0f/4.0f)*cos(thetaxy),(origin_f+a*3.0f/4.0f)*sin(thetaxy),b*0},{(origin_f+a*cos(0/180.0f*p))*cos(thetaxy),(origin_f+a*cos(0/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(60/180.0f*p))*cos(thetaxy),(origin_f+a*cos(60/180.0f*p))*sin(thetaxy),b},{(origin_f+a*-1.0f/4.0f)*cos(thetaxy),(origin_f+a*-1.0f/4.0f)*sin(thetaxy),b*0.5f}};
  // std::cout<<"x"<<endl;
  // for(int i=0;i<9;i++){
  //   std::cout<<orbit_magnitude_f[i][0]<<endl;
  // }
  // std::cout<<"y"<<endl;
  // for(int i=0;i<9;i++){
  //   std::cout<<orbit_magnitude_f[i][1]<<endl;
  // }
  // std::cout<<"z"<<endl;
  // for(int i=0;i<9;i++){
  //   std::cout<<orbit_magnitude_f[i][2]<<endl;
  // }
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{(origin_b+a*cos(90/180.0f*p))*cos(thetaxy),(origin_b+a*cos(90/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*-1.0f/4.0f)*cos(thetaxy),(origin_b+a*-1.0f/4.0f)*sin(thetaxy),b_b*0.5f},{(origin_b+a*cos(60/180.0f*p))*cos(thetaxy),(origin_b+a*cos(60/180.0f*p))*sin(thetaxy),b_b},{(origin_b+a*cos(0/180.0f*p))*cos(thetaxy),(origin_b+a*cos(0/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*3.0f/4.0f)*cos(thetaxy),(origin_b+a*3.0f/4.0f)*sin(thetaxy),b_b*0},{(origin_b+a*2.0f/4.0f)*cos(thetaxy),(origin_b+a*2.0f/4.0f)*sin(thetaxy),b_b*0},{(origin_b+a*1.0f/4.0f)*cos(thetaxy),(origin_b+a*1.0f/4.0f)*sin(thetaxy),b_b*0},{(origin_b+a*0.0f/4.0f)*cos(thetaxy),(origin_b+a*0.0f/4.0f)*sin(thetaxy),b_b*0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);

  orbit_list();
  // crawl_walk_pub();
  crawl_walk_rotate_pub();
  vector<float>().swap(fl_theta_list);
  vector<float>().swap(fr_theta_list);
  vector<float>().swap(bl_theta_list);
  vector<float>().swap(br_theta_list);

  value_reset();

  std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
  step_num=step_num_keep;
}

void dog_robot_class::crawl_walk_pub(){
  //1
  joint_rad.position[0] =fl1[0]+fl_r[0]-start_theta1;
  joint_rad.position[1] =fl2[0]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[2] =-1.0f*fl3[0]-1.0f*fl_r[2]-start_theta3;
  joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
  joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;
  joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
  joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
  joint_rad.position[9] =-1.0f*br1[0]-1.0f*br_r[0]-start_theta1;
  joint_rad.position[10]=br2[0]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[11]=-1.0f*br3[0]-1.0f*br_r[2]-start_theta3;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  
  //2//3//4
  for(int i=1;i<4;i++){
    joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;//1-3
    joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //5//6//7
  for(int i=1;i<4;i++){
    joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;//1-3
    joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //8//9
  for(int i=4;i<6;i++){
    joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;//4-5
    joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //10//11//12
  for(int i=1;i<4;i++){
    joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;//1-3
    joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //13//14//15
  for(int i=1;i<4;i++){
    joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;//1-3
    joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //16//17
  for(int i=4;i<6;i++){
    joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;//4-5
    joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //18//19
  for(int i=6;i<8;i++){
    joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;//6-7
    joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[i+4]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  
}
void dog_robot_class::crawl_walk_rotate_pub(){
  //1
  // rotate_bodt(RS左右、RS上下、)
  rotate_body(0,0,0);
  joint_rad.position[0] =fl1[0]+fl_r[0]-start_theta1;
  joint_rad.position[1] =fl2[0]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[2] =-1.0f*fl3[0]-1.0f*fl_r[2]-start_theta3;
  joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
  joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;
  joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
  joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
  joint_rad.position[9] =-1.0f*br1[0]-1.0f*br_r[0]-start_theta1;
  joint_rad.position[10]=br2[0]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[11]=-1.0f*br3[0]-1.0f*br_r[2]-start_theta3;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  
  //2//3//4
  rotate_body(-1.0f*cwalk_rotate,-1.0f*cwalk_rotate,0);//brに傾き
  for(int i=1;i<4;i++){
    joint_rad.position[0] =fl1[i];//1-3
    joint_rad.position[1] =fl2[i];
    joint_rad.position[2] =-1.0f*fl3[i];

    joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =-1.0f*br1[0]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[0]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[0]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //5//6//7
  rotate_body(cwalk_rotate,cwalk_rotate,0);//flに傾き
  for(int i=1;i<4;i++){
    joint_rad.position[9] =-1.0f*br1[i];//1-3
    joint_rad.position[10]=br2[i];
    joint_rad.position[11]=-1.0f*br3[i];

    joint_rad.position[0] =fl1[3]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[3]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[3]-1.0f*fl_r[2]-start_theta3;

    joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //8//9
  rotate_body(0,0,0);
  for(int i=4;i<6;i++){
    joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;//4-5
    joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;

    joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;

    joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //10//11//12
  rotate_body(cwalk_rotate,-1.0f*cwalk_rotate,0);//blに傾き
  for(int i=1;i<4;i++){
    joint_rad.position[3] =fr1[i];//1-3
    joint_rad.position[4] =-1.0f*fr2[i];
    joint_rad.position[5] =fr3[i]+fr_r[2];

    joint_rad.position[0] =fl1[5]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[5]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[5]-1.0f*fl_r[2]-start_theta3;

    joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;

    joint_rad.position[9] =-1.0f*br1[5]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[5]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[5]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //13//14//15
  rotate_body(-1.0f*cwalk_rotate,cwalk_rotate,0);//frに傾き
  for(int i=1;i<4;i++){
    joint_rad.position[6] =-1.0f*bl1[i];//1-3
    joint_rad.position[7] =-1.0f*bl2[i];
    joint_rad.position[8] =bl3[i];

    joint_rad.position[0] =fl1[5]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[5]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[5]-1.0f*fl_r[2]-start_theta3;

    joint_rad.position[3] =fr1[3]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[3]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[3]+fr_r[2]+1.0f*start_theta3;

    joint_rad.position[9] =-1.0f*br1[5]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[5]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[5]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //16//17
  rotate_body(0,0,0);
  for(int i=4;i<6;i++){
    joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;//4-5
    joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;

    joint_rad.position[0] =fl1[5]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[5]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[5]-1.0f*fl_r[2]-start_theta3;

    joint_rad.position[9] =-1.0f*br1[5]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[5]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[5]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //18//19
  rotate_body(0,0,0);
  for(int i=6;i<8;i++){
    joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;//6-7
    joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[i+4]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[i]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[i]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[i]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  
}

void dog_robot_class::crawl_walk_2(float vel_x,float vel_y,float ang_x,float vel_z){
  float speed = max_crawl_speed*sqrt(pow(vel_x,2)+pow(vel_y,2));//歩幅、倒す量0~1.0//max_speedで変更可能
  float a=speed;//進行速度を長軸の半分に//-1<=speed<=1//進む距離
  float dir=-1.0f*atan2(vel_x,vel_y);//進む角度
  int step_num_keep=step_num;

  step_num=7;
  b=0.05f;//前足の上げる量
  b_b=0.05f;//後ろ脚の上げる量
  if(R1_switch){//押してる間脚上げる量変える。
    b=0.1f;
  }
  else if(R2_switch){
    b=0.15f;
  }
  if(L1_switch){
    b_b=0.1f;
  }
  else if(L2_switch){
    b_b=0.15f;
  }
  // std::cout<<dir*180.0f/p<<endl;
  if(vel_x==0&&vel_y==0){//atan2値が発散しないように使用
    dir=0.0f;
  }
  float thetaxy=-1.0f*dir;//joyをどの角度に傾けるかを用いる、それが進行方向に
  float origin_f=0.1f;
  float origin_b=0.1f;
  origin_f=-1.0f*origin_culculate;
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  origin_b=origin_culculate;
  if(ang_x!=0){
    std::vector<std::vector<float>> fl_orb={{(origin_f+a*cos(90/180.0f*p))*cos(thetaxy),(origin_f+a*cos(90/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(45/180.0f*p))*cos(thetaxy),(origin_f+a*cos(45/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(0/180.0f*p))*cos(thetaxy),(origin_f+a*cos(0/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(90/180.0f*p))*cos(thetaxy),(origin_f+a*cos(90/180.0f*p))*sin(thetaxy),b},{(origin_f+a*cos(180/180.0f*p))*cos(thetaxy),(origin_f+a*cos(180/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b*0}};
    std::vector<std::vector<float>> fr_orb={{(origin_f+a*cos(90/180.0f*p))*cos(thetaxy),(origin_f+a*cos(90/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(45/180.0f*p))*cos(thetaxy),(origin_f+a*cos(45/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(0/180.0f*p))*cos(thetaxy),(origin_f+a*cos(0/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(90/180.0f*p))*cos(thetaxy),(origin_f+a*cos(90/180.0f*p))*sin(thetaxy),b},{(origin_f+a*cos(180/180.0f*p))*cos(thetaxy),(origin_f+a*cos(180/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b*0}};
    std::vector<std::vector<float>> bl_orb={{(origin_b+a*cos(90/180.0f*p))*cos(thetaxy),(origin_b+a*cos(90/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(135/180.0f*p))*cos(thetaxy),(origin_b+a*cos(135/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(180/180.0f*p))*cos(thetaxy),(origin_b+a*cos(180/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(90/180.0f*p))*cos(thetaxy),(origin_b+a*cos(90/180.0f*p))*sin(thetaxy),b_b},{(origin_b+a*cos(0/180.0f*p))*cos(thetaxy),(origin_b+a*cos(0/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b*0}};
    std::vector<std::vector<float>> br_orb={{(origin_b+a*cos(90/180.0f*p))*cos(thetaxy),(origin_b+a*cos(90/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(135/180.0f*p))*cos(thetaxy),(origin_b+a*cos(135/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(180/180.0f*p))*cos(thetaxy),(origin_b+a*cos(180/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(90/180.0f*p))*cos(thetaxy),(origin_b+a*cos(90/180.0f*p))*sin(thetaxy),b_b},{(origin_b+a*cos(0/180.0f*p))*cos(thetaxy),(origin_b+a*cos(0/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b*0}};
    
    std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(fl_orb,now_height_fl,a,0.0f);
    std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(fr_orb,now_height_fr,a,0.0f);
    std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(bl_orb,now_height_bl,a,0.0f);
    std::vector<std::vector<float>> orb_rotate_br=each_make_orb(br_orb,now_height_br,a,0.0f);
  //角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
    fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
    fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
    bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
    br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);

    orbit_list();
  // crawl_walk_pub();
  // crawl_walk_rotate_pub();
    crawl_walk_pub_2();
  }
  else{
  std::vector<std::vector<float>> orbit_magnitude_f={{(origin_f+a*cos(90/180.0f*p))*cos(thetaxy),(origin_f+a*cos(90/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(45/180.0f*p))*cos(thetaxy),(origin_f+a*cos(45/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(0/180.0f*p))*cos(thetaxy),(origin_f+a*cos(0/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(90/180.0f*p))*cos(thetaxy),(origin_f+a*cos(90/180.0f*p))*sin(thetaxy),b},{(origin_f+a*cos(180/180.0f*p))*cos(thetaxy),(origin_f+a*cos(180/180.0f*p))*sin(thetaxy),b*0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b*0}};
  // std::cout<<"x"<<endl;
  // for(int i=0;i<9;i++){
  //   std::cout<<orbit_magnitude_f[i][0]<<endl;
  // }
  // std::cout<<"y"<<endl;
  // for(int i=0;i<9;i++){
  //   std::cout<<orbit_magnitude_f[i][1]<<endl;
  // }
  // std::cout<<"z"<<endl;
  // for(int i=0;i<9;i++){
  //   std::cout<<orbit_magnitude_f[i][2]<<endl;
  // }
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{(origin_b+a*cos(90/180.0f*p))*cos(thetaxy),(origin_b+a*cos(90/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(135/180.0f*p))*cos(thetaxy),(origin_b+a*cos(135/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(180/180.0f*p))*cos(thetaxy),(origin_b+a*cos(180/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(90/180.0f*p))*cos(thetaxy),(origin_b+a*cos(90/180.0f*p))*sin(thetaxy),b_b},{(origin_b+a*cos(0/180.0f*p))*cos(thetaxy),(origin_b+a*cos(0/180.0f*p))*sin(thetaxy),b_b*0},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b*0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);
//角度を一次元配列で保存3*iがth1,3*i+1がth2,3*i+2がth3
  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);

  orbit_list();
  // crawl_walk_pub();
  // crawl_walk_rotate_pub();
  crawl_walk_pub_2();
}
  vector<float>().swap(fl_theta_list);
  vector<float>().swap(fr_theta_list);
  vector<float>().swap(bl_theta_list);
  vector<float>().swap(br_theta_list);

  value_reset();

  std::this_thread::sleep_for(std::chrono::milliseconds(1));//5ミリ秒止める
  step_num=step_num_keep;
}

void dog_robot_class::crawl_walk_pub_2(){
  //1 fl 2 fr 0 bl 0 br 2
  joint_rad.position[0] =fl1[2]+fl_r[0]-start_theta1;
  joint_rad.position[1] =fl2[2]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[2] =-1.0f*fl3[2]-1.0f*fl_r[2]-start_theta3;
  joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
  joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;
  joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
  joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
  joint_rad.position[9] =-1.0f*br1[2]-1.0f*br_r[0]-start_theta1;
  joint_rad.position[10]=br2[2]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[11]=-1.0f*br3[2]-1.0f*br_r[2]-start_theta3;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  //23  fl 345 fr 0 bl 0 br 2
  for(int i=3;i<5;i++){
    joint_rad.position[0] =fl1[i]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[i]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[i]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =-1.0f*br1[2]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[2]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[2]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //45 fl 5 fr 0 bl 0 br 345
  for(int i=3;i<5;i++){
    joint_rad.position[0] =fl1[4]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[4]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =-1.0f*fl3[4]-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[0]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =-1.0f*fr2[0]-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[0]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =-1.0f*bl1[0]-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =-1.0f*bl2[0]-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[0]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =-1.0f*br1[i]-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[i]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=-1.0f*br3[i]-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //6 fl 6 fr 1 bl 1 br 6
  joint_rad.position[0] =fl1[5]+fl_r[0]-start_theta1;
  joint_rad.position[1] =fl2[5]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[2] =fl3[5]*-1.0f-1.0f*fl_r[2]-start_theta3;
  joint_rad.position[3] =fr1[1]+fr_r[0]+1.0f*start_theta1;
  joint_rad.position[4] =fr2[1]*-1.0f-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[5] =fr3[1]+fr_r[2]+1.0f*start_theta3;
  joint_rad.position[6] =bl1[1]*-1.0f-1.0f*bl_r[0]+1.0f*start_theta1;
  joint_rad.position[7] =bl2[1]*-1.0f-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[8] =bl3[1]+bl_r[2]+1.0f*start_theta3;
  joint_rad.position[9] =br1[5]*-1.0f-1.0f*br_r[0]-start_theta1;
  joint_rad.position[10]=br2[5]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[11]=br3[5]*-1.0f-1.0f*br_r[2]-start_theta3;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  //7 fl 0 fr 2 bl 2 br 0
  joint_rad.position[0] =fl1[0]+fl_r[0]-start_theta1;
  joint_rad.position[1] =fl2[0]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[2] =fl3[0]*-1.0f-1.0f*fl_r[2]-start_theta3;
  joint_rad.position[3] =fr1[2]+fr_r[0]+1.0f*start_theta1;
  joint_rad.position[4] =fr2[2]*-1.0f-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[5] =fr3[2]+fr_r[2]+1.0f*start_theta3;
  joint_rad.position[6] =bl1[2]*-1.0f-1.0f*bl_r[0]+1.0f*start_theta1;
  joint_rad.position[7] =bl2[2]*-1.0f-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[8] =bl3[2]+bl_r[2]+1.0f*start_theta3;
  joint_rad.position[9] =br1[0]*-1.0f-1.0f*br_r[0]-start_theta1;
  joint_rad.position[10]=br2[0]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[11]=br3[0]*-1.0f-1.0f*br_r[2]-start_theta3;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  //89 fl 0 fr 345 bl 2 br 0
  for(int i=3;i<5;i++){
    joint_rad.position[0] =fl1[0]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[0]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =fl3[0]*-1.0f-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[i]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =fr2[i]*-1.0f-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[i]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =bl1[2]*-1.0f-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =bl2[2]*-1.0f-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[2]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =br1[0]*-1.0f-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[0]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=br3[0]*-1.0f-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //1011 fl 0 fr 5 bl 345 br 0
  for(int i=3;i<5;i++){
    joint_rad.position[0] =fl1[0]+fl_r[0]-start_theta1;
    joint_rad.position[1] =fl2[0]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[2] =fl3[0]*-1.0f-1.0f*fl_r[2]-start_theta3;
    joint_rad.position[3] =fr1[4]+fr_r[0]+1.0f*start_theta1;
    joint_rad.position[4] =fr2[4]*-1.0f-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[5] =fr3[4]+fr_r[2]+1.0f*start_theta3;
    joint_rad.position[6] =bl1[i]*-1.0f-1.0f*bl_r[0]+1.0f*start_theta1;
    joint_rad.position[7] =bl2[i]*-1.0f-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
    joint_rad.position[8] =bl3[i]+bl_r[2]+1.0f*start_theta3;
    joint_rad.position[9] =br1[0]*-1.0f-1.0f*br_r[0]-start_theta1;
    joint_rad.position[10]=br2[0]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
    joint_rad.position[11]=br3[0]*-1.0f-1.0f*br_r[2]-start_theta3;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //12 fl 1 fr 6 bl 6 br 1
  joint_rad.position[0] =fl1[1]+fl_r[0]-start_theta1;
  joint_rad.position[1] =fl2[1]+(fl_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[2] =fl3[1]*-1.0f-1.0f*fl_r[2]-start_theta3;
  joint_rad.position[3] =fr1[5]+fr_r[0]+1.0f*start_theta1;
  joint_rad.position[4] =fr2[5]*-1.0f-1.0f*(fr_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[5] =fr3[5]+fr_r[2]+1.0f*start_theta3;
  joint_rad.position[6] =bl1[5]*-1.0f-1.0f*bl_r[0]+1.0f*start_theta1;
  joint_rad.position[7] =bl2[5]*-1.0f-1.0f*(bl_r[1]+p/2.0f)-start_theta2;
  joint_rad.position[8] =bl3[5]+bl_r[2]+1.0f*start_theta3;
  joint_rad.position[9] =br1[1]*-1.0f-1.0f*br_r[0]-start_theta1;
  joint_rad.position[10]=br2[1]+(br_r[1]+p/2.0f)+1.0f*start_theta2;
  joint_rad.position[11]=br3[1]*-1.0f-1.0f*br_r[2]-start_theta3;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
}

void dog_robot_class::crawl_walk_pub_7(){
  //1 fl 2 fr 0 bl 0 br 2
  joint_rad.position[0] =fl1[2]+always_theta1;
  joint_rad.position[1] =fl2[2];
  joint_rad.position[2] =-1.0f*fl3[2];
  joint_rad.position[3] =fr1[0]-always_theta1;
  joint_rad.position[4] =-1.0f*fr2[0];
  joint_rad.position[5] =fr3[0];
  joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
  joint_rad.position[7] =-1.0f*bl2[0];
  joint_rad.position[8] =bl3[0];
  joint_rad.position[9] =-1.0f*br1[2]+always_theta1;
  joint_rad.position[10]=br2[2];
  joint_rad.position[11]=-1.0f*br3[2];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time_crawl));
  //23  fl 345 fr 0 bl 0 br 2
  // rotate_body_by_height(0.0f,0.0f,0.0f,by_height);//brに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[i]+always_theta1;
    joint_rad.position[1] =fl2[i];
    joint_rad.position[2] =-1.0f*fl3[i];
    joint_rad.position[3] =fr1[0]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[0];
    joint_rad.position[5] =fr3[0];
    joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[0];
    joint_rad.position[8] =bl3[0];
    joint_rad.position[9] =-1.0f*br1[2]+always_theta1;
    joint_rad.position[10]=br2[2];
    joint_rad.position[11]=-1.0f*br3[2];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    if(press_fl==0&&(i==3||i==4)){//fl脚が浮いているはずなのに、脚がついてしまった
      press_yes_check(press_fl);
    }
  //   if(press_fl==9&&i==3){  //一回pubの後、flが接地してない可能性
  //     press_no_check(press_fl,1.0f,-1.0f);
  // }
  }

  if(press_fl==9){  //一回pubの後、flが接地してない可能性
      press_no_check(press_fl,1.0f,-1.0f);
  }
  //45 fl 5 fr 0 bl 0 br 345
  // rotate_body_by_height(by_height,0.0f,0.0f,0.0f);//flに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[5]+always_theta1;
    joint_rad.position[1] =fl2[5];
    joint_rad.position[2] =-1.0f*fl3[5];
    joint_rad.position[3] =fr1[0]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[0];
    joint_rad.position[5] =fr3[0];
    joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[0];
    joint_rad.position[8] =bl3[0];
    joint_rad.position[9] =-1.0f*br1[i]+always_theta1;
    joint_rad.position[10]=br2[i];
    joint_rad.position[11]=-1.0f*br3[i];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    if(press_br==9&&(i==3||i==4)){//br脚が浮いているはずなのに、脚がついてしまった
      press_yes_check(press_br);
    }
  }
  //6 fl 6 fr 1 bl 1 br 6
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[6]+always_theta1;
  joint_rad.position[1] =fl2[6];
  joint_rad.position[2] =fl3[6]*-1.0f;
  joint_rad.position[3] =fr1[1]-always_theta1;
  joint_rad.position[4] =fr2[1]*-1.0f;
  joint_rad.position[5] =fr3[1];
  joint_rad.position[6] =bl1[1]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[1]*-1.0f;
  joint_rad.position[8] =bl3[1];
  joint_rad.position[9] =br1[6]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[6];
  joint_rad.position[11]=br3[6]*-1.0f;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  //7 fl 0 fr 2 bl 2 br 0
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[0]+always_theta1;
  joint_rad.position[1] =fl2[0];
  joint_rad.position[2] =fl3[0]*-1.0f;
  joint_rad.position[3] =fr1[2]-always_theta1;
  joint_rad.position[4] =fr2[2]*-1.0f;
  joint_rad.position[5] =fr3[2];
  joint_rad.position[6] =bl1[2]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[2]*-1.0f;
  joint_rad.position[8] =bl3[2];
  joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[0];
  joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time_crawl));
  //89 fl 0 fr 345 bl 2 br 0
  // rotate_body_by_height(0.0f,0.0f,by_height,0.0f);//blに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[0]+always_theta1;
    joint_rad.position[1] =fl2[0];
    joint_rad.position[2] =fl3[0]*-1.0f;
    joint_rad.position[3] =fr1[i]-always_theta1;
    joint_rad.position[4] =fr2[i]*-1.0f;
    joint_rad.position[5] =fr3[i];
    joint_rad.position[6] =bl1[2]*-1.0f-always_theta1;
    joint_rad.position[7] =bl2[2]*-1.0f;
    joint_rad.position[8] =bl3[2];
    joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
    joint_rad.position[10]=br2[0];
    joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    if(press_fr==3&&(i==3||i==4)){//fr脚が浮いているはずなのに、脚がついてしまった
      press_yes_check(press_fr);
    }
  //   if(press_fl==9&&i==3){  //一回pubの後、flが接地してない可能性
  //     press_no_check(press_fl,1.0f,-1.0f);
  // }
  }

  //1011 fl 0 fr 5 bl 345 br 0
  // rotate_body_by_height(0.0f,by_height,0.0f,0.0f);//frに傾ける
  if(press_fr==6){  //一回pubの後、frが接地してない可能性
      press_no_check(press_fr,-1.0f,1.0f);
  }
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[0]+always_theta1;
    joint_rad.position[1] =fl2[0];
    joint_rad.position[2] =fl3[0]*-1.0f;
    joint_rad.position[3] =fr1[5]-always_theta1;
    joint_rad.position[4] =fr2[5]*-1.0f;
    joint_rad.position[5] =fr3[5];
    joint_rad.position[6] =bl1[i]*-1.0f-always_theta1;
    joint_rad.position[7] =bl2[i]*-1.0f;
    joint_rad.position[8] =bl3[i];
    joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
    joint_rad.position[10]=br2[0];
    joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    if(press_bl==6&&(i==3||i==4)){//bl脚が浮いているはずなのに、脚がついてしまった
      press_yes_check(press_bl);
    }
  }

  //12 fl 1 fr 6 bl 6 br 1
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[1]+always_theta1;
  joint_rad.position[1] =fl2[1];
  joint_rad.position[2] =fl3[1]*-1.0f;
  joint_rad.position[3] =fr1[6]-always_theta1;
  joint_rad.position[4] =fr2[6]*-1.0f;
  joint_rad.position[5] =fr3[6];
  joint_rad.position[6] =bl1[6]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[6]*-1.0f;
  joint_rad.position[8] =bl3[6];
  joint_rad.position[9] =br1[1]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[1];
  joint_rad.position[11]=br3[1]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));

}




void dog_robot_class::crawl_walk_pub_8(){
  //1 fl 2 fr 0 bl 0 br 2
  joint_rad.position[0] =fl1[2]+always_theta1;
  joint_rad.position[1] =fl2[2];
  joint_rad.position[2] =-1.0f*fl3[2];
  joint_rad.position[3] =fr1[0]-always_theta1;
  joint_rad.position[4] =-1.0f*fr2[0];
  joint_rad.position[5] =fr3[0];
  joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
  joint_rad.position[7] =-1.0f*bl2[0];
  joint_rad.position[8] =bl3[0];
  joint_rad.position[9] =-1.0f*br1[2]+always_theta1;
  joint_rad.position[10]=br2[2];
  joint_rad.position[11]=-1.0f*br3[2];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time_crawl));
  //23  fl 345 fr 0 bl 0 br 2
  // rotate_body_by_height(0.0f,0.0f,0.0f,by_height);//brに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[i]+always_theta1;
    joint_rad.position[1] =fl2[i];
    joint_rad.position[2] =-1.0f*fl3[i];
    joint_rad.position[3] =fr1[0]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[0];
    joint_rad.position[5] =fr3[0];
    joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[0];
    joint_rad.position[8] =bl3[0];
    joint_rad.position[9] =-1.0f*br1[2]+always_theta1;
    joint_rad.position[10]=br2[2];
    joint_rad.position[11]=-1.0f*br3[2];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }

  //45 fl 5 fr 0 bl 0 br 345
  // rotate_body_by_height(by_height,0.0f,0.0f,0.0f);//flに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[5]+always_theta1;
    joint_rad.position[1] =fl2[5];
    joint_rad.position[2] =-1.0f*fl3[5];
    joint_rad.position[3] =fr1[0]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[0];
    joint_rad.position[5] =fr3[0];
    joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[0];
    joint_rad.position[8] =bl3[0];
    joint_rad.position[9] =-1.0f*br1[i]+always_theta1;
    joint_rad.position[10]=br2[i];
    joint_rad.position[11]=-1.0f*br3[i];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
  }
  //6 fl 6 fr 1 bl 1 br 6
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[6]+always_theta1;
  joint_rad.position[1] =fl2[6];
  joint_rad.position[2] =fl3[6]*-1.0f;
  joint_rad.position[3] =fr1[1]-always_theta1;
  joint_rad.position[4] =fr2[1]*-1.0f;
  joint_rad.position[5] =fr3[1];
  joint_rad.position[6] =bl1[1]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[1]*-1.0f;
  joint_rad.position[8] =bl3[1];
  joint_rad.position[9] =br1[6]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[6];
  joint_rad.position[11]=br3[6]*-1.0f;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  //7 fl 0 fr 2 bl 2 br 0
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[0]+always_theta1;
  joint_rad.position[1] =fl2[0];
  joint_rad.position[2] =fl3[0]*-1.0f;
  joint_rad.position[3] =fr1[2]-always_theta1;
  joint_rad.position[4] =fr2[2]*-1.0f;
  joint_rad.position[5] =fr3[2];
  joint_rad.position[6] =bl1[2]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[2]*-1.0f;
  joint_rad.position[8] =bl3[2];
  joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[0];
  joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time_crawl));
  //89 fl 0 fr 345 bl 2 br 0
  // rotate_body_by_height(0.0f,0.0f,by_height,0.0f);//blに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[0]+always_theta1;
    joint_rad.position[1] =fl2[0];
    joint_rad.position[2] =fl3[0]*-1.0f;
    joint_rad.position[3] =fr1[i]-always_theta1;
    joint_rad.position[4] =fr2[i]*-1.0f;
    joint_rad.position[5] =fr3[i];
    joint_rad.position[6] =bl1[2]*-1.0f-always_theta1;
    joint_rad.position[7] =bl2[2]*-1.0f;
    joint_rad.position[8] =bl3[2];
    joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
    joint_rad.position[10]=br2[0];
    joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));

  //   if(press_fl==9&&i==3){  //一回pubの後、flが接地してない可能性
  //     press_no_check(press_fl,1.0f,-1.0f);
  // }
  }

  //1011 fl 0 fr 5 bl 345 br 0
  // rotate_body_by_height(0.0f,by_height,0.0f,0.0f);//frに傾ける
  // if(press_fr==6){  //一回pubの後、frが接地してない可能性
  //     press_no_check(press_fr,-1.0f,1.0f);
  // }
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[0]+always_theta1;
    joint_rad.position[1] =fl2[0];
    joint_rad.position[2] =fl3[0]*-1.0f;
    joint_rad.position[3] =fr1[5]-always_theta1;
    joint_rad.position[4] =fr2[5]*-1.0f;
    joint_rad.position[5] =fr3[5];
    joint_rad.position[6] =bl1[i]*-1.0f-always_theta1;
    joint_rad.position[7] =bl2[i]*-1.0f;
    joint_rad.position[8] =bl3[i];
    joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
    joint_rad.position[10]=br2[0];
    joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));

  }

  //12 fl 1 fr 6 bl 6 br 1
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[1]+always_theta1;
  joint_rad.position[1] =fl2[1];
  joint_rad.position[2] =fl3[1]*-1.0f;
  joint_rad.position[3] =fr1[6]-always_theta1;
  joint_rad.position[4] =fr2[6]*-1.0f;
  joint_rad.position[5] =fr3[6];
  joint_rad.position[6] =bl1[6]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[6]*-1.0f;
  joint_rad.position[8] =bl3[6];
  joint_rad.position[9] =br1[1]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[1];
  joint_rad.position[11]=br3[1]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));

}

void dog_robot_class::crawl_walk_pub_9(){
  //1 fl 2 fr 0 bl 0 br 2
  joint_rad.position[0] =fl1[2]+always_theta1;
  joint_rad.position[1] =fl2[2];
  joint_rad.position[2] =-1.0f*fl3[2];
  joint_rad.position[3] =fr1[0]-always_theta1;
  joint_rad.position[4] =-1.0f*fr2[0];
  joint_rad.position[5] =fr3[0];
  joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
  joint_rad.position[7] =-1.0f*bl2[0];
  joint_rad.position[8] =bl3[0];
  joint_rad.position[9] =-1.0f*br1[2]+always_theta1;
  joint_rad.position[10]=br2[2];
  joint_rad.position[11]=-1.0f*br3[2];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time_crawl));
  //23  fl 345 fr 0 bl 0 br 2
  // rotate_body_by_height(0.0f,0.0f,0.0f,by_height);//brに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[i]+always_theta1;
    joint_rad.position[1] =fl2[i];
    joint_rad.position[2] =-1.0f*fl3[i];
    joint_rad.position[3] =fr1[0]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[0];
    joint_rad.position[5] =fr3[0];
    joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[0];
    joint_rad.position[8] =bl3[0];
    joint_rad.position[9] =-1.0f*br1[2]+always_theta1;
    joint_rad.position[10]=br2[2];
    joint_rad.position[11]=-1.0f*br3[2];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    // if(press_fl==0&&(i==3||i==4)){//fl脚が浮いているはずなのに、脚がついてしまった
    //   press_yes_check(press_fl);
    // }
  //   if(press_fl==9&&i==3){  //一回pubの後、flが接地してない可能性
  //     press_no_check(press_fl,1.0f,-1.0f);
  // }
  }

  if(press_fl==9){  //一回pubの後、flが接地してない可能性
      press_no_check(press_fl,1.0f,-1.0f);
  }
  //45 fl 5 fr 0 bl 0 br 345
  // rotate_body_by_height(by_height,0.0f,0.0f,0.0f);//flに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[5]+always_theta1;
    joint_rad.position[1] =fl2[5];
    joint_rad.position[2] =-1.0f*fl3[5];
    joint_rad.position[3] =fr1[0]-always_theta1;
    joint_rad.position[4] =-1.0f*fr2[0];
    joint_rad.position[5] =fr3[0];
    joint_rad.position[6] =-1.0f*bl1[0]-always_theta1;
    joint_rad.position[7] =-1.0f*bl2[0];
    joint_rad.position[8] =bl3[0];
    joint_rad.position[9] =-1.0f*br1[i]+always_theta1;
    joint_rad.position[10]=br2[i];
    joint_rad.position[11]=-1.0f*br3[i];
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    // if(press_br==9&&(i==3||i==4)){//br脚が浮いているはずなのに、脚がついてしまった
    //   press_yes_check(press_br);
    // }
  }
  //6 fl 6 fr 1 bl 1 br 6
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[6]+always_theta1;
  joint_rad.position[1] =fl2[6];
  joint_rad.position[2] =fl3[6]*-1.0f;
  joint_rad.position[3] =fr1[1]-always_theta1;
  joint_rad.position[4] =fr2[1]*-1.0f;
  joint_rad.position[5] =fr3[1];
  joint_rad.position[6] =bl1[1]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[1]*-1.0f;
  joint_rad.position[8] =bl3[1];
  joint_rad.position[9] =br1[6]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[6];
  joint_rad.position[11]=br3[6]*-1.0f;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));
  //7 fl 0 fr 2 bl 2 br 0
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[0]+always_theta1;
  joint_rad.position[1] =fl2[0];
  joint_rad.position[2] =fl3[0]*-1.0f;
  joint_rad.position[3] =fr1[2]-always_theta1;
  joint_rad.position[4] =fr2[2]*-1.0f;
  joint_rad.position[5] =fr3[2];
  joint_rad.position[6] =bl1[2]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[2]*-1.0f;
  joint_rad.position[8] =bl3[2];
  joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[0];
  joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time_crawl));
  //89 fl 0 fr 345 bl 2 br 0
  // rotate_body_by_height(0.0f,0.0f,by_height,0.0f);//blに傾ける
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[0]+always_theta1;
    joint_rad.position[1] =fl2[0];
    joint_rad.position[2] =fl3[0]*-1.0f;
    joint_rad.position[3] =fr1[i]-always_theta1;
    joint_rad.position[4] =fr2[i]*-1.0f;
    joint_rad.position[5] =fr3[i];
    joint_rad.position[6] =bl1[2]*-1.0f-always_theta1;
    joint_rad.position[7] =bl2[2]*-1.0f;
    joint_rad.position[8] =bl3[2];
    joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
    joint_rad.position[10]=br2[0];
    joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    // if(press_fr==3&&(i==3||i==4)){//fr脚が浮いているはずなのに、脚がついてしまった
    //   press_yes_check(press_fr);
    // }
  //   if(press_fl==9&&i==3){  //一回pubの後、flが接地してない可能性
  //     press_no_check(press_fl,1.0f,-1.0f);
  // }
  }

  //1011 fl 0 fr 5 bl 345 br 0
  // rotate_body_by_height(0.0f,by_height,0.0f,0.0f);//frに傾ける
  if(press_fr==6){  //一回pubの後、frが接地してない可能性
      press_no_check(press_fr,-1.0f,1.0f);
  }
  for(int i=3;i<6;i++){
    joint_rad.position[0] =fl1[0]+always_theta1;
    joint_rad.position[1] =fl2[0];
    joint_rad.position[2] =fl3[0]*-1.0f;
    joint_rad.position[3] =fr1[5]-always_theta1;
    joint_rad.position[4] =fr2[5]*-1.0f;
    joint_rad.position[5] =fr3[5];
    joint_rad.position[6] =bl1[i]*-1.0f-always_theta1;
    joint_rad.position[7] =bl2[i]*-1.0f;
    joint_rad.position[8] =bl3[i];
    joint_rad.position[9] =br1[0]*-1.0f+always_theta1;
    joint_rad.position[10]=br2[0];
    joint_rad.position[11]=br3[0]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
    pub(joint_rad);
    std::this_thread::sleep_for(std::chrono::milliseconds(time));
    // if(press_bl==6&&(i==3||i==4)){//bl脚が浮いているはずなのに、脚がついてしまった
    //   press_yes_check(press_bl);
    // }
  }

  //12 fl 1 fr 6 bl 6 br 1
  // rotate_body(0,0,0);
  joint_rad.position[0] =fl1[1]+always_theta1;
  joint_rad.position[1] =fl2[1];
  joint_rad.position[2] =fl3[1]*-1.0f;
  joint_rad.position[3] =fr1[6]-always_theta1;
  joint_rad.position[4] =fr2[6]*-1.0f;
  joint_rad.position[5] =fr3[6];
  joint_rad.position[6] =bl1[6]*-1.0f-always_theta1;
  joint_rad.position[7] =bl2[6]*-1.0f;
  joint_rad.position[8] =bl3[6];
  joint_rad.position[9] =br1[1]*-1.0f+always_theta1;
  joint_rad.position[10]=br2[1];
  joint_rad.position[11]=br3[1]*-1.0f;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.position[12]=crawl_walk_pub;
  // joint_rad.velocity[12]=car_left_speed;
  // joint_rad.velocity[12]=car_right_speed;
  pub(joint_rad);
  std::this_thread::sleep_for(std::chrono::milliseconds(time));

}

void dog_robot_class::press_no_check(int leg_num,float pm_th2,float pm_th3){//接地していないとき　fl 9 fr 6 bl 3 br 0
  joint_rad.position[leg_num]+=0.0f;
  joint_rad.position[leg_num+1]=std::abs(joint_rad.position[leg_num+1])*-1/2.0f*pm_th2;//20
  joint_rad.position[leg_num+2]=std::abs(joint_rad.position[leg_num+1])*pm_th3;//30がおすすめかも
  std::cout<<"脚ついてないよ"<<endl;
  std::cout<<joint_rad.position[leg_num]<<endl;
  std::cout<<joint_rad.position[leg_num+1]<<endl;
  std::cout<<joint_rad.position[leg_num+2]<<endl;
  pub(joint_rad);
  
  
  std::this_thread::sleep_for(std::chrono::milliseconds(700));
}
void dog_robot_class::press_yes_check(int leg_num){//接地したとき　fl 0 fr 3 bl 6 br 9
  //start_theta更新
std::this_thread::sleep_for(std::chrono::milliseconds(2000));
std::cout<<"脚がついてしまった"<<endl;
  st_th1[leg_num/3]=joint_rad.position[leg_num];
  st_th2[leg_num/3]=joint_rad.position[leg_num+1];
  st_th3[leg_num/3]=joint_rad.position[leg_num+2];
  culculate_height_origin_(st_th1[leg_num/3],st_th2[leg_num/3],st_th3[leg_num/3]);//脚がついてしまったときの角度より高さと原点からずらした距離を生成
  switch(leg_num)
  {
  case 0:
    now_height_fl=height_origin[0];
    break;
  
  case 3:
    now_height_fr=height_origin[0];
    break;
  case 6:
    now_height_bl=height_origin[0];
    break;  
  case 9:
    now_height_br=height_origin[0];
    break;
  }
  origin_culculate=height_origin[1];
//新たに脚の軌道を生成する
  vector<float>().swap(fl_theta_list);
  vector<float>().swap(fr_theta_list);
  vector<float>().swap(bl_theta_list);
  vector<float>().swap(br_theta_list);

  value_reset();

  float origin_f=-1.0f*origin_culculate;
  if(dir>-1.0f*p/2&&dir<p/2){
    origin_f=1.0f*origin_culculate;
  }
  float origin_b=origin_culculate;
  float thetaxy=-1.0f*dir;
  float a=speed/2;
  float add_front=0.05f;
  float b_under=0.02f;
  std::vector<std::vector<float>> orbit_magnitude_f={{origin_f*cos(thetaxy),origin_f*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0},{(a+origin_f)*cos(thetaxy),(a+origin_f)*sin(thetaxy),0},{(origin_f+a*cos(135/180.0f*p))*cos(thetaxy),(origin_f+a*cos(135/180.0f*p))*sin(thetaxy),b},{(-1.0f*a+origin_f-add_front)*cos(thetaxy),(-1.0f*a+origin_f-add_front)*sin(thetaxy),b/2.0f},{(-1.0f*a+origin_f)*cos(thetaxy),(-1.0f*a+origin_f)*sin(thetaxy),b_under},{(-1.0f/sqrt(2.0f)*a+origin_f)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_f)*sin(thetaxy),0}};
  //後ろの軌道
  std::vector<std::vector<float>> orbit_magnitude_b={{origin_b*cos(thetaxy),origin_b*sin(thetaxy),0},{(-1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0},{(-1.0f*a+origin_b)*cos(thetaxy),(-1.0f*a+origin_b)*sin(thetaxy),b_under},{(origin_b+a*cos(45/180.0f*p))*cos(thetaxy),(origin_b+a*cos(45/180.0f*p))*sin(thetaxy),b_b},{(a+origin_b+add_front)*cos(thetaxy),(a+origin_b+add_front)*sin(thetaxy),b_b/2.0f},{(a+origin_b)*cos(thetaxy),(a+origin_b)*sin(thetaxy),0},{(1.0f/sqrt(2.0f)*a+origin_b)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_b)*sin(thetaxy),0}};
//各々の高さから、軌道を回転させたxyz値を配列にする
  std::vector<std::vector<float>> orb_rotate_fl=each_make_orb(orbit_magnitude_f,now_height_fl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_fr=each_make_orb(orbit_magnitude_f,now_height_fr,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_bl=each_make_orb(orbit_magnitude_b,now_height_bl,a,0.0f);
  std::vector<std::vector<float>> orb_rotate_br=each_make_orb(orbit_magnitude_b,now_height_br,a,0.0f);

  fl_theta_list=make_theta(orb_rotate_fl,now_height_fl,1.0f);
  fr_theta_list=make_theta(orb_rotate_fr,now_height_fr,1.0f);
  bl_theta_list=make_theta(orb_rotate_bl,now_height_bl,1.0f);
  br_theta_list=make_theta(orb_rotate_br,now_height_br,1.0f);
  
  orbit_list();

}

void dog_robot_class::culculate_height_origin_(float leg_theta1,float leg_theta2,float leg_theta3){
  height_origin[0]=L2*cos(-1.0f*std::abs(leg_theta2))+L3*cos(std::abs(leg_theta3)-std::abs(leg_theta2));//初期height値の計算
  height_origin[1]=L2*sin(-1.0f*std::abs(leg_theta2))+L3*sin(std::abs(leg_theta3)-std::abs(leg_theta2));
    
}

void dog_robot_class::side_leg_updown(float vel_z){
  if(vel_z>0){
    right_updown=side_leg_add;
    left_updown=-1.0f*side_leg_add;
    std::cout<<"サイドの脚上げます"<<endl;
  }
  else if(vel_z<0){
    // right_updown=side_leg_add;
    // left_updown=-1.0f*side_leg_add;
    right_updown=0.0f;
    left_updown=0.0f;
    std::cout<<"サイドの脚下げます"<<endl;
  }
  joint_rad.position[13]=right_updown;
  joint_rad.position[12]=left_updown;
  pub(joint_rad);
  std::this_thread::sleep_for(chrono::milliseconds(1000));
}

void dog_robot_class::zero_velocity(){
  joint_rad.velocity[14]=0;
  joint_rad.velocity[15]=0;
  angle_pub.publish(joint_rad);
}
 //前出す動きから//origin_=a;
    // float orbit_magnitude[][3]={{(-a+origin_)*cos(thetaxy),(-a+origin_)*sin(thetaxy),0},{(-1*sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(-1*sqrt(3.0f)/2*a+origin_)*sin(thetaxy),1.0f/2.0f*b},{(-1.0f/2.0f*a+origin_)*cos(thetaxy),(-1.0f/2.0f*a+origin_)*sin(thetaxy),sqrt(3.0f)/2*b},{(0+origin_)*cos(thetaxy),(0+origin_)*sin(thetaxy),b},{(1.0f/2.0f*a+origin_)*cos(thetaxy),(1.0f/2.0f*a+origin_)*sin(thetaxy),sqrt(3.0f)/2*b},{(sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(sqrt(3.0f)/2*a+origin_)*sin(thetaxy),1.0f/2.0f*b},{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0},{(sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(sqrt(3.0f)/2*a+origin_)*sin(thetaxy),0},{(1.0f/2.0f*a+origin_)*cos(thetaxy),(1.0f/2.0f*a+origin_)*sin(thetaxy),0},{(0+origin_)*cos(thetaxy),(0+origin_)*sin(thetaxy),0},{(-1.0f/2.0f*a+origin_)*cos(thetaxy),(-1.0f/2.0f*a+origin_)*sin(thetaxy),0},{(-1*sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(-1*sqrt(3.0f)/2*a+origin_)*sin(thetaxy),0},{(-a+origin_)*cos(thetaxy),(-a+origin_)*sin(thetaxy),0}};
    
    //引く動きから//origin_=0;//何故か軌跡の回転が逆
    // float orbit_magnitude[][3]={{(0+origin_)*cos(thetaxy),(0+origin_)*sin(thetaxy),0},{(1.0f/2.0f*a+origin_)*cos(thetaxy),(1.0f/2.0f*a+origin_)*sin(thetaxy),0},{(1*sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(1*sqrt(3.0f)/2*a+origin_)*sin(thetaxy),0},{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0},{(1*sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(1*sqrt(3.0f)/2*a+origin_)*sin(thetaxy),1.0f/2.0f*b},{(1.0f/2.0f*a+origin_)*cos(thetaxy),(1.0f/2.0f*a+origin_)*sin(thetaxy),sqrt(3.0f)/2*b},{(0+origin_)*cos(thetaxy),(0+origin_)*sin(thetaxy),b},{(-1.0f/2.0f*a+origin_)*cos(thetaxy),(-1.0f/2.0f*a+origin_)*sin(thetaxy),sqrt(3.0f)/2*b},{(-1*sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(-1*sqrt(3.0f)/2*a+origin_)*sin(thetaxy),1.0f/2.0f*b},{(-a+origin_)*cos(thetaxy),(-a+origin_)*sin(thetaxy),0},{(-1*sqrt(3.0f)/2*a+origin_)*cos(thetaxy),(-1*sqrt(3.0f)/2*a+origin_)*sin(thetaxy),0},{(-1.0f/2.0f*a+origin_)*cos(thetaxy),(-1.0f/2.0f*a+origin_)*sin(thetaxy),0},{(0+origin_)*cos(thetaxy),(0+origin_)*sin(thetaxy),0}};

    //spotの動きを模倣
    // float orbit_magnitude[][3]={{(-1.0f*a+origin_)*cos(thetaxy),(-1.0f*a+origin_)*sin(thetaxy),0},{(-1.0f*cos(5.0f*M_PI/180.0f)*a+origin_)*cos(thetaxy),(-1.0f*cos(5.0f*M_PI/180.0f)*a+origin_)*sin(thetaxy),(sin(5.0f*M_PI/180.0f)*b)},{(-1.0f*cos(10.0f*M_PI/180.0f)*a+origin_)*cos(thetaxy),(-1.0f*cos(10.0f*M_PI/180.0f)*a+origin_)*sin(thetaxy),(sin(10.0f*M_PI/180.0f)*b)},{(-1.0f*cos(15.0f*M_PI/180.0f)*a+origin_)*cos(thetaxy),(-1.0f*cos(15.0f*M_PI/180.0f)*a+origin_)*sin(thetaxy),(sin(15.0f*M_PI/180.0f)*b)},{(-1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(-1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{a*cos(thetaxy),a*sin(thetaxy),b},{(1.0f/sqrt(2.0f)*a+origin_)*cos(thetaxy),(1.0f/sqrt(2.0f)*a+origin_)*sin(thetaxy),b*1.0f/sqrt(2.0f)},{(cos(10.0f*M_PI/180.0f)*a+origin_)*cos(thetaxy),(cos(10.0f*M_PI/180.0f)*a+origin_)*sin(thetaxy),(sin(10.0f*M_PI/180.0f)*b)},{(cos(5.0f*M_PI/180.0f)*a+origin_)*cos(thetaxy),(cos(5.0f*M_PI/180.0f)*a+origin_)*sin(thetaxy),(sin(5.0f*M_PI/180.0f)*b)},{(a+origin_)*cos(thetaxy),(a+origin_)*sin(thetaxy),0}};
 void dog_robot_class::pub(sensor_msgs::JointState pub_msg){
    pub_fl.header.stamp=ros::Time::now();
    pub_fr.header.stamp=ros::Time::now();
    pub_bl.header.stamp=ros::Time::now();
    pub_br.header.stamp=ros::Time::now();
    pub_other.header.stamp=ros::Time::now();

  for(int i=0;i<3;i++){
    pub_fl.position[i]=pub_msg.position[i];
    pub_fr.position[i]=pub_msg.position[i+3];
    pub_bl.position[i]=pub_msg.position[i+6];
    pub_br.position[i]=pub_msg.position[i+9];
  }

    pub_other.position[0]=pub_msg.position[12];
    pub_other.position[1]=pub_msg.position[13];
    pub_other.velocity[2]=pub_msg.velocity[14];
    pub_other.velocity[3]=pub_msg.velocity[15];

    angle_pub_fl.publish(pub_fl);
    angle_pub_fr.publish(pub_fr);
    angle_pub_bl.publish(pub_bl);
    angle_pub_br.publish(pub_br);
   // angle_pub.publish(pub_other);

 
 }