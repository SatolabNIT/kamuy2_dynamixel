#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <thread>
#include <bits/stdc++.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace std{};
class dog_robot_class{
    public:
        dog_robot_class(int motor_number){
            sub= nh.subscribe("/cmd_vel", 1,&dog_robot_class::angle_publish,this);
            sub1=nh.subscribe("/cmd_button",1,&dog_robot_class::button_push,this);
            sub2=nh.subscribe("/imu_rpy",1,&dog_robot_class::position_keep,this);
            sub3=nh.subscribe("/foot",1,&dog_robot_class::press_check,this);
            sub4=nh.subscribe("/joint_states",1,&dog_robot_class::now_angle_get,this);
            sub_imu_raw=nh.subscribe("/imu/data_raw",1,&dog_robot_class::CallbackIMU,this);
            
            joint_rad.header.stamp=ros::Time::now();
            joint_rad.name.resize(motor_number);
            joint_rad.position.resize(motor_number);
            joint_rad.velocity.resize(motor_number);

            pub_fl.header.stamp=ros::Time::now();
            pub_fl.name.resize(3);
            pub_fl.name={"front_left_link1","front_left_link2","front_left_link3"};
            pub_fl.position.resize(3);

            pub_fr.header.stamp=ros::Time::now();
            pub_fr.name.resize(3);
            pub_fr.name={"front_right_link1","front_right_link2","front_right_link3"};
            pub_fl.position.resize(3);

            pub_bl.header.stamp=ros::Time::now();
            pub_bl.name.resize(3);
            pub_bl.name={"back_left_link1","back_left_link2","back_left_link3"};
            pub_bl.position.resize(3);

            pub_br.header.stamp=ros::Time::now();
            pub_br.name.resize(3);
            pub_br.name={"back_right_link1","back_right_link2","back_right_link3"};
            pub_br.position.resize(3);
            
            pub_other.header.stamp=ros::Time::now();
            pub_other.name.resize(4);
            pub_other.name={"side_left","side_right","side_right_car","side_left_car"};
            pub_other.position.resize(4);
            pub_other.velocity.resize(4);

        //     joint_rad_fl.position.resize(4);
        //     joint_rad_fr.position.resize(4);
        //     joint_rad_bl.position.resize(4);
        //     joint_rad_br.position.resize(4);
        }
        ~dog_robot_class(){}
        //body_param(m)
        float p=3.141592653f;//円周率
        //8,10,4,7,6
        int step_num=7;//one_leg_oubを用いるときは9、kick_ground_pubを用いるときは,10.pull_push_pubは8、triangle_pubは4
        float start_theta1 = 0.0f;//肩
        float start_theta2 = 37*p/180.0f;//上関節(rad)//2.9度
        float start_theta3 = -50*p/180.0f;//下関節(rad)//29.2度
        float max_speed = 0.07f;//歩幅の最大の制限//0.14f(step_num=6)
        int time=200;//maxは100
        int time_crawl=200;
        //接地してるかしてないの閾値
        float press_th_fl=1;
        float press_th_fr=1;
        float press_th_bl=1;
        float press_th_br=1;
        //spider_modeのときのstart_position
        float sp_st_th1=0.0f;
        float sp_st_th2=135*p/180.0f;
        float sp_st_th3=-135*p/180.0f;

        float always_theta1=5/180.0f*p;//常に与える量で、常に外側+、内側-にする
        

        // float start_theta1 = 0.0f;//肩
        // float start_theta2 = 37.0f*p/180.0f;//上関節(rad)//2.9度
        // float start_theta3 = -50.0f*p/180.0f;//下関節(rad)//29.2度  


        float L1=0.04f;
        // float L2=0.18f;//足の肩から関節//実測値ではない//heightとstart_thetaを実測値と合わせるために変えている
        // float L3=0.198f;//足の関節からつま先//実測値ではない
        float L2=0.15f;//足の肩から関節//実測値ではない//heightとstart_thetaを実測値と合わせるために変えている
        float L3=0.185f;//足の関節からつま先//実測値ではない
        float b=0.05f;//足をどれだけ上げるか、初期値
        float b_b=0.05f;
        float b_under=0.0f;
        float width=0.3f;//ロボットの幅
        float length=0.16f;//ロボットの長さ
        float height=0;//基準の姿勢のときの地面から体までの高さ
        float origin_culculate=0;
        
        int count = 0;//position_keepの1回目
        int count_t = 0;//
      //基準の姿勢//rad
        
        float height_theta2 = 37.0*p/180.0f;
        float height_theta3 = -74*p/180.0f;
        
        bool spider_mode=false;//蜘蛛のように動く、モード
        bool arm_mode=false;//アームのように動かす
        bool crawl_mode=false;//クロール歩行モード
        bool kick_car_move=false;//動くか動かないかの判別、rotate_bodyのpub判別に使用
        bool kick_car_mode=false;//前後モードのSwitch
        bool car_mode=false;//車モードのSwitch
        bool robot_mode=true;//4footのSwitch
        bool robot_move=false;//4foot動きのSwitch
        bool robot_walk=false;//歩行on
        bool robot_rotate=false;//体の回転on
        bool robot_senkai=false;//旋回on
        bool end_pub=false;//pub終わる前に次の指令値を受け取らないようにする。
        bool robot_keep=false;//自動姿勢制御、このときは体の回転を操作できない。,imu使用
 
        bool crawlr_mode=false;//crwlrモードのSwitch
        //ボタン押してる間true
        bool L1_switch=false;
        bool R1_switch=false;
        bool Cross_switch=false;
        bool Square_switch=false;
        bool Circle_switch=false;
        bool L2_switch=false;
        bool R2_switch=false;
        bool SHARE_switch=false;
        bool OPTION_switch=false;
        bool PSButton_switch=false;
        bool L3_switch=false;
        bool R3_switch=false;
        bool Triangle_switch=false;

        float rotate_dir=0;
        

        float max_up = 0.05;
        float body_rotate_roll=p*10/180.0f;//体のroll回転できる最大//
        float body_rotate_pitch=p*20/180.0f;//体のpitch回転できる最大
        float senkai_theta=5/180.0f*p;//一度に旋回する角度
        float max_senkai = 10.0f/180.0f*p;//一度に旋回できる量の最大
        float now_height = 0.22;//足の付け根から接地点までのz座標の差
        float senkai_up = 0.1f;//旋回時にどれだけ足を上げるか
        float move_dir = 1.0f;        //体の回転の座標保存
        //体が傾きなが移動するには、height値を変更する必要があり、それぞれで置く
        float now_height_fl = 0;
        float now_height_fr = 0;
        float now_height_bl = 0;
        float now_height_br = 0;
        //start_theta値も変更
        float fl_sth2 = 0;
        float fl_sth3 = 0;
        float fr_sth2 = 0;
        float fr_sth3 = 0;
        float bl_sth2 = 0;
        float bl_sth3 = 0;
        float br_sth2 = 0;
        float br_sth3 = 0;


        float imu_roll_x=0;
        float imu_pitch_y=0;
        float imu_roll_x_st=0;
        float imu_pitch_y_st=0;
        float imu_roll_x_past=0;
        float imu_pitch_y_past=0;

        

        std::vector<float> transform_leg={0,0,0,0};//接地点の座標と角度を一時保存
        std::vector<float> transform_tar={0,0,0};//付け根の座標を一時保存
        
        //理想的な軌道の配列を格納
        std::vector<float> fl1={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> fl2={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> fl3={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> fr1={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> fr2={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> fr3={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> bl1={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> bl2={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> bl3={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> br1={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> br2={0,0,0,0,0,0,0,0,0,0};
        std::vector<float> br3={0,0,0,0,0,0,0,0,0,0};
        //各々のthetaの値を保存
        std::vector<float> fl_theta_list{};
        std::vector<float> fr_theta_list{};
        std::vector<float> bl_theta_list{};
        std::vector<float> br_theta_list{};
        //体の傾きのthetaの値を保存
        std::vector<float> fl_r={0,0,0,0};
        std::vector<float> fr_r={0,0,0,0};
        std::vector<float> bl_r={0,0,0,0};
        std::vector<float> br_r={0,0,0,0};
        //ダイナミクセルのid,nameを知るのに必要
        std::vector<std::string> name;
        void joint_state(int num,std::string name_joint);

        void make_orbit(float vel_x,float vel_y,float ang_x,float vel_z);//軌道を作成する、メインとなる関数
        float dir=0.0f;//進む向きをpress_yes_checkでも使いたいため
        float speed=0.0f;//歩幅をpress_yes_checkでも使いたいため
        //逆運動学、角度算出
        float inverce_kinetic_theta1(float y,float diff_y,float now);//(目標とする足の接地点y座標、アームの付け根が原点からどれだけずれてるか)
        float inverce_kinetic_theta2(float x,float y,float z,float diff_x,float diff_y,float diff_z);//(目標とする足の接地点xyz座標、アームの付け根が原点からどれだけずれてるか)
        float inverce_kinetic_theta3(float x,float y,float z,float theta2,float diff_x,float diff_y,float diff_z);//(目標とする足の接地点xyz座標、アームの付け根が原点からどれだけずれてるか)
        std::vector<float> rotate_pitch_roll(std::vector<float> transform,float pitch,float roll);//y軸周り//こっちから、次にrotate_roll
        void rotate_body(float ang_y,float ang_z,float vel_z);//体を傾ける関数
        void rotate_body_by_height(float ang_y,float ang_z);//高さによって変更する
        float by_height=0.03f;
        float max_rb_height=0.07f;//rotate_body_by_heightのmax値
        void senkai(float ang_x,float vel_z);
        float senkai_culculate(std::vector<float> transform);
        ros::NodeHandle nh;
        // ros::Publisher angle_pub = nh.advertise<sensor_msgs::JointState>("angle_pub",10);
        ///dxl_target_radはダイナミクセルのsubscribe
        ros::Publisher angle_pub = nh.advertise<sensor_msgs::JointState>("/dxl_target_rad",1);
        ros::Publisher angle_pub_fl = nh.advertise<sensor_msgs::JointState>("/dxl_target_rad_fl",10);
        ros::Publisher angle_pub_fr = nh.advertise<sensor_msgs::JointState>("/dxl_target_rad_fr",10);
        ros::Publisher angle_pub_bl = nh.advertise<sensor_msgs::JointState>("/dxl_target_rad_bl",10);
        ros::Publisher angle_pub_br = nh.advertise<sensor_msgs::JointState>("/dxl_target_rad_br",10);
        ros::Subscriber sub,sub1,sub2,sub3,sub4,sub_imu_raw;//ジョイを受け取る
        //sub1,ボタンを受け取る、L1、バツ、三角、R1を今は実行できる
        //sub2,imuを受け取る
        void button_push(const std_msgs::Float32MultiArray& msg_push);//ボタンのコールバック関数
        void angle_publish(const geometry_msgs::Twist& msg);//ジョイのコールバック関数
        sensor_msgs::JointState joint_rad;//メッセージ作成、ダイナミクセルに送るもの

        sensor_msgs::JointState joint_rad_fl;
        sensor_msgs::JointState joint_rad_fr;
        sensor_msgs::JointState joint_rad_bl;
        sensor_msgs::JointState joint_rad_br;

        void orbit_list();//軌道のvectorを作成
        void start_position();//基準姿勢に戻す
        void spot_orb_pub();//spotを模倣した動きをpubする、step_numは10
        void one_leg_pub();//一本ずつ動かす動きをpubする、step_numは9
        void pull_push_pub();//足を引いてから前に行く動きをpubする、step_numは8、onedrive/spot_moveの1-11.jpgと似た動き
        void one_point_pub();
        void add_front_pub();

        std::vector<float> fl_theta{};//pubする角度
        std::vector<float> fr_theta{};
        std::vector<float> rl_theta{};
        std::vector<float> rr_theta{};

        std::vector<float> fl_start={width/2.0f,1.0f*length/2.0f,0.0f};
        std::vector<float> fr_start={-1.0f*width/2.0f,1.0f*length/2.0f,0.0f};
        std::vector<float> bl_start={width/2.0f,-1.0f*length/2.0f,0.0f};
        std::vector<float> br_start={-1.0f*width/2.0f,-1.0f*length/2.0f,0.0f};
        
        std::vector<std::vector<float>> each_make_orb(std::vector<std::vector<float>> orbit_magnitude,float each_h,float a,float diff);//高さ、軌道から各々の軌道を作成する関数
        std::vector<float> make_theta(std::vector<std::vector<float>> orb_mag_rotate,float each_h,float judge); //pubする角度をvectorにいれる関数

        void value_reset();//fl_r,fl1などの値をリセットする
        void position_keep(const geometry_msgs::Vector3& msg);

        //クローラモード
        float c_st_th1=0.0f;
        float c_st_th2=0.0f;
        float c_st_th3=1.57f;
        std::vector<float> change_mode{};
        std::vector<float> change_mode_velocity{};
        void start_crawler();
        void crwlr_move(float ang_y,float ang_z);//前進、後進、旋回を作成、LStick
        void leg_height_th2(float ang_x,int leg);//フリッパの高さ調整、ボタン＋RStick
        void leg_height_th3(float vel_z,int leg);
        float max_crwlr_speed=7.0f;
        float fl_crwlr_th2=0;//フリッパ角度
        float fr_crwlr_th2=0;//フリッパ角度
        float bl_crwlr_th2=0;//フリッパ角度
        float br_crwlr_th2=0;//フリッパ角度
        float fl_crwlr_th3=0;//フリッパ角度
        float fr_crwlr_th3=0;//フリッパ角度
        float bl_crwlr_th3=0;//フリッパ角度
        float br_crwlr_th3=0;//フリッパ角度
        float flipper_rotate_speed=0.5f;//フリッパ一度に回転させる量
        float min_crwlr_th3=-2.18f;//th3の最小
        float min_crwlr_th2=-1.36f;//th2の最小
        float max_crwlr_th3=2.67f;//th3の最大
        float max_crwlr_th2=1.71f;//th2の最大
        bool leg_th3_move=false;//th3だけ動かすフリッパ
        bool leg_all_move=false;//th2,th3を同時に動かすフリッパ、地面に常に平行
        void crwlr_move_pub(float left_speed,float right_speed);
        void crwlr_flipper_pub();
        void crwlr_value_reset();

        void kick_ground_pub();//地面
        void stop_roll();//転ばないようにする

        float threshold_roll=20.0f/180*p;//imuがこの角度以上の値になったときに起動し、脚を動かし立て直す
        

        float cross_height=0.0f;

        void kick_car(float vel_x,float vel_y,float ang_y,float ang_z);
        void kick_car_pub();
        void kick_value_reset();
        void start_kick_car();
        std::vector<float> change_mode_kcf{};
        std::vector<float> change_mode_kcb{};
        //kick_car時の初期姿勢
        float kcf_st_th1=0.0f;
        float kcf_st_th2=3.14f/4.0f;
        float kcf_st_th3=-3.14f/2.0f;
        float kcb_st_th1=0.0f;
        float kcb_st_th2=3.14f/4.0f;
        float kcb_st_th3=-3.14f/2.0f;

        void triangle_pub();//４点で動かす
//脚をアームのように動かす
        void arm_move(float ang_z,float vel_y,float ang_x);
        float arm_theta1=0.0f;
        float max_x=L2+L3;
        float max_z=L2+L3;
        float arm_x=0;
        float arm_y=0;
        float arm_z=0;
//蜘蛛のような動かし方
        void start_spider();
        void spider_orb(float vel_x,float vel_y,float ang_y,float ang_z);
        void spider_pub();
        void spider_value_reset();      

        void press_check(const std_msgs::Float32MultiArray& foot_msg);//接地を確認
        int press_fl=9;//fl接地していれば0
        int press_fr=6;//fr接地していれば3
        int press_bl=3;//bl接地していれば6
        int press_br=0;//br接地していれば9
        std::vector<float> press_value={0,0,0,0};
        float th_press=0.5f;
        int now_max_index=0;//一番圧力がかかってるところ
        float press_height_fl=0;//rotate_bodyに送る値
        float press_height_fr=0;//rotate_bodyに送る値
        float press_height_bl=0;//rotate_bodyに送る値
        float press_height_br=0;//rotate_bodyに送る値
        float press_roll=0;//rotate_bodyに送る値
        int press_controll=20;//重心を真ん中にしようとする回数
        float press_height_change=0.005f;//一度にpitch回転する量//この値×body_rotate_pitch度
        float press_roll_rad=0.005f;//一度にroll回転する量//この値×body_rotate_roll度
        void controll_gravity();//圧力センサを全部同じくらいの値になるように体を傾ける
        int press_true_count=0;
        bool gp_center=false;//true or falseでpress_checkを行うかどうか
        void now_angle_get(const sensor_msgs::JointState& now_angle);//現在の角度を保存し、更新
        //現在の角度を保存
        float now_fl_th1=0;
        float now_fl_th2=0;
        float now_fl_th3=0;

        float now_fr_th1=0;
        float now_fr_th2=0;
        float now_fr_th3=0;

        float now_bl_th1=0;
        float now_bl_th2=0;
        float now_bl_th3=0;

        float now_br_th1=0;
        float now_br_th2=0;
        float now_br_th3=0;

        void crawl_walk(float vel_x,float vel_y,float ang_x,float vel_z);
        void crawl_walk_pub();
        void crawl_walk_rotate_pub();
        float cwalk_rotate=0.4f;
        float max_crawl_speed=0.1f;

        void crawl_walk_2(float vel_x,float vel_y,float ang_x,float vel_z);
        void crawl_walk_pub_2();
        void crawl_walk_pub_7();//step_num=7のクロール歩行

        double roll=0;//rad
        double pitch=0;//rad
        double before_pitch=0;
        double before_roll=0;

        void CallbackIMU(const sensor_msgs::Imu& imu_msgs){
                float low_pass_ratio=0.01;
                pitch=imu_msgs.linear_acceleration.y*low_pass_ratio+before_pitch*(1-low_pass_ratio);
                roll=imu_msgs.linear_acceleration.x*low_pass_ratio+before_roll*(1-low_pass_ratio);

                before_pitch=pitch;
                before_roll=roll;
                bool debug=false;
                if(debug){
                        ROS_INFO("pitch =%lf ,roll=%lf",pitch*180/M_PI,roll*180/M_PI);
                }
        }
        
        void press_no_check(int leg_num,float pm_th2,float pm_th3);//接地してないときに起動//plus_minus flが接地してない場合th2+,th3-、fr==th2-,th3+、bl==th2-,th3+、br==th2+,th3-
        void press_yes_check(int leg_num);//接地したら起動
        std::vector<float> st_th1={start_theta1,start_theta1,start_theta1,start_theta1};
        std::vector<float> st_th2={start_theta2,start_theta2,start_theta2,start_theta2};
        std::vector<float> st_th3={start_theta3,start_theta3,start_theta3,start_theta3};

        void culculate_height_origin_(float leg_theta1,float leg_theta2,float theta3);
        std::vector<float> height_origin={0,0};

        //5,6脚目のタイヤのspeed
        float car_left_speed=0.0f;
        float car_right_speed=0.0f;
        float car_speed=1.0f;
        float right_updown=0.0f;//thetaを上げ下げする、十字で常にできるようにする
        float left_updown=0.0f;//
        float side_leg_add=30/180.0f*p;
        void side_leg_updown(float vel_z);

        float side_width=0.75f;
        float gear_r=0.045f;
        
        void crawl_walk_pub_8();
        void crawl_walk_pub_9();

        void one_side_pub();//片サイドずつpub
        float one_side_under=0/-180.0f;

        void zero_velocity();

        float body_height=0.0f;
        void pub(sensor_msgs::JointState pub_msg);
        sensor_msgs::JointState pub_fl,pub_fr,pub_bl,pub_br,pub_other;
};

//cross、歩きをONにする
//LStick 前進、後退
//RStick 体の傾きを操作
//Cross key 旋回
//L2 押し具合で足を上げる量変化

//linear.x LStickの左右
//linear.y LStickの上下
//linear.z 十字キー上
//ang.x　十字キー左右
//ang.y　RStickの左右
//ang.z RStickの上下