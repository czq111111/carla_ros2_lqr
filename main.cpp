#include "lqr_carla_control.h"


double ControlNode::PointDistance(const plan_end_info &point, const double x, const double y) {
    double dx = point.plan_end_x - x;
    double dy = point.plan_end_y - y;
    // cout<<"--终点X "<<point.plan_end_x<<endl;
    // cout<<"--终点Y "<<point.plan_end_y<<endl;
    return dx * dx + dy * dy;
}


void ControlNode::OdomCallback(nav_msgs::msg::Odometry::SharedPtr msg){
    if(!first_record){
        first_record = true;
    }
    vehicle_state.vx = msg->twist.twist.linear.x;
    vehicle_state.vy = msg->twist.twist.linear.y;
    tf2::Quaternion quat_tf;
    tf2::convert(msg->pose.pose.orientation, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(vehicle_state.roll, vehicle_state.pitch, vehicle_state.yaw);
    vehicle_state.heading = vehicle_state.yaw;
    vehicle_state.x = msg->pose.pose.position.x;
    vehicle_state.y = msg->pose.pose.position.y;
    vehicle_state.velocity = std::sqrt(std::pow(vehicle_state.vx, 2) + std::pow(vehicle_state.vy, 2));
    vehicle_state.angular_velocity = std::sqrt(std::pow(msg->twist.twist.angular.x, 2) + std::pow(msg->twist.twist.angular.y, 2));
    vehicle_state.acceleration = 0.0;
    // std::ofstream accCurve;
    // accCurve.open("/home/yj/le_materials/CARLA_demo/src/lqr_control/data/path_carla_town04.txt", std::ios::app);
    // accCurve << vehicle_state.x << " " << vehicle_state.y << "\n";
    // accCurve.close();
    //将信息转成自己编写的结构，方便传输数据
    ego_info.host_x = vehicle_state.x;
    ego_info.host_y = vehicle_state.y;
    ego_info.host_heading_xy = vehicle_state.heading;
    ego_info.host_speed = vehicle_state.velocity;
    ego_info.host_vx = vehicle_state.vx;
    ego_info.host_vy = vehicle_state.vy;
    ego_info.host_yawrate = vehicle_state.angular_velocity;
    ego_info.host_acceleration = 0.0;
    ego_info.host_ax = 0;
    ego_info.host_ay = 0;
}

void ControlNode::timer1_callback()
{
	static int i = 0;
	auto message = std_msgs::msg::String();
	i++;
	//250ms中断四次加一秒
	if(i>=4){
		i=0;
		tim1[2]++;
		if(tim1[2]>=60){
			tim1[2]=0;
			tim1[1]++;
			if(tim1[1]>=60){
				tim1[1]=0;
				tim1[0]++;
				if(tim1[0]>=24)tim1[0]=0;
			}
		}
	}else{

	}
	ss.str("");
	ss.clear();
	ss<<"timer1   - "<<tim1[0]<<":"<<tim1[1]<<":"<<tim1[2];
	message.data = ss.str();
	//pub
	pub1->publish(message);
}
//timer2回调处理函数
//功能：获取系统时间并将其打印到终端。
void ControlNode::timer2_callback()
{
	//取得系统时间
	time_t tt;
	time( &tt );
	tt = tt + 8*3600;  // transform the time zone
	tm* t= gmtime( &tt );
	//打印到终端上
	cout<<"current time   - "<<t->tm_hour<<":"<<t->tm_min<<":"<<t->tm_sec<<endl;
}


void ControlNode::SmoothCallback(){
    //**********************把文件在的点坐标xy_points1转换到frenet坐标系下***************************//
    MatchingToFrenet fre;
    fre.pre_match_point_index_set.clear();
    std::vector<std::pair<double, double>> xy_set;
    //xy_set.resize(1);
    xy_set.clear();
    //cout<<"ego_info.host_x: "<<ego_info.host_x<<" "<<"ego_info.host_y: "<<ego_info.host_y<<endl;
    xy_set.push_back(std::make_pair(ego_info.host_x,ego_info.host_y));
    //cout<<"xy_set.size()"<<xy_set.size()<<endl;
    //cout<<"ego_info.host_x: "<<ego_info.host_x<<" "<<"ego_info.host_y: "<<ego_info.host_y<<endl;
    fre.tofrenetreferenceLine(xy_points,xy_set);
    fre.tofrenet(&headings,&kappas);
//**************************提取refercenceline未平滑的初值*************************************//
#if 1
    ExtractPoint ext;
    int host_match_point_index;
    host_match_point_index = fre.pre_match_point_index_set.at(0);
    //cout<<"host_match_point_index"<<host_match_point_index<<endl;
    ext.axtractglobalpath(xy_points);
    ext.referenceline_x_init.clear();
    ext.referenceline_y_init.clear();
    ext.axtractrefercenceline(host_match_point_index);
#endif
//*****************************************************************************************
//**********************把文件在的点坐标xy_points1进行曲线平滑处理*******************************//
    std::vector<double> x4, y4;
    x4.resize(ext.referenceline_x_init.size());
    y4.resize(ext.referenceline_y_init.size());
    x4.clear();
    y4.clear();
    for (int a = 0; a < (int)ext.referenceline_x_init.size(); a++) {
        x4.push_back(ext.referenceline_x_init.at(a));
        y4.push_back(ext.referenceline_y_init.at(a));
        //cout<<"ext.referenceline_x_init:  "<< ext.referenceline_x_init[a]<<endl;
    }
    std::vector<std::pair<double, double>> refercence_point;
    refercence_point.clear();//清理之前的数据
    for(int j=0;j<(int)ext.referenceline_x_init.size();j++)
    {
        refercence_point.push_back(std::make_pair(x4.at(j), y4.at(j)));
    }
    /////////////////////////////平滑////////////////////////////////////
    // refercenceline_smooth test;
    // test.ReferenceLine(refercence_point);
    // test.ref_x.clear();
    // test.ref_y.clear();
    // test.line_smooth(100, 50, 3, 0.2, 0.2, 0.2, 0.2);
    // //平滑的曲线
    // //std::vector<double> x_smooth, y_smooth;
    // x_smooth.resize(test.ref_x.size());
    // y_smooth.resize(test.ref_y.size());
    // cout<<"test.ref_x.size()"<<test.ref_x.size()<<endl;
    // // x_smooth.clear();
    // // y_smooth.clear();
    // for(int j=0;j<(int)ext.referenceline_x_init.size();j++)
    // {
    //     x_smooth.push_back(test.ref_x.at(j));
    //     y_smooth.push_back(test.ref_y.at(j));
    //     //cout<<"x_smooth: "<<x_smooth[j]<<endl;
    // }
    // //std::vector<std::pair<double, double>> smooth_points;
    // smooth_points.clear();
    // for(int j=0;j<(int)ext.referenceline_x_init.size();j++)
    // {
    //     smooth_points.push_back(std::make_pair(x_smooth.at(j), y_smooth.at(j)));
    // }
    std::vector<double> smooth_points_headings, smooth_points_accumulated, smooth_points_kappas, smooth_points_dkappas;
    std::unique_ptr<ReferenceLine> smooth_line = std::make_unique<ReferenceLine>(refercence_point);//smooth_points//refercence_point
    //访问计算出来的headings,kappas并保存到common.h的结构体中
    smooth_points_headings.clear();
    smooth_points_accumulated.clear();
    smooth_points_kappas.clear();
    smooth_points_dkappas.clear();
    smooth_line->ComputePathProfile(&smooth_points_headings, &smooth_points_accumulated, &smooth_points_kappas,
                                     &smooth_points_dkappas);
//*********************************************************************
#if 0
    //跟踪全局路径
    Trajectory_.resize(xy_points.size());//x_smooth
    for (int a = 0; a < (int)xy_points.size(); a++)//smooth_points
    {
        Trajectory_[a].trajectory_x = xy_points.at(a).first;
        Trajectory_[a].trajectory_y = xy_points.at(a).second;
        Trajectory_[a].trajectory_heading = headings[a];
        Trajectory_[a].trajectory_kappa = kappas[a];
        Trajectory_[a].trajectory_accel=0.0;
        Trajectory_[a].trajectory_speed=0.0;
        Trajectory_[a].trajectory_time=1.0;
    }
#endif
#if 1
    //跟踪提取线
    Trajectory_.resize(refercence_point.size());//x_smooth
    for (int a = 0; a < (int)refercence_point.size(); a++)//smooth_points
    {
        Trajectory_[a].trajectory_x = x4[a];//x4[a];//x_smooth[a];//xy_points.at(a).first;//x_smooth[a];
        Trajectory_[a].trajectory_y = y4[a];//y4[a];//y_smooth[a];//xy_points.at(a).second;//y_smooth[a];
        Trajectory_[a].trajectory_heading = smooth_points_headings[a];//smooth_points_headings[a];//headings[a];
        Trajectory_[a].trajectory_kappa = smooth_points_kappas[a];//smooth_points_kappas[a];//kappas[a];
        Trajectory_[a].trajectory_accel=0.0;
        Trajectory_[a].trajectory_speed=0.0;
        Trajectory_[a].trajectory_time=1.0;
    }
#endif
#if 0
    Trajectory_.resize(smooth_points.size());//x_smooth//refercence_point
    for (int a = 0; a < (int)smooth_points.size(); a++)//smooth_points//refercence_point
    {
        Trajectory_[a].trajectory_x = x_smooth[a];//test.ref_x.at(a);
        Trajectory_[a].trajectory_y = y_smooth[a];//test.ref_y.at(a);
        Trajectory_[a].trajectory_heading = smooth_points_headings[a];//smooth_points_headings[a];
        Trajectory_[a].trajectory_kappa = smooth_points_kappas[a];//smooth_points_kappas[a];
        Trajectory_[a].trajectory_accel=0.0;
        Trajectory_[a].trajectory_speed=0.0;
        Trajectory_[a].trajectory_time=1.0;
        // cout<<"smooth_points_headings: "<< smooth_points_headings[a]<<endl;
        // cout<<"Trajectory_[a].trajectory_x = " <<Trajectory_[a].trajectory_x<<endl;
        // cout<<"Trajectory_[a].trajectory_y = " <<Trajectory_[a].trajectory_y<<endl;
    }
#endif
    //画图
    //获取全局路径
    std::vector<double> x2, y2;
    //**********************全局路径***********************//
    x2.resize(xy_points.size());                          //
    y2.resize(xy_points.size());                          //
    x2.clear();                                           //
    y2.clear();                                           //
    for (int j = 0; j < (int)xy_points.size(); j++) {     //
        x2.push_back(xy_points.at(j).first);              //
        y2.push_back(xy_points.at(j).second);             //
    }

    //自身车辆运动轨迹
    x0.push_back(ego_info.host_x);
    y0.push_back(ego_info.host_y);

    plt::cla();                                           
    plt::named_plot("globel_line",x2, y2,"-.r");//全局路径 
    plt::named_plot("referenceline",x4, y4,"k-");//提取
    plt::named_plot("car_position",x0,y0,"-.bo");//自身位置
    plt::title("Map");
    plt::legend();
    plt::pause(0.001);
    //设置终点
    plan_end_info endpoint;
    endpoint.plan_end_x = xy_points.at(3038).first;
    endpoint.plan_end_y = xy_points.at(3038).second;
    double d_temp = PointDistance(endpoint, ego_info.host_x, ego_info.host_y);
    //**********************************************************************//
    if(d_temp<1)
    {
        //cout<<"终点X "<<xy_points.at(3038).first<<endl;
        //cout<<"终点Y "<<xy_points.at(3038).second;
        //cout<<"到终点的距离！ "<< d_temp<<endl;
        cout<<"已经到达终点！ "<<endl;
        plt::show();
    }  
}

void ControlNode::VehicleControllerIterationCallback() {
    if (this->first_record)
    {
        // double steer = lqr_controller_lateral->ComputeControlCommand(this->ego_info, this->Trajectory_);
        double steer = sl_lqr_controller_lateral->ComputeControlCommand(this->ego_info, this->Trajectory_);
        if(steer<-1 || steer >1){
            steer = 0;
        }
        control_cmd.throttle = 0.5;
        control_cmd.steer = steer;
        // control_cmd.brake = 0;
        // control_cmd.hand_brake = 0;
        // control_cmd.reverse = 0;
        // control_cmd.gear = 1;
        // control_cmd.manual_gear_shift = 0;
        vehicle_control_publisher->publish(control_cmd);
    }
}

ControlNode::ControlNode() : Node("lqr_vehicle")
{
    //发布者发布“time1”话题
	pub1= this->create_publisher<std_msgs::msg::String>("time1",10);
    //时间及处理
	time_t tt;
    time( &tt );
    tt = tt + 8*3600;  // transform the time zone
    tm* t= gmtime( &tt );
	ss<<"current time   - "<<t->tm_hour<<":"<<t->tm_min<<":"<<t->tm_sec;
	cout<< ss.str() <<endl;
	ss.str("");
	ss.clear();
	//存储开始时间
	tim1[0]=t->tm_hour;
	tim1[1]=t->tm_min;
	tim1[2]=t->tm_sec;


    first_record = false;
    // v_set = 10;   //设置车辆速度m/s
    // wheelbase = 2.875;  //轴距
    //*******************************读取参考线文件里的数据******************************************//
    //std::vector<std::pair<double, double>> xy_points;
    auto data = std::make_unique<readDate>();
    std::vector<std::pair<double, double>> xy_points1;
    data->readTxt(xy_points1);
    xy_points.clear();
    for(int i=0;i < (int)xy_points1.size();i++)
    {
        xy_points.push_back(std::make_pair(xy_points1[i].first, xy_points1[i].second));
    }
    //******************************计算heading和kappa*******************************************//
    //std::vector<double> headings, accumulated_s, kappas, dkappas;
    std::unique_ptr<ReferenceLine> reference_line = std::make_unique<ReferenceLine>(xy_points);
    //访问计算出来的headings,kappas并保存到common.h的结构体中
    headings.clear();
    accumulated_s.clear();
    kappas.clear();
    dkappas.clear();
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas,
                                     &dkappas);
//////////////////////////////////////////////////////////////////////////////////////////////////////////
    //lqr_controller_lateral = std::make_unique<LQR_control>();
    sl_lqr_controller_lateral = std::make_unique<lqr_control>();
    // 创建车辆状态信息的订阅者
    localization_data_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&ControlNode::OdomCallback, this, _1));
    // 创建车辆控制信息的发布者
    //global_path_publish_timer = this->create_wall_timer(500ms, std::bind(&ControlNode::GlobalPathPublishCallback, this));
    vehicle_control_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 1000);
    
    
    //定时器timer1初始化,250ms中断，回调函数timer1_callback
    // timer1 = this->create_wall_timer(250ms,std::bind(&ControlNode::timer1_callback),this);
	// //定时器timer2初始化,100ms中断，回调函数timer2_callback
    // timer2 = this->create_wall_timer(100ms,std::bind(&ControlNode::timer2_callback),this);
    timer1 = this->create_wall_timer(250ms,[this]() -> void { timer1_callback(); });
    timer2 = this->create_wall_timer(100ms,[this]() -> void { timer2_callback(); });
    
    // 创建定时器,每5000ms发布一个消息
    smooth_timer = this->create_wall_timer(100ms, std::bind(&ControlNode::SmoothCallback, this));
    vehicle_control_iteration_timer = this->create_wall_timer(10ms, std::bind(&ControlNode::VehicleControllerIterationCallback, this));

    

}

ControlNode::~ControlNode(){}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto n = std::make_shared<ControlNode>();
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}