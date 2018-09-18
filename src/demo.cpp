#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Useful customized headers
    #include "backward.hpp"
    #include "time_optimizer.h"
    #include "trajectory_generator_waypoint.h"
    #include "json.hpp"

// The trajectory class the demo use
    #include "traj_poly_mono.h"

using namespace std;
using namespace Eigen;
using json = nlohmann::json;

namespace backward {
    backward::SignalHandling sh;
}

// Param from launch file
    double _vis_traj_width;
    double _MAX_Vel, _MAX_Acc, _MAX_d_Acc, _d_s;
    double _start_x, _start_y, _start_z;
    int _dev_order, _min_order, _poly_num1D;
    bool _is_dump_data, _is_use_inte;
    string _pkg_path;

// ros related
    ros::Subscriber _way_pts_sub;
    ros::Publisher  _wp_traj_vis_pub, _wp_path_vis_pub;
    ros::Publisher  _vis_pos_pub, _vis_vel_pub, _vis_acc_pub;

// **** global variable *** //
// for planning
    MatrixXd _polyCoeff;
    VectorXd _polyTime;
    int _segment_num;
    ros::Time _traj_time_start, _traj_time_final;
    bool _has_traj = false;
// for visualization
    visualization_msgs::Marker _vis_pos, _vis_vel, _vis_acc;

// declare
    Allocator * _time_allocator = new Allocator();
    void visWayPointTraj( MatrixXd polyCoeff, VectorXd time);
    void visWayPointPath(MatrixXd path);

    Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t );
    Vector3d getVelPoly( MatrixXd polyCoeff, int k, double t );
    Vector3d getAccPoly( MatrixXd polyCoeff, int k, double t );
    void trajGeneration(Eigen::MatrixXd path);
    void rcvWaypointsCallback(const nav_msgs::Path & wp);

ofstream pos_result, vel_result, acc_result, t_result;
#if 1
void pubCmd()
{   
    if( _has_traj == false || ros::Time::now() > _traj_time_final )
    {   
        return;
    }
    else
    { 
        //publish position, velocity and acceleration command according to time bias
        double t = max(0.0, (ros::Time::now() - _traj_time_start).toSec() );
        
        if(_is_dump_data) t_result<<t<<endl;

        MatrixXd time     = _time_allocator->time;
        MatrixXd time_acc = _time_allocator->time_acc;

        int idx;
        for(idx = 0; idx < _segment_num; idx++)
        {   
            int K = _time_allocator->K(idx);
            if( t  > time(idx, K - 1))
                t -= time(idx, K - 1);
            else
                break;
        }
        double t_tmp = t;     

        int grid_num = _time_allocator->K(idx);
        
        // now we need to find which grid the time instance belongs to
        int grid_idx;
        for(grid_idx = 0; grid_idx < _time_allocator->K(idx); grid_idx++){
            if (t > time(idx, grid_idx)) continue;
            else{ 
                if(grid_idx > 0) t -= time(idx, grid_idx - 1);
                else             t -= 0.0;
                break;
            }
        }
        
        double delta_t;
        if(grid_idx > 0)
          delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
        else
          delta_t = time(idx, grid_idx) - 0.0;
        
        double delta_s = t * _time_allocator->s_step / delta_t;
        double s = _time_allocator->s(idx, grid_idx) + delta_s;

        // get position data 
        Vector3d position_s = getPosPoly(_polyCoeff, idx, s); 
        Vector3d position   = position_s;

        // get velocity data
        double s_k   = _time_allocator->s(idx, grid_idx);
        double s_k_1 = _time_allocator->s(idx, grid_idx + 1);
        double b_k   = _time_allocator->b(idx, grid_idx);
        double b_k_1 = _time_allocator->b(idx, grid_idx + 1);

        Vector3d velocity_s1 = getVelPoly(_polyCoeff, idx, s_k  ); 
        Vector3d velocity_s2 = getVelPoly(_polyCoeff, idx, s_k_1);

        Vector3d velocity1   = velocity_s1 * sqrt(b_k);
        Vector3d velocity2   = velocity_s2 * sqrt(b_k_1);
        Vector3d velocity   = velocity1 + (velocity2 - velocity1) * t / delta_t;

// ### NOTE: From what above we get the position and velocity easily.
// ###       positions are the same as the trajectory before re-timing; and velocity are obtained by interpolation between each grid.
// ###       In what follows, we will get the accleration. It's more complicated since each acceleration ais evaluated at the middle of a grid        
        // reset grid_idx and t for time acceleration axis
        t = t_tmp;
        for(grid_idx = 0; grid_idx < _time_allocator->K(idx); grid_idx++)
        {
            if (t > time_acc(idx, grid_idx)) continue;
            else{ 
                if(grid_idx > 0) t -= time_acc(idx, grid_idx - 1);
                else             t -= 0.0;
                break;
            }
        }
        
        if(grid_idx == grid_num)
            t -= time_acc(idx, grid_num - 1);

        // prepare to do accleration interpolation
        Vector3d velocity_s, acceleration_s, acceleration1, acceleration2;
        Vector3d acceleration;

        double a_k;
        // # special case 1: the very first grid of all segments of the trajectory, do interpolation in one grid
        if( grid_idx == 0 && idx == 0 ) 
        {   
            s_k   = _time_allocator->s(idx, 0);
            s_k_1 = _time_allocator->s(idx, 0 + 1);
            
            a_k   = _time_allocator->a(idx, 0);
            b_k   = _time_allocator->b(idx, 0);
            b_k_1 = _time_allocator->b(idx, 0 + 1);

            velocity_s     = getVelPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration1 << 0.0, 0.0, 0.0;
            
            acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / time_acc(0, 0); 
        }
        // # special case 2: the very last grid of all segments of the trajectory, do interpolation in one grid
        else if( grid_idx == grid_num && idx == (_segment_num - 1) )
        {   
            s_k   = _time_allocator->s(idx, grid_num - 1);
            s_k_1 = _time_allocator->s(idx, grid_num);
            
            a_k   = _time_allocator->a(idx, grid_num - 1);
            b_k   = _time_allocator->b(idx, grid_num - 1);
            b_k_1 = _time_allocator->b(idx, grid_num    );

            velocity_s     = getVelPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
            acceleration_s = getAccPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
            acceleration = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
        }
        // # regular case: do interpolation between two grids
        else 
        {   
            // sub-case 1: two grids are in the same segment
            if(grid_idx < grid_num && grid_idx > 0) // take average accleration in a same segment
            {   
                delta_t = (time_acc(idx, grid_idx) - time_acc(idx, grid_idx - 1));
                
                s_k   = _time_allocator->s(idx, grid_idx - 1);
                s_k_1 = _time_allocator->s(idx, grid_idx + 0);
                
                a_k   = _time_allocator->a(idx, grid_idx - 1);
                b_k   = _time_allocator->b(idx, grid_idx - 1);
                b_k_1 = _time_allocator->b(idx, grid_idx + 0);

                velocity_s     = getVelPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration_s = getAccPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k   = _time_allocator->s(idx, grid_idx + 0);
                s_k_1 = _time_allocator->s(idx, grid_idx + 1);

                a_k   = _time_allocator->a(idx, grid_idx + 0);
                b_k   = _time_allocator->b(idx, grid_idx + 0);
                b_k_1 = _time_allocator->b(idx, grid_idx + 1);              

                velocity_s     = getVelPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration_s = getAccPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;   
            }
            // sub-case 2: two grids are in consecutive segment, the current grid is in a segment's tail
            else if(grid_idx == grid_num)// take average accleration between two segments
            {   
                delta_t = (time(idx, grid_num - 1) - time_acc(idx, grid_num - 1) + time_acc(idx + 1, 0) );
                
                s_k   = _time_allocator->s(idx, grid_idx - 1);
                s_k_1 = _time_allocator->s(idx, grid_idx);
                
                a_k   = _time_allocator->a(idx, grid_idx - 1);
                b_k   = _time_allocator->b(idx, grid_idx - 1);
                b_k_1 = _time_allocator->b(idx, grid_idx);

                velocity_s     = getVelPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration_s = getAccPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                s_k   = _time_allocator->s(idx + 1, 0);
                s_k_1 = _time_allocator->s(idx + 1, 1);

                a_k   = _time_allocator->a(idx + 1, 0);
                b_k   = _time_allocator->b(idx + 1, 0);
                b_k_1 = _time_allocator->b(idx + 1, 1);              

                velocity_s     = getVelPoly(_polyCoeff, idx + 1, (s_k + s_k_1 ) / 2.0);
                acceleration_s = getAccPoly(_polyCoeff, idx + 1, (s_k + s_k_1 ) / 2.0);
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration  = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;        
            }
            // sub-case 3: two grids are in consecutive segment, the current grid is in a segment's head
            else if(grid_idx == 0)// take average accleration between two segments
            {   
                int grid_num_k = _time_allocator->K(idx - 1);
                delta_t = (time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1) + time_acc(idx, 0) );
                
                s_k   = _time_allocator->s(idx - 1, grid_num_k - 1);
                s_k_1 = _time_allocator->s(idx - 1, grid_num_k    );
                
                a_k   = _time_allocator->a(idx - 1, grid_num_k - 1);
                b_k   = _time_allocator->b(idx - 1, grid_num_k - 1);
                b_k_1 = _time_allocator->b(idx - 1, grid_num_k    );

                velocity_s     = getVelPoly(_polyCoeff, idx - 1, (s_k + s_k_1 ) / 2.0);
                acceleration_s = getAccPoly(_polyCoeff, idx - 1, (s_k + s_k_1 ) / 2.0);
                acceleration1  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k   = _time_allocator->s(idx, 0);
                s_k_1 = _time_allocator->s(idx, 0 + 1);
                
                a_k   = _time_allocator->a(idx, 0);
                b_k   = _time_allocator->b(idx, 0);
                b_k_1 = _time_allocator->b(idx, 0 + 1);

                velocity_s     = getVelPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration_s = getAccPoly(_polyCoeff, idx, (s_k + s_k_1 ) / 2.0);
                acceleration2  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * (t + time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1)) / delta_t;   
            } 
            else {
                // no else
            }
        }

        // dump data in 3 files
        if(_is_dump_data)
        {
            pos_result << position(0)     <<","<< position(1)     <<","<< position(2)    << endl;
            vel_result << velocity(0)     <<","<< velocity(1)     <<","<< velocity(2)    << endl;
            acc_result << acceleration(0) <<","<< acceleration(1) <<","<< acceleration(2)<< endl;
        }

        _vis_pos.header.stamp = ros::Time::now();
        _vis_vel.header.stamp = ros::Time::now();
        _vis_acc.header.stamp = ros::Time::now();
        _vis_pos.points.clear();
        _vis_vel.points.clear();
        _vis_acc.points.clear();

        geometry_msgs::Point pt;
        pt.x = position(0);
        pt.y = position(1);
        pt.z = position(2);

        _vis_pos.points.push_back(pt);
        _vis_vel.points.push_back(pt);
        _vis_acc.points.push_back(pt);
        
        pt.x = position(0) + velocity(0);
        pt.y = position(1) + velocity(1);
        pt.z = position(2) + velocity(2);

        _vis_vel.points.push_back(pt);

        pt.x = position(0) + acceleration(0);
        pt.y = position(1) + acceleration(1);
        pt.z = position(2) + acceleration(2);

        _vis_acc.points.push_back(pt);
        
        _vis_pos_pub.publish(_vis_pos);
        _vis_vel_pub.publish(_vis_vel);
        _vis_acc_pub.publish(_vis_acc);
    }
}

#else
void pubCmd()
{
    if(_has_traj == false)
        return;
    else if( ros::Time::now() > _traj_time_final)
        return;

    double t = max(0.0, ( ros::Time::now() - _traj_time_start ).toSec() );
    cout<<"t: "<<t<<endl;

    Vector3d pos, vel, acc;
    for (int idx = 0; idx < _polyCoeff.rows(); ++idx)
    {
        if (t > _polyTime[idx] && idx + 1 < _polyCoeff.rows())
        {
            t -= _polyTime[idx];
        }
        else
        {   
            pos = getPosPoly(_polyCoeff, idx, t);
            vel = getVelPoly(_polyCoeff, idx, t);
            acc = getAccPoly(_polyCoeff, idx, t);

            pos_result << pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
            vel_result << vel(0)<<","<<vel(1)<<","<<vel(2)<<endl;
            acc_result << acc(0)<<","<<acc(1)<<","<<acc(2)<<endl;
            break;
        } 
    }
}
#endif

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if(wp.poses[k].pose.position.z < 0.0)
            break;
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) << _start_x, _start_y, _start_z;
    
    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    trajGeneration(waypoints);
}

VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);

    for (int k = 0; k < (Path.rows() - 1); k++)
    {
        double dtxyz;

        Vector3d p0   = Path.row(k);        
        Vector3d p1   = Path.row(k + 1);    
        double D    = (p1 - p0).norm();             

        double acct = (_MAX_Vel) / _MAX_Acc;
        double accd = (_MAX_Acc * acct * acct / 2);
        double dcct = _MAX_Vel / _MAX_Acc;                                  
        double dccd = _MAX_Acc * dcct * dcct / 2;                           

        if (D < accd + dccd)
        {   
            double t1 = sqrt( _MAX_Acc * D ) / _MAX_Acc;
            double t2 = (_MAX_Acc * t1) / _MAX_Acc;
            dtxyz     = t1 + t2;    
        }
        else
        {                                        
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _MAX_Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
        }

        time(k) = dtxyz;
    }

    return time;
}

VectorXd timeAllocationNaive(MatrixXd Path)
{   
    VectorXd time(Path.rows() - 1);

    for(int i = 0; i < (Path.rows() - 1); i++)
        time(i) = 1.0;

    return time;
}

void trajGeneration(Eigen::MatrixXd path)
{   
    TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
    MinimumTimeOptimizer time_optimizer;
    
    MatrixXd vel = MatrixXd::Zero(2,3); 
    MatrixXd acc = MatrixXd::Zero(2,3);

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    _polyTime  = timeAllocation(path); //_polyTime  = timeAllocationNaive(path); 

    // generate a minimum-jerk piecewise monomial polynomial-based trajectory
    ros::Time time_1 = ros::Time::now();
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);   
    ros::Time time_2 = ros::Time::now();
    _segment_num = _polyCoeff.rows();

    // visualize the spatial fixed trajectory
    visWayPointPath(path);
    visWayPointTraj( _polyCoeff, _polyTime);

    ROS_WARN("[TimeOptimizer DEMO] Spatial trajectory generated");
    cout<<"[TimeOptimizer DEMO] time cunsume in spatial trajectory is: "<<(time_2 - time_1).toSec()<<endl;
    // use this structure to evaluate this monomial polynomial trajectory in the time optimizer; 
    // or your OWN implementation inheriting the base class for other types of piecewise trajectory (B-spline ...)
    TrajPolyMono polyTraj(_polyCoeff, _polyTime);
    
    _traj_time_final = _traj_time_start = ros::Time::now();

    ros::Time time_3 = ros::Time::now();
    // run the time optimizer
    if(time_optimizer.MinimumTimeGeneration( polyTraj, _MAX_Vel, _MAX_Acc, _MAX_d_Acc, _d_s))
    {   
        ros::Time time_4 = ros::Time::now();
        _has_traj = true;    
        ROS_WARN("[TimeOptimizer DEMO] Temporal trajectory generated");
        cout<<"[TimeOptimizer DEMO] time cunsume in temporal trajectory is: "<<(time_4 - time_3).toSec()<<endl;
        
        // pull out the results in an allocator data structure
        _time_allocator = time_optimizer.GetTimeAllcoation();

        for(int i = 0; i < _time_allocator->time.rows(); i++)
        {   
            int K = _time_allocator->K(i);
            _traj_time_final += ros::Duration(_time_allocator->time(i, K - 1));
        }

        cout<<"[TimeOptimizer DEMO] now start publishing commands"<<endl;
    }
    else
    {
        cout<<"[TimeOptimizer DEMO] temporal optimization fail"<<endl;
        cout<<"[TimeOptimizer DEMO] possible resons : " << "\n" <<
        "1 - please check the spatial trajectory,"     <<  "\n" <<
        "2 - numerical issue of the solver, try setting a larger d_s"<<endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_optimizer_demo");
    ros::NodeHandle nh("~");

    nh.param("demo/use_interactive", _is_use_inte, false);
    nh.param("demo/pkg_path",        _pkg_path, string("") );

    nh.param("planning/start_x",   _start_x,   0.0 );
    nh.param("planning/start_y",   _start_y,   0.0 );
    nh.param("planning/start_z",   _start_z,   0.0 );
    
    nh.param("planning/d_s",       _d_s,       0.01);
    nh.param("planning/max_vel",   _MAX_Vel,   1.0 );
    nh.param("planning/max_acc",   _MAX_Acc,   1.0 );
    nh.param("planning/max_d_acc", _MAX_d_Acc, 1.0 );
    nh.param("planning/dev_order", _dev_order,  3 );
    nh.param("planning/min_order", _min_order,  3 );

    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);
    nh.param("vis/is_dump_data",   _is_dump_data,   true);
    
    _way_pts_sub     = nh.subscribe( "waypoints",  1, rcvWaypointsCallback );

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("spatial_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("waypoint_path"     , 1);

    _vis_pos_pub     = nh.advertise<visualization_msgs::Marker>("desired_position", 50);    
    _vis_vel_pub     = nh.advertise<visualization_msgs::Marker>("desired_velocity", 50);    
    _vis_acc_pub     = nh.advertise<visualization_msgs::Marker>("desired_acceleration", 50);

    _poly_num1D = 2 * _dev_order;

    // define the visualization information of the published velocity and acceleration commands
    {
        _vis_pos.id = _vis_vel.id = _vis_acc.id = 0;
        _vis_pos.header.frame_id = _vis_vel.header.frame_id = _vis_acc.header.frame_id = "/map";
        
        _vis_pos.ns = "pos";
        _vis_pos.type   = visualization_msgs::Marker::SPHERE;
        _vis_pos.action = visualization_msgs::Marker::ADD;
        _vis_pos.color.a = 1.0; _vis_pos.color.r = 0.0; _vis_pos.color.g = 0.0; _vis_pos.color.b = 0.0;
        _vis_pos.scale.x = 0.2; _vis_pos.scale.y = 0.2; _vis_pos.scale.z = 0.2;

        _vis_vel.ns = "vel";
        _vis_vel.type = visualization_msgs::Marker::ARROW;
        _vis_vel.action = visualization_msgs::Marker::ADD;
        _vis_vel.color.a = 1.0; _vis_vel.color.r = 0.0; _vis_vel.color.g = 1.0; _vis_vel.color.b = 0.0;
        _vis_vel.scale.x = 0.2; _vis_vel.scale.y = 0.4; _vis_vel.scale.z = 0.4;

        _vis_acc.ns = "acc";
        _vis_acc.type = visualization_msgs::Marker::ARROW;
        _vis_acc.action = visualization_msgs::Marker::ADD;
        _vis_acc.color.a = 1.0; _vis_acc.color.r = 1.0; _vis_acc.color.g = 1.0; _vis_acc.color.b = 0.0;
        _vis_acc.scale.x = 0.2; _vis_acc.scale.y = 0.4; _vis_acc.scale.z = 0.4;
    }

    // if want to plot data afterwards, write down all data in local files
    if(_is_dump_data)
    {
        auto pos_path = _pkg_path + "/pos.txt";
        auto vel_path = _pkg_path + "/vel.txt";
        auto acc_path = _pkg_path + "/acc.txt";
        auto t_path   = _pkg_path + "/time.txt";
        pos_result.open(pos_path);
        vel_result.open(vel_path);
        acc_result.open(acc_path);
        t_result.open(t_path);
    }

    // wait 2 seconds for rviz to be launched.
    ROS_WARN("[TimeOptimizer DEMO] Wait for RVIZ open");
    sleep(2);
    
    MatrixXd waypoints;
    // if do not use the interactive tool, read waypoints directly from a local file.
    if( !_is_use_inte )
    {
        ROS_WARN("[TimeOptimizer DEMO] Read data from file");
        std::string file = _pkg_path + "/traj.json";
        std::ifstream f(file);
        json          j;

        f >> j;

        int size;
        vector<double> vectorX, vectorY, vectorZ;
        vectorX = j["x"].get<std::vector<double> >();
        vectorY = j["y"].get<std::vector<double> >();
        vectorZ = j["z"].get<std::vector<double> >();

        if (vectorX.size() == vectorY.size() && vectorY.size() == vectorZ.size() && vectorZ.size() != 0)
        {
            size = vectorX.size();
            waypoints.resize(size, 3);
        
            for(int k = 0; k < size; k++)
                waypoints.row(k) << vectorX[k], vectorY[k], vectorZ[k];

            trajGeneration(waypoints);
        }
        else
        {
            std::cout << "unexpected test input: " 
            << vectorX.size() << " "<< vectorY.size() << " " << vectorZ.size() << " , exit." << std::endl;
            exit(0);
        }
    }

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();
        pubCmd();        
        rate.sleep();
    }

    if(_is_dump_data)
    {
        pos_result.close();
        vel_result.close();
        acc_result.close();
        t_result.  close();
    }

    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{        
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "map";

    _traj_vis.ns = "time_optimal/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }

    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "/map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_path";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);

      if( i < (path.rows() - 1) )
      {
          geometry_msgs::Point p_line;
          p_line = p;
          line_list.points.push_back(p_line);
          p_line.x = path(i+1, 0);
          p_line.y = path(i+1, 1); 
          p_line.z = path(i+1, 2);
          line_list.points.push_back(p_line);
      }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Vector3d getVelPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
            if(j==0)
                time(j) = 0.0;
            else
                time(j) = j * pow(t, j-1);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

Vector3d getAccPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );

        for(int j = 0; j < _poly_num1D; j ++)
            if( j==0 || j==1 )
                time(j) = 0.0;
            else
                time(j) = j * (j - 1) * pow(t, j-2);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}