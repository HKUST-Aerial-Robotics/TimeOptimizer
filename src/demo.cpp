#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <random>
#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "backward.hpp"
#include "time_optimizer.h"
#include "trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

namespace backward {
    backward::SignalHandling sh;
}

// Param from launch file
    double _vis_traj_width;
    double _MAX_Vel, _MAX_Acc, _MAX_d_Acc, _d_s;
    double _start_x, _start_y, _start_z;
    int _traj_order, _min_order, _poly_num1D;

// ros related
    ros::Subscriber _dest_pts_sub;
    ros::Publisher  _wp_traj_vis_pub, _wp_path_vis_pub;

// **** global variable *** //
// for planning
    MatrixXd _polyCoeff;
    VectorXd _polyTime;
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

#if 0
void pubCmd()
{   
    if(_traj_finish == false)
    {   
        return;
    }
    else if(_odom_time > _traj_time_final)
    {     

    }   
    else
    { 
        ROS_WARN("pub command");
        //publish position, velocity and acceleration command according to time bias
        double t = (_odom_time - _traj_time_start).toSec();
        if(t < 0) return;
        //ROS_WARN("[Time Optimal Trajectory Node] publish command, time is %f", t);
        
        MatrixXd time     = _time_allocator->time;
        MatrixXd time_acc = _time_allocator->time_acc;

        int idx;
        for(idx = 0; idx < _seg_num; idx++)
        {   
            int K = _time_allocator->K(idx);
            if( t  > time(idx, K - 1))
                t -= time(idx, K - 1);
            else
                break;
        }
        double t_tmp = t;     

        int grid_num = _time_allocator->K(idx);
        //ROS_WARN("[Time Optimal Trajectory Node] publish command, segm index is %d, segm time is %f", idx, t);
        
        // now we need to find which grid the time instance belongs to
        int grid_idx;
        for(grid_idx = 0; grid_idx < _time_allocator->K(idx); grid_idx++)
        {
            if (t > time(idx, grid_idx))
              continue;
            else
            { 
                //cout<<"grid_idx: "<<grid_idx<<", time(idx, grid_idx): "<<time(idx, grid_idx)<<endl;
                if(grid_idx > 0)
                  t -= time(idx, grid_idx - 1);
                else
                  t -= 0.0;

                break;
            }
        }
        
        //ROS_WARN("[Time Optimal Trajectory Node] publish command, grid index is %d, grid time is %f", grid_idx, t);
        double delta_t;
        if(grid_idx > 0)
          delta_t = (time(idx, grid_idx) - time(idx, grid_idx - 1));
        else
          delta_t = time(idx, grid_idx) - 0.0;
        // seemed has BUG ? shoudl be grid_idx - 1 ?
        
        double delta_s = t * _time_allocator->s_step / delta_t;
        double s = _time_allocator->s(idx, grid_idx) + delta_s;

        //cout<<"s: "<<s<<endl;
        Vector3d position_s = getPos(idx, s/_time_best(idx)) * _time_best(idx); 
        Vector3d position   = position_s;
        /*cout<<"getPos(idx, s): \n"<<getPos(idx, s)<<endl;
        cout<<"position: \n"<<position<<endl;*/

        double s_k   = _time_allocator->s(idx, grid_idx);
        double s_k_1 = _time_allocator->s(idx, grid_idx + 1);
        double b_k   = _time_allocator->b(idx, grid_idx);
        double b_k_1 = _time_allocator->b(idx, grid_idx + 1);

        //cout<<"s_k: "<<s_k<<"normalized s_k: "<<s_k   /_time_best(idx)<<endl;
        Vector3d velocity_s1 = getVel(idx, s_k   /_time_best(idx)); 
        Vector3d velocity_s2 = getVel(idx, s_k_1 /_time_best(idx));

        Vector3d velocity1   = velocity_s1 * sqrt(b_k);
        Vector3d velocity2   = velocity_s2 * sqrt(b_k_1);
        //cout<<"velocity1: \n"<<velocity1<<endl;
        Vector3d velocity   = velocity1 + (velocity2 - velocity1) * t / delta_t;

        // reset grid_idx and t for acc time
        t = t_tmp;
        for(grid_idx = 0; grid_idx < _time_allocator->K(idx); grid_idx++)
        {
            if (t > time_acc(idx, grid_idx))
              continue;
            else
            { 
                //cout<<"grid_idx: "<<grid_idx<<", time(idx, grid_idx): "<<time(idx, grid_idx)<<endl;
                if(grid_idx > 0)
                  t -= time_acc(idx, grid_idx - 1);
                else
                  t -= 0.0;

                break;
            }
        }
        
        if(grid_idx == grid_num)
            t -= time_acc(idx, grid_num - 1);

        //ROS_WARN("[Time Optimal Trajectory Node] publish command, grid index is %d, grid time is %f", grid_idx, t);
        Vector3d velocity_s, acceleration_s, acceleration1, acceleration2;
        Vector3d acceleration;

        double a_k;
        if( grid_idx == 0 && idx == 0 )
        {   
            s_k   = _time_allocator->s(idx, 0);
            s_k_1 = _time_allocator->s(idx, 0 + 1);
            
            a_k   = _time_allocator->a(idx, 0);
            b_k   = _time_allocator->b(idx, 0);
            b_k_1 = _time_allocator->b(idx, 0 + 1);

            velocity_s     = getVel(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx));
            acceleration_s = getAcc(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx)) /_time_best(idx);
            acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            acceleration1 << 0.0, 0.0, 0.0;
            
       /*     cout<<"acceleration2: \n"<<acceleration2<<endl;
            cout<<"time_acc(0, 0): "<<time_acc(0, 0)<<endl;*/

            acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / time_acc(0, 0); 
        }
        else if( grid_idx == grid_num && idx == (_seg_num - 1) )
        {   
            s_k   = _time_allocator->s(idx, grid_num - 1);
            s_k_1 = _time_allocator->s(idx, grid_num);
            
            a_k   = _time_allocator->a(idx, grid_num - 1);
            b_k   = _time_allocator->b(idx, grid_num - 1);
            b_k_1 = _time_allocator->b(idx, grid_num    );

            velocity_s     = getVel(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx));
            acceleration_s = getAcc(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx)) / _time_best(idx);
            acceleration = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
            //cout<<"final acceleration: \n"<<acceleration<<endl;
            //acceleration << 0.0, 0.0, 0.0;
        }
        else
        {   
            if(grid_idx < grid_num && grid_idx > 0) // take average accleration in a same segment
            {   
                delta_t = (time_acc(idx, grid_idx) - time_acc(idx, grid_idx - 1));
                
                s_k   = _time_allocator->s(idx, grid_idx - 1);
                s_k_1 = _time_allocator->s(idx, grid_idx + 0);
                
                a_k   = _time_allocator->a(idx, grid_idx - 1);
                b_k   = _time_allocator->b(idx, grid_idx - 1);
                b_k_1 = _time_allocator->b(idx, grid_idx + 0);

                velocity_s     = getVel(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx));
                acceleration_s = getAcc(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx)) / _time_best(idx);
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                //acceleration = acceleration1;

                s_k   = _time_allocator->s(idx, grid_idx + 0);
                s_k_1 = _time_allocator->s(idx, grid_idx + 1);

                a_k   = _time_allocator->a(idx, grid_idx + 0);
                b_k   = _time_allocator->b(idx, grid_idx + 0);
                b_k_1 = _time_allocator->b(idx, grid_idx + 1);              
                //cout<<"a_k: "<<a_k<<" , "<<"s_k: "<<s_k<<" , "<<"s_k+1: "<<s_k_1<<endl;

                velocity_s     = getVel(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx));
                acceleration_s = getAcc(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx)) / _time_best(idx);
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;   
            }
            else if(grid_idx == grid_num)// take average accleration between two segments
            {   
                delta_t = (time(idx, grid_num - 1) - time_acc(idx, grid_num - 1) + time_acc(idx + 1, 0) );
                
                s_k   = _time_allocator->s(idx, grid_idx - 1);
                s_k_1 = _time_allocator->s(idx, grid_idx);
                
                a_k   = _time_allocator->a(idx, grid_idx - 1);
                b_k   = _time_allocator->b(idx, grid_idx - 1);
                b_k_1 = _time_allocator->b(idx, grid_idx);

                velocity_s     = getVel(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx));
                acceleration_s = getAcc(idx, (s_k + s_k_1 ) / 2.0 /_time_best(idx)) / _time_best(idx);
                acceleration1 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                s_k   = _time_allocator->s(idx + 1, 0);
                s_k_1 = _time_allocator->s(idx + 1, 1);

                a_k   = _time_allocator->a(idx + 1, 0);
                b_k   = _time_allocator->b(idx + 1, 0);
                b_k_1 = _time_allocator->b(idx + 1, 1);              

                velocity_s     = getVel(idx + 1, (s_k + s_k_1 ) / 2.0 / _time_best(idx + 1));
                acceleration_s = getAcc(idx + 1, (s_k + s_k_1 ) / 2.0 / _time_best(idx + 1)) / _time_best(idx + 1);
                acceleration2 = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration  = acceleration1 + (acceleration2 - acceleration1) * t / delta_t;        
            }
            else if(grid_idx == 0)// take average accleration between two segments
            {   
                int grid_num_k = _time_allocator->K(idx - 1); // last segment's grid num
                delta_t = (time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1) + time_acc(idx, 0) );
                
                s_k   = _time_allocator->s(idx - 1, grid_num_k - 1);
                s_k_1 = _time_allocator->s(idx - 1, grid_num_k    );
                
                a_k   = _time_allocator->a(idx - 1, grid_num_k - 1);
                b_k   = _time_allocator->b(idx - 1, grid_num_k - 1);
                b_k_1 = _time_allocator->b(idx - 1, grid_num_k    );

                velocity_s     = getVel(idx - 1, (s_k + s_k_1 ) / 2.0 / _time_best(idx - 1));
                acceleration_s = getAcc(idx - 1, (s_k + s_k_1 ) / 2.0 / _time_best(idx - 1)) / _time_best(idx - 1);
                acceleration1  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;

                s_k   = _time_allocator->s(idx, 0);
                s_k_1 = _time_allocator->s(idx, 0 + 1);
                
                a_k   = _time_allocator->a(idx, 0);
                b_k   = _time_allocator->b(idx, 0);
                b_k_1 = _time_allocator->b(idx, 0 + 1);

                velocity_s     = getVel(idx, (s_k + s_k_1 ) / 2.0 / _time_best(idx));
                acceleration_s = getAcc(idx, (s_k + s_k_1 ) / 2.0 / _time_best(idx)) / _time_best(idx);
                acceleration2  = velocity_s * a_k + acceleration_s * (b_k + b_k_1) / 2.0;
                acceleration   = acceleration1 + (acceleration2 - acceleration1) * (t + time(idx - 1, grid_num_k - 1) - time_acc(idx - 1, grid_num_k - 1)) / delta_t;   
            } 
            else
              ROS_BREAK();
        }

        _vis_pos.header.stamp = _odom_time;
        _vis_vel.header.stamp = _odom_time;
        _vis_acc.header.stamp = _odom_time;
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

    //ROS_WARN("[Benchmark] finish allocating time for waypoints trajecotry");
    return time;
}

VectorXd timeAllocationNaive(VectorXd path)
{   
    VectorXd time;
    time.resize(path.rows() - 1);

    for(int i = 0; i < time.size(); i++)
        time(i) = 1.0;

    return time;
}

void trajGeneration(Eigen::MatrixXd path)
{   
    TrajectoryGeneratorWaypoint  trajectoryGeneratorWaypoint;
    MinimumTimeOptimizer time_optimizer;
    
    MatrixXd vel = MatrixXd::Zero(2,3); 
    MatrixXd acc = MatrixXd::Zero(2,3);

    _polyTime  = timeAllocation(path); 
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(path, vel, acc, _polyTime);   
    
    visWayPointPath(path);
    visWayPointTraj( _polyCoeff, _polyTime);

    time_optimizer.MinimumTimeGeneration( _polyCoeff, _polyTime, _MAX_Vel, _MAX_Acc, _MAX_d_Acc, _d_s);   
            
    _has_traj = true;    
    _traj_time_final = _traj_time_start = ros::Time::now();

    for(int i = 0; i < _time_allocator->time.rows(); i++)
    {   
        int K = _time_allocator->K(i);
        _traj_time_final += ros::Duration(_time_allocator->time(i, K - 1));
    }

    cout<<"start and final time: "<<_traj_time_start<<" , "<<_traj_time_final<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "time_optimizer_demo");
    ros::NodeHandle nh("~");

    nh.param("planning/start_x",   _start_x,   0.0 );
    nh.param("planning/start_y",   _start_y,   0.0 );
    nh.param("planning/start_z",   _start_z,   0.0 );
    
    nh.param("planning/d_s",       _d_s,       0.01);
    nh.param("planning/max_vel",   _MAX_Vel,   1.0 );
    nh.param("planning/max_acc",   _MAX_Acc,   1.0 );
    nh.param("planning/max_d_acc", _MAX_d_Acc, 1.0 );
    nh.param("planning/traj_order", _traj_order, 6 );
    nh.param("planning/min_order",  _min_order,  3 );

    nh.param("vis/vis_traj_width", _vis_traj_width,  0.15);
        
    _poly_num1D = _traj_order + 1;

    _vis_pos.ns = "pos";
    _vis_pos.id = 0;
    _vis_pos.header.frame_id = "/map";
    _vis_pos.type = visualization_msgs::Marker::SPHERE;
    _vis_pos.action = visualization_msgs::Marker::ADD;
    _vis_pos.color.a = 1.0;
    _vis_pos.color.r = 0.0;
    _vis_pos.color.g = 0.0;
    _vis_pos.color.b = 0.0;
    _vis_pos.scale.x = 0.2;
    _vis_pos.scale.y = 0.2;
    _vis_pos.scale.z = 0.2;

    _vis_vel.ns = "vel";
    _vis_vel.id = 0;
    _vis_vel.header.frame_id = "/map";
    _vis_vel.type = visualization_msgs::Marker::ARROW;
    _vis_vel.action = visualization_msgs::Marker::ADD;
    _vis_vel.color.a = 1.0;
    _vis_vel.color.r = 0.0;
    _vis_vel.color.g = 1.0;
    _vis_vel.color.b = 0.0;
    _vis_vel.scale.x = 0.2;
    _vis_vel.scale.y = 0.4;
    _vis_vel.scale.z = 0.4;

    _vis_acc.ns = "acc";
    _vis_acc.id = 0;
    _vis_acc.header.frame_id = "/map";
    _vis_acc.type = visualization_msgs::Marker::ARROW;
    _vis_acc.action = visualization_msgs::Marker::ADD;
    _vis_acc.color.a = 1.0;
    _vis_acc.color.r = 1.0;
    _vis_acc.color.g = 1.0;
    _vis_acc.color.b = 0.0;
    _vis_acc.scale.x = 0.2;
    _vis_acc.scale.y = 0.4;
    _vis_acc.scale.z = 0.4;

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();  
        status = ros::ok();
        //pubCmd();
        rate.sleep();
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