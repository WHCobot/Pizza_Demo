#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <trac_ik/trac_ik.hpp>
#include <tf/transform_listener.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <list>

class pizza_demo;

class pizza_demo{
    typedef  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  Client;

public:
  pizza_demo(ros::NodeHandle _nh, const std::string & _urdf_param, const std::string & _chain_start, const std::string & _chain_end, double _timeout){
    //init
    nh_ = _nh;
    joint_name_[0] = "shoulder_pan_joint";
    joint_name_[1] = "shoulder_lift_joint";
    joint_name_[2] = "elbow_joint";
    joint_name_[3] = "wrist_1_joint";
    joint_name_[4] = "wrist_2_joint";
    joint_name_[5] = "wrist_3_joint";
    prefix_ = "ur3_1_";//todo
    current_JntArr_.resize(6);
    for(int i=0;i<6;++i)
    {
        current_JntArr_(i) = 0;
    }
    //force
    force_[0]=0;
    force_[1]=0;
    force_[2]=0;
    forceSize_ = 20;
    for(int i=0;i<forceSize_;++i)
    {
       force_list_.push_back(force_);
    }
    //velocity
    velSize_ = 5;//5
    for(int i=0;i<velSize_;++i)
    {
       velocity_list_.push_back(force_);//0
    }
    //sub
    bUR3_1_sub_ = false;
    bForce_sub_= false;
    bPos_sub_ = false;
   ur3_1_sub_=nh_.subscribe("ur3_1/joint_states", 1, &pizza_demo::subUR3_1JointStatesCB, this);//todo
   force_sub_=nh_.subscribe("forcesensor_data", 1, &pizza_demo::subForceStatesCB, this);
   pos_sub_=nh_.subscribe("sendJointState", 1, &pizza_demo::subPositionStatesCB, this);

   //action client
   client_ = new Client("follow_joint_trajectory_ur3_1", true);//todo
   ROS_INFO("wait for server follow_joint_trajectory_ur3_1");
   client_->waitForServer(ros::Duration());//test
   ROS_INFO("server connect! ");

   //ik
   double eps = 1e-5;
   p_tracik_solver_ = new TRAC_IK::TRAC_IK(_chain_start, _chain_end, _urdf_param, _timeout, eps);

   //fk
   KDL::Chain chain;
   bool valid = p_tracik_solver_->getKDLChain(chain);
   p_fk_solver = new KDL::ChainFkSolverPos_recursive(chain);

   //demo param
   sPos_.p.data[0] = 0.315745;
   sPos_.p.data[1] = 0.0665643;
   sPos_.p.data[2] = 0.45174;
   sPos_.M.data[0] = 0.707107;
   sPos_.M.data[1] = 0.0;
   sPos_.M.data[2] = -0.707107;
   sPos_.M.data[3] = 0.707107;
   sPos_.M.data[4] = 0.0;
   sPos_.M.data[5] = 0.707107;
   sPos_.M.data[6] = 0.0;
   sPos_.M.data[7] = -1;
   sPos_.M.data[8] = 0.0;
   center_x_ = 0.1;
   center_y_ = 0.35;
   center_z_ = 0.395;
   center_rot_ = -M_PI/4;
   latest_z_ = 0.40;
  }

  ~pizza_demo(){
    delete client_;
    delete p_tracik_solver_;
  }

  void first_kneading()
  {
//    ROS_INFO("first_kneading");
    //move to spos
    ros::Duration(0.5).sleep();
//    moveToPt(sPos_, 8);
    //sub
  bPos_sub_ = false;
  ros::spinOnce();
  int tc=0;
  while (bPos_sub_ == false){
        ros::Duration(0.001).sleep();
        ros::spinOnce();
        tc++;
        if(tc>1200)
          break;
  }

    //center point
    ROS_INFO_STREAM("centerPoint"<<center_rot_);
    double current_rot = center_rot_;
    KDL::Frame centerPos;
    centerPos.p.data[0] = center_x_;
    centerPos.p.data[1] = center_y_;
    centerPos.p.data[2] = center_z_;
    centerPos.M.data[0] = 1;
    centerPos.M.data[1] = 0;
    centerPos.M.data[2] = 0;
    centerPos.M.data[3] = 0;
    centerPos.M.data[4] = 0;
    centerPos.M.data[5] = 1;
    centerPos.M.data[6] = 0;
    centerPos.M.data[7] = -1;
    centerPos.M.data[8] = 0;
    KDL::Rotation rot, tmp(centerPos.M);
    rot.data[0] = cos(current_rot);
    rot.data[1] = sin(current_rot);
    rot.data[2] = 0;
    rot.data[3] = -sin(current_rot);
    rot.data[4] = cos(current_rot);
    rot.data[5] = 0;
    rot.data[6] = 0;
    rot.data[7] = 0;
    rot.data[8] = 1;
    centerPos.M = rot*tmp;
//    ROS_INFO_STREAM("centerPos  p:  "<<centerPos.p.data[0]<<";   "<<centerPos.p.data[1]<<";   "<<centerPos.p.data[2]);
//    ROS_INFO_STREAM("centerPos  M: "<<centerPos.M.data[0]<<";   "<<centerPos.M.data[1]<<";   "<<centerPos.M.data[2]<<";   "<<centerPos.M.data[3]<<";   "<<centerPos.M.data[4]<<";   "<<centerPos.M.data[5]<<";   "<<centerPos.M.data[6]<<";   "<<centerPos.M.data[7]<<";   "<<centerPos.M.data[8]);
    ros::Duration(0.5).sleep();
    while (!moveToPt(centerPos, 8))
    {
      ROS_INFO("send again");
    }

    //step down until touch the paste
    double dstep=0.001, dtime=0.1;//todo
    KDL::Frame targetPos = centerPos;
    ros::Duration(0.5).sleep();
    KDL::Vector force0 = force_;
//    ROS_INFO_STREAM("Force "<<force0.data[0]<<";  "<<force0.data[1]<<";  "<<force0.data[2]);
    ros::Duration(0.5).sleep();
    while (ros::ok())
    {
//      ROS_INFO("step_down");
      targetPos.p.data[2] -= dstep;
      moveToPt(targetPos, dtime);
      if(bForce_sub_ == true)
      {
        if(abs(force_[2]-force0[2])>12)
        {
          latest_z_ = targetPos.p.data[2];
          break;
        }
      }
      else
      {
        ROS_ERROR("No force data!!!");
        break;
      }
    }

    //do kneading
    dstep = 0.001;
    dtime = 0.005;
    int dir = 1, count=0;
    double stime = ros::Time::now().toSec();
    double tmptime = stime;
    while (ros::ok())
    {
      if(bForce_sub_ == true)
      {
//        ROS_INFO("do kneading");
//        targetPos.p.data[0] += (dir*dstep*sin(M_PI/2-current_rot));
//        targetPos.p.data[1] += (dir*dstep*cos(M_PI/2-current_rot));
        targetPos.p.data[0] += (dir*dstep*sin(current_rot));
        targetPos.p.data[1] += (dir*dstep*cos(current_rot));
        ROS_INFO_STREAM("targetPos_1: " <<targetPos.p.data[0]<<";  "<<targetPos.p.data[1]);
        moveToPt(targetPos, dtime);

        if(abs(force_[2]-force0[2])<0.5)
        {
//          ROS_INFO("force 0");
          if((ros::Time::now().toSec()-tmptime) >1)
          {
            ROS_WARN("change direction");
            tmptime = ros::Time::now().toSec();
            dir *= (-1);
            targetPos.p.data[2] -= 0.0005;
            count++;
            if (count>3)
            {
              ROS_WARN_STREAM("count "<<count);
              break;
            }
          }
//          else
//            break;
        }
      }
      else
      {
        ROS_ERROR("No force data!!!");
        break;
      }
      if (abs(ros::Time::now().toSec()-stime)>60)
      {
        ROS_WARN_STREAM("time "<<ros::Time::now().toSec()-stime);
        break;
      }
    }

    //move back to spos
    ros::Duration(0.5).sleep();
    moveToPt(centerPos, 4);
    ros::Duration(0.5).sleep();
    while (!moveToPt(sPos_, 8))
    {
      ROS_INFO("send again");
    }

    ros::Duration(0.5).sleep();
    for(int i=0;i<10;++i)
    {
      if(bForce_sub_ == true)
      {
        //sub
      bPos_sub_ = false;
      ros::spinOnce();
      int tc=0;
      while (bPos_sub_ == false){
            ros::Duration(0.001).sleep();
            ros::spinOnce();
            tc++;
            if(tc>1200)
              break;
      }
      ROS_INFO_STREAM("sub pos wait: 0.001*"<<tc);
     if (bPos_sub_)
        loop_kneading();
      if(latest_z_<0.328)
        break;
    }
    }
  }

  void loop_kneading()
  {
//    ROS_INFO("loop_kneading");

    //center point
    ROS_INFO_STREAM("centerPoint"<<center_rot_);
    double current_rot = center_rot_;
    KDL::Frame centerPos;
    centerPos.p.data[0] = center_x_;
    centerPos.p.data[1] = center_y_;
    centerPos.p.data[2] = latest_z_+0.004;
    centerPos.M.data[0] = 1;
    centerPos.M.data[1] = 0;
    centerPos.M.data[2] = 0;
    centerPos.M.data[3] = 0;
    centerPos.M.data[4] = 0;
    centerPos.M.data[5] = 1;
    centerPos.M.data[6] = 0;
    centerPos.M.data[7] = -1;
    centerPos.M.data[8] = 0;
    KDL::Rotation rot, tmp(centerPos.M);
    rot.data[0] = cos(current_rot);
    rot.data[1] = sin(current_rot);
    rot.data[2] = 0;
    rot.data[3] = -sin(current_rot);
    rot.data[4] = cos(current_rot);
    rot.data[5] = 0;
    rot.data[6] = 0;
    rot.data[7] = 0;
    rot.data[8] = 1;
    centerPos.M = rot*tmp;
    ROS_INFO_STREAM("centerPos  p:  "<<centerPos.p.data[0]<<";   "<<centerPos.p.data[1]<<";   "<<centerPos.p.data[2]);
    ROS_INFO_STREAM("centerPos  M: "<<centerPos.M.data[0]<<";   "<<centerPos.M.data[1]<<";   "<<centerPos.M.data[2]<<";   "<<centerPos.M.data[3]<<";   "<<centerPos.M.data[4]<<";   "<<centerPos.M.data[5]<<";   "<<centerPos.M.data[6]<<";   "<<centerPos.M.data[7]<<";   "<<centerPos.M.data[8]);
    ros::Duration(0.5).sleep();
    while (!moveToPt(centerPos, 8))
    {
      ROS_INFO("send again");
    }

    //step down until touch the paste
    double dstep=0.002, dtime=0.1;//todo
    KDL::Frame targetPos = centerPos;
    ros::Duration(0.5).sleep();
    KDL::Vector force0 = force_;
    ROS_INFO_STREAM("Force "<<force0.data[0]<<";  "<<force0.data[1]<<";  "<<force0.data[2]);
    ros::Duration(0.5).sleep();
    while (ros::ok())
    {
      ROS_INFO("step_down");
      targetPos.p.data[2] -= dstep;
      moveToPt(targetPos, dtime);
      if (targetPos.p.data[2]+2*dstep<latest_z_)
      {
        ROS_WARN("Error Position!");
        break;
      }
      if(bForce_sub_ == true)
      {
        if(abs(force_[2]-force0[2])>10)
        {
          latest_z_ = targetPos.p.data[2];
          break;
        }
      }
      else
      {
        ROS_ERROR("No force data!!!");
        break;
      }
    }

    //do kneading
    dstep = 0.001;
    dtime = 0.005;
    int dir = 1, count=0;
    double stime = ros::Time::now().toSec();
    double tmptime = stime;
    while (ros::ok())
    {
      if(bForce_sub_ == true)
      {
//        ROS_INFO("do kneading");
//        targetPos.p.data[0] += (dir*dstep*sin(M_PI/2-current_rot));
//        targetPos.p.data[1] += (dir*dstep*cos(M_PI/2-current_rot));
        targetPos.p.data[0] += (dir*dstep*sin(current_rot));
        targetPos.p.data[1] += (dir*dstep*cos(current_rot));
        ROS_INFO_STREAM("targetPos_2: " <<targetPos.p.data[0]<<";  " <<targetPos.p.data[1]);
        moveToPt(targetPos, dtime);

        if(abs(force_[2]-force0[2])<0.5)
        {
//          ROS_INFO("force 0");
          if((ros::Time::now().toSec()-tmptime) >1)
          {
//            ROS_WARN("change direction");
            tmptime = ros::Time::now().toSec();
            dir *= (-1);
            targetPos.p.data[2] -= 0.0005;
            count++;
            if (count>5)
            {
//              ROS_WARN_STREAM("count "<<count);
              break;
            }
          }
//          else
//            break;
        }
      }
      else
      {
        ROS_ERROR("No force data!!!");
        break;
      }
      if (abs(ros::Time::now().toSec()-stime)>60)
      {
        ROS_WARN_STREAM("time "<<ros::Time::now().toSec()-stime);
        break;
      }
    }

    //move back to spos
    ros::Duration(0.5).sleep();
    moveToPt(centerPos, 4);
    ros::Duration(0.5).sleep();
    while (!moveToPt(sPos_, 8))
    {
      ROS_INFO("send again");
    }
  }

  void trac_adjust()
  {
    ROS_INFO("trac_adjust");
    //F/T parameter
    KDL::Rotation rotFT_EE(0,0,1,0,-1,0,1,0,0);//adjust //todo
    KDL::Rotation rotEE_Base;
    double deltaVel=0.1;//0.01;//param //todo
    //time
//    ros::Rate r(1000);
    double deltaTime = 0.01;//0.001;//param //todo
    //
    KDL::Vector forceInit =  force_;
    KDL::Vector forceNet;
    double timeStart = ros::Time::now().toSec();
    double timeLast=0;
    while(ros::ok())
    {
      if (bForce_sub_==true)
      {
        bForce_sub_=false;
      //time
      double timeNow=ros::Time::now().toSec();
      double timeTmp = timeNow-timeStart;
      double dtime = timeTmp - timeLast;
      timeLast = timeTmp;
      ROS_ERROR_STREAM("run time: "<<timeTmp<<"s,  loop  delta time : "<<dtime);
//      if(timeTmp>60)    return;//test //todo
      //tf
      tf::StampedTransform tfEE_Base;
//      listener_.waitForTransform(prefix_+"base_link",prefix_+"ee_link", ros::Time(0), ros::Duration(1));//test
      listener_.lookupTransform(prefix_+"base_link",prefix_+"ee_link", ros::Time(0), tfEE_Base);//todo
//      ROS_INFO_STREAM("tfEE_Base.getOrigin():  "<<tfEE_Base.getOrigin().x()<<";   "<<tfEE_Base.getOrigin().y()<<";   "<<tfEE_Base.getOrigin().z());
      tf::Quaternion q = tfEE_Base.getRotation();
      rotEE_Base = KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w());

      //
      if (bUR3_1_sub_== true)
      {
      //force
      forceNet[0]= force_[0]-forceInit[0];
      forceNet[1]= force_[1]-forceInit[1];
      forceNet[2]= force_[2]-forceInit[2];

      KDL::Vector forceEE = rotFT_EE*forceNet;

      //velocity
      KDL::Vector velEE = forceEE*deltaVel;

      KDL::Vector velBase = rotEE_Base*velEE;

      KDL::Vector velFiltered = VelocityFilter(velBase);

      //pos
      KDL::Frame currentPos;
      p_fk_solver->JntToCart(current_JntArr_, currentPos);
      ROS_INFO_STREAM("currentPos.p:  "<<currentPos.p[0]<<";   "<<currentPos.p[1]<<";   "<<currentPos.p[2]);
      KDL::Frame targetPos = currentPos;
      targetPos.p += velFiltered*deltaTime;
      ROS_INFO_STREAM("targetPos.p:  "<<targetPos.p[0]<<";   "<<targetPos.p[1]<<";   "<<targetPos.p[2]);
      //move
      moveToPt( targetPos, deltaTime);
      }
     }
     else
      {
        ROS_ERROR("No Force Data!");
        break;
      }
    }
  }


  bool moveToPt(KDL::Frame &targetPos, double time)
  {
    bool ret = true;
    KDL::JntArray targetJnt;
    int rc=p_tracik_solver_->CartToJnt(current_JntArr_, targetPos, targetJnt);
    ROS_INFO_STREAM("ik result: "<<rc);
    if (rc == 1&&bUR3_1_sub_==true)
      {
        control_msgs::FollowJointTrajectoryGoal g;
        g.trajectory.header.stamp = ros::Time::now();
        for(int i=0;i<6;++i)
        {
            g.trajectory.joint_names.push_back(prefix_ + joint_name_[i]);
        }

//        //current
//        trajectory_msgs::JointTrajectoryPoint currentPt;
//        for(int i=0;i<6;++i)
//        {
//            currentPt.velocities.push_back(0);
//            currentPt.positions.push_back(current_JntArr_(i));
//        }
//        currentPt.time_from_start = ros::Duration(0);
//        g.trajectory.points.push_back(currentPt);
        //target
        trajectory_msgs::JointTrajectoryPoint targetPt;
        for(int i=0;i<6;++i)
        {
            targetPt.velocities.push_back(0);
            targetPt.positions.push_back(targetJnt(i));
        }
        double minT = MinTrajTimeNeeded(current_JntArr_, targetJnt);
        time = (time>minT)?time:minT;
        targetPt.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(targetPt);

       //send
       client_->cancelAllGoals();
       client_->sendGoal(g);
//       ROS_INFO("send goal");
       client_->waitForResult(ros::Duration(time));
  //     ROS_INFO("wait for result");
       int tt=0;
       double dt=(time>0.01)?time/10:0.001;
       while(client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            ++tt;
            client_->waitForResult(ros::Duration(dt));
            if(tt==50)
            {
               client_->cancelGoal();
               ROS_WARN("cancel goal");
               ret = false;
               break;
            }
       }
       ROS_INFO_STREAM("action client wait for result: "<<time<<"  +  "<<(dt*tt));
       bUR3_1_sub_ = false;
       bForce_sub_ = false;
     }
    //sub
    ros::spinOnce();
    int tc=0;
    while (bUR3_1_sub_ == false){
          ros::Duration(0.001).sleep();
          ros::spinOnce();
          tc++;
    }
    ROS_INFO_STREAM("sub ur3_1 wait: 0.001*"<<tc);
    tc=0;
    while (bForce_sub_==false&&tc<200){
          ros::Duration(0.001).sleep();
          ros::spinOnce();
          tc++;
    }
    if (tc!=200)
      ROS_INFO_STREAM("additional sub force wait: 0.001*"<<tc);
    else
      ROS_INFO_STREAM("No force data!!!!!!!!");
    return ret;
  }

  void moveTo(KDL::Frame &endPos, double time, bool interpolate=false)
  {
//    ROS_INFO("move ");
    KDL::Frame currentPos;
    p_fk_solver->JntToCart(current_JntArr_, currentPos);
    // pos vector
    std::vector<KDL::Frame>  vecPos;
    //interpolate
    double dx = endPos.p.data[0] - currentPos.p.data[0];
    double dy = endPos.p.data[1] - currentPos.p.data[1];
    double dz = endPos.p.data[2] - currentPos.p.data[2];
    ROS_INFO_STREAM("dx="<<dx<<"; dy="<<dy<<"; dz="<<dz);
    if (interpolate)
    {
      int isize=1+int(1000*sqrt(dx*dx+dy*dy+dz*dz));
//      ROS_INFO_STREAM("interpolate with size : "<<isize);
      for(int i=1;i<isize;++i)
      {
        KDL::Frame tmpPos = currentPos;
        tmpPos.p.data[0] += i*dx/isize;
        tmpPos.p.data[1] += i*dy/isize;
        tmpPos.p.data[2] += i*dz/isize;
        vecPos.push_back(tmpPos);
      }
    }
    vecPos.push_back(endPos);//don't forget end
    //ik  jnt vector
    std::vector<KDL::JntArray> vecJnt;
    KDL::JntArray tmpJnt = current_JntArr_;
    for(int i=0;i<vecPos.size();++i)
    {
      KDL::JntArray resultJnt;
      int rc=p_tracik_solver_->CartToJnt(tmpJnt, vecPos[i], resultJnt);
//        ROS_INFO_STREAM("ik tmp "<<i<<": "<<rc);
      if (rc == 1)
      {
        tmpJnt = resultJnt;
        vecJnt.push_back(resultJnt);
      }
    }

//        bUR3_1_sub_ = false;
//        //sub
//        ros::Duration(0.001).sleep();
//        ros::spinOnce();
//        int tc=0;
//        while (bUR3_1_sub_ == false){
//              ros::Duration(0.001).sleep();
//              ros::spinOnce();
//              tc++;
//        }
//        ROS_INFO_STREAM("sub wait: 0.001*"<<tc);

    if(bUR3_1_sub_ == true&&vecJnt.size()>0)
    {
      control_msgs::FollowJointTrajectoryGoal g;
      g.trajectory.header.stamp = ros::Time::now();
      for(int i=0;i<6;++i)
      {
          g.trajectory.joint_names.push_back(prefix_ + joint_name_[i]);
      }

      //current
      trajectory_msgs::JointTrajectoryPoint currentPt;
      for(int i=0;i<6;++i)
      {
          currentPt.velocities.push_back(0);
          currentPt.positions.push_back(current_JntArr_(i));
      }
      currentPt.time_from_start = ros::Duration(0);
      g.trajectory.points.push_back(currentPt);
      //trajectory
      for(int c=0;c<vecJnt.size();++c)
      {
        KDL::JntArray tJnt = vecJnt[c];
        trajectory_msgs::JointTrajectoryPoint tPt;
        for(int i=0;i<6;++i)
        {
          double vel=0;
//          if(c==0)
//            vel=vecJnt.size()*(tJnt(i)-current_JntArr_(i))/time;
//          else
//            vel=vecJnt.size()*(tJnt(i)-(vecJnt[c-1])(i))/time;
          tPt.velocities.push_back(vel);
          tPt.positions.push_back(tJnt(i));
       }
       tPt.time_from_start = ros::Duration((c+1)*time/vecJnt.size());
       g.trajectory.points.push_back(tPt);
     }
     //send
     client_->cancelAllGoals();
     client_->sendGoal(g);
     ROS_INFO("send goal");
     client_->waitForResult(ros::Duration(time));
//     ROS_INFO("wait for result");
     int tt=0;
     double dt=(time>0.01)?time/10:0.001;
     while(client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
          ++tt;
          client_->waitForResult(ros::Duration(dt));
          if(tt==50)
          {
             client_->cancelGoal();
             ROS_WARN("cancel goal");
             break;
          }
     }
     ROS_INFO_STREAM("action client wait for result: "<<time<<"  +  "<<(dt*tt));
     bUR3_1_sub_ = false;
     //sub
     ros::spinOnce();
     int tc=0;
     while (bUR3_1_sub_ == false){
           ros::Duration(0.001).sleep();
           ros::spinOnce();
           tc++;
     }
     ROS_INFO_STREAM("sub ur3_1 wait: 0.001*"<<tc);
     tc=0;
     while (bForce_sub_==false&&tc<200){
           ros::Duration(0.001).sleep();
           ros::spinOnce();
           tc++;
     }
     if (tc!=200)
       ROS_INFO_STREAM("additional sub force wait: 0.001*"<<tc);
     else
       ROS_INFO_STREAM("No force data!!!!!!!");
   }
  }

private:
  double MinTrajTimeNeeded(KDL::JntArray &sJnt, KDL::JntArray tJnt)
  {
    double time=0, max_vel=10.0*0.8;
    for(int i=0;i<sJnt.rows()*sJnt.columns();++i)
    {
      double tmp=(tJnt(i)-sJnt(i))/max_vel;
      time = (time>tmp)?time:tmp;
    }
    return time;
  }
  KDL::Vector ForceFilter(KDL::Vector &forceInput)
  {
  //    ROS_INFO("ForceFilter ");
    force_list_.pop_front();
    force_list_.push_back(forceInput);
    //average
    KDL::Vector sum(0,0,0);
    for(std::list<KDL::Vector>::iterator it=force_list_.begin();it!=force_list_.end();++it)
    {
      sum[0]+=it->data[0];
      sum[1]+=it->data[1];
      sum[2]+=it->data[2];
    }
    return sum/forceSize_;
  }
  KDL::Vector VelocityFilter(KDL::Vector &velInput)
  {
    //    ROS_INFO("VelocityFilter ");
    velocity_list_.pop_front();
    velocity_list_.push_back(velInput);
    //average
    KDL::Vector sum(0,0,0);
    for(std::list<KDL::Vector>::iterator it=velocity_list_.begin();it!=velocity_list_.end();++it)
    {
      sum[0]+=it->data[0];
      sum[1]+=it->data[1];
      sum[2]+=it->data[2];
    }
    return sum/velSize_;
  }

  void subUR3_1JointStatesCB(sensor_msgs::JointState state)
  {
//    ROS_INFO("sub UR3_1");
     //
      int n=state.name.size();
      current_JntArr_.resize(n);
    for(int i=0;i<n;++i)//state
    {
          int  x=0;
          for(;x<6;++x)//joint_name_
          {
            if(state.name[i] == (prefix_ + joint_name_[x]))
            {
               current_JntArr_(x)  = state.position[i];
//               ROS_INFO_STREAM("name: "<<state.name[i]<<"   pos: "<<state.position[i]<<"\n");
              break;
            }
          }

          if(x ==6)
          {
            ROS_ERROR_STREAM("Error,  joint name : "<<state.name[i]<<" , not found.  ");
            return;
          }
    }
//    //pos
//    KDL::Frame currentPos;
//    p_fk_solver->JntToCart(current_JntArr_, currentPos);
//    ROS_INFO_STREAM("currentPos  p:  "<<currentPos.p.data[0]<<";   "<<currentPos.p.data[1]<<";   "<<currentPos.p.data[2]);
//    ROS_INFO_STREAM("currentPos  M: "<<currentPos.M.data[0]<<";   "<<currentPos.M.data[1]<<";   "<<currentPos.M.data[2]<<";   "<<currentPos.M.data[3]<<";   "<<currentPos.M.data[4]<<";   "<<currentPos.M.data[5]<<";   "<<currentPos.M.data[6]<<";   "<<currentPos.M.data[7]<<";   "<<currentPos.M.data[8]);
    //
    bUR3_1_sub_ = true;
  }
  void subForceStatesCB(geometry_msgs::Wrench state)
  {
//    ROS_INFO("sub Force");
    KDL::Vector vForce;
    vForce[0] = state.force.x;
    vForce[1] = state.force.y;
    vForce[2] = state.force.z;
//    force_ = ForceFilter(vForce);
    force_ = vForce;
    //
    bForce_sub_ = true;
  }
  void subPositionStatesCB(sensor_msgs::JointState state)
  {
//    ROS_INFO("sub position");
     //
    center_x_ = 0.001*state.position[1]+0.075;
    center_y_ = 0.001*state.position[0]+0.450;
    center_rot_ = state.position[2]*M_PI/180;//
    bPos_sub_ = true;
  }
private:
  ros::NodeHandle nh_;
  Client *client_;
  TRAC_IK::TRAC_IK *p_tracik_solver_;
  KDL::ChainFkSolverPos_recursive *p_fk_solver;
  tf::TransformListener listener_;
  ros::Subscriber ur3_1_sub_;
  ros::Subscriber force_sub_;
  ros::Subscriber pos_sub_;
  std::string joint_name_[6];
  std::string prefix_;
  KDL::JntArray current_JntArr_;
  int forceSize_;
  KDL::Vector force_;
  std::list<KDL::Vector> force_list_;
  int velSize_;
  std::list<KDL::Vector> velocity_list_;
  bool bUR3_1_sub_;
  bool bForce_sub_;
  bool bPos_sub_;
  //demo param
  KDL::Frame sPos_;
  double center_x_;
  double center_y_;
  double center_z_;
  double center_rot_;
  double latest_z_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pizza_demo", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  std::string urdf_param;
  double timeout;
  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  pizza_demo demo(nh, urdf_param, "ur3_1_base_link", "ur3_1_wrist_3_link", timeout);//todo

  ros::Duration(1).sleep();
  ros::spinOnce();

  demo.first_kneading();
}

