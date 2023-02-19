#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.005f)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    //change k1, k2 in my experiment (and alpha2,4)
    //Prof Du: the parameters would determine how much error it's adding to the RTR action- 
    //so you can see how increasing or decreasing the param values impact the spread of particles)
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        previousPose_ = odometry;
        initialized_ = true;
    }

    float deltaX = odometry.x - previousPose_.x;
    float deltaY = odometry.y - previousPose_.y;
    float deltaTheta = angle_diff(odometry.theta, previousPose_.theta);
    float direction = 1.0; //rotation direction

    //we are implementing the RTR model from lecture
    trans_ = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousPose_.theta);

    if(std::abs(rot1_) > M_PI_2){
        //sanity check if it has rotated over pi/2 --> we went backwards and changed direction
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(deltaTheta, rot1_);

    bool moved = (deltaX!=0) || (deltaY!=0) || (deltaTheta!=0); //has the robot moved?
    if(moved){
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_)); //k1 is std dev scalar for rot1
        tranStd_ = std::sqrt(k2_ * std::abs(trans_));
        rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
    }

    utime_ = odometry.utime;
    previousPose_ = odometry;
    trans_ *= direction;

    return moved;
}

particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    particle_t newSample = sample; //= previous sample
    
    //this demo: easy and simple one --> normal distribution
    //we should implement code to sample new poses from the desired distribution as updated above

    // float x = sample.pose.x; //current pose
    // float y = sample.pose.y; //current pose
    // float theta = sample.pose.theta; //current pose

    // float delta_rot_1_hat = delta_rot_1 - std::normal_distribution<>(0, rot1Std_)(numberGenerator_);
    // float delta_trans_hat = delta_trans - std::normal_distribution<>(0, tranStd_)(numberGenerator_);
    // float delta_rot_2_hat = delta_rot_2 - std::normal_distribution<>(0, rot2Std_)(numberGenerator_);

    // newSample.pose.x = x + delta_trans_hat * std::cos(theta + delta_rot_1_hat);
    // newSample.pose.y = y + delta_trans_hat*std::sin(theta + delta_rot_1_hat);
    // newSample.pose.theta = wrap_to_2pi(theta + delta_rot_1_hat + delta_rot_2_hat);
    // newSample.pose.utime = utime_; //updated current time
    // newSample.parent_pose = sample.pose;

    float sampleRot1 = std::normal_distribution<float>(rot1_, rot1Std_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<float>(rot2_, rot2Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<float>(trans_, tranStd_)(numberGenerator_);

    newSample.pose.x += sampleTrans * std::cos(sample.pose.theta + sampleRot1);
    newSample.pose.y += sampleTrans * std::sin(sample.pose.theta + sampleRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
    newSample.pose.utime = utime_; //updated current time
    newSample.parent_pose = sample.pose;
    std::cout << "(x,y,theta) = (" << newSample.pose.x << "," << newSample.pose.y << "," << newSample.pose.theta << ")" << std::endl;

    return newSample; //newly sampled after applying action model (just with a normal distribution)
}
