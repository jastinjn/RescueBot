#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.005f)
, alpha2(0.02f)
, alpha4(0.02f)
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
    
    /* FROM HER VIDEO
    float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);
    float sampleTrans = std::normal_distribution<>(trans_, tranStd_)(numberGenerator_);

    newSample.pose.x += sampleTrans * std::cos(sample.pose.theta + sampleRot1);
    newSample.pose.y += sampleTrans * std::sin(sample.pose.theta + sampleRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
    newSample.pose.utime = utime_; //updated current time
    newSample.parent_pose = sample.pose;
    */

    float x = sample.pose.x;
    float y = sample.pose.y;
    float theta = sample.pose.theta;

    // float x_bar = previousPose_.x;
    // float y_bar = previousPose_.y;
    // float theta_bar = previousPose_.theta;

    // float x_bar_prime = previousPose_.parent_pose.x;
    // float y_bar_prime = previousPose_.parent_pose.y;
    // float theta_bar_prime = previousPose_.parent_pose.theta;

    // float delta_rot_1 = atan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - theta_bar;
    // float delta_trans = std::sqrt(std::pow(x_bar_prime - x_bar, 2) + std::pow(y_bar_prime - y_bar, 2));
    // float delta_rot_2 = theta_bar_prime - theta_bar - delta_rot_1;

    //the book: The function sample(b) generates a random sample from a zero-centered
    //distribution with variance b. It may, for example, be implemented using the algorithms in
    //Table 5.4.
    //k1=alpha1, k2 =alpha3, cross correlations = alpha2,alpha4
    // rot1Std_ = k1_ * delta_rot_1 + alpha2 * delta_trans;
    // tranStd_ = k2_ * delta_trans + alpha4*(delta_rot_1 + delta_rot_2);
    // rot2Std_ = k1_*delta_rot_2 + alpha2*delta_trans;
    
    float delta_rot_1_hat = rot1_ - std::normal_distribution<float>(0, rot1Std_)(numberGenerator_);
    float delta_trans_hat = trans_ - std::normal_distribution<float>(0, tranStd_)(numberGenerator_);
    float delta_rot_2_hat = rot2_ - std::normal_distribution<float>(0, rot2Std_)(numberGenerator_);

    // float x_prime = x + delta_trans_hat * std::cos(theta + delta_rot_1_hat);
    // float y_prime = y + delta_trans_hat*std::sin(theta + delta_rot_1_hat);
    // float theta_prime = theta + delta_rot_1_hat + delta_rot_2_hat;

    newSample.pose.x = x + delta_trans_hat * std::cos(theta + delta_rot_1_hat);
    newSample.pose.y = y + delta_trans_hat*std::sin(theta + delta_rot_1_hat);
    newSample.pose.theta = theta + delta_rot_1_hat + delta_rot_2_hat;
    newSample.pose.utime = utime_; //updated current time
    newSample.parent_pose = sample.pose;

    return newSample; //newly sampled after applying action model (just with a normal distribution)
}
