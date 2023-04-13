#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <lcmtypes/life_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include <unistd.h>
#include "maneuver_controller.h"
#include <mutex>

bool reverse = false;
bool newReverse;
std::mutex lifeLock_;  

class StraightManeuverController : public ManeuverControllerBase
{

private:
    float fwd_sum_error = 0;
    float fwd_last_error = 0;
    float turn_sum_error = 0;
    float turn_last_error = 0;
    /*************************************************************
    * TODO:
    *  - Week 4: Secion 3.2
    *      - Tune PID parameters for forward and turning during DRIVE state
    *************************************************************/
    float fwd_pid[3] = {0.6, 0.0, 0.0};
    float turn_pid[3] = {0.8, 0.0, 0.0};
    // float fwd_pid[3] = {0.6, 0, 0};
    // float turn_pid[3] = {1.3, 0.005, 0.};
    /*************************************************************
    * End of TODO
    *************************************************************/
public:
    StraightManeuverController() = default;   
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_fwd = sqrt(pow(dx,2) + pow(dy,2));
        float d_theta = angle_diff(atan2(dy,dx), pose.theta);


        // PID separately for the fwd and the angular velocity output given the fwd and angular error
        fwd_sum_error += d_fwd;
        float fwd_der = 0;
        if (fwd_last_error > 0)
            fwd_der = (d_fwd - fwd_last_error) / 0.05;
        
        float fwd_vel = fwd_pid[0] * d_fwd + fwd_pid[1] * fwd_sum_error + fwd_pid[2] * fwd_der;
        // fprintf(stdout,"Fwd error: %f\tFwd vel: %f\n", d_fwd, fwd_vel);

        turn_sum_error += d_theta;
        float turn_der = 0;
        if (turn_last_error > 0)
            turn_der = angle_diff(d_theta, turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\n", d_theta, turn_vel);
        if(reverse){
            fwd_vel = -fwd_vel;
        }



        return {0, fwd_vel, turn_vel};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target, bool is_end_pose)  override
    {
        return ((fabs(pose.x - target.x) < 0.04) && (fabs(pose.y - target.y)  < 0.04));
    }
};

class TurnManeuverController : public ManeuverControllerBase
{
private:    
    float turn_sum_error = 0;
    float turn_last_error = 0;
    /*************************************************************
    * TODO:
    *      - Tune PID parameters for turning during TURN state
    *************************************************************/
    float turn_pid[3] = {0.8, 0.0, 0.0};
    // float turn_pid[3] = {0.3, 0.0001, 0};
    /*************************************************************
    * End of TODO
    *************************************************************/
public:
    TurnManeuverController() = default;   
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d_theta = angle_diff(atan2(dy,dx), pose.theta);
        // fprintf(stdout,"dx: %f\tdy: %f\td_theta: %f\n", dx, dy, d_theta);

        // PID for the angular velocity given the delta theta
        turn_sum_error += d_theta;
        float turn_der = 0.0;
        if (turn_last_error > 0)
            turn_der = (d_theta - turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        // fprintf(stdout,"Turn error: %f\tTurn vel: %f\tPose theta: %f\n", d_theta, turn_vel, pose.theta);
        return {0, 0, turn_vel};
    }
    mbot_motor_command_t get_command_final_turn(const pose_xyt_t& pose, const pose_xyt_t& target)
    {
        float d_theta = angle_diff(target.theta, pose.theta);

        // PID for the angular velocity given the delta theta
        turn_sum_error += d_theta;
        float turn_der = 0;
        if (turn_last_error > 0)
            turn_der = (d_theta - turn_last_error) / 0.05;
        
        float turn_vel = turn_pid[0] * d_theta + turn_pid[1] * turn_sum_error + turn_pid[2] * turn_der;
        return {0, 0, turn_vel};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target, bool is_end_pose)  override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        // Handle the case when the target is on the same x,y but on a different theta
        return (fabs(angle_diff(pose.theta, target_heading)) < 0.1);
    }
    bool target_reached_final_turn(const pose_xyt_t& pose, const pose_xyt_t& target)
    {
        // Handle the case when the target is on the same x,y but on a different theta
        return (fabs(angle_diff(target.theta, pose.theta)) < 0.1);
    }

};

class MotionController
{ 
public: 
    
    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance)
    :
        lcmInstance(instance),
        odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();
        last_spin = utime_now();
	    time_offset = 0;
	    timesync_initialized_ = false;
    } 
    
    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    * 
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void) 
    {
        mbot_motor_command_t cmd {now(), 0.0, 0.0};
        
        if(!targets_.empty() && !odomTrace_.empty()) 
        {
            pose_xyt_t target = targets_.back();
            bool is_last_target = targets_.size() == 1;
            pose_xyt_t pose = currentPose();
            //std::cout << "targets size " << targets_.size() << std::endl;
            if(reverse){
                pose.theta = wrap_to_pi(pose.theta + M_PI);
                //target.theta = wrap_to_pi(target.theta + M_PI);
            }

            ///////  TODO (Optional): Add different states when adding maneuver controls /////// 
            // if(utime_now() - last_spin > 1e+7)
            // {
            //     if(angle_diff(pose.theta, 360) )
            //     cmd = {0, 0, M_PI/6};
            // }
            if(state_ == TURN)
            { 
                if(turn_controller.target_reached(pose, target, is_last_target))
                {
		            state_ = DRIVE;
                    printf("turn complete\n");
                } 
                else
                {
                    cmd = turn_controller.get_command(pose, target);
                    cmd.utime = now();
                    printf("turning\n");
                }
            }
            else if(state_ == DRIVE) 
            {
                if(straight_controller.target_reached(pose, target, is_last_target))
                {
                    printf("straight complete\n");
                    if (is_last_target)
                    {
                        state_ = FINAL_TURN;
                    }
                    else if (!assignNextTarget()){}
                }
                else
                { 
                    cmd = straight_controller.get_command(pose, target);
                    cmd.utime = now();
                    printf("straight\n");
                }
		    }
            else if(state_ == FINAL_TURN)
            {
                if(turn_controller.target_reached_final_turn(pose, target))
                {
                    printf("final turn complete\n");
                    
                    if (!newTargets_.empty()){
                        targets_ = newTargets_;
                        newTargets_.clear();
                        reverse = newReverse;
                    }

		            if(!assignNextTarget())
                    {
                        printf("Target reached! (%f,%f,%f)\n", target.x, target.y, target.theta);
                        // targets_ = newTargets_;
                        // newTargets_.clear();
                        // assignNextTarget();
                    }
                } 
                else
                {
                    cmd = turn_controller.get_command_final_turn(pose, target);
                    cmd.utime = now();
                    // printf("pose: %f target %f \n", pose.theta, target.theta);
                    printf("final turn\n");

                }
            }
            else if(state_ == SLEEP)
            {
                std::cout << "sleeping\n";
                // if(currentLife_.life_detected == 1 && currentLife_.distance < 0.5){
                //     state_ = TURN;
                // }
                // else {
                cmd = {0, 0, M_PI/6};
                if (reverse && currentLife_.life_detected == 1 && currentLife_.distance < 0.5){
                    std::cout << "spin complete\n";
                    cmd = {0, 0, 0};
                    lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
                    state_ = TURN; 
                }
                // }
            }
            else if (state_ == SPIN){
                std::cout << "spinning\n";
                cmd = {0, 0, M_PI/6};
                if ((now() - last_spin)/1000000 >= 12 || (reverse && currentLife_.life_detected == 1)){
                    std::cout << "spin complete\n";
                    cmd = {0, 0, 0};
                    lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
                    state_ = TURN; 
                }

            }
            else
            {
                std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            }
		} 
        return cmd; 
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync)
    {
	    timesync_initialized_ = true;
	    time_offset = timesync->utime-utime_now();

    }
    
    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        newTargets_ = path->path;
        // reverse = path->rescue;
        //targets_ = path->path;
        if(reverse)
            std::cout << "driving in reverse" << std::endl;

        std::cout << "received new path at time: " << path->utime << "\n"; 
    	for(auto pose : newTargets_)
        {
    		std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
    	}  	
        std::cout << std::endl;

        std::reverse(newTargets_.begin(), newTargets_.end()); // store first at back to allow for easy pop_back()  
        
        newReverse = path->rescue;

        if (targets_.empty()){
            reverse = newReverse;
            targets_ = newTargets_;
            newTargets_.clear();
            assignNextTarget();
        }

        //confirm that the path was received
        message_received_t confirm {now(), path->utime, channel};
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }
    
    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        pose_xyt_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }
    
    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        computeOdometryOffset(*pose);
    }

    void handleLifeDetection(const lcm::ReceiveBuffer* buf, const std::string& channel, const life_t* message){
        std::lock_guard<std::mutex> autoLock(lifeLock_);
        life_t prevLife = currentLife_;
        currentLife_.life_detected = static_cast<int8_t>(message->life_detected);
        currentLife_.distance = static_cast<float>(message->distance);
        
        std::cout << "Life detected at: " << currentLife_.life_detected << std::endl;
        if (currentLife_.life_detected == 1 && !prevLife.life_detected){
            targets_.clear();
        }
    }

    mbot_motor_command_t spinAround(){
        mbot_motor_command_t cmd {0, 0, M_PI/6};
        return cmd;
        //lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        //sleep(12);

        // for (int i = 0; i < 12; ++i){
        //     if (currentLife_.life_detected == 1) break;
        //     sleep(1);
        // }
        // std::cout << "spin complete\n";
        // cmd = {0, 0, 0};
        // lcmInstance->publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    }

private:
    
    enum State
    {
        TURN,
        FINAL_TURN, // to get to the pose heading
        DRIVE,
        SPIN,
        SLEEP,
    };
    
    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;
    std::vector<pose_xyt_t> newTargets_;
    
    State state_;
    int64_t last_spin;
    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;
 
    TurnManeuverController turn_controller;
    StraightManeuverController straight_controller;
    int pointCounter = 0;

    life_t currentLife_;

    int64_t now()
    {
	    return utime_now() + time_offset;
    }
    
    bool assignNextTarget(void)
    {   
        if(!targets_.empty()) { targets_.pop_back(); }

        pointCounter++;
        if(!reverse){ 
            std::cout << "num points: " << pointCounter << "\n";
            if (pointCounter > 4){
                pointCounter = 0;
                // spinAround();
                state_ = SPIN;
                last_spin = now();
                return !targets_.empty();
            }
            
        }
        else{
            if (pointCounter > 1){
                pointCounter = 0;
                state_ = SLEEP;
                return !targets_.empty();
            }
            
        }
        
        state_ = TURN; 
        return !targets_.empty();
    }
    
    void computeOdometryOffset(const pose_xyt_t& globalPose)
    {
        pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));
         
        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated; 
        odomToGlobalFrame_.theta = deltaTheta;
    }
    
    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());
        
        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta)) 
            + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
            + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);
        
        return pose;
    }

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
        lcmInstance->subscribe(DETECT_LIFE, &MotionController::handleLifeDetection, this);
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    signal(SIGINT, exit);
    
    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

    	if(controller.timesync_initialized()){
            	mbot_motor_command_t cmd = controller.updateCommand();

                /*************************************************************
                * TODO:
                *      - Change the limitation of forward and angular velocity commands
                *************************************************************/

                // Limit command values
                // Fwd vel
                float max_fwd_vel = 0.2;
                if (cmd.trans_v > max_fwd_vel) cmd.trans_v = max_fwd_vel;
                else if (cmd.trans_v < -max_fwd_vel) cmd.trans_v = -max_fwd_vel;

                // Angular vel
                float max_ang_vel = M_PI * 2.0 / 3.0;
                if (cmd.angular_v > max_ang_vel) cmd.angular_v = max_ang_vel;
                else if (cmd.angular_v < -max_ang_vel) cmd.angular_v = -max_ang_vel;
                
                /*************************************************************
                * End of TODO
                *************************************************************/

            	lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
    	}
    }
    
    return 0;
}

