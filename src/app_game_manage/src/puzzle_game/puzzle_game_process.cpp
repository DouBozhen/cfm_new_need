#include <cstring>
#include <fstream>
#include <sys/time.h>
#include "path_file.h"
#include "puzzle_game.h"

using namespace ur_data_type;
using namespace app_game_model;
using namespace force_data_type;
using namespace math_calculate;
using namespace planar_data_type;

std::ofstream puzzle_score_file(PUZZEL_SCORE_DIR, fstream::out|ios_base::trunc);
void PuzzleGame::initCutAndMoveForce(unsigned int force_level)
{
    switch(force_level)
    {
        case FORCE_LEVEL_ZERO:
        {
            cut_force_ = 0.0;
            move_force_ = 0.0;
        }
        break;
        case FORCE_LEVEL_SMALL:
        {
            cut_force_ = 4.0;
            move_force_ = 4.0;
        }
        break;
        case FORCE_LEVEL_MEDIUM:
        {
            cut_force_ = 6.0;
            move_force_ = 6.0;
        }
        break;
        case FORCE_LEVEL_LARGE:
        {
            cut_force_ = 8.0;
            move_force_ = 8.0;
        }
        break;
        default:;
    };
}

void PuzzleGame::initRuntimeParams()
{
    tcp_counter_ = 0;
    score_ = 0;
    fix_times_ = 0;
    fixing_state_ = EXIT_FIXING;

    ordinal_cutting_start_index_ = counter_;
    inverse_cutting_start_index_ = unity2ubuntu_.data(line_num_) - 1; 
}

void PuzzleGame::calculateTotalDistance(planar_data_type::PlanarPoint point_last, 
    planar_data_type::PlanarPoint point_actual,
    double &total_distance)
{
    total_distance += getPlanarP2PDistance(point_last, point_actual);
}

void PuzzleGame::getTcpPoseInWpCoordinate(
    ur_data_type::CartPose tool_pose_in_base,
    double trans_workpiece2base[4][4],
    ur_data_type::CartPose &tcp_pose_in_wp)
{
    double transition_tcp2base[4][4];
    double transition_base2tcp[4][4];
    double transition_wp2tcp[4][4];
    double transition_tcp2wp[4][4];   
    double rot_tcp2wp[3][3];
    double rpy[3];

    getTransitionTcp2Base(tool_pose_in_base, transition_tcp2base);
    matrix_inverse_44_.invMatrix44ByGaussianElimination(transition_tcp2base, transition_base2tcp);

    matrixMul44(transition_base2tcp, trans_workpiece2base, transition_wp2tcp);
    matrix_inverse_44_.invMatrix44ByGaussianElimination(transition_wp2tcp, transition_tcp2wp);

    getRotationFromTran(transition_tcp2wp, rot_tcp2wp);
    rotation2Rpy(rot_tcp2wp, rpy);

    tcp_pose_in_wp.point.x = transition_tcp2wp[0][3];
    tcp_pose_in_wp.point.y = transition_tcp2wp[1][3];
    tcp_pose_in_wp.point.z = transition_tcp2wp[2][3];
    tcp_pose_in_wp.rpy.rx = rpy[0];
    tcp_pose_in_wp.rpy.ry = rpy[1];
    tcp_pose_in_wp.rpy.rz = rpy[2];

    last_corner_point_.x = 0.0;
    last_corner_point_.y = 0.0;
}

void PuzzleGame::processPassiveForceOnMoving(CartPose tcp_pose_in_base)
{
    PlanarPoint line_start, line_end; // line_start: start_point0, line_end: start_point1
    line_start.x = unity2ubuntu_.point_cut(ordinal_cutting_start_index_).x();
    line_start.y = unity2ubuntu_.point_cut(ordinal_cutting_start_index_).y();
    getPoseInWpToBase(transition_wp2base_, line_start);
 
    ordinal_cutting_start_index_ = counter_;
    inverse_cutting_start_index_ = counter_ + unity2ubuntu_.data(line_num_) - 1; 

    line_end.x = unity2ubuntu_.point_cut(inverse_cutting_start_index_).x();
    line_end.y = unity2ubuntu_.point_cut(inverse_cutting_start_index_).y();
    getPoseInWpToBase(transition_wp2base_, line_end);

    PlanarPoint planar_tcp_point;
    planar_tcp_point.x = tcp_pose_in_base.point.x;
    planar_tcp_point.y = tcp_pose_in_base.point.y;

    double distance_start2tcp = getPlanarP2PDistance(line_start, planar_tcp_point);
    double distance_end2tcp = getPlanarP2PDistance(line_end, planar_tcp_point);
    double distance_delta = distance_start2tcp - distance_end2tcp;

    if (distance_end2tcp < puzzle_cut_switch_
        || distance_start2tcp < puzzle_cut_switch_)
    {
        ubuntu2unity_.set_game_result(PuzzleGameMsg::GAMING);
    }

    if (distance_delta > 0.01)
    {
        passive_force_ = force_control_.getPlanarPassiveForce(last_corner_point_, line_end, planar_tcp_point, cut_force_, 0.0);
        passive_force_ *= force_control_.getPlanarPassiveForceScaleFactor(last_corner_point_, line_end, planar_tcp_point, puzzle_cut_slow_);
        force_control_.setPlanarPassiveForce(passive_force_);
        
        if (distance_end2tcp < puzzle_cut_switch_ && ft_sensor_->getSensorData().force.fz > force_down_)
        {
            ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_DOWN);
            game_action_ = PuzzleGameAction::CUTTING; 
            game_order_ = GameOrder::INVERSE;
        }
    }
    else if (distance_delta > 0.0)
    {
        if (line_num_ == 0)
        {
            passive_force_ = force_control_.getPlanarPassiveForce(line_end, planar_tcp_point, cut_force_, 0.01);
        }
        else
        {
            passive_force_ = force_control_.getPlanarPassiveForce(last_corner_point_, line_end, planar_tcp_point, cut_force_, 0.0);
        }
        passive_force_ *= (distance_delta / 0.01);
    }
    else if (distance_delta >= -0.01)
    {
        if (line_num_ == 0)
        {
            passive_force_ = force_control_.getPlanarPassiveForce(line_start, planar_tcp_point, cut_force_, 0.01);
        }
        else
        {
            passive_force_ = force_control_.getPlanarPassiveForce(last_corner_point_, line_start, planar_tcp_point, cut_force_, 0.0);
        }
        passive_force_ *= (-distance_delta / 0.01);
    }
    else 
    {
        passive_force_ = force_control_.getPlanarPassiveForce(last_corner_point_, line_start, planar_tcp_point, cut_force_, 0.0);
        passive_force_ *= force_control_.getPlanarPassiveForceScaleFactor(last_corner_point_, line_start, planar_tcp_point, puzzle_cut_slow_);

        if (distance_start2tcp < puzzle_cut_switch_ && ft_sensor_->getSensorData().force.fz > force_down_)
        {
            ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_DOWN);
            game_action_ = PuzzleGameAction::CUTTING; 
            game_order_ = GameOrder::ORDINAL;
        }
    }

    passive_force_ += (pazzle_fault_force_ *= 0.98);
    force_control_.setPlanarPassiveForce(passive_force_);
}

void PuzzleGame::processPassiveForceOnCuting(ur_data_type::CartPose tcp_pose_in_base)
{
    PlanarPoint planar_tcp_point;
    planar_tcp_point.x = tcp_pose_in_base.point.x;
    planar_tcp_point.y = tcp_pose_in_base.point.y;

    if (!is_cutting_)
    {
        cut_total_distance_ = 0.0;
        is_cutting_ = true;
        last_corner_point_ = planar_tcp_point;
    }
    else
    {
        cut_total_distance_ += getPlanarP2PDistance(last_corner_point_, planar_tcp_point);
    }

    planar_data_type::PlanarPoint point_ordinal_start; 
    planar_data_type::PlanarPoint point_ordinal_end;

    if(game_order_ == GameOrder::ORDINAL)
    {
        point_ordinal_start.x = unity2ubuntu_.point_cut(ordinal_cutting_start_index_).x();
        point_ordinal_start.y = unity2ubuntu_.point_cut(ordinal_cutting_start_index_).y();

        point_ordinal_end.x = unity2ubuntu_.point_cut(ordinal_cutting_start_index_ + 1).x();
        point_ordinal_end.y = unity2ubuntu_.point_cut(ordinal_cutting_start_index_ + 1).y();
    }
    else 
    {
        point_ordinal_start.x = unity2ubuntu_.point_cut(inverse_cutting_start_index_).x();
        point_ordinal_start.y = unity2ubuntu_.point_cut(inverse_cutting_start_index_).y();
        
        point_ordinal_end.x = unity2ubuntu_.point_cut(inverse_cutting_start_index_ + 1).x();
        point_ordinal_end.y = unity2ubuntu_.point_cut(inverse_cutting_start_index_ + 1).y();
    }

    getPoseInWpToBase(transition_wp2base_, point_ordinal_start);
    getPoseInWpToBase(transition_wp2base_, point_ordinal_end);
    float distance_tcp2end = getPlanarP2PDistance(planar_tcp_point, point_ordinal_end);

    int cut_direction = force_control_.variableDigidity(point_ordinal_start, point_ordinal_end, planar_tcp_point, puzzle_rigidity_, PUZZLE_PATH_WIDTH, passive_force_);
    if (cut_direction == CuttingDirection::NEGATIVE) 
    {
        pazzle_fault_force_ = passive_force_;
        last_corner_point_ = planar_tcp_point;

        initRuntimeParams();
        game_action_ = PuzzleGameAction::MOVING;
        lose_num_++;

        ubuntu2unity_.set_game_result(PuzzleGameMsg::LOSE);
        ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_HANG);
    }
    else
    {
        updatePassiveForceByTrainMode(training_mode_, point_ordinal_start, point_ordinal_end, planar_tcp_point);
    }

    if (distance_tcp2end < puzzle_cut_switch_)
    {
        if (game_order_ == GameOrder::ORDINAL)
        {
            if (ordinal_cutting_start_index_ == counter_ + unity2ubuntu_.data(line_num_) - 2)
            {
                game_action_ = PuzzleGameAction::APARTING;
                passive_force_.fx = 0.0;
                passive_force_.fy = 0.0;
                force_control_.setPlanarPassiveForce(passive_force_);

                ordinal_cutting_start_index_ = counter_;
                inverse_cutting_start_index_ = counter_ + unity2ubuntu_.data(line_num_) - 1;

                ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_HANG);
                ubuntu2unity_.set_game_result(PuzzleGameMsg::WIN);
            }
            else
            {
                ordinal_cutting_start_index_++; /* aim for the next line */
            }
        }
        else
        {
            if (inverse_cutting_start_index_ == counter_ + 1) /* arreived the 2nd last point */
            {
                tcp_counter_ = 0;
                game_action_ = PuzzleGameAction::APARTING;
                passive_force_.fx = 0.0;
                passive_force_.fy = 0.0;
                force_control_.setPlanarPassiveForce(passive_force_);
                inverse_cutting_start_index_ = counter_ + unity2ubuntu_.data(line_num_) - 1;

                ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_HANG);
                ubuntu2unity_.set_game_result(PuzzleGameMsg::WIN);
            }
            else
            {
                inverse_cutting_start_index_--; /* aim for the next line */
                ubuntu2unity_.set_game_result(PuzzleGameMsg::GAMING);
            }
        }
    }
}

void PuzzleGame::processPassiveForceOnAparting(ur_data_type::CartPose tcp_pose_in_base, int puzzle_rigidity, double path_wide)
{
    PlanarPoint planar_tcp_point;
    planar_tcp_point.x = tcp_pose_in_base.point.x;
    planar_tcp_point.y = tcp_pose_in_base.point.y;

    PlanarPoint point_center;
    PlanarPoint point_position;

    point_center.x = unity2ubuntu_.point_center(line_num_).x();
    point_center.y = unity2ubuntu_.point_center(line_num_).y();
    point_position.x = unity2ubuntu_.point_target(line_num_).x();
    point_position.y = unity2ubuntu_.point_target(line_num_).y();

    getPoseInWpToBase(transition_wp2base_, point_center);
    getPoseInWpToBase(transition_wp2base_, point_position);

    double diatance_tcp2centor = getPlanarP2PDistance(point_center, planar_tcp_point);
    double diatance_tcp2position = getPlanarP2PDistance(point_position, planar_tcp_point);

    if (ubuntu2unity_.tcp_state() == PuzzleGameMsg::TCP_HANG)
    {
        PlanarPoint point_cut;
        if (game_order_ == GameOrder::ORDINAL)
        {
            point_cut.x = unity2ubuntu_.point_cut(counter_ + unity2ubuntu_.data(line_num_) - 1).x();
            point_cut.y = unity2ubuntu_.point_cut(counter_ + unity2ubuntu_.data(line_num_) - 1).y();
        }
        else 
        {
            point_cut.x = unity2ubuntu_.point_cut(counter_).x();
            point_cut.y = unity2ubuntu_.point_cut(counter_).y();
        }
        getPoseInWpToBase(transition_wp2base_, point_cut);
        updatePassiveForceByTrainMode(training_mode_, puzzle_rigidity, path_wide, point_cut, point_center, planar_tcp_point);
  
        if ((diatance_tcp2centor < puzzle_cut_switch_) && ft_sensor_->getSensorData().force.fz > force_up_)
        {
            passive_force_.fx = 0.0;
            passive_force_.fy = 0.0;
            force_control_.setPlanarPassiveForce(passive_force_);

            ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_UP); /* tell unity this line is finished */
            ubuntu2unity_.set_game_result(PuzzleGameMsg::GAMING);
        }
    }
    else if (ubuntu2unity_.tcp_state() == PuzzleGameMsg::TCP_DOWN)
    {
        if (tcp_counter_ < TEST_COUNT)
        {
            tcp_counter_++;
        }
        else
        {
            tcp_counter_ = 0;
            if (line_num_ == unity2ubuntu_.data_size() - 1)
            {
                line_num_ = 0;
                game_action_ = app_game_model::FIXTING;
                fixing_state_ = ENTER_FIXING;
                std::cout << "switch to fixing" << std::endl;
            }
            else
            {
                counter_ += unity2ubuntu_.data(line_num_);
                line_num_++;                                /* start fixing or start cutting next puzzle */
                last_corner_point_ = planar_tcp_point;
                game_action_ = app_game_model::MOVING;

                ubuntu2unity_.set_game_result(PuzzleGameMsg::GAMING);
            }
        }
    }
    else if (ubuntu2unity_.tcp_state() == PuzzleGameMsg::TCP_UP) /* moveing a single puzzle */
    {
        updatePassiveForceByTrainMode(training_mode_, puzzle_rigidity, path_wide, point_position, point_center, planar_tcp_point);
 
        if ((diatance_tcp2position < puzzle_cut_switch_) && ft_sensor_->getSensorData().force.fz > force_down_)
        {
            ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_DOWN); /* tell unity this line is finished */
        }
    }
}

void PuzzleGame::processPassiveForceOnFixing(ur_data_type::CartPose tcp_pose_in_base, int puzzle_rigidity, double path_wide)
{
    PlanarPoint planar_tcp_point;
    planar_tcp_point.x = tcp_pose_in_base.point.x;
    planar_tcp_point.y = tcp_pose_in_base.point.y;

    PlanarPoint point_center;
    PlanarPoint point_position;

    point_center.x = unity2ubuntu_.point_center(line_num_).x();
    point_center.y = unity2ubuntu_.point_center(line_num_).y();
    point_position.x = unity2ubuntu_.point_target(line_num_).x();
    point_position.y = unity2ubuntu_.point_target(line_num_).y();

    getPoseInWpToBase(transition_wp2base_, point_center);
    getPoseInWpToBase(transition_wp2base_, point_position);

    double diatance_tcp2centor = getPlanarP2PDistance(point_center, planar_tcp_point);
    double diatance_tcp2position = getPlanarP2PDistance(point_position, planar_tcp_point);

    switch (fixing_state_)
    {
        case ENTER_FIXING:
        {
            passive_force_ = force_control_.getPlanarPassiveForce(point_position, planar_tcp_point, speed_,0.02);
            force_control_.setPlanarPassiveForce(passive_force_);

            if (getPlanarP2PDistance(point_position, planar_tcp_point) < puzzle_cut_switch_
            && ft_sensor_->getSensorData().force.fz < force_up_)
            {
                ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_UP);
                fixing_state_ = TO_TARGET;
            }
        }
        break;
        case TO_TARGET:
        {
            updatePassiveForceByTrainMode(training_mode_, puzzle_rigidity, path_wide, point_position, point_center, planar_tcp_point);
            if (getPlanarP2PDistance(point_center, planar_tcp_point) < puzzle_cut_switch_
                && ft_sensor_->getSensorData().force.fz  > force_down_)
            {
                if (line_num_ == unity2ubuntu_.data_size() - 1)
                {
                    if (tcp_counter_ < TEST_COUNT)
                    {
                        tcp_counter_++;
                    }
                    else
                    {
                        tcp_counter_ = 0;
                        double cut_area = 0.0;
                        
                        ubuntu2unity_.set_game_score(calculateScore(start_time_, lose_num_, cut_total_distance_, cut_area));

                        fixing_state_ = EXIT_FIXING;
                        line_num_ = 0;
                    }
                }
                else
                {
                    line_num_++;
                    fixing_state_ = TO_CENTER;
                }

                ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_DOWN);
            }
        }
        break;
        case TO_CENTER: 
        {
            point_center.x = unity2ubuntu_.point_center(line_num_ - 1).x();
            point_center.y = unity2ubuntu_.point_center(line_num_ - 1).y();

            getPoseInWpToBase(transition_wp2base_, point_center);
            updatePassiveForceByTrainMode(training_mode_, puzzle_rigidity, path_wide, point_position, point_center, planar_tcp_point);
    
            if (getPlanarP2PDistance(point_center, planar_tcp_point) < puzzle_cut_switch_ 
                && ft_sensor_->getSensorData().force.fz  > force_down_)
            {
                ubuntu2unity_.set_tcp_state(PuzzleGameMsg::TCP_UP);
                fixing_state_ = TO_TARGET;
            }
        }
        break;
        case EXIT_FIXING:
            // todo...
        break;
        default:;
    }
}

float PuzzleGame::calculateScore(double start_time, int& lose_number, double& cut_length, double& cut_cntegral)
{
	static float best_cntegral = 0.05;
	static float best_time = 180;
	static float best_length = 2.5;
	double finish_time1 = getWallTime();
	double totaltime = finish_time1 - start_time;

	puzzle_score_file << "totaltime: " << totaltime << ", ";
	totaltime <= best_time ? totaltime = best_time : totaltime;

	puzzle_score_file << "cut_cntegral: " << cut_cntegral << ", ";
	cut_cntegral <= best_cntegral ? cut_cntegral = best_cntegral : cut_cntegral;

	puzzle_score_file << "cut_length: " << cut_length << ", ";
	cut_length <= best_length ? cut_length = best_length : cut_length;

	puzzle_score_file << "lose_number: " << lose_number << ", ";
	double total_score = 25 * (best_length / cut_length + 1 - 0.1*lose_number + best_cntegral / cut_cntegral + best_time / totaltime);
	std::cout << "total score: " << total_score << std::endl;

	puzzle_score_file.close();
	return total_score;
}

double PuzzleGame::getWallTime()
{
	struct timeval time;
	if (gettimeofday(&time, NULL))
	{
		return 0;
	}
	return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void PuzzleGame::updatePassiveForceByTrainMode(
    int train_mode, 
    PlanarPoint start,
    PlanarPoint end,
    PlanarPoint tcp)
{
    switch (train_mode)
    {
    case TrainingMode::ACTIVE:
        break;
    case TrainingMode::ASSIST:
    {
        passive_force_ += force_control_.getPlanarPassiveForce(start, end, tcp, cut_force_, 0.0);
        force_control_.setPlanarPassiveForce(passive_force_);
    }
        break;
    case TrainingMode::IMPEDANCE:
    {
        ForceValue cart_force;
        cart_force = force_control_.getCartRandomPassiveForce(random_force_max_, random_during_);
        passive_force_.fx = cart_force.fx;
        passive_force_.fy = cart_force.fy;
        force_control_.setPlanarPassiveForce(passive_force_);
    }
        break;
    case PASSIVE:
        break;
    default:;
    }

    passive_force_ *= force_control_.getPlanarPassiveForceScaleFactor(start, end, tcp, puzzle_cut_slow_);
    force_control_.setPlanarPassiveForce(passive_force_);
}

void PuzzleGame::updatePassiveForceByTrainMode(
    int train_mode, 
    double puzzle_rigidity,
    double path_wide,
    PlanarPoint start,
    PlanarPoint end,
    PlanarPoint tcp)
{
    switch (train_mode)
    {
        case TrainingMode::ACTIVE:
        {
            PlanarForceValue planner_force = force_control_.getPlanarPassiveForce();
            force_control_.variableDigidity(start, end, tcp, puzzle_rigidity, path_wide, planner_force);
        }
        break;
        case TrainingMode::ASSIST:
        {
            PlanarForceValue planner_force = force_control_.getPlanarPassiveForce();
            force_control_.variableDigidity(start, end, tcp, puzzle_rigidity, path_wide, planner_force);
            
            passive_force_ = planner_force + force_control_.getPlanarPassiveForce(start, end, tcp, speed_level_, 0.0);
            force_control_.setPlanarPassiveForce(passive_force_);
        }
        break;
        case TrainingMode::IMPEDANCE:
        {
            force_control_.setPassiveForce(force_control_.getCartRandomPassiveForce(max_force_, RANDOM_TIME));
        }
        break;
        default:;
    };

    passive_force_ = force_control_.getPlanarPassiveForce() * force_control_.getPlanarPassiveForceScaleFactor(start, end, tcp, 0.027);
    force_control_.setPlanarPassiveForce(passive_force_);
}
