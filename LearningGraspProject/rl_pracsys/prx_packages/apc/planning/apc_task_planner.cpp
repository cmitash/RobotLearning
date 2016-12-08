/**
 * @file apc_task_planner.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "planning/apc_task_planner.hpp"
#include "planning/specifications/manipulation_specification.hpp"

#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "utilities/definitions/manip_defs.hpp"

#include <boost/range/adaptor/map.hpp>
#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>

#include <time.h>

#define MAX_TRIES 9999999999

PLUGINLIB_EXPORT_CLASS(prx::packages::apc::apc_task_planner_t ,prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;

    namespace packages
    {
        using namespace manipulation;
        namespace apc
        {

            apc_task_planner_t::apc_task_planner_t()
            {

            }

            apc_task_planner_t::~apc_task_planner_t()
            {

            }

            void apc_task_planner_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                failedmanualretr1 = 0;
                failedmanualretr2 = 0;
                transferfailed = 0;
                movefailed = 0;

                PRX_INFO_S("Initializing apc_task_planner_t task planner ...");
                task_planner_t::init(reader,template_reader);
                left_context_name = parameters::get_attribute("left_context_name", reader, template_reader);
                right_context_name = parameters::get_attribute("right_context_name", reader, template_reader);
                left_camera_context_name = parameters::get_attribute("left_camera_context_name", reader, template_reader);
                right_camera_context_name = parameters::get_attribute("right_camera_context_name", reader, template_reader);
                manipulation_task_planner_name = parameters::get_attribute("manipulation_task_planner_name", reader, template_reader);

                manip_planner = dynamic_cast<manipulation_tp_t*>(planners[manipulation_task_planner_name]);

                // float gripper_offset = 0.21;
                float gripper_offset = 0.0;
                examination_profile_t* profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0.27);
                profile->focus.push_back(1.45);
                profile->base_viewpoint.set(0,1,0,1);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['A'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0);
                profile->focus.push_back(1.45);
                profile->base_viewpoint.set(0,1,0,1);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['B'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(-0.27);
                profile->focus.push_back(1.45);
                profile->base_viewpoint.set(0,1,0,1);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['C'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0.27);
                profile->focus.push_back(1.20);
                profile->base_viewpoint.set(0,1,0,1);
                // profile->base_viewpoint.set_from_euler( 0,0,90);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['D'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0);
                profile->focus.push_back(1.20);
                profile->base_viewpoint.set(0,1,0,1);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['E'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(-0.27);
                profile->focus.push_back(1.20);
                profile->base_viewpoint.set(0,1,0,1);
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['F'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0.27);
                profile->focus.push_back(1.05);
                // profile->base_viewpoint.set(0,1,0,1);
                // profile->base_viewpoint.set(0.70711,0,0.70711,0);
                profile->base_viewpoint.set(0.79335,0,0.60876,0);//Y->+75
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['G'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0);
                profile->focus.push_back(1.05);
                // profile->base_viewpoint.set(0,0,0,1);
                // profile->base_viewpoint.set(0.70711,0,0.70711,0);//Y->+90
                profile->base_viewpoint.set(0.79335,0,0.60876,0);//Y->+75
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['H'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(-0.27);
                profile->focus.push_back(1.05);
                // profile->base_viewpoint.set(0,0,0,1);
                // profile->base_viewpoint.set(0.70711,0,0.70711,0);
                profile->base_viewpoint.set(0.79335,0,0.60876,0);//Y->+75
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['I'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0.27);
                profile->focus.push_back(0.87);
                // profile->base_viewpoint.set(0,0,0,1);
                // profile->base_viewpoint.set(0.70711,0,0.70711,0);
                // profile->base_viewpoint.set(0.79335,0,0.60876,0);//Y->+75
                profile->base_viewpoint.set(0.84339,0,0.5373,0);//Y->+65
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['J'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(0);
                profile->focus.push_back(0.87);
                // profile->base_viewpoint.set(0,0,0,1);
                // profile->base_viewpoint.set(0.70711,0,0.70711,0);
                // profile->base_viewpoint.set(0.79335,0,0.60876,0);//Y->+75
                profile->base_viewpoint.set(0.84339,0,0.5373,0);//Y->+65
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['K'] = profile;

                profile = new examination_profile_t();
                profile->focus.push_back(.8-gripper_offset);
                profile->focus.push_back(-0.27);
                profile->focus.push_back(0.77);
                // profile->base_viewpoint.set(0,0,0,1);
                // profile->base_viewpoint.set(0.70711,0,0.70711,0);
                // profile->base_viewpoint.set(0.79335,0,0.60876,0);//Y->+75
                profile->base_viewpoint.set(0.84339,0,0.5373,0);//Y->+65
                profile->base_viewpoint.normalize();
                profile->offsets.push_back(profile->base_viewpoint);
                profile->offsets.push_back(profile->base_viewpoint);
                profile->distance = .3;
                camera_positions['L'] = profile;

                left_arm_order_bin  = boost::assign::list_of(-1.57)(1.661659)(0.677508)(0)(-1.120185)(0)(-0.165669)(0)(0);
                right_arm_order_bin = boost::assign::list_of( 1.57)(1.661659)(0.677508)(0)(-1.120185)(0)(-0.165669)(0);

                total_move_time=0;
                total_resolve_time=0;
                total_tries=0;
                total_failures=0;
            }

            void apc_task_planner_t::link_world_model(world_model_t * const model)
            {
                task_planner_t::link_world_model(model);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                if(manipulation_model == NULL)
                    PRX_FATAL_S("The manipulation task planner can work only with manipulation world model!");
            }

            void apc_task_planner_t::link_query(query_t* new_query)
            {
                task_planner_t::link_query(new_query);
                in_query = static_cast<apc_task_query_t*>(new_query);
            }

            void apc_task_planner_t::setup()
            {
                // TODO: This should probably link all the underlying task planner's specifications, not just manip tp...
                // set the manip tp spec
                output_specifications[manip_planner->get_name()]->setup( manipulation_model );
                manip_planner->link_specification(output_specifications[manipulation_task_planner_name]);
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {
                    PRX_INFO_S("setup planner: " << planner->get_name());
                    planner->setup();
                }
            }

            bool apc_task_planner_t::execute()
            {
                foreach(planner_t* planner, planners | boost::adaptors::map_values)
                {                    
                    PRX_INFO_S("execute planner: " << planner->get_name());
                    planner->execute();
                    // planner->update_visualization();
                }
                left_manipulation_query = new manipulation_query_t();
                right_manipulation_query = new manipulation_query_t();

                manipulation_model->use_context(left_context_name);
                manipulator = manipulation_model->get_current_manipulation_info()->manipulator;
                full_manipulator_state_space = manipulation_model->get_current_manipulation_info()->full_manipulator_state_space;
                full_manipulator_control_space = manipulation_model->get_current_manipulation_info()->full_manipulator_control_space;

                manipulation_specification_t* manip_spec = dynamic_cast<manipulation_specification_t*>(output_specifications[manipulation_task_planner_name]);
                left_manipulation_query->path_constraints = manip_spec->validity_checker->alloc_constraint();
                left_manipulation_query->set_valid_constraints(manip_spec->validity_checker->alloc_constraint());
                left_manipulation_query->set_search_mode(LAZY_SEARCH);
                right_manipulation_query->path_constraints = manip_spec->validity_checker->alloc_constraint();
                right_manipulation_query->set_valid_constraints(manip_spec->validity_checker->alloc_constraint());
                right_manipulation_query->set_search_mode(LAZY_SEARCH);

                return true;
            }

            bool apc_task_planner_t::succeeded() const
            {
                return true;
            }

            const util::statistics_t* apc_task_planner_t::get_statistics()
            {
                return NULL;
            }
            bool apc_task_planner_t::move()
            {
                // debug

                manipulation_specification_t* manip_spec = dynamic_cast<manipulation_specification_t*>(output_specifications[manipulation_task_planner_name]);

                PRX_INFO_S("Move");
                //taking a full manipulator state, move both arms in sequence to that position
                state_t* full_init = full_manipulator_state_space->alloc_point();
                manipulation_model->use_context(left_context_name);
                space_t* left_space =  manipulation_model->get_state_space();
                manipulation_model->use_context(right_context_name);
                space_t* right_space =  manipulation_model->get_state_space();


                state_t* initial_state_left = left_space->alloc_point();
                state_t* left_goal = left_space->alloc_point();
                state_t* initial_state_right = right_space->alloc_point();
                state_t* right_goal = right_space->alloc_point();


                manipulation_model->convert_spaces(left_space,left_goal,full_manipulator_state_space,in_query->goal_state);
                //in order to make sure the right movement takes into account any movement on the left, this update is needed
                full_manipulator_state_space->copy_from_point(full_init);
                manipulation_model->convert_spaces(right_space,initial_state_right,left_space,left_goal);
                manipulation_model->convert_spaces(right_space,right_goal,full_manipulator_state_space,in_query->goal_state);

                full_manipulator_state_space->free_point(full_init);
                //left hand
                manipulation_model->use_context(left_context_name);
                PRX_PRINT("Initial: "<<manipulation_model->get_state_space()->print_point(initial_state_left),PRX_TEXT_GREEN);
                PRX_PRINT("Target: "<<manipulation_model->get_state_space()->print_point(left_goal),PRX_TEXT_GREEN);
                left_manipulation_query->setup_move(manipulation_model->get_current_context(),initial_state_left,left_goal);
                left_manipulation_query->plan.clear();
                manip_planner->link_query(left_manipulation_query);

                resolve_timer.restart();
                manip_planner->resolve_query();
                total_resolve_time += resolve_timer.elapsed();

                in_query->move_plan.clear();

                if(left_manipulation_query->found_path)
                {
                    manipulation_model->convert_plan(in_query->move_plan, manipulator->get_control_space(), left_manipulation_query->plan, manipulation_model->get_control_space());
                    manipulation_model->get_state_space()->free_point(initial_state_left);
                    manipulation_model->get_state_space()->free_point(left_goal);
                    plan_t plan(manipulation_model->get_control_space());
                    manipulation_model->engage_grasp(plan,manipulation_model->get_current_manipulation_info()->end_effector_state_space->at(0),manipulation_model->get_current_manipulation_info()->is_end_effector_closed());
                }
                else
                {
                    manipulation_model->get_state_space()->free_point(initial_state_left);
                    manipulation_model->get_state_space()->free_point(left_goal);
                    manipulation_model->get_state_space()->free_point(initial_state_right);
                    manipulation_model->get_state_space()->free_point(right_goal);
                    return false;

                }

                PRX_DEBUG_COLOR(" PRE CHECK Validity checker is null? " << (manip_spec->validity_checker == NULL ? "YES" : "NO"), PRX_TEXT_RED);

                //right hand
                manipulation_model->use_context(right_context_name);
                right_manipulation_query->setup_move(manipulation_model->get_current_context(),initial_state_right,right_goal);
                right_manipulation_query->plan.clear();
                manip_planner->link_query(right_manipulation_query);
                resolve_timer.restart();
                manip_planner->resolve_query();
                total_resolve_time += resolve_timer.elapsed();

                if(right_manipulation_query->found_path)
                {
                    manipulation_model->convert_plan(in_query->move_plan, manipulator->get_control_space(), right_manipulation_query->plan, manipulation_model->get_control_space());
                    plan_t plan(manipulation_model->get_control_space());
                    manipulation_model->engage_grasp(plan,manipulation_model->get_current_manipulation_info()->end_effector_state_space->at(0),manipulation_model->get_current_manipulation_info()->is_end_effector_closed());

                    PRX_INFO_S("Move 4");
                    manipulation_model->get_state_space()->free_point(initial_state_right);
                    PRX_INFO_S("Move 4.1");
                    manipulation_model->get_state_space()->free_point(right_goal);
                    PRX_INFO_S("Move 4.2");
                    PRX_DEBUG_COLOR(" POST CHECK Validity checker is null? " << (manip_spec->validity_checker == NULL ? "YES" : "NO"), PRX_TEXT_RED);
                    return true;
                }
                else
                {

                PRX_INFO_S("Move5");
                    manipulation_model->get_state_space()->free_point(initial_state_right);
                    manipulation_model->get_state_space()->free_point(right_goal);
                PRX_DEBUG_COLOR(" POST CHECK Validity checker is null? " << (manip_spec->validity_checker == NULL ? "YES" : "NO"), PRX_TEXT_RED);
                    return false;
                }

            }

            bool apc_task_planner_t::move_and_detect()
            {
                double gripper_offset=0.0;
                if(in_query->hand=="left")
                {
                    // manipulation_model->use_context(left_camera_context_name);
                    manipulation_model->use_context(left_context_name);
                    // gripper_offset=0.075;
                }
                else
                {
                    // manipulation_model->use_context(right_camera_context_name);   
                    manipulation_model->use_context(right_context_name);
                    gripper_offset=0.0;
                }


                //////////////////////////////////NEW IK STEERING TEST
                if(false)
                {
                    int number_of_states = 500;
                    std::vector<bounds_t*> bounds;
                    trajectory_t valid_states(manipulation_model->get_state_space());
                    {
                        for( unsigned i = 0; i < 3; i++ )
                        {
                            bounds.push_back(new bounds_t());
                        }
                        bounds[0]->set_bounds(.7,1.38);
                        bounds[1]->set_bounds(-.14,.14);
                        bounds[2]->set_bounds(1.35,1.55);
                        state_t* arm_start = manipulation_model->get_state_space()->alloc_point();
                        config_t start_config;

                        while(valid_states.size()<number_of_states)
                        {
                            manipulation_model->get_state_space()->uniform_sample(arm_start);
                            manipulation_model->get_state_space()->copy_from_point(arm_start);
                            manipulation_model->FK(start_config);
                            double x,y,z;
                            start_config.get_position(x,y,z);
                            if(true)//bounds[0]->is_valid(x) && bounds[1]->is_valid(y) && bounds[2]->is_valid(z))
                            {
                                bool colliding = !manipulation_model->valid_state(arm_start);
                                if(!colliding)
                                {
                                    valid_states.copy_onto_back(arm_start);
                                    PRX_STATUS(valid_states.size(),PRX_TEXT_GREEN);
                                }
                            }
                            PRX_STATUS(valid_states.size(),PRX_TEXT_RED);
                        }
                    }
                    valid_states.save_to_file("/home/zak/Desktop/valid_states.txt");
                    int num_edges = 20;
                    std::vector<data_ik> data(num_edges*number_of_states);
                    typedef std::pair<state_t*,double> stupid_boost;  
                    std::vector<std::pair<state_t*,double> > connect_states;
                    int data_count=0;
                    boost::timer ik_steering_timer;
                    state_t* result_state = manipulator->get_state_space()->alloc_point();
                    state_t* seed_state = manipulator->get_state_space()->alloc_point();
                    state_t* initial_state = manipulator->get_state_space()->alloc_point();
                    trajectory_t traj(manipulation_model->get_state_space());
                    foreach(state_t* state, valid_states)
                    {
                        connect_states.clear();
                        //find n closest states
                        foreach(state_t* state2, valid_states)
                        {
                            if(state2!=state)
                            {
                                double dist = manipulation_model->get_state_space()->distance(state,state2);
                                if(connect_states.size()<num_edges/2)
                                {
                                    connect_states.push_back(std::make_pair(state2,dist));
                                }
                                else if(connect_states.back().second > dist)
                                {
                                    connect_states.back().first = state2;
                                    connect_states.back().second = dist;
                                }
                                //swap up
                                bool done=false;
                                for(int b_iter=connect_states.size()-1;b_iter>=1 && !done;b_iter--)
                                {
                                    if(connect_states[b_iter].second < connect_states[b_iter-1].second)
                                    {
                                        std::swap(connect_states[b_iter],connect_states[b_iter-1]);
                                    }
                                    else
                                        done = true;
                                }  
                            }
                        }

                        foreach(state_t* state2, valid_states)
                        {
                            if(state2!=state)
                            {
                                double dist = manipulation_model->get_state_space()->distance(state,state2);
                                if(connect_states.size()<num_edges)
                                {
                                    connect_states.push_back(std::make_pair(state2,dist));
                                }
                                else if(connect_states.back().second < dist)
                                {
                                    connect_states.back().first = state2;
                                    connect_states.back().second = dist;
                                }
                                //swap up
                                bool done=false;
                                for(int b_iter=connect_states.size()-1;b_iter>=num_edges/2+1 && !done;b_iter--)
                                {
                                    if(connect_states[b_iter].second > connect_states[b_iter-1].second)
                                    {
                                        std::swap(connect_states[b_iter],connect_states[b_iter-1]);
                                    }
                                    else
                                        done = true;
                                }  
                            }
                        }

                        //now loop over the states and try to connect them
                        foreach(stupid_boost p, connect_states)
                        {
                            manipulation_model->get_state_space()->copy_from_point(p.first);
                            manipulator->get_state_space()->copy_to_point(seed_state);
                            manipulation_model->get_state_space()->copy_from_point(state);
                            manipulator->get_state_space()->copy_to_point(initial_state);


                            manipulator->get_state_space()->copy_from_point(seed_state);
                            config_t goal_config;
                            config_t start_config;
                            manipulation_model->FK(goal_config);
                            manipulator->get_state_space()->copy_from_point(initial_state);
                            manipulation_model->FK(start_config);
                            data[data_count].ee_distance = sqrt( (start_config.get_position()[0] - goal_config.get_position()[0] ) * (start_config.get_position()[0] - goal_config.get_position()[0] ) + 
                                                        (start_config.get_position()[1] - goal_config.get_position()[1] ) * (start_config.get_position()[1] - goal_config.get_position()[1] ) + 
                                                        (start_config.get_position()[2] - goal_config.get_position()[2] ) * (start_config.get_position()[2] - goal_config.get_position()[2] ))
                                                + goal_config.get_orientation().distance(start_config.get_orientation());

                            ik_steering_timer.restart();
                            in_query->move_gripper_to_bin.clear();
                            // bool res = true;
                            // manipulator->steering_function(initial_state,seed_state,in_query->move_gripper_to_bin);
                            bool res = manipulator->IK_steering(in_query->move_gripper_to_bin, result_state, initial_state, goal_config, manipulation_model->get_current_manipulation_info()->chain.first, manipulation_model->get_current_manipulation_info()->chain.second);
                            data[data_count].time_elapsed=ik_steering_timer.elapsed();
                            data[data_count].success = res;
                            if(res)
                            {
                                double err=0;
                                manipulator->get_state_space()->copy_from_point(result_state);
                                for(int i=0;i<manipulation_model->get_state_space()->get_dimension();i++)
                                {
                                    err+=pow(manipulation_model->get_state_space()->at(i) ,2.0);
                                }

                                data[data_count].error = sqrt(err);
                                data[data_count].plan_length = in_query->move_gripper_to_bin.length();
                                traj.clear();
                                manipulation_query->plan.clear();
                                manipulation_model->convert_plan(manipulation_query->plan, manipulation_model->get_control_space(),in_query->move_gripper_to_bin, manipulator->get_control_space() );
                                manipulation_model->propagate_plan(state,manipulation_query->plan,traj);
                                bool collision_free = true;
                                ik_steering_timer.restart();
                                int count=0;
                                for (trajectory_t::iterator s = traj.begin(); s != traj.end() && collision_free; ++s)
                                {
                                    collision_free = collision_free && manipulation_model->valid_state(*s);
                                    count++;
                                }
                                data[data_count].collision_time = ik_steering_timer.elapsed();
                                data[data_count].collision_free = collision_free;
                            }
                            PRX_STATUS(data_count++,PRX_TEXT_RED);
                        }

                    }


                    {
                        double time_elapsed=0;
                        double num_successes=0;
                        double collision_free=0;
                        double avg_plan_length=0;
                        double avg_error=0;
                        double avg_collision_time = 0;
                        for (int iter = 0; iter < data.size(); ++iter)
                        {
                            if(iter%(num_edges)<num_edges/2)
                            {
                                time_elapsed+=data[iter].time_elapsed;
                                if(data[iter].success)
                                {
                                    num_successes++;
                                    collision_free+=data[iter].collision_free;
                                    avg_plan_length+=data[iter].plan_length;
                                    avg_error+=data[iter].error;
                                    avg_collision_time+=data[iter].collision_time;
                                }
                            }
                        }
                        PRX_PRINT("\nSteer Successes: "<<num_successes<<"/"<<data.size()/2
                                <<"\n"<<"In collision: "<<num_successes-collision_free<<"/"<<num_successes
                                <<"\n"<<"Successful connections: "<<collision_free<<"/"<<data.size()/2
                                <<"\n"<<"Total Steer time for "<<data.size()/2<<" attempts: "<<time_elapsed<<" sec"
                                <<"\nAvg Plan Duration out of "<<num_successes<<" plans: "<<avg_plan_length/num_successes <<" sec"
                                <<"\nAvg Error out of "<<num_successes<<" end states: "<<avg_error/num_successes
                                <<"\nCollision Time for all "<<num_successes<<" paths: "<<avg_collision_time<<" sec",PRX_TEXT_RED);
                    }
                    {
                        double time_elapsed=0;
                        double num_successes=0;
                        double collision_free=0;
                        double avg_plan_length=0;
                        double avg_error=0;
                        double avg_collision_time = 0;
                        for (int iter = 0; iter < data.size(); ++iter)
                        {
                            if(iter%(num_edges)>=num_edges/2)
                            {
                                time_elapsed+=data[iter].time_elapsed;
                                if(data[iter].success)
                                {
                                    num_successes++;
                                    collision_free+=data[iter].collision_free;
                                    avg_plan_length+=data[iter].plan_length;
                                    avg_error+=data[iter].error;
                                    avg_collision_time+=data[iter].collision_time;
                                }
                            }
                        }
                        PRX_FATAL_S("\nSteer Successes: "<<num_successes<<"/"<<data.size()/2
                                <<"\n"<<"In collision: "<<num_successes-collision_free<<"/"<<num_successes
                                <<"\n"<<"Successful connections: "<<collision_free<<"/"<<data.size()/2
                                <<"\n"<<"Total Steer time for "<<data.size()/2<<" attempts: "<<time_elapsed<<" sec"
                                <<"\nAvg Plan Duration out of "<<num_successes<<" plans: "<<avg_plan_length/num_successes <<" sec"
                                <<"\nAvg Error out of "<<num_successes<<" end states: "<<avg_error/num_successes
                                <<"\nCollision Time for all "<<num_successes<<" paths: "<<avg_collision_time<<" sec");
                    }


                }
                //////////////////////////////////////////////////////



                
                PRX_INFO_S("Move and Detect: "<<manipulation_model->get_current_context());
                PRX_INFO_S("QUAT = "<<camera_positions[in_query->bin]->base_viewpoint);
                //TODO move to the first bin position
                //TODO move to second mapping position
                //TODO move to third mapping position
                //prepare for the grasp
                config_t gripper_config;
                gripper_config.set_position(camera_positions[in_query->bin]->focus[0]-gripper_offset,camera_positions[in_query->bin]->focus[1],camera_positions[in_query->bin]->focus[2]);
                gripper_config.set_orientation(camera_positions[in_query->bin]->base_viewpoint);
                PRX_INFO_S("Test 1");

                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();
                state_t* result_state = manipulation_model->get_state_space()->alloc_point();
                state_t* seed_state = manipulation_model->get_state_space()->alloc_point();


                // state_t* initial_state = manipulator->get_state_space()->alloc_point();
                // state_t* result_state = manipulator->get_state_space()->alloc_point();
                // state_t* seed_state = manipulator->get_state_space()->alloc_point();
                // bool res = manipulation_model->get_current_manipulation_info()->manipulator->IK_steering(in_query->move_gripper_to_bin, result_state, initial_state, gripper_config, manipulation_model->get_current_manipulation_info()->chain.first, manipulation_model->get_current_manipulation_info()->chain.second);

                PRX_INFO_S("Test 2");
                int i=0;
                bool found = false;
                while(i<100 && !found)
                {
                    found = manipulation_model->IK(result_state,seed_state,gripper_config);
                    manipulation_model->get_state_space()->uniform_sample(seed_state);
                    PRX_DEBUG_COLOR("SEED: "<<seed_state<<"  i = "<<i,PRX_TEXT_RED);
                    i++;
                }
                if(!found)
                {
                    manipulation_model->get_state_space()->free_point(initial_state);
                    manipulation_model->get_state_space()->free_point(result_state);
                    manipulation_model->get_state_space()->free_point(seed_state);
                    return false;
                }
                PRX_INFO_S("Test 3");
                if(in_query->hand=="left")
                {
                    manipulation_model->use_context(left_context_name);
                }
                else
                {
                    manipulation_model->use_context(right_context_name);
                }
                PRX_INFO_S("Move and Detect: "<<manipulation_model->get_current_context());
                manipulation_query->setup_move(manipulation_model->get_current_context(),initial_state,result_state);
                manipulation_query->plan.clear();
                manip_planner->link_query(manipulation_query);
                manip_planner->resolve_query();
                in_query->move_gripper_to_bin.clear();

                manipulation_model->get_state_space()->free_point(initial_state);
                manipulation_model->get_state_space()->free_point(result_state);
                manipulation_model->get_state_space()->free_point(seed_state);
                bool success = manipulation_query->found_path;
                if(manipulation_query->found_path)
                {
                    manipulation_model->convert_plan(in_query->move_gripper_to_bin, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                }
                return success;
            }

            bool apc_task_planner_t::perform_grasp()
            {
                PRX_INFO_S("Perform Grasp");
                //pick the object 
                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();
                state_t* result_state = manipulation_model->get_state_space()->alloc_point();
                const space_t* object_space = in_query->object->get_state_space();
                state_t* stored_object_state = object_space->alloc_point();
                state_t* initial_object = object_space->alloc_point();

                //left arm pick and place
                config_t retract_config;
                if(in_query->hand=="left")
                {
                    retract_config.set_position(0,0,-.03);
                    retract_config.set_orientation(0,0,0,1);
                    manipulation_model->use_context(left_context_name);
                }
                else
                {
                    retract_config.set_position(0,0,-.07);
                    retract_config.set_orientation(0,0,0,1);
                    manipulation_model->use_context(right_context_name);
                }


                // int nr_grasps = manip_planner->get_nr_grasps(manipulation_model->get_current_context(),in_query->object);
                // std::vector<int> indices;
                // for (int i = 0; i < nr_grasps; ++i)
                // {
                //     indices.push_back(i);
                // }
                // for (int i = 0; i < 9001; ++i)
                // {
                //     unsigned i1,i2;
                //     i1 = uniform_int_random(0,nr_grasps-1);
                //     i2 = uniform_int_random(0,nr_grasps-1);
                //     std::swap(indices[i1],indices[i2]);
                // }

                bool success = false;
                int previous_grasp_mode = manipulation_model->get_current_grasping_mode();
                // for( unsigned i=0; i<indices.size() && !success; ++i )
                // {

                //TODO FIX

                // manipulation_query->setup_pick(manipulation_model->get_current_context(), manipulation_query_t::PRX_GRASP_GREEDY, in_query->object, (in_query->hand=="left"?1:3), retract_config,initial_state,result_state,initial_object );
                // manipulation_query->setup(manipulation_model->get_current_context(), manipulation_query_t::PRX_PICK_AND_RETRACT, manipulation_query_t::PRX_GRASP_GREEDY, in_query->object, (in_query->hand=="left"?1:3), retract_config, initial_state, NULL, result_state, initial_object, NULL, NULL, false);
                // manipulation_query->setup_pick( manipulation_model->get_current_context(), true, GRASP_EXHAUSTIVE, in_query->object, (in_query->hand=="left"?1:3), retract_config, initial_state, initial_object);


                state_t* final_state = manipulation_model->get_state_space()->alloc_point();

                state_t* arm_final = manipulation_model->get_current_manipulation_info()->arm_state_space->alloc_point();
                if(in_query->hand == "left")
                    manipulation_model->get_current_manipulation_info()->arm_state_space->set_from_vector(left_arm_order_bin,arm_final);
                else
                    manipulation_model->get_current_manipulation_info()->arm_state_space->set_from_vector(right_arm_order_bin,arm_final);

                manipulation_model->convert_spaces(manipulation_model->get_state_space(),final_state,manipulation_model->get_current_manipulation_info()->arm_state_space,arm_final);

                manipulation_model->get_current_manipulation_info()->arm_state_space->free_point(arm_final);


                manipulation_query->setup_pick_and_move( manipulation_model->get_current_context(), GRASP_EXHAUSTIVE, in_query->object, (in_query->hand=="left"?1:3), retract_config, initial_state, final_state, initial_object);
                manipulation_query->ik_steer_paths = true;
                manipulation_query->plan.clear();
                manip_planner->link_query(manipulation_query);
                manip_planner->resolve_query();
                success = manipulation_query->found_path;
                // }
                if(success)
                {
                    
                    PRX_PRINT("\n\n\n\n\n\n\n\n\n\n\n\n\n WE GOT A PICK \n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n",PRX_TEXT_GREEN);
                    manipulation_model->convert_plan(in_query->retrieve_object, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                    // config_t zero_config;
                    // zero_config.zero();

                    // in_query->approach_object.clear();
                    // manipulation_model->convert_plan(in_query->approach_object, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                    
                    // initial_object->at(2) = initial_object->at(2) +0.02;
                    // // manipulation_query->setup_transfer(manipulation_model->get_current_context(), in_query->object, result_state, initial_object );
                    // manipulation_query->setup_place( manipulation_model->get_current_context(), false, in_query->object, (in_query->hand=="left"?1:3), zero_config, result_state, initial_object);         
                    // manipulation_query->ik_steer_paths = true;
                    // manipulation_query->plan.clear();
                    // manip_planner->link_query(manipulation_query);
                    // manip_planner->resolve_query();

                    // in_query->retrieve_object.clear();
                    // if(manipulation_query->found_path)
                    // {
                    //     manipulation_model->get_state_space()->copy_to_point(result_state);

                    //     manipulation_model->convert_plan(in_query->retrieve_object, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                    //     PRX_PRINT("Succeeded to retract z->0.2", PRX_TEXT_MAGENTA);
                    // }
                    // else
                    // {
                    //     failedmanualretr1++;
                    //     PRX_PRINT("Failed manual retraction 1", PRX_TEXT_MAGENTA);
                    //     initial_object->at(0) = initial_object->at(0) - 0.02;
                    //     // manipulation_query->setup_transfer(manipulation_model->get_current_context(), in_query->object, result_state, initial_object );
                    //     manipulation_query->setup_place( manipulation_model->get_current_context(), false, in_query->object, (in_query->hand=="left"?1:3), zero_config, result_state, initial_object);         
                    //     manipulation_query->ik_steer_paths = true;
                    //     manipulation_query->plan.clear();
                    //     manip_planner->link_query(manipulation_query);
                    //     manip_planner->resolve_query();
                    //     if(manipulation_query->found_path)
                    //     {
                    //         manipulation_model->get_state_space()->copy_to_point(result_state);
                    //         manipulation_model->convert_plan(in_query->retrieve_object, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                    //         PRX_PRINT("Succeeded to retract z->0.2 x->-0.2", PRX_TEXT_MAGENTA);
                    //     }
                    //     else
                    //     {
                    //         failedmanualretr2++;
                    //         PRX_PRINT("Failed manual retraction 2", PRX_TEXT_MAGENTA);
                    //     }
                    // }

                    // //retract from the shelf
                    // initial_object->at(0) = camera_positions[in_query->bin]->focus[0];
                    // initial_object->at(1) = camera_positions[in_query->bin]->focus[1];
                    // initial_object->at(2) = camera_positions[in_query->bin]->focus[2];
                    // // manipulation_query->setup_transfer(manipulation_model->get_current_context(), in_query->object, result_state, initial_object );
                    // manipulation_query->setup_place( manipulation_model->get_current_context(), false, in_query->object, (in_query->hand=="left"?1:3), zero_config, result_state, initial_object);         
                    // manipulation_query->ik_steer_paths = true;
                    // manipulation_query->plan.clear();
                    // manip_planner->link_query(manipulation_query);
                    // manip_planner->resolve_query();
                    // if(manipulation_query->found_path)
                    // {
                    //     manipulation_model->convert_plan(in_query->retrieve_object, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                    //     PRX_PRINT("Transfer worked!", PRX_TEXT_MAGENTA);
                    // }
                    // else
                    // {
                    //     transferfailed++;
                    //     manipulation_query->setup_move(manipulation_model->get_current_context(),result_state,initial_state);
                    //     manipulation_query->plan.clear();
                    //     manip_planner->link_query(manipulation_query);
                    //     manip_planner->resolve_query();
                    //     if(manipulation_query->found_path)
                    //     {
                    //         manipulation_model->convert_plan(in_query->retrieve_object, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                    //         PRX_PRINT("Move worked!", PRX_TEXT_MAGENTA);
                    //     }
                    //     else
                    //     {
                    //         movefailed++;
                    //         PRX_INFO_S(manipulation_model->get_full_state_space()->print_memory(1));
                    //         PRX_INFO_S("Failed to retrieve object...releasing the grasp by engaging mode:  "<<previous_grasp_mode);
                    //         manipulation_model->engage_grasp(manipulation_query->plan,previous_grasp_mode,false);
                    //         object_space->copy_from_point(stored_object_state);
                    //         PRX_INFO_S(manipulation_model->get_full_state_space()->print_memory(1));
                    //         success = false;
                    //     }
                    // }
                }
                std::cout<<"Failed Manual Retraction1: "<<failedmanualretr1<<std::endl;
                std::cout<<"Failed Manual Retraction2: "<<failedmanualretr2<<std::endl;
                std::cout<<"Transfer Failed: "<<transferfailed<<std::endl;
                std::cout<<"Move Failed: "<<movefailed<<std::endl;
                //cleanup
                manipulation_model->get_state_space()->free_point(initial_state);
                manipulation_model->get_state_space()->free_point(result_state);
                manipulation_model->get_state_space()->free_point(final_state);
                object_space->free_point(initial_object);
                object_space->free_point(stored_object_state);
                return success;
            }

            bool apc_task_planner_t::move_to_order_bin()
            {
                PRX_INFO_S("Move to order bin");
                //move the object to the target location
                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();
                state_t* final_state = manipulation_model->get_state_space()->alloc_point();
                state_t* arm_final = manipulation_model->get_current_manipulation_info()->arm_state_space->alloc_point();
                if(in_query->hand == "left")
                    manipulation_model->get_current_manipulation_info()->arm_state_space->set_from_vector(left_arm_order_bin,arm_final);
                else
                    manipulation_model->get_current_manipulation_info()->arm_state_space->set_from_vector(right_arm_order_bin,arm_final);

                manipulation_model->convert_spaces(manipulation_model->get_state_space(),final_state,manipulation_model->get_current_manipulation_info()->arm_state_space,arm_final);

                manipulation_model->get_current_manipulation_info()->arm_state_space->free_point(arm_final);

                manipulation_query->setup_move(manipulation_model->get_current_context(),initial_state,final_state);
                manipulation_query->plan.clear();
                manip_planner->link_query(manipulation_query);

                manip_planner->resolve_query();
                //release the object
                manipulation_model->engage_grasp(manipulation_query->plan,1,false);
                in_query->move_to_order_bin.clear();
                bool success = manipulation_query->found_path;
                if(manipulation_query->found_path)
                {
                    manipulation_model->convert_plan(in_query->move_to_order_bin, manipulator->get_control_space(), manipulation_query->plan, manipulation_model->get_control_space());
                }

                //cleanup
                manipulation_model->get_state_space()->free_point(initial_state);
                manipulation_model->get_state_space()->free_point(final_state);
                return success;
            }

            bool apc_task_planner_t::test_bin_roadmap()
            {
                //Randomly populate the initial state with a conf inside the bin
                PRX_PRINT("Starting Test Bin Roadmap...", PRX_TEXT_RED);
                state_t* initial_state = manipulation_model->get_state_space()->alloc_point();
                state_t* seed_state = manipulator->get_state_space()->alloc_point();
                util::quaternion_t quat;
                quat.set(0,1,0,1);
                quat.normalize();
                config_t gripper_config;
                
                int i=0;
                bool found = false;
                while(i<100 && !found)
                {
                    gripper_config.set_position( double(rand() % 550 + 850)/1000,double(rand() % 140 - 140)/1000,double(rand() % 230 + 1370)/1000);
                    gripper_config.set_orientation(quat);
                    PRX_PRINT("Gripper Config: "<<gripper_config.print(), PRX_TEXT_RED);
                    found = manipulation_model->IK(initial_state,seed_state,gripper_config);
                    manipulation_model->get_state_space()->uniform_sample(seed_state);
                    // PRX_PRINT("SEED: "<<i,PRX_TEXT_RED);
                    i++;
                }
                state_t* final_state = manipulation_model->get_state_space()->alloc_point();
                manipulation_model->push_state(initial_state);
                PRX_PRINT("Initial State: "<<manipulation_model->get_state_space()->print_point(initial_state,3),PRX_TEXT_MAGENTA);
                //Populate the final state with a configuration outside the bin
                double f_state_arr[] = {-1.48441,-0.95249,-0.5822,-1.86202,-1.2274,-0.5354,-0.36095,-0.8788,1.0459, 1.0};
                std::vector<double> f_state_vec(f_state_arr, f_state_arr + sizeof(f_state_arr) / sizeof(double) );
                manipulation_model->get_state_space()->copy_vector_to_point(f_state_vec, final_state); 
                PRX_PRINT("Final State: "<<manipulation_model->get_state_space()->print_point(final_state,3),PRX_TEXT_MAGENTA);
                //setup the move 
                //left hand
                manipulation_model->use_context(left_context_name);
                left_manipulation_query->setup_move(manipulation_model->get_current_context(),initial_state,final_state);
                left_manipulation_query->plan.clear();
                manip_planner->link_query(left_manipulation_query);

                resolve_timer.restart();
                manip_planner->resolve_query();
                total_resolve_time += resolve_timer.elapsed();

                in_query->move_plan.clear();
                bool success = manipulation_query->found_path;
                if(left_manipulation_query->found_path)
                {
                    manipulation_model->convert_plan(in_query->move_plan, manipulator->get_control_space(), left_manipulation_query->plan, manipulation_model->get_control_space());
                    manipulation_model->get_state_space()->free_point(initial_state);
                    manipulation_model->get_state_space()->free_point(final_state);
                    plan_t plan(manipulation_model->get_control_space());
                    manipulation_model->engage_grasp(plan,manipulation_model->get_current_manipulation_info()->end_effector_state_space->at(0),manipulation_model->get_current_manipulation_info()->is_end_effector_closed());
                }
                PRX_PRINT("Finished Test Bin Roadmap...", PRX_TEXT_RED);
                return success;

            }

            void apc_task_planner_t::resolve_query()
            {
                if(in_query->stage == apc_task_query_t::MOVE)
                {
                    move_timer.restart();
                    in_query->found_solution = move();
                    total_move_time +=move_timer.elapsed();
                    total_tries++;
                    if(in_query->found_solution == false)
                    {
                        total_failures++;
                    }
                    if (total_tries>=MAX_TRIES)
                    {
                        PRX_FATAL_S("Total Resolve Time: "<<total_resolve_time<<"   Total Move Time: "<<total_move_time<<"   Total Failures: "<<total_failures <<"/"<<MAX_TRIES);
                    }
                }
                else
                {
                    PRX_INFO_S("APC Query requests use of the "<<in_query->hand<<" hand.");
                    if(in_query->hand=="left")
                    {
                        manipulation_model->use_context(left_context_name);
                        manipulation_query = left_manipulation_query;
                    }
                    else if(in_query->hand=="right")
                    {
                        manipulation_model->use_context(right_context_name);
                        manipulation_query = right_manipulation_query;
                    }
                    else
                    {
                        PRX_FATAL_S("Requested hand in the apc_task_query is neither left nor right.");
                    }

                    if(in_query->stage == apc_task_query_t::MOVE_AND_DETECT)
                    {
                        in_query->found_solution = move_and_detect();
                    }
                    else if(in_query->stage == apc_task_query_t::PERFORM_GRASP)
                    {
                        in_query->found_solution = perform_grasp();
                    }
                    else if(in_query->stage == apc_task_query_t::MOVE_TO_ORDER_BIN)
                    {
                        in_query->found_solution = move_to_order_bin();
                    }
                    else if(in_query->stage == apc_task_query_t::TEST_BIN_ROADMAP)
                    {
                        in_query->found_solution = test_bin_roadmap();   
                    }
                    else
                    {
                        PRX_FATAL_S("Invalid apc task query stage: "<<in_query->stage);
                    }     
                }                  
            }
        }
    }
}

