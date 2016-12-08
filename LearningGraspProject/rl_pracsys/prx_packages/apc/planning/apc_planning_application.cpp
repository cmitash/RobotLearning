/**
 * @file apc_planning_application.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "prx/planning/task_planners/task_planner.hpp"
#include "prx/planning/communication/planning_comm.hpp"
#include "planning/apc_planning_application.hpp"

#include "prx_simulation/object_msg.h"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include <pluginlib/class_list_macros.h>

#include <sstream>

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

            PLUGINLIB_EXPORT_CLASS( prx::packages::apc::apc_planning_application_t, prx::plan::planning_application_t)

            apc_planning_application_t::apc_planning_application_t()
            {
                total_resolves = 0;
            }

            apc_planning_application_t::~apc_planning_application_t() { }

            void apc_planning_application_t::init(const parameter_reader_t* reader)
            {
                planning_application_t::init(reader);
                full_manipulator_context_name = parameters::get_attribute("full_manipulator_context_name", reader,NULL);
                manipulation_model = dynamic_cast<manipulation_world_model_t*>(model);
                manipulation_model->use_context(full_manipulator_context_name);
                output_query = new apc_task_query_t(manipulation_model->get_control_space());
                current_manipulator_state = manipulation_model->get_state_space()->alloc_point();
                propagated_manipulator_state = manipulation_model->get_state_space()->clone_point(current_manipulator_state);
                plan_counter = 0;

            }

            void apc_planning_application_t::execute()
            {
                // sleep(10);
                std::vector<movable_body_plant_t* > objects;
                manipulation_model->get_objects(objects);
                std::vector<double> dummy_state;
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(3);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(0);
                dummy_state.push_back(1);
                object_publisher = n.advertise<prx_simulation::object_msg>("prx/object_state", 100);
                foreach(movable_body_plant_t* manip_obj, objects)
                {
                    manip_obj->get_state_space()->set_from_vector(dummy_state);
                    prx_simulation::object_msg object_placement;
                    object_placement.name = manip_obj->get_object_type();
                    for(unsigned i=0;i<7;i++)
                    {
                        object_placement.elements.push_back(dummy_state[i]);
                    }
                    object_publisher.publish(object_placement);    
                    PRX_INFO_S("Publish "<<object_placement.name);
                    dummy_state[0]+=2;
                    dummy_state[1]+=2;
                    dummy_state[2]+=2;
                    ros::spinOnce();

                }

                manipulation_model->use_context(full_manipulator_context_name);
                prx_simulation::send_trajectoryGoal command;

                this->root_task->link_specification(root_specifications[0]);
                this->root_task->setup();
                this->root_task->execute();
                output_query->clear();

                state_subscriber = n.subscribe("prx/manipulator_state", 1, &apc_planning_application_t::get_state_callback,this);
                execute_client = new send_traj_client("prx/execute_plan_action",false);
                q_server = new query_server(n,"prx/apc_action", boost::bind(&apc_planning_application_t::resolve_query, this, _1), false);
                q_server->start();
            }

            void apc_planning_application_t::resolve_query(const prx_planning::apc_queryGoalConstPtr& req)
            {
                // PRX_INFO_S("\n\n\n\n\n TOTAL COMPLETED: "<<total_resolves++<<"\n\n\n\n\n\n\n\n");
                // PRX_PRINT("\nStart: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);
                prx_simulation::send_trajectoryGoal command;

                bool found_ac_server = false;
                while( !found_ac_server )
                {
                    found_ac_server = execute_client->waitForServer(ros::Duration(1));
                }
                manipulation_model->use_context(full_manipulator_context_name);
                manipulation_model->get_state_space()->copy_from_point(current_manipulator_state);
                manipulation_model->get_state_space()->copy_to_point(propagated_manipulator_state);
                manipulation_model->get_state_space()->copy_from_point(propagated_manipulator_state);
                // PRX_PRINT("\nStart: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);

                if(req->stage == apc_task_query_t::MOVE)
                {
                    state_t* goal_state = manipulation_model->get_state_space()->alloc_point();
                    manipulation_model->get_state_space()->set_from_vector(req->goal_state,goal_state);
                    //move only, bin, hand, and object don't matter
                    output_query->setup("",NULL,apc_task_query_t::MOVE,'A',goal_state);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();
                    // PRX_PRINT("\nStar2: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);

                    if(output_query->found_solution)
                    {
                        convert_to_plan_msg(output_query->move_plan,command);
                        send_command(command,output_query->move_plan.length());

                        PRX_PRINT("Executing plan: "<<output_query->move_plan.print(2), PRX_TEXT_MAGENTA);
                        manipulation_model->use_context(full_manipulator_context_name);
                        manipulation_model->get_state_space()->free_point(goal_state);
                        q_server->setSucceeded();
                    }
                    else
                    {
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::MOVE_AND_DETECT)
                {
                    //move and detect
                    output_query->setup(req->hand,NULL,apc_task_query_t::MOVE_AND_DETECT,req->bin[0],NULL);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        convert_to_plan_msg(output_query->first_mapping_plan,command);
                        send_command(command,output_query->first_mapping_plan.length());

                        convert_to_plan_msg(output_query->second_mapping_plan,command);
                        send_command(command,output_query->second_mapping_plan.length());

                        convert_to_plan_msg(output_query->third_mapping_plan,command);
                        send_command(command,output_query->third_mapping_plan.length());

                        convert_to_plan_msg(output_query->move_gripper_to_bin,command);
                        send_command(command,output_query->move_gripper_to_bin.length());
                        q_server->setSucceeded();
                    }
                    else
                    {
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::PERFORM_GRASP)
                {
                    std::vector<movable_body_plant_t* > objects;
                    manipulation_model->get_objects(objects);

                    movable_body_plant_t* chosen_object = NULL;
                    int count=0;
                    std::vector<double> dummy_state;
                    dummy_state.push_back(3);
                    dummy_state.push_back(3);
                    dummy_state.push_back(3);
                    dummy_state.push_back(0);
                    dummy_state.push_back(0);
                    dummy_state.push_back(0);
                    dummy_state.push_back(1);

                    foreach(movable_body_plant_t* manip_obj, objects)
                    {
                        if(manip_obj->get_object_type()==req->object)
                        {
                            chosen_object = manip_obj;
                            chosen_object->get_state_space()->set_from_vector(req->object_state);
                            prx_simulation::object_msg object_placement;
                            object_placement.name = chosen_object->get_object_type();
                            for(unsigned i=0;i<7;i++)
                            {
                                object_placement.elements.push_back(req->object_state[i]);
                            }
                            object_publisher.publish(object_placement);
                        }
                        else
                        {
                            manip_obj->get_state_space()->set_from_vector(dummy_state);
                            prx_simulation::object_msg object_placement;
                            object_placement.name = manip_obj->get_object_type();
                            for(unsigned i=0;i<7;i++)
                            {
                                object_placement.elements.push_back(dummy_state[i]);
                            }
                            object_publisher.publish(object_placement);    
                            dummy_state[0]+=2;
                            dummy_state[1]+=2;
                            dummy_state[2]+=2;                        
                        }

                    }

                    //perform grasp
                    output_query->setup(req->hand,chosen_object,apc_task_query_t::PERFORM_GRASP,req->bin[0],NULL);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        convert_to_plan_msg(output_query->approach_object,command);
                        send_command(command,output_query->approach_object.length());

                        
                        convert_to_plan_msg(output_query->retrieve_object,command);
                        send_command(command,output_query->retrieve_object.length());

                        q_server->setSucceeded();
                    }
                    else
                    {
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::MOVE_TO_ORDER_BIN)
                {
                    //move to order bin
                    output_query->setup(req->hand,NULL,apc_task_query_t::MOVE_TO_ORDER_BIN,'A',NULL);
                    this->root_task->resolve_query();

                    if(output_query->found_solution)
                    {
                        convert_to_plan_msg(output_query->move_to_order_bin,command);
                        send_command(command,output_query->move_to_order_bin.length());
                        q_server->setSucceeded();
                    }
                    else
                    {
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::TEST_BIN_ROADMAP)
                {
                    state_t* goal_state = manipulation_model->get_state_space()->alloc_point();
                    manipulation_model->get_state_space()->set_from_vector(req->goal_state,goal_state);
                    //move only, bin, hand, and object don't matter
                    output_query->setup(req->hand,NULL,apc_task_query_t::TEST_BIN_ROADMAP,'A',goal_state);
                    this->root_task->link_query(output_query);
                    this->root_task->resolve_query();
                    // PRX_PRINT("\nStar2: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);

                    if(output_query->found_solution)
                    {
                        convert_to_plan_msg(output_query->move_plan,command);
                        send_command(command,output_query->move_plan.length());
                        manipulation_model->use_context(full_manipulator_context_name);
                        manipulation_model->get_state_space()->free_point(goal_state);
                        q_server->setSucceeded();
                    }
                    else
                    {
                        q_server->setAborted();
                    }
                }
                else if(req->stage == apc_task_query_t::EXECUTE_SAVED_TRAJ)
                {
                    //UPDATE SCENE
                    std::vector<movable_body_plant_t* > objects;
                    manipulation_model->get_objects(objects);

                    movable_body_plant_t* chosen_object = NULL;
                    int count=0;
                    std::vector<double> dummy_state;
                    dummy_state.push_back(3);
                    dummy_state.push_back(3);
                    dummy_state.push_back(3);
                    dummy_state.push_back(0);
                    dummy_state.push_back(0);
                    dummy_state.push_back(0);
                    dummy_state.push_back(1);

                    foreach(movable_body_plant_t* manip_obj, objects)
                    {
                        if(manip_obj->get_object_type()==req->object)
                        {
                            chosen_object = manip_obj;
                            chosen_object->get_state_space()->set_from_vector(req->object_state);
                            prx_simulation::object_msg object_placement;
                            object_placement.name = chosen_object->get_object_type();
                            for(unsigned i=0;i<7;i++)
                            {
                                object_placement.elements.push_back(req->object_state[i]);
                            }
                            object_publisher.publish(object_placement);
                        }
                        else
                        {
                            manip_obj->get_state_space()->set_from_vector(dummy_state);
                            prx_simulation::object_msg object_placement;
                            object_placement.name = manip_obj->get_object_type();
                            for(unsigned i=0;i<7;i++)
                            {
                                object_placement.elements.push_back(dummy_state[i]);
                            }
                            object_publisher.publish(object_placement);    
                            dummy_state[0]+=2;
                            dummy_state[1]+=2;
                            dummy_state[2]+=2;                        
                        }
                    }
                    // std::string object_name = req->object;
                    // std::string hand = req->hand;
                    // std::string pose = req->pose;
                    // std::string grasp_id = req->grasp_id;
                    /////////////EXECUTE SAVED PLANS ONLY/////////////////////////////////
                    std::ostringstream file_name;
                    file_name << "/saved_plans/"<<req->object<<"_"<<req->hand<<"_"<<req->pose<<"_"<<req->grasp_id<<".plan";
                    execute_plan_from_file(file_name.str().c_str());
                    sleep(2);
                    q_server->setSucceeded();
                    // PRX_FATAL_S("Executed all saved plans! Exiting now...");
                    /////////////////////////////////////////////////////////////////////
                }
                output_query->clear();
                // PRX_PRINT("\n  End: "<<manipulation_model->get_full_state_space()->print_memory(3),PRX_TEXT_RED);

            }

            void apc_planning_application_t::send_command(prx_simulation::send_trajectoryGoal& goal,double timeout)
            {
                execute_client->sendGoal(goal);
                bool finished = false;
                finished = execute_client->waitForResult(ros::Duration(0));
                // sleep(2);

            }

            void apc_planning_application_t::convert_to_plan_msg(sim::plan_t& in_plan, prx_simulation::send_trajectoryGoal& goal)
            {
                //////////////////SAVE PLANS TO FILE////////////////////////
                // std::ostringstream file_name;
                // file_name << "/home/pracsys/repos/pracsys_ws/src/pracsys/prx_input/saved_plans/sp" << plan_counter;
                // plan_counter++;
                // in_plan.save_to_file(file_name.str().c_str());
                ////////////////////////////////////////////////////////////

                manipulation_model->use_context(full_manipulator_context_name);
                space_t* control_space = manipulation_model->get_control_space();
                space_t* state_space = manipulation_model->get_state_space();
                sim::plan_t gripper_on_plan;
                gripper_on_plan.link_control_space(control_space);
                gripper_on_plan.clear();

                goal.plan.clear();
                goal.traj.clear();
                int counter = 0;
                int position = 0;
                PRX_PRINT("Plan Before: "<<in_plan.print(2), PRX_TEXT_RED);
                foreach(plan_step_t step, in_plan)
                {
                    counter ++;
                    if(counter>5 && step.control->at(16)==2){
                        position = counter-50;
                        break;
                        // PRX_PRINT("step.control->at(16) = "<<step.control->at(16)<<"  duration = "<<step.duration,PRX_TEXT_CYAN);
                    }
                }
                counter = 0;
                foreach(plan_step_t step, in_plan)
                {
                    counter++;
                    if(counter == position){
                        control_t* zero_control = control_space->alloc_point();
                        control_space->zero(zero_control);
                        zero_control->at(16)=2;
                        gripper_on_plan.copy_onto_back(zero_control, 0.02);
                    }
                    gripper_on_plan.copy_onto_back(step.control,step.duration);
                }
                PRX_PRINT("Plan After: "<<gripper_on_plan.print(2), PRX_TEXT_RED);
                foreach(plan_step_t step, gripper_on_plan)
                {
                    goal.plan.push_back(prx_simulation::control_msg());
                    goal.plan.back().duration = step.duration;
                    int i=0;
                    for(i=0;i<control_space->get_dimension();++i)
                    {
                        goal.plan.back().control.push_back(step.control->at(i));
                    }
                }
            }

            void apc_planning_application_t::get_state_callback(const prx_simulation::state_msg& stateMsg)
            {
                for(unsigned i=0;i<stateMsg.elements.size();i++)
                {
                    current_manipulator_state->at(i) = stateMsg.elements[i];
                }
            }

            void apc_planning_application_t::execute_plan_from_file(std::string file_name)
            {
                prx_simulation::send_trajectoryGoal goal;
                PRX_PRINT("Reading plan from file: "<<file_name,PRX_TEXT_CYAN);
                manipulation_model->use_context(full_manipulator_context_name);
                space_t* control_space = manipulation_model->get_control_space();
                saved_plan.link_control_space(control_space);
                saved_plan.clear();
                saved_plan.read_from_file(file_name.c_str());
                PRX_PRINT("Executing plan: "<<saved_plan.print(2), PRX_TEXT_MAGENTA);
                convert_to_plan_msg(saved_plan, goal);
                send_command(goal,saved_plan.length());
            }
        }
    }
}
