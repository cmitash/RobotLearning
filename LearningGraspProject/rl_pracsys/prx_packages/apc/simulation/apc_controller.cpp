/**
 * @file apc_controller.cpp 
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

#include "simulation/apc_controller.hpp"
#include "prx_simulation/state_msg.h"
#include "prx_simulation/gripper_change_srv.h"
#include "prx_simulation/UnigripperVacuumOn.h"

#ifdef PRX_SENSING_FOUND
#include "prx_sensing/UpdateObjectList.h"
#include "prx_sensing/PublishObjectList.h"
#include "prx_sensing/UpdateShelfPosition.h"
#endif

#include <boost/range/adaptor/map.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::apc::apc_controller_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    namespace packages
    {
        using namespace manipulation;
        namespace apc
        {
            const std::string dof_names[16] = {"torso_joint_b1", "arm_left_joint_1_s", "arm_left_joint_2_l", "arm_left_joint_3_e", "arm_left_joint_4_u", "arm_left_joint_5_r", "arm_left_joint_6_b", "arm_left_joint_7_t", "arm_right_joint_1_s", "arm_right_joint_2_l", "arm_right_joint_3_e", "arm_right_joint_4_u", "arm_right_joint_5_r", "arm_right_joint_6_b", "arm_right_joint_7_t", "torso_joint_b2"};
            apc_controller_t::apc_controller_t()
            {
                has_goal = false;
                send_to_robot = false;
            }

            apc_controller_t::~apc_controller_t()
            {

            }

            void apc_controller_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                controller_t::init(reader, template_reader);

                // Set up communication with paired planning node
                execute_plan_server = new TrajectoryServer(n,"prx/execute_plan_action",true);
                execute_plan_server->registerGoalCallback( boost::bind(&apc_controller_t::receive_goal, this, _1) );
                execute_plan_server->registerCancelCallback( boost::bind(&apc_controller_t::cancel_goal, this, _1) );
#ifdef PRX_SENSING_FOUND
                // Testing calling services advertised by prx_sensing
                ros::ServiceClient prx_sensing_update_cl = n.serviceClient<prx_sensing::UpdateObjectList>("prx/sensing/update_object_list");

                prx_sensing::UpdateObjectList sensing_update_srv;
                sensing_update_srv.request.object_list.push_back("crayola_64_ct");
                sensing_update_srv.request.object_list.push_back("expo_dry_erase_board_eraser");

                prx_sensing_update_cl.call(sensing_update_srv);
                PRX_INFO_S(sensing_update_srv.response);
#endif                
                foreach(system_ptr_t plant, subsystems | boost::adaptors::map_values)
                {
                    //Create and initialize information for all the end effectors. 
                    if(dynamic_cast<manipulator_t*>(plant.get()) != NULL)
                    {
                        //Create end effector infos for each end effector.
                        manipulator = dynamic_cast<manipulator_t*>(plant.get());
                    }

                    //Create a list of all the movable bodies.
                    if(dynamic_cast<movable_body_plant_t*>(plant.get()) != NULL)
                    {
                        movable_bodies.push_back(dynamic_cast<movable_body_plant_t*>(plant.get()));
                    }
                }

                child_state_space = manipulator->get_state_space();

                // create contingency plan
                contingency_plan.link_control_space(output_control_space);
                plan.link_control_space(output_control_space);
                robot_plan.link_control_space(output_control_space);
                manipulator->append_contingency(contingency_plan, 0.0);

                state_publisher = n.advertise<prx_simulation::state_msg>("prx/manipulator_state", 1);
                execute_plan_server->start();

                object_subscriber = n.subscribe("prx/object_state", 100, &apc_controller_t::place_object_callback,this);
                moved = NULL;


                real_robot = parameters::get_attribute_as<bool>("real_robot",reader,template_reader,false);
                if(real_robot)
                {
                    real_robot_state = n.subscribe("/joint_states",4,&apc_controller_t::real_robot_state_callback,this);
                    robot_state = child_state_space->alloc_point();
                    name_index_map["torso_joint_b1"] = 0;
                    name_index_map["torso_joint_b2"] = 0;

                    name_index_map["arm_left_joint_1_s"] = 1;
                    name_index_map["arm_left_joint_2_l"] = 2;
                    name_index_map["arm_left_joint_3_e"] = 3;
                    name_index_map["arm_left_joint_4_u"] = 4;
                    name_index_map["arm_left_joint_5_r"] = 5;
                    name_index_map["arm_left_joint_6_b"] = 6;
                    name_index_map["arm_left_joint_7_t"] = 7;
                    name_index_map["head_hinge"] = 8;

                    name_index_map["arm_right_joint_1_s"] = 9;
                    name_index_map["arm_right_joint_2_l"] = 10;
                    name_index_map["arm_right_joint_3_e"] = 11;
                    name_index_map["arm_right_joint_4_u"] = 12;
                    name_index_map["arm_right_joint_5_r"] = 13;
                    name_index_map["arm_right_joint_6_b"] = 14;
                    name_index_map["arm_right_joint_7_t"] = 15;
                    robot_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("joint_trajectory_action", true);
                
                    unigripper_ac = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("unigripper/unigripper_joint_trajectory_action", true);

                    unigripper_command.trajectory.joint_names.push_back("head_hinge");
                    for( int i = 0; i < 16; i++ )
                        robot_command.trajectory.joint_names.push_back(dof_names[i]);
                }
            }

            void apc_controller_t::receive_goal(TrajectoryServer::GoalHandle& gh)
            {
                if(has_goal)
                {
                    PRX_INFO_S("Rejecting goal");
                    gh.setRejected();
                }
                else
                {
                    gh.setAccepted();
                    active_goal = gh;
                    control_t* control = output_control_space->alloc_point();
                    robot_plan.clear();
                    foreach(const control_msg& control_msg, active_goal.getGoal()->plan)
                    {
                        for( unsigned int i = 0; i < control_msg.control.size(); ++i )
                            control->at(i) = control_msg.control[i];
                        robot_plan.copy_onto_back(control, control_msg.duration);
                    }
                    output_control_space->free_point(control);
                    plan = robot_plan;
                    has_goal = true;
                }
            }

            void apc_controller_t::place_object_callback(const prx_simulation::object_msg& objectMsg)
            {
                config_t base_config;
                foreach(movable_body_plant_t* plant, movable_bodies)
                {
                    // PRX_INFO_S("Try Setting "<<objectMsg.name);
                    if(plant->get_object_type() == objectMsg.name)
                    {
                        // PRX_INFO_S("Setting "<<plant->get_object_type());
                        moved = plant;
                        new_config.set_position(objectMsg.elements[0],objectMsg.elements[1],objectMsg.elements[2]);
                        new_config.set_orientation(objectMsg.elements[3],objectMsg.elements[4],objectMsg.elements[5],objectMsg.elements[6]);
                        base_config.zero();
                        moved->move_object(base_config,new_config);
                        moved = NULL;
                    }
                }
            }

            void apc_controller_t::real_robot_state_callback(const sensor_msgs::JointState& stateMsg)
            {
                if(real_robot && has_goal)
                    return;
                child_state_space->copy_to_point(robot_state);
                sensor_msgs::JointState start_state = stateMsg;
                for( unsigned i = 0; i < start_state.name.size(); i++ )
                {
                    robot_state->at(name_index_map[start_state.name[i]]) = start_state.position[i];
                }
                child_state_space->copy_from_point(robot_state);
            }

            void apc_controller_t::cancel_goal(TrajectoryServer::GoalHandle& gh)
            {
                //TODO
            }
            void apc_controller_t::propagate(const double simulation_step)
            {
#ifdef PRX_SENSING_FOUND

                ros::ServiceClient prx_sensing_find_shelf_cl = n.serviceClient<prx_sensing::UpdateShelfPosition>("prx/sensing/update_shelf_pose");

                prx_sensing::UpdateShelfPosition srv_shelf;
                srv_shelf.request.update_shelf = true;
                prx_sensing_find_shelf_cl.call(srv_shelf);

                config_t new_config;
                config_t base_config;
                
                // TODO: find shelf, place shelf
                foreach(movable_body_plant_t* plant, movable_bodies) {
                    if(plant->get_object_type() == "shelf") {
                        
                        moved = plant;
                        double x,y,z,qx,qy,qz,qw;
                        x = srv_shelf.response.shelf_pose.pose.position.x;
                        y = srv_shelf.response.shelf_pose.pose.position.y;
                        z = srv_shelf.response.shelf_pose.pose.position.z;
                        qx = srv_shelf.response.shelf_pose.pose.orientation.x;
                        qy = srv_shelf.response.shelf_pose.pose.orientation.y;
                        qz = srv_shelf.response.shelf_pose.pose.orientation.z;
                        qw = srv_shelf.response.shelf_pose.pose.orientation.w;
                        new_config.set_position(x,y,z);
                        new_config.set_orientation(qx,qy,qz,qw);
                        base_config.zero();
                        moved->move_object(base_config,new_config);
                        moved = NULL;
                    }                    
                }


                ros::ServiceClient prx_sensing_publish_cl = n.serviceClient<prx_sensing::PublishObjectList>("prx/sensing/publish_object_list");

                prx_sensing::PublishObjectList srv;
                srv.request.publish_objects = true;
                prx_sensing_publish_cl.call(srv);

                foreach(prx_sensing::ObjectPose& obj, srv.response.object_list)
                {

                    std::string object_name = obj.object_name;

                    config_t base_config;
                    config_t new_config;
                    foreach(movable_body_plant_t* plant, movable_bodies)
                    {

                        if(plant->get_object_type() == object_name)
                        {
                           
                            moved = plant;
                            double x,y,z,qx,qy,qz,qw;
                            x = obj.pose.pose.position.x;
                            y = obj.pose.pose.position.y;
                            z = obj.pose.pose.position.z;
                            qx = obj.pose.pose.orientation.x;
                            qy = obj.pose.pose.orientation.y;
                            qz = obj.pose.pose.orientation.z;
                            qw = obj.pose.pose.orientation.w;
                            new_config.set_position(x,y,z);
                            new_config.set_orientation(qx,qy,qz,qw);
                            base_config.zero();
                            moved->move_object(base_config,new_config);
                            moved = NULL;
                        }
                    }
                }


#endif


                controller_t::propagate(simulation_step);
                // if(moved!=NULL)
                // {
                //     config_t base_config;
                //     base_config.zero();
                //     moved->move_object(base_config,new_config);
                //     moved = NULL;
                // }
                prx_simulation::state_msg msg;
                for(unsigned i=0;i<child_state_space->get_dimension();++i)
                {
                    msg.elements.push_back(child_state_space->at(i));
                }
                state_publisher.publish(msg);
                // PRX_STATUS(child_state_space->print_memory(6),PRX_TEXT_RED);
            }

            void apc_controller_t::compute_control()
            {
                if(!real_robot)
                {
                    control_t* new_control = plan.get_next_control(simulation::simulation_step);

                    // PRX_STATUS("Plan: "<<plan.length(),PRX_TEXT_GREEN);

                    if( new_control == NULL )
                    {
                        new_control = contingency_plan.get_control_at(0);
                        if(has_goal)
                        {
                            active_goal.setSucceeded();
                            has_goal = false;
                        }
                    }

                    output_control_space->copy_from_point(new_control);
                    manipulator->compute_control();

                }
                else
                {
                    if(has_goal)
                    {
                        std::vector<trajectory_t*> robot_trajs;
                        std::vector<plan_t*> robot_plans;
                        std::vector<bool> grasp_plan;


                        state_t* local_state = child_state_space->alloc_point();
                        robot_trajs.push_back(new trajectory_t(child_state_space));
                        robot_plans.push_back(new plan_t(output_control_space));
                        grasp_plan.push_back(false);

                        // robot_trajs.back()->copy_onto_back(local_state);
                        bool last_was_grasp = false;
                        foreach(plan_step_t step, robot_plan)
                        {
                            int steps = (int)((step.duration / simulation::simulation_step) + .1);
                            output_control_space->copy_from_point(step.control);
                            bool new_grasp = (output_control_space->at(16)>=1 || output_control_space->at(17)>=1);
                            if(last_was_grasp || new_grasp)
                            {
                                //end old traj, start new one
                                robot_trajs.push_back(new trajectory_t(child_state_space));
                                robot_plans.push_back(new plan_t(output_control_space));
                                grasp_plan.push_back(new_grasp);
                                // robot_trajs.back()->copy_onto_back(local_state);
                                if(last_was_grasp)
                                    last_was_grasp = false;
                                last_was_grasp = new_grasp;

                            }
                            robot_plans.back()->copy_onto_back(step.control,step.duration);
                        }

                        for(unsigned index = 0;index<robot_trajs.size();index++)
                        {
                            if(!grasp_plan[index] && robot_plans[index]->length()>0)
                            {

                                child_state_space->copy_to_point(local_state);
                                robot_trajs[index]->copy_onto_back(local_state);

                                foreach(plan_step_t step, *robot_plans[index])
                                {
                                    int steps = (int)((step.duration / simulation::simulation_step) + .1);
                                    output_control_space->copy_from_point(step.control);
                                    for(int i=0;i<steps;i++)
                                    {
                                        controller_t::propagate(simulation::simulation_step);
                                        child_state_space->copy_to_point(local_state);
                                        robot_trajs[index]->copy_onto_back(local_state);
                                    }
                                }

                                robot_command.trajectory.points.clear();
                                unigripper_command.trajectory.points.clear();
                                //create message and send to robot
                                double duration=0;
                                for(unsigned i=0;i<robot_trajs[index]->size()-1;i++)
                                {
                                    trajectory_msgs::JointTrajectoryPoint point;
                                    trajectory_msgs::JointTrajectoryPoint point_uni;
                                    point.time_from_start = ros::Duration(duration);
                                    point_uni.time_from_start = ros::Duration(duration);
                                    control_t* control = robot_plans[index]->get_control_at(duration);
                                    //add the state to the trajectory point
                                    for( unsigned j = 0; j < 16; j++ )
                                    {
                                        point.positions.push_back((*robot_trajs[index])[i]->at(name_index_map[dof_names[j]]));
                                        point.velocities.push_back(control->at(name_index_map[dof_names[j]]));
                                        point.accelerations.push_back(0);
                                    }
                                    duration+=simulation::simulation_step;
                                    robot_command.trajectory.points.push_back(point);

                                    point_uni.positions.push_back((*robot_trajs[index])[i]->at(8));
                                    point_uni.positions.push_back(control->at(8));
                                    point_uni.accelerations.push_back(0);

                                    unigripper_command.trajectory.points.push_back(point_uni);

                                }
                                PRX_INFO_S("Waiting");
                                bool found_ac_server = false;
                                while( !found_ac_server )
                                    found_ac_server = robot_ac->waitForServer(ros::Duration(0)) && unigripper_ac->waitForServer(ros::Duration(0));
                                PRX_INFO_S("Waiting part 2");

                                PRX_INFO_S("Plan: \n"<<robot_plans[index]->print(5));
                                //PRX_INFO_S("Traj: \n"<<robot_trajs[index]->print(2));
                                if(duration>0)
                                {
                                    robot_command.trajectory.header.stamp = ros::Time::now();
                                    unigripper_command.trajectory.header.stamp = ros::Time::now();
                                    robot_ac->sendGoal(robot_command);
                                    unigripper_ac->sendGoal(unigripper_command);
                                }
                                bool finished_before_timeout;
                                // PRX_WARN_S(ros::Duration(robot_plans[index]->length()+1.0));
                                finished_before_timeout= robot_ac->waitForResult(ros::Duration(robot_plans[index]->length()+5.0));
                                if(!finished_before_timeout)
                                {
                                    PRX_INFO_S("Didn't finish before timeout: "<<robot_plans[index]->length()+5.0);
                                    robot_ac->cancelGoal();
                                    unigripper_ac->cancelGoal();
                                }
                                else
                                {
                                    actionlib::SimpleClientGoalState state = robot_ac->getState();
                                    ROS_INFO("Action finished: %s",state.toString().c_str());
                                }
                                ros::Rate rate(10);
                                int counter =0;
                                has_goal = false;
                                while(ros::ok() && counter++<11)
                                {
                                    rate.sleep();
                                }
                                has_goal = true;
                            }
                            else if(grasp_plan[index])
                            {
                                PRX_INFO_S("Plan: \n"<<robot_plans[index]->print(5));
                                //trigger a grasp
                                control_t* control = robot_plans[index]->get_control_at(0);
                                if(control->at(16)>=1)
                                {
                                    PRX_INFO_S("Sending UniGripper command");
                                    ros::ServiceClient uni_client = n.serviceClient<prx_simulation::UnigripperVacuumOn>("unigripper_vacuum");
                                    prx_simulation::UnigripperVacuumOn uni_srv;
                                    if(control->at(16) == 1)//Vacuum OFF
                                    {
                                        uni_srv.request.TurnVacuumOn = false;
                                    }
                                    else if(control->at(16) == 2)//Vacuum ON
                                    {
                                        uni_srv.request.TurnVacuumOn = true;
                                    }
                                    else
                                    {
                                        PRX_ERROR_S("Wrong UniGripper Control!");
                                    }

                                    uni_client.waitForExistence();
                                    uni_client.call(uni_srv);
                                    sleep(2);

                                }
                                if(control->at(17)>=1)
                                {
                                    PRX_INFO_S("Sending gripper command");
                                    ros::ServiceClient client = n.serviceClient<prx_simulation::gripper_change_srv>("/prx/reflex_grasp");
                                    prx_simulation::gripper_change_srv srv;
                                    srv.request.gripper_state = control->at(17);

                                    client.waitForExistence();
                                    client.call(srv);
                                }

                            }
                            delete robot_trajs[index];
                            delete robot_plans[index];
                        }
                        child_state_space->free_point(local_state);

                        PRX_INFO_S("Done");
                        has_goal = false;
                        send_to_robot = false;
                        robot_plan.clear();
                        active_goal.setSucceeded();
                    }
                    control_t* new_control = contingency_plan.get_control_at(0);
                    output_control_space->copy_from_point(new_control);
                    manipulator->compute_control();
                }
                // else
                // {
                //     //still look at plan as if we are simulating it
                //     control_t* new_control = plan.get_next_control(simulation::simulation_step);
                //     if( new_control == NULL )
                //     {
                //         //we should be close to done on the real robot as well
                //         if(has_goal)
                //         {
                //             robot_ac->waitForResult();
                //             actionlib::SimpleClientGoalState state = robot_ac->getState();
                //             //TODO: This is a basic check. Hopefully, if the driver is good, this is all we need. Otherwise, we will need to check for a timeout.
                //             if(state.toString() == "SUCCEEDED")
                //             {
                //                 active_goal.setSucceeded();
                //                 has_goal = false;
                //                 plan.clear();
                //             }
                //         }
                //     }                    
                // }
            }
        }
    }
}
