/**
 * @file apc_planning_application.hpp 
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

#include "prx/planning/applications/planning_application.hpp"
#include "prx/planning/queries/query.hpp"
#include "planning/apc_task_query.hpp"
#include "planning/manipulation_world_model.hpp"

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <prx_simulation/send_trajectoryAction.h>
#include <prx_planning/apc_queryAction.h>
#include "prx_simulation/state_msg.h"

#ifndef PRX_APC_PLANNING_APPLICATION_HPP
#define	PRX_APC_PLANNING_APPLICATION_HPP

namespace prx
{
    namespace packages
    {
        namespace apc
        {

            typedef actionlib::SimpleActionClient<prx_simulation::send_trajectoryAction> send_traj_client;
            typedef actionlib::SimpleActionServer<prx_planning::apc_queryAction> query_server;

            class apc_planning_application_t : public plan::planning_application_t
            {

              public:
                apc_planning_application_t();
                virtual ~apc_planning_application_t();

                virtual void init(const util::parameter_reader_t* reader);

                virtual void execute();

              protected:

                void resolve_query(const prx_planning::apc_queryGoalConstPtr& req);

                void convert_to_plan_msg(sim::plan_t& in_plan, prx_simulation::send_trajectoryGoal& goal);
                void send_command(prx_simulation::send_trajectoryGoal& goal,double timeout);
                void get_state_callback(const prx_simulation::state_msg& stateMsg);

                void execute_plan_from_file(std::string file_name);
                std::ifstream saved_plan_file;
                sim::plan_t saved_plan;
                int plan_counter;

                ros::NodeHandle n;
                std::string full_manipulator_context_name;
                apc_task_query_t* output_query;
                manipulation::manipulation_world_model_t* manipulation_model;
                send_traj_client* execute_client;
                query_server* q_server;
                ros::Subscriber state_subscriber;
                sim::state_t* current_manipulator_state;
                sim::state_t* propagated_manipulator_state;
                ros::Publisher object_publisher;
                int total_resolves;

            };
        }

    }
}

#endif

