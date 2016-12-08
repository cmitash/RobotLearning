/**
 * @file unigripper.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2016, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "simulation/systems/plants/motoman/unigripper.hpp"

#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/param.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::unigripper_t, prx::sim::system_t)




namespace prx
{
    using namespace sim;
    using namespace util;
    namespace packages
    {
        namespace manipulation
        {
            unigripper_t::unigripper_t()
            {
                state_memory = boost::assign::list_of(&_x)(&_y)(&_z)(&_qx)(&_qy)(&_qz)(&_qw);

                input_path = pracsys_path + "/prx_packages/manipulation/input/motoman/";
            }

            void unigripper_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                manipulator_t::init(reader, template_reader);   
                KDL::Chain chain1;
                kdl_tree->getChain(kdl_tree->getRootSegment()->first, reverse_split_path(effector_names[0]).second, chain1);          
                KDL::JntArray q(chain1.getNrOfJoints());  

                KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                fk_solver.JntToCart(q,static_transform); 
            }


            void unigripper_t::propagate(const double simulation_step)
            {
                // integrator->integrate(derivative_function, simulation_step);
                std::vector<double> temp;
                input_control_space->copy_to_vector(temp);
                state_space->set_from_vector(temp);
            }

            void unigripper_t::create_spaces()
            {
                state_space = new space_t("SE3|D", state_memory);
                input_control_space = new space_t("SE3|D", control_memory);
            }

            void unigripper_t::update_collision_info()
            {
                root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                manipulator_t::update_collision_info();     
            }

            void unigripper_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                root_frame = KDL::Frame(KDL::Rotation::Quaternion(_qx,_qy,_qz,_qw),KDL::Vector(_x, _y, _z));
                manipulator_t::update_phys_configs(configs,index);  
            }
            bool unigripper_t::IK_solver( space_point_t* result_state, const space_point_t* start_state, const config_t& goal_config, std::string start_link, std::string end_link)
            {      
                //ignoring the names of the start and end link
                KDL::Frame end_frame;
                double qx,qy,qz,qw,x,y,z;
                goal_config.get_position(x,y,z);
                goal_config.get_orientation().get(qx,qy,qz,qw);
                end_frame.p = KDL::Vector(x,y,z);
                end_frame.M = KDL::Rotation::Quaternion(qx,qy,qz,qw); 
                end_frame = end_frame*static_transform.Inverse();
                state_space->copy_point(result_state,start_state);

                result_state->at(0) = end_frame.p.x();
                result_state->at(1) = end_frame.p.y();
                result_state->at(2) = end_frame.p.z();
                convert_to_quaternion(end_frame.M,qx,qy,qz,qw);
                result_state->at(3) = qx;
                result_state->at(4) = qy;
                result_state->at(5) = qz;
                result_state->at(6) = qw;
                return true;
            }

            bool unigripper_t::IK_steering( sim::plan_t& result_plan,  util::space_point_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, std::string start_link, std::string end_link)
            {
                this->IK_solver(result_state,start_state,goal_config,start_link,end_link);
                result_plan.append_onto_back(simulation::simulation_step);
                input_control_space->copy_point(result_plan.back().control,result_state);
                return true;
            }
            void unigripper_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
            {
                result_plan.copy_onto_back(goal, simulation::simulation_step);
            }

            void unigripper_t::update_derivative(state_t * const result)
            {
                //update state 
                for(unsigned index=0;index<state_space->get_dimension();index++)
                {
                    result->memory[index] = (*control_memory[index] - *state_memory[index])*GRASP_STEP;
                }
            }
        }
    }
}
