/**
 * @file unigripper.hpp 
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
#pragma once

#ifndef PRX_UNIGRIPPER_HPP
#define	PRX_UNIGRIPPER_HPP

#include "prx/utilities/definitions/defs.hpp"

#include "simulation/systems/plants/manipulator.hpp"

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {
            /**
             * 
             */
            class unigripper_t : public manipulator_t
            {

              public:
                unigripper_t();

                virtual ~unigripper_t(){ }

                /** @copydoc plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
                virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);
                virtual void propagate(const double simulation_step = 0);
                

                /** @copoydoc plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
                virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;

                virtual void update_collision_info();

                virtual bool IK_solver( util::space_point_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, std::string start_link, std::string end_link);

                virtual bool IK_steering( sim::plan_t& result_plan,  util::space_point_t* result_state, const util::space_point_t* start_state, const util::config_t& goal_config, std::string start_link, std::string end_link);

                virtual void steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan);
                
                virtual void update_derivative(sim::state_t * const result);

              protected:

                virtual void create_spaces();

                KDL::Frame static_transform;

                /**
                 * Internal state memory for the \c x position coordinate.
                 * @brief Internal state memory for the \c x position coordinate.
                 */
                double _x;

                /**
                 * Internal state memory for the \c y position coordinate.
                 * @brief Internal state memory for the \c y position coordinate.
                 */
                double _y;

                /**
                 * Internal state memory for the \c z position coordinate.
                 * @brief Internal state memory for the \c z position coordinate.
                 */
                double _z;

                /**
                 * Internal state memory for the \c qx coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qx coordinate of the orientation quaternion.
                 */
                double _qx;

                /**
                 * Internal state memory for the \c qy coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qy coordinate of the orientation quaternion.
                 */
                double _qy;

                /**
                 * Internal state memory for the \c qz coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qz  coordinate of the orientation quaternion.
                 */
                double _qz;

                /**
                 * Internal state memory for the \c qw coordinate of the orientation quaternion.
                 * @brief Internal state memory for the \c qw coordinate of the orientation quaternion.
                 */
                double _qw;

            };
        }

    }
}

#endif
