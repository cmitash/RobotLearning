/**
 * @file 2link_acrobot.hpp 
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
#pragma once

#ifndef  PRX_TWO_LINK_ACROBOT_HPP
#define  PRX_TWO_LINK_ACROBOT_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/systems/plants/integration_plant.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * Represents a two link_acrobot system. Implementation of the integration functions 
         * for simulating a two link_acrobot.\n
         * State: [theta1, theta2, velocity_theta1, velocity_theta2] \n
         * Control: [torque on second joint].
         * 
         * @brief <b> Represents a two link_acrobot system (passive,active joints). </b>
         * 
         * @author Zakary Littlefield
         * 
         */
        class two_link_acrobot_t : public integration_plant_t
        {

          public:

            two_link_acrobot_t();

            virtual ~two_link_acrobot_t();

            /** @copydoc integration_plant_t::init(const util::parameter_reader_t *, const util::parameter_reader_t*) */
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc integration_plant_t::propagate(const double) */
            void propagate(const double simulation_step);

            /** @copoydoc integration_plant_t::update_phys_configs(util::config_list_t&, unsigned&) const */
            virtual void update_phys_configs(util::config_list_t& configs, unsigned& index) const;


          protected:

            /** @copoydoc plant_t::update_derivative (state_t* const) */
            virtual void update_derivative(state_t * const result);

            /**
             * Indexer for state variable : THETA1.
             * @brief Indexer for state variable : THETA1.
             */
            const static unsigned STATE_THETA_1;

            /**
             * Indexer for state variable : THETA2.
             * @brief Indexer for state variable : THETA2.
             */
            const static unsigned STATE_THETA_2;

            /**
             * Indexer for state variable : V1.
             * @brief Indexer for state variable : V1.
             */
            const static unsigned STATE_V_1;

            /**
             * Indexer for state variable : V2.
             * @brief Indexer for state variable : V2.
             */
            const static unsigned STATE_V_2;

            /**
             * Indexer for control variable T.
             * @brief Indexer for control variable T.
             */
            const static unsigned CONTROL_T;

            /**
             * Internal state for the variable \c theta1.
             * @brief Internal state for the variable \c theta1.
             */
            double _theta1;

            /**
             * Internal state for the variable \c theta2.
             * @brief Internal state for the variable \c theta2.
             */
            double _theta2;

            /**
             * Internal state for the variable \c theta1dot.
             * @brief Internal control for the variable \c theta1dot.
             */
            double _theta1dot;

            /**
             * Internal state for the variable \c theta2dot.
             * @brief Internal control for the variable \c theta2dot.
             */
            double _theta2dot;

            /**
             * Internal control for torque.
             * @brief Internal control for torque.
             */
            double _tau;

            /**
             * Length of the links.
             * @brief Length of the links.
             */
            double length;

            /**
             * Mass of the links.
             * @brief Mass of the links.
             */
            double m;

            /**
             * Damping factor.
             * @brief Damping factor.
             */
            double damp;

        };

    }
}

#endif


