
/**
 * @file floaint_hand_mappings.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#pragma once

#ifndef PRX_FLOATING_HAND_MAPPINGS_HPP
#define PRX_FLOATING_HAND_MAPPINGS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/spaces/mapping_functions/subset_mapping.hpp"

#include <boost/assign/list_of.hpp>

namespace prx
{
    namespace packages
    {
        namespace manipulation
        {

        	class floating_hand_full_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            floating_hand_full_state_mapping_t() 
		            {
		                domain = 8;
		                range = 8;
		                mapped_indices = boost::assign::list_of(0)(1)(2)(3)(4)(5)(6)(7);
		                output_space_name = "SE3|D";
		                mapping_name = "floating_hand_full_state_mapping";
		            }
		            virtual ~floating_hand_full_state_mapping_t() {}
        	};

        	//hand 
        	class floating_hand_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            floating_hand_state_mapping_t() 
		            {
		                domain = 8;
		                range = 7;
		                mapped_indices = boost::assign::list_of(0)(1)(2)(3)(4)(5)(6);
		                output_space_name = "SE3";
		                mapping_name = "floating_hand_state_mapping";
		            }
		            virtual ~floating_hand_state_mapping_t() {}
        	};

        	//gripper
        	class floating_hand_gripper_state_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            floating_hand_gripper_state_mapping_t() 
		            {
		                domain = 8;
		                range = 1;
		                mapped_indices = boost::assign::list_of(7);
		                output_space_name = "D";
		                mapping_name = "floating_hand_gripper_state_mapping";
		            }
		            virtual ~floating_hand_gripper_state_mapping_t() {}
        	};


        	///////////////////////////////////////////
        	// Same mappings for the control subsets //
        	///////////////////////////////////////////


        	class floating_hand_full_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            floating_hand_full_control_mapping_t() 
		            {
		                domain = 8;
		                range = 8;
		                mapped_indices = boost::assign::list_of(0)(1)(2)(3)(4)(5)(6)(7);
		                output_space_name = "SE3|D";
		                mapping_name = "floating_hand_full_control_mapping";
		            }
		            virtual ~floating_hand_full_control_mapping_t() {}
        	};

        	class floating_hand_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            floating_hand_control_mapping_t() 
		            {
		                domain = 8;
		                range = 7;
		                mapped_indices = boost::assign::list_of(0)(1)(2)(3)(4)(5)(6);
		                output_space_name = "SE3";
		                mapping_name = "floating_hand_control_mapping";
		            }
		            virtual ~floating_hand_control_mapping_t() {}
        	};

        	class floating_hand_gripper_control_mapping_t : public util::subset_mapping_t
        	{
		        public:
		            floating_hand_gripper_control_mapping_t() 
		            {
		                domain = 8;
		                range = 1;
		                mapped_indices = boost::assign::list_of(7);
		                output_space_name = "D";
		                mapping_name = "floating_hand_gripper_control_mapping";
		            }
		            virtual ~floating_hand_gripper_control_mapping_t() {}
        	};
        }
    }
}

#endif