/**
 * @file grasp.cpp
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

#include "planning/modules/grasp.hpp"

namespace prx
{
    using namespace util;

    namespace packages
    {
        namespace manipulation
        {
            grasp_t::grasp_t()
            {
                release_mode = -1;
                grasping_mode = -1;
            }

            grasp_t::grasp_t(const util::config_t& conf, int open, int grasp)
            {
                relative_config = conf;
                release_mode = open;
                grasping_mode = grasp;
            }

            void grasp_t::clear()
            {
                release_mode = -1;
                grasping_mode = -1;
                relative_config.zero();
            }

            grasp_t& grasp_t::operator=(const grasp_t& g)
            {
                relative_config = g.relative_config;
                release_mode = g.release_mode;
                grasping_mode = g.grasping_mode;
                return *this;
            }

            bool grasp_t::operator==(const grasp_t& g)
            {
                return (release_mode == g.release_mode && grasping_mode == g.grasping_mode && relative_config.is_approximate_equal(g.relative_config));
            }

            std::ostream& operator<<(std::ostream& os, const grasp_t& grasp)
            {
                os << "-";
                os << std::endl << "  relative_config: " << grasp.relative_config.serialize();
                os << std::endl << "  grasp_mode:  " << grasp.grasping_mode;
                os << std::endl << "  release_mode:  " << grasp.release_mode;
                os << std::endl;
            }
        }

    }
}
