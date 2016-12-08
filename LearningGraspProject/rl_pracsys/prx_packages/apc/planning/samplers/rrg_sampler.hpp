#pragma once

#ifndef PRX_RRG_SAMPLER_HPP
#define PRX_RRG_SAMPLER_HPP

#include "prx/planning/modules/samplers/sampler.hpp"
#include "prx/utilities/math/configurations/bounds.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"

namespace prx 
{ 
     namespace packages 
     {
        namespace manipulation
        {
            class manipulation_world_model_t;
        }
        namespace apc
        {
            class rrg_sampler_t : public plan::sampler_t
            {
                
                public:
                    
                    rrg_sampler_t() : sampler_t() 
                    {
                        manip_model = NULL;
                    }

                    virtual ~rrg_sampler_t() {};
                    
                    virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader = NULL);

                    /**
                     * @copydoc sampler_t::sample(const space_t*, space_point_t*)
                     */
                    virtual void sample(const util::space_t* space,util::space_point_t* point);

                    /**
                     * @copydoc sampler_t::sample_near(const space_t*, space_point_t*, std::vector<bounds_t*>&, space_point_t*)
                     */
                    virtual void sample_near(const util::space_t* space, util::space_point_t* near_point, std::vector<util::bounds_t*>& bounds, util::space_point_t* point);


                    virtual void link_world_model(plan::world_model_t* wm);
                protected:
                    manipulation::manipulation_world_model_t* manip_model;
                    std::vector<util::bounds_t*> bounds;
                    util::quaternion_t quat;
                    std::string manipulation_context;
                    util::space_point_t* manipulation_state;
                    std::vector<util::bounds_t*> initial_sample_bounds;
                    std::vector<util::bounds_t*> euler_bounds;
                    int tries;
                    int first_sample_tries;
                    bool rejection_sampling;
            };
        }

    } 
}

#endif