/**
 * @file grasp_visualizer.cpp 
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

#include "simulation/systems/controllers/grasp_visualizer.hpp"
#include "simulation/systems/plants/movable_body_plant.hpp"
#include "prx/simulation/simulators/simulator.hpp"

#include <boost/assign/list_of.hpp>
#include <pluginlib/class_list_macros.h>
#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

PLUGINLIB_EXPORT_CLASS(prx::packages::manipulation::grasp_visualizer_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace manipulation
        {

            std::vector<double> end_state = boost::assign::list_of(0.0)(0.0)(0.0)(0.0)(0.0)(0.0)(0.0)(0.0);

            grasp_visualizer_t::grasp_visualizer_t()
            {
                current_grasp_state=0;
            }

            grasp_visualizer_t::~grasp_visualizer_t()
            {
            }

            void grasp_visualizer_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
            {
                // std::cout << std::endl << "@@@MESSAGE!!!" << std::endl;
                // PRX_FATAL_S("Grasping visualizer requires an object and a floating manipulator");
                controller_t::init(reader, template_reader);
                std::string grasp_file = parameters::get_attribute_as<std::string > ("grasp_file", reader, template_reader);
                std::string start_link = parameters::get_attribute_as<std::string > ("start_link", reader, template_reader,"");
                std::string end_link = parameters::get_attribute_as<std::string > ("end_link", reader, template_reader,"");

                movable_body_plant_t* object=NULL;
                manipulator = NULL;

                foreach(std::string name, subsystem_names)
                {
                    if(manipulator==NULL)
                        manipulator = dynamic_cast<manipulator_t*>(subsystems[name].get());
                    if(object==NULL)
                        object = dynamic_cast<movable_body_plant_t*>(subsystems[name].get());
                }
                if(manipulator==NULL || object == NULL)
                {
                    PRX_FATAL_S("Grasping visualizer requires an object and a floating manipulator");
                }

                config_t ee_config;
                config_t grasp_config;
                config_t object_config;
                config_t config;
                object->get_configuration(object_config);
                manipulator->get_end_effector_configuration(ee_config, 0);
                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                filename+="/"+grasp_file;
                if(boost::filesystem::exists( filename ))
                {
                    grasping_states.link_space(manipulator->get_state_space());
                    state_t* temp_state = manipulator->get_state_space()->alloc_point();
                    std::vector<double> temp_vec(7);

                    YAML::Node doc = YAML::LoadFile(filename);
                    PRX_INFO_S(filename);
                    unsigned i;
                    for( i=0;i<doc.size();i++)
                    {
                        for( unsigned j=0; j < 7; ++j )
                        {
                            temp_vec[j] = doc[i]["relative_config"][j].as<double>();
                        }
                        grasp_config.set_position(temp_vec[0],temp_vec[1],temp_vec[2]);
                        grasp_config.set_xyzw_orientation(temp_vec[3],temp_vec[4],temp_vec[5],temp_vec[6]);
                        if(i==0)
                        {
                            PRX_INFO_S(ee_config);
                            PRX_INFO_S(grasp_config);
                            PRX_INFO_S(object_config);
                        }

                        // config = ee_config;
                        // if(i==0)
                        // {
                        //     PRX_INFO_S(config);
                        // }
                        // config.relative_to_global(grasp_config);
                        // if(i==0)
                        // {
                        //     PRX_INFO_S(config);
                        // }
                        // config.relative_to_global(object_config);
                        // if(i==0)
                        // {
                        //     PRX_INFO_S(config);
                        // }

                        int grasp_mode = doc[i]["grasp_mode"].as<int>();
                        manipulator->IK_solver(temp_state,temp_state,grasp_config,start_link,end_link);
                        temp_state->at(7) = grasp_mode;

                        grasping_states.copy_onto_back(temp_state);

                        // Testing adding incremental waypoint config for 
                        // each relative config loaded

                        std::vector<double> vec1, vec2, vec3;
                        prx::quaternion_t q1;
                        prx::config_t incr_config;
                        object_config.get_position(vec1);
                        grasp_config.get_position(vec2);
                        q1 = grasp_config.get_orientation();
                        vec3.resize(3);
                        vec3[0] = vec1[0] - vec2[0];
                        vec3[1] = vec1[1] - vec2[1];        
                        vec3[2] = vec1[2] - vec2[2];
                        // std::cout << "w -> obj: " << vec1 << std::endl;
                        // std::cout << "w -> ee: " << vec2 << std::endl;
                        prx::vector_t res_vec(vec3[0],vec3[1],vec3[2]);
                        // res_vec = q1.qv_rotation(res_vec);
                        vec3[0] = res_vec[0];//+ vec2[0];
                        vec3[1] = res_vec[1];//+ vec2[1];
                        vec3[2] = res_vec[2];//+ vec2[2];
                        // std::cout << "new vec: " << vec3 << std::endl;
                        double mag = sqrt((vec3[0] * vec3[0]) + (vec3[1] * vec3[1]) + (vec3[2] * vec3[2]));

                        std::vector<double> new_vec;
                        new_vec.resize(3);
                        for (int i = 0; i < 99; i++) {                            
                            new_vec[0] = ((vec3[0] / mag) * 0.003) + vec2[0];
                            new_vec[1] = ((vec3[1] / mag) * 0.003) + vec2[1];
                            new_vec[2] = ((vec3[2] / mag) * 0.003) + vec2[2];

                            vec2 = new_vec;

                            incr_config.set_position(new_vec[0], new_vec[1], new_vec[2]);
                            incr_config.set_xyzw_orientation(q1[0], q1[1], q1[2], q1[3]);

                            manipulator->IK_solver(temp_state,temp_state,incr_config,start_link,end_link);
                            temp_state->at(7) = grasp_mode;

                            grasping_states.copy_onto_back(temp_state);

                        }

                    }
                    manipulator->get_state_space()->free_point(temp_state);
                }
                else
                {
                    PRX_FATAL_S("Grasping visualizer received incorrect filename: "<<filename);
                }

            }

            void grasp_visualizer_t::compute_control()
            {
                manipulator->get_control_space()->copy_from_point(grasping_states[current_grasp_state++%grasping_states.size()]);
                std::vector<double> curr_state;

                manipulator->get_control_space()->copy_to_vector(curr_state);
                PRX_PRINT("Grasp ID: "<<(current_grasp_state-1)%grasping_states.size(), PRX_TEXT_BLUE);

                bool in_collision = simulation::simulator_ptr->in_collision();                
                
                config_t finger_1_current;
                config_t finger_2_current;
                config_t finger_3_current;

                finger_1.lookup("simulator/consumer/manipulator/distal_1", finger_1_current);
                finger_2.lookup("simulator/consumer/manipulator/distal_2", finger_2_current);
                finger_3.lookup("simulator/consumer/manipulator/distal_3", finger_3_current);                  

                if (in_collision) {
                    if (!simulation::found_end) {
                        simulation::found_end = true;
                        manipulator->get_control_space()->copy_to_vector(end_state);

                        finger_1.lookup("simulator/consumer/manipulator/distal_1", finger_1_config);
                        finger_2.lookup("simulator/consumer/manipulator/distal_2", finger_2_config);
                        finger_3.lookup("simulator/consumer/manipulator/distal_3", finger_3_config);                  
                    }
                    PRX_PRINT("In Collision: "<< in_collision, PRX_TEXT_RED);
                }
                if (current_grasp_state % 100 == 0) {
                    if (!simulation::found_end) {
                        manipulator->get_control_space()->copy_to_vector(end_state);
                        
                        finger_1.lookup("simulator/consumer/manipulator/distal_1", finger_1_config);
                        finger_2.lookup("simulator/consumer/manipulator/distal_2", finger_2_config);
                        finger_3.lookup("simulator/consumer/manipulator/distal_3", finger_3_config);
                    }

                    PRX_PRINT("Trajectory End State: " << end_state, PRX_TEXT_MAGENTA);
                    PRX_PRINT("Finger 1 End State: " << finger_1_config, PRX_TEXT_MAGENTA);
                    PRX_PRINT("Finger 2 End State: " << finger_2_config, PRX_TEXT_MAGENTA);
                    PRX_PRINT("Finger 3 End State: " << finger_3_config, PRX_TEXT_MAGENTA);
                    simulation::found_end = false;
                }
                
            }

            void grasp_visualizer_t::read_database_file()
            {
                grasping_states.clear();

                config_t ee_config;
                config_t grasp_config;
                config_t object_config;
                config_t config;
                object->get_configuration(object_config);

                // Find out which file to load
                std::string top_front_face;
                std::vector<std::string> alternative_faces;
                determine_top_and_front_faces(object, top_front_face, alternative_faces);

                manipulator->get_end_effector_configuration(ee_config, 0);
                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                filename+="/"+grasp_path+"/"+object->get_object_type()+top_front_face+".yaml";
                PRX_PRINT("FILENAME: " << filename, PRX_TEXT_GREEN);

                if(boost::filesystem::exists( filename ))
                {
                    grasping_states.link_space(manipulator->get_state_space());
                    state_t* temp_state = manipulator->get_state_space()->alloc_point();
                    std::vector<double> temp_vec(7);

                    YAML::Node doc = YAML::LoadFile(filename);
                    PRX_INFO_S(filename);
                    unsigned i;
                    for( i=0;i<doc.size();i++)
                    {
                        for( unsigned j=0; j < 7; ++j )
                        {
                            temp_vec[j] = doc[i]["relative_config"][j].as<double>();
                        }
                        grasp_config.set_position(temp_vec[0],temp_vec[1],temp_vec[2]);
                        grasp_config.set_xyzw_orientation(temp_vec[3],temp_vec[4],temp_vec[5],temp_vec[6]);
                        if(i==0)
                        {
                            PRX_INFO_S(ee_config);
                            PRX_INFO_S(grasp_config);
                            PRX_INFO_S(object_config);
                        }

                        config = ee_config;
                        if(i==0)
                        {
                            PRX_INFO_S(config);
                        }
                        config.relative_to_global(grasp_config);
                        if(i==0)
                        {
                            PRX_INFO_S(config);
                        }
                        config.relative_to_global(object_config);
                        if(i==0)
                        {
                            PRX_INFO_S(config);
                        }

                        int grasp_mode = doc[i]["grasp_mode"].as<int>();
                        manipulator->IK_solver(temp_state,temp_state,config,start_link,end_link);
                        temp_state->at(7) = grasp_mode;

                        grasping_states.copy_onto_back(temp_state);
                    }
                    manipulator->get_state_space()->free_point(temp_state);
                }
                else
                {
                    PRX_FATAL_S("Grasping visualizer received incorrect filename: "<<filename);
                }
            }


            void grasp_visualizer_t::determine_top_and_front_faces(movable_body_plant_t* input_object, std::string& top_front_face, std::vector<std::string>& alternative_faces)
            {
                alternative_faces.clear();

                // if(original_object_state == NULL)
                //     original_object_state = object->get_state_space()->alloc_point();
                // else
                //     object->get_state_space()->copy_to_point(original_object_state);

                /** Get configuration of object **/
                config_t config;
                input_object->get_configuration(config);

                PRX_PRINT ("Object config in determine faces: " << config, PRX_TEXT_GREEN);

                /** Calculate rotation to apply to axis **/
                quaternion_t quat;
                quat.zero();
                quat/=config.get_orientation();

                /** Find Top Axis **/
                vector_t top_axis(0,0,1);
                std::string top_face;
                determine_face(top_axis, quat, top_face);

                /** Find Front Axis **/
                vector_t front_axis(-1,0,0);
                std::string front_face;
                determine_face(front_axis, quat, front_face);

                /** We've found the closest database, let's remember it now **/
                top_front_face = top_face+front_face;

                /** We want the first database evaluate to be the closest one **/
                alternative_faces.push_back(top_front_face);

                /** Let's add the remaining "nearest" databases **/
                if (front_face == "_x-" || front_face == "_x+")
                {
                    if (top_face == "_z-" || top_face == "_z+")
                    {
                        alternative_faces.push_back(top_face+"_y+");
                        alternative_faces.push_back(top_face+"_y-");
                    }
                    else
                    {
                        alternative_faces.push_back(top_face+"_z+");
                        alternative_faces.push_back(top_face+"_z-");
                    }

                }
                else if (front_face == "_y-" || front_face == "_y+")
                {
                    if (top_face == "_z-" || top_face == "_z+")
                    {
                        alternative_faces.push_back(top_face+"_x+");
                        alternative_faces.push_back(top_face+"_x-");
                    }
                    else
                    {
                        alternative_faces.push_back(top_face+"_z+");
                        alternative_faces.push_back(top_face+"_z-");
                    }

                }
                else if (front_face == "_z-" || front_face == "_z+")
                {
                    if (top_face == "_x-" || top_face == "_x+")
                    {
                        alternative_faces.push_back(top_face+"_y+");
                        alternative_faces.push_back(top_face+"_y-");
                    }
                    else
                    {
                        alternative_faces.push_back(top_face+"_x+");
                        alternative_faces.push_back(top_face+"_x-");
                    }

                }


            }


            void grasp_visualizer_t::determine_face(const vector_t& axis, const quaternion_t& rotation, std::string& face)
            {
                /** Find Top Axis **/
                vector_t rotated_axis = rotation.qv_rotation(axis);
                double max_val = fabs(rotated_axis[0]);
                int max_index = 0;
                face = "_x";
                if(fabs(rotated_axis[1]) > max_val)
                {
                    max_val = fabs(rotated_axis[1]);
                    max_index = 1;
                    face = "_y";
                }
                if(fabs(rotated_axis[2]) > max_val)
                {
                    max_val = fabs(rotated_axis[2]);
                    max_index = 2;
                    face = "_z";
                }
                // PRX_INFO_S(max_index);
                if(rotated_axis[max_index]<0)
                {
                    face+= "-";
                }
                else
                {
                    face+= "+";
                }
            }
        }
    }
}
