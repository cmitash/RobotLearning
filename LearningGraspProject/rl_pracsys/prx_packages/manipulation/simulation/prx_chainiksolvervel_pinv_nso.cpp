// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "simulation/prx_chainiksolvervel_pinv_nso.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <kdl/frames_io.hpp>

namespace prx
{
    namespace sim
    {
        namespace simulation
        {
            extern double simulation_step;            
        }
    }
    
    namespace packages
    {
        using namespace util;
        using namespace sim;
        using namespace KDL;

        namespace manipulation
        {
            prx_chainiksolvervel_pinv_nso::prx_chainiksolvervel_pinv_nso(const Chain& _chain, JntArray _opt_pos, JntArray _weights, double _eps, int _maxiter, double _alpha):
                chain(_chain),
                jnt2jac(chain),
                nj(chain.getNrOfJoints()),
                jac(nj),
                U(MatrixXd::Zero(6,nj)),
                I(MatrixXd::Identity(nj,nj)),
                S(VectorXd::Zero(nj)),
                Sinv(VectorXd::Zero(nj)),
                SinvL(VectorXd::Zero(nj)),
                qmin(chain.getNrOfJoints()),
                qmax(chain.getNrOfJoints()),
                V(MatrixXd::Zero(nj,nj)),
                tmp(VectorXd::Zero(nj)),
                tmp2(VectorXd::Zero(nj)),
                eps(_eps),
                maxiter(_maxiter),
                alpha(_alpha),
                weights(_weights),
                opt_pos(_opt_pos)
            {
            }

            prx_chainiksolvervel_pinv_nso::prx_chainiksolvervel_pinv_nso(const Chain& _chain, double _eps, int _maxiter, double _alpha):
                chain(_chain),
                jnt2jac(chain),
                nj(chain.getNrOfJoints()),
                jac(nj),
                U(MatrixXd::Zero(6,nj)),
                I(MatrixXd::Identity(nj,nj)),
                S(VectorXd::Zero(nj)),
                Sinv(VectorXd::Zero(nj)),
                SinvL(VectorXd::Zero(nj)),
                ones(VectorXd::Ones(nj)),
                qmin(chain.getNrOfJoints()),
                qmax(chain.getNrOfJoints()),
                V(MatrixXd::Zero(nj,nj)),
                tmp(VectorXd::Zero(nj)),
                tmp2(VectorXd::Zero(nj)),
                eps(_eps),
                maxiter(_maxiter),
                alpha(_alpha)
            {
            }

            prx_chainiksolvervel_pinv_nso::~prx_chainiksolvervel_pinv_nso()
            {
            }  


            int prx_chainiksolvervel_pinv_nso::CartToJntChiaverini(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                jnt2jac.JntToJac(q_in,jac);

                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    qdot_out.data.setZero() ;
                    return svdResult;
                }

                if( S(5) < PRX_ZERO_CHECK )
                {
                    PRX_FATAL_S("SVD decomposition gave something not full-rank!");
                }

                nj = q_in.data.size();
                unsigned int i;
                double lambda = 1e-3;

                for (i = 0; i < nj; ++i) 
                {
                    SinvL(i) = S(i)/(S(i)*S(i)+lambda*lambda);
                }
                for (i = 0; i < 6; ++i) 
                {
                    tmp(i) = v_in(i);
                }
                MatrixXd temp_mat = MatrixXd::Zero(nj,1);
                VectorXd ones = VectorXd::Ones(nj);
                MatrixXd null_space_movement = MatrixXd::Zero(nj,1);
                mt_v_m(temp_mat,SinvL,U,tmp.head(6),V,0,5);
                qdot_out.data = temp_mat;

                double min_diff=9999;
                int index_best = -1;
                alpha = 0;
                for (i = 0; i < nj; ++i) 
                {
                    double range = qmax(i)-qmin(i);

                    double s_i;
                    double q_max_val = qmax(i)-range/10.0;
                    double q_min_val = qmin(i)+range/10.0;
                    if(q_in(i) > q_max_val)
                        s_i = (q_in(i) - q_max_val)/range;
                    else if(q_in(i) < q_min_val)
                        s_i = (q_in(i) - q_min_val)/range;
                    else
                        s_i = 0;

                    if(fabs(q_in(i)+.0002*qdot_out(i)-qmax(i)) < min_diff )
                    {
                        min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmax(i));
                        index_best = i;
                    }
                    if(fabs(q_in(i)+.0002*qdot_out(i)-qmin(i)) < min_diff )
                    {
                        min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmin(i));
                        index_best = i;
                    }

                    tmp(i) = s_i;
                }

                MatrixXd tmp_mat2 = MatrixXd::Zero(nj,1);
                mt_v_m(tmp_mat2,ones,V,tmp,V,6,8);
                // if(index_best!=0)
                //     std::cout<<"index"<<index_best<<std::endl;
                if(tmp(index_best)==0)
                {
                    alpha = 0;

                    ones*=-alpha;

                    mt_v_m(null_space_movement,ones,V,tmp_mat2,V,6,8);

                    qdot_out.data += null_space_movement;
                }
                else
                {   
                    // PRX_WARN_S("aCTUALLY APPLYING");
                    alpha = qdot_out(index_best)/(tmp_mat2(index_best,0));

                    ones*=-alpha;

                    // PRX_INFO_S("\n"<<qdot_out.data);
                    mt_v_m(null_space_movement,ones,V,tmp_mat2,V,6,8);

                    // PRX_INFO_S("\n"<<null_space_movement);
                    qdot_out.data += null_space_movement;
                    // PRX_INFO_S("\n"<<qdot_out.data);
                }

                return svdResult;

            }

            int prx_chainiksolvervel_pinv_nso::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
            {

                jnt2jac.JntToJac(q_in,jac);

                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    qdot_out.data.setZero() ;
                    return svdResult;
                }

                unsigned int i;
                double lambda = 1e-3;// 1e-4;

                for (i = 0; i < nj; ++i) {
                    SinvL(i) = S(i)/(S(i)*S(i)+lambda*lambda);
                    // SinvL(i) = fabs(S(i))<eps ? 0 : 1.0/S(i);
                    // S(i) = fabs(S(i))<eps ? 0 : S(i);//
                    // SinvL(i) = 1.0/S(i);
                }
                for (i = 0; i < 6; ++i) {
                    tmp(i) = v_in(i);
                }

                //This is the equivalent computation using the vector cross products method
                // MatrixXd VSiUt(MatrixXd::Zero(nj,6));
                // vector_cross_product( VSiUt, SinvL, V, U, 0, 5 );
                // qdot_out.data = VSiUt * tmp.head( 6 );
                
                qdot_out.data = V * SinvL.asDiagonal() * U.transpose() * tmp.head(6);

                //This pre-empts any NULL-space stuff
                return svdResult;

                // double min_diff=9999;
                // int index_best = -1;
                // alpha = 0;
                // for (i = 0; i < nj; ++i) 
                // {
                //     double range = qmax(i)-qmin(i);

                //     double s_i;
                //     double q_max_val = qmax(i)-range/10.0;
                //     double q_min_val = qmin(i)+range/10.0;
                //     if(q_in(i) > q_max_val)
                //         s_i = (q_in(i) - q_max_val)/range;
                //     else if(q_in(i) < q_min_val)
                //         s_i = (q_in(i) - q_min_val)/range;
                //     else
                //         s_i = 0;

                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmax(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmax(i));
                //         index_best = i;
                //     }
                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmin(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmin(i));
                //         index_best = i;
                //     }

                //     tmp2(i) = s_i;
                // }

                MatrixXd temp_mat = MatrixXd::Zero(nj,1);
                mt_v_m(temp_mat,SinvL,U,tmp.head(6),V,0,5);

                qdot_out.data = temp_mat;
                // qdot_out.data = V * SinvL.asDiagonal() * U.transpose() * tmp.head(6);

                return svdResult;

                /////////////////// basic bounds avoidance
                // double min_diff=9999;
                // int index_best = -1;
                // alpha = 0;
                // for (i = 0; i < nj; ++i) 
                // {
                //     double range = qmax(i)-qmin(i);

                //     double s_i;
                //     double q_max_val = qmax(i)-range/10.0;
                //     double q_min_val = qmin(i)+range/10.0;
                //     if(q_in(i) > q_max_val)
                //         s_i = (q_in(i) - q_max_val)/range;
                //     else if(q_in(i) < q_min_val)
                //         s_i = (q_in(i) - q_min_val)/range;
                //     else
                //         s_i = 0;

                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmax(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmax(i));
                //         index_best = i;
                //     }
                //     if(fabs(q_in(i)+.0002*qdot_out(i)-qmin(i)) < min_diff )
                //     {
                //         min_diff = fabs(q_in(i)+.0002*qdot_out(i)-qmin(i));
                //         index_best = i;
                //     }

                //   tmp(i) = s_i;
                // }
                // MatrixXd temp_mat1 = MatrixXd::Zero(nj,nj);
                // vector_cross_product(temp_mat1,V,V,0,5);
                // MatrixXd proj = I - temp_mat1;

                // tmp2 = (proj3*tmp);
                // // if(index_best!=0)
                // //     std::cout<<"index"<<index_best<<std::endl;
                // if(tmp(index_best)==0)
                //     alpha = 0;
                // else
                //     alpha = qdot_out(index_best)/(tmp2(index_best));

                // tmp2 = -alpha*(proj3*tmp);
                // for (i = 0; i < nj; ++i) 
                // {
                //     qdot_out(i) += tmp2(i);
                // }
                // //return the return value of the svd decomposition
                // return svdResult;
                ///////////////////////


                //////////////////////////////////old existing code
                // double g = 0; // g(q)
                // double A = 0; // normalizing term
                // for (i = 0; i < nj; ++i) {
                //     double qd = q_in(i) - opt_pos(i);
                //     g += 0.5 * qd*qd * weights(i);
                //     A += qd*qd * weights(i)*weights(i);
                // }

                // if (A > 1e-9) {
                //   // Calculate inverse Jacobian Jc^-1
                //   for (i = 0; i < nj; ++i) {
                //       tmp(i) = weights(i)*(q_in(i) - opt_pos(i)) / A;
                //   }

                //   // Calcualte J^-1 * J * Jc^-1 = V*S^-1*U' * U*S*V' * tmp
                //   tmp2 = V * Sinv.asDiagonal() * U.transpose() * U * S.asDiagonal() * V.transpose() * tmp;

                //   for (i = 0; i < nj; ++i) {
                //       //std::cerr << i <<": "<< qdot_out(i) <<", "<< -2*alpha*g * (tmp(i) - tmp2(i)) << std::endl;
                //       qdot_out(i) += -2*alpha*g * (tmp(i) - tmp2(i));
                //   }
                // }
                // //return the return value of the svd decomposition
                // return svdResult;
            }


            int prx_chainiksolvervel_pinv_nso::CartToJntSDLS(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                // PRX_PRINT("================ START Cart to Joint (SDLS) =====================", PRX_TEXT_BLUE);
                // PRX_PRINT("Twist: " << v_in, PRX_TEXT_LIGHTGRAY);
                
                jnt2jac.JntToJac(q_in,jac);

                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    qdot_out.data.setZero() ;
                    return svdResult;
                }

                if( S(5) < PRX_ZERO_CHECK )
                {
                    PRX_FATAL_S("SVD decomposition gave something not full-rank!");
                }

                //Get the dimensionality of our C-space
                nj = q_in.data.size();
                unsigned int i;

                //Get our e vector copied over
                for (i = 0; i < 6; ++i) 
                {
                    Sinv(i) = 1.0/S(i);
                }
                for (i = 0; i < 6; ++i)
                {
                    tmp(i) = v_in(i);
                }

                //If we are close to our goal, just turn off damping?
                if( v_in.vel.Norm() < 0.01 )
                {
                    qdot_out.data = V * Sinv.asDiagonal() * U.transpose() * tmp.head(6);
                    // PRX_PRINT("Qdot Normal: " << qdot_out.data.norm(), PRX_TEXT_GREEN);
                    return svdResult;
                }
                
                //First, compute the SDLS damping values
                double gamma_max = 0.78; //0.262;//Approx pi/12
                Eigen::VectorXd gamma( VectorXd::Zero(6) );
                compute_SDLS_damping( gamma, gamma_max );
                
                // PRX_PRINT("Damping values:", PRX_TEXT_BROWN);
                // PRX_PRINT("gamma: \n" << gamma, PRX_TEXT_LIGHTGRAY);
                
                //Next, we clamp based on the individual damping factors
                Eigen::MatrixXd phi_i (MatrixXd::Zero( nj, 1 ));
                Eigen::VectorXd phi_i_vec (VectorXd::Zero( nj ));
                // PRX_PRINT("Checking for vector clampage...", PRX_TEXT_MAGENTA);
                for( unsigned i=0; i<6; ++i )
                {
                    // PRX_STATUS("[" << i << "]", PRX_TEXT_LIGHTGRAY);
                    mt_v_m( phi_i, Sinv, U, tmp.head(6), V, 0, 5 );
                    //Convert the matrix result into a vector
                    for( unsigned k=0; k<nj; ++k )
                    {
                        phi_i_vec(k) = phi_i.col(0)(k);
                    }
                    clamp_max_abs( phi_i_vec, gamma(i) );
                    //Go ahead and add this to the resulting q^dot
                    qdot_out.data += phi_i_vec;
                }
                
                // PRX_PRINT("Checking for GLOBAL clampage...", PRX_TEXT_MAGENTA);
                //Then, the computed q^dot is the clamping over the sum of the PHI vectors
                clamp_max_abs( qdot_out.data, gamma_max );

                // PRX_PRINT("Qdot Damping: " << qdot_out.data.norm(), PRX_TEXT_BLUE);
                // PRX_PRINT("qdot: \n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);
                // qdot_out.data.normalize();
                // PRX_PRINT("qdot_norm: \n" << qdot_out.data, PRX_TEXT_CYAN);

                //This pre-empts any NULL-space stuff
                return svdResult;
            }


            int prx_chainiksolvervel_pinv_nso::CartToJntSDLS_SVF(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                // PRX_PRINT("================ START Cart to Joint (SDLS + SVF) =====================", PRX_TEXT_BLUE);
                // PRX_PRINT("Twist: " << v_in, PRX_TEXT_LIGHTGRAY);
                
                jnt2jac.JntToJac(q_in,jac);

                double curve = 10;
                double sigma_0 = 0.005;

                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    qdot_out.data.setZero() ;
                    return svdResult;
                }

                if( S(5) < PRX_ZERO_CHECK )
                {
                    PRX_FATAL_S("SVD decomposition gave something not full-rank!");
                }

                //Get the dimensionality of our C-space
                nj = q_in.data.size();
                unsigned int i;

                //Get our e vector copied over
                for (i = 0; i < 6; ++i) 
                {
                    Sinv(i) = 1.0 / ( ( (S(i)*S(i)*S(i)) + (curve*S(i)*S(i)) + (2*S(i)) + (2*sigma_0) ) /
                                      ( (S(i)*S(i)) + (curve*S(i)) + 2 ) );
                }
                for (i = 0; i < 6; ++i)
                {
                    tmp(i) = v_in(i);
                }

                //If we are close to our goal, just turn off damping?
                if( v_in.vel.Norm() < 0.01 )
                {
                    qdot_out.data = V * Sinv.asDiagonal() * U.transpose() * tmp.head(6);
                    // PRX_PRINT("Qdot Normal: " << qdot_out.data.norm(), PRX_TEXT_GREEN);
                    return svdResult;
                }
                
                //First, compute the SDLS damping values
                double gamma_max = 0.78; //0.262;//Approx pi/12
                Eigen::VectorXd gamma( VectorXd::Zero(6) );
                compute_SDLS_damping( gamma, gamma_max );
                
                // PRX_PRINT("Damping values:", PRX_TEXT_BROWN);
                // PRX_PRINT("gamma: \n" << gamma, PRX_TEXT_LIGHTGRAY);
                
                //Next, we clamp based on the individual damping factors
                Eigen::MatrixXd phi_i (MatrixXd::Zero( nj, 1 ));
                Eigen::VectorXd phi_i_vec (VectorXd::Zero( nj ));
                // PRX_PRINT("Checking for vector clampage...", PRX_TEXT_MAGENTA);
                for( unsigned i=0; i<6; ++i )
                {
                    // PRX_STATUS("[" << i << "]", PRX_TEXT_LIGHTGRAY);
                    mt_v_m( phi_i, Sinv, U, tmp.head(6), V, 0, 5 );
                    //Convert the matrix result into a vector
                    for( unsigned k=0; k<nj; ++k )
                    {
                        phi_i_vec(k) = phi_i.col(0)(k);
                    }
                    clamp_max_abs( phi_i_vec, gamma(i) );
                    //Go ahead and add this to the resulting q^dot
                    qdot_out.data += phi_i_vec;
                }
                
                // PRX_PRINT("Checking for GLOBAL clampage...", PRX_TEXT_MAGENTA);
                //Then, the computed q^dot is the clamping over the sum of the PHI vectors
                clamp_max_abs( qdot_out.data, gamma_max );

                // PRX_PRINT("Qdot Damping: " << qdot_out.data.norm(), PRX_TEXT_BLUE);
                // PRX_PRINT("qdot: \n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);
                // qdot_out.data.normalize();
                // PRX_PRINT("qdot_norm: \n" << qdot_out.data, PRX_TEXT_CYAN);

                //This pre-empts any NULL-space stuff
                return svdResult;
            }

            void prx_chainiksolvervel_pinv_nso::compute_SDLS_damping( Eigen::VectorXd& gamma, double gamma_max )
            {
                Eigen::VectorXd M(VectorXd::Zero(6));
                Eigen::VectorXd N(VectorXd::Zero(6));
                
                //First, compute N_i
                for(int i = 0; i < 6; ++i)
                {
                    //In our case, we have a single end-effector, so k=1
                    N(i) = U.col(i).norm();
                }
                
                //Then, compute M_i
                Eigen::VectorXd col_j( Eigen::VectorXd::Zero(6) );
                for( int i=0; i<6; ++i )
                {
                    for( int j=0; j<nj; ++j )
                    {
                        for( unsigned t=0; t<6; ++t )
                        {
                            col_j( t ) = jac.getColumn(j)(t);
                        }
                        M(i) += fabs(V.row(j)(i)) * col_j.norm();
                    }
                    //Note : S(i) should never be so small that this returns zero!
                    M(i) *= (fabs(S(i)) < eps ? 0 : 1.0/S(i));
                }
                
                //Finally, get the final gamma values (damping) for each DoF
                for( int i=0; i<6; ++i )
                {
                    gamma(i) = PRX_MINIMUM( 1, N(i)/M(i) ) * gamma_max;
                }
            }
            
            int prx_chainiksolvervel_pinv_nso::CartToJntClamping(const KDL::JntArray& q_in, const std::vector< bounds_t* >& q_bounds, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
            {
                unsigned int i;
                double lambda = 1e-3;

                //Compute Jacobian
                jnt2jac.JntToJac(q_in,jac);

                //Determine our dimensionality
                nj = q_in.data.size();

                //Initially, none of the joints are clamped
                Eigen::VectorXd proj_j0( VectorXd::Ones(nj) );
                
                //Start off in a "clamping" state to make sure the loop starts
                bool clamping_detected = true;
                int svdResult;
                
                while( clamping_detected && proj_j0.count() >= 6 )
                {
                    //Compute the SVD of the Jacobian
                    svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                    
                    //If the SVD does not work out
                    if (0 != svdResult)
                    {
                        //Report the failure
                        qdot_out.data.setZero() ;
                        return svdResult;
                    }

                    //Compute the Damped Pseudoinverse
                    for (i = 0; i < nj; ++i) 
                    {
                        SinvL(i) = S(i)/(S(i)*S(i)+lambda*lambda);
                    }
                    //Copy over the input velocity (unfortunately have to do this because SVD trashes it?)
                    for (i = 0; i < 6; ++i) 
                    {
                        tmp(i) = v_in(i);
                    }

                    //Compute q^dot 
                    qdot_out.data = V * SinvL.asDiagonal() * U.transpose() * tmp.head(6);                    

                    //Now, we assume no clamping has happened
                    clamping_detected = false;

                    //Then, for each DoF
                    for( i=0; i<nj; ++i )
                    {
                        //If that DoF is not yet clamped
                        if( proj_j0(i) != 0 )
                        {
                            //Compute the theta update for this DoF (Quick, crummy approximation)
                            double theta_up = q_in(i) + simulation::simulation_step * qdot_out(i);
                            double DTCi = 0;
                            //If the update is not within bounds
                            if( !q_bounds[i]->is_valid( theta_up ) )
                            {
                                // PRX_PRINT("[" << i << "] has clampage!", PRX_TEXT_BROWN);
                                
                                //Need to get Delta Theta_Ci (The clamping variation)
                                if( q_bounds[i]->get_lower_bound() > theta_up )
                                {
                                    DTCi = theta_up - q_bounds[i]->get_lower_bound();
                                }
                                else if( q_bounds[i]->get_upper_bound() < theta_up )
                                {
                                    DTCi = theta_up - q_bounds[i]->get_upper_bound();
                                }
                                else
                                {
                                    PRX_FATAL_S("This should not ever have happened!");
                                }
                                
                                //We have detected a clamping
                                clamping_detected = true;
                                //Need to extract J_i as a vector
                                Eigen::VectorXd J_i_vec (VectorXd::Zero(6));
                                KDL::Twist J_i = jac.getColumn(i);
                                for( unsigned t=0; t<6; ++t )
                                {
                                    J_i_vec( t ) = J_i( t );
                                }
                                //Then, tmp -= J_i DEL Theta_Ci
                                tmp -= J_i_vec * DTCi;
                                //Set theta_i to the breached limit  (I'm not sure I do have to explicitly do this)
                                
                                //Zero out J_i
                                jac.setColumn( i, KDL::Twist::Zero() );
                                //Zero out the Jacobian_naught projection at i
                                proj_j0(i) = 0;
                            }
                        }
                    }
                }
                return svdResult;
            }


            int prx_chainiksolvervel_pinv_nso::CartToJnt_IDF(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
            {
                //Go get Mr. Jacobean
                jnt2jac.JntToJac(q_in,jac);

                //Perform SVD
                int svdResult = svd_eigen_HH(jac.data,U,S,V,tmp,maxiter);
                if (0 != svdResult)
                {
                    //Report out if SVD decomp does not work
                    qdot_out.data.setZero() ;
                    return svdResult;
                }
                //Always be checkin' for full rankin'
                if( S(5) < PRX_ZERO_CHECK )
                {
                    PRX_FATAL_S("SVD decomposition gave something not full-rank!");
                }

                // PRX_PRINT("Decomposition complete:", PRX_TEXT_BLUE);
                // PRX_PRINT("S: \n" << S, PRX_TEXT_LIGHTGRAY);

                //Some parameters
                nj = q_in.data.size();
                unsigned int i;
                double lambda = 1e-3;
                
                // PRX_PRINT("Distance: " << v_in.vel.Norm(), PRX_TEXT_LIGHTGRAY);
                
                //We need to compute a substitute for lamda^2 (If sigma_m is sufficiently large, we will use no lambda damping)
                double lambda_sq = 0;
                if( S(5) < eps )
                {
                    double s = ( S(5)/eps );
                    lambda_sq = ( lambda * lambda ) * ( 1.0 - ( s * s ) );
                }
                double beta_sq = lambda_sq * 1e-4;
                
                //For the first m-1 singular values
                for (i = 0; i < 5; ++i)
                {
                    //We perform damping with beta squared instead of lambda
                    SinvL(i) = S(i)/(S(i)*S(i)+beta_sq);
                }
                //For the mth singular value, we use beta squared and lambda squared
                SinvL(5) = S(5)/( S(5)*S(5) + beta_sq + lambda_sq );
                //All of the singular values at 7 (index 6) and above should be 0 already...
                for( i=6; i<nj; ++i )
                {
                    SinvL(i) = 0;
                }
                
                // PRX_PRINT("Computed Pseudoinverse:", PRX_TEXT_MAGENTA);
                // PRX_PRINT("J^o: \n" << SinvL, PRX_TEXT_LIGHTGRAY);
                
                //Copy over x^dot
                for (i = 0; i < 6; ++i) 
                {
                    tmp(i) = v_in(i);
                }

                //This is the equivalent computation using the vector cross products method
                // MatrixXd VSiUt(MatrixXd::Zero(nj,6));
                // vector_cross_product( VSiUt, SinvL, V, U, 0, 5 );
                // qdot_out.data = VSiUt * tmp.head( 6 );
                
                qdot_out.data = V * SinvL.asDiagonal() * U.transpose() * tmp.head(6);
                
                // PRX_PRINT("Results in a q^dot", PRX_TEXT_GREEN);
                // PRX_PRINT("q^dot:\n" << qdot_out.data, PRX_TEXT_LIGHTGRAY);

                //This pre-empts any NULL-space stuff
                return svdResult;
            }

            void prx_chainiksolvervel_pinv_nso::vector_cross_product( Eigen::MatrixXd & result, const Eigen::VectorXd & constants, const Eigen::MatrixXd & left, const Eigen::MatrixXd & right, unsigned min_range, unsigned max_range )
            {
                result.setZero();
                for( unsigned i=min_range; i<max_range+1; ++i )
                {
                    result += constants(i) * left.col(i) * right.col(i).transpose();
                }
            }

            void prx_chainiksolvervel_pinv_nso::mt_v_m( Eigen::MatrixXd & result, const Eigen::VectorXd & constants, const Eigen::MatrixXd & left, const Eigen::VectorXd & vec, const Eigen::MatrixXd & right, unsigned min_range, unsigned max_range )
            {
                result.setZero();
                for( unsigned i=min_range; i<max_range+1; ++i )
                {
                    Eigen::VectorXd val(constants(i) * (left.col(i).transpose() * vec));
                    result +=  (val(0) * right.col(i));
                }
            }            

            void prx_chainiksolvervel_pinv_nso::clamp_max_abs( Eigen::VectorXd & w, double d )
            {
                double norm = 0;// w.lpNorm<1>();
                for( unsigned i=0; i<w.size(); ++i )
                {
                    norm = PRX_MAXIMUM( norm, fabs(w(i)) );
                }
                if( norm > d )
                {
                    // PRX_PRINT("Clampage executing over: \n" << w, PRX_TEXT_RED);
                    w *= d;
                    w /= norm;
                    // PRX_PRINT("Created:\n" << w, PRX_TEXT_LIGHTGRAY);
                }
            }

            int prx_chainiksolvervel_pinv_nso::setWeights(const JntArray & _weights)
            {
              weights = _weights;
              return 0;
            }

            int prx_chainiksolvervel_pinv_nso::setOptPos(const JntArray & _opt_pos)
            {
              opt_pos = _opt_pos;
              return 0;
            }

            int prx_chainiksolvervel_pinv_nso::setAlpha(const double _alpha)
            {
              alpha = _alpha;
              return 0;
            }
        }
    }
}
