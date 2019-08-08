// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_RBPF_H
#define RBPF_RBPF_H

#include "Particle.h"
#include <Eigen/StdVector>

using namespace Eigen;

void add_control_noise(float, float, MatrixXf&, int, float*);

void predict_true(VectorXf&, float, float, float, float);

void compute_steering(VectorXf&, MatrixXf&, int&, float, float&, float, float, float);

void data_associate_known(std::vector <VectorXf>&, std::vector<int>&, VectorXf&, int, std::vector <VectorXf>&, std::vector<int>&, std::vector<VectorXf>&);

void feature_update(Particle&, std::vector<VectorXf>&, std::vector<int>&, MatrixXf&);

std::vector<VectorXf> get_observations(VectorXf&, MatrixXf, std::vector<int>&, float);

void get_visible_landmarks(VectorXf&, MatrixXf&, std::vector<int>&, float);

std::vector<VectorXf> compute_range_bearing(VectorXf&, MatrixXf&);

std::vector<int> find2(std::vector<float>&, std::vector<float>&, float, float);

void KF_cholesky_update(VectorXf&, MatrixXf&, VectorXf&, MatrixXf&, MatrixXf&);

void KF_joseph_update(VectorXf&, MatrixXf&, float, float, MatrixXf&);

MatrixXf make_symmetric(MatrixXf&);

MatrixXf line_plot_conversion(MatrixXf&);

MatrixXf make_laser_lines(std::vector<VectorXf>&, VectorXf&);

void make_covariance_ellipse(MatrixXf&, MatrixXf&, MatrixXf&);

void add_observation_noise(std::vector <VectorXf>&, MatrixXf&, int);

VectorXf multivariate_gauss(VectorXf&, MatrixXf&, int);

void pi_to_pi(VectorXf&);

float pi_to_pi(float);

float pi_to_pi2(float ang);

void add_feature(Particle&, std::vector<VectorXf>&, MatrixXf&);

void compute_jacobians(Particle&, std::vector<int>&, MatrixXf&, std::vector<VectorXf>&, std::vector<MatrixXf>*, std::vector<MatrixXf>*, std::vector<MatrixXf>*);

void resample_particles(std::vector<Particle>&, int, int);

void stratified_random(unsigned long, std::vector<float>&);

double unifRand();

void stratified_resample(VectorXf, std::vector<int>&, float&);

void cumsum(VectorXf&);

void TransformToGlobal(MatrixXf&, VectorXf&);

MatrixXf randn(int, int);

MatrixXf rand(int, int);


inline auto sqr(float x) {
    return x*x;
}


#endif //RBPF_RBPF_H
