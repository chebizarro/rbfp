// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_EKF_H
#define RBPF_EKF_H

#include <Eigen/Dense>
using namespace Eigen;

namespace ekf {

    void predict(VectorXf &, MatrixXf &, float, float, MatrixXf &, float, float);

    void observe_heading(VectorXf&, MatrixXf&, float, int, float);

    void observe_model(VectorXf&, int, VectorXf&, MatrixXf&);

    void compute_association(VectorXf&, MatrixXf&, VectorXf&, MatrixXf&, int, float&, float&);

    void data_associate(VectorXf&, MatrixXf&, std::vector<VectorXf>&, MatrixXf&,
                            float, float, std::vector<VectorXf>&, std::vector<int>&, std::vector<VectorXf>&);

    void data_associate_known(VectorXf&, std::vector<VectorXf>&, std::vector<int>&,
                                  std::vector<VectorXf>&, std::vector<int>&, std::vector<VectorXf>&, std::vector<int>&);

    void batch_update(VectorXf&, MatrixXf&, std::vector<VectorXf>&, MatrixXf&, std::vector<int>&);

    void update(VectorXf&, MatrixXf&, std::vector<VectorXf>&, MatrixXf&, std::vector<int>&, int);

    void add_one_z(VectorXf&, MatrixXf&, VectorXf&, MatrixXf&);

    void augment(VectorXf&, MatrixXf&, std::vector<VectorXf>&, MatrixXf&);

};


#endif //RBPF_EKF_H
