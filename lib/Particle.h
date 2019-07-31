// rbpf
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#ifndef RBPF_PARTICLE_H
#define RBPF_PARTICLE_H


#include <vector>
#include <Eigen/StdVector>

using namespace Eigen;

class Particle {

public:
    Particle() noexcept;

    Particle(float&, VectorXf&, MatrixXf&, std::vector<VectorXf>&, std::vector<MatrixXf>&, float*);

    float& w();
    VectorXf& xv();
    MatrixXf& Pv();
    std::vector<VectorXf>& xf();
    std::vector<MatrixXf>& Pf();
    float* da();
    void setW(float);
    void setXv(VectorXf&);
    void setPv(MatrixXf&);
    void setXf(std::vector<VectorXf>&);
    void setXfi(unsigned long, VectorXf&);
    void setPf(std::vector<MatrixXf>&);
    void setPfi(unsigned long, MatrixXf&);
    void setDa(float*);

private:
    float _w;
    VectorXf _xv;
    std::vector<VectorXf> _xf;
    std::vector <MatrixXf> _Pf;
    MatrixXf _Pv;
    float* _da;

};


#endif //RBPF_PARTICLE_H
