// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include "Particle.h"

using namespace Eigen;

Particle::Particle() noexcept {
    _w = 1.0;
    _xv = VectorXf(3);
    _xv.setZero(3);
    _Pv = MatrixXf(3, 3);
    _Pv.setZero(3, 3);
    _da = nullptr;
}

Particle::Particle(float& w, VectorXf& xv, MatrixXf& Pv, std::vector <VectorXf>& xf, std::vector <MatrixXf>& Pf, float* da) {
    _w = w;
    _xv = xv;
    _Pv = Pv;
    _xf = xf;
    _Pf = Pf;
    _da = da;
}


//getters
float& Particle::w() {
    return _w;
}

VectorXf& Particle::xv() {
    return _xv;
}

MatrixXf& Particle::Pv() {
    return _Pv;
}

std::vector<VectorXf>& Particle::xf() {
    return _xf;
}

std::vector<MatrixXf>& Particle::Pf() {
    return _Pf;
}

float* Particle::da() {
    return _da;
}

//setters
void Particle::setW(float w) {
    _w = w;
}

void Particle::setXv(VectorXf& xv) {
    _xv = xv;
}

void Particle::setPv(MatrixXf& Pv) {
    _Pv = Pv;
}

void Particle::setXf(std::vector<VectorXf>& xf) {
    _xf = xf;
}

void Particle::setXfi(unsigned long i, VectorXf& vec) {
    if (i >= _xf.size()) {
        _xf.resize(i + 1);
    }
    _xf[i] = vec;
}

void Particle::setPf(std::vector<MatrixXf>& Pf) {
    _Pf = Pf;
}

void Particle::setPfi(unsigned long i, MatrixXf& m) {
    if (i >= _Pf.size()) {
        _Pf.resize(i + 1);
    }
    _Pf[i] = m;
}

void Particle::setDa(float *da) {
    _da = da;
}
