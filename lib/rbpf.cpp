// rbpf.world
// Copyright (c) 2019 Chris Daley <chebizarro@gmail.com>
// This code is licensed under MIT license (see LICENSE.txt for details)

#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "rbpf.h"

using namespace Eigen;


void add_control_noise(float V, float G, MatrixXf &Q, int addnoise, float *VnGn) {
    if (addnoise == 1) {
        VectorXf A(2);
        A(0) = V;
        A(1) = G;
        VectorXf C(2);
        C = multivariate_gauss(A, Q, 1);
        VnGn[0] = C(0);
        VnGn[1] = C(1);
    }
}

void predict_true(VectorXf &xv, float V, float G, float WB, float dt) {
    xv(0) = xv(0) + V * dt * cos(G + xv(2));
    xv(1) = xv(1) + V * dt * sin(G + xv(2));
    xv(2) = pi_to_pi(xv(2) + V * dt * sin(G) / WB);
}

void compute_steering(VectorXf &x, MatrixXf &wp, int &iwp, float minD,
                      float &G, float rateG, float maxG, float dt) {
    /*
        % INPUTS:
        %   x - true position
        %   wp - waypoints
        %   iwp - index to current waypoint
        %   minD - minimum distance to current waypoint before switching to next
        %   rateG - max steering rate (rad/s)
        %   maxG - max steering angle (rad)
        %   dt - timestep
        % Output:
        %   G - current steering angle
    */

    //determine if current waypoint reached
    Vector2d cwp;
    cwp[0] = wp(0, iwp);    //-1 since indexed from 0
    cwp[1] = wp(1, iwp);

    auto d2 = pow((cwp[0] - x[0]), 2) + pow((cwp[1] - x[1]), 2);

    if (d2 < minD * minD) {
        iwp++; //switch to next
        if (iwp >= wp.cols()) {
            iwp = -1;
            return;
        }

        cwp[0] = wp(0, iwp); //-1 since indexed from 0
        cwp[1] = wp(1, iwp);
    }

    //compute change in G to point towards current waypoint
    auto deltaG = atan2(cwp[1] - x[1], cwp[0] - x[0]) - x[2] - G;
    deltaG = pi_to_pi(deltaG);

    //limit rate
    auto maxDelta = rateG * dt;
    if (abs(deltaG) > maxDelta) {
        auto sign = (deltaG > 0) ? 1 : ((deltaG < 0) ? -1 : 0);
        deltaG = sign * maxDelta;
    }

    //limit angle
    G = G + deltaG;
    if (abs(G) > maxG) {
        auto sign2 = (G > 0) ? 1 : ((G < 0) ? -1 : 0);
        G = sign2 * maxG;
    }
}


//z is range and bearing of visible landmarks
// find associations (zf) and new features (zn)
void data_associate_known(std::vector<VectorXf> &z, std::vector<int> &idz, VectorXf &table, int Nf,
                          std::vector<VectorXf> &zf, std::vector<int> &idf, std::vector<VectorXf> &zn) {
    idf.clear();
    std::vector<int> idn;

    for (unsigned long i = 0; i < idz.size(); i++) {
        auto ii = idz[i];
        VectorXf z_i;
        if (table(ii) == -1) { //new feature
            z_i = z[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        } else {
            z_i = z[i];
            zf.push_back(z_i);
            idf.push_back(table(ii));
        }
    }

    assert(idn.size() == zn.size());
    for (unsigned long i = 0; i < idn.size(); i++) {
        table(idn[i]) = Nf + i;
    }
}

// z is the list of measurements conditioned on the particle.
void feature_update(Particle &particle, std::vector<VectorXf> &z, std::vector<int> &idf, MatrixXf &R) {
    //Having selected a new pose from the proposal distribution,
    //  this pose is assumed perfect and each feature update maybe
    //  computed independently and without pose uncertainty
    std::vector<VectorXf> xf;    //updated EKF means
    std::vector<MatrixXf> Pf;    //updated EKF covariances

    for (auto i : idf) {
        xf.push_back(particle.xf()[i]); //means
        Pf.push_back(particle.Pf()[i]); //covariances
    }

    std::vector<VectorXf> zp;
    std::vector<MatrixXf> Hv;
    std::vector<MatrixXf> Hf;
    std::vector<MatrixXf> Sf;

    compute_jacobians(particle, idf, R, zp, &Hv, &Hf, &Sf);

    std::vector<VectorXf> v; //difference btw two measurements (used to update mean)
    for (unsigned long i = 0; i < z.size(); i++) {
        VectorXf measure_diff = z[i] - zp[i];
        measure_diff[1] = pi_to_pi(measure_diff[1]);
        v.push_back(measure_diff);
    }

    VectorXf vi;
    MatrixXf Hfi;
    MatrixXf Pfi;
    VectorXf xfi;

    for (unsigned long i = 0; i < idf.size(); i++) {
        vi = v[i];
        Hfi = Hf[i];
        Pfi = Pf[i];
        xfi = xf[i];
        KF_cholesky_update(xfi, Pfi, vi, R, Hfi);
        xf[i] = xfi;
        Pf[i] = Pfi;
    }

    for (unsigned long i = 0; i < idf.size(); i++) {
        particle.setXfi(idf[i], xf[i]);
        particle.setPfi(idf[i], Pf[i]);
    }
}

std::vector<VectorXf> get_observations(VectorXf &x, MatrixXf lm, std::vector<int> &idf, float rmax) {
    get_visible_landmarks(x, lm, idf, rmax);
    return compute_range_bearing(x, lm);
}

void get_visible_landmarks(VectorXf &x, MatrixXf &lm, std::vector<int> &idf, float rmax) {
    //select set of landmarks that are visible within vehicle's
    //semi-circular field of view
    std::vector<float> dx;
    std::vector<float> dy;

    for (int c = 0; c < lm.cols(); c++) {
        dx.push_back(lm(0, c) - x(0));
        dy.push_back(lm(1, c) - x(1));
    }

    auto phi = x(2);

    //distant points are eliminated
    auto ii = find2(dx, dy, phi, rmax);

    MatrixXf lm_new(lm.rows(), ii.size());
    unsigned j, k;
    for (j = 0; j < lm.rows(); j++) {
        for (k = 0; k < ii.size(); k++) {
            lm_new(j, k) = lm(j, ii[k]);
        }
    }
    lm = MatrixXf(lm_new);

    std::vector<int> idf_backup(idf);
    idf.clear();
    for (auto i : ii) {
        idf.push_back(idf_backup[i]);
    }
}

std::vector<VectorXf> compute_range_bearing(VectorXf &x, MatrixXf &lm) {
    std::vector<float> dx;
    std::vector<float> dy;

    for (auto c = 0; c < lm.cols(); c++) {
        dx.push_back(lm(0, c) - x(0));
        dy.push_back(lm(1, c) - x(1));
    }

    assert(dx.size() == lm.cols());
    assert(dy.size() == lm.cols());

    auto phi = x(2);
    std::vector<VectorXf> z;

    for (auto i = 0; i < lm.cols(); i++) {
        VectorXf zvec(2);
        zvec << sqrt(pow(dx[i], 2) + pow(dy[i], 2)), atan2(dy[i], dx[i]) - phi;
        z.push_back(zvec);
    }

    return z;
}

std::vector<int> find2(std::vector<float> &dx, std::vector<float> &dy, float phi, float rmax) {
    std::vector<int> index;
    //incremental tests for bounding semi-circle
    for (unsigned long j = 0; j < dx.size(); j++) {
        if ((abs(dx[j]) < rmax) && (abs(dy[j]) < rmax)
            && ((dx[j] * cos(phi) + dy[j] * sin(phi)) > 0.0)
            && (((float) pow(dx[j], 2) + (float) pow(dy[j], 2)) < (float) pow(rmax, 2))) {
            index.push_back(j);
        }
    }
    return index;
}

void KF_cholesky_update(VectorXf &x, MatrixXf &P, VectorXf &v, MatrixXf &R, MatrixXf &H) {
    MatrixXf PHt = P * H.transpose();
    MatrixXf S = H * PHt + R;

    // FIXME: why use conjugate()?
    S = (S + S.transpose()) * 0.5; //make symmetric
    MatrixXf SChol = S.llt().matrixU();
    //SChol.transpose();
    //SChol.conjugate();


    MatrixXf SCholInv = SChol.inverse(); //tri matrix
    MatrixXf W1 = PHt * SCholInv;
    MatrixXf W = W1 * SCholInv.transpose();

    x = x + W * v;
    P = P - W1 * W1.transpose();
}


void KF_joseph_update(VectorXf &x, MatrixXf &P, float v, float R, MatrixXf &H) {
    VectorXf PHt = P * H.transpose();
    MatrixXf S = H * PHt;
    MatrixXf _t = S;
    _t.setOnes();
    _t = _t * R;
    S = S + _t;
    MatrixXf Si = S.inverse();
    //Si = make_symmetric(Si);
    make_symmetric(Si);
    MatrixXf PSD_check = Si.llt().matrixU(); //chol of scalar is sqrt
    //PSD_check.transpose();
    //PSD_check.conjugate();

    VectorXf W = PHt * Si;
    x = x + W * v;

    //Joseph-form covariance update
    MatrixXf eye(P.rows(), P.cols());
    eye.setIdentity();
    MatrixXf C = eye - W * H;
    P = C * P * C.transpose() + W * R * W.transpose();

    auto eps = 2.2204 * pow(10.0, -16); //numerical safety
    P = P + eye * eps;

    PSD_check = P.llt().matrixL();
    PSD_check.transpose();
    PSD_check.conjugate(); //for upper tri
}

MatrixXf make_symmetric(MatrixXf &P) {
    return (P + P.transpose()) * 0.5;
}


MatrixXf line_plot_conversion(MatrixXf &lnes) {
    auto len = lnes.cols() * 3 - 1;
    MatrixXf p(2, len);

    for (auto j = 0; j < len; j += 3) {
        int k = floor((j + 1) / 3); //reverse of len calculation
        p(0, j) = lnes(0, k);
        p(1, j) = lnes(1, k);
        p(0, j + 1) = lnes(2, k);
        p(1, j + 1) = lnes(3, k);
        if (j + 2 < len) {
            p(0, j + 2) = 0;
            p(1, j + 2) = 0;
        }
    }
    return p;
}

//rb is measurements
//xv is robot pose
MatrixXf make_laser_lines(std::vector<VectorXf> &rb, VectorXf &xv) {
    if (rb.empty()) {
        return MatrixXf(0, 0);
    }

    auto len = rb.size();
    MatrixXf lnes(4, len);

    MatrixXf globalMat(2, rb.size());
    int j;
    for (j = 0; j < globalMat.cols(); j++) {
        globalMat(0, j) = rb[j][0] * cos(rb[j][1]);
        globalMat(1, j) = rb[j][0] * sin(rb[j][1]);
    }

    TransformToGlobal(globalMat, xv);

    for (auto c = 0; c < lnes.cols(); c++) {
        lnes(0, c) = xv(0);
        lnes(1, c) = xv(1);
        lnes(2, c) = globalMat(0, c);
        lnes(3, c) = globalMat(1, c);
    }

    //return line_plot_conversion(lnes);
    return lnes;
}


void make_covariance_ellipse(MatrixXf &x, MatrixXf &P, MatrixXf &lines) {

    MatrixXf r = P.sqrt();
    auto N = 16;
    auto p_inc = 2.0 * M_PI / N;
    auto s = 2.0;

    lines.setZero(2, N + 1);

    for (auto i = 0; i <= N; i++) {
        auto p = i * p_inc;
        auto cp = cos(p);
        auto sp = sin(p);

        lines(0, i) = x(0) + s * (r(0, 0) * cp + r(0, 1) * sp);
        lines(1, i) = x(1) + s * (r(1, 0) * cp + r(1, 1) * sp);
    }
}

//http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
MatrixXf randn(int m, int n) {
    // use formula 30.3 of Statistical Distributions (3rd ed)
    // Merran Evans, Nicholas Hastings, and Brian Peacock
    auto urows = m * n + 1;
    VectorXf u(urows);

    //u is a random matrix
#if 1
    for (auto r = 0; r < urows; r++) {
        // FIXME: better way?
        u(r) = std::rand() * 1.0 / RAND_MAX;
    }
#else
    u = ( (VectorXf::Random(urows).array() + 1.0)/2.0 ).matrix();
#endif


    MatrixXf x(m, n);

    float amp, angle;

    auto k = 0;
    for (auto i = 0; i < m; i++) {
        for (auto j = 0; j < n; j++) {
            if (k % 2 == 0) {
                auto square = -2. * std::log(u(k));
                if (square < 0.)
                    square = 0.;
                amp = sqrt(square);
                angle = 2. * M_PI * u(k + 1);
                x(i, j) = amp * std::sin(angle);
            } else
                x(i, j) = amp * std::cos(angle);

            k++;
        }
    }

    return x;
}

MatrixXf rand(int m, int n) {
    MatrixXf x(m, n);
    auto rand_max = float(RAND_MAX);

    for (auto i = 0; i < m; i++) {
        for (auto j = 0; j < n; j++)
            x(i, j) = float(std::rand()) / rand_max;
    }
    return x;
}

//add random measurement noise. We assume R is diagnoal matrix
void add_observation_noise(std::vector<VectorXf> &z, MatrixXf &R, int addnoise) {
    if (addnoise == 1) {
        auto len = z.size();
        if (len > 0) {
            MatrixXf randM1 = randn(1, len);
            MatrixXf randM2 = randn(1, len);

            for (unsigned long c = 0; c < len; c++) {
                z[c][0] = z[c][0] + randM1(0, c) * sqrt(R(0, 0));
                z[c][1] = z[c][1] + randM2(0, c) * sqrt(R(1, 1));
            }
        }
    }
}


VectorXf multivariate_gauss(VectorXf &x, MatrixXf &P, int n) {
    auto len = x.size();

    //choleksy decomposition
    MatrixXf S = P.llt().matrixL();
    auto X = randn(len, n);

    return S * X + x;
}


void pi_to_pi(VectorXf &angle) {

    for (auto i = 0; i < angle.size(); i++) {
        if ((angle[i] < (-2 * M_PI)) || (angle[i] > (2 * M_PI))) {
            auto n = floor(angle[i] / (2 * M_PI));
            angle[i] = angle[i] - n * (2 * M_PI);
        }

        if (angle[i] > M_PI)
            angle[i] = angle[i] - (2 * M_PI);

        if (angle[i] < -M_PI)
            angle[i] = angle[i] + (2 * M_PI);
    }
}

float pi_to_pi(float ang) {

    if ((ang < -2 * M_PI) || (ang > 2 * M_PI)) {
        auto n = floor(ang / (2 * M_PI));
        ang = ang - n * (2 * M_PI);
    }

    if (ang > M_PI)
        ang = ang - (2 * M_PI);

    if (ang < -M_PI)
        ang = ang + (2 * M_PI);

    return ang;
}

float pi_to_pi2(float ang) {
    if (ang > M_PI)
        ang = ang - (2 * M_PI);

    if (ang < -M_PI)
        ang = ang + (2 * M_PI);

    return ang;
}


//
// add new features
//
void add_feature(Particle &particle, std::vector<VectorXf> &z, MatrixXf &R) {
    auto lenz = z.size();
    std::vector<VectorXf> xf;
    std::vector<MatrixXf> Pf;
    auto xv = particle.xv();

    MatrixXf Gz(2, 2);

    for (auto i = 0; i < lenz; i++) {
        auto r = z[i][0];
        auto b = z[i][1];
        auto s = sin(xv(2) + b);
        auto c = cos(xv(2) + b);

        VectorXf measurement(2);
        measurement(0) = xv(0) + r * c;
        measurement(1) = xv(1) + r * s;
        xf.push_back(measurement);
        Gz << c, -r * s, s, r * c;

        Pf.emplace_back(Gz * R * Gz.transpose());
    }

    auto lenx = particle.xf().size();

    for (auto i = lenx; i < lenx + lenz; i++) {
        particle.setXfi(i, xf[(i - lenx)]);
        particle.setPfi(i, Pf[(i - lenx)]);
    }
}

void compute_jacobians(
        Particle &particle,
        std::vector<int> &idf,
        MatrixXf &R,
        std::vector<VectorXf> &zp,   // measurement (range, bearing)
        std::vector<MatrixXf> *Hv,   // jacobians of function h (deriv of h wrt pose)
        std::vector<MatrixXf> *Hf,   // jacobians of function h (deriv of h wrt mean)
        std::vector<MatrixXf> *Sf)   // measurement covariance
{
    auto xv = particle.xv();

    std::vector<VectorXf> xf;
    std::vector<MatrixXf> Pf;

    for (auto i : idf) {
        xf.push_back(particle.xf()[i]);
        Pf.push_back((particle.Pf())[i]); //particle.Pf is a array of matrices
    }

    MatrixXf HvMat(2, 3);
    MatrixXf HfMat(2, 2);

    for (auto i = 0; i < idf.size(); i++) {
        auto dx = xf[i](0) - xv(0);
        auto dy = xf[i](1) - xv(1);
        auto d2 = pow(dx, 2) + pow(dy, 2);
        auto d = sqrt(d2);

        VectorXf zp_vec(2);

        //predicted observation
        zp_vec[0] = d;
        zp_vec[1] = pi_to_pi(atan2(dy, dx) - xv(2));
        zp.push_back(zp_vec);

        //Jacobian wrt vehicle states
        HvMat << -dx / d, -dy / d, 0,
                dy / d2, -dx / d2, -1;

        //Jacobian wrt feature states
        HfMat << dx / d, dy / d,
                -dy / d2, dx / d2;

        Hv->push_back(HvMat);
        Hf->push_back(HfMat);

        //innovation covariance of feature observation given the vehicle'
        MatrixXf SfMat = HfMat * Pf[i] * HfMat.transpose() + R;
        Sf->push_back(SfMat);
    }
}


void resample_particles(std::vector<Particle> &particles, int Nmin, int doresample) {
    auto N = particles.size();
    VectorXf w(N);

    for (auto i = 0; i < N; i++) {
        w(i) = particles[i].w();
    }

    auto ws = w.sum();
    for (auto i = 0; i < N; i++) {
        particles[i].setW(w(i) / ws);
    }

    auto Neff = .0f;
    std::vector<int> keep;

    stratified_resample(w, keep, Neff);

    std::vector<Particle> old_particles(particles);
    particles.resize(keep.size());

    if ((Neff < Nmin) && (doresample == 1)) {
        for (auto i = 0; i < keep.size(); i++) {
            particles[i] = old_particles[keep[i]];
        }

        for (auto i = 0; i < N; i++) {
            auto new_w = 1.0f / (float) N;
            particles[i].setW(new_w);
        }
    }
}

void stratified_random(unsigned long N, std::vector<float> &di) {

    auto k = 1.0 / (float) N;

    //deterministic intervals
    auto temp = k / 2;
    while (temp < (1 - k / 2)) {
        di.push_back(temp);
        temp = temp + k;
    }

    // FIXME: when set NPARTICLES = 30, this whill failed
    assert(di.size() == N);

    //dither within interval
    std::vector<float>::iterator diter;
    for (diter = di.begin(); diter != di.end(); diter++) {
        *diter = (*diter) + unifRand() * k - (k / 2);
    }
}

//
// Generate a random number between 0 and 1
// return a uniform number in [0,1].
double unifRand() {
    return rand() / double(RAND_MAX);
}

// FIXME: input w will be modified?
void stratified_resample(VectorXf w, std::vector<int> &keep, float &Neff) {
    VectorXf wsqrd(w.size());
    auto w_sum = w.sum();

    for (auto i = 0; i < w.size(); i++) {
        // FIXME: matlab version is: w = w / sum(w)
        w(i) = w(i) / w_sum;
        wsqrd(i) = pow(w(i), 2);
    }
    Neff = 1 / wsqrd.sum();

    auto len = w.size();
    keep.resize(len);
    for (auto i = 0; i < len; i++) {
        keep[i] = -1;
    }

    std::vector<float> select;
    stratified_random(len, select);
    cumsum(w);

    auto ctr = 0;
    for (auto i = 0; i < len; i++) {
        while ((ctr < len) && (select[ctr] < w(i))) {
            keep[ctr] = i;
            ctr++;
        }
    }
}

//
//returns a cumulative sum array
//
void cumsum(VectorXf &w) {
    VectorXf csumVec(w.size());
    for (auto i = 0; i < w.size(); i++) {
        auto sum = .0f;
        for (auto j = 0; j <= i; j++) {
            sum += w(j);
        }
        csumVec(i) = sum;
    }

    w = VectorXf(csumVec); //copy constructor. Double check
}

void TransformToGlobal(MatrixXf &p, VectorXf &b) {
    //rotate
    MatrixXf rot(2, 2);
    rot << cos(b(2)), -sin(b(2)), sin(b(2)), cos(b(2));

    MatrixXf p_resized(p);
    p_resized.conservativeResize(2, p_resized.cols());
    p_resized = rot * p_resized;

    //translate
    for (auto c = 0; c < p_resized.cols(); c++) {
        p(0, c) = p_resized(0, c) + b(0);
        p(1, c) = p_resized(1, c) + b(1);
    }

    //if p is a pose and not a point
    if (p.rows() == 3) {
        for (int k = 0; k < p_resized.cols(); k++) {
            auto input = p(2, k) + b(2);
            p(2, k) = pi_to_pi(input);
        }
    }
}

