// rbpf
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

#include "ekf.h"
#include "rbpf.h"

using namespace ekf;
using namespace Eigen;

void ekf::predict(VectorXf &x, MatrixXf &P, float V, float G, MatrixXf &Q, float WB, float dt) {
    MatrixXf Gv(3, 3), Gu(3, 2);         // Jacobians

    auto s = sin(G + x(2));
    auto c = cos(G + x(2));
    auto vts = V * dt * s;
    auto vtc = V * dt * c;

    Gv << 1, 0, -vts,
            0, 1, vtc,
            0, 0, 1;
    Gu << dt * c, -vts,
            dt * s, vtc,
            dt * sin(G) / WB, V * dt * cos(G) / WB;

    // predict covariance
    //      P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
    P.block(0, 0, 3, 3) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() + Gu * Q * Gu.transpose();

    auto m = P.rows();
    if (m > 3) {
        P.block(0, 3, 3, m - 3) = Gv * P.block(0, 3, 3, m - 3);
        P.block(3, 0, m - 3, 3) = P.block(0, 3, 3, m - 3).transpose();
    }

    // predict state
    x(0) = x(0) + vtc;
    x(1) = x(1) + vts;
    x(2) = pi_to_pi(x(2) + V * dt * sin(G) / WB);
}

void ekf::observe_heading(VectorXf &x, MatrixXf &P, float phi, int useheading, float sigmaPhi) {
    if (useheading == 0) return;

    MatrixXf H = MatrixXf(1, x.size());

    H.setZero(1, x.size());
    H(2) = 1;
    auto v = pi_to_pi(phi - x(2));

    KF_joseph_update(x, P, v, sigmaPhi * sigmaPhi, H);
}

void ekf::observe_model(VectorXf &x, int idf,
                       VectorXf &z, MatrixXf &H) {
    auto Nxv = 3;
    H.setZero(2, x.size());
    z.setZero(2);

    // position of xf in state
    auto fpos = Nxv + idf * 2;

    auto dx = x(fpos) - x(0);
    auto dy = x(fpos + 1) - x(1);
    auto d2 = sqr(dx) + sqr(dy);
    auto d = sqrtf(d2);
    auto xd = dx / d;
    auto yd = dy / d;
    auto xd2 = dx / d2;
    auto yd2 = dy / d2;

    z(0) = d;
    z(1) = atan2(dy, dx) - x(2);

    H(0, 0) = -xd;
    H(0, 1) = -yd;
    H(0, 2) = 0;
    H(1, 0) = yd2;
    H(1, 1) = -xd2;
    H(1, 2) = -1;

    H(0, fpos) = xd;
    H(0, fpos + 1) = yd;
    H(1, fpos) = -yd2;
    H(1, fpos + 1) = xd2;
}

void ekf::compute_association(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &R, int idf,
                             float &nis, float &nd) {
    VectorXf zp;
    MatrixXf H;
    VectorXf v(2);
    MatrixXf S;

    observe_model(x, idf, zp, H);

    v = z - zp;
    v(1) = pi_to_pi(v(1));

    S = H * P * H.transpose() + R;

    nis = v.transpose() * S.inverse() * v;
    nd = nis + log(S.determinant());
}

void ekf::data_associate(VectorXf &x, MatrixXf &P, std::vector <VectorXf> &z, MatrixXf &R,
                        float gate1, float gate2,
                        std::vector <VectorXf> &zf, std::vector<int> &idf, std::vector <VectorXf> &zn) {
    zf.clear();
    zn.clear();
    idf.clear();

    auto Nxv = 3u;                        // number of vehicle pose states
    auto Nf = (x.size() - Nxv) / 2;       // number of features already in map

    // linear search for nearest-neighbour, no clever tricks (like a quick
    // bounding-box threshold to remove distant features; or, better yet,
    // a balanced k-d tree lookup). TODO: implement clever tricks.
    for (auto i : z) {
        long jbest = -1;
        auto nbest = 1e60;
        auto outer = 1e60;

        float nis, nd;

        for (auto j = 0; j < Nf; j++) {
            compute_association(x, P, i, R, j,
                                    nis, nd);

            if (nis < gate1 && nd < nbest) {
                nbest = nd;
                jbest = j;
            } else if (nis < outer) {
                outer = nis;
            }
        }

        if (jbest > -1) {
            // add nearest-neighbour to association list
            zf.push_back(i);
            idf.push_back(jbest);
        } else if (outer > gate2) {
            // z too far to associate, but far enough to be a new feature
            zn.push_back(i);
        }
    }
}

// z is range and bearing of visible landmarks
// find associations (zf) and new features (zn)
void ekf::data_associate_known(VectorXf &x, std::vector <VectorXf> &z, std::vector<int> &idz,
                              std::vector <VectorXf> &zf, std::vector<int> &idf, std::vector <VectorXf> &zn, std::vector<int> &table) {
    std::vector<int> idn;

    zf.clear();
    zn.clear();
    idf.clear();
    idn.clear();

    for (auto i = 0; i < idz.size(); i++) {
        auto ii = idz[i];
        VectorXf z_i;

        if (table[ii] == -1) {
            // new feature
            z_i = z[i];
            zn.push_back(z_i);
            idn.push_back(ii);
        } else {
            // exist feature
            z_i = z[i];
            zf.push_back(z_i);
            idf.push_back(table[ii]);
        }
    }

    // add new feature IDs to lookup table
    auto Nxv = 3;                        // number of vehicle pose states
    auto Nf = (x.size() - Nxv) / 2;       // number of features already in map

    // add new feature positions to lookup table
    //      table[idn]= Nf + (1:size(zn,2));
    for (auto i = 0; i < idn.size(); i++) {
        table[idn[i]] = Nf + i;
    }
}


void ekf::batch_update(VectorXf &x, MatrixXf &P, std::vector <VectorXf> &zf, MatrixXf &R, std::vector<int> &idf) {

    auto lenz = zf.size();
    auto lenx = x.size();

    MatrixXf H, RR;
    VectorXf v;
    VectorXf zp;
    MatrixXf H_;

    H.setZero(2 * lenz, lenx);
    v.setZero(2 * lenz);
    RR.setZero(2 * lenz, 2 * lenz);
    zp.setZero(2);

    for (auto i = 0; i < lenz; i++) {
        observe_model(x, idf[i], zp, H_);
        H.block(i * 2, 0, 2, lenx) = H_;

        v(2 * i) = zf[i](0) - zp(0);
        v(2 * i + 1) = pi_to_pi(zf[i](1) - zp(1));

        RR.block(i * 2, i * 2, 2, 2) = R;
    }

    KF_cholesky_update(x, P, v, RR, H);
}

void ekf::update(VectorXf &x, MatrixXf &P, std::vector <VectorXf> &zf, MatrixXf &R, std::vector<int> &idf, int batch) {
    if (batch == 1)
        batch_update(x, P, zf, R, idf);
}


void ekf::add_one_z(VectorXf &x, MatrixXf &P, VectorXf &z, MatrixXf &Re) {

    auto len = x.size();
    auto r = z(0);
    auto b = z(1);
    auto s = sin(x(2) + b);
    auto c = cos(x(2) + b);

    // augment x
    // x = [x; x(1)+r*c; x(2)+r*s];
    x.conservativeResize(len + 2);
    x(len) = x(0) + r * c;
    x(len + 1) = x(1) + r * s;

    // Jacobians
    MatrixXf Gv, Gz;

    Gv.setZero(2, 3);
    Gz.setZero(2, 2);

    Gv << 1, 0, -r * s,
            0, 1, r * c;
    Gz << c, -r * s,
            s, r * c;

    // augment P
    P.conservativeResize(len + 2, len + 2);

    // feature cov
    // P(rng,rng)= Gv*P(1:3,1:3)*Gv' + Gz*R*Gz';
    P.block(len, len, 2, 2) = Gv * P.block(0, 0, 3, 3) * Gv.transpose() +
                              Gz * Re * Gz.transpose();

    // vehicle to feature xcorr
    //      P(rng,1:3)= Gv*P(1:3,1:3);
    //      P(1:3,rng)= P(rng,1:3)';
    P.block(len, 0, 2, 3) = Gv * P.block(0, 0, 3, 3);
    P.block(0, len, 3, 2) = P.block(len, 0, 2, 3).transpose();

    if (len > 3) {
        // map to feature xcoor
        //   P(rng,rnm)= Gv*P(1:3,rnm);
        P.block(len, 3, 2, len - 3) = Gv * P.block(0, 3, 3, len - 3);

        //   P(rnm,rng)= P(rng,rnm)';
        P.block(3, len, len - 3, 2) = P.block(len, 3, 2, len - 3).transpose();
    }
}

void ekf::augment(VectorXf& x, MatrixXf& P, std::vector<VectorXf>& zn, MatrixXf& Re) {
    for (auto i = 0; i < zn.size(); i++)
        add_one_z(x, P, zn[i], Re);
}