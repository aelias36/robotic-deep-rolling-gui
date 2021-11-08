#include <iostream>
#include <vector>
#include <math.h>
#include <stdexcept>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

// Overload operators for vector
// vector<double> operator*(const vector<double>& a, const double n)
// {
//     vector<double>b; for (uint8_t i = 0; i < a.size(); i++) { b.push_back(a[i] * n); }return b;
// }
// vector<double> operator/(const vector<double>& a, const double n)
// {
//     vector<double>b; for (uint8_t i = 0; i < a.size(); i++) { b.push_back(a[i] / n); }return b;
// }
// vector<double> operator+(const vector<double>& a, const vector<double>& b)
// {
//     vector<double>c; for (uint8_t i = 0; i < a.size(); i++) { c.push_back(a[i] + b[i]); }return c;
// }
// vector<double> operator-(const vector<double>& a, const vector<double>& b)
// {
//     vector<double>c; for (uint8_t i = 0; i < a.size(); i++) { c.push_back(a[i] - b[i]); }return c;
// }

const Vector3d ex(1,0,0);
const Vector3d ey(0,1,0);
const Vector3d ez(0,0,1);

class SphericalKin {

private:
    vector<Vector3d> P;
    vector<Vector3d> H;
    Matrix4d T;
    Matrix<double, 6, 6> J;
    vector<double> q;

public:
    SphericalKin() {
        double l1,l2,l3,l4,l5,l6;
        l1=0.32; l2=0.78; l3=1.075; l4=0.2; l5=1.1425; l6=0.2;
        
        this->P.push_back(0*ez);
        this->P.push_back(l1*ex+l2*ez);
        this->P.push_back(l3*ez);
        this->P.push_back(l4*ez+l5*ex);
        this->P.push_back(0*ex);
        this->P.push_back(0*ex);
        this->P.push_back(l6*ex);

        this->H.push_back(ez);
        this->H.push_back(ey);
        this->H.push_back(ey);
        this->H.push_back(ex);
        this->H.push_back(ey);
        this->H.push_back(ex);
    }
    void set_q(vector<double> q){this->q=q;}
    void set_T(Matrix4d T){this->T=T;}
    vector<double> get_q(){return q;}
    Matrix4d get_T(){return T;}
    Matrix<double, 6, 6> get_J(){return J;}
    vector<vector<double>> invkin(const Matrix4d T){
        Matrix3d Rt = T.block<3,3>(0,0);
        Vector3d pt = T.block<3,1>(0,3);
        vector<vector<double>> theta;

        // find q1, sub4
        double d = ey.dot(this->P[1]+this->P[2]+this->P[3]);
        Vector3d h1=ey;
        Vector3d h2=pt-Rt*this->P[6];
        vector<double> q1=subprob4(d,-1*this->H[0],h1,h2);
        for(uint8_t i=0; i<q1.size(); i++){
            Matrix3d R01=rot(this->H[0],q1[i]);
            // find q3, sub3
            Vector3d p26_1=R01.transpose()*(pt-this->P[0]-Rt*this->P[6])-this->P[1];
            d=p26_1.norm();
            Vector3d q=this->P[2];
            Vector3d p=-1*this->P[3];
            cout << "debug flag" <<endl;
            vector<double> q3=subprob3(d,p,q,this->H[2]);
            cout << "q3 size" << q3.size() <<endl;
            for(uint8_t j=0; j<q3.size(); j++){
                Matrix3d R23=rot(this->H[2],q3[j]);
                Vector3d p24_2=this->P[2]+R23*this->P[3];
                // find q2, sub1
                double q2=subprob1(this->H[1],p24_2,p26_1);
                Matrix3d R12=rot(this->H[1],q2);
                // find q4, q5, sub2
                Matrix3d R30=R23.transpose()*R12.transpose()*R01.transpose();
                q=this->H[5];
                p=R30*Rt*this->H[5];
                vector<vector<double>> q45=subprob2(p,q,-1*this->H[3],this->H[4]);
                cout << "q45 size" << q45.size() <<endl;
                for(uint8_t m=0; m<q45.size(); m++){
                    Matrix3d R34=rot(this->H[3],q45[m][0]);
                    Matrix3d R45=rot(this->H[4],q45[m][1]);
                    // find q6, sub1
                    Matrix3d R50=R45.transpose()*R34.transpose()*R30;
                    p=this->H[4];
                    q=R50*Rt*this->H[4];
                    double q6=subprob1(this->H[5],p,q);

                    vector<double> sol{q1[i],q2,q3[j],q45[m][0],q45[m][1],q6};
                    theta.push_back(sol);
                }
            }
        }

        return theta;
    }
    Matrix4d fwdkin(const vector<double> q){
        Matrix<double, 6, 6> J=Matrix<double, 6, 6>::Zero();
        Matrix4d T=Matrix4d::Identity();
        Matrix3d Ri; Vector3d p; Matrix<double,6,6>Hi;
        Matrix<double,6,6> phi;

        for(uint8_t i=0; i<this->H.size(); i++){
            
            phi.block<3,3>(0,0)=Matrix3d::Identity();
            phi.block<3,3>(0,3)=Matrix3d::Zero();
            phi.block<3,3>(3,0)=-1*hat(T.block<3,3>(0,0) * this->P[i]);
            phi.block<3,3>(3,3)=Matrix3d::Identity();

            Ri = rot(this->H[i],q[i]);
            T = T*homoT(Ri,this->P[i]);

            Hi = Matrix<double, 6, 6>::Zero();
            Hi.block<3,1>(0,i)=T.block<3,3>(0,0)*this->H[i];
            J = phi*J+Hi;
            
        }
        phi.block<3,3>(0,0)=Matrix3d::Identity();
        phi.block<3,3>(0,3)=Matrix3d::Zero();
        phi.block<3,3>(3,0)=-1*hat(T.block<3,3>(0,0) * this->P[6]);
        phi.block<3,3>(3,3)=Matrix3d::Identity();
        T = T*homoT(Matrix3d::Identity(),this->P[6]);
        J = phi*J;

        this->T = T;
        this->J = J;
        return T;
    }
    vector<double> subprob4(double d, Vector3d k, Vector3d h1, Vector3d h2) {
        vector<double> theta;
        k = k.normalized();
        if (h1.normalized()==k) { theta.push_back(0); theta.push_back(0); return theta; }

        Vector3d kh2c = k.cross(h2);
        Vector3d kkh2c = k.cross(kh2c);
        double h1kkh2c = h1.dot(kkh2c);
        double a = h1.dot(kh2c);
        double b = -1 * h1kkh2c;
        double c = d - h1.dot(h2) + b;
        double phi = atan2(-1 * a, b);
        double costh = c / sqrt(a * a + b * b);

        if (fabs(costh) > 1) {cout<<"larger than 1"<<endl;}
        else {
            theta.push_back(acos(costh) - phi);
            if (fabs(costh) < 1) { theta.push_back(-1 * acos(costh) - phi); }
        }
        return theta;

    }
    // vector<double> subprob4(double d, vector<double>k, vector<double>h1, vector<double>h2) {
    //     vector<double> theta;
    //     k = k / norm(k);
    //     if (h1[0] / k[0] == h1[1] / k[1] && h1[1] / k[1] == h1[2] / k[2]) { theta.push_back(0); theta.push_back(0); return theta; }
    //
    //     vector<double> kh2c = cross(k, h2);
    //     vector<double> kkh2c = cross(k, kh2c);
    //     double h1kkh2c = dot(h1, kkh2c);
    //     double a = dot(h1, kh2c);
    //     double b = -1 * h1kkh2c;
    //     double c = d - dot(h1, h2) + b;
    //     double phi = atan2(-1 * a, b);
    //     double costh = c / sqrt(a * a + b * b);
    //
    //     if (fabs(costh) > 1) {}
    //     else {
    //         theta.push_back(acos(costh) - phi);
    //         if (fabs(costh) < 1) { theta.push_back(-1 * acos(costh) - phi); }
    //     }
    //     return theta;
    // }
    vector<double> subprob3(double d, Vector3d p, Vector3d q, Vector3d k) {
        double dp = sqrt(d * d - pow(k.dot(p-q), 2));
        Vector3d pp = p - k * p.dot(k);
        Vector3d qp = q - k * q.dot(k);
        return subprob3a(dp, pp, qp, k);
    }
    vector<double> subprob3a(double d, Vector3d p, Vector3d q, Vector3d k) {
        double pn = p.norm(), qn = q.norm();
        vector<double> theta;

        if (qn == 0 && pn == d) { theta.push_back(0); theta.push_back(0); return theta; }

        double cosphi = (pn * pn + qn * qn - d * d) / (2 * pn * qn);
        double qq = subprob1(k,p,q);

        if (fabs(cosphi) > 1) {}
        else {
            double q1 = qq + acos(cosphi);
            theta.push_back(q1);
            if (cosphi != 1) { double q2 = qq - acos(cosphi); theta.push_back(q2); }
        }
        return theta;
    }
    // vector<double> subprob3a(double d, vector<double>p, vector<double>q, vector<double>k) {
    //     double pn = norm(p), qn = norm(q);
    //     vector<double> theta;
    //
    //     if (qn == 0 && pn == d) { theta.push_back(0); theta.push_back(0); return theta; }
    //
    //     double cosphi = (pn * pn + qn * qn - d * d) / (2 * pn * qn);
    //     double qq = subprob1(p, q, k);
    //
    //     if (fabs(cosphi) > 1) {}
    //     else {
    //         double q1 = qq + acos(cosphi);
    //         theta.push_back(q1);
    //         if (cosphi != 1) { double q2 = qq - acos(cosphi); theta.push_back(q2); }
    //     }
    //     return theta;
    // }
    vector<vector<double>> subprob2(Vector3d p, Vector3d q, Vector3d k1, Vector3d k2) {
        p = p.normalized();
        q = q.normalized();
        k1 = k1.normalized();
        k2 = k2.normalized();
        double kk = k1.dot(k2);
        double k1p = k1.dot(p);
        double k2q = k2.dot(q);
        double a = (k1p - kk * k2q) / (1 - kk * kk);
        double b = (k2q - kk * k1p) / (1 - kk * kk);
        Vector3d kkc = k1.cross(k2);
        double kkcn = kkc.norm();
        double c2 = (p.dot(p) - a * a - b * b - 2 * a * b * kk) / (kkcn * kkcn);

        vector<vector<double>> theta;
        if (c2 < 0) {}
        else {
            Vector3d v1 = k1 * a + k2 * b, v2;
            if (c2 == 0) {
                vector<double> sol{ subprob1(k1,p,v1),subprob1(k2,q,v1) };
                theta.push_back(sol);
            }
            else { //c2 > 0
                v1 = k1 * a + k2 * b + kkc * sqrt(c2);
                v2 = k1 * a + k2 * b - kkc * sqrt(c2);
                vector<double> sol1{ subprob1(k1,p,v1),subprob1(k2,q,v1) };
                vector<double> sol2{ subprob1(k1,p,v2),subprob1(k2,q,v2) };
                theta.push_back(sol1); theta.push_back(sol2);
            }
        }
        return theta;
    }
    // vector<vector<double>> subprob2(vector<double>p, vector<double>q, vector<double>k1, vector<double>k2) {
    //     p = p / norm(p);
    //     q = q / norm(q);
    //     k1 = k1 / norm(k1);
    //     k2 = k2 / norm(k2);
    //     double kk = dot(k1, k2);
    //     double k1p = dot(k1, p);
    //     double k2q = dot(k2, q);
    //     double a = (k1p - kk * k2q) / (1 - kk * kk);
    //     double b = (k2q - kk * k1p) / (1 - kk * kk);
    //     vector<double> kkc = cross(k1, k2);
    //     double kkcn = norm(kkc);
    //     double c2 = (dot(p, p) - a * a - b * b - 2 * a * b * kk) / (kkcn * kkcn);
    //
    //     vector<vector<double>> theta;
    //     if (c2 < 0) {}
    //     else {
    //         vector<double> v1 = k1 * a + k2 * b, v2;
    //         if (c2 == 0) {
    //             vector<double> sol{ subprob1(k1,p,v1),subprob1(k2,q,v1) };
    //             theta.push_back(sol);
    //         }
    //         else { //c2 > 0
    //             v1 = k1 * a + k2 * b + kkc * sqrt(c2);
    //             v2 = k1 * a + k2 * b - kkc * sqrt(c2);
    //             vector<double> sol1{ subprob1(k1,p,v1),subprob1(k2,q,v1) };
    //             vector<double> sol2{ subprob1(k1,p,v2),subprob1(k2,q,v2) };
    //             theta.push_back(sol1); theta.push_back(sol2);
    //         }
    //     }
    //     return theta;
    // }
    double subprob1(Vector3d k, Vector3d p, Vector3d q) {
        double theta = 0;
        p = p.normalized();
        q = q.normalized();
        theta = subprob0(k, p-k*p.dot(k), q-k*q.dot(k));
        return theta;
    }
    // double subprob1(vector<double>k, vector<double>p, vector<double>q) {
    //     double theta = 0;
    //     p = p / norm(p);
    //     q = q / norm(q);
    //     theta = subprob0(k, p - k * dot(p, k), q - k * dot(q, k));
    //     return theta;
    // }
    double subprob0(Vector3d k, Vector3d p, Vector3d q) {
        double theta = 0;

        if (k.dot(p) != 0 || k.dot(q) != 0) { throw std::invalid_argument("k must be perpendicular to p & q"); }
        p =p.normalized();
        q =q.normalized();

        theta = 2 * atan2((p-q).norm(), (p+q).norm());
        if (k.dot(p.cross(q))< 0) { theta = -1 * theta; }

        return theta;
    }
    // double subprob0(vector<double>k, vector<double>p, vector<double>q) {
    //     double theta = 0;
    //
    //     if (dot(k, p) != 0 || dot(k, q) != 0) { throw std::invalid_argument("k must be perpendicular to p & q"); }
    //     p = p / norm(p);
    //     q = q / norm(q);
    //
    //     theta = 2 * atan2(norm(p - q), norm(p + q));
    //     if (dot(k, cross(p, q)) < 0) { theta = -1 * theta; }
    //
    //     return theta;
    // }
    Matrix4d homoT(Matrix3d R, Vector3d p){
        Matrix4d T = Matrix4d::Identity();
        T.block<3,3>(0,0)=R;
        T.block<3,1>(0,3)=p;
        return T;
    }
    Matrix3d rot(Vector3d k, double theta) {
        Matrix3d R;
        k=k.normalized();
        R = Matrix3d::Identity()+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
        return R;
    }
    Matrix3d hat(Vector3d k) {
        Matrix3d km;
        km << 0,-1*k(2),k(1),
            k(2),0,-1*k(0),
            -1*k(1),k(0),0;
        return km;
    }
    // double dot(vector<double>const& a, vector<double>const& b) {
    //     double sol = 0;
    //     for (uint8_t i = 0; i < a.size(); i++) { sol += a[i] * b[i]; }
    //     return sol;
    // }
    // vector<double> cross(vector<double>const& a, vector<double>const& b) {
    //     vector<double> c;
    //     c.push_back(a[1] * b[2] - a[2] * b[1]);
    //     c.push_back(a[2] * b[0] - a[0] * b[2]);
    //     c.push_back(a[0] * b[1] - a[1] * b[0]);
    //     return c;
    // }
    // double norm(vector<double>const& a) {
    //     double sol = 0;
    //     for (uint8_t i = 0; i < a.size(); i++) { sol += a[i] * a[i]; }
    //     return sqrt(sol);
    // }
    double testFunc() {
        cout << "This is test function" << endl;

        Vector3d v(1, 2, 3), w(4,5,6);
        double sol;
        sol = 2+3;

        return v.dot(v);
    }
};

extern "C" {
    SphericalKin* SphericalKin_new() { return new SphericalKin(); }
    double* fwdkin(SphericalKin* kin, double* q) { 
        vector<double> q_vec;
        for(uint8_t i=0; i<6; i++){q_vec.push_back(q[i]);}
        Matrix4d T = kin->fwdkin(q_vec);
        double flat_T[16];
        for(uint8_t i=0; i<16; i++){flat_T[i]=T(i);}
        return flat_T; }
    double* invkin(SphericalKin* kin, double* flat_T) { 
        Matrix4d T;
        for(uint8_t i=0; i<16; i++){T(i)=flat_T[i];}
        vector<vector<double>> q=kin->invkin(T);
        double *q_sol;
        q_sol = new double[q.size()*6];
        for(uint8_t i=0; i<q.size(); i++){
            for(uint8_t j=0; j<6; j++){q_sol[i*6+j]=q[i][j];}
        }
        return q_sol;}
    // double get_q(SphericalKin* kin) { return kin->get_q(); }
    // double get_T(SphericalKin* kin) { return kin->get_T(); }
    // double get_J(SphericalKin* kin) { return kin->get_J(); }
    // double set_q(SphericalKin* kin) { return kin->get_q(); }
    // __declspec(dllexport) double get_q(SphericalKin* kin) { return kin->get_q(); }
}

int main()
{
    // vector<vector<double>> T;
    // vector<double> Trol{1,2,3,4};
    // T.push_back(Trol);
    // cout << T[1:3][2] << endl;
    // Vector3d a(1,0,0);
    // Matrix3d b; b<<1,2,3,4,5,6,7,8,9;
    // cout << a.transpose()*b << endl;

    SphericalKin* robot = new SphericalKin();

    vector<double> q{M_PI/2,M_PI/8,M_PI/10,M_PI/10,M_PI/8,M_PI/10};
    cout << "Original" << endl;
    for(int8_t i=0; i<q.size(); i++){ 
        cout << q[i] << ", ";
    }
    cout << endl;
    
    Matrix4d T=robot->fwdkin(q);
    cout << "FwdKin" << endl;
    cout << T << endl;
    cout<< T(8) <<endl;

    cout << "=================" << endl;
    vector<vector<double>> q_af = robot->invkin(T);
    cout << "InvKin" << endl;
    for(uint8_t i=0; i<q_af.size(); i++){
        for(uint8_t j=0; j<q_af[i].size(); j++){cout << q_af[i][j] << ", ";}
        cout << endl;
    }

    return 0;
}