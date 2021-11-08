#include <iostream>
#include <vector>
#include <math.h>
#include <stdexcept>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// Overload operators for vector
vector<double> operator*(const vector<double>& a, const double n)
{
    vector<double>b; for (uint8_t i = 0; i < a.size(); i++) { b.push_back(a[i] * n); }return b;
}
vector<double> operator/(const vector<double>& a, const double n)
{
    vector<double>b; for (uint8_t i = 0; i < a.size(); i++) { b.push_back(a[i] / n); }return b;
}
vector<double> operator+(const vector<double>& a, const vector<double>& b)
{
    vector<double>c; for (uint8_t i = 0; i < a.size(); i++) { c.push_back(a[i] + b[i]); }return c;
}
vector<double> operator-(const vector<double>& a, const vector<double>& b)
{
    vector<double>c; for (uint8_t i = 0; i < a.size(); i++) { c.push_back(a[i] - b[i]); }return c;
}

class SphericalKin {

private:
    const vector<double> ex{ 1,0,0 }, ey{ 0,1,0 }, ez{ 0,0,1 };
    vector<vector<double>> P;
    vector<vector<double>> H;
    vector<vector<double>> q;

public:
    SphericalKin() {
        cout << "invKin Constructor" << endl;
    }
    vector<double> subprob4(double d, vector<double>k, vector<double>h1, vector<double>h2) {
        vector<double> theta;
        k = k / norm(k);
        if (h1[0] / k[0] == h1[1] / k[1] && h1[1] / k[1] == h1[2] / k[2]) { theta.push_back(0); theta.push_back(0); return theta; }

        vector<double> kh2c = cross(k, h2);
        vector<double> kkh2c = cross(k, kh2c);
        double h1kkh2c = dot(h1, kkh2c);
        double a = dot(h1, kh2c);
        double b = -1 * h1kkh2c;
        double c = d - dot(h1, h2) + b;
        double phi = atan2(-1 * a, b);
        double costh = c / sqrt(a * a + b * b);

        if (fabs(costh) > 1) {}
        else {
            theta.push_back(acos(costh) - phi);
            if (fabs(costh) < 1) { theta.push_back(-1 * acos(costh) - phi); }
        }
        return theta;

    }
    vector<double> subprob3(double d, vector<double>p, vector<double>q, vector<double>k) {
        double dp = sqrt(d * d - pow(dot(k, p - q), 2));
        vector<double> pp = p - k * dot(p, k);
        vector<double> qp = q - k * dot(q, k);
        return subprob3a(dp, pp, qp, k);
    }
    vector<double> subprob3a(double d, vector<double>p, vector<double>q, vector<double>k) {
        double pn = norm(p), qn = norm(q);
        vector<double> theta;

        if (qn == 0 && pn == d) { theta.push_back(0); theta.push_back(0); return theta; }

        double cosphi = (pn * pn + qn * qn - d * d) / (2 * pn * qn);
        double qq = subprob1(p, q, k);

        if (fabs(cosphi) > 1) {}
        else {
            double q1 = qq + acos(cosphi);
            theta.push_back(q1);
            if (cosphi != 1) { double q2 = qq - acos(cosphi); theta.push_back(q2); }
        }
        return theta;
    }
    vector<vector<double>> subprob2(vector<double>p, vector<double>q, vector<double>k1, vector<double>k2) {
        p = p / norm(p);
        q = q / norm(q);
        k1 = k1 / norm(k1);
        k2 = k2 / norm(k2);
        double kk = dot(k1, k2);
        double k1p = dot(k1, p);
        double k2q = dot(k2, q);
        double a = (k1p - kk * k2q) / (1 - kk * kk);
        double b = (k2q - kk * k1p) / (1 - kk * kk);
        vector<double> kkc = cross(k1, k2);
        double kkcn = norm(kkc);
        double c2 = (dot(p, p) - a * a - b * b - 2 * a * b * kk) / (kkcn * kkcn);

        vector<vector<double>> theta;
        if (c2 < 0) {}
        else {
            vector<double> v1 = k1 * a + k2 * b, v2;
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
    double subprob1(vector<double>k, vector<double>p, vector<double>q) {
        double theta = 0;
        p = p / norm(p);
        q = q / norm(q);
        theta = subprob0(k, p - k * dot(p, k), q - k * dot(q, k));
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

        theta = 2 * atan2(norm(p - q), norm(p + q));
        if (dot(k, cross(p, q)) < 0) { theta = -1 * theta; }

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
    vector<vector<double>> rot(vector<double>k, double theta) {
        k = k / norm(k);

    }
    double dot(vector<double>const& a, vector<double>const& b) {
        double sol = 0;
        for (uint8_t i = 0; i < a.size(); i++) { sol += a[i] * b[i]; }
        return sol;
    }
    vector<vector<double>> hat(vector<double>k) {
        // vector<vector<double>> km;
        // vector<double> rol{0,-1.*k[2],k[1]};
        // km.push_back(rol);
        // rol[0]=k[2]; rol[1]=0; rol[2]=-1.*k[1];

    }
    vector<double> cross(vector<double>const& a, vector<double>const& b) {
        vector<double> c;
        c.push_back(a[1] * b[2] - a[2] * b[1]);
        c.push_back(a[2] * b[0] - a[0] * b[2]);
        c.push_back(a[0] * b[1] - a[1] * b[0]);
        return c;
    }
    double norm(vector<double>const& a) {
        double sol = 0;
        for (uint8_t i = 0; i < a.size(); i++) { sol += a[i] * a[i]; }
        return sqrt(sol);
    }
    double testFunc() {
        cout << "This is test function" << endl;

        Vector3d v(1, 2, 3), w(4,5,6);
        double sol;
        sol = 2+3;

        return v.dot(v);
    }
};

extern "C" {
    __declspec(dllexport) SphericalKin* SphericalKin_new() { return new SphericalKin(); }
    __declspec(dllexport) double inv_testFunv(SphericalKin* kin) { return kin->testFunc(); }
}

int main()
{
    // vector<vector<double>> T;
    // vector<double> Trol{1,2,3,4};
    // T.push_back(Trol);
    // cout << T[1:3][2] << endl;
    return 0;
}