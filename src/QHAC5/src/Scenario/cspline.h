/* Title: CSpline class for generating a trajectory from via-points in 3D
 * Date: 2014. 2. Oct. ver. 1.2
 * Author: Jang, Jong Tai @ KARI(Korea Aerospace Research Institute) jjt@kari.re.kr
 * Purpose: To get the trajectory coordinates with speed profile by c-spline method in 3D
 * Remark: Numerical methods were used for CSpline, SparseLU and Gauss-Legendre Quadrature
 * Generation Mode : Speed-Specified mode, Time-Specified mode
 * Input on Speed-Specified mode : (actual time, x, y, z, speed) for each via-points
 * Input on Time-Specified mode : (actual time, x, y, z, virtual time) for each via-points
 * 'actual time' is the real time and 'virtual time' is used to make C-spline path
 * Calling procedure
 *  1) Make an object of this CSpline class
 *  2) initSpline(mode)
 *  3) addPoint(actual time, x, y, z, speed) or addPoint(actual time, x, y, z, virtual time)
 *  4) startSpeed() and endSpeed() (if these are ommited then automatically calculated)
 *  5) makeSpline()
 *  6) initTrajectory()
 *  7) nextPoint() (call iteratively until it returns false)
 * Modified Items in this version
 *  1) the SparseLU solver routine was improved in the cspline1D()
 *  2) the number of via-points in the arguments of initSpline() was removed
 */

#ifndef CSPLINE_H
#define CSPLINE_H

#include <Eigen/Core>
#include <Eigen/SparseLU>

class CSpline
{
public:    
    CSpline(); // constructor

    enum Mode { TIME_SPECIFIED, SPEED_SPECIFIED };

    Mode mode;
    bool startSpeedSpecified, endSpeedSpecified;

    /* input variables for c-splines */

    int N; // the quantity of via-points of spline including the first and the end points
    int j; // the index of current spline while making trajectory
    int k; // the interation index in real time while making trajectory

    double t; // the virtual time on spline while making trajectory
    double T; // the real time on trajectory while making trajectory

    double dt; // the variable time increment in virtual time in spline
    double dT; // the constant time increment in real time in trajectory

    double xp, yp, zp; // the position of the original trajectory
    double xv, yv, zv; // the velocity of the original trajectory
    double cv; // the current speed of the original trajectory
    double dv; // the desired speed at current time t

    Eigen::VectorXd tp; // the virtual time at the via points to be curve-fitted
    Eigen::VectorXd px; // the x values of the via points to be curve-fitted
    Eigen::VectorXd py; // the y values of the via points to be curve-fitted
    Eigen::VectorXd pz; // the z values of the via points to be curve-fitted
    Eigen::VectorXd pv; // the speed at the via points to be curve-fitted
    Eigen::VectorXd vp; // the speed values at the via points to be curve-fitted
    Eigen::Vector3d vs; // the velocity at the start point
    Eigen::Vector3d ve; // the velocity at the end point

    /* output varaibles for c-splines */

    // The polynomial function of the j-th c-spline:
    // sj(t) = aj + bj*(t - tj) + cj*(t - tj)^2 + dj*(t - tj)^3

    // coefficient for x coordinates of spline
    Eigen::VectorXd ax; // the coefficients of 0 degree term of x coordinate polynomial of c-splines
    Eigen::VectorXd bx; // the coefficients of 1 degree term of x coordinate polynomial of c-splines
    Eigen::VectorXd cx; // the coefficients of 2 degree term of x coordinate polynomial of c-splines
    Eigen::VectorXd dx; // the coefficients of 3 degree term of x coordinate polynomial of c-splines

    // coefficient for y coordinates of spline
    Eigen::VectorXd ay; // the coefficients of 0 degree term of y coordinate polynomial of c-splines
    Eigen::VectorXd by; // the coefficients of 1 degree term of y coordinate polynomial of c-splines
    Eigen::VectorXd cy; // the coefficients of 2 degree term of y coordinate polynomial of c-splines
    Eigen::VectorXd dy; // the coefficients of 3 degree term of y coordinate polynomial of c-splines

    // coefficient for z coordinates of spline
    Eigen::VectorXd az; // the coefficients of 0 degree term of z coordinate polynomial of c-splines
    Eigen::VectorXd bz; // the coefficients of 1 degree term of z coordinate polynomial of c-splines
    Eigen::VectorXd cz; // the coefficients of 2 degree term of z coordinate polynomial of c-splines
    Eigen::VectorXd dz; // the coefficients of 3 degree term of z coordinate polynomial of c-splines

    Eigen::VectorXd L; // lengths of each spline
    Eigen::VectorXd Tp; // the real time at the via points to be curver-fitted

    void initSpline(Mode m); // to initialize class, n is the number of spline points
    void resize(void); // to resize variables according to the number of spline points
    void addPoint(double time, double x, double y, double z, double v); // add a via point sequentially with speed or virtual time
    void addPoint(double time, double x, double y, double z); // add a via point sequentially without speed and virtual time
    void startSpeed(double sx, double sy, double sz); // to set the speed at the start point
    void endSpeed(double ex, double ey, double ez); // to set the speed at the end point
    void makeSpline(void); // to make trajectory

    void initTrajectory(void); // to initialize to generate trajectory
    bool nextPoint(double &t, double &x, double &y, double &z); // to get the next point of trajectory

    // to make c-splines in one dimension (this is called by CSpline::makeSpline())
    void cspline1D(Eigen::VectorXd t, Eigen::VectorXd p, double vs, double ve, Eigen::VectorXd &a, Eigen::VectorXd &b, Eigen::VectorXd &c, Eigen::VectorXd &d);
};

#endif // CSPLINE_H
