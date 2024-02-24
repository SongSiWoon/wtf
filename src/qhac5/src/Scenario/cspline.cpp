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

#include <iostream>
#include <fstream>

using namespace std;

#include "customconfig.h"
#include "cspline.h"

CSpline::CSpline() // constructor
{
}

void CSpline::initSpline(Mode m) // to initailize class, N is the number of spline points
{
    mode = m;

    startSpeedSpecified = false;
    endSpeedSpecified = false;

    N = 0;
    j = 0;

    vs.resize(3);
    ve.resize(3);

    vs.setZero();
    ve.setZero();
}

void CSpline::resize(void) // to resize variables according to the number of spline points
{
    Eigen::VectorXd copy_tp;
    Eigen::VectorXd copy_Tp;
    Eigen::VectorXd copy_px;
    Eigen::VectorXd copy_py;
    Eigen::VectorXd copy_pz;
    Eigen::VectorXd copy_vp;
    Eigen::VectorXd copy_ax;
    Eigen::VectorXd copy_bx;
    Eigen::VectorXd copy_cx;
    Eigen::VectorXd copy_dx;
    Eigen::VectorXd copy_ay;
    Eigen::VectorXd copy_by;
    Eigen::VectorXd copy_cy;
    Eigen::VectorXd copy_dy;
    Eigen::VectorXd copy_az;
    Eigen::VectorXd copy_bz;
    Eigen::VectorXd copy_cz;
    Eigen::VectorXd copy_dz;
    Eigen::VectorXd copy_L;

    N++; // to increase the size of vectors by one

    // to store the old values of the vectors
    if(N > 1)
    {
        copy_tp = tp;
        copy_Tp = Tp;
        copy_px = px;
        copy_py = py;
        copy_pz = pz;
        copy_vp = vp;
        copy_ax = ax;
        copy_bx = bx;
        copy_cx = cx;
        copy_dx = dx;
        copy_ay = ay;
        copy_by = by;
        copy_cy = cy;
        copy_dy = dy;
        copy_az = az;
        copy_bz = bz;
        copy_cz = cz;
        copy_dz = dz;
        copy_L = L;
    }

    // to increase the sizes of vectors by one

    tp.resize(N);
    Tp.resize(N);
    px.resize(N);
    py.resize(N);
    pz.resize(N);
    vp.resize(N);

    ax.resize(N);
    bx.resize(N);
    cx.resize(N);
    dx.resize(N);

    ay.resize(N);
    by.resize(N);
    cy.resize(N);
    dy.resize(N);

    az.resize(N);
    bz.resize(N);
    cz.resize(N);
    dz.resize(N);

    L.resize(N);

    // to initialize vectors to zero

    tp.setZero();
    Tp.setZero();
    px.setZero();
    py.setZero();
    pz.setZero();
    vp.setZero();

    ax.setZero();
    bx.setZero();
    cx.setZero();
    dx.setZero();

    ay.setZero();
    by.setZero();
    cy.setZero();
    dy.setZero();

    az.setZero();
    bz.setZero();
    cz.setZero();
    dz.setZero();

    L.setZero();

    // to restore previous values to the resized vectors

    if(N > 1)
    {
        tp.head(N-1) = copy_tp;
        Tp.head(N-1) = copy_Tp;
        px.head(N-1) = copy_px;
        py.head(N-1) = copy_py;
        pz.head(N-1) = copy_pz;
        vp.head(N-1) = copy_vp;
        ax.head(N-1) = copy_ax;
        bx.head(N-1) = copy_bx;
        cx.head(N-1) = copy_cx;
        dx.head(N-1) = copy_dx;
        ay.head(N-1) = copy_ay;
        by.head(N-1) = copy_by;
        cy.head(N-1) = copy_cy;
        dy.head(N-1) = copy_dy;
        az.head(N-1) = copy_az;
        bz.head(N-1) = copy_bz;
        cz.head(N-1) = copy_cz;
        dz.head(N-1) = copy_dz;
        L.head(N-1) = copy_L;
    }
}

// add a via point sequentially with speed or virtual time
void CSpline::addPoint(double time, double x, double y, double z, double v)
{
    resize();

    if(mode == SPEED_SPECIFIED)
    {
        tp[j] = time;
        px[j] = x;
        py[j] = y;
        pz[j] = z;
        vp[j] = v;
    }
    else if(mode == TIME_SPECIFIED)
    {
        tp[j] = v;
        px[j] = x;
        py[j] = y;
        pz[j] = z;
        Tp[j] = time;
    }

    j++;
}

// add a via point sequentially without speed and virtual time
void CSpline::addPoint(double time, double x, double y, double z)
{
    resize();

    tp[j] = time;
    px[j] = x;
    py[j] = y;
    pz[j] = z;
    Tp[j] = time;

    j++;
}

// to set the speed at the start point
void CSpline::startSpeed(double sx, double sy, double sz)
{
    vs << sx, sy, sz;

    startSpeedSpecified = true;
}

// to set the speed at the end point
void CSpline::endSpeed(double ex, double ey, double ez)
{
    ve << ex, ey, ez;

    endSpeedSpecified = true;
}

void CSpline::makeSpline(void) // to make trajectory
{
    // to calculate start velocity of the path when not specified
    if(!startSpeedSpecified)
    {
        double sx, sy, sz;

        sx = (px[1] - px[0])/(tp[1] - tp[0]);
        sy = (py[1] - py[0])/(tp[1] - tp[0]);
        sz = (pz[1] - pz[0])/(tp[1] - tp[0]);

        vs << sx, sy, sz;
    }

    // to calculate end velocity of the path when not specified
    if(!endSpeedSpecified)
    {
        double ex, ey, ez;

        ex = (px[N-1] - px[N-2])/(tp[1] - tp[0]);
        ey = (py[N-1] - py[N-2])/(tp[1] - tp[0]);
        ez = (pz[N-1] - pz[N-2])/(tp[1] - tp[0]);

        ve << ex, ey, ez;
    }

    // to make c-spline 3D for the x, y, z coordinate respectively
    cspline1D(tp, px, vs[0], ve[0], ax, bx, cx, dx);
    cspline1D(tp, py, vs[1], ve[1], ay, by, cy, dy);
    cspline1D(tp, pz, vs[2], ve[2], az, bz, cz, dz);

    cout << "ax\n" << ax << endl << "bx\n" << bx << endl << "cx\n" << cx << endl << "gainDx\n" << dx << endl;
    cout << "ay\n" << ay << endl << "by\n" << by << endl << "cy\n" << cy << endl << "gainDy\n" << dy << endl;
    cout << "az\n" << az << endl << "bz\n" << bz << endl << "cz\n" << cz << endl << "gainDz\n" << dz << endl;
}

// to initialize to generate trajectory
void CSpline::initTrajectory(void)
{
    /* Gauss-Legendre quadrature using 5 evaluation points to integrate speed of c-spline
     * for getting the lengths of each spline from here */

    double z[5]; // evalutation points
    double w[5]; // weight values
    double x; // the transformed evaluation point from the z value (= t)
    double f; // the evaluation from the x

    // to calculate evaluation points from the table known well
    z[0] = -sqrt(5 + 2*sqrt(10.0/7.0))/3.0;
    z[1] = -sqrt(5 - 2*sqrt(10.0/7.0))/3.0;
    z[2] = 0;
    z[3] = -z[1];
    z[4] = -z[0];

    // to calculate weight values from the table knwon well
    w[0] = (322.0 - 13.0*sqrt(70.0))/900.0;
    w[1] = (322.0 + 13.0*sqrt(70.0))/900.0;
    w[2] = 128.0/225.0;
    w[3] = w[1];
    w[4] = w[0];

    // to get the lengths of each spline
    for(int i = 0; i < N-1; i++)
    {
        // to sum the multiplication of the weights and the evaluations of speed
        for(int j = 0; j < 5; j++)
        {
            // to transform from z to x
            x = ((tp[i+1] - tp[i])*z[j] + (tp[i] + tp[i+1]))/2.0;

            f = sqrt(pow(bx(i) + 2.0*cx(i)*(x - tp(i)) + 3.0*dx(i)*pow(x - tp(i), 2), 2)
                   + pow(by(i) + 2.0*cy(i)*(x - tp(i)) + 3.0*dy(i)*pow(x - tp(i), 2), 2)
                   + pow(bz(i) + 2.0*cz(i)*(x - tp(i)) + 3.0*dz(i)*pow(x - tp(i), 2), 2));

            // to sum the muliplication of the weight and the evaluation
            L(i) = L(i) + (tp[i+1] - tp[i])/2.0*w[j]*f;
        }

        cout << "L(" << i << ") = " << L(i) << endl;
    }

    /* to here */

    if(mode == TIME_SPECIFIED)
    {
        vp[0] = 0; // to set the start speed of the trajectory (TBD)

        // to calculate the speeds at each via-points after start point
        for(int i = 0; i < N-1; i++)
        {
            vp(i+1) = 2*L(i)/(Tp(i+1) - Tp(i)) - vp(i);
        }

        for(int i = 0; i < N; i++)
        {
            cout << "vp(" << i << ") = " << vp(i) << endl;
        }
    }
    else if(mode == SPEED_SPECIFIED)
    {
        Tp[0] = tp[0]; // the real time and pseudo time is same at start point

        // to calculate the real times at each via-points after start point
        for(int i = 0; i < N-1; i++)
        {
            Tp(i+1) = Tp(i) + 2*L(i)/(vp(i+1) + vp(i));
        }

        for(int i = 0; i < N; i++)
        {
            cout << "Tp(" << i << ") = " << Tp(i) << endl;
        }
    }
    else
    {
        cout << "unknown mode";
    }

    // to initialize variables used for CSpline::nextPoint() function
    j = 0; // to initialize the spline index
    k = 0; // to initialize the interation index in real time
    t = tp(0); // to initialize the virtual time on spline
    T = Tp(0); // to initialize the real time on trajectory
    dt = 0.001; // to initialize the variable time increment in virtual time in spline
    dT = 1./CONTROL_RATE; // to initialize the constant time increment in real time in trajectory
    dv = vp(0); // the desired speed at current time t
}

// to get the next point of trajectory one by one by each call
bool CSpline::nextPoint(double &time, double &x, double &y, double &z)
{
    double delta, square, cubic; // the values repeated in this function

    time = T; // to return the actual time

    if(j < N-1)
    {
        // to get the values repeated in this scope
        delta = t - tp(j); // (t - tp(j))
        square = delta*delta; // (t - tp(j))^2
        cubic = square*delta; // (t - tp(j))^3

        // to get the postion of the original trajectory (this is the position of trajectory)
        xp = ax(j) + bx(j)*delta + cx(j)*square + dx(j)*cubic; // to get x coordinate of trajectory
        yp = ay(j) + by(j)*delta + cy(j)*square + dy(j)*cubic; // to get y coordinate of trajectory
        zp = az(j) + bz(j)*delta + cz(j)*square + dz(j)*cubic; // to get z coordinate of trajectory

        // velocity of the original trajectory
        xv = bx(j) + 2*cx(j)*delta + 3*dx(j)*square;
        yv = by(j) + 2*cy(j)*delta + 3*dy(j)*square;
        zv = bz(j) + 2*cz(j)*delta + 3*dz(j)*square;

        // to get the speed of current iteration from the spline
        cv = sqrt(xv*xv + yv*yv + zv*zv);

        dt = (dv*dT)/cv; // to get time increment in pseudo time

        // to get the desired speed at the current time t from the speed profile
        dv = vp(j) + (vp(j+1) - vp(j))/(Tp(j+1) - Tp(j))*(T - Tp(j));

        t = t + dt; // to increase pseudo time

//        tt(k) = t;

        k = k + 1; // to increase iteration index

        T = Tp(0) + k*dT; // to increase current real time

        if(T >= Tp(j+1)) j++;

        // to output the trajectory coordinates
        x = xp;
        y = yp;
        z = zp;

        return true; // this means that the trajectory is on going
    }
    else
    {
        return false; // this means the end of the trajectory
    }

//    kk = k;
}

// to make c-splines in one dimension
void CSpline::cspline1D(Eigen::VectorXd t, Eigen::VectorXd p, double vs, double ve, Eigen::VectorXd &a, Eigen::VectorXd &b, Eigen::VectorXd &c, Eigen::VectorXd &d)
{
    // The j-th spline function:
    // sj(x) = aj + bj*(x - xj) + cj*(x - xj)^2 + dj*(x - xj)^3

    Eigen::VectorXd h(N); // difference of x coordinate of the points
    Eigen::MatrixXd E(N, N);
    Eigen::VectorXd f(N);

    // to initialize matrices and vectors to zero
    h.setZero();
    E.setZero();
    f.setZero();

    // The coefficients a is the y values of the four points
    a = p;

    // Calculates the h which is the difference of x coordinate of the points
    for(int k = 0; k < N - 1; k++)
    {
        h(k) = t(k+1) - t(k);
    }

    // no need to transpose h to make row vector into column vector

    // The associated equations for getting coefficient c:
    // E * c = f

    // To make E matrix
    E(0, 0) = 2*h(0);
    E(0, 1) = h(0);

    for(int k = 1; k < N - 1; k++)
    {
        E(k, k - 1) = h(k - 1);
        E(k, k) = 2*(h(k - 1) + h(k));
        E(k, k + 1) = h(k);
    }

    E(N-1, N-2) = h(N-2);
    E(N-1, N-1) = 2*h(N-2);

    // To make f vector

    f(0) = (3/h(0))*(a(1) - a(0)) - 3*vs;

    for(int k = 1; k < N - 1; k++)
    {
        f(k) = (3/h(k))*(a(k+1) - a(k)) - (3/h(k-1))*(a(k) - a(k-1));
    }

    f(N-1) = 3*ve - (3/h(N-2))*(a(N-1) - a(N-2));

    /* The coefficient c is calculated by solving E*c = f for c
       not using inverse of E but instead using sparse matrix solver (very accurate)
      because the inverse method of E results poor accuracy (about 5% error) from here */

    /* Instead of using SparseLU method, aprroximate solution can be obtained by other methods:
     * c = E.partialPivLu().solve(f);  or  c = E.llt().solve(f); */

    Eigen::SparseMatrix<double> Es(N, N);

    Es.setZero();

    // to fill in the sparse matrix Es with the elements of E, which are not zeros
    for(int i = 0; i < N; i++)
    {
        for(int j = 0; j < N; j++)
        {
            if(E(i,j) != 0)
            {
                Es.insert(i,j) = E(i,j);
            }
        }
    }

    Es.makeCompressed();

    Eigen::SparseLU< Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int> > solver;

    solver.compute(Es); // Changed in this version

    c = solver.solve(f);

    /* to here */

    // The coefficient b is calculated by the coefficients a and c:
    for(int k = 0; k < N - 1; k++)
    {
        b(k) = (a(k + 1) - a(k))/h(k) - (h(k)/3)*(2*c(k) + c(k + 1));
    }

    // no need to transpose b to make row vector into column vector

    // The coefficient d is calculated only by the coefficient c:
    for(int k = 0; k < N - 1; k++)
    {
        d(k) = (c(k + 1) - c(k))/(3*h(k));
    }

    // no need to transpose d to make row vector into column vector

    // Now we've got all the coefficients a, b, c and d for the three c-splines.
}
