#include <BasicLinearAlgebra.h>
using namespace BLA;

float d = 0.0005;
float m = 0.0005502217882599091;

Matrix<2,2> A = { 0, 1,
                  0, -d/m };

Matrix<2,1> B = { 0,
                  1/m };

Matrix<1,2> C = { 1,
                  0 };

Matrix<2,2> sig = { 25, 0,
                    0, 25 };

Matrix<2,2> sig_u = { 100, 0,
                      0, 100 };

Matrix<1,1> sig_z = { 400 };

Matrix<2,1> x = {800, 0}; // gets set by other functions - format [ Front ToF Sensor, PWM ]
Matrix<2,2> Ad = { 1,  0.01511407,
                   0,  0.98626548 };
       
Matrix<2,1> Bd = { 0,
                   27.4690485 };

Matrix<2,2> I_2 = { 1, 0,
                    0, 1 };
                    
Matrix<1,1> oldTOFVal;

bool kfPWMReady = false;

float performKF(float tof, float motorSpeed) {
    Matrix<2,1> mu_p = Ad * x + Bd * motorSpeed/100;
    Matrix<2,2> sigma_p = Ad * sig * ~Ad + sig_u;

    Matrix<1,1> tofVal;
    if (tof == -1000) { // no new value
      tofVal = oldTOFVal;
    } else {
      tofVal = { tof };
      oldTOFVal = { tof };
    }
    
    Matrix<1,1> y_m = tofVal - C * mu_p;
    Matrix<1,1> sigma_m = C * sigma_p * ~C + sig_z;

    Matrix<1,1> sigma_m_invert = sigma_m;
    Invert(sigma_m_invert);

    Matrix<2,1> kkf_gain = sigma_p * ~C * sigma_m_invert;

    if (tof != -1000) { // only update where there is a new value
      x = mu_p + kkf_gain * y_m;
      sig = (I_2 - (kkf_gain * C)) * sigma_p;
    }
    
    kfPWMReady = true;
    return x(0,0); // return the new value
}
