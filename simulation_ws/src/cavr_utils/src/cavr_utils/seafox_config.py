#!/usr/bin/env python
from math import pi, sin, cos
from numpy import array
from numpy.linalg import inv

# Specify the hydrodynamic model coefficients
## RIGID BODY DYNAMICS
m = 20.9 # mass, kg
W = m*9.81 # weight
B = W # neutrally buoyant
# moments of inertia
Ixx = 0.56# kg.m^2
Ixy = 0
Ixz = 0
Iyx = 0
Iyy = 0.95
Iyz = 0
Izx = 0
Izy = 0
Izz = 0.9
# location of center of gravity relative to body origin
xg = 0
yg = 0
zg = 0
# location of center of buoyancy relative to body origin
xb = 0
yb = 0
zb = 0

# Rigid body mass matrix
M_RB = array([[m, 0, 0, 0, m*zg,m*yg],
              [0, m, 0, -m*zg, 0, m*xg],
              [0, 0, m, m*yg, -m*xg, 0],
              [0, -m*zg, m*yg, Ixx, -Ixy, -Ixz],
              [m*zg, 0, -m*xg, -Iyx, Iyy, -Iyz],
              [-m*yg, m*xg, 0, -Izx, -Izy, Izz]])

def C_RB(nu):
    return array([[0, 0, 0, m*(yg*nu[4]+zg*nu[5]), -m*(xg*nu[4]-nu[2]), -m*(xg*nu[5]+nu[1])],
                  [0, 0, 0, -m*(yg*nu[3]+nu[2]), m*(zg*nu[5]+xg*nu[3]), -m*(yg*nu[5]-nu[0])],
                  [0, 0, 0, -m*(zg*nu[3]-nu[1]), -m*(zg*nu[4]+nu[0]), m*(xg*nu[3]+yg*nu[4])],
                  [-m*(yg*nu[4]+zg*nu[5]), m*(yg*nu[3]+nu[2]), m*(zg*nu[3]-nu[1]), 0, -Iyz*nu[4]-Ixz*nu[3]+Izz*nu[5], Iyz*nu[5]+Ixy*nu[3]-Iyy*nu[4]],
                  [m*(xg*nu[4]-nu[2]), -m*(zg*nu[5]+xg*nu[3]), m*(zg*nu[4]+nu[0]), Iyz*nu[4]+Ixz*nu[3]-Izz*nu[5], 0, -Ixz*nu[5]-Ixy*nu[4]+Ixx*nu[3]],
                  [m*(xg*nu[5]+nu[1]), m*(yg*nu[5]-nu[0]), -m*(xg*nu[3]+yg*nu[4]), -Iyz*nu[5]-Ixy*nu[3]+Iyy*nu[4], Ixz*nu[5]+Ixy*nu[4]-Ixx*nu[3], 0]])
    
## ADDED MASS DYNAMICS
X_du = -30.0 # kg
X_dv = 0
X_dw = 0
X_dp = 0
X_dq = 0
X_dr = 0
Y_du = 0
Y_dv = -21.6
Y_dw = 0
Y_dp = 0
Y_dq = 0
Y_dr = 0
Z_du = 0
Z_dv = 0
Z_dw = -21.4
Z_dp = 0
Z_dq = 0
Z_dr = 0
K_du = 0 # kg.m^2
K_dv = 0
K_dw = 0
K_dp = -23.9
K_dq = 0
K_dr = 0
M_du = 0
M_dv = 0
M_dw = 0
M_dp = 0
M_dq = -46.7
M_dr = 0
N_du = 0
N_dv = 0
N_dw = 0
N_dp = 0
N_dq = 0
N_dr = -0.4

# Added mass inertia matrix
M_A = array([[X_du, X_dv, X_dw, X_dp, X_dq, X_dr],
             [Y_du, Y_dv, Y_dw, Y_dp, Y_dq, Y_dr],
             [Z_du, Z_dv, Z_dw, Z_dp, Z_dq, Z_dr],
             [K_du, K_dv, K_dw, K_dp, K_dq, K_dr],
             [M_du, M_dv, M_dw, M_dp, M_dq, M_dr],
             [N_du, N_dv, N_dw, N_dp, N_dq, N_dr]])

M_inv = inv(M_RB+M_A)

# Added mass Coriolis
def C_A(nu):
    a1 = X_du*nu[0]+X_dv*nu[1]+X_dw*nu[2]+X_dp*nu[3]+X_dq*nu[4]+X_dr*nu[5]
    a2 = X_du*nu[0]+Y_dv*nu[1]+Y_dw*nu[2]+Y_dp*nu[3]+Y_dq*nu[4]+Y_dr*nu[5]
    a3 = X_du*nu[0]+Y_dv*nu[1]+Z_dw*nu[2]+Z_dp*nu[3]+Z_dq*nu[4]+Z_dr*nu[5]
    b1 = X_du*nu[0]+Y_dv*nu[1]+Z_dw*nu[2]+K_dp*nu[3]+K_dq*nu[4]+K_dr*nu[5]
    b2 = X_du*nu[0]+Y_dv*nu[1]+Z_dw*nu[2]+K_dp*nu[3]+M_dq*nu[4]+M_dr*nu[5]
    b3 = X_du*nu[0]+Y_dv*nu[1]+Z_dw*nu[2]+K_dp*nu[3]+M_dq*nu[4]+N_dr*nu[5]
    return array([[0,0,0,0,-a3,a2],
                  [0,0,0,a3,0,-a1],
                  [0,0,0,-a2,a1,0],
                  [0,-a3,a2,0,-b3,b2],
                  [a3,0,-a1,b3,0,-b1],
                  [-a2,a1,0,-b2,b1,0]])

## DAMPING
# linear damping
X_u = 0
Y_v = 0
Z_w = 0
K_p = 0
M_q = 0
N_r = 0
# quadratic damping
X_uu = -4.6
Y_vv = -55.0
Z_ww = -67.8
K_pp = -103.3
M_qq = -46.7
N_rr = -0.9

def D(nu):
    return array([[-X_u-X_uu*abs(nu[0]), 0, 0, 0, 0, 0],
                  [0, -Y_v-Y_vv*abs(nu[1]), 0, 0, 0, 0],
                  [0, 0, -Z_w-Z_ww*abs(nu[2]), 0, 0, 0],
                  [0, 0, 0, -K_p-K_pp*abs(nu[3]), 0, 0],
                  [0, 0, 0, 0, -M_q-M_qq**abs(nu[4]), 0],
                  [0, 0, 0, 0, 0, -N_r-N_rr*abs(nu[5])]])

# restoring forces
def g(eta):
    return array([[(W-B)*sin(eta[3])],
                  [-(W-B)*cos(eta[3])*sin(eta[4])],
                  [-(W-B)*cos(eta[3])*cos(eta[4])],
                  [-(yg*W-yb*B)*cos(eta[3])*cos(eta[4])+(zg*W-zb*B)*cos(eta[3])*sin(eta[4])],
                  [(zg*W-zb*B)*sin(eta[3])+(xg*W-xb*B)*cos(eta[3])*cos(eta[4])],
                  [-(xg*W-xb*B)*cos(eta[3])*sin(eta[4])-(yg*W-yb*B)*sin(eta[3])]])

# Earth-fixed representation
def J(eta):
    return array([[cos(psi)*cos(eta[4]), -sin(psi)*cos(eta[3])+cos(psi)*sin(eta[4])*sin(eta[3]), sin(psi)*sin(eta[3])+cos(psi)*cos(eta[3])*sin(eta[4]), 0, 0, 0],
                  [sin(psi)*cos(eta[4]), cos(psi)*cos(eta[3])+sin(psi)*sin(eta[4])*sin(eta[3]), -cos(psi)*sin(eta[3])+sin(psi)*cos(eta[3])*sin(eta[4]), 0, 0, 0],
                  [-sin(eta[4]), cos(eta[4])*sin(eta[3]), cos(eta[4])*cos(eta[3]), 0, 0, 0]
                  [0, 0, 0, 1, sin(eta[3])*tan(eta[4]), cos(eta[3])*tan(eta[4])],
                  [0, 0, 0, 0, cos(eta[3]), -sin(eta[3])],
                  [0, 0, 0, 0, sin(eta[3])/cos(eta[4]), cos(eta[3])/cos(eta[4])]])
    