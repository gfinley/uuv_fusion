#! /usr/bin/env python
import numpy as np
from copy import deepcopy

class System:
    A = None
    B = None
    C = None
    D = None
    nx = None
    ny = None
    nu = None
    
    def __init__(self):
        return
    
class Weights:
    R = None
    Q = None
        
    def __init__(self):
        return

class KalmanFilter:
    # Kalman Filter implementation
    
    # prediction    
    xk_k1 = None # predicted estimate
    Pk_k1 = None # predicted covariance
    # update
    xk_k = None # updated estimate
    Pk_k = None # updated covariance

    def __init__(self, x0, P0=None, system = None, weights = None):
        # expects matrices as numpy array, x0 as Pose
        # set the dynamics
        if system==None:
            self.A = np.identity(nx)
            self.B = np.zeros(nx,nu)
            self.C = np.eye(ny, nx)
            self.nx = 6
            self.nu = 1
            self.ny = 6
        else:
            self.A = deepcopy(system.A)
            self.B = deepcopy(system.B)
            self.C = deepcopy(system.C)
            self.nx = system.nx
            self.nu = system.nu
            self.ny = system.ny
            
        if weights==None:
            self.Q = np.identity(self.nx)
            self.R = np.identity(self.ny)
        else:
            self.Q = deepcopy(weights.Q)
            self.R = deepcopy(weights.R)
            
        if P0==None:
            self.P0 = np.identity(self.nx)
        else:
            self.P0 = deepcopy(P0)
        self.P0_n = np.linalg.norm(P0, 2) # matrix 2 norm -> largest singular value
        
        # initial estimate
        x = [x0.position.x, x0.position.y, x0.position.z]
        x.extend(geo_utils.euler_from_quat_q(x0.orientation))
        self.xk_k = np.array(x) # column vector
        self.xk_k.shape = (len(x), 1)
        self.xk_k1 = deepcopy(self.xk_k)
        #print self.xk_k 
        self.Pk_k = deepcopy(self.P0)
        self.Pk_k1 = deepcopy(self.Pk_k)
        self.Pk_k_n = self.P0_n
#         print "A = ", self.A.shape
#         print "B = ", self.B.shape        
#         print "C = ", self.C.shape 
#         print "xk_k1 = ", self.xk_k1.shape 
#         print "xk_k = ", self.xk_k.shape 
#         print "Pk_k1 = ", self.Pk_k1.shape 
#         print "Pk_k = ", self.Pk_k.shape
        
        return
    
    def predict(self, u=None):
        # expects u as numpy array or None
        if u == None:
            u = np.zeros((self.nu, 1))
        
        self.xk_k1 = self.A.dot(self.xk_k) + self.B.dot(u)
        self.Pk_k1 = self.A.dot(self.Pk_k.dot(np.transpose(self.A))) + self.Q
        #print "xk_k1 = ", self.xk_k1.shape 
        #print "Pk_k1 = ", self.Pk_k1.shape 
        return
    
    def update(self, z=None, matrix_norm=2):
        if z == None:
            # no new measurement, so using predicted state
            self.xk_k = deepcopy(self.xk_k1)
            self.Pk_k = deepcopy(self.Pk_k1)
        else:
            zk = np.array(z)
            zk.shape = (len(z), 1)
            yk = zk - self.C.dot(self.xk_k1)
            #print "yk = ", yk.shape
            Sk = self.C.dot(self.Pk_k1.dot(np.transpose(self.C))) + self.R
            #print "Sk = ", Sk.shape
            Kk = self.Pk_k1.dot(np.transpose(self.C).dot(np.linalg.inv(Sk)))
            #print "Kk = ", Kk.shape
            self.xk_k = self.xk_k1 + Kk.dot(yk)
            
            self.Pk_k = (np.identity(6) - Kk.dot(self.C)).dot(self.Pk_k1)

        #print "xk_k = ", self.xk_k.shape 
        #print "Pk_k = ", self.Pk_k.shape
        
        # check if the covariance has grown too large, in which case we want to restrict it
        self.Pk_k_n = np.linalg.norm(self.Pk_k, matrix_norm)
        if self.Pk_k_n > self.P0_n:  # matrix 2 norm -> largest singular value
            self.Pk_k = deepcopy(self.P0)
        return
    
    def getEstimate(self):
        return deepcopy(self.xk_k)
    
    def getCovariance(self):
        return deepcopy(self.Pk_k)
    
    def getCovarianceSize(self):
        return self.Pk_k_n