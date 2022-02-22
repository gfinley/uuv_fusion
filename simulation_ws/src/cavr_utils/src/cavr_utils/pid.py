#The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
#More information: http://en.wikipedia.org/wiki/PID_controller
#
#cnr437@gmail.com
#
#######	Example	#########
#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#

# modified by ndutoit: 5/29/14

class PID:
    """
    Discrete PID control
    """
    
    def __init__(self, P=2.0, I=0.0, D=1.0, dt=0.1, Int=0, Int_max=500, Int_min=-500):
        
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.dt=dt
        self.Int=Int
        self.Int_max=Int_max
        self.Int_min=Int_min
        
        self.e1=0.0 # error at t-1

    def update(self, e):
        """
        Calculate PID output value for given reference input and feedback
        """
        
        self.P_value = self.Kp * e
        
        self.D_value = self.Kd * (e - self.e1) / self.dt
        
        self.Int = self.Int + self.dt*(self.e1 + e)/2.0 # trapezoidal rule
        
        if self.Int > self.Int_max:
            self.Int = self.Int_max
        elif self.Int < self.Int_min:
            self.Int = self.Int_min
            
        self.I_value = self.Int * self.Ki

        # update the error history
        self.e1 = e
            
        PID = self.P_value + self.I_value + self.D_value

        return PID

        
    def updateVel(self, e, de=0):
        """
        Calculate PID output value for given reference input and feedback
        """
        
        self.P_value = self.Kp * e
        
        self.D_value = self.Kd * de
        
        self.Int = self.Int + self.dt*(self.e1 + e)/2.0 # trapezoidal rule
        
        if self.Int > self.Int_max:
            self.Int = self.Int_max
        elif self.Int < self.Int_min:
            self.Int = self.Int_min
            
        self.I_value = self.Int * self.Ki

        # update the error history
        self.e1 = e
            
        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setIntegrator_max(self, int_max):
        self.Int_max = int_max

    def setIntegrator_min(self, int_min):
        self.Int_min = int_min

    def setDt(self,dt):
        self.dt=dt
        
    def setIntegrator(self, Integrator):
        self.Int = Integrator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getError(self):
        return self.e1
    
    def getIntegrator(self):
        return self.Int

# incremental PID algorithm
class PID_inc:
    """
    Incremental PID control
    """
    
    def __init__(self, P=2.0, I=0.0, D=1.0, dt=0.1, u_max = 500, u_min = -500):
        
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.dt=dt
        
        self.e1=0.0 # error at t-1
        self.e2=0.0 # error at t-2
        self.u1=0.0 # control at t-1
        self.u_max=u_max
        self.u_min=u_min

    def update(self, e, debug = 0):
        """
        Calculate PID output value using incremental algorithm:
        u(t) = u(t-1) + Kp*(e(t)-e(t-1)) + Ki*dt*e(t) + Kd/dt*(e(t) - 2*e(t-1) + e(t-2))
        """
        
        self.P_value = self.Kp * (e - self.e1)
        
        self.D_value = self.Kd * (e-2*self.e1+self.e2) / self.dt
        
        self.I_value = self.Ki * e * self.dt

        if debug>0:
            print 'self.P_value = %f, self.Kp = %f, e = %f, self.e1 = %f' % (self.P_value, self.Kp, e, self.e1)
            print 'self.D_value = %f, self.Kd = %f, e = %f, self.e1 = %f, self.e2 = %f, self.dt = %f' % (self.D_value, self.Kd, e, self.e1, self.e2, self.dt)
            print 'self.I_value = %f, self.Ki = %f, e = %f, self.dt = %f' % (self.I_value, self.Ki, e, self.dt) 
            print 'Error = %f, P = %f, I = %f, D = %f' % (e, self.P_value, self.I_value, self.D_value) 

        # update the error (history)
        self.e2 = self.e1
        self.e1 = e
            
        PID = self.u1 + self.P_value + self.I_value + self.D_value
        
        if PID > self.u_max:
            PID = self.u_max
        elif PID < self.u_min:
            PID = self.u_min
        
        # store the control
        self.u1 = PID

        if debug>0:
           print "PID: ", PID

        return PID

    def setDt(self,dt):
        self.dt=dt
        
    def setU1(self, u1):
        self.u1 = u1

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getError(self):
        return self.e1
