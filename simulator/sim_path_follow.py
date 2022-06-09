import numpy as np
import math
import matplotlib.pyplot as plt
import numpy.linalg as la
import control as c

#Dimensions

l_wb = 0.25#wheelbase length
r_w = 0.05#wheel radius
r_G = r_w
T_S = 4.06 # stall torque [Nm]
w_nl = 86. * ((2*math.pi) /60) # no load speed [rad/s]
m = 10. #mass of robot, I just guessed this
J_B = 0.5*m**0.25*1.2*l_wb**2 #Same here for mass moment of inertia

#make a state-space model of the robot
c1= T_S / (m * r_G**2)
c2 = T_S / (m * w_nl * r_G**2)
c3 = T_S / (4 * J_B * r_G**2)
c4 = T_S / (4 * J_B * w_nl * r_G**2)
c5 = r_G / 2
c6 = r_G / l_wb

A = np.matrix([	[-c2-c4, -c2+c4, 0, 0],
				[-c2+c4, -c2-c4, 0, 0],
				[-1, 0 ,0, 0		],
				[0, -1, 0, 0		]]) #State x = [w_R, w_L, E_R, E_L] (E_R and E_L are the inputs integrated for the I term)
B = np.matrix([	[c1+c3, c1-c3],
				[c1-c3, c1+c3],
				[1, 0],
				[0, 1]]) #Input u = [u_R, u_L] (where u is the input duty cycle)s

C = np.matrix([	[1, 0, 0, 0],
				[0, 1, 0, 0],
				[c5, c5, 0, 0],
				[c6, -c6, 0, 0],
				[1, 0, 0, 0],
				[0, 1, 0, 0],
				[-c2-c4, -c2+c4, 0, 0],
				[-c2+c4, -c2-c4, 0, 0],
				[0, 0, 1, 0],
				[0, 0, 0, 1]]) #Outputs y = [w_R, w_L, v_B, w_B, e_R, e_L, wdot_R, wdot_L, E_R, E_L], more stuff could be added if needed
				
D = np.matrix([	[0, 0],
				[0, 0],
				[0, 0],
				[0, 0],
				[1, 0],
				[0, 1],
				[c1+c3, c1-c3],
				[c1-c3, c1+c3],
				[0, 0],
				[0, 0]])
				

pal_sys = c.ss(A, B, C, D)

#used for testing, may flesh this out into splines but there is also scipy I guess
class cubic:
	def __init__(self, a, b, c, d, s=1.):
		self.a = float(a)*s
		self.b = float(b)*s
		self.c = float(c)*s
		self.d = float(d)*s
		
	def f(self, t):
		return self.a*t**3 + self.b*t**2 + self.c*t + self.d
		
	def df_dx(self, t):
		return 3*self.a*t**2 + 2*self.b*t + self.c
		
	def d2f_dx2(self, t):
		return 6*self.a*t + 2*self.b

def v_t(cubicx, cubicy, t):
	v = [cubicx.df_dx(t), cubicy.df_dx(t)]
	return la.norm(v)

def omega_p(cubicx, cubicy, t):

	xdot, ydot = (cubicx.df_dx(t), cubicy.df_dx(t))
	xddot, yddot = (cubicx.d2f_dx2(t), cubicy.d2f_dx2(t))
	omega = (xdot*yddot - ydot*xddot)/la.norm([xdot, ydot])
	return omega
	
	
'''not used anymore
def kin_robot(v_L, v_R):
	omega_BL = v_L/(l_wb*0.5)
	omega_BR = v_R(l_wb*0.5)
	v_B = (v_L + v_R) * 0.5
	omega_B = omega_BR-omega_BL
	return v_B, omega_B
''' 
def inv_kin_robot(v_B, omega_B):
	v_L = v_B - omega_B*l_wb
	v_R = v_B + omega_B*l_wb
	return v_L, v_R
	

s = 1
x = cubic(3, 0.5, -4.5, 0, s)
y = cubic(2.8, -7.1, 2.7, 0, s)


dt = 0.01
tb = 0
tf = 1
t = np.arange(tb, tf+dt, dt)

vv_t = []
oo_p = []
for t_i in t:
	vv_t.append(v_t(x, y, t_i))
	oo_p.append(omega_p(x, y, t_i))
vv_t = np.array(vv_t)
oo_p = np.array(oo_p)

vv_L, vv_R = inv_kin_robot(vv_t, oo_p)


fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2,2)
ax1.plot(x.f(t), y.f(t))
ax2.plot(t, vv_t, t, oo_p)
ax3.plot(t, vv_L, t, vv_R)


ax1.set_title("path")
ax2.set_title("tangential and angular velocities")
ax3.set_title("wheel speeds")

plt.grid(True)
plt.show()


