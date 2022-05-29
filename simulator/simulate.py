import numpy as np
import math
import matplotlib.pyplot as plt
import scipy.optimize as opt
import scipy.integrate as integrate
import scipy.misc as misc


def ramp(t, t0, t1, size):
    return size*(t>=t0)*(1.-(t<=t1)*(1.-t/(t1-t0)))

def triangle_wave(t):
	periodic = t - np.floor(t)
	return np.abs(periodic-.5)*2

# --- Input constants --- #
w_n = 86. * ((2*math.pi) /60) # no load speed [rad/s]
T_S = 4.06 # stall torque [Nm]
I_G = 1 # load axis inertia [kg*m^2]

setpoint = 4
Kp = 3# Proportional gain
Kd = 0.15 # Derivative gain
Ki = 0.15 #Integral gain

# --- Sim inputs --- #
dt 	= 0.05 # timestep
t_final = 10
x_0 = np.array([0,0])
t = np.arange(0, t_final, dt)

#Some options for the reference signal
#R_func = lambda time: setpoint + 0*time #Constant setpoint
#R_func = lambda time :np.maximum(np.minimum(triangle_wave(time/8)*2*setpoint, setpoint), 1) #Weird triangular wave
R_func = lambda time: ramp(time, 0, 3, setpoint) - ramp(time-5, 5, 8, setpoint) #Ramp 

R = R_func(t)
RI = np.zeros(t.shape)
RD = np.zeros(t.shape)
for i, t1 in enumerate(t):
	RI[i] = integrate.quad(R_func, 0, t1)[0]
	RD[i] = misc.derivative(R_func, t1, dx = dt/2)
# --- State space matrices --- #
#State [x0, x1] = [position, velocity]
A = np.matrix([	[0.,	1.	],
		[0., -T_S/(w_n*I_G)]])
		
B = np.matrix([	[0.	],
		[T_S/I_G]])

#Output [y0, y1, y2] = [position, velocity, acc]
C = np.matrix([	[1., 	0.	],
		[0., 	1.	],
		[0., -T_S/(w_n*I_G)]])

D = np.matrix([	[0.	],
		[0,	],
		[T_S/I_G]])

#Control position
#K = np.matrix([Kp, Kd, 0])

#Control velocity
K = np.matrix([Kd, Kp, Ki])
		
# --- Simulation --- #

def ss_mod(x, u):
	return np.matmul(A, x) + np.matmul(B, [u])
	
def y_out(x, u):
	return np.matmul(C, x) + np.matmul(D, [u])


	
def be(df_dx, x_0, u, t, dt):
	xx = np.zeros((2, t.shape[0]))
	uu = np.zeros((t.shape[0]))
	yy = np.zeros((3, t.shape[0]))
	xx[:, 0] = x_0
	uu[0] = 0
	yy[:, 0] = y_out(x_0, uu[0])
	for i in range(len(t)-1):
		uu[i+1] = max(min(np.matmul(K, [RD[i],R[i], RI[i]]-yy[:, i]), 1),-1)
		
		f_solve = lambda y: y - dt * df_dx(y, uu[i+1]) - xx[:, i]
		x_next = opt.root(f_solve, xx[:, i], method="broyden1")
		xx[:, i+1] = x_next.x
		yy[:, i+1] = y_out(x_next.x, uu[i+1])
	return xx, uu	

xx_1, uu_1 =  be(ss_mod, x_0, R, t, dt)
yy = y_out(xx_1, uu_1)
fig, (ax1, ax2, ax3) = plt.subplots(3)
ax1.plot(t, uu_1*5, label="Duty cycle")
ax1.plot(t, R, label="Reference")

ax3.plot(t, yy[0, :].T, label="Angular pos")
ax1.plot(t, yy[1, :].T, label="Angular vel")
ax2.plot(t, yy[2, :].T, label="Angular acc")


ax3.plot(t, RI, label="RI")
ax2.plot(t, RD, label="RD")
ax1.legend(loc="lower right")
ax2.legend()
ax3.legend()
plt.show()





		
		
		
		
		
		
		

