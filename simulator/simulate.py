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
t = np.arange(0, t_final+dt, dt)

#Some options for the reference signal
#R_func = lambda time: setpoint + 0*time #Constant setpoint
#R_func = lambda time :np.maximum(np.minimum(triangle_wave(time/8)*2*setpoint, setpoint), 1) #Weird triangular wave
#R_func = lambda time: ramp(time, 0, 3, setpoint) - ramp(time-5, 5, 8, setpoint) #Ramp 
v_L =np.array([0.0275,0.0267,0.0262,0.0259,0.0259,0.0261,0.0264,0.0267,0.027 ,0.0273,
 0.0278,0.0287,0.0299,0.0314,0.0332,0.0353,0.0377,0.0403,0.043 ,0.046 ,
 0.0492,0.0525,0.056 ,0.0596,0.0634,0.0674,0.0715,0.0758,0.0802,0.0847,
 0.0894,0.0942,0.0992,0.1043,0.1096,0.115 ,0.1205,0.1262,0.132 ,0.138 ,
 0.1441,0.1504,0.1568,0.1634,0.17  ,0.1769,0.1839,0.191 ,0.1983,0.2057,
 0.2133,0.221 ,0.2288,0.2368,0.245 ,0.2533,0.2617,0.2703,0.2791,0.2879,
 0.297 ,0.3061,0.3155,0.3249,0.3346,0.3443,0.3542,0.3643,0.3745,0.3849,
 0.3954,0.406 ,0.4168,0.4278,0.4389,0.4501,0.4615,0.4731,0.4848,0.4966,
 0.5086,0.5207,0.533 ,0.5454,0.558 ,0.5707,0.5836,0.5967,0.6098,0.6232,
 0.6366,0.6503,0.664 ,0.678 ,0.692 ,0.7062,0.7206,0.7351,0.7498,0.7646,
 0.7796,0.7947,0.81  ,0.8254,0.8409,0.8567,0.8725,0.8885,0.9047,0.921 ,
 0.9375,0.9541,0.9708,0.9877,1.0048,1.022 ,1.0394,1.0569,1.0745,1.0923,
 1.1103,1.1284,1.1466,1.165 ,1.1836,1.2023,1.2211,1.2401,1.2593,1.2786,
 1.298 ,1.3176,1.3374,1.3573,1.3773,1.3975,1.4179,1.4384,1.459 ,1.4798,
 1.5008,1.5219,1.5431,1.5645,1.5861,1.6078,1.6296,1.6516,1.6737,1.696 ,
 1.7185,1.7411,1.7638,1.7867,1.8097,1.8329,1.8563,1.8798,1.9034,1.9272,
 1.9511,1.9752,1.9995,2.0239,2.0484,2.0731,2.0979,2.1229,2.1481,2.1734,
 2.1988,2.2244,2.2501,2.276 ,2.3021,2.3282,2.3546,2.3811,2.4077,2.4345,
 2.4614,2.4885,2.5158,2.5432,2.5707,2.5984,2.6262,2.6542,2.6823,2.7106,
 2.7391,2.7676,2.7964,2.8253,2.8543,2.8835,2.9128,2.9423,2.972 ,3.0017,
 3.0317])



R = v_L
RI = np.zeros(t.shape)
RD = np.zeros(t.shape)
for i, t1 in enumerate(t):
	RI[i] = integrate.simpson(R[t<=t1], dx=dt)
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

def rk4(df_dx, x_0, u, t, dt):
	xx = np.zeros((2, t.shape[0]))
	uu = np.zeros((t.shape[0]))
	yy = np.zeros((3, t.shape[0]))
	xx[:, 0] = x_0
	uu[0] = 0
	yy[:, 0] = y_out(x_0, uu[0])
	for i in range(len(t)-1):
		uu[i+1] = max(min(np.matmul(K, [RD[i],R[i], RI[i]]-yy[:, i]), 1),-1)
		
		
		
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
print("final pos error: {}".format(abs(RI[-1]-yy[0, -1])))

plt.show()





		
		
		
		
		
		
		

