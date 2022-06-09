import numpy as np
import math
import matplotlib.pyplot as plt


def step_func(x, x_offset):
	return (x>=x_offset)*(x-x_offset)
	

def cubic_splines(func, grid, bc=[0,0]):
	n = np.shape(grid)[0]
	M = np.zeros((n+2,(n+2)))
	M[0,2] = M[-1,2] =2.
	M[-1,3:] = 6*step_func(grid[-1], grid[0:-1])
	M[1:-1,:3] = np.array([np.ones(n), grid, grid**2]).T
	M[1:-1,3:] = step_func(np.full((n-1,n),grid).T, grid[:-1])**3
	f = np.zeros(n+2)
	f[0] = bc[0]
	f[-1] = bc[1]
	f[1:-1] = func(grid)
	
	a = np.matmul(np.linalg.inv(M), f)
	
	return lambda x: np.dot([1, x, x**2],a[0:3]) +np.dot(step_func(np.full((n-1,np.shape(x)[0]),x).T, grid[:-1])**3,a[3:])

	
f_sin = lambda x: np.sin(math.pi*x)
f_runge = lambda x: 1/(1+x**2)
f_kink = lambda x: np.abs(x)
f_step = lambda x: x > 0
f_exp = lambda x: math.e**x


function = f_sin
a = 0
b = 4
N = 4

grid = np.linspace(a,b,N)
f_approx = cubic_splines(function, grid)
xx = np.linspace(a,b,1000)
plot = True
if plot:
	fig, (ax1, ax2) = plt.subplots(2, sharex=True)
	ax1.plot(xx, function(xx))
	ax1.plot(xx, f_approx(xx))
	#ax1.plot(grid, function(grid) ,'ko')
	ax2.plot(xx, np.abs(f_approx(xx)-function(xx)))
	ax2.set_xlabel("x")
	ax2.set_ylabel("error")
	ax1.set_ylabel("f(x)")
	plt.show()


