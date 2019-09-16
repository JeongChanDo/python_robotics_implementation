import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

#our 2-dimensional Distribution will be over variables X and Y

N = 60

X = np.linspace(-3,3,N)
Y = np.linspace(-3,4,N)
X, Y = np.meshgrid(X,Y)

#mean vector and cov mat
mu = np.array([0,1])
sigma = np.array([[1,-0.5],[-0.5,1.5]])

#pack x and y into a single 3-dimensonal array
pos = np.empty(X.shape+(2,))
pos[:,:,0] = X
pos[:,:,1] = Y

def multivariate_gaussian(pos,mu,sigma):
    """
    return the multivariate gaussian distribution on array pos.
    """

    n = mu.shape[0]
    sigma_det = np.linalg.det(sigma)
    sigma_inv = np.linalg.inv(sigma)
    N = np.sqrt((2*np.pi)**n*sigma_det)

    #this einsum call calculates (x-mu)^.sigma-1.(x-mu)
    
    print("pos.shape : ", pos.shape)
    print("mu.shpae : ",mu.shape)
    fac = np.einsum('...k,kl,...l->...',pos-mu,sigma_inv,pos-mu)
    return np.exp(-fac/2)/N

#the distribution on the variable X, Y packed into pos

Z = multivariate_gaussian(pos,mu,sigma)

#create a surface plot and projected filled contour plot under it
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(X,Y,Z,rstride=3,cstride=3,linewidth=1, antialiased = True,
    cmap=cm.viridis)

#cset = ax.contourf(X, Y, Z, zdir='z', offset= -0.15,cmap=cm.viridis)

#Adjust the limits, ticks and view angle
ax.set_zlim(-0.15,0.2)
ax.set_zticks(np.linspace(0,0.2,5))
ax.view_init(27,-21)
plt.show()