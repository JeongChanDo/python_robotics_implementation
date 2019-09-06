import numpy as np
import matplotlib.pyplot as plt


mean = 0
std = 1
variance = std^2
x = np.arange(-10,50,0.1)
f = np.exp(-np.square(x-mean)/2*variance)/(np.sqrt(2*np.pi*variance))

plt.plot(x,f,label ="p(x0)")
plt.ylabel("gaussian distribution")
plt.legend()
plt.show()