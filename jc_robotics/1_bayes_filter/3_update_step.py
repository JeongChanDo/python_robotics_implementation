import numpy as np
import matplotlib.pyplot as plt

import scipy.stats as stats


mean = 0
variance = 1

min_range = -5
max_range =20
resolution = 0.1

x = np.arange(min_range,max_range,resolution)

x0 =  stats.norm.pdf(x,mean,variance)


motion = 5
motion_var = 1

x1predmean = mean+motion
x1predvar = variance+motion_var

x1pred =  stats.norm.pdf(x,x1predmean,x1predvar)



landmark = 10


z1mean = 6
measurement_var = 1
z1 =  stats.norm.pdf(x,z1mean,measurement_var)
"""
x1est = np.convolve(x1pred,z1)

tmp = np.array([])
for i in range(len(x1est)):
    if i%2 ==0:
        tmp = np.append(tmp,x1est[i])

x1est = tmp
"""

x1estmean = (x1predmean*measurement_var + z1mean*x1predvar)/(x1predvar+measurement_var)
x1estvar = 1/(1/measurement_var + 1/x1predvar)
x1est =  stats.norm.pdf(x,x1estmean, x1estvar)


plt.annotate("N~({:.1f},{:.3f})".format(mean,variance),xy=(mean,max(x0)))
plt.plot(x,x0,label ="p(x0)")

plt.annotate("N~({:.1f},{:.3f})".format(x1predmean,x1predvar),xy=(x1predmean,max(x1pred)))
plt.plot(x,x1pred,label ="p(x1|x0,u1)")

plt.annotate("N~({:.1f},{:.3f})".format(z1mean,measurement_var),xy=(z1mean,max(z1)))
plt.plot(x,z1,label ="p(z1|x1)")

plt.annotate("N~({:.1f},{:.3f})".format(x1estmean,x1estvar),xy=(x1estmean,max(x1est)))
#plt.annotate("N~({},{})".format(x1estmean,np.var(x1est)),xy=(x1estmean,max(x1est)))
plt.plot(x,x1est,label ="p(x1|z1,u1)")

plt.ylabel("gaussian distribution")
plt.legend()
plt.show()