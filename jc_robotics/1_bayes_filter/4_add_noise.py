import numpy as np
import matplotlib.pyplot as plt
import random

import scipy.stats as stats


mean = 0
variance = 1

min_range = -5
max_range =20
resolution = 0.1

x = np.arange(min_range,max_range,resolution)

x0 =  stats.norm.pdf(x,mean,variance)


motion = 5
motion_noise = 1

z1mean = 6
measurement_var = 1

x1predmean = mean+ motion_noise*np.random.randn()

x1predvar = variance+motion_var
x1pred =  stats.norm.pdf(x,x1predmean,x1predvar)

plt.annotate("N~({:.1f},{:.3f})".format(mean,variance),xy=(mean,max(x0)))
plt.plot(x,x0,label ="p(x0)")

plt.annotate("N~({:.1f},{:.3f})".format(x1predmean,x1predvar),xy=(x1predmean,max(x1pred)))
plt.plot(x,x1pred,label ="p(x1|x0,u1)")

plt.ylabel("gaussian distribution")
plt.legend()
plt.show()