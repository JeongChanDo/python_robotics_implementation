import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats


mean = 0
variance = 1
x = np.arange(-5,20,0.1)
x0 = stats.norm.pdf(x,mean,variance)




motion = 1
motion_var = 2

x1predmean = mean+motion
x1predvar = variance+motion_var

x1pred = stats.norm.pdf(x,x1predmean,x1predvar)


plt.annotate("N~({},{})".format(mean,variance),xy=(mean,max(x0)))
plt.plot(x,x0,label ="p(x0)")
plt.annotate("N~({},{})".format(x1predmean,x1predvar),xy=(x1predmean,max(x1pred)))
plt.plot(x,x1pred,label ="p(x1|x0,u1)")

plt.ylabel("gaussian distribution")
plt.legend()
plt.show()