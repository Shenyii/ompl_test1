from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path.dat')
data1 = numpy.loadtxt('path0.dat')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,0],data[:,1],data[:,2],'.-')
plt.hold('on')
plt.grid('on')
ax.plot(data1[:,0],data1[:,1],data1[:,2],'r-')
plt.show()
