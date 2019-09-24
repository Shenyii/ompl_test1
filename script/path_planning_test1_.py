from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path.dat')
data1 = numpy.loadtxt('obstacle.dat')
fig = plt.figure()
#ax = fig.fca(projection='3d')
plt.plot(data[:,0],data[:,1],'.-')
plt.hold('on')
plt.grid('on')
plt.fill(data1[:,0],data1[:,1],'.-')
plt.hold('on')
plt.grid('on')
plt.show()