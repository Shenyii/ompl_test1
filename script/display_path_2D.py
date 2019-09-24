from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path.dat')
fig = plt.figure()
#ax = fig.fca(projection='3d')
plt.plot(data[:,0],data[:,1],'.')
#plt.plot(numpy.arange(0,1,0.01), data[:,0], '.')
plt.hold('on')
plt.grid('on')
plt.show()
