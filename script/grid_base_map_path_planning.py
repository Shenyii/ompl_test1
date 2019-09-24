from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('path.dat')
data1= numpy.loadtxt('22.txt')
#data1= numpy.loadtxt('obstacle.dat')
 
x=[]
y=[]
fig = plt.figure()
for i in range(15):
        for j in range(15):
                if (data1[i][j]==1):
                        x.append(i)
                        y.append(j)
                        
                
 
#ax = fig.gca(projection='3d')
plt.plot(data[:,0],data[:,1],'.-')
plt.hold('on')
plt.grid('on')
 
plt.scatter(x,y)
plt.hold('on')
plt.grid('on')
plt.show()
