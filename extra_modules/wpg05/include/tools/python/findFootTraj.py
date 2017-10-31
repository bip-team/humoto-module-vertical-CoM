import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

def polynomial(coeff, x):
  res = coeff[0]*x**4 + coeff[1]*x**3 + coeff[2]*x**2 + coeff[3]*x + coeff[4]
  return res

T = 0.8
M = np.array([[0, 0, 0, 0, 1],[0, 0, 0, 1, 0],[T**4,T**3,T**2,T**1,T**0],[4*T**3, 3*T**2, 2*T, 1, 0],[(T/2)**4,(T/2)**3,(T/2)**2, T/2, 1]])
invM = np.linalg.inv(M)
#print "M:", M
#print "invM:", invM
res = invM.dot(M)
#print "invM*M", res
X0 = np.array([0,0,0])
dX0 = np.array([0,0,0])
X1 = np.array([1,0.5,0.2])
dX1 = np.array([0,0,0])
h = 0.4
top = np.array([0,0,h]) + (X1+X0)/2.0
f = [[],[],[]]

for i in range(0,3):
    #print "i:", i
    b = np.array([X0[i],dX0[i],X1[i],dX1[i],top[i]]).transpose()
    #print "B:", b
    f[i] = np.linalg.solve(M,b)
    #print "f: ", f
    xinv = invM.dot(b)
    #print "xInv: ",xinv

tRange = np.arange(0,T,0.01)

x = np.array([])
y = np.array([])
z = np.array([])

i = 0
for ti in tRange:
  x = np.append(x, polynomial(f[0], ti))
  y = np.append(y, polynomial(f[1], ti))
  z = np.append(z, polynomial(f[2], ti))
  i = i+1

#print "x:", x
#print "y:", y
#print "z:", z

plt.plot(tRange,x,'b',label='x')
plt.plot(tRange,y,'r',label='y')
plt.plot(tRange,z,'g',label='z')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(x, y, z, label='parametric curve')
ax.legend()

plt.show()
