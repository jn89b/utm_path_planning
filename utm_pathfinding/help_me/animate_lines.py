import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.style.use('dark_background')


NumOfPoints = 6
deltaT = 0.005
duration = 50
steps = int(duration / deltaT)
speed = 0.2
num = 0

CurrentXPoints = []
CurrentYPoints = []
DeltaX = np.zeros(NumOfPoints)
DeltaY = np.zeros(NumOfPoints)
MagnitudeDelta = np.zeros(NumOfPoints)
VelocityX = np.zeros(NumOfPoints)
VelocityY = np.zeros(NumOfPoints)


def update(num, AllXPoints, AllYPoints, MainLines, SightLines):
    out = []
    out.append(UpdateMain(num, AllXPoints, AllYPoints, MainLines))
    out.append(UpdateSight(num, AllXPoints, AllYPoints, SightLines))
    return out
 
def UpdateMain(num, AllXPoints, AllYPoints, MainLines):    
    for line in MainLines:
        position = MainLines.index(line)
        line.set_data([AllXPoints[i][position] for i in range(num)], [AllYPoints[i][position] for i in range(num)])
    return MainLines
        
    
    
def UpdateSight(num, AllXPoints, AllYPoints, SightLines): 
    for line in SightLines:
        position = SightLines.index(line)

        if position < (NumOfPoints-1):
            line.set_data([AllXPoints[num][position],AllXPoints[num][position+1]],
                         [AllYPoints[num][position],AllYPoints[num][position+1]])
        else:
            line.set_data([AllXPoints[num][position],AllXPoints[num][0]],
                         [AllYPoints[num][position],AllYPoints[num][0]])
    return SightLines

  
#Creates Initial Points by equally spacing the points around a polygon inscribed around circle
for i in range(0,NumOfPoints): 
    x = np.cos(((i/NumOfPoints)*2)*np.pi)
    y = np.sin(((i/NumOfPoints)*2)*np.pi)

    CurrentXPoints.append(x)
    CurrentYPoints.append(y)

AllXPoints = np.array([CurrentXPoints])
AllYPoints = np.array([CurrentYPoints])

#Fills out both AllXPoints and AllYPoints with all points in duration 
for i in range(int(steps)):
    for j in range(0,NumOfPoints-1): #Calculates deltaX and deltaY at this timestep
        DeltaX[j] = CurrentXPoints[j+1] - CurrentXPoints[j]
        DeltaY[j] = CurrentYPoints[j+1] - CurrentYPoints[j]

    DeltaX[NumOfPoints-1] = CurrentXPoints[0] - CurrentXPoints[NumOfPoints-1]
    DeltaY[NumOfPoints-1] = CurrentYPoints[0] - CurrentYPoints[NumOfPoints-1]

    for j in range(0,NumOfPoints): # calculats new X and Y Points
        MagnitudeDelta[j] = ((DeltaX[j])**2 + (DeltaY[j])**2)**(1/2)
        VelocityX[j] = speed * (DeltaX[j]/MagnitudeDelta[j])
        VelocityY[j] = speed * (DeltaY[j]/MagnitudeDelta[j])
        CurrentXPoints[j] += deltaT * VelocityX[j]
        CurrentYPoints[j] += deltaT * VelocityY[j]

    CurrentXPointsArr = np.array(CurrentXPoints)
    CurrentYPointsArr = np.array(CurrentYPoints)
    AllXPoints = np.vstack((AllXPoints,CurrentXPointsArr))
    AllYPoints = np.vstack((AllYPoints,CurrentYPointsArr))

fig = plt.figure(figsize=(5,5))
ax = plt.axes()
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
ax.get_xaxis().set_visible(False)
ax.get_yaxis().set_visible(False)

MainLines = []
SightLines= []
AllLines = MainLines + SightLines

for i in range(NumOfPoints):
    line1, = ax.plot([AllXPoints[j][i] for j in range(steps)], [AllYPoints[j][i] for j in range(steps)])
    line2, = ax.plot([AllXPoints[j][i] for j in range(steps)], [AllYPoints[j][i] for j in range(steps)])
    MainLines.append(line1)
    SightLines.append(line2)

ani = animation.FuncAnimation(fig, update, steps, fargs=[AllXPoints, AllYPoints, MainLines, SightLines], interval=1, blit=True)

plt.show()