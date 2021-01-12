from matplotlib import pyplot as plt

filename_px4Pose = '/home/linux/work/dataset/saveFile/'+ 'px4Pose-2021-01-08-17-57-00'+'.txt'
px4Pose = [[],[],[],[]]
with open(filename_px4Pose, 'r') as f:
    lines = f.readlines()
    linecnt = 0
    for line in lines:
        temp = line.split('\t')
        linecnt = linecnt + 1
        mystr="".join(temp[0])
        if not mystr[0].isdigit() :
            print("px4Pose-xxx.txt: %d line is not number" %(linecnt)) 
            continue
        px4Pose[0].append(float(temp[0]))
        px4Pose[1].append(float(temp[1]))
        px4Pose[2].append(float(temp[2]))
        px4Pose[3].append(float(temp[3]))
        
filename_t265Pose = '/home/linux/work/dataset/saveFile/'+ 't265Pose-2021-01-08-17-57-00'+'.txt'
t265Pose = [[],[],[],[]]
with open(filename_t265Pose, 'r') as f:
    lines = f.readlines()
    linecnt = 0
    for line in lines:
        temp = line.split('\t')
        linecnt = linecnt + 1
        mystr="".join(temp[0])
        if not mystr[0].isdigit() :
            print("t265Pose-xxx.txt: %d line is not number" %(linecnt)) 
            continue
        t265Pose[0].append(float(temp[0]))
        t265Pose[1].append(float(temp[1]))
        t265Pose[2].append(float(temp[2]))
        t265Pose[3].append(float(temp[3]))

filename_vrpnPose = '/home/linux/work/dataset/saveFile/'+'vrpnPose-2021-01-08-17-57-00'+'.txt'
vrpnPose = [[],[],[],[]]
with open(filename_vrpnPose, 'r') as f:
    lines = f.readlines()
    linecnt = 0
    for line in lines:
        temp = line.split('\t')
        linecnt = linecnt + 1
        mystr="".join(temp[0])
        if not mystr[0].isdigit() :
            print("vrpnPose-xxx.txt: %d line is not number" %(linecnt)) 
            continue
        vrpnPose[0].append(float(temp[0]))
        vrpnPose[1].append(float(temp[1]))
        vrpnPose[2].append(float(temp[2]))
        vrpnPose[3].append(float(temp[3]))

filename_targetPose = '/home/linux/work/dataset/saveFile/'+ 'targetPose-2021-01-08-17-57-00' +'.txt'
targetPose = [[],[],[],[]]
with open(filename_targetPose, 'r') as f:
    lines = f.readlines()
    linecnt = 0
    for line in lines:
        temp = line.split('\t')
        linecnt = linecnt + 1
        mystr="".join(temp[0])
        if not mystr[0].isdigit() :
            print("targetPose-xxx.txt: %d line is not number" %(linecnt)) 
            continue
        targetPose[0].append(float(temp[0]))
        targetPose[1].append(float(temp[1]))
        targetPose[2].append(float(temp[2]))
        targetPose[3].append(float(temp[3])+0.7)    

fig = plt.figure(num=1) 
ax1 = fig.add_subplot(111)
# ax1.plot(px4Pose[0], px4Pose[1], 'r', label='px4Pose')  
# ax1.plot(t265Pose[0], t265Pose[1], 'g', label='t265Pose') 
ax1.plot(vrpnPose[0], vrpnPose[1], 'orange', label='vrpnPose') 
# ax1.plot(targetPose[0], targetPose[1], 'b', label='targetPose') 
ax1.legend(loc='upper right')  
ax1.set_xlabel('time') 
ax1.set_ylabel('x') 

fig = plt.figure(num=2)
ax2 = fig.add_subplot(111)
# ax2.plot(px4Pose[0], px4Pose[2], 'r', label='px4Pose')  
# ax2.plot(t265Pose[0], t265Pose[2], 'g', label='t265Pose') 
ax2.plot(vrpnPose[0], vrpnPose[2], 'orange', label='vrpnPose') 
# ax2.plot(targetPose[0], targetPose[2], 'b', label='targetPose') 
ax2.legend(loc='upper right')  
ax2.set_xlabel('time') 
ax2.set_ylabel('y') 

fig = plt.figure(num=3)
ax3 = fig.add_subplot(111)
# ax3.plot(px4Pose[0], px4Pose[3], 'r', label='px4Pose')  
ax3.plot(t265Pose[0], t265Pose[3], 'g', label='t265Pose') 
ax3.plot(vrpnPose[0], vrpnPose[3], 'orange', label='vrpnPose') 
# ax3.plot(targetPose[0], targetPose[3], 'b', label='targetPose') 
ax3.legend(loc='upper right')  
ax3.set_xlabel('time') 
ax3.set_ylabel('z') 

# ****************************** 3D ********************************
fig = plt.figure(num=4) 
ax1_3d = plt.axes(projection='3d')
# ax1_3d.plot3D(px4Pose[1], px4Pose[2], px4Pose[3],'r', label='px4Pose') 
# ax1_3d.plot3D(t265Pose[1], t265Pose[2], t265Pose[3],'g', label='t265Pose') 
ax1_3d.plot3D(vrpnPose[1], vrpnPose[2], vrpnPose[3],'orange', label='vrpnPose') 
# ax1_3d.plot3D(targetPose[1], targetPose[2], targetPose[3],'b', label='targetPose')
ax1_3d.set_xlabel('x') 
ax1_3d.set_ylabel('y') 
ax1_3d.set_zlabel('z') 


# fig = plt.figure(num=5)
# ax5 = fig.add_subplot(111)
# ax5.plot(vrpnPose[0], vrpnPose[2], 'r', label='vrpnPose')  
# ax5.legend(loc='upper right')  
# ax5.set_xlabel('x') 
# ax5.set_ylabel('y') 

plt.show() 