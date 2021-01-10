from matplotlib import pyplot as plt
 
 
filename_px4Pose = '/home/nvidia/work/dataset/saveFile/'+ 'px4Pose-2020-12-19-17-11-48'+'.txt'
px4Pose = [[],[],[],[]]
with open(filename_px4Pose, 'r') as f:
    lines = f.readlines()
    for line in lines:
        temp = line.split('\t')
        px4Pose[0].append(float(temp[0]))
        px4Pose[1].append(float(temp[1]))
        px4Pose[2].append(float(temp[2]))
        px4Pose[3].append(float(temp[3]))
        
# filename_t265Pose = '/home/linux/work/dataset/saveFile/'+ 't265Pose-2020-12-10-09-54-33'+'.txt'
# t265Pose = [[],[],[],[]]
# with open(filename_t265Pose, 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#         temp = line.split('\t')
#         t265Pose[0].append(float(temp[0]))
#         t265Pose[1].append(float(temp[1]))
#         t265Pose[2].append(float(temp[2]))
#         t265Pose[3].append(float(temp[3]))

# filename_vrpnPose = '/home/linux/work/dataset/saveFile/'+ 'vrpnPose-2020-12-10-09-54-33'+'.txt'
# vrpnPose = [[],[],[],[]]
# with open(filename_vrpnPose, 'r') as f:
#     lines = f.readlines()
#     for line in lines:
#         temp = line.split('\t')
#         vrpnPose[0].append(float(temp[0]))
#         vrpnPose[1].append(float(temp[1]))
#         vrpnPose[2].append(float(temp[2]))
#         vrpnPose[3].append(float(temp[3]))


fig = plt.figure(num=1) 
ax1 = fig.add_subplot(111)
ax1.plot(px4Pose[0], px4Pose[1], 'r', label='x')  
ax1.plot(px4Pose[0], px4Pose[2], 'b', label='y')
ax1.plot(px4Pose[0], px4Pose[3], 'g', label='z')
ax1.legend(loc='upper right')  
ax1.set_xlabel('time') 
ax1.set_ylabel('px4Pose') 

fig = plt.figure(num=2) 
ax2 = plt.axes(projection='3d')
ax2.plot3D(px4Pose[1], px4Pose[1], px4Pose[3],'r', label='xyz') 
ax2.set_xlabel('x') 
ax2.set_ylabel('y') 
ax2.set_zlabel('z') 

plt.show() 
