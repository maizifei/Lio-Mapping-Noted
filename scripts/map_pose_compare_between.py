import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

def readPoseFromFile(filename):
    file = open(filename)
    lines = file.readlines()
    length = len(lines)
    Pos = np.zeros((length,6))
    index = 0
    for line in lines:
        line = line.strip()
        lineData = line.split( )  #以空格为分隔符，对每行字符串进行分隔
        Pos[index,:] = lineData[2:5] + lineData[8:11]
        index += 1
    return Pos

#从文件读取位置数据
#文件格式[序号，帧序号，时间，local_odom.x，local_odom.y，local_odom.z，local_odom.eulerx，local_odom.eulery，local_odom.eulerz，...
#                        mapped_odom.x，mapped_odom.y，mapped_odom.z，mapped_odom.eulerx，mapped_odom.eulery，mapped_odom.eulerz]
filename1 = "./test1/local_mapped_odom4.txt"
filename2 = "./test1/local_mapped_odom5.txt"
filename3 = "./test1/local_mapped_odom6.txt"

Pos_map1 = readPoseFromFile(filename1)
Pos_map2 = readPoseFromFile(filename2)
Pos_map3 = readPoseFromFile(filename3)

#画图
mpl.rcParams['legend.fontsize'] = 10

fig1 = plt.figure()
ax1 = fig1.gca(projection='3d')
fig2 = plt.figure()
ax2 = fig2.gca(projection='3d')

ax1.plot(Pos_map1[:,0], Pos_map1[:,1], Pos_map1[:,2], label='odom_trajectory1')
ax1.plot(Pos_map2[:,0], Pos_map2[:,1], Pos_map2[:,2], label='odom_trajectory2')
ax1.plot(Pos_map3[:,0], Pos_map3[:,1], Pos_map3[:,2], label='odom_trajectory3')
ax1.legend()
ax2.plot(Pos_map1[:,3], Pos_map1[:,4], Pos_map1[:,5], label='map_trajectory1')
ax2.plot(Pos_map2[:,3], Pos_map2[:,4], Pos_map2[:,5], label='map_trajectory2')
ax2.plot(Pos_map3[:,3], Pos_map3[:,4], Pos_map3[:,5], label='map_trajectory3')
ax2.legend()


plt.show()