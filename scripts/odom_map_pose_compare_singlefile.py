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
filename = "./test1/local_mapped_odom6.txt"

Pos_local_map = readPoseFromFile(filename)

#画图
mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.plot(Pos_local_map[:,0], Pos_local_map[:,1], Pos_local_map[:,2], label='odom_trajectory')
ax.plot(Pos_local_map[:,3], Pos_local_map[:,4], Pos_local_map[:,5], label='map_trajectory')
ax.legend()

plt.show()