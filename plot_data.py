from math import floor
import numpy as np
import matplotlib.pyplot as plt

with open("10_map_ros.txt",'r') as f:
    lines = f.readlines()

# x = np.fromstring(lines[0], dtype=float, sep = ",")
# y = np.fromstring(lines[1], dtype=float, sep = ",")
# xp = np.fromstring(lines[2], dtype=float, sep = ",")
# yp = np.zeros(xp.shape)
# x_t = np.fromstring(lines[3], dtype=float, sep = ",")
# # y_t = np.fromstring(lines[4], dtype=float, sep = ",")
# mu_t = np.fromstring(lines[4], dtype=float, sep = ",")
# s = np.fromstring(lines[5], dtype=float, sep = ",")

loc_x = np.fromstring(lines[0],dtype=np.float64,sep = " ")
loc_y = np.fromstring(lines[1],dtype=np.float64,sep = " ")
map_res = 0.5
mu = 1.5
map_size = int(floor(loc_x.shape[0] ** 0.5)*map_res)*2
print(loc_y.shape[0],loc_x.shape[0], map_size)

map = np.full((map_size,map_size),-1)
off_x = map_size//2
off_y = map_size//2
sdf_vals = np.fromstring(lines[2],dtype=np.float64,sep = " ")
for i in range(loc_x.shape[0]):
    map[int(loc_x[i]) + off_x,int(loc_y[i]) + off_y] = (sdf_vals[i]*255)/(2*mu)
plt.figure(figsize=(16,8))
plt.imshow(map,interpolation='none',cmap="hot")
plt.colorbar()
plt.title('prediction plot')
plt.show()
# plt.savefig("predictive.png", bbox_inches='tight', dpi=300)

