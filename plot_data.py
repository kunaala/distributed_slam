from math import floor
import numpy as np
import matplotlib.pyplot as plt

mu = 1.5
def convert_rgb(sdf_val):
    return max(-mu,min((sdf_val*255)/(2*mu),mu))

with open("21_map_ros.txt",'r') as f:
# with open("best_maps/cumulated/45_map_ros.txt",'r') as f:
# with open("best_maps/cumulated/66_map_ros.txt",'r') as f:
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
sdf_vals = np.fromstring(lines[2],dtype=np.float64,sep = " ")
odom_x = np.fromstring(lines[3],dtype=np.float64,sep = " ")
odom_y = np.fromstring(lines[4],dtype=np.float64,sep = " ")
map_size = 3000
map_res = 0.1
#to avoid discontinuities in map
scale_factor = 1/map_res
map = np.full((map_size,map_size),mu+1)
off_x = map_size//2
off_y = map_size//2
#updating sdf values in map
for i in range(loc_x.shape[0]):
    map[int(-loc_y[i]*scale_factor) + off_y, int(loc_x[i]*scale_factor) + off_x] = (mu+1) if (sdf_vals[i] == (mu+1)) else convert_rgb(sdf_vals[i]-mu-1)

plt.figure(figsize=(16,8))
plt.imshow(map,cmap = "hot")
plt.colorbar()

#updating odom values in map
plt_odom_x = odom_x*scale_factor + off_x 
plt_odom_y = odom_y*scale_factor + off_y
plt.scatter(plt_odom_x, plt_odom_y,s=50,c='blue',marker = '+')
plt.title('prediction plot')
plt.show()
# plt.savefig("predictive.png", bbox_inches='tight', dpi=300)

