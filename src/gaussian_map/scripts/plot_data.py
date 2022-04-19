import numpy as np
import matplotlib.pyplot as plt

with open("map_ros4.txt",'r') as f:
    lines = f.readlines()

# x = np.fromstring(lines[0], dtype=float, sep = ",")
# y = np.fromstring(lines[1], dtype=float, sep = ",")
# xp = np.fromstring(lines[2], dtype=float, sep = ",")
# yp = np.zeros(xp.shape)
# x_t = np.fromstring(lines[3], dtype=float, sep = ",")
# # y_t = np.fromstring(lines[4], dtype=float, sep = ",")
# mu_t = np.fromstring(lines[4], dtype=float, sep = ",")
# s = np.fromstring(lines[5], dtype=float, sep = ",")

map = np.zeros((len(lines),len(lines)))
print(map.shape)
for i in range(map.shape[0]):
    map[i,:]= np.fromstring(lines[i],dtype=np.float64,sep = ",")
plt.figure(figsize=(16,8))

# plt.plot(x, y,'k+', ms=8)
# plt.plot(x_t, mu_t,'b-' , ms = 18)
# plt.plot(xp,yp,'r+', ms=14)
# plt.gca().fill_between(x_t, mu_t-2*s, mu_t+2*s, color="#dddddd")

# plt.savefig("predictive.png", bbox_inches='tight', dpi=300)
plt.imshow(map)
plt.title('pseudo plot')
plt.show()

