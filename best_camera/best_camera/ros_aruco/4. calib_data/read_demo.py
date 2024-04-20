import numpy as np 

loaded_arrays = np.load("/home/kkyu/Best_camera/src/best_camera/best_camera/ros_aruco/4. calib_data/MultiMatrix.npz")
print(loaded_arrays.files)
print(loaded_arrays["camMatrix"])