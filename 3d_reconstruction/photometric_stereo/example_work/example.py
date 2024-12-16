## This Example has been picked up from, 
## https://github.com/visiont3lab/photometric_stereo.git.

from photostereo import photometry
import cv2 as cv
import time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

IMAGES = 12
root_fold = "experiments/3d_reconstruction/photometric_stereo/example_work/samples/buddha12/buddha12/"
obj_name = "buddha."
format = ".bmp"
light_manual = False

def plot_normal_map_3d(normal_map, mask):
    """
    Visualize the normal map as a 3D interactive plot.
    
    Parameters:
        - normal_map: Normal map (h x w x 3), where each pixel contains (nx, ny, nz).
        - mask: Mask (h x w) defining the region of interest.
    """
    # Get the dimensions
    h, w, _ = normal_map.shape
    
    # Normalize normal map to range [-1, 1] for 3D plotting
    normals = normal_map / 127.5 - 1.0  # Scale from [0, 255] to [-1, 1]
    nx, ny, nz = normals[:, :, 0], normals[:, :, 1], normals[:, :, 2]

    # Generate grid for plotting
    X, Y = np.meshgrid(np.arange(w), np.arange(h))

    # Mask out invalid regions
    nx[mask == 0] = 0
    ny[mask == 0] = 0
    nz[mask == 0] = 0

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(X, Y, np.zeros_like(X), nx, ny, nz, length=0.1, normalize=True, color="blue")
    ax.set_title("Interactive Normal Map Visualization")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Normals")
    plt.show()

#Load input image array
image_array = []
for id in range(0, IMAGES):
    try:
        filename = root_fold + str(obj_name) + str(id) + format
        im = cv.imread(root_fold + str(obj_name) + str(id) + format, cv.IMREAD_GRAYSCALE)
        image_array.append(im)
    except cv.error as err:
        print(err)

myps = photometry(IMAGES, True) # params: num_images, display (bool)

if light_manual:
    # SETTING LIGHTS MANUALLY
    #tilts = [136.571, 52.4733, -40.6776, -132.559]
    #slants = [52.6705, 53.2075, 47.3992, 48.8037]
    #slants = [37.3295, 36.7925, 42.6008, 41.1963]

    #tilts = [139.358, 50.7158, -42.5016, -132.627]
    #slants = [74.3072, 70.0977, 69.9063, 69.4498]
    #tilts = [0, 270, 180, 90]
    #slants = [45, 45, 45, 45]

    slants = [71.4281, 66.8673, 67.3586, 67.7405]
    tilts = [140.847, 47.2986, -42.1108, -132.558]

    slants = [42.9871, 49.5684, 45.9698, 43.4908]
    tilts = [-137.258, 140.542, 44.8952, -48.3291]

    myps.setlmfromts(tilts, slants)
    print(myps.settsfromlm())
else:
    # LOADING LIGHTS FROM FILE
    fs = cv.FileStorage(root_fold + "LightMatrix.yml", cv.FILE_STORAGE_READ)
    fn = fs.getNode("Lights")
    light_mat = fn.mat()
    myps.setlightmat(light_mat)
    #print(myps.settsfromlm())

tic = time.process_time()
# The mask provided gives the region of interest (ROI). This mask is used to exclude regions without valid data.
mask = cv.imread(root_fold + "mask" + format, cv.IMREAD_GRAYSCALE)
normal_map = myps.runphotometry(image_array, np.asarray(mask, dtype=np.uint8))
normal_map = cv.normalize(normal_map, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC3)
# Normalizes the normal map to the range [0, 255] for visualization or further processing.
# Converts the data to an 8-bit, 3-channel (RGB) format.
albedo = myps.getalbedo()
albedo = cv.normalize(albedo, None, 0, 255, cv.NORM_MINMAX, cv.CV_8UC1)
# Normalizes the albedo map to the range [0, 255] and converts it to an 8-bit, single-channel (grayscale) format.
#gauss = myps.computegaussian()
#med = myps.computemedian()

#cv.imwrite('normal_map.png',normal_map)
#cv.imwrite('albedo.png',albedo)
#cv.imwrite('gauss.png',gauss)
#cv.imwrite('med.png',med)

toc = time.process_time()
print("Process duration: " + str(toc - tic))

# TEST: 3d reconstruction
myps.computedepthmap()
# myps.computedepth2()
# myps.display3dobj()
# cv.imshow("normal", normal_map)
plot_normal_map_3d(normal_map, mask)
#cv.imshow("mean", med)
#cv.imshow("gauss", gauss)
cv.waitKey(0)
cv.destroyAllWindows()