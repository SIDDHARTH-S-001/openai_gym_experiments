import cv2
import glob
import os
import numpy as np
import re
import yaml
import matplotlib.pyplot as plt

def numerical_sort(value):
    """
    Helper function to sort files numerically by their filenames.
    """
    numbers = re.compile(r'(\d+)')
    parts = numbers.split(value)
    parts[1::2] = map(int, parts[1::2])
    return parts

def load_light_matrix(yaml_file):
    """
    Loads the LightMatrix.yml file using OpenCV's FileStorage.
    """
    fs = cv2.FileStorage(yaml_file, cv2.FILE_STORAGE_READ)
    light_matrix = fs.getNode("Lights").mat()
    fs.release()
    return light_matrix


# Dataset processing
root_folder = "D:/Onedrive/experiments/experiments/3d_reconstruction/photometric_stereo/example_work/samples/shrek/shrek"  # Replace with the actual path to the 'samples' folder

# Initialize storage for visualization
images, img_albedo, img_normal_rgb = None, None, None

print(f"Processing root folder: {root_folder}")

# Load light directions from YAML
# Corrected YAML file loader
yaml_path = os.path.join(root_folder, "LightMatrix.yml")  # Ensure the file extension is correct
if not os.path.exists(yaml_path):
    print(f"LightMatrix.yml not found in {root_folder}. Exiting.")
    exit()
light_directions = load_light_matrix(yaml_path)

# Filter images, excluding 'mask.bmp' and 'all.bmp'
img_names = [f for f in glob.glob(os.path.join(root_folder, "*.bmp")) if not (f.endswith("mask.bmp") or f.endswith("all.bmp"))]
img_names = sorted(img_names, key=numerical_sort)[:3]  # Pick at most 3 images

print(img_names)

if len(img_names) < 3:
    print(f"Not enough valid images found in {root_folder}. Found {len(img_names)} valid images. Exiting.")
    exit()

# Load images
images = [cv2.imread(img, cv2.IMREAD_GRAYSCALE) for img in img_names]
images = np.array(images)

# Load mask
mask_path = os.path.join(root_folder, "mask.bmp")
mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
if mask is None:
    print(f"Mask file not found in {root_folder}. Exiting.")
    exit()

# Apply mask to images
for i in range(len(images)):
    images[i] = cv2.bitwise_and(images[i], images[i], mask=mask)

print(f"Loaded {len(images)} valid images and light directions from the root folder.")

# Display results
if images is not None:
    print("Images and LightMatrix successfully loaded.")
    plt.figure(figsize=(10, 5))
    plt.subplot(1, len(images), 1)
    plt.imshow(images[0], cmap='gray')
    plt.title("Example Image 1")
    plt.xticks([])
    plt.yticks([])

    if len(images) > 1:
        plt.subplot(1, len(images), 2)
        plt.imshow(images[1], cmap='gray')
        plt.title("Example Image 2")
        plt.xticks([])
        plt.yticks([])

    if len(images) > 2:
        plt.subplot(1, len(images), 3)
        plt.imshow(images[2], cmap='gray')
        plt.title("Example Image 3")
        plt.xticks([])
        plt.yticks([])

    plt.show()
else:
    print("No valid data to display. Check dataset structure or paths.")
