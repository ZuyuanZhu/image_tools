import os
import cv2
import numpy as np
from glob import glob

# ORB parameters
n_features = 2000
scale_factor = 1.4
n_levels = 8
ini_th_FAST = 35
min_th_FAST = 15

# KITTI dataset path
dataset_path = '/media/zuyuan/DATA1TB/kitti/2011_10_03/'

# Create ORB detector and descriptor extractor
# orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=8)
orb = cv2.ORB_create(nfeatures=n_features, scaleFactor=scale_factor, nlevels=n_levels,
                     edgeThreshold=ini_th_FAST, patchSize=min_th_FAST)

# Find all image paths in dataset path
image_paths = sorted(glob(os.path.join(dataset_path, '*/image_00/data/*.png')))

# Create BOWKMeansTrainer with k=1000
bow_trainer = cv2.BOWKMeansTrainer(1000)

# Loop over all image paths and extract features
for image_path in image_paths:
    # Load image
    image = cv2.imread(image_path)
    image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_AREA)
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Extract keypoints and descriptors
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    # print(descriptors.shape, descriptors.dtype)  # Debugging line
    # Convert descriptors to float32 data type
    descriptors_float = np.float32(descriptors)
    # Add descriptors to trainer
    bow_trainer.add(descriptors_float)

# Perform k-means clustering to generate vocabulary
vocabulary = bow_trainer.cluster()

# Create directory to save vocabulary
vocab_path = '/media/zuyuan/DATA1TB/kitti/ORBvocab'
if not os.path.exists(vocab_path):
    os.makedirs(vocab_path)

# Save vocabulary as numpy array
np.save(os.path.join(vocab_path, 'orb_vocab.npy'), vocabulary)

# Save vocabulary as text file
with open(os.path.join(vocab_path, 'orb_vocab.txt'), 'w') as f:
    for word in vocabulary:
        f.write(' '.join(map(str, word)) + '\n')

print('Vocabulary saved successfully.')
