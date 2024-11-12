import os
import tensorflow as tf
import numpy as np
import cv2
from tensorflow.keras.applications import VGG16
from tensorflow.keras.applications.vgg16 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from sklearn.svm import LinearSVC
from sklearn.multiclass import OneVsRestClassifier
from sklearn.preprocessing import LabelEncoder
from sklearn.linear_model import LinearRegression
from sklearn.metrics import average_precision_score
from tqdm import tqdm

# Suppress TensorFlow warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# Paths to the datasets
train_tfrecord = 'D:/Onedrive/experiments/experiments/Objectron/bounding_box_regression/apple_dataset/train/Apple.tfrecord'
train_pbtxt = 'D:/Onedrive/experiments/experiments/Objectron/bounding_box_regression/apple_dataset/train/Apple_label_map.pbtxt'
test_tfrecord = 'D:/Onedrive/experiments/experiments/Objectron/bounding_box_regression/apple_dataset/test/Apple.tfrecord'
test_pbtxt = 'D:/Onedrive/experiments/experiments/Objectron/bounding_box_regression/apple_dataset/test/Apple_label_map.pbtxt'
valid_tfrecord = 'D:/Onedrive/experiments/experiments/Objectron/bounding_box_regression/apple_dataset/valid/Apple.tfrecord'
valid_pbtxt = 'D:/Onedrive/experiments/experiments/Objectron/bounding_box_regression/apple_dataset/valid/Apple_label_map.pbtxt'

# Function to parse TFRecord files
def parse_tfrecord(example_proto):
    features = {
        'image/encoded': tf.io.FixedLenFeature([], tf.string),
        'image/object/bbox/xmin': tf.io.VarLenFeature(tf.float32),
        'image/object/bbox/xmax': tf.io.VarLenFeature(tf.float32),
        'image/object/bbox/ymin': tf.io.VarLenFeature(tf.float32),
        'image/object/bbox/ymax': tf.io.VarLenFeature(tf.float32),
        'image/object/class/text': tf.io.VarLenFeature(tf.string),
    }
    parsed_features = tf.io.parse_single_example(example_proto, features)
    image = tf.io.decode_jpeg(parsed_features['image/encoded'], channels=3)
    xmin = tf.sparse.to_dense(parsed_features['image/object/bbox/xmin'])
    xmax = tf.sparse.to_dense(parsed_features['image/object/bbox/xmax'])
    ymin = tf.sparse.to_dense(parsed_features['image/object/bbox/ymin'])
    ymax = tf.sparse.to_dense(parsed_features['image/object/bbox/ymax'])
    classes = tf.sparse.to_dense(parsed_features['image/object/class/text'], default_value='')
    return image, (xmin, xmax, ymin, ymax), classes

# Function to load dataset
def load_dataset(tfrecord_path):
    dataset = tf.data.TFRecordDataset(tfrecord_path)
    dataset = dataset.map(parse_tfrecord)
    images = []
    bboxes = []
    labels = []
    for image, bbox, label in dataset:
        images.append(image.numpy())
        bboxes.append(np.stack(bbox, axis=1))
        labels.append([l.decode('utf-8') for l in label.numpy()])
    return images, bboxes, labels

# Load datasets
train_images, train_bboxes, train_labels = load_dataset(train_tfrecord)
test_images, test_bboxes, test_labels = load_dataset(test_tfrecord)
valid_images, valid_bboxes, valid_labels = load_dataset(valid_tfrecord)

# Label encoding
label_encoder = LabelEncoder()
all_labels = [label for sublist in train_labels for label in sublist]
label_encoder.fit(all_labels)

# Pre-trained CNN model for feature extraction
cnn_model = VGG16(weights='imagenet', include_top=False, pooling='avg')

# Function to generate region proposals using Selective Search
def selective_search(image):
    ss = cv2.ximgproc.segmentation.createSelectiveSearchSegmentation()
    ss.setBaseImage(image)
    ss.switchToSelectiveSearchFast()
    rects = ss.process()
    proposals = []
    boxes = []
    for (x, y, w, h) in rects[:500]:
        roi = image[y:y+h, x:x+w]
        if roi.size == 0:
            continue
        roi = cv2.resize(roi, (224, 224))
        roi = img_to_array(roi)
        proposals.append(roi)
        boxes.append((x, y, x+w, y+h))
    return np.array(proposals), boxes

# Function to compute IoU
def compute_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interArea = max(0, xB - xA) * max(0, yB - yA)
    if interArea == 0:
        return 0.0
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    iou = interArea / float(boxAArea + boxBArea - interArea)
    return iou

# Prepare training data
X_train = []
y_train = []
bbox_targets = []

# Prepare training data
print("Preparing training data...")
for idx in tqdm(range(len(train_images))):
    image = train_images[idx]
    gt_boxes = train_bboxes[idx]  # gt_boxes is a NumPy array of shape (num_boxes, 4)
    gt_labels = train_labels[idx]
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    proposals, boxes = selective_search(image_rgb)

    # Preprocess proposals before passing to the CNN
    proposals_preprocessed = preprocess_input(proposals)
    features = cnn_model.predict(proposals_preprocessed)

    for i, box in enumerate(boxes):
        max_iou = 0
        assigned_label = None
        assigned_bbox = None
        for j, gt_box in enumerate(gt_boxes):
            # No need to call gt_box.numpy()
            # Convert normalized coordinates to absolute pixel coordinates
            gt_box_abs = [
                gt_box[0] * image.shape[1],  # xmin * image width
                gt_box[1] * image.shape[1],  # xmax * image width
                gt_box[2] * image.shape[0],  # ymin * image height
                gt_box[3] * image.shape[0],  # ymax * image height
            ]
            iou = compute_iou(box, gt_box_abs)
            if iou > max_iou:
                max_iou = iou
                assigned_label = gt_labels[j]
                assigned_bbox = gt_box_abs
        if max_iou > 0.5:
            X_train.append(features[i])
            y_train.append(assigned_label)
            # Compute bbox regression targets
            tx = (assigned_bbox[0] - box[0]) / (box[2] - box[0])
            ty = (assigned_bbox[2] - box[1]) / (box[3] - box[1])
            tw = np.log((assigned_bbox[1] - assigned_bbox[0]) / (box[2] - box[0]))
            th = np.log((assigned_bbox[3] - assigned_bbox[2]) / (box[3] - box[1]))
            bbox_targets.append([tx, ty, tw, th])
        elif max_iou < 0.3:
            X_train.append(features[i])
            y_train.append('background')
            bbox_targets.append([0, 0, 0, 0])

X_train = np.array(X_train)
y_train = np.array(y_train)
bbox_targets = np.array(bbox_targets)
y_train_enc = label_encoder.transform(y_train)

# Train SVM classifier
print("Training SVM classifier...")
svm_classifier = OneVsRestClassifier(LinearSVC(max_iter=100))
svm_classifier.fit(X_train, y_train_enc)

# Train bounding box regressor
print("Training bounding box regressor...")
bbox_regressor = LinearRegression()
bbox_regressor.fit(X_train, bbox_targets)

# Prepare validation data
X_valid = []
y_valid = []
bbox_valid_targets = []

print("Preparing validation data...")
for idx in tqdm(range(len(valid_images))):
    image = valid_images[idx]
    gt_boxes = valid_bboxes[idx]
    gt_labels = valid_labels[idx]
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    proposals, boxes = selective_search(image_rgb)

    features = cnn_model.predict(preprocess_input(proposals))

    for i, box in enumerate(boxes):
        max_iou = 0
        assigned_label = None
        assigned_bbox = None
        for j, gt_box in enumerate(gt_boxes):
            gt_box = gt_box.numpy()
            gt_box = [gt_box[0] * image.shape[1], gt_box[2] * image.shape[0],
                      gt_box[1] * image.shape[1], gt_box[3] * image.shape[0]]
            iou = compute_iou(box, gt_box)
            if iou > max_iou:
                max_iou = iou
                assigned_label = gt_labels[j]
                assigned_bbox = gt_box
        if max_iou > 0.5:
            X_valid.append(features[i])
            y_valid.append(assigned_label)
            tx = (assigned_bbox[0] - box[0]) / (box[2] - box[0])
            ty = (assigned_bbox[1] - box[1]) / (box[3] - box[1])
            tw = np.log((assigned_bbox[2] - assigned_bbox[0]) / (box[2] - box[0]))
            th = np.log((assigned_bbox[3] - assigned_bbox[1]) / (box[3] - box[1]))
            bbox_valid_targets.append([tx, ty, tw, th])
        elif max_iou < 0.3:
            X_valid.append(features[i])
            y_valid.append('background')
            bbox_valid_targets.append([0, 0, 0, 0])

X_valid = np.array(X_valid)
y_valid = np.array(y_valid)
bbox_valid_targets = np.array(bbox_valid_targets)
y_valid_enc = label_encoder.transform(y_valid)

# Evaluate SVM classifier
print("Evaluating SVM classifier...")
y_pred = svm_classifier.decision_function(X_valid)
average_precision = average_precision_score(y_valid_enc, y_pred, average='weighted')
print(f"Average Precision Score: {average_precision}")

# Evaluate bounding box regressor
print("Evaluating bounding box regressor...")
bbox_pred = bbox_regressor.predict(X_valid)
bbox_mse = np.mean((bbox_pred - bbox_valid_targets) ** 2)
print(f"Bounding Box Regressor MSE: {bbox_mse}")

# Function to apply bounding box regression
def apply_bbox_regression(box, bbox_target):
    x = box[0] + bbox_target[0] * (box[2] - box[0])
    y = box[1] + bbox_target[1] * (box[3] - box[1])
    w = (box[2] - box[0]) * np.exp(bbox_target[2])
    h = (box[3] - box[1]) * np.exp(bbox_target[3])
    x2 = x + w
    y2 = y + h
    return [int(x), int(y), int(x2), int(y2)]

# Non-Maximum Suppression
def non_max_suppression_fast(boxes, scores, overlap_thresh=0.3):
    if len(boxes) == 0:
        return []
    boxes = np.array(boxes)
    scores = np.array(scores)
    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]
    areas = (x2 - x1) * (y2 - y1)
    idxs = np.argsort(scores)
    pick = []
    while len(idxs) > 0:
        last = idxs[-1]
        pick.append(last)
        suppress = [last]
        for pos in range(len(idxs)-1):
            i = idxs[pos]
            xx1 = max(x1[last], x1[i])
            yy1 = max(y1[last], y1[i])
            xx2 = min(x2[last], x2[i])
            yy2 = min(y2[last], y2[i])
            w = max(0, xx2 - xx1)
            h = max(0, yy2 - yy1)
            overlap = (w * h) / areas[i]
            if overlap > overlap_thresh:
                suppress.append(i)
        idxs = np.delete(idxs, suppress)
    return pick

# Testing on test dataset
print("Testing on test dataset...")
for idx in tqdm(range(len(test_images))):
    image = test_images[idx]
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    proposals, boxes = selective_search(image_rgb)

    features = cnn_model.predict(preprocess_input(proposals))
    scores = svm_classifier.decision_function(features)
    pred_labels = svm_classifier.predict(features)
    bbox_targets = bbox_regressor.predict(features)

    # Apply bounding box regression
    refined_boxes = []
    refined_scores = []
    refined_labels = []
    for i in range(len(boxes)):
        if pred_labels[i] != label_encoder.transform(['background'])[0]:
            refined_box = apply_bbox_regression(boxes[i], bbox_targets[i])
            refined_boxes.append(refined_box)
            refined_scores.append(scores[i])
            refined_labels.append(pred_labels[i])

    # Apply Non-Maximum Suppression
    keep_idxs = non_max_suppression_fast(refined_boxes, refined_scores)
    final_boxes = [refined_boxes[i] for i in keep_idxs]
    final_labels = [label_encoder.inverse_transform([refined_labels[i]])[0] for i in keep_idxs]

    # Draw boxes on the image
    for i, box in enumerate(final_boxes):
        cv2.rectangle(image_rgb, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
        cv2.putText(image_rgb, final_labels[i], (box[0], box[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

    # Display the image
    cv2.imshow('Detected Objects', image_rgb)
    cv2.waitKey(0)

cv2.destroyAllWindows()
