import cv2
import numpy as np
import time
from scipy.ndimage import gaussian_filter

class Tracker:
    def __init__(self, initial_bbox):
        self.bbox = initial_bbox
        self.tracking_points = None
        self.median_displacement = None
        self.failure_threshold = 10  # Threshold for residual error to detect failure

    def initialize_tracking_points(self, frame):
        x, y, w, h = self.bbox
        grid_x, grid_y = np.meshgrid(
            np.linspace(x, x + w, 10), 
            np.linspace(y, y + h, 10)
        ) # start, stop and number of points
        self.tracking_points = np.array([grid_x.ravel(), grid_y.ravel()], dtype=np.float32).T

    def track(self, prev_frame, curr_frame):
        if self.tracking_points is None:
            return False

        new_points, status, _ = cv2.calcOpticalFlowPyrLK(prev_frame, curr_frame, self.tracking_points, None)
        if new_points is None or status is None:
            return False

        valid_new_points = new_points[status.flatten() == 1]
        valid_old_points = self.tracking_points[status.flatten() == 1]

        if len(valid_new_points) < 5:
            return False

        displacements = valid_new_points - valid_old_points
        self.median_displacement = np.median(displacements, axis=0)
        residual_error = np.median(np.linalg.norm(displacements - self.median_displacement, axis=1))

        if residual_error > self.failure_threshold:
            return False

        self.tracking_points = valid_new_points
        x, y, w, h = self.bbox
        self.bbox = (int(x + self.median_displacement[0]), int(y + self.median_displacement[1]), w, h)
        return True

    def draw_bbox(self, frame):
        x, y, w, h = self.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Tracking", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

class Detector:
    def __init__(self, initial_patch, relative_similarity_threshold=0.6, variance_threshold=0.5):
        self.initial_patch = initial_patch
        self.relative_similarity_threshold = relative_similarity_threshold
        self.variance_threshold = variance_threshold
        self.gaussian_std = 3
        self.base_classifiers = 13  # Number of binary comparisons in each base classifier
        self.posterior_table = self._initialize_posterior_table()
        self.positive_samples = []
        self.negative_samples = []
    
    def generate_scanning_windows(self, frame):
        """Generates scanning windows across the frame with specified scale and shift."""
        height, width = frame.shape[:2]
        scale_factor = 1.2
        windows = []
        
        # Scaling the window from 1.0 to accommodate different object sizes
        for scale in np.arange(1.0, 3.0, scale_factor):
            win_width, win_height = int(self.initial_patch[2] * scale), int(self.initial_patch[3] * scale)
            for y in range(0, height - win_height, int(0.1 * height)):
                for x in range(0, width - win_width, int(0.1 * width)):
                    windows.append((x, y, win_width, win_height))
        
        return windows

    def patch_variance(self, frame, patch):
        """Calculate the gray-value variance of a patch and compare it to initial patch."""
        x, y, w, h = patch
        patch_frame = frame[y:y+h, x:x+w]
        initial_patch_frame = frame[
            self.initial_patch[1]:self.initial_patch[1] + self.initial_patch[3],
            self.initial_patch[0]:self.initial_patch[0] + self.initial_patch[2]
        ]
        
        # Calculate variance
        initial_variance = np.var(initial_patch_frame)
        patch_variance = np.var(patch_frame)
        
        # Check if the patch variance is more than 50% of the initial patch variance
        return patch_variance >= self.variance_threshold * initial_variance

    def ensemble_classification(self, patch_frame):
        """Perform ensemble classification using pixel comparisons to generate binary codes."""
        patch_frame = gaussian_filter(patch_frame, sigma=self.gaussian_std)  # Gaussian blur

        # Discretize and perform pixel comparisons
        comparisons = self._generate_pixel_comparisons(patch_frame)
        
        # Generate binary codes and get posterior probability
        binary_code = self._generate_binary_code(comparisons)
        posterior_probability = np.mean([self.posterior_table[code] for code in binary_code])

        # Classify as object if the average posterior is larger than 0.5
        return posterior_probability > 0.5

    def _generate_pixel_comparisons(self, patch_frame):
        """Generate pixel comparisons (both horizontal and vertical) within a patch."""
        h, w = patch_frame.shape
        comparisons = []

        # Normalize and discretize the pixel locations within the patch
        num_comparisons = self.base_classifiers  # Number of comparisons per classifier
        for _ in range(num_comparisons):
            y1, x1 = np.random.randint(0, h), np.random.randint(0, w)
            y2, x2 = np.random.randint(0, h), np.random.randint(0, w)
            comparisons.append((patch_frame[y1, x1], patch_frame[y2, x2]))

        return comparisons

    def _generate_binary_code(self, comparisons):
        """Generate binary code from pixel comparisons for ensemble classification."""
        binary_code = []
        for val1, val2 in comparisons:
            binary_code.append(1 if val1 > val2 else 0)
        return binary_code

    def _initialize_posterior_table(self):
        """Initialize posterior table based on #p/(#p + #n) for binary codes."""
        num_codes = 2 ** self.base_classifiers
        posterior_table = np.zeros(num_codes)

        # This table could be updated as new positive and negative examples are added
        for i in range(num_codes):
            num_positives = np.random.randint(1, 10)  # Placeholder positive counts
            num_negatives = np.random.randint(1, 10)  # Placeholder negative counts
            posterior_table[i] = num_positives / (num_positives + num_negatives)

        return posterior_table

    def nearest_neighbor_classification(self, patch_frame):
        """Perform nearest neighbor classification to determine if patch matches object."""
        if not self.positive_samples or not self.negative_samples:
            return False

        # Flatten the patch for simplicity
        patch_feature = patch_frame.flatten()
        
        # Calculate similarities with positive and negative patches
        S_plus = self._max_similarity(patch_feature, self.positive_samples)
        S_minus = self._max_similarity(patch_feature, self.negative_samples)
        S_plus_50 = self._max_similarity_50_percent(patch_feature, self.positive_samples)

        # Calculate relative similarity
        relative_similarity = S_plus / (S_plus + S_minus)

        # Calculate conservative similarity
        conservative_similarity = S_plus_50 / (S_plus_50 + S_minus)

        # Determine if the patch is positive based on relative similarity threshold
        return relative_similarity > self.relative_similarity_threshold

    def _max_similarity(self, patch_feature, sample_set):
        """Calculate the maximum similarity between the patch and a sample set (positive or negative)."""
        return max(np.dot(patch_feature, sample.flatten()) / 
                (np.linalg.norm(patch_feature) * np.linalg.norm(sample.flatten()))
                for sample in sample_set)

    def _max_similarity_50_percent(self, patch_feature, positive_samples):
        """Calculate the maximum similarity within the first 50% of positive samples."""
        num_samples = len(positive_samples)
        num_50_percent = max(1, num_samples // 2)  # Ensure at least one sample
        return max(np.dot(patch_feature, positive_samples[i].flatten()) / 
                (np.linalg.norm(patch_feature) * np.linalg.norm(positive_samples[i].flatten()))
                for i in range(num_50_percent))


    def detect_object(self, frame):
        """Main function to detect object by iterating over patches in the frame."""
        detected_patches = []
        windows = self.generate_scanning_windows(frame)

        for patch in windows:
            if self.patch_variance(frame, patch):
                x, y, w, h = patch
                patch_frame = frame[y:y+h, x:x+w]

                # Run ensemble classification
                if self.ensemble_classification(patch_frame):
                    # Run nearest neighbor classifier
                    if self.nearest_neighbor_classification(patch_frame):
                        detected_patches.append(patch)

        return detected_patches

class Learning:
    def __init__(self, object_model, precision_P=0.9, recall_P=0.8, precision_N=0.9, recall_N=0.8, lambda_param=0.1):
        """
        Initializes the TLD Learning component.

        Parameters:
        object_model: The object model with positive and negative samples (patches).
        precision_P: Precision of the positive expert.
        recall_P: Recall of the positive expert.
        precision_N: Precision of the negative expert.
        recall_N: Recall of the negative expert.
        lambda_param: Margin threshold for adding new samples.
        """
        self.object_model = object_model
        self.precision_P = precision_P
        self.recall_P = recall_P
        self.precision_N = precision_N
        self.recall_N = recall_N
        self.lambda_param = lambda_param

    def update_model(self, new_patch, label, classifier_label, detection_confidence):
        """
        Updates the object model with a new labeled patch.

        Parameters:
        new_patch: The new patch to be added.
        label: The true label of the new patch (1 for positive, 0 for negative).
        classifier_label: The label given by the NN classifier (1 for positive, 0 for negative).
        detection_confidence: Confidence score from detection (probability or score).
        """
        if label == 1:
            self._add_positive_sample(new_patch, classifier_label, detection_confidence)
        elif label == 0:
            self._add_negative_sample(new_patch, classifier_label)

    def _add_positive_sample(self, patch, classifier_label, detection_confidence):
        """
        Adds a positive sample to the model if it meets the criteria based on the P-expert.

        Parameters:
        patch: The patch to be considered.
        classifier_label: The label given by the NN classifier (1 for positive, 0 for negative).
        detection_confidence: Confidence score from detection.
        """
        if classifier_label != 1 and detection_confidence > self.precision_P:
            self.object_model.add_positive_patch(patch)
        elif abs(detection_confidence - 0.5) < self.lambda_param:
            self.object_model.add_positive_patch(patch)

    def _add_negative_sample(self, patch, classifier_label):
        """
        Adds a negative sample to the model if it meets the criteria based on the N-expert.

        Parameters:
        patch: The patch to be considered.
        classifier_label: The label given by the NN classifier (1 for positive, 0 for negative).
        """
        if classifier_label == 1 and np.random.rand() < self.recall_N:
            self.object_model.add_negative_patch(patch)

    def run_PN_learning(self, tracked_patches, detected_patches):
        """
        Runs P-N learning to update the model based on tracked and detected patches.

        Parameters:
        tracked_patches: List of patches from the tracker.
        detected_patches: List of patches from the detector.
        """
        for patch, detection_confidence in tracked_patches:
            label = 1
            classifier_label = self._classify_patch(patch)
            self.update_model(patch, label, classifier_label, detection_confidence)

        for patch, label, detection_confidence in detected_patches:
            classifier_label = self._classify_patch(patch)
            self.update_model(patch, label, classifier_label, detection_confidence)

    def _classify_patch(self, patch):
        """
        Classifies a patch using the nearest neighbor classifier in the object model.

        Parameters:
        patch: The patch to classify.

        Returns:
        classifier_label: 1 if classified as positive, 0 otherwise.
        """
        classifier_label, _ = self.object_model.classify_patch(patch)
        return classifier_label

class ObjectModel:
    def __init__(self):
        self.positive_patches = []
        self.negative_patches = []

    def add_positive_patch(self, patch):
        self.positive_patches.append(patch)

    def add_negative_patch(self, patch):
        self.negative_patches.append(patch)

    def classify_patch(self, patch):
        max_positive_similarity = max([self._similarity(patch, pos_patch) for pos_patch in self.positive_patches], default=0)
        max_negative_similarity = max([self._similarity(patch, neg_patch) for neg_patch in self.negative_patches], default=0)

        if max_positive_similarity + max_negative_similarity > 0:
            relative_similarity = max_positive_similarity / (max_positive_similarity + max_negative_similarity)
        else:
            relative_similarity = 0

        label = 1 if relative_similarity > 0.5 else 0
        return label, relative_similarity

    def _similarity(self, patch1, patch2):
        return np.dot(patch1.flatten(), patch2.flatten()) / (np.linalg.norm(patch1) * np.linalg.norm(patch2))


# Main code to use the TLDTracker
# Try using a specific backend (e.g., cv2.CAP_DSHOW for DirectShow on Windows)
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Check if the camera is opened successfully
if not cap.isOpened():
    print("Error: Camera could not be opened.")
    cap.release()
    exit()

# Keep trying to read frames until we get a valid frame
while True:
    ret, first_frame = cap.read()
    if ret and np.mean(first_frame) > 10:  # Ensure the frame is not too dark
        break
    print("Retrying to capture a valid frame...")
    time.sleep(0.1)

# Let the user select the ROI on a valid frame
initial_bbox = cv2.selectROI("Select Object", first_frame, False)
cv2.destroyWindow("Select Object")

prev_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)

# Initialize TLD Tracker
tracker = Tracker(initial_bbox)
tracker.initialize_tracking_points(prev_gray)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    curr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Track the object
    success = tracker.track(prev_gray, curr_gray)
    if success:
        tracker.draw_bbox(frame)
    else:
        cv2.putText(frame, "Tracking Failed", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Display the result
    cv2.imshow("TLD Tracker", frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    prev_gray = curr_gray

cap.release()
cv2.destroyAllWindows()
