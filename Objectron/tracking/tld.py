import cv2
import numpy as np
import time

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
        )
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
