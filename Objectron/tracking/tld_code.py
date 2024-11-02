import cv2
import numpy as np

# Parameters
NN_THRESHOLD = 0.6  # Threshold for Nearest Neighbor classification

class TLDTracker:
    def __init__(self):
        self.tracker = cv2.legacy.TrackerMedianFlow_create()  # Initialize with MedianFlow
        self.model = []  # Model to store positive and negative patches
        self.nn_threshold = NN_THRESHOLD

    def initialize(self, frame, bbox):
        self.tracker.init(frame, bbox)
        self.model.append(frame[int(bbox[1]):int(bbox[1]+bbox[3]), int(bbox[0]):int(bbox[0]+bbox[2])])

    def update(self, frame):
        success, bbox = self.tracker.update(frame)
        if success:
            self.learn(frame, bbox)
        return success, bbox

    def learn(self, frame, bbox):
        patch = frame[int(bbox[1]):int(bbox[1]+bbox[3]), int(bbox[0]):int(bbox[0]+bbox[2])]
        similarity = max(self.compute_similarity(patch, m) for m in self.model)
        if similarity < self.nn_threshold:
            self.model.append(patch)  # Add to the model if similarity is low

    def compute_similarity(self, patch1, patch2):
        # Calculate normalized cross-correlation similarity
        patch1 = cv2.resize(patch1, (15, 15))
        patch2 = cv2.resize(patch2, (15, 15))
        return cv2.matchTemplate(patch1, patch2, cv2.TM_CCOEFF_NORMED).flatten()[0]

# Mouse callback to select ROI
def select_roi(event, x, y, flags, param):
    global bbox, selecting, tracking, tld_tracker
    if event == cv2.EVENT_LBUTTONDOWN:
        bbox = (x, y, 0, 0)
        selecting = True
    elif event == cv2.EVENT_MOUSEMOVE and selecting:
        bbox = (bbox[0], bbox[1], x - bbox[0], y - bbox[1])
    elif event == cv2.EVENT_LBUTTONUP:
        selecting = False
        tracking = True
        tld_tracker.initialize(frame, bbox)

# Main code
cap = cv2.VideoCapture(0)  # Change to file path for a video
ret, frame = cap.read()
cv2.namedWindow("TLD Tracker")
cv2.setMouseCallback("TLD Tracker", select_roi)

tld_tracker = TLDTracker()
bbox = None
selecting = False
tracking = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    if tracking:
        success, bbox = tld_tracker.update(frame)
        if success:
            x, y, w, h = map(int, bbox)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, "Tracking", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        else:
            cv2.putText(frame, "Lost", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    elif selecting and bbox:
        x, y, w, h = map(int, bbox)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "Select object", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("TLD Tracker", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
