import cv2

# Initialize the tracker
tracker = cv2.legacy.TrackerTLD_create()

# Open the video source
cap = cv2.VideoCapture(0)  # Change 0 to your video file path if you're using a file

# Read the first frame
ret, first_frame = cap.read()
if not ret:
    print("Failed to read video.")
    cap.release()
    exit()

# Let the user select the ROI
bbox = cv2.selectROI("Select Object", first_frame, fromCenter=False, showCrosshair=True)
cv2.destroyWindow("Select Object")  # Close the ROI selection window

# Initialize the tracker with the selected region and the first frame
ret = tracker.init(first_frame, bbox)

while True:
    # Capture each frame
    ret, frame = cap.read()
    if not ret:
        break

    # Update the tracker for the current frame
    ret, bbox = tracker.update(frame)

    if ret:
        # Draw a bounding box if tracking is successful
        x, y, w, h = map(int, bbox)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(frame, "Tracking", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    else:
        # Tracking failure
        cv2.putText(frame, "Lost", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    # Display the frame
    cv2.imshow("TLD Tracker", frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the resources
cap.release()
cv2.destroyAllWindows()
