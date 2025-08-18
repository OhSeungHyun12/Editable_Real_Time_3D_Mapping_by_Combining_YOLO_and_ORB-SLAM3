import cv2

n = 48
# Open webcam
cap = cv2.VideoCapture(n)

# Check if the webcam is opened properly
if not cap.isOpened():
    print("Error: Unable to open webcam.")
    exit()

# Continuously read video frames and display them on the screen
while True:
    # Reading frames
    ret, frame = cap.read()

    # Check if the frame was read successfully
    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the read frames in a window named 'Webcam'
    cv2.imshow('Webcam', frame)

    # Press the 'q' key to end the loop.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After use, release the resource
cap.release()
cv2.destroyAllWindows()
