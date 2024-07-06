from threading import Thread, Event
from queue import Queue
from picamera2 import Picamera2
import cv2 as cv
import numpy as np

"""
Steps in the computer vision file
1. Image de-noising/smoothing
2. Convert BGR to HSL for object masking
3. Object masking (Morphological Transformations if (1) and (2)'s results not good enough)
4. Optimise object mask by eroding and dilating
5. Pick the largest contour, if any
6. Get the image moment of the picked contour
7. Draw the contour and its center and bounding box, along with its y-axis
8. Determine 
"""

# The colour of the red traffic signs is RGB (238, 39, 55).
# The colour of the green traffic signs is RGB (68, 214, 44).

KERNEL = np.ones((5,5),np.float32) / 25
# HSV_RED = cv.cvtColor(np.uint8([[[55, 39, 238]]]), cv.COLOR_BGR2HSV)
# HSV_GREEN = cv.cvtColor(np.uint8([[[44, 214, 68]]]), cv.COLOR_BGR2HSV)
HSV_RED = cv.cvtColor(np.uint8([[[75, 28, 171]]]), cv.COLOR_BGR2HSV)
HSV_GREEN = cv.cvtColor(np.uint8([[[90, 210, 27]]]), cv.COLOR_BGR2HSV)

HSV_THRESHOLD = 30
BOUNDARY_THRSHOLD_RATIO = 4

# Initialize the camera
camera = Picamera2()
camera.configure(camera.create_preview_configuration(main={"format": 'XRGB8888'}))
camera.start()

# Queue to hold frames
frames_queue = Queue(maxsize=5) 


# Function to continuously read frames from the camera
def read_frames(event):
    while True:
        if event.is_set():
            break

        # in Blue-Red-Green-Alpha (BRGA) format
        frame = camera.capture_array()
        frames_queue.put(frame)


# Function to process and display frames
def process_frames(event, **kwargs):
    while True:
        if event.is_set():
            break

        frame = frames_queue.get()
        original = frame.copy()

        cv.imshow("original", original)
        height, width, channels = frame.shape

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(gray, (5, 5), 0)

        # Threshold the HSV image to get either red or green
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask_red = cv.inRange(hsv, (HSV_RED - HSV_THRESHOLD)[0][0], (HSV_RED + HSV_THRESHOLD)[0][0])
        mask_green = cv.inRange(hsv, (HSV_GREEN - HSV_THRESHOLD)[0][0], (HSV_GREEN + HSV_THRESHOLD)[0][0])

        # Combine the red and green masks to form one mask
        mask = cv.bitwise_or(mask_green, mask_red)
        mask = cv.erode(mask, None, iterations=2)
        mask = cv.dilate(mask, None, iterations=2)
        
        # Bitwise-AND mask and original image
        res = cv.bitwise_and(blurred, blurred, mask=mask)

        # thresh = cv.threshold(res, 60, 255, cv.THRESH_BINARY)
        # ret, thresh = cv.threshold(res, 60, 255, 0)
        # thresh = cv.adaptiveThreshold(res, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)

        ret, thresh =  cv.threshold(res, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Draw boundary lines
        boundary_left = int(width / BOUNDARY_THRSHOLD_RATIO)
        boundary_right = width - boundary_left
        boundary_positions = [boundary_left, boundary_right]
        
        for x in boundary_positions:
            cv.line(frame, (x, 0), (x, height), (0, 255, 0), thickness=2)

        # Only proceed if at least one contour was found
        if len(contours) > 0:
            # Find the largest contour in the mask and compute centroid and bounding box
            c = max(contours, key=cv.contourArea)
            
            # area = cv.contourArea(c)
            # if area < 100: # Arbitrary value of 100. To adjust when testing 
            #     continue

            M = cv.moments(c)

            if M["m00"] != 0: 
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                
                x, y, w, h = cv.boundingRect(c)
                cv.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)

                box = np.int0(cv.boxPoints(cv.minAreaRect(c)))
                cv.drawContours(frame, [box], 0, (0,0,255), 2)
            
                # draw the contour and center of the shape on the image
                if "construction" in kwargs and kwargs["construction"]:
                    cv.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    cv.circle(frame, center, 7, (255, 255, 255), -1)
                
                cv.line(frame, (center[0], 0), (center[0], height), (0, 255, 255), thickness=2)

                #
                # Performing maneuver analysis
                #

                object_color = cv.cvtColor(np.uint8([[ original[center[1], center[0]] ]]), cv.COLOR_BGR2HSV)[0][0]

                # Comparing hues of object colour between the lower and upper bound

                if ((HSV_GREEN - HSV_THRESHOLD)[0][0])[0] <= object_color[0] <= ((HSV_GREEN + HSV_THRESHOLD)[0][0])[0]:
                    if center[0] <= boundary_right:
                        # Green object is on the left side, perform left maneuver
                        print("Perform left maneuver (green)")

                if ((HSV_RED - HSV_THRESHOLD)[0][0])[0] <= object_color[0] <= ((HSV_RED + HSV_THRESHOLD)[0][0])[0]:
                    if center[0] >= boundary_left:
                        # Red object is on the right, perform right maneuver
                        print("Perform right maneuver (red)")

        print("END OF FUNCTION")
        cv.imshow('Rendered result', frame)
        
        # if the 'q' key is pressed, stop the loop
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"):
            event.set()
            break


event = Event()

# Create threads
read_thread = Thread(target=read_frames, args=(event,))
process_thread = Thread(target=process_frames, args=(event,))

# Start threads
read_thread.start()
process_thread.start()

# Wait for threads to finish
read_thread.join()
process_thread.join()
camera.close()

print("Stopping")
cv.destroyAllWindows()