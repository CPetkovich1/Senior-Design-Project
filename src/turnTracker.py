import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# --- SETTINGS ---
PIN_D = 28  # Right turn pin to Arduino
TURN_THRESHOLD = 500  # ADJUST THIS: Total horizontal "pixels" for a 90-degree turn

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_D, GPIO.OUT)

def get_90_degree_turn():
    cap = cv2.VideoCapture(0)
    
    # Parameters for ShiTomasi corner detection
    feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
    
    # Parameters for Lucas Kanade optical flow
    lk_params = dict(winSize=(15, 15), maxLevel=2, 
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # Take first frame and find corners
    ret, old_frame = cap.read()
    old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

    total_rotation = 0
    GPIO.output(PIN_D, GPIO.HIGH) # Start turning
    
    print("Turning...")

    while total_rotation < TURN_THRESHOLD:
        ret, frame = cap.read()
        if not ret: break
        
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Calculate optical flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

        # Select good points
        if p1 is not None:
            good_new = p1[st == 1]
            good_old = p0[st == 1]

            # Calculate average horizontal movement
            dx = np.mean(good_new[:, 0] - good_old[:, 0])
            total_rotation += abs(dx)

            # Update for next frame
            old_gray = frame_gray.copy()
            p0 = good_new.reshape(-1, 1, 2)
            
            # If we lose points, find new ones
            if len(p0) < 10:
                p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)
        
    GPIO.output(PIN_D, GPIO.LOW) # Stop turning
    print(f"Turn Complete. Total movement: {total_rotation}")
    cap.release()

# Run the turn
try:
    get_90_degree_turn()
finally:
    GPIO.cleanup()
