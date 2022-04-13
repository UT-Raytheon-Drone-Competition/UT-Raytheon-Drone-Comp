import numpy as np
import cv2

def getContourWithMaxAR(contours):
    max_ar = 0
    max_contour = contours[0]
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = float(w) / h
        if aspect_ratio > max_ar:
            max_contour = c
            max_ar = aspect_ratio

    return max_contour

def contours():
    video_capture = cv2.VideoCapture(1)
    video_capture.set(3, 160)
    video_capture.set(4, 120)

    while (True):
        # Capture the frames
        ret, frame = video_capture.read()
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        # Color thresholding
        ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
        # Find the contours of the frame
        contours, hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
        # Find the biggest contour (if detected)

        if len(contours) > 0:
            c = getContourWithMaxAR(contours)
            M = cv2.moments(c)
            cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else 0
            cy = int(M['m01'] / M['m00']) if M['m00'] != 0 else 0
            cv2.line(blur, (cx, 0), (cx, 720), (255, 0, 0), 1)
            cv2.line(blur, (0, cy), (1280, cy), (255, 0, 0), 1)
            cv2.drawContours(blur, contours, -1, (0, 255, 0), 1)

        else:
            print("No lines detected")

        cv2.imshow('frame', blur)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    contours()