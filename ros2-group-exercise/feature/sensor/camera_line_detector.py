"""
Camera-based Line Detection for Raspberry Pi Camera Module V2
Detects masking tape lines on dark floors for robot guidance
"""

import cv2
import numpy as np
from pathlib import Path
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class CameraLineDetector:
    """
    Image processing pipeline for detecting masking tape lines.
    Segments tape from background and provides line coordinates.
    """
    
    def __init__(self, camera_index=0, resolution=(640, 480)):
        """
        Initialize the camera and detection parameters.
        
        Args:
            camera_index: Camera device index (0 for primary camera)
            resolution: Tuple of (width, height) for capture
        """
        self.camera_index = camera_index
        self.resolution = resolution
        self.cap = None
        
        # Image processing parameters (tunable)
        self.blur_kernel = (5, 5)
        self.canny_low = 50
        self.canny_high = 150
        self.hough_threshold = 50
        self.hough_min_length = 30
        self.hough_max_gap = 10
        
        self._initialize_camera()
    
    def _initialize_camera(self):
        """Initialize the video capture object."""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera {self.camera_index}")
            
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            
            logger.info(f"Camera initialized: {self.resolution[0]}x{self.resolution[1]}")
        
        except Exception as e:
            logger.error(f"Camera initialization error: {e}")
            raise
    
    def capture_frame(self):
        """
        Capture a frame from the camera.
        
        Returns:
            np.ndarray: BGR image frame, or None if capture failed
        """
        if self.cap is None:
            return None
        
        ret, frame = self.cap.read()
        if not ret:
            logger.warning("Failed to capture frame")
            return None
        
        return frame
    
    def detect_line(self, frame):
        """
        Detect masking tape line in the image.
        
        Pipeline:
        1. Convert to grayscale
        2. Apply Gaussian blur
        3. Threshold to isolate tape
        4. Edge detection (Canny)
        5. Hough line detection
        
        Args:
            frame: BGR image from camera
        
        Returns:
            dict: Detection results containing:
                - 'lines': List of detected line coordinates
                - 'binary': Binary mask of detected tape
                - 'edges': Edge map
                - 'centroid': Estimated line centroid (x, y)
        """
        if frame is None:
            return None
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, self.blur_kernel, 0)
        
        # Threshold to isolate tape (masking tape is typically lighter than dark floor)
        # Adjust the threshold value based on your lighting conditions
        _, binary = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)
        
        # Morphological operations to clean up the binary image
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # Edge detection
        edges = cv2.Canny(binary, self.canny_low, self.canny_high)
        
        # Hough line detection
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_length,
            maxLineGap=self.hough_max_gap
        )
        
        # Calculate centroid of detected tape
        centroid = self._calculate_centroid(binary)
        
        return {
            'lines': lines,
            'binary': binary,
            'edges': edges,
            'centroid': centroid
        }
    
    def _calculate_centroid(self, binary_image):
        """
        Calculate the centroid of the detected tape region.
        
        Args:
            binary_image: Binary mask
        
        Returns:
            tuple: (x, y) centroid coordinates, or None if no tape detected
        """
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate moments to find centroid
        M = cv2.moments(largest_contour)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            return (cx, cy)
        
        return None
    
    def visualize_detection(self, frame, detection_results):
        """
        Visualize detection results on the frame.
        
        Args:
            frame: Original BGR image
            detection_results: Results from detect_line()
        
        Returns:
            np.ndarray: Annotated frame with detections drawn
        """
        if detection_results is None or frame is None:
            return frame
        
        output = frame.copy()
        
        # Draw detected lines
        if detection_results['lines'] is not None:
            for line in detection_results['lines']:
                x1, y1, x2, y2 = line[0]
                cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw centroid
        if detection_results['centroid'] is not None:
            cx, cy = detection_results['centroid']
            cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(output, f"Line at ({cx}, {cy})", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return output
    
    def run_live(self, display=True):
        """
        Run live line detection from camera feed.
        
        Args:
            display: If True, show the detection visualization
        """
        try:
            while True:
                frame = self.capture_frame()
                if frame is None:
                    break
                
                detection = self.detect_line(frame)
                
                if display and detection is not None:
                    output = self.visualize_detection(frame, detection)
                    cv2.imshow('Line Detection', output)
                
                # Exit on 'q' key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        
        finally:
            self.release()
    
    def release(self):
        """Release camera resources."""
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        logger.info("Camera released")


def main():
    """Test the line detector with live camera feed."""
    detector = CameraLineDetector(resolution=(640, 480))
    
    print("Starting line detection...")
    print("Press 'q' to quit")
    
    detector.run_live(display=True)


if __name__ == "__main__":
    main()
