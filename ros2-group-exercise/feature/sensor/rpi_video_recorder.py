"""
Raspberry Pi Camera Module V2 - Video Recording
Records video streams for line detection testing and debugging
"""

import cv2
import time
import argparse
from pathlib import Path
from datetime import datetime


class RaspberryPiVideoRecorder:
    """Record video from Raspberry Pi Camera Module V2"""
    
    def __init__(self, output_dir="recordings", resolution=(640, 480), fps=30):
        """
        Initialize video recorder.
        
        Args:
            output_dir: Directory to save recordings
            resolution: Tuple of (width, height)
            fps: Frames per second
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.resolution = resolution
        self.fps = fps
        self.camera_index = 0
        
        # Initialize camera
        self.cap = None
        self.writer = None
        self._initialize_camera()
    
    def _initialize_camera(self):
        """Initialize camera capture"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            
            # Set FPS
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            if not self.cap.isOpened():
                raise RuntimeError("Failed to open camera")
            
            print(f"✓ Camera initialized: {self.resolution[0]}x{self.resolution[1]} @ {self.fps} FPS")
        
        except Exception as e:
            print(f"✗ Camera error: {e}")
            raise
    
    def _setup_video_writer(self, filename):
        """Setup video writer with codec"""
        filepath = self.output_dir / filename
        
        # Use H.264 codec (efficient for Raspberry Pi)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4 format
        
        self.writer = cv2.VideoWriter(
            str(filepath),
            fourcc,
            self.fps,
            self.resolution
        )
        
        if not self.writer.isOpened():
            raise RuntimeError(f"Failed to create video writer for {filepath}")
        
        print(f"✓ Recording to: {filepath}")
        return filepath
    
    def record_duration(self, duration_seconds=10, filename=None):
        """
        Record video for specified duration.
        
        Args:
            duration_seconds: How long to record (in seconds)
            filename: Output filename (auto-generated if None)
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.mp4"
        
        filepath = self._setup_video_writer(filename)
        
        start_time = time.time()
        frame_count = 0
        
        try:
            print(f"Recording for {duration_seconds} seconds... (Press Ctrl+C to stop)")
            
            while True:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("Failed to read frame")
                    break
                
                # Write frame to video file
                self.writer.write(frame)
                frame_count += 1
                
                # Display progress
                elapsed = time.time() - start_time
                if frame_count % 30 == 0:  # Update every 30 frames
                    print(f"  Recorded: {elapsed:.1f}s ({frame_count} frames)")
                
                # Check if duration reached
                if elapsed >= duration_seconds:
                    break
        
        except KeyboardInterrupt:
            print("\nRecording stopped by user")
        
        finally:
            self.writer.release()
            elapsed = time.time() - start_time
            print(f"✓ Saved: {frame_count} frames in {elapsed:.2f}s")
    
    def record_with_preview(self, duration_seconds=None, filename=None):
        """
        Record video with live preview window.
        
        Args:
            duration_seconds: Max duration (None = record until Ctrl+C)
            filename: Output filename (auto-generated if None)
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.mp4"
        
        filepath = self._setup_video_writer(filename)
        
        start_time = time.time()
        frame_count = 0
        
        try:
            print("Recording with preview (Press 'q' to quit)...")
            
            while True:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("Failed to read frame")
                    break
                
                # Write frame to video file
                self.writer.write(frame)
                frame_count += 1
                
                # Display preview
                cv2.imshow('Recording Preview', frame)
                
                # Add recording indicator
                elapsed = time.time() - start_time
                remaining_text = f"Recording... {elapsed:.1f}s ({frame_count} frames)"
                cv2.putText(frame, remaining_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Check exit condition
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\nRecording stopped by user")
                    break
                
                if duration_seconds and elapsed >= duration_seconds:
                    print(f"\nDuration limit reached ({duration_seconds}s)")
                    break
        
        finally:
            self.writer.release()
            cv2.destroyAllWindows()
            elapsed = time.time() - start_time
            print(f"✓ Saved: {frame_count} frames in {elapsed:.2f}s to {filepath}")
    
    def record_and_process(self, duration_seconds, filename=None, process_func=None):
        """
        Record video while processing frames in real-time.
        Useful for testing line detection while recording.
        
        Args:
            duration_seconds: How long to record
            filename: Output filename
            process_func: Function to process each frame (e.g., line detection)
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recording_{timestamp}.mp4"
        
        filepath = self._setup_video_writer(filename)
        
        start_time = time.time()
        frame_count = 0
        
        try:
            print(f"Recording for {duration_seconds}s with processing...")
            
            while True:
                ret, frame = self.cap.read()
                
                if not ret:
                    break
                
                # Process frame if function provided
                output_frame = frame.copy()
                if process_func:
                    output_frame = process_func(frame)
                
                # Write processed frame
                self.writer.write(output_frame)
                frame_count += 1
                
                elapsed = time.time() - start_time
                
                # Display
                cv2.imshow('Recording + Processing', output_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                if elapsed >= duration_seconds:
                    break
        
        finally:
            self.writer.release()
            cv2.destroyAllWindows()
            elapsed = time.time() - start_time
            print(f"✓ Saved: {frame_count} frames in {elapsed:.2f}s")
    
    def record_manual_push(self, countdown_seconds=3, filename=None):
        """
        Record while manually pushing robot along line.
        
        Workflow:
        1. Shows live preview
        2. Press 'r' to start recording (with countdown)
        3. Press 'q' to stop recording
        
        Args:
            countdown_seconds: Countdown before recording starts
            filename: Output filename (auto-generated if None)
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_push_{timestamp}.mp4"
        
        frame_count = 0
        is_recording = False
        
        try:
            print("=" * 60)
            print("MANUAL ROBOT PUSH RECORDING")
            print("=" * 60)
            print("Instructions:")
            print("  1. Position robot at starting point")
            print("  2. Press 'r' to BEGIN COUNTDOWN")
            print("  3. After countdown, push robot along the line")
            print("  4. Press 'q' to STOP RECORDING")
            print("=" * 60)
            
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame")
                    break
                
                display_frame = frame.copy()
                
                if is_recording:
                    # Recording state
                    elapsed = time.time() - start_time
                    frame_count += 1
                    
                    # Write to file
                    self.writer.write(frame)
                    
                    # Display recording indicator
                    cv2.putText(display_frame, "REC", (10, 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
                    cv2.putText(display_frame, f"Time: {elapsed:.1f}s | Frames: {frame_count}",
                               (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.putText(display_frame, "Press 'q' to stop",
                               (10, self.resolution[1] - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                else:
                    # Preview/waiting state
                    cv2.putText(display_frame, "PREVIEW", (10, 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
                    cv2.putText(display_frame, "Press 'r' to START",
                               (10, self.resolution[1] - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('Robot Line Push Recording', display_frame)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('r') and not is_recording:
                    # Start countdown
                    print(f"\nStarting countdown ({countdown_seconds}s)...")
                    for i in range(countdown_seconds, 0, -1):
                        print(f"  {i}...")
                        time.sleep(1)
                    
                    print("GO! Recording started...\n")
                    is_recording = True
                    start_time = time.time()
                    filepath = self._setup_video_writer(filename)
                    frame_count = 0
                
                elif key == ord('q') and is_recording:
                    print("\nStopping recording...")
                    is_recording = False
                    break
                
                elif key == ord('q') and not is_recording:
                    print("Exiting without recording")
                    break
        
        finally:
            if is_recording and self.writer:
                self.writer.release()
                elapsed = time.time() - start_time
                print(f"✓ Recording saved: {frame_count} frames in {elapsed:.2f}s")
                print(f"✓ File: {filepath}")
            
            cv2.destroyAllWindows()
    
    def release(self):
        """Release camera and writer resources"""
        if self.writer:
            self.writer.release()
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        print("✓ Resources released")


def main():
    parser = argparse.ArgumentParser(description='Record video from Raspberry Pi Camera')
    parser.add_argument('--duration', type=int, default=10, help='Recording duration in seconds (default: 10)')
    parser.add_argument('--output', type=str, help='Output filename')
    parser.add_argument('--mode', choices=['simple', 'preview', 'process', 'manual-push'], 
                       default='simple', help='Recording mode')
    parser.add_argument('--resolution', type=str, default='640x480', help='Resolution (WxH)')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second')
    parser.add_argument('--countdown', type=int, default=3, help='Countdown seconds before recording (manual-push mode)')
    
    args = parser.parse_args()
    
    # Parse resolution
    w, h = map(int, args.resolution.split('x'))
    
    # Create recorder
    recorder = RaspberryPiVideoRecorder(
        resolution=(w, h),
        fps=args.fps
    )
    
    try:
        if args.mode == 'simple':
            recorder.record_duration(args.duration, args.output)
        elif args.mode == 'preview':
            recorder.record_with_preview(args.duration, args.output)
        elif args.mode == 'process':
            # Example: record with line detection processing
            # Uncomment and customize if using with camera_line_detector
            recorder.record_duration(args.duration, args.output)
        elif args.mode == 'manual-push':
            recorder.record_manual_push(args.countdown, args.output)
    
    finally:
        recorder.release()


if __name__ == "__main__":
    main()
