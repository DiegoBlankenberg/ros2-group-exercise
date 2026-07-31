#!/usr/bin/env python3
"""
Headless Robot Line Push Recorder (Fixed for Raspberry Pi Camera V2)
Records video without GUI (works over SSH)
"""

import cv2
import time
import sys
from pathlib import Path
from datetime import datetime


class HeadlessVideoRecorder:
    """Record video from Raspberry Pi Camera without display"""

    def __init__(self, output_dir="recordings", resolution=(640, 480), fps=30):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        self.resolution = resolution
        self.fps = fps

        self.cap = None
        self.writer = None
        self._initialize_camera()

    def _initialize_camera(self):
        """Initialize camera capture with warm-up for Pi Camera V2"""
        try:
            # FIX 1: Force V4L2 backend — most reliable on Raspberry Pi
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            if not self.cap.isOpened():
                raise RuntimeError("Failed to open camera")

            # FIX 2: Give the sensor time to auto-expose
            print("Warming up camera (2s)...")
            time.sleep(2)

            # FIX 3: Flush stale dark frames sitting in the buffer
            print("Flushing frame buffer...")
            for _ in range(30):
                self.cap.read()

            # FIX 4: Verify a real frame is bright enough to record
            ret, test_frame = self.cap.read()
            if not ret:
                raise RuntimeError("Camera opened but could not read a frame")

            brightness = test_frame.mean()
            if brightness < 5:
                print(f"⚠ WARNING: Test frame is very dark (mean={brightness:.1f}).")
                print("  Check: raspi-config → Interface Options → Camera → Enable")
                print("  Check: vcgencmd get_camera  (should show detected=1)")
                print("  Check: ribbon cable is seated correctly")
            else:
                print(f"✓ Camera check passed (mean brightness: {brightness:.1f})")

            print(f"✓ Camera ready: {self.resolution[0]}x{self.resolution[1]} @ {self.fps} FPS")

        except Exception as e:
            print(f"✗ Camera error: {e}")
            raise

    def _setup_writer(self, filepath):
        """
        Setup video writer — tries XVID/AVI first (most reliable on Pi),
        falls back to mp4v/MP4 if the output name ends in .mp4.
        """
        path_str = str(filepath)

        # FIX 5: XVID into .avi is the most reliable combo on Pi/OpenCV
        if path_str.endswith(".mp4"):
            # Use mp4v for MP4 containers; some Pi builds have issues with it
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        else:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')

        writer = cv2.VideoWriter(path_str, fourcc, self.fps, self.resolution)

        if not writer.isOpened():
            # Fallback: try AVI with XVID regardless of requested extension
            avi_path = Path(path_str).with_suffix('.avi')
            print(f"⚠ mp4v writer failed — retrying as AVI: {avi_path}")
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            writer = cv2.VideoWriter(str(avi_path), fourcc, self.fps, self.resolution)
            if not writer.isOpened():
                raise RuntimeError(f"Could not open any video writer for {filepath}")
            filepath = avi_path

        print(f"✓ Writing to: {filepath}")
        return writer, filepath

    def record_headless(self, duration_seconds=None, filename=None):
        """
        Record video without display (headless / SSH mode).

        Args:
            duration_seconds: Max recording time (None = record until Ctrl+C)
            filename: Output filename (auto-generated if None)
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_push_{timestamp}.avi"   # .avi is safer on Pi

        filepath = self.output_dir / filename
        self.writer, filepath = self._setup_writer(filepath)

        print("\n" + "=" * 60)
        print("HEADLESS RECORDING STARTED")
        print("=" * 60)
        print(f"File      : {filepath}")
        print(f"Resolution: {self.resolution[0]}x{self.resolution[1]} @ {self.fps} FPS")
        if duration_seconds:
            print(f"Duration  : {duration_seconds}s")
        else:
            print("Duration  : unlimited (Ctrl+C to stop)")
        print("=" * 60 + "\n")

        start_time = time.time()
        frame_count = 0

        try:
            while True:
                ret, frame = self.cap.read()

                if not ret:
                    print("⚠ Failed to read frame — retrying...")
                    time.sleep(0.05)
                    continue

                self.writer.write(frame)
                frame_count += 1

                elapsed = time.time() - start_time

                if frame_count % 30 == 0:
                    fps_actual = frame_count / elapsed if elapsed > 0 else 0
                    print(
                        f"  {elapsed:6.1f}s | {frame_count:5d} frames | {fps_actual:.1f} fps",
                        end='\r'
                    )

                if duration_seconds and elapsed >= duration_seconds:
                    print(f"\n✓ Duration limit reached ({duration_seconds}s)")
                    break

        except KeyboardInterrupt:
            print("\n\n✓ Recording stopped by user")

        finally:
            elapsed = time.time() - start_time
            self.writer.release()
            self.writer = None
            print(f"✓ Saved  : {frame_count} frames in {elapsed:.2f}s")
            print(f"✓ File   : {filepath}")
            print("=" * 60)

    def release(self):
        """Release resources"""
        if self.writer:
            self.writer.release()
        if self.cap:
            self.cap.release()
        print("✓ Camera released")


def main():
    print("\n" + "=" * 60)
    print("HEADLESS ROBOT LINE PUSH RECORDER")
    print("=" * 60)
    print("Runs WITHOUT a display (SSH-safe)")
    print("=" * 60 + "\n")

    recorder = HeadlessVideoRecorder(
        resolution=(640, 480),
        fps=30
    )

    try:
        recorder.record_headless(duration_seconds=None)
    except Exception as e:
        print(f"\n✗ Error: {e}")
        sys.exit(1)
    finally:
        recorder.release()


if __name__ == "__main__":
    main()