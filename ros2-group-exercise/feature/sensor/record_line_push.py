#!/usr/bin/env python3
"""
Simple wrapper script for recording robot pushing along a line
Just run this on the Raspberry Pi and follow the on-screen instructions
"""

from rpi_video_recorder import RaspberryPiVideoRecorder
import sys


def main():
    print("\n" + "=" * 70)
    print("RASPBERRY PI ROBOT LINE PUSH RECORDER")
    print("=" * 70)
    print("\nConfiguration:")
    print("  Resolution: 640x480")
    print("  FPS: 30")
    print("  Countdown: 3 seconds")
    print("\n" + "=" * 70 + "\n")
    
    # Create recorder with default settings optimized for line detection
    recorder = RaspberryPiVideoRecorder(
        resolution=(640, 480),
        fps=30
    )
    
    try:
        # Start manual push recording
        recorder.record_manual_push(countdown_seconds=3)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError: {e}")
        sys.exit(1)
    
    finally:
        recorder.release()


if __name__ == "__main__":
    main()
