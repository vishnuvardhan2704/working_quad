"""
Optimized Human Detection for 30 FPS on Raspberry Pi
- Frame skipping for inference
- Reduced resolution
- Optimized drawing
- Intel RealSense color camera support via pyrealsense2
"""

import cv2
import time
import argparse
import numpy as np
from yolo_detector import YOLODetector
import threading
from collections import deque

# Try to import pyrealsense2 for RealSense camera
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("[WARN] pyrealsense2 not available, falling back to OpenCV")


# -------------------- ARGUMENTS --------------------
parser = argparse.ArgumentParser(description="Optimized human detection (30 FPS target)")
parser.add_argument("-d", "--duration", type=int, default=0,
                    help="Recording duration in seconds (0 = no recording)")
parser.add_argument("-c", "--confidence", type=float, default=0.6,
                    help="Confidence threshold (0-1)")
parser.add_argument("--show-fps", action="store_true",
                    help="Display FPS counter")
parser.add_argument("--skip", type=int, default=3,
                    help="Process every Nth frame (3 = process 1 out of 3 frames)")
parser.add_argument("--model", type=str, 
                    default="/home/dart2/duplicate_scout_drone/working_quad/best.onnx",
                    help="Path to ONNX model")
parser.add_argument("--no-realsense", action="store_true",
                    help="Force OpenCV camera instead of RealSense SDK")
parser.add_argument("--camera", type=int, default=2,
                    help="Camera device index for OpenCV fallback (default: 2)")
args = parser.parse_args()


# -------------------- CAMERA SETUP --------------------
use_realsense = REALSENSE_AVAILABLE and not args.no_realsense
pipeline = None
cap = None

if use_realsense:
    print("[Camera] Initializing Intel RealSense with pyrealsense2...")
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        # Enable color stream at 640x480 @ 30fps
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        print("[Camera] RealSense COLOR stream started (640x480 @ 30fps)")
    except Exception as e:
        print(f"[Camera] RealSense init failed: {e}")
        use_realsense = False
        pipeline = None

if not use_realsense:
    print(f"[Camera] Using OpenCV VideoCapture at /dev/video{args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        for idx in [2, 4, 0, 1]:
            if idx == args.camera:
                continue
            print(f"[Camera] Trying /dev/video{idx}...")
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                break
        if not cap.isOpened():
            print("[Camera] ERROR: No camera available")
            exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    print(f"[Camera] OpenCV camera started (640x480 @ 30fps)")


# -------------------- DETECTOR SETUP --------------------
print("\n[Detector] Loading YOLOv8 model...")
detector = YOLODetector(
    model_path=args.model,
    confidence_threshold=args.confidence,
    iou_threshold=0.45,
    person_class_only=True
)
print(f"[Detector] Frame skip rate: {args.skip} (process every {args.skip} frames)")
print("[Detector] Ready!\n")


# -------------------- OPTIONAL RECORDING --------------------
video_writer = None
if args.duration > 0:
    print(f"[Recording] Will record for {args.duration} seconds to video.mp4")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video_writer = cv2.VideoWriter("video.mp4", fourcc, 30, (320, 240))
    start_time = time.time()


# -------------------- SHARED STATE --------------------
# Use threading to separate camera capture from detection
latest_boxes = []
latest_scores = []
detection_lock = threading.Lock()
detection_running = True
frame_queue = deque(maxlen=1)  # Only keep latest frame


# -------------------- DETECTION THREAD --------------------
def detection_thread():
    """Background thread for running detection"""
    global latest_boxes, latest_scores
    frame_count = 0
    
    while detection_running:
        # Get frame from queue
        if len(frame_queue) == 0:
            time.sleep(0.001)
            continue
        
        frame = frame_queue[0]
        frame_count += 1
        
        # Skip frames to maintain higher display FPS
        if frame_count % args.skip != 0:
            continue
        
        # Run detection
        boxes, scores = detector.detect(frame)
        
        # Update shared state
        with detection_lock:
            latest_boxes = boxes
            latest_scores = scores


# Start detection thread
detection_thread_obj = threading.Thread(target=detection_thread, daemon=True)
detection_thread_obj.start()


# -------------------- FPS TRACKING --------------------
fps_start_time = time.time()
fps_counter = 0
fps_display = 0


# -------------------- MAIN LOOP --------------------
print("[Main] Starting detection loop. Press 'q' to quit.\n")

try:
    while True:
        # Capture frame from camera
        if use_realsense:
            # Get frame from RealSense pipeline
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            # Convert to numpy array (already BGR8)
            frame = np.asanyarray(color_frame.get_data())
            ret = True
        else:
            # Get frame from OpenCV
            ret, frame = cap.read()
        
        if not ret:
            print("[Main] ERROR: Failed to read frame")
            break
        
        # Resize frame for consistency (640x480 -> 320x240 for faster processing)
        frame = cv2.resize(frame, (320, 240))
        
        # Convert BGR to RGB for detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Add to queue for detection thread
        if len(frame_queue) == 0 or args.skip == 1:
            frame_queue.append(frame_rgb.copy())
        else:
            frame_queue[0] = frame_rgb.copy()
        
        # Use BGR frame directly for display (no conversion needed)
        frame_display = frame.copy()
        
        # Get latest detections (from detection thread)
        with detection_lock:
            boxes = latest_boxes.copy()
            scores = latest_scores.copy()
        
        # Draw bounding boxes (optimized - draw directly without intermediate variables)
        for i, (box, score) in enumerate(zip(boxes, scores)):
            x1, y1, x2, y2 = box
            cv2.rectangle(frame_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_display, f"P {score:.2f}",
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1)
        
        # Calculate and display FPS
        fps_counter += 1
        elapsed = time.time() - fps_start_time
        if elapsed > 1.0:
            fps_display = fps_counter / elapsed
            fps_counter = 0
            fps_start_time = time.time()
        
        if args.show_fps:
            cv2.putText(frame_display, f"FPS: {fps_display:.1f}",
                        (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)
        
        # Display detection count (optimized text)
        cv2.putText(frame_display, f"Det: {len(boxes)}",
                    (5, frame_display.shape[0] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (255, 255, 255), 1)
        
        # Show frame
        cv2.imshow("Human Detection - 30 FPS", frame_display)
        
        # Record video if enabled
        if video_writer:
            video_writer.write(frame_display)
            
            if time.time() - start_time > args.duration:
                print(f"\n[Recording] Finished recording {args.duration} seconds")
                break
        
        # Check for quit (1ms wait for 30+ FPS)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n[Main] User quit")
            break

finally:
    print("\n[Cleanup] Stopping...")
    detection_running = False
    detection_thread_obj.join(timeout=1.0)
    if use_realsense and pipeline:
        pipeline.stop()
    if cap:
        cap.release()
    if video_writer:
        video_writer.release()
    cv2.destroyAllWindows()
    print("[Cleanup] Done!")
