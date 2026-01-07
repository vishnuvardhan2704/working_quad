"""
YOLOv8 ONNX Human Detection Module
Standalone detection module optimized for Raspberry Pi Camera Module 3

This module provides:
- Proper YOLOv8 preprocessing (letterbox resize, normalization)
- ONNX runtime inference
- Postprocessing with NMS (Non-Maximum Suppression)
- Configurable confidence and IoU thresholds

Usage:
    from yolo_detector import YOLODetector
    
    detector = YOLODetector(
        model_path="path/to/best.onnx",
        confidence_threshold=0.5,
        iou_threshold=0.45
    )
    
    boxes, scores = detector.detect(rgb_frame)
"""

import cv2
import numpy as np
import onnxruntime as ort


class YOLODetector:
    def __init__(
        self,
        model_path,
        confidence_threshold=0.5,
        iou_threshold=0.45,
        input_size=640,
        person_class_only=True
    ):
        """
        Initialize YOLOv8 ONNX detector
        
        Args:
            model_path: Path to ONNX model file
            confidence_threshold: Minimum confidence for detections (0-1)
            iou_threshold: IoU threshold for NMS (0-1)
            input_size: Model input size (default 640x640)
            person_class_only: Only detect "person" class (class 0 in COCO)
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.iou_threshold = iou_threshold
        self.input_size = input_size
        self.person_class_only = person_class_only
        
        # Initialize ONNX Runtime session
        self.session = ort.InferenceSession(
            model_path,
            providers=["CPUExecutionProvider"]
        )
        
        # Get input/output names
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        
        # Get model input shape
        model_input_shape = self.session.get_inputs()[0].shape
        self.model_height = model_input_shape[2] if len(model_input_shape) > 2 else input_size
        self.model_width = model_input_shape[3] if len(model_input_shape) > 3 else input_size
        
        print(f"[YOLODetector] Model loaded: {model_path}")
        print(f"[YOLODetector] Input shape: {model_input_shape}")
        print(f"[YOLODetector] Resolution: {self.model_width}x{self.model_height}")
        print(f"[YOLODetector] Confidence threshold: {confidence_threshold}")
        print(f"[YOLODetector] IoU threshold: {iou_threshold}")
    
    def letterbox_resize(self, img, new_shape=(640, 640), color=(114, 114, 114)):
        """
        Resize image with letterboxing to preserve aspect ratio
        This is the standard YOLOv8 preprocessing approach
        
        Args:
            img: Input image (H, W, C)
            new_shape: Target size (width, height)
            color: Padding color (RGB)
        
        Returns:
            Resized and padded image
        """
        shape = img.shape[:2]  # current shape [height, width]
        
        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        
        # Compute padding
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        
        dw /= 2  # divide padding into 2 sides
        dh /= 2
        
        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, 
                                cv2.BORDER_CONSTANT, value=color)
        
        return img
    
    def preprocess(self, frame):
        """
        Preprocess frame for YOLOv8 ONNX inference
        
        IMPORTANT: This model was trained with BGR images at lower resolution.
        
        Preprocessing pipeline:
        1. Convert RGB to BGR (model expects BGR from OpenCV)
        2. Letterbox resize to model input size (640x640)
        3. Normalize to [0, 1] range
        4. Convert from HWC to CHW format
        5. Add batch dimension
        
        Args:
            frame: RGB image from camera (H, W, 3)
        
        Returns:
            Preprocessed tensor (1, 3, H, W) ready for inference
        """
        # Convert RGB to BGR (model was trained with OpenCV BGR images)
        img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # Letterbox resize with padding
        img = self.letterbox_resize(img, (self.model_width, self.model_height))
        
        # Normalize to [0, 1]
        img = img.astype(np.float32) / 255.0
        
        # HWC to CHW format (Height, Width, Channels -> Channels, Height, Width)
        img = np.transpose(img, (2, 0, 1))
        
        # Add batch dimension
        img = np.expand_dims(img, axis=0)
        
        return img
    
    def nms(self, boxes, scores):
        """
        Apply Non-Maximum Suppression to remove overlapping boxes
        
        Args:
            boxes: List of [x1, y1, x2, y2] coordinates
            scores: List of confidence scores
        
        Returns:
            Filtered boxes and scores after NMS
        """
        if len(boxes) == 0:
            return [], []
        
        boxes = np.array(boxes)
        scores = np.array(scores)
        
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            
            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h
            
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            
            inds = np.where(iou <= self.iou_threshold)[0]
            order = order[inds + 1]
        
        return boxes[keep].tolist(), scores[keep].tolist()
    
    def postprocess(self, outputs, frame_shape):
        """
        Postprocess YOLOv8 ONNX outputs
        
        YOLOv8 output format for this model:
        - Shape: [1, 5, 8400] → (cx, cy, w, h, conf) for each of 8400 anchors
        - Coordinates are in model space (0-640 range for 640x640 input)
        
        Args:
            outputs: Raw ONNX model output [1, 5, 8400]
            frame_shape: Original frame shape (H, W)
        
        Returns:
            boxes: List of [x1, y1, x2, y2] in original frame coordinates
            scores: List of confidence scores
        """
        h, w = frame_shape
        boxes = []
        scores = []
        
        # Output shape is [1, 5, 8400]
        # Need to transpose to [8400, 5] for easier processing
        outputs = outputs[0]  # Remove batch dimension -> [5, 8400]
        outputs = outputs.T   # Transpose to [8400, 5]
        
        # Calculate scale factors from model input size to original frame
        scale_x = w / self.model_width
        scale_y = h / self.model_height
        
        # Process each detection (8400 anchors)
        for detection in outputs:
            # Detection format: [cx, cy, w, h, conf]
            cx, cy, w_box, h_box, conf = detection
            
            # Apply confidence threshold
            if conf < self.confidence_threshold:
                continue
            
            # Convert from center coordinates to corner coordinates
            # Coordinates are in model space (640x640), scale to original frame
            x1 = (cx - w_box / 2) * scale_x
            y1 = (cy - h_box / 2) * scale_y
            x2 = (cx + w_box / 2) * scale_x
            y2 = (cy + h_box / 2) * scale_y
            
            # Clip to frame boundaries
            x1 = int(max(0, min(x1, w)))
            y1 = int(max(0, min(y1, h)))
            x2 = int(max(0, min(x2, w)))
            y2 = int(max(0, min(y2, h)))
            
            # Skip invalid boxes
            if x2 <= x1 or y2 <= y1:
                continue
            
            boxes.append([x1, y1, x2, y2])
            scores.append(float(conf))
        
        # Apply Non-Maximum Suppression
        boxes, scores = self.nms(boxes, scores)
        
        return boxes, scores
    
    def detect(self, frame):
        """
        Run YOLOv8 detection on a frame
        
        Args:
            frame: RGB image from camera (H, W, 3)
                  Note: Will be converted to BGR internally (model expects BGR)
        
        Returns:
            boxes: List of [x1, y1, x2, y2] bounding boxes
            scores: List of confidence scores
        """
        # Preprocess frame (converts RGB->BGR internally)
        input_tensor = self.preprocess(frame)
        
        # Run inference
        outputs = self.session.run([self.output_name], {self.input_name: input_tensor})[0]
        
        # Postprocess outputs
        boxes, scores = self.postprocess(outputs, frame.shape[:2])
        
        return boxes, scores
    
    def update_threshold(self, confidence=None, iou=None):
        """
        Update detection thresholds dynamically
        
        Args:
            confidence: New confidence threshold
            iou: New IoU threshold for NMS
        """
        if confidence is not None:
            self.confidence_threshold = confidence
            print(f"[YOLODetector] Confidence threshold updated to {confidence}")
        
        if iou is not None:
            self.iou_threshold = iou
            print(f"[YOLODetector] IoU threshold updated to {iou}")


# Convenience function for quick usage
def create_detector(model_path, confidence=0.5, iou=0.45):
    """
    Quick initialization function
    
    Args:
        model_path: Path to ONNX model
        confidence: Confidence threshold (default 0.5)
        iou: IoU threshold for NMS (default 0.45)
    
    Returns:
        YOLODetector instance
    """
    return YOLODetector(
        model_path=model_path,
        confidence_threshold=confidence,
        iou_threshold=iou
    )


if __name__ == "__main__":
    """
    Test the detector with a sample image
    """
    import argparse
    
    parser = argparse.ArgumentParser(description="Test YOLOv8 detector")
    parser.add_argument("--model", required=True, help="Path to ONNX model")
    parser.add_argument("--image", required=True, help="Path to test image")
    parser.add_argument("--confidence", type=float, default=0.5, help="Confidence threshold")
    args = parser.parse_args()
    
    # Load detector
    detector = YOLODetector(args.model, confidence_threshold=args.confidence)
    
    # Load test image
    img = cv2.imread(args.image)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    
    # Run detection
    boxes, scores = detector.detect(img_rgb)
    
    print(f"\nDetected {len(boxes)} person(s)")
    
    # Draw results
    for box, score in zip(boxes, scores):
        x1, y1, x2, y2 = box
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f"Person {score:.2f}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    # Save result
    cv2.imwrite("detection_result.jpg", img)
    print("Result saved to detection_result.jpg")
