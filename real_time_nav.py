import cv2
import numpy as np
import torch
from PIL import Image
from ultralytics import YOLO
from transformers import OneFormerProcessor, OneFormerForUniversalSegmentation
import time
from helper import analyze_scene, check_for_blockage, make_navigation_decision, visualize_output

print("Loading models... This may take a moment.")
# Check for GPU (highly recommended for performance)
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using device: {device}")

# --- YOLOv11 for Object Detection ---
yolo_model = YOLO("yolo11n.pt")
yolo_model.to(device)

# --- OneFormer for Semantic Segmentation ---
oneformer_processor = OneFormerProcessor.from_pretrained("shi-labs/oneformer_ade20k_swin_large")
oneformer_model = OneFormerForUniversalSegmentation.from_pretrained("shi-labs/oneformer_ade20k_swin_large")
oneformer_model.to(device)

# Identify all plausible walkable surfaces from the model's configuration
walkable_keywords = ["floor", "path", "road", "sidewalk", "pavement", "ground"]
walkable_ids = [
    k for k, v in oneformer_model.config.id2label.items()
    if any(keyword in v.lower() for keyword in walkable_keywords)
]
print(f"Found walkable class IDs: {walkable_ids}")

print("Models loaded successfully.")

# --- Configuration ---
# Process every Nth frame to improve performance and prevent freezing.
# A lower number is more accurate but slower. A higher number is faster but less responsive.
FRAME_SKIP = 5 

# Use 0 for the default built-in webcam
cap = cv2.VideoCapture(0) 

if not cap.isOpened():
    print("Error: Could not open webcam.")
else:
    print("Webcam opened successfully. Press 'q' on the video window to quit.")
    frame_count = 0
    last_decision = "Initializing..."
    
    # Initialize variables before the loop to prevent reference-before-assignment on skipped frames
    ret, sample_frame = cap.read()
    if ret:
        H_init, W_init, _ = sample_frame.shape
        # Create empty placeholders for the analysis results
        safe_path_mask = np.zeros((H_init, W_init), dtype=np.uint8)
        human_boxes = []
        obstacle_boxes = []
    else:
        print("Error: Could not read the first frame from webcam.")
        cap.release() # Release the webcam if it can't be read

    try:
        # Set the capture back to the beginning if it was read from
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        
        while ret: # Continue as long as we can read frames
            ret, frame = cap.read()
            if not ret:
                print("End of video stream or webcam error.")
                break
            
            frame_count += 1
            
            # --- Performance Boost: Only process every Nth frame ---
            if frame_count % FRAME_SKIP == 0:
                # 1. Analyze the scene to get fresh data
                safe_path_mask, human_boxes, obstacle_boxes = analyze_scene(
                    frame, yolo_model, oneformer_processor, oneformer_model, walkable_ids, device
                )
                
                # 2. Make a new navigation decision and cache it
                decision = make_navigation_decision(safe_path_mask, human_boxes, obstacle_boxes, frame.shape[:2])
                last_decision = decision 
            
            # 3. Create the visualization using the most recent data
            # For skipped frames, this uses the cached data from the last processed frame for a smooth display
            output_frame = visualize_output(
                frame, safe_path_mask, human_boxes, obstacle_boxes, last_decision
            )
            
            # 4. Display the frame in a new window
            cv2.imshow('Real-time Navigation', output_frame)

            # 5. Check for 'q' key to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"An error occurred during the loop: {e}")
    finally:
        # Release everything when done
        cap.release()
        cv2.destroyAllWindows()
        print("Webcam released and windows closed.")

