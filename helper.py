import cv2
import cv2
import numpy as np
import torch
from PIL import Image

def analyze_scene(frame, yolo_model, oneformer_processor, oneformer_model, walkable_ids, device):
    """Performs segmentation and object detection on a single frame."""
    # 1. Semantic Segmentation
    image_pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    inputs = oneformer_processor(images=image_pil, task_inputs=["semantic"], return_tensors="pt").to(device)
    with torch.no_grad():
        outputs = oneformer_model(**inputs)
    predicted_map = oneformer_processor.post_process_semantic_segmentation(outputs, target_sizes=[image_pil.size[::-1]])[0]
    walkable_mask = np.isin(predicted_map.cpu().numpy(), walkable_ids).astype(np.uint8)

    # 2. Object Detection
    yolo_results = yolo_model(frame, verbose=False)
    human_boxes = []
    obstacle_boxes = []
    for result in yolo_results:
        for box in result.boxes:
            class_id = int(box.cls[0])
            class_name = yolo_model.names[class_id]
            if class_name == 'person':
                human_boxes.append(box.xyxy[0].cpu().numpy().astype(int))
            else:
                obstacle_boxes.append(box.xyxy[0].cpu().numpy().astype(int))
    
    # 3. Combine Masks for "Safe Path"
    obstacle_mask = np.zeros_like(walkable_mask)
    all_obstacles = human_boxes + obstacle_boxes
    for box in all_obstacles:
        x1, y1, x2, y2 = box
        cv2.rectangle(obstacle_mask, (x1, y1), (x2, y2), 1, -1)
    safe_path_mask = cv2.bitwise_and(walkable_mask, cv2.bitwise_not(obstacle_mask))
    
    return safe_path_mask, human_boxes, obstacle_boxes

def check_for_blockage(boxes, region_x_start, region_x_end, frame_height):
    """Checks if any box overlaps with a specified vertical region in the bottom half of the frame."""
    for x1, y1, x2, y2 in boxes:
        # Consider only boxes that are at least partially in the lower (navigation) half of the frame
        if y2 > frame_height / 2:
            # Check for horizontal overlap with the central region
            # AABB overlap condition: (box1.left < box2.right) and (box1.right > box2.left)
            if x1 < region_x_end and x2 > region_x_start:
                return True # Found a blocking object
    return False # No blocking objects in the region    

def make_navigation_decision(safe_path_mask, human_boxes, obstacle_boxes, frame_shape):
    """Makes an intelligent navigation decision based on the safe path and detected objects."""
    H, W = frame_shape
    left_boundary = W // 3
    right_boundary = 2 * W // 3
    
    # Analyze safe path in the lower half of the frame (where the robot would move)
    navigation_area = safe_path_mask[H//2:, :]
    left_region = navigation_area[:, :left_boundary]
    center_region = navigation_area[:, left_boundary:right_boundary]
    right_region = navigation_area[:, right_boundary:]

    left_score = cv2.countNonZero(left_region)
    center_score = cv2.countNonZero(center_region)
    right_score = cv2.countNonZero(right_region)

    # Thresholds for making decisions
    center_threshold = center_region.size * 0.20  # 20% of center must be clear to move forward
    side_threshold = left_region.size * 0.15      # 15% of a side must be clear to consider turning

    # 1. Primary Check: Is the way forward clear based on the segmentation mask?
    if center_score > center_threshold:
        return "Move Forward"

    # 2. If blocked, determine the cause by checking for objects in the central path
    is_human_blocking = check_for_blockage(human_boxes, left_boundary, right_boundary, H)
    is_obstacle_blocking = check_for_blockage(obstacle_boxes, left_boundary, right_boundary, H)
    
    if is_human_blocking:
        return "Waiting for human..."
    
    if is_obstacle_blocking:
        # Path is blocked by a static object, suggest a more drastic action
        return "Obstacle Blockage. Scan 360"

    # 3. If blocked but not by a detected object (e.g., a wall), find an alternate path
    if left_score > right_score and left_score > side_threshold:
        return "Turn Left"
        
    if right_score > left_score and right_score > side_threshold:
        return "Turn Right"

    # 4. If no other option, declare a full stop
    return "STOP - FULLY BLOCKED"

def visualize_output(frame, safe_path_mask, human_boxes, obstacle_boxes, decision):
    """Draws all annotations and text onto the frame for display."""
    H, W, _ = frame.shape
    left_boundary = W // 3
    right_boundary = 2 * W // 3

    # Create a green overlay for the safe path
    safe_path_viz = np.zeros_like(frame)
    if safe_path_mask is not None:
        safe_path_viz[safe_path_mask == 1] = (0, 255, 0) # Green
    output_frame = cv2.addWeighted(frame, 0.7, safe_path_viz, 0.3, 0)

    # Draw boxes for humans (blue) and obstacles (red)
    for x1, y1, x2, y2 in human_boxes:
        cv2.rectangle(output_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(output_frame, "Human", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
    for x1, y1, x2, y2 in obstacle_boxes:
        cv2.rectangle(output_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(output_frame, "Obstacle", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Draw navigation guide lines
    cv2.line(output_frame, (left_boundary, H//2), (left_boundary, H), (255, 255, 0), 2)
    cv2.line(output_frame, (right_boundary, H//2), (right_boundary, H), (255, 255, 0), 2)
    
    # Display the final decision
    cv2.putText(output_frame, f"Decision: {decision}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)
    
    return output_frame