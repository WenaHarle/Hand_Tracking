import cv2
import mediapipe as mp
import numpy as np
import math
import serial
import time

# Arduino communication settings
ARDUINO_PORT = 'COM10'  # Change this to match your Arduino port
BAUD_RATE = 9600

class HandTracker:
    def __init__(self, mode=False, max_hands=2, detection_confidence=0.5, tracking_confidence=0.5):
        self.mode = mode
        self.max_hands = max_hands
        self.detection_confidence = detection_confidence
        self.tracking_confidence = tracking_confidence
        
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=self.mode,
            max_num_hands=self.max_hands,
            min_detection_confidence=self.detection_confidence,
            min_tracking_confidence=self.tracking_confidence
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # Finger tip and pip landmark IDs
        self.tip_ids = [4, 8, 12, 16, 20]  # Thumb, Index, Middle, Ring, Pinky tips
        self.pip_ids = [3, 6, 10, 14, 18]  # PIP joints for finger counting
        
    def find_hands(self, img, draw=True):
        """Find hands in the image and optionally draw landmarks"""
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        
        if self.results.multi_hand_landmarks:
            for hand_landmarks in self.results.multi_hand_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(
                        img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                    )
        return img
    
    def find_position(self, img, hand_no=0, draw=True):
        """Get landmark positions for a specific hand"""
        x_list = []
        y_list = []
        bbox = []
        self.landmark_list = []
        
        if self.results.multi_hand_landmarks:
            if hand_no < len(self.results.multi_hand_landmarks):
                my_hand = self.results.multi_hand_landmarks[hand_no]
                
                for id, lm in enumerate(my_hand.landmark):
                    h, w, c = img.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    x_list.append(cx)
                    y_list.append(cy)
                    self.landmark_list.append([id, cx, cy])
                    
                    if draw:
                        cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
                
                # Bounding box
                x_min, x_max = min(x_list), max(x_list)
                y_min, y_max = min(y_list), max(y_list)
                bbox = x_min, y_min, x_max, y_max
                
                if draw:
                    cv2.rectangle(img, (x_min - 20, y_min - 20), 
                                (x_max + 20, y_max + 20), (0, 255, 0), 2)
        
        return self.landmark_list, bbox
        
    def get_hand_type(self, hand_landmarks):
        """Determine if hand is left or right based on thumb position relative to other fingers"""
        if len(self.landmark_list) < 21:
            return "Right"  # Default
            
        # Get thumb tip and wrist positions
        thumb_tip = self.landmark_list[4]  # Thumb tip
        wrist = self.landmark_list[0]      # Wrist
        index_mcp = self.landmark_list[5]  # Index finger MCP
        
        # Calculate relative position
        # If thumb is to the right of the line from wrist to index MCP, it's likely a right hand
        # If thumb is to the left, it's likely a left hand
        
        thumb_x = thumb_tip[1]
        wrist_x = wrist[1]
        index_x = index_mcp[1]
        
        # Simple heuristic: if thumb is on the right side relative to index finger, it's right hand
        if thumb_x > index_x:
            return "Right"
        else:
            return "Left"
    
    def fingers_up(self, hand_type="Right"):
        """Determine which fingers are up"""
        fingers = []
        
        if len(self.landmark_list) != 0:
            # Thumb - different logic for left and right hand
            # For right hand: thumb up when tip is to the right of pip
            # For left hand: thumb up when tip is to the left of pip
            if hand_type == "Right":
                if self.landmark_list[self.tip_ids[0]][1] > self.landmark_list[self.tip_ids[0] - 1][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            else:  # Left hand
                if self.landmark_list[self.tip_ids[0]][1] < self.landmark_list[self.tip_ids[0] - 1][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
            
            # Four fingers - compare y coordinates
            for id in range(1, 5):
                if self.landmark_list[self.tip_ids[id]][2] < self.landmark_list[self.pip_ids[id]][2]:
                    fingers.append(1)
                else:
                    fingers.append(0)
        
        return fingers
    
    def calculate_finger_angles(self):
        """Calculate angles for each finger"""
        angles = []
        if len(self.landmark_list) != 0:
            # For each finger
            for finger_id in range(5):
                if finger_id == 0:  # Thumb
                    # Calculate angle between thumb tip, MCP and wrist
                    p1 = np.array([self.landmark_list[4][1], self.landmark_list[4][2]])  # Thumb tip
                    p2 = np.array([self.landmark_list[2][1], self.landmark_list[2][2]])  # Thumb MCP
                    p3 = np.array([self.landmark_list[0][1], self.landmark_list[0][2]])  # Wrist
                else:
                    # For other fingers, calculate angle between tip, PIP and MCP
                    tip_id = self.tip_ids[finger_id]
                    pip_id = tip_id - 2
                    mcp_id = tip_id - 3
                    p1 = np.array([self.landmark_list[tip_id][1], self.landmark_list[tip_id][2]])
                    p2 = np.array([self.landmark_list[pip_id][1], self.landmark_list[pip_id][2]])
                    p3 = np.array([self.landmark_list[mcp_id][1], self.landmark_list[mcp_id][2]])
                
                # Calculate angle
                radians = np.arctan2(p3[1] - p2[1], p3[0] - p2[0]) - np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
                angle = np.abs(radians * 180.0 / np.pi)
                
                # Normalize angle to 0-180 range
                if angle > 180.0:
                    angle = 360 - angle
                
                # Map the angle to servo range (0-180)
                servo_angle = int(np.interp(angle, [0, 180], [0, 180]))
                angles.append(servo_angle)
        
        # If not enough landmarks detected, return default angles
        while len(angles) < 5:
            angles.append(90)
        
        return angles

    def calculate_finger_open_close(self):
        """Return 180 if finger up (open), 0 if down (close) for each finger"""
        fingers = self.fingers_up(self.get_hand_type(self.landmark_list))
        # Jika tidak terdeteksi, default semua 0 (tutup)
        if not fingers or len(fingers) < 5:
            return [0, 0, 0, 0, 0]
        return [180 if f == 1 else 0 for f in fingers]

    def calculate_finger_open_close_binary(self):
        """Return 1 if finger up (open), 0 if down (close) for each finger"""
        fingers = self.fingers_up(self.get_hand_type(self.landmark_list))
        if not fingers or len(fingers) < 5:
            return [0, 0, 0, 0, 0]
        return [1 if f == 1 else 0 for f in fingers]

    def find_distance(self, p1, p2, img, draw=True, r=15, t=3):
        """Calculate distance between two landmarks"""
        if len(self.landmark_list) != 0:
            x1, y1 = self.landmark_list[p1][1:]
            x2, y2 = self.landmark_list[p2][1:]
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            
            if draw:
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), t)
                cv2.circle(img, (x1, y1), r, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (x2, y2), r, (255, 0, 255), cv2.FILLED)
                cv2.circle(img, (cx, cy), r, (0, 0, 255), cv2.FILLED)
            
            length = math.hypot(x2 - x1, y2 - y1)
            return length, img, [x1, y1, x2, y2, cx, cy]
        
        return 0, img, []

def main():
    # Initialize webcam
    cap = cv2.VideoCapture(0)
    cap.set(3, 1280)  # Width
    cap.set(4, 720)   # Height
    
    # Initialize hand tracker
    detector = HandTracker()
    
    print("Hand and Finger Tracking Started!")
    print("Controls:")
    print("- Press 'q' to quit")
    print("- Press 'c' to toggle finger counting display")
    print("- Press 'd' to toggle distance measurement between index and thumb")
    
    show_finger_count = True
    show_distance = False
    
    # Inisialisasi Arduino dengan try-except
    try:
        arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Koneksi ke Arduino berhasil")
    except Exception as e:
        print(f"Gagal koneksi ke Arduino: {e}")
        arduino = None
    
    while True:
        success, img = cap.read()
        if not success:
            break
        
        # Flip image horizontally for mirror effect
        img = cv2.flip(img, 1)
        
        # Find hands
        img = detector.find_hands(img)
        landmark_list, bbox = detector.find_position(img)
        
        if len(landmark_list) != 0:
            # Detect hand type (left or right)
            hand_type = detector.get_hand_type(landmark_list)
            # Gunakan status buka/tutup jari (1=open, 0=close)
            fingers_bin = detector.calculate_finger_open_close_binary()
            arduino_data = f"<{fingers_bin[0]},{fingers_bin[1]},{fingers_bin[2]},{fingers_bin[3]},{fingers_bin[4]}>\n"
            if arduino is not None:
                arduino.write(arduino_data.encode())
            # Display status buka/tutup pada layar
            for i, val in enumerate(fingers_bin):
                finger_name = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky'][i]
                status = 'OPEN' if val == 1 else 'CLOSE'
                cv2.putText(img, f"{finger_name}: {status}", (200, 50 + i * 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if val == 1 else (0, 0, 255), 2)
            
            # Finger counting
            if show_finger_count:
                fingers = detector.fingers_up(hand_type)
                total_fingers = fingers.count(1)
                
                # Display finger count
                cv2.rectangle(img, (20, 225), (170, 425), (0, 255, 0), cv2.FILLED)
                cv2.putText(img, str(total_fingers), (45, 375), 
                           cv2.FONT_HERSHEY_PLAIN, 10, (255, 0, 0), 25)
            
            # Distance measurement between thumb tip and index finger tip
            if show_distance:
                length, img, line_info = detector.find_distance(4, 8, img)
                cv2.putText(img, f"Distance: {int(length)}", (50, 50), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            
            # Display landmark coordinates for index finger tip
            if len(landmark_list) > 8:
                cv2.putText(img, f"Index Tip: ({landmark_list[8][1]}, {landmark_list[8][2]})", 
                           (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display instructions
        cv2.putText(img, "Press 'q': Quit | 'c': Toggle Count | 'd': Toggle Distance", 
                   (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Show image
        cv2.imshow("Hand and Finger Tracking", img)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            show_finger_count = not show_finger_count
        elif key == ord('d'):
            show_distance = not show_distance
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    if arduino is not None:
        arduino.close()

if __name__ == "__main__":
    main()
