import time
import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_tasks_python
from mediapipe.tasks.python import vision as mp_tasks_vision
import serial

# -----------------------
# Initialize Arduino (robust open + reopen on PermissionError)
# -----------------------
com_port = 'COM4'  # CHANGE COM PORT IF NEEDED
arduino = None
last_open_attempt = 0.0

def open_arduino():
    global arduino, last_open_attempt
    last_open_attempt = time.time()
    try:
        arduino = serial.Serial(com_port, 115200, timeout=1)
        time.sleep(2)
        print(f"Opened serial port {com_port}")
    except Exception as e:
        arduino = None
        print(f"Warning: Arduino not connected: {e}")

# try initial open
open_arduino()

# -----------------------
# Initialize MediaPipe
# -----------------------
# Use MediaPipe Tasks HandLandmarker (model file included in workspace)
model_path = "hand_landmarker.task"
options = mp_tasks_vision.HandLandmarkerOptions(
    base_options=mp_tasks_python.BaseOptions(model_asset_path=model_path),
    running_mode=mp_tasks_vision.RunningMode.VIDEO,
    num_hands=1,
    min_hand_detection_confidence=0.7,
    min_hand_presence_confidence=0.7,
    min_tracking_confidence=0.7,
)
hand_landmarker = mp_tasks_vision.HandLandmarker.create_from_options(options)

# drawing utils
drawing_utils = mp_tasks_vision.drawing_utils
HandConnections = mp_tasks_vision.HandLandmarksConnections

# -----------------------
# Servo Settings
# -----------------------
servo_pos = 90
servo_min = 0
servo_max = 180
step = 10
# Gesture update rate control
previous_gesture = "NEUTRAL"
last_move_time = 0.0
last_sent_pos = None
move_interval = 0.15  # seconds between incremental moves while holding gesture

# -----------------------
# Servo Move Function
# -----------------------
def send_servo_command(pos):
    """Send servo position to Arduino. Returns True if successful."""
    global arduino
    if not arduino:
        return False
    try:
        arduino.write(f"{pos}\n".encode())
        return True
    except PermissionError as e:
        print(f"Arduino error (PermissionError): {e} — attempting to reopen port.")
        try:
            arduino.close()
        except Exception:
            pass
        arduino = None
        open_arduino()
        if arduino:
            try:
                arduino.write(f"{pos}\n".encode())
                return True
            except Exception as e2:
                print(f"Arduino error after reopen: {e2}")
        return False
    except serial.SerialException as e:
        print(f"Arduino serial error: {e}")
        try:
            arduino.close()
        except Exception:
            pass
        arduino = None
        return False
    except Exception as e:
        print(f"Arduino error: {e}")
        return False

def move_servo(pos):
    """Command servo to position, with retry logic."""
    pos = max(servo_min, min(servo_max, int(pos)))
    global arduino, last_open_attempt
    
    if send_servo_command(pos):
        print(f"Servo Position: {pos}")
    else:
        now = time.time()
        if now - last_open_attempt > 5:
            open_arduino()
        print(f"Servo Position: {pos} (NOT SENT - no Arduino)")

# -----------------------
# Camera Setup
# -----------------------
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

if not cap.isOpened():
    print("Camera not accessible")
    exit()

# -----------------------
# Main Loop
# -----------------------
try:
    while True:
        success, img = cap.read()
        if not success:
            break

        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, _ = img.shape
        # Create MediaPipe Image and run hand landmarker
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=imgRGB)
        timestamp_ms = int(time.time() * 1000)
        results = hand_landmarker.detect_for_video(mp_image, timestamp_ms)

        gesture_text = "NO HAND"

        if results and results.hand_landmarks:
            for hand_landmarks in results.hand_landmarks:
                # draw landmarks on the BGR image
                drawing_utils.draw_landmarks(
                    img,
                    hand_landmarks,
                    HandConnections.HAND_CONNECTIONS,
                )

                lmList = []
                for id, lm in enumerate(hand_landmarks):
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    lmList.append([id, cx, cy])

                # Thumb & Index tips (landmark indices 4 and 8)
                thumbX, thumbY = lmList[4][1], lmList[4][2]
                indexX, indexY = lmList[8][1], lmList[8][2]

                distance = ((indexX - thumbX)**2 + (indexY - thumbY)**2) ** 0.5

                # Draw line & center
                cv2.line(img, (thumbX, thumbY), (indexX, indexY), (255, 0, 0), 3)
                cv2.circle(img, ((thumbX + indexX)//2, (thumbY + indexY)//2),
                           5, (255, 0, 0), -1)

                # Gesture Logic (debounced incremental moves)
                now = time.time()
                if distance < 40:
                    gesture_text = "DOWN"
                    if now - last_move_time >= move_interval:
                        servo_pos -= step
                        last_move_time = now
                elif distance > 80:
                    gesture_text = "UP"
                    if now - last_move_time >= move_interval:
                        servo_pos += step
                        last_move_time = now
                else:
                    gesture_text = "NEUTRAL"
                    # reset timing so next gesture can trigger immediately
                    if previous_gesture != "NEUTRAL":
                        last_move_time = 0.0

                previous_gesture = gesture_text
                print(f"Detected: {gesture_text}, Distance: {distance:.1f}, Servo: {servo_pos}")

        # Limit servo range and only send when changed
        servo_pos = max(servo_min, min(servo_max, servo_pos))
        if last_sent_pos != servo_pos:
            move_servo(servo_pos)
            last_sent_pos = servo_pos

        # -----------------------
        # DISPLAY TEXT (UPPER-LEFT) - APPLE-CLEAN HUD
        # -----------------------
        hud_x, hud_y = 15, 15
        hud_w, hud_h = 340, 140

        overlay = img.copy()

        # Soft white glass panel
        cv2.rectangle(
            overlay,
            (hud_x, hud_y),
            (hud_x + hud_w, hud_y + hud_h),
            (245, 245, 245), -1
        )

        # Blend (frosted effect)
        img = cv2.addWeighted(overlay, 0.75, img, 0.25, 0)

        # Title (Subtle, macOS-style)
        cv2.putText(
            img, "Hand Tracker",
            (hud_x + 18, hud_y + 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75, (60, 60, 60), 2
        )

        # Thin divider line
        cv2.line(
            img,
            (hud_x + 18, hud_y + 42),
            (hud_x + hud_w - 18, hud_y + 42),
            (200, 200, 200), 1
        )

        # Gesture text (Soft emphasis with conditional color)
        gesture_color = (60, 60, 60)
        if gesture_text != "NO HAND":
            gesture_color = (0, 120, 255)  # Apple blue

        cv2.putText(
            img, f"Gesture  {gesture_text}",
            (hud_x + 18, hud_y + 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, gesture_color, 2
        )

        # Servo position (matching style)
        cv2.putText(
            img, f"Servo  {servo_pos}°",
            (hud_x + 18, hud_y + 102),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (60, 60, 60), 2
        )

        # Show Window
        cv2.imshow("Hand Tracker", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    if arduino:
        arduino.close()
    cap.release()
    cv2.destroyAllWindows()
