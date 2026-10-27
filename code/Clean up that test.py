import numpy as np
import cv2
import rtde_control
import rtde_receive
from time import sleep
ROBOT_IP = "192.168.0.99"

rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

CAMERA_INDEX = 1
MIN_OBJECT_AREA = 300
def home():
    return [-0.13, -0.213, 0.05, 0.0, 3.14, 0.0]
def bin():
    return [0.03, -0.340, 0.05, 0.0, 3.14, 0.0]
def find_rubbish():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read camera.")
                return None
            # Convert image to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Detect blue regions
            mask = cv2.inRange(hsv,np.array([100, 100, 50]),np.array([140, 255, 255]))
            # Reduce noise
            mask = cv2.GaussianBlur(mask, (5, 5), 0)
            # Find contours
            contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            display = frame.copy()
            if contours:
                # Select the largest blue object
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)
                if area > MIN_OBJECT_AREA:
                    M = cv2.moments(largest)
                    if M["m00"] != 0:
                        # Object centre in pixels
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        # Object position relative to dish centre
                        cx_rel = cx - dish_x
                        cy_rel = cy - dish_y
                        # Convert pixels to millimetres
                        x_mm = cx_rel * mm_per_pixel
                        y_mm = -cy_rel * mm_per_pixel
                        # I have to recalculate this
                        # Convert millimetres to metres
                        offset = np.array([x_mm / 1000,y_mm / 1000,0,0,0,0])
                        # Calculate robot target pose
                        result = dish_robot + offset
                        # Display detected object
                        cv2.drawContours(display,[largest],-1,(0, 255, 0),2)
                        cv2.putText(display,f"Object: ({cx}, {cy})",(cx + 10, cy),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255, 255, 255),2)
                        print("Detected pixel position:", cx, cy)
                        print("Robot target pose:", result.tolist())
                        cv2.imshow("Detection Debug", display)
                        cv2.waitKey(1)
                        return result.tolist()
            cv2.imshow("Detection Debug", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Detection cancelled.")
                return None
    finally:
        cap.release()
        cv2.destroyAllWindows()
def close_gripper():
    return "Close gripper"
def open_gripper():
    return "Open gripper"
def pick_up(pose):
    # Approach 3 cm above the target
    approach = pose.copy()
    approach[2] += 0.03
    # Picking height
    pick = pose.copy()
    pick[2] -= 0.02
    print("Moving above rubbish")
    rtde_c.moveL(approach, 0.05, 0.1)
    sleep(0.3)
    print("Moving to picking position")
    rtde_c.moveL(pick, 0.03, 0.1)
    sleep(0.3)
    close_gripper()
    sleep(0.5)
    print("Lifting rubbish")
    rtde_c.moveL(approach, 0.05, 0.1)
    sleep(0.3)
def place(pose):
    approach = pose.copy()
    approach[2] += 0.03
    target = pose.copy()
    target[2] -= 0.02
    print("Moving above bin")
    rtde_c.moveL(approach, 0.05, 0.1)
    sleep(0.3)
    print("Moving to bin")
    rtde_c.moveL(target, 0.03, 0.1)
    sleep(0.3)
    open_gripper()
    sleep(0.5)
    print("Moving away from bin")
    rtde_c.moveL(approach, 0.05, 0.1)
    sleep(0.3)
def loop():
    try:
        print("Moving to home position")
        rtde_c.moveL(home(), 0.05, 0.1)
        while True:
            # Detect rubbish
            location = find_rubbish()
            if location is None:
                print("No rubbish detected. Stopping.")
                break
            print("Rubbish detected at:", location)
            # Move to rubbish
            pick_up(location)
            # Move to bin
            place(bin())
            # Return to home
            rtde_c.moveL(home(), 0.05, 0.1)
            print("Collection cycle complete")
    except KeyboardInterrupt:
        print("Program interrupted.")
    finally:
        # Stop any ongoing linear movement
        try:
            rtde_c.stopL(2.0)
        except Exception:
            pass

        rtde_c.disconnect()
        rtde_r.disconnect()
if __name__ == "__main__":
    print("Are you ready?")
    ready = input()
    if ready =="y":
        loop()