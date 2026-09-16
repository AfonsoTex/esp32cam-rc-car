import socket
import secrets
import cv2
import numpy as np
import pygame
import time
import datetime
import threading    

from control_protocol import (
    CONTROL_PORT,
    UINT32_MAX,
    encode_control_command,
    resolve_esp32_address,
)
from video_protocol import (
    VIDEO_DATAGRAM_MAX_SIZE,
    VideoFrameAssembler,
)

# ==========================================
# NETWORK CONFIGURATION
# ==========================================
HOST = '0.0.0.0'      # listen on ANY of this PC's network interfaces (not just one)
PORT_CONTROL = CONTROL_PORT  # UDP discovery and ESP32 control port
PORT_VIDEO = 1884     # UDP port for the camera's JPEG frames
# (IP picks the machine; the port picks which service on it — like building + apartment)

# ==========================================
# LINE FOLLOWER AND VISION PARAMETERS
# Tune these to your own car, floor, and tape. Every value is a starting
# point, not a universal setting — see the note on the battery below.
# ==========================================

GAIN = 2                  # steering strength.
                          # Not turning enough / drifting off the line -> raise it.
                          # Snaking side to side -> lower it.

DEADBAND = 8              # dead zone (pixels) around the centre where error is ignored,
                          # so the car doesn't twitch on a straight from tiny noise.
                          # Twitchy on straights -> raise it.

BASE_SPEED = 0.45         # normal forward speed on straights.

MAX_CURVE_SPEED = 0.6     # max speed in curves, to give the outer side torque to turn.

CURVE_THRESHOLD = 0.70    # steering level above which the car starts adding curve speed.

REVERSE_SPEED = 0.49      # speed while reversing during recovery.


# --- IMAGE PROCESSING ---

THRESH_VAL = 60           # grayscale cutoff (0-255): above this becomes white.
                          # (Only if using a global threshold, not adaptive.)

MAX_WIDTH_PCT = 0.65      # max contour width (% of image); wider than this is treated
                          # as a reflection / floor patch and rejected.

MIN_AREA = 600            # min contour area (pixels) to count as the line;
                          # smaller blobs are noise. Line rejected when far/thin -> lower it.

# ==========================================
# SHARED STATE (global variables passed between threads)
# ==========================================
autonomous_mode = False  # mode switch: False = manual (gamepad), True = autonomous
auto_direction = 0.0     # steering the vision computes (-1..1), written by the
                         # processing thread and read by main to send to the ESP
state = "FOLLOW"         # state machine: "FOLLOW" or "REVERSE"
auto_move = 0.0          # speed the vision commands (-1..1); negative = reverse
                         # (needed so recovery can drive the car backwards)



# Two independent hand-offs between threads, so two separate locks:
#   - the image: receive thread -> processing thread    (frame_lock)
#   - direction & speed (auto_direction, auto_move):
#       processing thread -> main                        (state_lock)
# Separate locks so protecting one doesn't block the other.
latest_frame = None            # holds ONLY the most recent captured frame
frame_lock = threading.Lock()
state_lock = threading.Lock()
esp32_control_address = None
control_address_lock = threading.Lock()


def control_discovery_thread():
    """Discover the ESP32 from valid UDP HELLO packets."""
    global esp32_control_address

    discovery_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discovery_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    discovery_socket.bind((HOST, PORT_CONTROL))

    print(f"Waiting for ESP32 HELLO on UDP port {PORT_CONTROL}...")

    while True:
        try:
            payload, sender = discovery_socket.recvfrom(256)
        except OSError as error:
            print(f"[{timestamp()}] UDP discovery error: {error}")
            continue

        address = resolve_esp32_address(payload, sender)
        if address is None:
            continue

        with control_address_lock:
            address_changed = address != esp32_control_address
            esp32_control_address = address

        if address_changed:
            print(
                f"[{timestamp()}] ESP32 discovered by UDP HELLO: "
                f"{esp32_control_address}"
            )



def timestamp():
    # Current time as text (HH:MM:SS.mmm) for debug prints, so you can see
    # WHEN each event happened and spot if the video stalls (times stop advancing).
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def normalize(value):
    # Map a -1..1 value to 0..1: -1 -> 0, 0 -> 0.5, 1 -> 1.
    # Used when something needs a 0..1 scale from a value that lives in -1..1.
    return (value + 1) / 2
    #example: convert xbox commands to velocity



def video_rx_thread():
    global latest_frame

    server_video = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    )
    server_video.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1,
    )
    server_video.bind((HOST, PORT_VIDEO))

    assembler = VideoFrameAssembler(timeout_seconds=0.5)

    print(
        f"Waiting for fragmented video on UDP port "
        f"{PORT_VIDEO}..."
    )

    while True:
        try:
            datagram, sender = server_video.recvfrom(
                VIDEO_DATAGRAM_MAX_SIZE + 1
            )

            with control_address_lock:
                control_address = esp32_control_address

            if (
                control_address is None
                or sender[0] != control_address[0]
            ):
                continue

            completed = assembler.add_datagram(datagram)

            if completed is None:
                continue

            frame_id, jpeg = completed

            if (
                not jpeg.startswith(b"\xff\xd8")
                or not jpeg.endswith(b"\xff\xd9")
            ):
                continue

            image_data = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(
                image_data,
                cv2.IMREAD_COLOR,
            )

            if frame is None:
                print(
                    f"[{timestamp()}] Video frame "
                    f"{frame_id} could not be decoded"
                )
                continue

            with frame_lock:
                latest_frame = frame.copy()

        except OSError as error:
            print(f"[{timestamp()}] Video socket error: {error}")
            continue

        except Exception as error:
            print(f"[{timestamp()}] Video receive error: {error}")
            continue


# ==========================================
# THREAD 2: OPENCV PROCESSING
# ==========================================
def image_processing_thread():
    global auto_direction, state, auto_move, latest_frame
    
    print(f"[{timestamp()}] Image Processing Thread Active")

    last_error = 0.0  # Initialized for tracking the previous error in control algorithms, even if not currently in active use
    last_cx = 320     # Stores the last known X coordinate of the line; initialized to 320 (half of the 640 resolution width)

    while True:
        frame_to_process = None

        # Safely accesses the shared global frame variable using a thread lock
        with frame_lock:
            if latest_frame is not None:
                frame_to_process = latest_frame.copy()  # Creates a local copy for processing
                latest_frame = None                     # Clears the global variable to avoid processing duplicates

        # If no new frame arrived, pauses briefly to prevent 100% CPU usage and restarts the loop
        if frame_to_process is None:
            time.sleep(0.005)
            continue

        # Standardize resolution (acts as a safeguard to guarantee 640x480 even if camera config changes)
        frame = cv2.resize(frame_to_process, (640, 480))
        height, width, _ = frame.shape

        # Define the Region of Interest (ROI) at the bottom of the frame where the line is visible
        roi_top = int(height * 0.80)
        roi_bottom = int(height * 0.95)
        roi = frame[roi_top:roi_bottom, :]

        # --- GRAYSCALE PROCESSING ---
        # Convert the cropped region to grayscale to eliminate unnecessary color channels
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Apply adaptive thresholding to binarize the image, handling lighting variations and inverting colors
        thresh = cv2.adaptiveThreshold(
            gray_roi, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            151,  # Block size: size of the pixel neighborhood used to calculate the local threshold (must be an odd number)
            15    # Constant C: value subtracted from the calculated mean to fine-tune sensitivity and eliminate background noise
        )

        # Creates an elliptical structural element (kernel) with a size of 9x9 pixels to be used in morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        
        # Opening operation: removes small isolated noise by eroding away boundary pixels, 
        # then dilates the remaining structure back to its original size.
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # --- CONTOUR DETECTION AND FILTERING ---
        # Scans the binary image to detect continuous outer boundaries, 
        # returning a list of contour point arrays (contours) and hierarchical tree data (_)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_contour = None          # Stores the selected target line contour
        min_distance = float('inf')  # Initializes distance tracking to infinity for subsequent minimum comparisons

        # Sort detected contours by area in descending order, putting the largest shapes first
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for c in contours:
            area = cv2.contourArea(c)
            
            # Discard contours smaller than the minimum area threshold
            if area < MIN_AREA:
                continue

            x, y, w, h = cv2.boundingRect(c)

            # Discard contours that are too wide to be the track line
            if w > (width * MAX_WIDTH_PCT):
                continue

            # Discard contours that are too short vertically
            if h < 30:
                continue

            # Calculate spatial moments to find the geometric properties of the contour
            M_temp = cv2.moments(c)
            
            # - M["m00"]: Zeroth-order spatial moment, representing the total area (sum of all pixels within the contour).
            if M_temp["m00"] == 0:
                continue
            
            # - M["m10"]: First-order spatial moment along the x-axis, representing the sum of the x-coordinates of all pixels in the contour.
            #   Dividing m10 by m00 (the total pixel count) calculates the average x-position, yielding the horizontal center (cx = m10 / m00).
            cx_temp = int(M_temp["m10"] / M_temp["m00"])
            distance = abs(cx_temp - last_cx)

            # Reject the contour if it appears more than 150 pixels away from the last known location,
            # as it is physically impossible for the true line to jump that far within a single frame.
            if distance > 150:
                continue

            # If multiple valid contours exist (e.g., a broken line), select the one closest to the last known position
            if distance < min_distance:
                min_distance = distance
                line_contour = c

        # ==========================================
        # MOVEMENT LOGIC AND RECOVERY MODE
        # ==========================================
        if line_contour is not None:
            state = "FOLLOW"
            
            # Calculate moments of the selected line contour to find its center coordinates
            M = cv2.moments(line_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                last_cx = cx  # Update memory with the real horizontal coordinate
                
                roi_center = width // 2
                error = cx - roi_center
                last_error = error

                max_error = width / 2

                # Apply deadband check to eliminate minor steering jitters when near the center
                if abs(error) <= DEADBAND:
                    dir_normal = 0.0
                else:
                    # Calculate adaptive proportional gain:
                    # 1. Normalize the maximum pixel error (1.0 / max_error)
                    # 2. Scale by the user-defined GAIN factor
                    Kp = (1.0 / (width / 2)) * GAIN
                    
                    # Compute the final steering command and clamp it safely between -1.0 and 1.0:
                    # 1. Kp * error: Multiplies the proportional gain by the pixel error to convert 
                    #    the distance into a proportional motor control signal (e.g., 0.0 for center, 
                    #    0.5 for half-deviation, and 1.0 for maximum edge error).
                    #
                    # 2. min(1.0, Kp * error): Compares the calculated signal with 1.0 and selects 
                    #    the smaller value, imposing a strict upper ceiling to prevent values above 1.0 
                    #    if the pixel error exceeds the expected maximum.
                    #
                    # 3. max(-1.0, ...): Takes the resulting value and compares it with -1.0, 
                    #    selecting the larger value to impose a strict lower floor, ensuring negative 
                    #    commands never drop below -1.0.
                    dir_normal = max(-1.0, min(1.0, Kp * error))

                # Safely update global motor control variables using a thread lock
                with state_lock:
                    auto_direction = dir_normal
                    auto_move = BASE_SPEED

                # Draw tracking markers on the images for visual debugging
                cv2.circle(roi, (cx, cy), 6, (0, 255, 0), -1)
                cv2.circle(thresh, (cx, cy), 6, 128, -1)

        else:
            # LINE NOT FOUND - Recovery Mode (Backs up and steers towards the last known error side)
            state = "REVERSE"
            
            with state_lock:
                auto_move = -REVERSE_SPEED
                
                # If the last known error was negative, the line was to the left, 
                # so steer right (1.0) while reversing to recapture it. 
                # Otherwise, steer left (-1.0).
                if last_error < 0:
                    auto_direction = 1.0
                else:
                    auto_direction = -1.0

        # Draw the Region of Interest boundary rectangle on the main frame and display the windows
        cv2.rectangle(frame, (0, roi_top), (width, roi_bottom), (0, 0, 255), 2)
        cv2.imshow('ESP32-CAM', frame)
        cv2.imshow('Processed ROI', thresh)
        
        # Check if the 'q' key is pressed to exit the processing loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release all OpenCV GUI windows upon exiting the thread
    cv2.destroyAllWindows()

# ==========================================
# MAIN: SERVIDOR DE CONTROLO E INTERFACE PYGAME
# ==========================================
def main():
    global autonomous_mode, auto_direction, auto_move

    t_discovery = threading.Thread(
        target=control_discovery_thread,
        daemon=True,
    )
    t_discovery.start()

    t_rx = threading.Thread(target=video_rx_thread, daemon=True)
    t_rx.start()

    t_proc = threading.Thread(target=image_processing_thread, daemon=True)
    t_proc.start()

    pygame.init()
    pygame.joystick.init()
    joystick = None

    udp_control_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control_session = secrets.randbits(32) or 1
    control_sequence = 0
    last_logged_state = None

    print(f"[{timestamp()}] UDP control session: {control_session}")

    def advance_control_identity():
        nonlocal control_session, control_sequence

        if control_sequence < UINT32_MAX:
            control_sequence += 1
            return

        previous_session = control_session
        while control_session == previous_session:
            control_session = secrets.randbits(32) or 1
        control_sequence = 0

        print(f"[{timestamp()}] UDP control session rotated: {control_session}")

    def send_udp_state(address, move, direction):
        payload = encode_control_command(
            control_session,
            control_sequence,
            move,
            direction,
        )

        try:
            udp_control_sender.sendto(payload, address)
        except OSError as error:
            print(f"[{timestamp()}] UDP control send failed: {error}")
            return False

        advance_control_identity()
        return True

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    if joystick is not None:
                        joystick.quit()
                    joystick = pygame.joystick.Joystick(event.device_index)
                    joystick.init()
                    print(f"[{timestamp()}] Controller connected!")

                elif event.type == pygame.JOYDEVICEREMOVED:
                    if joystick is not None:
                        joystick.quit()
                        joystick = None
                        print(f"[{timestamp()}] Controller disconnected!")

                elif event.type == pygame.JOYBUTTONDOWN and event.button == 0:
                    autonomous_mode = not autonomous_mode
                    mode_name = "AUTOMATIC" if autonomous_mode else "MANUAL"
                    print(
                        f"\n>>> [{timestamp()}] "
                        f"MODE CHANGED TO: {mode_name} <<<\n"
                    )

            if autonomous_mode:
                with state_lock:
                    direction = auto_direction
                    move_cmd = auto_move

                if move_cmd < 0:
                    move = move_cmd
                else:
                    abs_dir = abs(direction)

                    if abs_dir > CURVE_THRESHOLD:
                        strength_factor = (
                            (abs_dir - CURVE_THRESHOLD)
                            / (1.0 - CURVE_THRESHOLD)
                        )
                        move = (
                            BASE_SPEED
                            + strength_factor
                            * (MAX_CURVE_SPEED - BASE_SPEED)
                        )
                    else:
                        move = BASE_SPEED
            else:
                if joystick is not None:
                    pygame.event.pump()

                    dir_joy = joystick.get_axis(0)
                    accelerate = joystick.get_axis(5)
                    brake = joystick.get_axis(4)

                    accelerate_norm = normalize(accelerate)
                    brake_norm = normalize(brake)

                    if abs(dir_joy) < 0.1:
                        dir_joy = 0.0

                    move = accelerate_norm - brake_norm
                    direction = dir_joy
                else:
                    move = 0.0
                    direction = 0.0

            state_to_log = (
                "AUTO" if autonomous_mode else "MANUAL",
                round(move, 2),
                round(direction, 2),
            )
            if state_to_log != last_logged_state:
                print(
                    f"[{state_to_log[0]}] Current state: "
                    f"MOV:{state_to_log[1]:.2f},"
                    f"DIR:{state_to_log[2]:.2f}"
                )
                last_logged_state = state_to_log

            with control_address_lock:
                udp_address = esp32_control_address

            if udp_address is not None:
                send_udp_state(udp_address, move, direction)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nServer shutting down...")

    finally:
        with control_address_lock:
            udp_address = esp32_control_address

        if udp_address is not None:
            for _ in range(3):
                send_udp_state(udp_address, 0.0, 0.0)
                time.sleep(0.02)

        if joystick is not None:
            joystick.quit()

        udp_control_sender.close()
        pygame.quit()
        print("Resources released")


if __name__ == "__main__":
    main()