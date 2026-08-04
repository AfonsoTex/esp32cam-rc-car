import socket
import cv2
import numpy as np
import pygame
import time
import datetime
import threading    

# ==========================================
# NETWORK CONFIGURATION
# ==========================================
HOST = '0.0.0.0'      # listen on ANY of this PC's network interfaces (not just one)
PORT_CONTROL = 1883   # TCP port for control: MOV/DIR commands and heartbeat
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
    global latest_frame  #otherwise creates local variables; we need global for thread communication

    server_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Added: Creates a UDP socket to receive the camera's video stream
    server_video.bind(('0.0.0.0', 1884))                             # Added: Binds the socket to all local interfaces on port 1884
    
    print("Waiting for video on UDP port 1884 (Accumulation Mode)...")
    
    frame_buffer = bytearray()
    # Used to accumulate binary packets received from the network until a complete JPEG file is reconstructed. 
    # The code monitors incoming data in the buffer looking for JPEG start and end markers. 
    
    while True:
        try:  # Prevents the thread from crashing if a network error or data glitch occurs

            data, addr = server_video.recvfrom(65535)  # Receives up to 65535 bytes of binary data and the sender's address from the UDP socket
            
            if not data:
                continue
            
            # Checks if it is the start of a JPEG frame (Universal FF D8 markers for JPEG images)
            if data[0] == 0xFF and data[1] == 0xD8:
                frame_buffer = bytearray(data)  # When the start marker is detected, it means a new frame has begun transmitting. 
                                                #The code discards any previous junk data and reinitializes the frame_buffer with 
                                                #the data from this new packet.
            else:
                frame_buffer.extend(data)  # Executes if the first two bytes of 
                                           #the packet do not match the FF D8 marker. 
                                           #This means the current packet is a continuation (middle or end) of the video 
                                           #frame currently being received.
            
            # This packet ends a JPEG frame if its last two bytes are FF D9 (EOI).
            # len(data) >= 2 guards against a 1-byte packet where data[-2] would fail.
            if len(data) >= 2 and data[-2] == 0xFF and data[-1] == 0xD9:

                # Reinterpret the accumulated bytes as a NumPy array (no copy),
                # then decompress the JPEG into a pixel image.
                img_array = np.frombuffer(frame_buffer, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                # None means the JPEG was corrupt (e.g. a lost UDP packet) — skip it.
                if frame is not None:

                # Hand the frame to the processing thread, under the lock.
                # Safely passes a deep copy of the frame to the processing thread. 
                # Without .copy(), both threads would share the exact same memory reference, 
                # causing data corruption if the video thread overwrites the frame while the 
                # processing thread is reading it.
                    with frame_lock:
                        latest_frame = frame.copy()
                    #the processing thread will handle the display

                # New bytearray, not .clear(): NumPy above points at these same
                # bytes. .clear() would wipe the bytes NumPy still needs (garbage).
                # Reassigning leaves the old buffer intact for NumPy and starts a
                # fresh one here — the old one lives on until NumPy is done with it.
                frame_buffer = bytearray()

        except Exception as e:
            print(f"Video receive error: {e}")
            break

    server_video.close()

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
    
    # Inicia a Thread 1 (Receção de Vídeo)
    t_rx = threading.Thread(target=video_rx_thread, daemon=True)
    t_rx.start()

    # Inicia a Thread 2 (Processamento de Imagem)
    t_proc = threading.Thread(target=image_processing_thread, daemon=True)
    t_proc.start()

    pygame.init()
    pygame.joystick.init()
    joystick = None  # Placeholder for the gamepad/controller instance

    # Create a TCP/IP socket for control communication
    server_control = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Allow immediate reuse of the port to prevent "Address already in use" errors on restart
    server_control.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind the socket to the defined host and control port
    server_control.bind((HOST, PORT_CONTROL))
    
    # Listen for incoming connection requests (maximum queue of 1)
    server_control.listen(1)
    
    # Set a 2.0-second timeout on the server socket:
    # 1. Without a timeout, server_control.accept() blocks execution indefinitely 
    #    while waiting for a client connection, freezing the program.
    # 2. With settimeout(2.0), if no client connects within 2 seconds, the method 
    #    raises a timeout exception instead of locking the thread permanently, 
    #    allowing the main loop to handle other events.
    server_control.settimeout(2.0)
    
    print(f"[{timestamp()}] Control Server active on port {PORT_CONTROL}")

    try:
        while True:
            # Poll Pygame events to handle dynamic controller connection or disconnection
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

            # Wait for an incoming client connection with a 2-second timeout
            try:
                print(f"[{timestamp()}] Waiting for ESP32 on control port...")
                conn, addr = server_control.accept()
                print(f"[{timestamp()}] ESP32 Control connected: {addr}")
            except socket.timeout:
                continue

            previous_command = None
            last_sent_time = time.time()

            # Active connection loop for handling game controller inputs
            try:
                while True:
                    for event in pygame.event.get():
                        # Toggle autonomous/manual mode when button 0 is pressed
                        if event.type == pygame.JOYBUTTONDOWN and event.button == 0:
                            autonomous_mode = not autonomous_mode
                            mode_str = "AUTOMATIC" if autonomous_mode else "MANUAL"
                            print(f"\n>>> [{timestamp()}] MODE CHANGED TO: {mode_str} <<<\n")

                    if autonomous_mode:
                        # --------------------------------------------------
                        # AUTONOMOUS MODE CONTROL
                        # --------------------------------------------------
                        # Retrieve current direction and move commands safely from shared threads using the state lock
                        with state_lock:
                            direction = auto_direction
                            move_cmd = auto_move

                        # Check if the robot is reversing (negative move command)
                        if move_cmd < 0:
                            # 1. If negative, maintain reverse movement speed
                            move = move_cmd
                        else:
                            # 2. Calculate dynamic curve acceleration only when moving FORWARD
                            abs_dir = abs(direction)
                            
                            # If the steering deviation exceeds the defined curve threshold, scale the speed up
                            if abs_dir > CURVE_THRESHOLD:
                                strength_factor = (abs_dir - CURVE_THRESHOLD) / (1.0 - CURVE_THRESHOLD)
                                move = BASE_SPEED + strength_factor * (MAX_CURVE_SPEED - BASE_SPEED)
                            else:
                                # Otherwise, maintain standard baseline cruising speed
                                move = BASE_SPEED
                    else:
                        # --------------------------------------------------
                        # MANUAL MODE CONTROL (GAMEPAD / JOYSTICK)
                        # --------------------------------------------------
                        if joystick is not None:
                            pygame.event.pump()  # Refresh internal joystick states
                            
                            # Read raw axes from the gamepad (Axis 0 for steering, Axes 5 and 4 for triggers)
                            dir_joy = joystick.get_axis(0)
                            accelerate = joystick.get_axis(5)
                            brake = joystick.get_axis(4)

                            # Normalize trigger inputs to standard ranges
                            accelerate_norm = normalize(accelerate)
                            brake_norm = normalize(brake)

                            # Apply deadzone filtering to eliminate minor joystick centering jitter
                            if abs(dir_joy) < 0.1:
                                dir_joy = 0.0

                            # Calculate movement as the net difference between acceleration and braking triggers
                            move = accelerate_norm - brake_norm
                            direction = dir_joy
                        else:
                            # Fallback state if manual mode is active but no controller is physically connected
                            move = 0.0
                            direction = 0.0

                    # Format the final telemetry command string to send to the robot over TCP
                    command = f"MOV:{move:.2f},DIR:{direction:.2f}\n"

                    try:
                        # Send command only if it changes or if a heartbeat interval has passed
                        if command != previous_command:
                            print(f"[{'AUTO' if autonomous_mode else 'MANUAL'}] Sending: {command.strip()}")
                            conn.send(command.encode())
                            previous_command = command
                            last_sent_time = time.time()

                        elif time.time() - last_sent_time > 0.2:
                            # Send a heartbeat packet if no movement command changed for over 200ms
                            conn.send(b'HB\n')
                            last_sent_time = time.time()

                    except OSError as e:
                        print(f"[{timestamp()}] Control connection lost: {e}")
                        break

                    time.sleep(0.05)

            finally:
                conn.close()

    except KeyboardInterrupt:
        print("\nServer shutting down...")
    finally:
        server_control.close()
        pygame.quit()
        print("Resources released")

if __name__ == "__main__":
    main()