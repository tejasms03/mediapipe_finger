# full_sim_with_model_based_control.py
import pygame
import numpy as np
import cv2
import mediapipe as mp
import math
import time

# ------------------- MEDIAPIPE SETUP -------------------
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils
HAND_CONNECTIONS = mp_hands.HAND_CONNECTIONS

def angle(a, b, c):
    ang = math.degrees(
        math.atan2(c[1]-b[1], c[0]-b[0]) -
        math.atan2(a[1]-b[1], a[0]-b[0])
    )
    return abs(ang)

def get_point(lm, img_w, img_h):
    return int(lm.x * img_w), int(lm.y * img_h)

def bend_percent(raw_angle):
    raw_angle = max(30, min(180, raw_angle))
    return float((180 - raw_angle) / (180 - 30) * 100)

# ------------------- SIM / MECH PARAMETERS (ASSUMED) ------------
# Each finger: 3 links (proximal, middle, distal)
FINGER_NAMES = ["Index"]
L = [np.array([60,40,30],dtype=float) for _ in FINGER_NAMES]   # link lengths (pixels)
K = [np.array([0.5,0.4,0.2],dtype=float) for _ in FINGER_NAMES] # dorsal spring stiffness (Nm/rad) - visual scaling
r = [np.array([0.008,0.005,0.003],dtype=float) for _ in FINGER_NAMES] # tendon moment arms (m)
R = 0.01  # motor pulley radius (m)
theta_close = [np.array([np.pi/2,np.pi/2,np.pi/2],dtype=float) for _ in FINGER_NAMES]

# static split coefficients & phi_max per finger (same approach used in sim)
alpha = [R * (r[i] / K[i]) / np.sum(r[i]**2 / K[i]) for i in range(1)]
phi_max = [np.dot(r[i], theta_close[i]) / R for i in range(1)]

# Plant parameters for computed-torque model (example values)
# These are per-finger (single DOF motor/rotating bar approximations)
J_vals = [0.00133]*4   # moment of inertia (kg*m^2) example
b_vals = [0.01]*4      # viscous damping (N*m*s/rad)
k_vals = [0.5]*4       # torsional spring stiffness (N*m/rad) - from dorsal springs

# actuator torque limit
TAU_MAX = 1.0  # N*m (sat)

# controller design (same for each finger, but could be individualized)
omega_n = 6.0   # desired natural frequency (rad/s) for smooth response
zeta = 1.0      # damping ratio >=1 for minimal overshoot

Kp_vals = [J_vals[i] * (omega_n**2) for i in range(4)]
Kd_vals = [max(0.0, 2*zeta*J_vals[i]*omega_n - b_vals[i]) for i in range(4)]
# small Ki can be added if needed; we use PD + feedforward

# derivative filter time constant
tau_d = 0.02

# quintic traj default duration (seconds)
DEFAULT_TRAJ_T = 0.45

# ------------------- PYGAME SETUP -------------------
pygame.init()
WIDTH, HEIGHT = 1200, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("4-Finger Tendon-Spring Hand - Model-based Control")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

# base positions for finger bases (so the hand sits nicely inside window)
base_positions = [
    (WIDTH//2 - 200, HEIGHT//2 + 100),
    (WIDTH//2 - 70, HEIGHT//2 + 100),
    (WIDTH//2 + 70, HEIGHT//2 + 100),
    (WIDTH//2 + 200, HEIGHT//2 + 100)
]

motor_positions = [(100, HEIGHT - 100)]*4

# UI buttons
break_button = pygame.Rect(WIDTH-180, 20, 160, 40)
reattach_button = pygame.Rect(WIDTH-180, 70, 160, 40)

# overlay positions
overlay_offset = (WIDTH-250, HEIGHT-250)  # bottom-right area for skeleton
assump_box_rect = pygame.Rect(WIDTH-250, HEIGHT-320, 230, 100)

# ------------------- HELPER FUNCTIONS -------------------
def midpoint(P0, P1):
    return ((P0[0]+P1[0])/2, (P0[1]+P1[1])/2)

def draw_spring(P0,P1,N=12,amp=4):
    dx, dy = P1[0]-P0[0], P1[1]-P0[1]
    Lline = max(1e-6, math.hypot(dx,dy))
    ux, uy = dx/Lline, dy/Lline
    px, py = -uy, ux
    pts=[]
    for k in range(N+1):
        t=k/N
        x = P0[0] + t*dx + amp*math.sin(2*math.pi*t*N)*px
        y = P0[1] + t*dy + amp*math.sin(2*math.pi*t*N)*py
        pts.append((x,y))
    pygame.draw.lines(screen, (0,255,0), False, pts, 2)

def draw_finger_geom(points):
    for i in range(3):
        pygame.draw.line(screen,(255,255,255),points[i],points[i+1],6)
        pygame.draw.circle(screen,(0,255,255),(int(points[i][0]),int(points[i][1])),5)
    pygame.draw.circle(screen,(255,0,0),(int(points[-1][0]),int(points[-1][1])),5)

def draw_motor_vis(phi):
    # single visible motor on left bottom (we draw same for each finger)
    pygame.draw.circle(screen,(200,200,0),(motor_positions[0][0],motor_positions[0][1]),30,4)
    x_end = motor_positions[0][0] + 25*math.cos(-phi)
    y_end = motor_positions[0][1] + 25*math.sin(-phi)
    pygame.draw.line(screen,(255,255,0),(motor_positions[0][0],motor_positions[0][1]),(x_end,y_end),4)

def draw_tendon_vis(points, phi, tendon_broken):
    if tendon_broken:
        # draw a slack short line near base to indicate break
        pygame.draw.line(screen, (120,0,120), (motor_positions[0][0]+20, motor_positions[0][1]-10), points[0], 1)
        return
    Nwrap = max(2, int(abs(phi)*20) )
    wrap_pts=[]
    for i in range(Nwrap+1):
        angle = -i*phi/Nwrap
        x = motor_positions[0][0]+R*50*math.cos(angle)
        y = motor_positions[0][1]+R*50*math.sin(angle)
        wrap_pts.append((x,y))
    pygame.draw.lines(screen,(255,0,255),False,wrap_pts,2)
    pygame.draw.line(screen,(255,0,255),wrap_pts[-1],points[0],2)
    for i in range(len(points)-1):
        pygame.draw.line(screen,(255,0,255),points[i],points[i+1],2)

# quintic trajectory
def quintic_traj(x0, xf, T, t):
    if T <= 0:
        return xf, 0.0, 0.0
    tau = np.clip(t / T, 0.0, 1.0)
    s = 10*tau**3 - 15*tau**4 + 6*tau**5
    ds_dt = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    d2s_dt2 = (60*tau - 180*tau**2 + 120*tau**3) / (T**2)
    x = x0 + (xf - x0) * s
    x_dot = (xf - x0) * ds_dt
    x_dd = (xf - x0) * d2s_dt2
    return x, x_dot, x_dd

def compute_finger_positions_from_phi(phi, idx):
    # phi (motor angle) -> joint angles via static split alpha -> joint thetas -> FK to points
    theta = alpha[idx] * phi   # quasi-static split
    theta_abs = np.cumsum(theta)
    x0, y0 = base_positions[idx]
    points = [(x0,y0)]
    for i in range(3):
        x1 = points[-1][0] - L[idx][i] * math.sin(theta_abs[i])
        y1 = points[-1][1] - L[idx][i] * math.cos(theta_abs[i])
        points.append((x1,y1))
    return points, theta, theta_abs

# draw text box of assumed parameters
def draw_assumptions():
    pygame.draw.rect(screen,(40,40,40),assump_box_rect)
    pygame.draw.rect(screen,(200,200,200),assump_box_rect,2)
    txts = [
        f"L (px): {list(map(int, L[0]))}",
        f"K (Nm/rad): {list(K[0])}",
        f"r (m): {list(r[0])}",
        f"Pulley R (m): {R}",
        f"Controller (ωn,zeta): ({omega_n:.1f},{zeta:.1f})"
    ]
    for i,t in enumerate(txts):
        screen.blit(font.render(t, True, (255,255,255)), (assump_box_rect.x+6, assump_box_rect.y+6 + i*18))

# ------------------- CONTROL STATE (per finger) -------------------
finger_state = []
for i in range(4):
    st = {
        "phi": 0.0,          # actual motor angle (simulated)
        "phi_dot": 0.0,
        "phi_dd": 0.0,
        "tau_cmd": 0.0,
        "deriv_filtered": 0.0,
        "traj_active": False,
        "traj_t": 0.0,
        "traj_T": DEFAULT_TRAJ_T,
        "traj_from": 0.0,
        "traj_to": 0.0,
        # controller gains and plant params for this finger
        "J": J_vals[i],
        "b": b_vals[i],
        "k": k_vals[i],
        "Kp": Kp_vals[i],
        "Kd": Kd_vals[i]
    }
    finger_state.append(st)

# ------------------- OPENCV SETUP -------------------
cap = cv2.VideoCapture(0)
hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)
finger_percents = [0.0]*4
skeleton_points = []

tendon_broken = False

# small helper to start trajectory to new phi target
def start_trajectory(st, phi_target, T=None):
    st["traj_active"] = True
    st["traj_t"] = 0.0
    st["traj_from"] = st["phi"]
    st["traj_to"] = float(phi_target)
    if T is not None:
        st["traj_T"] = T
    else:
        st["traj_T"] = DEFAULT_TRAJ_T

# ------------------- MAIN LOOP -------------------
last_time = time.time()
while True:
    now = time.time()
    dt = now - last_time
    last_time = now
    if dt <= 0:
        dt = 1/60.0

    screen.fill((20,20,20))

    # --- read webcam and compute bend percents and skeleton ---
    ret, frame = cap.read()
    skeleton_points = []
    if ret:
        frame = cv2.flip(frame, 1)
        h,w,_ = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)
        if result.multi_hand_landmarks:
            handLms = result.multi_hand_landmarks[0]
            # fingers used: index=5,6,8 ; middle=9,10,12 ; ring=13,14,16 ; pinky=17,18,20
            finger_ids = [(5,6,8),(9,10,12),(13,14,16),(17,18,20)]
            for idx, (a_id,b_id,c_id) in enumerate(finger_ids):
                a = get_point(handLms.landmark[a_id], w, h)
                b = get_point(handLms.landmark[b_id], w, h)
                c = get_point(handLms.landmark[c_id], w, h)
                ang = angle(a,b,c)
                finger_percents[idx] = bend_percent(ang)
            # skeleton points scaled for overlay (scale to fit a small box)
            overlay_scale = 0.6
            for lm_pt in handLms.landmark:
                x = int(lm_pt.x * WIDTH * overlay_scale)
                y = int(lm_pt.y * HEIGHT * overlay_scale)
                skeleton_points.append((x,y))

    # --- UI events ---
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            cap.release()
            pygame.quit()
            exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx,my = event.pos
            if break_button.collidepoint((mx,my)):
                tendon_broken = True
            if reattach_button.collidepoint((mx,my)):
                tendon_broken = False

    # --- update each finger: controller + plant integration ---
    for i in range(1):
        st = finger_state[i]
        # desired percent from camera --> desired motor angle phi_des
        desired_percent = finger_percents[i]
        phi_des = (desired_percent / 100.0) * phi_max[i]

        # if new goal differs significantly from current target, start a new smooth trajectory
        if not st["traj_active"]:
            # if target isn't already at phi_des (tolerance small) start trajectory
            if abs(phi_des - st["phi"]) > 1e-4:
                start_trajectory(st, phi_des, T=DEFAULT_TRAJ_T)
        else:
            # if an external change happens (camera moves) update the target smoothly:
            # we replace target to new phi_des but keep current traj_from as current phi to prevent jumps
            if abs(phi_des - st["traj_to"]) > 1e-3:
                # restart trajectory from current phi to new target
                start_trajectory(st, phi_des, T=DEFAULT_TRAJ_T)

        # increment trajectory time
        if st["traj_active"]:
            st["traj_t"] += dt
            t_rel = st["traj_t"]
            phi_ref, phi_ref_dot, phi_ref_dd = quintic_traj(st["traj_from"], st["traj_to"], st["traj_T"], t_rel)
            if st["traj_t"] >= st["traj_T"]:
                st["traj_active"] = False
        else:
            # hold steady at current phi (no active trajectory)
            phi_ref = st["phi"]
            phi_ref_dot = 0.0
            phi_ref_dd = 0.0

        # Computed-torque feedforward (single-DOF approximation)
        J = st["J"]; b = st["b"]; k = st["k"]
        tau_ff = J * phi_ref_dd + b * phi_ref_dot + k * phi_ref

        # Feedback PD (on motor angle phi)
        error = phi_ref - st["phi"]
        edot = phi_ref_dot - st["phi_dot"]

        # PD control
        Kp = st["Kp"]; Kd = st["Kd"]
        tau_fb = Kp * error + Kd * edot

        # total torque command and saturation (anti-windup would be required if integrator is present)
        tau_unsat = tau_ff + tau_fb
        tau_cmd = float(np.clip(tau_unsat, -TAU_MAX, TAU_MAX))
        st["tau_cmd"] = tau_cmd

        # integrate plant dynamics (simulate motor+bar)
        # full rotational dynamics: J phi_dd + b phi_dot + k phi = tau_cmd
        phi_dd = (tau_cmd - b * st["phi_dot"] - k * st["phi"]) / J
        st["phi_dot"] += phi_dd * dt
        st["phi"] += st["phi_dot"] * dt
        st["phi_dd"] = phi_dd

    # --- DRAW UI ---
    # buttons
    pygame.draw.rect(screen,(255,50,50),break_button)
    screen.blit(font.render("Break Tendon", True, (255,255,255)), (break_button.x+6, break_button.y+10))
    pygame.draw.rect(screen,(50,200,50),reattach_button)
    screen.blit(font.render("Reattach Tendon", True, (255,255,255)), (reattach_button.x+6, reattach_button.y+10))

    # draw fingers (visuals per finger)
    for i in range(1):
        st = finger_state[i]
        pts, theta_vec, theta_abs = compute_finger_positions_from_phi(st["phi"], i)
        # tendon drawn using phi reference (visual)
        draw_tendon_vis(pts, st["phi"], tendon_broken)
        draw_finger_geom(pts)
        # draw 3 dorsal springs (base->mid1, mid1->mid2, mid2->mid3)
        mid1 = midpoint(pts[0], pts[1])
        mid2 = midpoint(pts[1], pts[2])
        mid3 = midpoint(pts[2], pts[3])
        draw_spring(pts[0], mid1)
        draw_spring(mid1, mid2)
        draw_spring(mid2, mid3)

        # angle and tau display for each finger (left side blocks)
        xtxt = 10
        ytxt = 10 + i*80
        for j in range(3):
            deg = math.degrees(theta_vec[j])
            screen.blit(font.render(f"{FINGER_NAMES[i]} θ{j+1}: {deg:5.1f}°", True, (255,255,255)), (xtxt, ytxt + j*18))
        screen.blit(font.render(f"{FINGER_NAMES[i]} tau: {st['tau_cmd']:.2f} Nm", True, (200,200,255)), (xtxt+180, ytxt+10))
        screen.blit(font.render(f"Bend %: {finger_percents[i]:.1f}%", True, (200,255,200)), (xtxt+180, ytxt+28))

    # assumed values box
    draw_assumptions()

    # skeleton overlay bottom-right
    overlay_offset = (overlay_offset := overlay_offset)[0], overlay_offset[1]  # keep tuple
    if skeleton_points:
        # draw connections
        for connection in HAND_CONNECTIONS:
            start_idx, end_idx = connection
            if start_idx < len(skeleton_points) and end_idx < len(skeleton_points):
                spt = skeleton_points[start_idx]
                ept = skeleton_points[end_idx]
                start = (overlay_offset[0] + spt[0], overlay_offset[1] + spt[1])
                end = (overlay_offset[0] + ept[0], overlay_offset[1] + ept[1])
                pygame.draw.line(screen, (0,255,255), start, end, 2)
        # draw points
        for pt in skeleton_points:
            x = overlay_offset[0] + pt[0]
            y = overlay_offset[1] + pt[1]
            pygame.draw.circle(screen, (255,255,0), (int(x), int(y)), 3)

    # small motor visual based on first finger phi
    draw_motor_vis(finger_state[0]["phi"])

    pygame.display.flip()
    clock.tick(60)

# cleanup (not reached in loop)
cap.release()
pygame.quit()
