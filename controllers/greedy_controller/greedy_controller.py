from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
rx = robot.getDevice('rx0')
if rx: rx.enable(timestep)

# --- A HIBA OKA ÉS A MEGOLDÁS: A "SNAPBACK" MEGELŐZÉSE ---
left_ps = left_motor.getPositionSensor()
right_ps = right_motor.getPositionSensor()
if left_ps: left_ps.enable(timestep)
if right_ps: right_ps.enable(timestep)

# Várunk picit, hogy a szenzor felébredjen
for _ in range(2): robot.step(timestep)

# Lekérjük az aktuális "feltekert" pozíciót
current_left_target = left_ps.getValue() if left_ps else 0.0
current_right_target = right_ps.getValue() if right_ps else 0.0

# Ezt állítjuk be célnak, így nem próbál meg visszaugrani 0-ra!
left_motor.setPosition(current_left_target)
right_motor.setPosition(current_right_target)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
# ---------------------------------------------------------

WHEEL_RADIUS = 0.0975
AXLE_LENGTH = 0.33 

STATE_INIT_SCAN = "INIT_SCAN"
STATE_SCAN_TURN = "SCAN_TURN"
STATE_SCAN_READ = "SCAN_READ"
STATE_DECIDE = "DECIDE"
STATE_MOVE_TURN = "MOVE_TURN"
STATE_MOVE = "MOVE"

current_state = STATE_INIT_SCAN
timer = 0
scan_directions = [0.0, math.pi/2, math.pi, -math.pi/2]
dir_names = ["Kelet", "Észak", "Nyugat", "Dél"]
current_scan_index = 0
scan_results = {}
target_yaw, target_gx, target_gy = 0.0, 0, 0

sig_E, sig_N, sig_W, sig_S = 0.0, 0.0, 0.0, 0.0
curr_x, curr_y, current_yaw = 0.0, 0.0, 0.0
has_data = False 

heatmap_grid = [[-1.0 for _ in range(10)] for _ in range(10)]
visited_tiles = set() 
robot_known_walls = set() 

def print_heatmap(rx_int, ry_int):
    print("\n" + "="*34)
    print("      AKNA HŐTÉRKÉP")
    print("="*34)
    for y in range(9, -1, -1):
        row_str = ""
        for x in range(10):
            val = heatmap_grid[x][y]
            if x == rx_int and y == ry_int:
                row_str += "[ R] " 
            elif val == -1:
                row_str += " ..  " 
            else:
                row_str += f"{val:02.0f}  " 
        print(row_str)
    print("="*34 + "\n(Falak a terminalban nem latszanak, de a robot kerulgeti oket!)")

def show_graphical_heatmap(rx_int, ry_int, title="Akna heatmap"):
    try:
        import matplotlib.pyplot as plt
        import numpy as np
        
        plot_data = np.array(heatmap_grid).T 
        plt.figure(figsize=(8, 8))
        plt.imshow(plot_data, origin='lower', cmap='hot', interpolation='nearest', vmin=-1)
        
        for y in range(10):
            for x in range(10):
                val = plot_data[y, x]
                if x == rx_int and y == ry_int:
                    text_val = "☠️" if "BUMM" in title else "🤖"
                else:
                    text_val = "X" if val == -1 else f"{val:.0f}"
                color = "white" if val > 3 else "black"
                plt.text(x, y, text_val, ha="center", va="center", color=color, fontsize=12, fontweight='bold')
        
        for (p1, p2) in robot_known_walls:
            wx = (p1[0] + p2[0]) / 2.0
            wy = (p1[1] + p2[1]) / 2.0
            if p1[0] != p2[0]: 
                plt.plot([wx, wx], [wy - 0.5, wy + 0.5], color='#00aaff', linewidth=5)
            else: 
                plt.plot([wx - 0.5, wx + 0.5], [wy, wy], color='#00aaff', linewidth=5)

        plt.colorbar(label='Jelerősség')
        plt.title(title)
        plt.xlabel('X ')
        plt.ylabel('Y')
        plt.xticks(range(10))
        plt.yticks(range(10))
        
        filename = "szakdoga_heatmap_lose.png" if "BUMM" in title else "szakdoga_heatmap_win.png"
        plt.savefig(filename)
        print(f"\nKép sikeresen elmentve '{filename}' néven a projekt mappájába!")
    except ImportError:
        pass

# --- Relatív mozgatás a belső változók növelésével ---
def move_wheels(left_rad, right_rad, speed=2.0):
    global current_left_target, current_right_target
    current_left_target += left_rad
    current_right_target += right_rad
    
    left_motor.setPosition(current_left_target)
    right_motor.setPosition(current_right_target)
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)

while robot.step(timestep) != -1:
    while rx and rx.getQueueLength() > 0:
        data_str = rx.getString()
        data = data_str.split('|')
        
        if data[0] == "BOOM":
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            death_x, death_y = int(round(float(data[1]))), int(round(float(data[2])))
            print("\nRáléptél egy aknára!")
            print_heatmap(death_x, death_y)
            show_graphical_heatmap(death_x, death_y, title="A robot felrobbant!")
            break

        if len(data) >= 7:
            sig_E = float(data[0])
            sig_N = float(data[1])
            sig_W = float(data[2])
            sig_S = float(data[3])
            curr_x = float(data[4])
            curr_y = float(data[5])
            current_yaw = float(data[6])
            has_data = True 
        rx.nextPacket()

    if 'data_str' in locals() and data_str.startswith("BOOM"):
        break

    if not has_data: continue 

    rx_int, ry_int = round(curr_x), round(curr_y)
    rx_int = max(0, min(9, rx_int))
    ry_int = max(0, min(9, ry_int))

    if rx_int >= 9 and ry_int >= 9:
        left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)
        print("CÉL ELÉRVE!")
        print_heatmap(rx_int, ry_int)
        show_graphical_heatmap(rx_int, ry_int, title="Túlélte a robot.")
        break

    if current_state == STATE_INIT_SCAN:
        visited_tiles.add((rx_int, ry_int))
        heatmap_grid[rx_int][ry_int] = 0.0
        
        scan_results.clear()
        current_scan_index = 0
        current_state = STATE_SCAN_TURN

    elif current_state == STATE_SCAN_TURN:
        target_angle = scan_directions[current_scan_index]
        error = target_angle - current_yaw
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        
        wheel_turn = (error * (AXLE_LENGTH / 2.0)) / WHEEL_RADIUS
        move_wheels(-wheel_turn, wheel_turn, speed=1.5)
        current_state = STATE_SCAN_READ
        timer = 0

    elif current_state == STATE_SCAN_READ:
        timer += 1
        if timer > 70: 
            if current_scan_index == 0: current_signal = sig_E
            elif current_scan_index == 1: current_signal = sig_N
            elif current_scan_index == 2: current_signal = sig_W
            else: current_signal = sig_S
            
            scan_results[scan_directions[current_scan_index]] = current_signal
            
            current_scan_index += 1
            if current_scan_index < 4:
                current_state = STATE_SCAN_TURN
            else:
                for angle, signal in scan_results.items():
                    nx = int(round(rx_int + math.cos(angle)))
                    ny = int(round(ry_int + math.sin(angle)))
                    if 0 <= nx <= 9 and 0 <= ny <= 9:
                        if signal > 900.0:
                            robot_known_walls.add(((rx_int, ry_int), (nx, ny)))
                            robot_known_walls.add(((nx, ny), (rx_int, ry_int)))
                        elif (nx, ny) not in visited_tiles:
                            current_val = heatmap_grid[nx][ny]
                            heatmap_grid[nx][ny] = signal if current_val == -1 else max(current_val, signal)
                            
                print_heatmap(rx_int, ry_int)
                current_state = STATE_DECIDE 

    elif current_state == STATE_DECIDE:
        best_score = 99999
        chosen_yaw = None
        
        for angle, signal in scan_results.items():
            nx = int(round(rx_int + math.cos(angle)))
            ny = int(round(ry_int + math.sin(angle)))
            
            if 0 <= nx <= 9 and 0 <= ny <= 9:
                if signal < 10.0: 
                    d_goal = math.sqrt((9 - nx)**2 + (9 - ny)**2)
                    visited_penalty = 50.0 if (nx, ny) in visited_tiles else 0.0
                    score = (signal * 5.0) + d_goal + visited_penalty
                    
                    if score < best_score:
                        best_score, chosen_yaw, target_gx, target_gy = score, angle, nx, ny
        
        if chosen_yaw is not None:
            target_yaw = chosen_yaw
            current_state = STATE_MOVE_TURN
        else:
            print("Teljesen beszorultam, nincs hova lépni!")
            current_state = STATE_INIT_SCAN

    elif current_state == STATE_MOVE_TURN:
        error = target_yaw - current_yaw
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        wheel_turn = (error * (AXLE_LENGTH / 2.0)) / WHEEL_RADIUS
        move_wheels(-wheel_turn, wheel_turn, speed=1.5)
        current_state = "STATE_WAIT_MOVE_TURN"
        timer = 0

    elif current_state == "STATE_WAIT_MOVE_TURN":
        timer += 1
        if timer > 115: 
            current_state = STATE_MOVE
            timer = 0

    elif current_state == STATE_MOVE:
        timer += 1
        if timer == 1:
            dist_to_go = math.sqrt((rx_int - target_gx)**2 + (ry_int - target_gy)**2)
            dist_to_go = min(1.1, dist_to_go)
            wheel_forward = dist_to_go / WHEEL_RADIUS
            move_wheels(wheel_forward, wheel_forward, speed=3.0)
        
        if timer > 120:
            current_state = STATE_INIT_SCAN