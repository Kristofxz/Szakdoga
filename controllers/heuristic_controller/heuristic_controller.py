from controller import Robot
import math
import heapq

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
rx = robot.getDevice('rx0')
if rx: rx.enable(timestep)

left_ps = left_motor.getPositionSensor()
right_ps = right_motor.getPositionSensor()
if left_ps: left_ps.enable(timestep)
if right_ps: right_ps.enable(timestep)

for _ in range(2): robot.step(timestep)

current_left_target = left_ps.getValue() if left_ps else 0.0
current_right_target = right_ps.getValue() if right_ps else 0.0

left_motor.setPosition(current_left_target)
right_motor.setPosition(current_right_target)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

WHEEL_RADIUS = 0.0975
AXLE_LENGTH = 0.33 

STATE_INIT_SCAN = "INIT_SCAN"
STATE_SCAN_TURN = "SCAN_TURN"
STATE_SCAN_READ = "SCAN_READ"
STATE_DECIDE = "DECIDE"
STATE_MOVE_TURN = "MOVE_TURN"
STATE_WAIT_MOVE_TURN = "WAIT_MOVE_TURN"
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
known_cells = [[-1.0 for _ in range(10)] for _ in range(10)]
visited_tiles = set() 
robot_known_walls = set()

def reset_robot_memory():
    global heatmap_grid, known_cells, visited_tiles, robot_known_walls
    global current_state, has_data, scan_results, current_scan_index, timer
    
    heatmap_grid = [[-1.0 for _ in range(10)] for _ in range(10)]
    known_cells = [[-1.0 for _ in range(10)] for _ in range(10)]
    visited_tiles.clear()
    robot_known_walls.clear()
    scan_results.clear()
    
    current_scan_index = 0
    timer = 0
    has_data = False
    current_state = STATE_INIT_SCAN

def print_heatmap(rx_int, ry_int):
    print("\n" + "="*34)
    print("     AKNA HŐTÉRKÉP")
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
        import copy
        
        plot_data = np.array(heatmap_grid).T 
        display_data = np.copy(plot_data)
        display_data = np.where(display_data >= 900.0, np.nan, display_data)
        display_data = np.where(display_data == -1.0, np.nan, display_data)
        
        fig, ax = plt.subplots(figsize=(8, 8))
        
        cmap = copy.copy(plt.get_cmap('plasma'))
        cmap.set_bad(color='#2b2b2b') 
        
        cax = ax.imshow(display_data, origin='lower', cmap=cmap, vmin=0, vmax=15, interpolation='nearest')
        
        ax.set_xticks(np.arange(-0.5, 10, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, 10, 1), minor=True)
        ax.grid(which="minor", color="#555555", linestyle='-', linewidth=1.5)
        ax.tick_params(which="minor", bottom=False, left=False)
        
        for y in range(10):
            for x in range(10):
                val = plot_data[y, x]
                if x == rx_int and y == ry_int:
                    text_val = "B" if "felrobbant" in title or "BUMM" in title else "R"
                elif val == -1:
                    text_val = "" 
                elif val >= 900:
                    text_val = "" 
                else:
                    text_val = f"{val:.0f}"
                
                color = "black" if (0 < val > 8 and val < 900) else "white"
                    
                if text_val:
                    ax.text(x, y, text_val, ha="center", va="center", color=color, fontsize=12, fontweight='bold')
        
        drawn_walls = set()
        for (p1, p2) in robot_known_walls:
            wall_edge = tuple(sorted((p1, p2)))
            if wall_edge in drawn_walls: continue
            drawn_walls.add(wall_edge)
            
            wx = (p1[0] + p2[0]) / 2.0
            wy = (p1[1] + p2[1]) / 2.0
            
            if p1[0] != p2[0]: 
                ax.plot([wx, wx], [wy - 0.5, wy + 0.5], color='#ff4444', linewidth=6, solid_capstyle='round')
            else: 
                ax.plot([wx - 0.5, wx + 0.5], [wy, wy], color='#ff4444', linewidth=6, solid_capstyle='round')

        cbar = fig.colorbar(cax, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Aknajelző emisszió erőssége', fontsize=12)
        
        ax.set_title(title, fontsize=15, fontweight='bold', pad=15)
        ax.set_xlabel('X koordináta', fontsize=12)
        ax.set_ylabel('Y koordináta', fontsize=12)
        ax.set_xticks(range(10))
        ax.set_yticks(range(10))
        
        filename = "szakdoga_heatmap_lose.png" if "felrobbant" in title or "BUMM" in title else "szakdoga_heatmap_win.png"
        plt.savefig(filename, dpi=200, bbox_inches='tight') 
    except ImportError:
        pass

def move_wheels(left_rad, right_rad, speed=2.0):
    global current_left_target, current_right_target
    current_left_target += left_rad
    current_right_target += right_rad
    
    left_motor.setPosition(current_left_target)
    right_motor.setPosition(current_right_target)
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)

def find_best_path(start_x, start_y, goal_x=9, goal_y=9):
    open_set = []
    heapq.heappush(open_set, (0, (start_x, start_y)))
    came_from = {}
    g_score = {(start_x, start_y): 0}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        cx, cy = current
        
        if cx == goal_x and cy == goal_y:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in [(1,0), (0,1), (-1,0), (0,-1)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx <= 9 and 0 <= ny <= 9:
                if ((cx, cy), (nx, ny)) in robot_known_walls or ((nx, ny), (cx, cy)) in robot_known_walls:
                    continue
                
                cell_val = known_cells[nx][ny]
                if cell_val >= 10.0:  
                    continue
                
                step_cost = 1.0 
                if cell_val == -1:
                    step_cost = 1.0  
                else:
                    step_cost += cell_val * 0.3  
                
                if (nx, ny) in visited_tiles:
                    step_cost += 1.5 
                
                tentative_g = g_score.get(current, 0) + step_cost
                
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    f_score = tentative_g + (abs(goal_x - nx) + abs(goal_y - ny))
                    heapq.heappush(open_set, (f_score, (nx, ny)))
                    
    return None

while robot.step(timestep) != -1:
    reset_triggered = False
    
    while rx and rx.getQueueLength() > 0:
        data_str = rx.getString()
        data = data_str.split('|')
        rx.nextPacket()
        
        if data[0] in ["BOOM", "WIN", "STALL"]:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            px, py = int(round(float(data[1]))), int(round(float(data[2])))
            
            if data[0] == "BOOM":
                print("\n[BUMM] Ráléptél egy aknára!")
                title = "A robot felrobbant!"
            elif data[0] == "WIN":
                print("\n[SIKER] CÉL ELÉRVE!")
                title = "Túlélte a robot."
            else:
                print("\n[HIBA] Beszorult a robot (STALL)!")
                title = "A robot beszorult."
                
            print_heatmap(px, py)
            show_graphical_heatmap(px, py, title=title)
            
            reset_robot_memory()
            reset_triggered = True
            
        elif len(data) >= 7:
            sig_E, sig_N, sig_W, sig_S = float(data[0]), float(data[1]), float(data[2]), float(data[3])
            curr_x, curr_y, current_yaw = float(data[4]), float(data[5]), float(data[6])
            has_data = True 
            
    if reset_triggered: continue
    if not has_data: continue 

    rx_int, ry_int = round(curr_x), round(curr_y)
    rx_int = max(0, min(9, rx_int))
    ry_int = max(0, min(9, ry_int))

    if current_state == STATE_INIT_SCAN:
        visited_tiles.add((rx_int, ry_int))
        heatmap_grid[rx_int][ry_int] = 0.0
        known_cells[rx_int][ry_int] = 0.0
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
                        else:
                            if signal >= 10.0:
                                known_cells[nx][ny] = 999.0 
                            else:
                                known_cells[nx][ny] = signal 
                                
                            if (nx, ny) not in visited_tiles:
                                current_val = heatmap_grid[nx][ny]
                                heatmap_grid[nx][ny] = signal if current_val == -1 else max(current_val, signal)
                                
                print_heatmap(rx_int, ry_int)
                current_state = STATE_DECIDE 

    elif current_state == STATE_DECIDE:
        path = find_best_path(rx_int, ry_int)
        
        if path:
            target_gx, target_gy = path[0] 
            target_yaw = math.atan2(target_gy - ry_int, target_gx - rx_int)
            current_state = STATE_MOVE_TURN
        else:
            print("Az A* nem talál érvényes utat a célhoz!")
            
            best_fallback_score = 99999
            chosen_yaw = None
            
            for angle, signal in scan_results.items():
                nx = int(round(rx_int + math.cos(angle)))
                ny = int(round(ry_int + math.sin(angle)))
                
                if 0 <= nx <= 9 and 0 <= ny <= 9:
                    if signal > 900.0: 
                        continue
                        
                    visited_penalty = 50.0 if (nx, ny) in visited_tiles else 0.0
                    danger_score = signal * 10.0
                    score = visited_penalty + danger_score
                    
                    if score < best_fallback_score:
                        best_fallback_score = score
                        chosen_yaw = angle
                        target_gx = nx
                        target_gy = ny
                        
            if chosen_yaw is not None:
                print(f"-> Fallback: Próbálok a ({target_gx}, {target_gy}) felé menekülni.")
                target_yaw = chosen_yaw 
                current_state = STATE_MOVE_TURN
            else:
                print("-> Totális csapda, nincs hova lépni!")
                current_state = STATE_INIT_SCAN

    elif current_state == STATE_MOVE_TURN:
        error = target_yaw - current_yaw
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        wheel_turn = (error * (AXLE_LENGTH / 2.0)) / WHEEL_RADIUS
        move_wheels(-wheel_turn, wheel_turn, speed=1.5)
        current_state = STATE_WAIT_MOVE_TURN
        timer = 0

    elif current_state == STATE_WAIT_MOVE_TURN:
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