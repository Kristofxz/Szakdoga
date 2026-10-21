from controller import Supervisor
import random
import math

TIME_STEP = 32
robot = Supervisor()
root_children = robot.getRoot().getField("children")

robot_node = robot.getFromDef("ROBOT") 
emitter = robot.getDevice("emitter")

receiver = robot.getDevice("rx_sup")
if receiver:
    receiver.enable(TIME_STEP)

game_walls = set()
current_level = 1 # Alapértelmezett nehézségi szint

def get_difficulty_params(level):
    if level == 1: return 5, 5
    elif level == 2: return 10, 10
    elif level == 3: return 15, 15
    elif level == 4: return 20, 20
    else: return 20, 25 # 5. szint (Max)

def generate_map(level):
    global game_walls
    game_walls.clear()
    
    num_mines, num_walls = get_difficulty_params(level)
    
    grid = [[0 for _ in range(10)] for _ in range(10)]
    safe = set()
    safe_edges = set() 
    cx, cy = 0, 0
    safe.add((cx, cy))
    
    while cx < 9 or cy < 9:
        nx, ny = cx, cy
        if cx < 9 and (cy == 9 or random.random() < 0.5): nx += 1
        else: ny += 1
        safe_edges.add(((cx, cy), (nx, ny)))
        safe_edges.add(((nx, ny), (cx, cy)))
        cx, cy = nx, ny
        safe.add((cx, cy))
        
    cells = [(x, y) for x in range(10) for y in range(10) if (x, y) not in safe]
    random.shuffle(cells)
    
    for i in range(min(num_mines, len(cells))):
        x, y = cells[i]; grid[x][y] = 1 

    all_edges = []
    for x in range(10):
        for y in range(10):
            if x < 9: all_edges.append(((x,y), (x+1,y))) 
            if y < 9: all_edges.append(((x,y), (x,y+1))) 
            
    possible_walls = [e for e in all_edges if e not in safe_edges and (e[1], e[0]) not in safe_edges]
    random.shuffle(possible_walls)
    
    for i in range(min(num_walls, len(possible_walls))):
        e = possible_walls[i]
        game_walls.add(e)
        game_walls.add((e[1], e[0])) 

    return grid

def spawn_everything(grid):
    for i in range(root_children.getCount()-1, -1, -1):
        node = root_children.getMFNode(i)
        if node:
            def_name = node.getDef()
            if def_name in ["ARENA", "TILE_GROUP"]: root_children.removeMF(i)
    
    robot.step(TIME_STEP)
    arena_s = f"""
    DEF ARENA Group {{
      children [
        Solid {{ translation 4.5 4.5 -0.05 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 1.0 }} geometry Box {{ size 10.5 10.5 0.1 }} }} ] boundingObject Box {{ size 10.5 10.5 0.1 }} }}
        Solid {{ translation 4.5 -0.75 0.1 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.4 0.4 0.4 }} geometry Box {{ size 11 0.1 0.5 }} }} ] boundingObject Box {{ size 11 0.1 0.5 }} }}
        Solid {{ translation 4.5 9.75 0.1 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.4 0.4 0.4 }} geometry Box {{ size 11 0.1 0.5 }} }} ] boundingObject Box {{ size 11 0.1 0.5 }} }}
        Solid {{ translation -0.75 4.5 0.1 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.4 0.4 0.4 }} geometry Box {{ size 0.1 11 0.5 }} }} ] boundingObject Box {{ size 0.1 11 0.5 }} }}
        Solid {{ translation 9.75 4.5 0.1 children [ Shape {{ appearance PBRAppearance {{ baseColor 0.4 0.4 0.4 }} geometry Box {{ size 0.1 11 0.5 }} }} ] boundingObject Box {{ size 0.1 11 0.5 }} }}
      ]
    }}
    """
    root_children.importMFNodeFromString(-1, arena_s)
    robot.step(TIME_STEP)
    root_children.importMFNodeFromString(-1, "DEF TILE_GROUP Group { children [] }")
    robot.step(TIME_STEP)
    tile_group = robot.getFromDef("TILE_GROUP").getField("children")
    
    for x in range(10):
        for y in range(10):
            c = "0.7 0.7 0.7" if grid[x][y] == 0 else "0.8 0.1 0.1"
            t = f"Pose {{ translation {x} {y} 0.002 children [ Shape {{ appearance PBRAppearance {{ baseColor {c} roughness 1.0 }} geometry Plane {{ size 0.95 0.95 }} }} ] }}"
            tile_group.importMFNodeFromString(-1, t)

    spawned_walls = set()
    for e in game_walls:
        edge = tuple(sorted(e)) 
        if edge in spawned_walls: continue
        spawned_walls.add(edge)
        p1, p2 = edge
        x1, y1 = p1; x2, y2 = p2
        c = "0.3 0.3 0.5" 
        if x1 != x2: 
            wx, wy = (x1 + x2) / 2.0, y1
            t = f"Solid {{ translation {wx} {wy} 0.25 children [ Shape {{ appearance PBRAppearance {{ baseColor {c} roughness 0.8 }} geometry Box {{ size 0.1 1.0 0.5 }} }} ] boundingObject Box {{ size 0.1 1.0 0.5 }} }}"
        else: 
            wx, wy = x1, (y1 + y2) / 2.0
            t = f"Solid {{ translation {wx} {wy} 0.25 children [ Shape {{ appearance PBRAppearance {{ baseColor {c} roughness 0.8 }} geometry Box {{ size 1.0 0.1 0.5 }} }} ] boundingObject Box {{ size 1.0 0.1 0.5 }} }}"
        tile_group.importMFNodeFromString(-1, t)

game_grid = generate_map(current_level)
spawn_everything(game_grid)

def get_metal_signal(cx, cy, dx, dy):
    nx, ny = cx + dx, cy + dy
    if ((cx, cy), (nx, ny)) in game_walls: return 999.0
    signal = 0.0
    for i in range(1, 4):
        prev_x, prev_y = cx + dx*(i-1), cy + dy*(i-1)
        curr_x, curr_y = cx + dx*i, cy + dy*i
        if ((prev_x, prev_y), (curr_x, curr_y)) in game_walls: break 
        if 0 <= curr_x <= 9 and 0 <= curr_y <= 9:
            if game_grid[curr_x][curr_y] == 1:
                if i == 1: signal += 10.0
                elif i == 2: signal += 3.0
                elif i == 3: signal += 1.0
        else:
            if i == 1: signal += 100.0 
    return signal

exploded = False
timeout_timer = 0 

while robot.step(TIME_STEP) != -1:
    timeout_timer += 1
    
    manual_reset = False
    if receiver and receiver.getQueueLength() > 0:
        msg = receiver.getString()
        receiver.nextPacket()
        if msg.startswith("RESET"):
            manual_reset = True
            parts = msg.split('|')
            if len(parts) > 1:
                current_level = int(parts[1]) # Szint beolvasása a Gym-től

    if robot_node and not exploded:
        pos = robot_node.getPosition()
        rot = robot_node.getOrientation()
        px, py = pos[0], pos[1]
        
        yaw_angle = math.atan2(rot[3], rot[0])
        cx, cy = int(round(px)), int(round(py))
        
        is_dead = (0 <= cx <= 9 and 0 <= cy <= 9 and game_grid[cx][cy] == 1)
        is_win = (cx >= 9 and cy >= 9)
        is_stalled = manual_reset 

        if is_dead or is_win or is_stalled:
            exploded = True
            if emitter:
                if is_dead: msg = f"BOOM|{px:.2f}|{py:.2f}"
                elif is_win: msg = f"WIN|{px:.2f}|{py:.2f}"
                else: msg = f"STALL|{px:.2f}|{py:.2f}"
                emitter.send(msg.encode('utf-8'))
            
            for _ in range(3): robot.step(TIME_STEP)
            game_grid = generate_map(current_level) # Új pálya generálása a szintnek megfelelően
            spawn_everything(game_grid)
            
            trans_field = robot_node.getField("translation")
            rot_field = robot_node.getField("rotation")
            if trans_field and rot_field:
                trans_field.setSFVec3f([0.0, 0.0, 0.0969]) 
                rot_field.setSFRotation([0.0, 0.0, 1.0, 0.0])
                robot_node.resetPhysics()
            
            timeout_timer = 0 
            exploded = False
            continue

        sig_E = get_metal_signal(cx, cy, 1, 0)
        sig_N = get_metal_signal(cx, cy, 0, 1)
        sig_W = get_metal_signal(cx, cy, -1, 0)
        sig_S = get_metal_signal(cx, cy, 0, -1)

        if emitter:
            msg = f"{sig_E:.2f}|{sig_N:.2f}|{sig_W:.2f}|{sig_S:.2f}|{px:.2f}|{py:.2f}|{yaw_angle:.2f}"
            emitter.send(msg.encode('utf-8'))