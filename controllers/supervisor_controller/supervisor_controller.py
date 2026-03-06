from controller import Supervisor
import random
import math

TIME_STEP = 32
robot = Supervisor()
root_children = robot.getRoot().getField("children")
# FIGYELEM: Most már ROBOT néven keressük a Pioneer-t!
robot_node = robot.getFromDef("ROBOT") 
emitter = robot.getDevice("emitter")

def generate_map():
    grid = [[0 for _ in range(10)] for _ in range(10)]
    safe = set()
    cx, cy = 0, 0
    safe.add((cx, cy))
    while cx < 9 or cy < 9:
        if cx < 9 and (cy == 9 or random.random() < 0.5): cx += 1
        else: cy += 1
        safe.add((cx, cy))
    cells = [(x, y) for x in range(10) for y in range(10) if (x, y) not in safe]
    random.shuffle(cells)
    for i in range(min(15, len(cells))):
        x, y = cells[i]; grid[x][y] = 1
    return grid

def spawn_everything(grid):
    for i in range(root_children.getCount()-1, -1, -1):
        def_name = root_children.getMFNode(i).getDef()
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

game_grid = generate_map()
spawn_everything(game_grid)

while robot.step(TIME_STEP) != -1:
    if robot_node:
        pos = robot_node.getPosition()
        rot = robot_node.getOrientation()
        px, py = pos[0], pos[1]
        
        # A Pioneer 3-DX abszolút Z-tengely körüli elfordulása (Yaw)
        yaw_angle = math.atan2(rot[3], rot[0])
        
        # 1. Alapértelmezett távolságok a pálya 4 faláig
        dist_E = 9.5 - px  # Kelet (0 rad)
        dist_N = 9.5 - py  # Észak (pi/2 rad)
        dist_W = px - (-0.5) # Nyugat (pi rad)
        dist_S = py - (-0.5) # Dél (-pi/2 rad)
        
        # 2. Akna védelem
        for x in range(10):
            for y in range(10):
                if game_grid[x][y] == 1:
                    if abs(y - py) < 0.5:
                        if x > px: dist_E = min(dist_E, x - px - 0.5)
                        elif x < px: dist_W = min(dist_W, px - x - 0.5)
                    if abs(x - px) < 0.5:
                        if y > py: dist_N = min(dist_N, y - py - 0.5)
                        elif y < py: dist_S = min(dist_S, py - y - 0.5)

        if emitter:
            # 4 irány távolsága | GPS X | GPS Y | Iránytű
            msg = f"{dist_E:.2f}|{dist_N:.2f}|{dist_W:.2f}|{dist_S:.2f}|{px:.2f}|{py:.2f}|{yaw_angle:.2f}"
            emitter.send(msg.encode('utf-8'))