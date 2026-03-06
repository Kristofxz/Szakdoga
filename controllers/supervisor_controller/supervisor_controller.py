from controller import Supervisor
import random

TIME_STEP = 32
WIDTH = 10
HEIGHT = 10
MINES_COUNT = 15
TILE_SIZE = 1

robot = Supervisor()

root_node = robot.getRoot()
root_children_field = root_node.getField("children")

robot_node = robot.getFromDef("BB8")

#Pálya generálása
def generate_map(height, width, mines):
    grid = [[0 for col in range(height)] for row in range(width)]

    safe_path = set()
    curr_x, curr_y = 0, 0
    safe_path.add((curr_x, curr_y))

    while curr_x < width - 1 or curr_y < height - 1:
        can_move_right = curr_x < width - 1
        can_move_up = curr_y < height - 1

        if can_move_right and (not can_move_up or random.random() < 0.5):
            curr_x += 1
        else:
            curr_y += 1

        safe_path.add((curr_x, curr_y))

    all_cells = [(x, y) for x in range(width) for y in range(height)]
    random.shuffle(all_cells)

    placed = 0
    for x, y in all_cells:
        if placed >= mines:
            break
        if (x, y) not in safe_path:
            grid[x][y] = 1
            placed += 1

    return grid, safe_path
    
def spawn_tiles(grid):
    group_string = "DEF TILE_GROUP Group { children [] }"
    root_children_field.importMFNodeFromString(-1, group_string)
    
    tile_group_node = robot.getFromDef("TILE_GROUP")
    group_children_field = tile_group_node.getField("children")
    
    for x in range(WIDTH):
        for y in range(HEIGHT):
            tile_string = f"""
            Pose {{
              translation {x * TILE_SIZE} {y * TILE_SIZE} 0.005
              children [
                DEF TILE_{x}_{y} Solid {{
                  translation 0 0 0
                  children [
                    Shape {{
                      appearance PBRAppearance {{
                        baseColor 0.8 0.8 0.8
                        roughness 1
                      }}
                      geometry DEF TILE_GEOM Box {{
                        size {TILE_SIZE * 0.95} {TILE_SIZE * 0.95} 0.01
                      }}
                    }}
                  ]
                  boundingObject USE TILE_GEOM
                }}
              ]
            }}
            """
            group_children_field.importMFNodeFromString(-1, tile_string)

def spawn_floor():
    arena_width = WIDTH * TILE_SIZE
    arena_height = HEIGHT * TILE_SIZE
    wall_thickness = 0.05
    wall_height = 0.2 

    center_x = (arena_width / 2) - (TILE_SIZE / 2)
    center_y = (arena_height / 2) - (TILE_SIZE / 2)

    arena_string = f"""
    DEF ARENA Pose {{
      translation {center_x} {center_y} 0
      children [

        Solid {{
          children [
            Shape {{
              appearance PBRAppearance {{ baseColor 0.1 0.1 0.1 roughness 1 }}
              geometry DEF GROUND_GEOM Box {{ size {arena_width} {arena_height} 0.001 }}
            }}
          ]
          boundingObject USE GROUND_GEOM
        }}
        
        Solid {{
          translation 0 {-arena_height/2 - wall_thickness/2} {wall_height/2}
          children [ 
            DEF WALL_SHAPE Shape {{
              appearance PBRAppearance {{ baseColor 0.5 0.5 0.5 }}
              geometry DEF WALL_GEOM Box {{ size {arena_width + 2*wall_thickness} {wall_thickness} {wall_height} }}
            }} 
          ]
          boundingObject USE WALL_GEOM
        }}
        Solid {{
          translation 0 {arena_height/2 + wall_thickness/2} {wall_height/2}
          children [ USE WALL_SHAPE ]
          boundingObject USE WALL_GEOM
        }}
        Solid {{
          translation {-arena_width/2 - wall_thickness/2} 0 {wall_height/2}
          children [ 
            DEF SIDE_WALL_SHAPE Shape {{
              appearance PBRAppearance {{ baseColor 0.5 0.5 0.5 }}
              geometry DEF SIDE_WALL_GEOM Box {{ size {wall_thickness} {arena_height} {wall_height} }}
            }} 
          ]
          boundingObject USE SIDE_WALL_GEOM
        }}
        Solid {{
          translation {arena_width/2 + wall_thickness/2} 0 {wall_height/2}
          children [ USE SIDE_WALL_SHAPE ]
          boundingObject USE SIDE_WALL_GEOM
        }}
      ]
    }}
    """
    root_children_field.importMFNodeFromString(-1, arena_string)


def count_neighbors(grid, x, y):
    count = 0
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0: continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                if grid[nx][ny] == 1:
                    count += 1
    return count


def flood_fill_reveal(x, y, grid, revealed):
    if not (0 <= x < WIDTH and 0 <= y < HEIGHT) or revealed[x][y]:
        return

    revealed[x][y] = True
    
    if count_neighbors(grid, x, y) > 0:
        return

    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0: continue
            flood_fill_reveal(x + dx, y + dy, grid, revealed)


def print_board(grid, revealed, safe_path):
    print("\n--- AKNAKERESŐ TESZT ---")
    for y in reversed(range(HEIGHT)):
        row_str = ""
        for x in range(WIDTH):
            if not revealed[x][y]:
                row_str += "[]"  
            elif grid[x][y] == 1:
                row_str += "X "  
            else:
                c = count_neighbors(grid, x, y)
                row_str += ". " if c == 0 else f"{c} "
        print(row_str)



#csak teszteleshez
my_map, path = generate_map(HEIGHT,WIDTH,MINES_COUNT)
game_grid, path = generate_map(WIDTH, HEIGHT, MINES_COUNT)
revealed_map = [[False for _ in range(HEIGHT)] for _ in range(WIDTH)]
goal_pos = (WIDTH - 1, HEIGHT - 1)
flood_fill_reveal(0, 0, game_grid, revealed_map)
spawn_floor()
spawn_tiles(game_grid)
last_pos = (-1, -1)

playing = True


while robot.step(TIME_STEP) != -1:
        
    if robot_node:
        pos = robot_node.getPosition()

        curr_x = int(round(pos[0] / TILE_SIZE))
        curr_y = int(round(pos[1] / TILE_SIZE))

        if 0 <= curr_x < WIDTH and 0 <= curr_y < HEIGHT:
            if (curr_x, curr_y) != last_pos:
                last_pos = (curr_x, curr_y)
                
                if not revealed_map[curr_x][curr_y]:
                    if game_grid[curr_x][curr_y] == 1:
                        revealed_map[curr_x][curr_y] = True
                    else:
                        flood_fill_reveal(curr_x, curr_y, game_grid, revealed_map)
                    
                    print_board(game_grid, revealed_map, goal_pos)


    pass

