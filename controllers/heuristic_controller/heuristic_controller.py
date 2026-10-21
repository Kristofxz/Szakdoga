"""
Webots Ágens Modul: Heurisztikus (A*) Útvonalkereső Vezérlő.

A modul egy A-csillag gráfkereső algoritmussal kiegészített véges
állapotgépet implementál, amely dinamikusan frissíti a kognitív 
térképét, és elkerüli a halálos zónákat, illetve az ismert falakat.
"""

from controller import Robot
import math
import heapq

# Az ágens entitás példányosítása és a szimulációs időlépés kinyerése.
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Beavatkozó szervek és kommunikációs interfészek inicializálása.
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
rx = robot.getDevice('rx0')
if rx:
    rx.enable(timestep)

# A proprioceptív szenzorok aktiválása a kinematikai visszacsatoláshoz.
left_ps = left_motor.getPositionSensor()
right_ps = right_motor.getPositionSensor()
if left_ps:
    left_ps.enable(timestep)
if right_ps:
    right_ps.enable(timestep)

# A fizikai motor tranziens állapotainak stabilizálása.
for _ in range(2):
    robot.step(timestep)

# Az inkrementális pozíciószabályozás kezdeti értékeinek rögzítése.
current_left_target = left_ps.getValue() if left_ps else 0.0
current_right_target = right_ps.getValue() if right_ps else 0.0

left_motor.setPosition(current_left_target)
right_motor.setPosition(current_right_target)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# A differenciálhajtású robot fizikai paraméterei (méterben).
WHEEL_RADIUS = 0.0975
AXLE_LENGTH = 0.33

# Véges Állapotgép diszkrét állapotainak definíciója.
STATE_INIT_SCAN = "INIT_SCAN"
STATE_SCAN_TURN = "SCAN_TURN"
STATE_SCAN_READ = "SCAN_READ"
STATE_DECIDE = "DECIDE"
STATE_MOVE_TURN = "MOVE_TURN"
STATE_WAIT_MOVE_TURN = "WAIT_MOVE_TURN"
STATE_MOVE = "MOVE"

# Az állapotgép belső vezérlőváltozóinak inicializálása.
current_state = STATE_INIT_SCAN
timer = 0
scan_directions = [0.0, math.pi / 2, math.pi, -math.pi / 2]
current_scan_index = 0
scan_results = {}
target_yaw, target_gx, target_gy = 0.0, 0, 0

sig_E, sig_N, sig_W, sig_S = 0.0, 0.0, 0.0, 0.0
curr_x, curr_y, current_yaw = 0.0, 0.0, 0.0
has_data = False

# A kognitív térkép adatszerkezetei a környezet modellezésére.
heatmap_grid = [[-1.0 for _ in range(10)] for _ in range(10)]
known_cells = [[-1.0 for _ in range(10)] for _ in range(10)]
visited_tiles = set()
robot_known_walls = set()


def reset_robot_memory():
    """
    A belső állapot-reprezentáció teljes törlése.
    
    Egy új szimulációs iteráció megkezdésekor alaphelyzetbe 
    állítja a kognitív térképet és az állapotgép változóit.
    """
    global heatmap_grid, known_cells, visited_tiles, robot_known_walls
    global current_state, has_data, scan_results, current_scan_index
    global timer

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
    """
    A feltérképezett diszkrét állapottér terminál-alapú vizualizációja.

    Args:
        rx_int (int): A robot aktuális X rácskoordinátája.
        ry_int (int): A robot aktuális Y rácskoordinátája.
    """
    print("\n" + "=" * 34)
    print("      AKNA HŐTÉRKÉP")
    print("=" * 34)
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


def show_graphical_heatmap(rx_int, ry_int, title="Akna heatmap"):
    """
    A kognitív térkép nagy felbontású, grafikus exportálása.

    Args:
        rx_int (int): A robot aktuális X rácskoordinátája.
        ry_int (int): A robot aktuális Y rácskoordinátája.
        title (str): A kimentett ábra címe.
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np
        import copy

        # A mátrix transzponálása a megfelelő Descartes-rendszerhez.
        plot_data = np.array(heatmap_grid).T
        display_data = np.copy(plot_data)
        display_data = np.where(display_data >= 900.0, np.nan, display_data)
        display_data = np.where(display_data == -1.0, np.nan, display_data)

        fig, ax = plt.subplots(figsize=(8, 8))

        cmap = copy.copy(plt.get_cmap('plasma'))
        cmap.set_bad(color='#2b2b2b')

        cax = ax.imshow(
            display_data, origin='lower', cmap=cmap, vmin=0, vmax=15, 
            interpolation='nearest'
        )

        ax.set_xticks(np.arange(-0.5, 10, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, 10, 1), minor=True)
        ax.grid(which="minor", color="#555555", linestyle='-', linewidth=1.5)
        ax.tick_params(which="minor", bottom=False, left=False)

        for y in range(10):
            for x in range(10):
                val = plot_data[y, x]
                if x == rx_int and y == ry_int:
                    is_boom = "felrobbant" in title or "BUMM" in title
                    text_val = "B" if is_boom else "R"
                elif val == -1:
                    text_val = ""
                elif val >= 900:
                    text_val = ""
                else:
                    text_val = f"{val:.0f}"

                color = "black" if (0 < val > 8 and val < 900) else "white"

                if text_val:
                    ax.text(
                        x, y, text_val, ha="center", va="center", 
                        color=color, fontsize=12, fontweight='bold'
                    )

        # Falak szuperponálása.
        drawn_walls = set()
        for (p1, p2) in robot_known_walls:
            wall_edge = tuple(sorted((p1, p2)))
            if wall_edge in drawn_walls:
                continue
            drawn_walls.add(wall_edge)

            wx = (p1[0] + p2[0]) / 2.0
            wy = (p1[1] + p2[1]) / 2.0

            if p1[0] != p2[0]:
                ax.plot(
                    [wx, wx], [wy - 0.5, wy + 0.5], color='#ff4444', 
                    linewidth=6, solid_capstyle='round'
                )
            else:
                ax.plot(
                    [wx - 0.5, wx + 0.5], [wy, wy], color='#ff4444', 
                    linewidth=6, solid_capstyle='round'
                )

        cbar = fig.colorbar(cax, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label('Aknajelző emisszió erőssége', fontsize=12)

        ax.set_title(title, fontsize=15, fontweight='bold', pad=15)
        ax.set_xlabel('X koordináta', fontsize=12)
        ax.set_ylabel('Y koordináta', fontsize=12)
        ax.set_xticks(range(10))
        ax.set_yticks(range(10))

        is_boom = "felrobbant" in title or "BUMM" in title
        filename = "szakdoga_heatmap_lose.png" if is_boom else \
            "szakdoga_heatmap_win.png"
        
        plt.savefig(filename, dpi=200, bbox_inches='tight')
    except ImportError:
        pass


def move_wheels(left_rad, right_rad, speed=2.0):
    """
    A differenciálhajtású robot pozíció-alapú mozgásvezérlése.

    Args:
        left_rad (float): Bal kerék elfordulása radiánban.
        right_rad (float): Jobb kerék elfordulása radiánban.
        speed (float): A manőver végrehajtási sebessége.
    """
    global current_left_target, current_right_target
    current_left_target += left_rad
    current_right_target += right_rad

    left_motor.setPosition(current_left_target)
    right_motor.setPosition(current_right_target)
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)


def find_best_path(start_x, start_y, goal_x=9, goal_y=9):
    """
    Heurisztikus útvonalkereső algoritmus (A*).

    Kiszámítja az optimális útvonalat a lokális tudásbázis alapján.

    Args:
        start_x (int): Indulási X koordináta.
        start_y (int): Indulási Y koordináta.
        goal_x (int): Cél X koordináta.
        goal_y (int): Cél Y koordináta.

    Returns:
        list or None: A koordináták listája a célig, vagy None.
    """
    open_set = []
    # A prioritási sor inicializálása az f(n) költség alapján.
    heapq.heappush(open_set, (0, (start_x, start_y)))
    came_from = {}
    g_score = {(start_x, start_y): 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        cx, cy = current

        # Terminálási feltétel: a globális célkoordináta elérése.
        if cx == goal_x and cy == goal_y:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        # Az érvényes 4-irányú szomszédos csomópontok bejárása.
        for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
            nx, ny = cx + dx, cy + dy

            if 0 <= nx <= 9 and 0 <= ny <= 9:
                # Az ismert statikus akadályok kizárása.
                if ((cx, cy), (nx, ny)) in robot_known_walls:
                    continue

                cell_val = known_cells[nx][ny]
                
                # A halálos zónák kihagyása.
                if cell_val >= 10.0:
                    continue

                # Dinamikus költségfüggvény meghatározása.
                step_cost = 1.0
                if cell_val == -1:
                    step_cost = 1.0
                else:
                    step_cost += cell_val * 0.3

                # Hurok-elkerülési mechanizmus.
                if (nx, ny) in visited_tiles:
                    step_cost += 1.5

                tentative_g = g_score.get(current, 0) + step_cost

                # Állapottér frissítése rövidebb útvonal esetén.
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    
                    # Heurisztika (Manhattan) hozzáadása.
                    f_score = tentative_g + (
                        abs(goal_x - nx) + abs(goal_y - ny)
                    )
                    heapq.heappush(open_set, (f_score, (nx, ny)))

    return None


# Fő szimulációs és vezérlési ciklus.
while robot.step(timestep) != -1:
    reset_triggered = False

    # Aszinkron kommunikáció és telemetria feldolgozása.
    while rx and rx.getQueueLength() > 0:
        data_str = rx.getString()
        data = data_str.split('|')
        rx.nextPacket()

        # Végállapotok detektálása a Supervisor felől.
        if data[0] in ["BOOM", "WIN", "STALL"]:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            px = int(round(float(data[1])))
            py = int(round(float(data[2])))

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

            # Az epizodikus memória törlése az új környezethez.
            reset_robot_memory()
            reset_triggered = True

        # Az állapotvektor integrálása.
        elif len(data) >= 7:
            sig_E = float(data[0])
            sig_N = float(data[1])
            sig_W = float(data[2])
            sig_S = float(data[3])
            
            curr_x = float(data[4])
            curr_y = float(data[5])
            current_yaw = float(data[6])
            has_data = True

    # Állapot-védelmi megszakítások.
    if reset_triggered:
        continue
    if not has_data:
        continue

    # Koordináták kvantálása biztonsági korlátozással.
    rx_int, ry_int = round(curr_x), round(curr_y)
    rx_int = max(0, min(9, rx_int))
    ry_int = max(0, min(9, ry_int))

    # A véges állapotgép logikájának végrehajtása.
    if current_state == STATE_INIT_SCAN:
        # A kognitív térkép inicializálása biztonságos pozícióval.
        visited_tiles.add((rx_int, ry_int))
        heatmap_grid[rx_int][ry_int] = 0.0
        known_cells[rx_int][ry_int] = 0.0
        scan_results.clear()
        current_scan_index = 0
        current_state = STATE_SCAN_TURN

    elif current_state == STATE_SCAN_TURN:
        # Orientáció illesztése a vizsgálandó környezeti vektorhoz.
        target_angle = scan_directions[current_scan_index]
        error = target_angle - current_yaw
        
        # A szöghiba normalizálása a legrövidebb forgási pályához.
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        wheel_turn = (error * (AXLE_LENGTH / 2.0)) / WHEEL_RADIUS
        move_wheels(-wheel_turn, wheel_turn, speed=1.5)
        current_state = STATE_SCAN_READ
        timer = 0

    elif current_state == STATE_SCAN_READ:
        # Várakozás a mechanikai tranziensek lecsengésére.
        timer += 1
        if timer > 70:
            if current_scan_index == 0:
                current_signal = sig_E
            elif current_scan_index == 1:
                current_signal = sig_N
            elif current_scan_index == 2:
                current_signal = sig_W
            else:
                current_signal = sig_S

            scan_results[scan_directions[current_scan_index]] = current_signal
            current_scan_index += 1

            # Ha a teljes analízis lezárult, következik az integráció.
            if current_scan_index < 4:
                current_state = STATE_SCAN_TURN
            else:
                for angle, signal in scan_results.items():
                    nx = int(round(rx_int + math.cos(angle)))
                    ny = int(round(ry_int + math.sin(angle)))

                    if 0 <= nx <= 9 and 0 <= ny <= 9:
                        # Akadály gráf-élként történő rögzítése.
                        if signal > 900.0:
                            robot_known_walls.add(
                                ((rx_int, ry_int), (nx, ny))
                            )
                            robot_known_walls.add(
                                ((nx, ny), (rx_int, ry_int))
                            )
                        else:
                            # Halálos akadályok transzformációja.
                            if signal >= 10.0:
                                known_cells[nx][ny] = 999.0
                            else:
                                known_cells[nx][ny] = signal

                            # A hőtérkép pesszimista frissítése.
                            if (nx, ny) not in visited_tiles:
                                current_val = heatmap_grid[nx][ny]
                                if current_val == -1:
                                    heatmap_grid[nx][ny] = signal
                                else:
                                    heatmap_grid[nx][ny] = max(
                                        current_val, signal
                                    )

                print_heatmap(rx_int, ry_int)
                current_state = STATE_DECIDE

    elif current_state == STATE_DECIDE:
        # Globális útvonaltervezés inicializálása a tudásbázis alapján.
        path = find_best_path(rx_int, ry_int)

        if path:
            # Sikeres tervezés esetén az első szegmens kijelölése.
            target_gx, target_gy = path[0]
            target_yaw = math.atan2(target_gy - ry_int, target_gx - rx_int)
            current_state = STATE_MOVE_TURN
        else:
            print("Az A* nem talál érvényes utat a célhoz!")

    elif current_state == STATE_MOVE_TURN:
        # Átváltás sebességszabályozásra a zárt hurkú vezérléshez.
        left_motor.setPosition(float('inf'))
        right_motor.setPosition(float('inf'))
        current_state = STATE_WAIT_MOVE_TURN

    elif current_state == STATE_WAIT_MOVE_TURN:
        # Valós idejű szöghiba számítása a telemetria alapján.
        error = target_yaw - current_yaw
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        # Célérkezési tolerancia alkalmazása (Zénón-paradoxon).
        if abs(error) < 0.05:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            current_state = STATE_MOVE
        else:
            # Proporcionális (P) irányszög-szabályozó futtatása.
            speed = max(1.0, min(5.0, abs(error) * 8.0))
            if error > 0:
                left_motor.setVelocity(-speed)
                right_motor.setVelocity(speed)
            else:
                left_motor.setVelocity(speed)
                right_motor.setVelocity(-speed)

    elif current_state == STATE_MOVE:
        # Folytonos Euklideszi távolságmérés a cél rácspontig.
        dist_to_go = math.sqrt(
            (curr_x - target_gx)**2 + (curr_y - target_gy)**2
        )

        # Pozíciótűrés: ha elég közel van a cella közepéhez, megáll.
        if dist_to_go < 0.15:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

            # Visszaállás inkrementális pozícióvezérlésre.
            current_left_target = left_ps.getValue() if left_ps else 0.0
            current_right_target = right_ps.getValue() if right_ps else 0.0
            
            left_motor.setPosition(current_left_target)
            right_motor.setPosition(current_right_target)

            current_state = STATE_INIT_SCAN
        else:
            # Dinamikus útvonalkövetés előrehaladás közben.
            target_angle = math.atan2(target_gy - curr_y, target_gx - curr_x)
            error = target_angle - current_yaw
            
            while error > math.pi:
                error -= 2 * math.pi
            while error < -math.pi:
                error += 2 * math.pi

            base_speed = 5.0
            correction = error * 8.0

            # Aszimmetrikus hajtás a letérés kompenzálására.
            left_speed = max(-10.0, min(10.0, base_speed - correction))
            right_speed = max(-10.0, min(10.0, base_speed + correction))
            
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)