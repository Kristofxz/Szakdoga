"""
Webots Ágens Modul: Mohó (Greedy) Útvonalkereső Vezérlő.

A modul egy mohó algoritmussal vezérelt véges állapotgépet
implementál, amely a lokális célfüggvény minimalizálására törekszik 
a jövőbeli állapotok kiterjesztett vizsgálata nélkül.
"""

from controller import Robot
import math

# Az ágens entitás példányosítása és a szimulációs időlépés kinyerése.
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Beavatkozó szervek és kommunikációs interfészek inicializálása.
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
rx = robot.getDevice('rx0')
if rx:
    rx.enable(timestep)

# A proprioceptív szenzorok aktiválása a kinematikai számításokhoz.
left_ps = left_motor.getPositionSensor()
right_ps = right_motor.getPositionSensor()
if left_ps:
    left_ps.enable(timestep)
if right_ps:
    right_ps.enable(timestep)

# A fizikai motor tranziens állapotainak stabilizálása.
for _ in range(2):
    robot.step(timestep)

# Az inkrementális pozíciószabályozás alapértékeinek rögzítése.
current_left_target = left_ps.getValue() if left_ps else 0.0
current_right_target = right_ps.getValue() if right_ps else 0.0

left_motor.setPosition(current_left_target)
right_motor.setPosition(current_right_target)
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# A robot fizikai modelljének kinematikai konstansai (méterben).
WHEEL_RADIUS = 0.0975
AXLE_LENGTH = 0.33

# Véges Állapotgép állapotainak definíciója.
STATE_INIT_SCAN = "INIT_SCAN"
STATE_SCAN_TURN = "SCAN_TURN"
STATE_SCAN_READ = "SCAN_READ"
STATE_DECIDE = "DECIDE"
STATE_MOVE_TURN = "MOVE_TURN"
STATE_WAIT_MOVE_TURN = "WAIT_MOVE_TURN"
STATE_MOVE = "MOVE"

# Az állapotgép belső változóinak inicializálása.
current_state = STATE_INIT_SCAN
timer = 0
scan_directions = [0.0, math.pi / 2, math.pi, -math.pi / 2]
current_scan_index = 0
scan_results = {}
target_yaw, target_gx, target_gy = 0.0, 0, 0

sig_E, sig_N, sig_W, sig_S = 0.0, 0.0, 0.0, 0.0
curr_x, curr_y, current_yaw = 0.0, 0.0, 0.0
has_data = False

# A lokális kognitív térkép adatszerkezetei.
heatmap_grid = [[-1.0 for _ in range(10)] for _ in range(10)]
visited_tiles = set()
robot_known_walls = set()


def print_heatmap(rx_int, ry_int):
    """
    A diszkrét állapottér terminál-alapú vizualizációja.

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
    A lokális tudásbázis nagy felbontású, grafikus exportálása.

    Args:
        rx_int (int): A robot aktuális X rácskoordinátája.
        ry_int (int): A robot aktuális Y rácskoordinátája.
        title (str): A kimentett ábra címe.
    """
    try:
        import matplotlib.pyplot as plt
        import numpy as np

        # A mátrix transzponálása a megfelelő térbeli orientációhoz.
        plot_data = np.array(heatmap_grid).T
        plt.figure(figsize=(8, 8))
        plt.imshow(
            plot_data, origin='lower', cmap='hot', 
            interpolation='nearest', vmin=-1
        )

        for y in range(10):
            for x in range(10):
                val = plot_data[y, x]
                if x == rx_int and y == ry_int:
                    text_val = "B" if "BUMM" in title else "R"
                else:
                    text_val = "X" if val == -1 else f"{val:.0f}"
                
                color = "white" if val > 3 else "black"
                plt.text(
                    x, y, text_val, ha="center", va="center", 
                    color=color, fontsize=12, fontweight='bold'
                )

        # A detektált akadályok szuperponálása.
        for (p1, p2) in robot_known_walls:
            wx = (p1[0] + p2[0]) / 2.0
            wy = (p1[1] + p2[1]) / 2.0
            if p1[0] != p2[0]:
                plt.plot(
                    [wx, wx], [wy - 0.5, wy + 0.5], 
                    color='#00aaff', linewidth=5
                )
            else:
                plt.plot(
                    [wx - 0.5, wx + 0.5], [wy, wy], 
                    color='#00aaff', linewidth=5
                )

        plt.colorbar(label='Jelerősség')
        plt.title(title)
        plt.xlabel('X ')
        plt.ylabel('Y')
        plt.xticks(range(10))
        plt.yticks(range(10))

        is_boom = "BUMM" in title
        filename = "szakdoga_heatmap_lose.png" if is_boom else \
            "szakdoga_heatmap_win.png"
        
        plt.savefig(filename)
        print(f"\nKép elmentve '{filename}' néven a projekt mappájába!")
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


# Fő szimulációs és vezérlési ciklus.
while robot.step(timestep) != -1:

    # 1. Aszinkron kommunikáció és telemetria feldolgozása.
    while rx and rx.getQueueLength() > 0:
        data_str = rx.getString()
        data = data_str.split('|')

        # Végállapot detektálása a kapott üzenet alapján.
        if data[0] == "BOOM":
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            
            death_x = int(round(float(data[1])))
            death_y = int(round(float(data[2])))
            
            print("\n[BUMM] Ráléptél egy aknára!")
            print_heatmap(death_x, death_y)
            show_graphical_heatmap(
                death_x, death_y, title="A robot felrobbant!"
            )
            break

        # Az állapotvektor parszeolása.
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

    # Megszakítás kényszerítése külső halál-esemény esetén.
    if 'data_str' in locals() and data_str.startswith("BOOM"):
        break

    # Szinkronizációs blokkoló valid adatcsomag beérkezéséig.
    if not has_data:
        continue

    # A valós koordináták kvantálása biztonsági korlátozással.
    rx_int, ry_int = round(curr_x), round(curr_y)
    rx_int = max(0, min(9, rx_int))
    ry_int = max(0, min(9, ry_int))

    # A globális terminálási feltétel kiértékelése.
    if rx_int >= 9 and ry_int >= 9:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        print("[SIKER] CÉL ELÉRVE!")
        print_heatmap(rx_int, ry_int)
        show_graphical_heatmap(rx_int, ry_int, title="Túlélte a robot.")
        break

    # 2. A véges állapotgép logikájának végrehajtása.
    if current_state == STATE_INIT_SCAN:
        # Az aktuális pozíció biztonságosként való rögzítése.
        visited_tiles.add((rx_int, ry_int))
        heatmap_grid[rx_int][ry_int] = 0.0

        scan_results.clear()
        current_scan_index = 0
        current_state = STATE_SCAN_TURN

    elif current_state == STATE_SCAN_TURN:
        # A szenzor-orientáció beállítása a következő égtáj felé.
        target_angle = scan_directions[current_scan_index]
        error = target_angle - current_yaw

        # A szöghiba normalizálása a legrövidebb forgási úthoz.
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        # Inverz kinematikai transzformáció.
        wheel_turn = (error * (AXLE_LENGTH / 2.0)) / WHEEL_RADIUS
        move_wheels(-wheel_turn, wheel_turn, speed=1.5)
        current_state = STATE_SCAN_READ
        timer = 0

    elif current_state == STATE_SCAN_READ:
        # Nyílt hurkú várakozás a mechanikai stabilizálódásra.
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
            
            # Ha az összes irány kiértékelése megtörtént.
            if current_scan_index < 4:
                current_state = STATE_SCAN_TURN
            else:
                for angle, signal in scan_results.items():
                    nx = int(round(rx_int + math.cos(angle)))
                    ny = int(round(ry_int + math.sin(angle)))
                    
                    if 0 <= nx <= 9 and 0 <= ny <= 9:
                        # Ha a jel kritikus, az akadály rögzítésre kerül.
                        if signal > 900.0:
                            robot_known_walls.add(
                                ((rx_int, ry_int), (nx, ny))
                            )
                            robot_known_walls.add(
                                ((nx, ny), (rx_int, ry_int))
                            )
                        # Ellenkező esetben a hőtérkép frissítése.
                        elif (nx, ny) not in visited_tiles:
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
        # A mohó döntéshozatali algoritmus.
        best_score = 99999
        chosen_yaw = None

        for angle, signal in scan_results.items():
            nx = int(round(rx_int + math.cos(angle)))
            ny = int(round(ry_int + math.sin(angle)))

            # Csak a valid és nem végzetes cellák értékelése.
            if 0 <= nx <= 9 and 0 <= ny <= 9:
                if signal < 10.0:
                    # 1. Heurisztikus tag: Euklideszi távolság.
                    d_goal = math.sqrt((9 - nx)**2 + (9 - ny)**2)
                    
                    # 2. Hurok-elkerülési büntetés.
                    if (nx, ny) in visited_tiles:
                        visited_penalty = 50.0 
                    else:
                        visited_penalty = 0.0
                        
                    # A célfüggvény lineáris kombinációja.
                    score = (signal * 5.0) + d_goal + visited_penalty

                    # Az optimális paraméterek rögzítése.
                    if score < best_score:
                        best_score = score
                        chosen_yaw = angle
                        target_gx = nx
                        target_gy = ny

        # A kiválasztott optimális lépés inicializálása.
        if chosen_yaw is not None:
            target_yaw = chosen_yaw
            current_state = STATE_MOVE_TURN
        else:
            print("Teljesen beszorult, nincs hova lépni!")
            current_state = STATE_INIT_SCAN

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

        # Célérkezési tolerancia (0.05 radián).
        if abs(error) < 0.05:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            current_state = STATE_MOVE
        else:
            # Proporcionális irányszög-szabályozó.
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

        # Pozíciótűrés (0.15 méter).
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
            # Kiszámolja az ideális vektort az aktuális pozícióból.
            target_angle = math.atan2(target_gy - curr_y, target_gx - curr_x)
            error = target_angle - current_yaw
            
            while error > math.pi:
                error -= 2 * math.pi
            while error < -math.pi:
                error += 2 * math.pi

            # Alapsebesség + Proporcionális korrekció.
            base_speed = 5.0
            correction = error * 8.0

            # A motorok aszimmetrikus hajtása a letérés kompenzálására.
            left_speed = max(-10.0, min(10.0, base_speed - correction))
            right_speed = max(-10.0, min(10.0, base_speed + correction))
            
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)