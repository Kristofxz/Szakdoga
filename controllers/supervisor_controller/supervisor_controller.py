"""
Webots Supervisor modul a környezet generálásához és kiértékeléséhez.

A szkript egy 10x10-es aknakereső rácsot hoz létre garantáltan
biztonságos útvonallal, és összehasonlítja a különböző vezérlő
algoritmusok (Supervisor, Greedy, Heuristic, PPO) teljesítményét.
"""

from controller import Supervisor
import random
import math
import csv

TIME_STEP = 32  # A szimulációs fizikai motor frissítési frekvenciája.
robot = Supervisor()  # A Supervisor entitás globális példányosítása.
root_children = robot.getRoot().getField("children")

robot_node = robot.getFromDef("ROBOT")
emitter = robot.getDevice("emitter")  # Adatok továbbítása.
receiver = robot.getDevice("rx_sup")  # Telemetria fogadása.
if receiver:
    receiver.enable(TIME_STEP)

# A robot alapértelmezett pozíciója az iterációk közötti visszaálláshoz.
INITIAL_TRANS = [0.0, 0.0, 0.0969]
INITIAL_ROT = [0.0, 0.0, 1.0, 0.0]  # Északi orientáció.

# A kiértékelendő navigációs algoritmusok listája.
CONTROLLERS = ["greedy_controller", "heuristic_controller", "tf_controller"]
NUM_MAPS = 100

# A determinisztikus véletlenszám-generálás a reprodukálhatóságért.
MAP_SEEDS = [8574 + i for i in range(NUM_MAPS)]

current_map_idx = 0
current_ctrl_idx = 0
steps_taken = 0
MAX_STEPS = 50000  # Holtpont elleni védelem maximális lépésszámmal.

game_walls = set()
NUM_MINES = 20
NUM_WALLS = 25

# A kiértékelési metrikákat rögzítő CSV fájl inicializálása.
with open('evaluation_results.csv', mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)
    writer.writerow(["Map_Seed", "Controller", "Result", "Steps", "Sim_Time_Sec"])

# Az első futtatandó vezérlő algoritmus betöltése.
controller_field = robot_node.getField("controller")
if controller_field:
    controller_field.setSFString(CONTROLLERS[current_ctrl_idx])


def generate_map():
    """
    Garantáltan megoldható, 10x10-es diszkrét rácsot generál.

    A start (0,0) és a cél (9,9) koordináták között létrehoz egy
    biztonságos útvonalat, majd a maradék helyekre aknákat generál.

    Returns:
        list: 2D mátrix a pálya adataival (1 = akna, 0 = tiszta).
    """
    global game_walls, current_map_idx
    game_walls.clear()

    random.seed(MAP_SEEDS[current_map_idx])

    grid = [[0 for _ in range(10)] for _ in range(10)]
    safe = set()
    safe_edges = set()
    cx, cy = 0, 0
    safe.add((cx, cy))

    # Kizárólag pozitív x és y irányú lépések engedélyezésével
    # garantált egy akadálymentes útvonal a célkoordinátákig.
    while cx < 9 or cy < 9:
        nx, ny = cx, cy
        if cx < 9 and (cy == 9 or random.random() < 0.5):
            nx += 1
        else:
            ny += 1

        # A gráf éleinek mindkét irányú rögzítése a gyors kereséshez.
        safe_edges.add(((cx, cy), (nx, ny)))
        safe_edges.add(((nx, ny), (cx, cy)))
        cx, cy = nx, ny
        safe.add((cx, cy))

    # A biztonságos útvonalon kívül eső cellák azonosítása,
    # majd a veszélyzónák véletlenszerű elhelyezése.
    cells = [(x, y) for x in range(10) for y in range(10) if (x, y) not in safe]
    random.shuffle(cells)

    for i in range(min(NUM_MINES, len(cells))):
        x, y = cells[i]
        grid[x][y] = 1

    # A topológia összes lehetséges élének begyűjtése.
    all_edges = []
    for x in range(10):
        for y in range(10):
            if x < 9:
                all_edges.append(((x, y), (x + 1, y)))
            if y < 9:
                all_edges.append(((x, y), (x, y + 1)))

    # A biztonságos útvonalat metsző élek kizárása.
    possible_walls = [
        e for e in all_edges
        if e not in safe_edges and (e[1], e[0]) not in safe_edges
    ]
    random.shuffle(possible_walls)

    # Az akadályok rögzítése a globális állapottérben.
    for i in range(min(NUM_WALLS, len(possible_walls))):
        e = possible_walls[i]
        game_walls.add(e)
        game_walls.add((e[1], e[0]))

    return grid


def spawn_everything(grid):
    """
    A logikai mátrix transzformációja 3D VRML objektumokká.

    Args:
        grid (list): A 10x10-es pálya logikai mátrixa.
    """
    # A csomópontok fordított sorrendű törlése megelőzi a listahibákat.
    for i in range(root_children.getCount() - 1, -1, -1):
        node = root_children.getMFNode(i)
        if node:
            def_name = node.getDef()
            if def_name in ["ARENA", "TILE_GROUP"]:
                root_children.removeMF(i)

    robot.step(TIME_STEP)

    # A statikus határolóelemek (aréna) definiálása VRML formátumban.
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

    # A diszkrét térbeli rács vizualizációja. A z=0.002 eltolás
    # megakadályozza az egymásba nyúló síkok renderelési hibáját.
    for x in range(10):
        for y in range(10):
            c = "0.7 0.7 0.7" if grid[x][y] == 0 else "0.8 0.1 0.1"
            t = f"Pose {{ translation {x} {y} 0.002 children [ Shape {{ appearance PBRAppearance {{ baseColor {c} roughness 1.0 }} geometry Plane {{ size 0.95 0.95 }} }} ] }}"
            tile_group.importMFNodeFromString(-1, t)

    spawned_walls = set()
    for e in game_walls:
        edge = tuple(sorted(e))
        # Deduplikáció a redundáns 3D objektumok elkerülése végett.
        if edge in spawned_walls:
            continue
        spawned_walls.add(edge)

        p1, p2 = edge
        x1, y1 = p1
        x2, y2 = p2
        c = "0.3 0.3 0.5"

        # A statikus akadályok befoglaló testének (Bounding Box) generálása.
        if x1 != x2:
            wx, wy = (x1 + x2) / 2.0, y1
            t = f"Solid {{ translation {wx} {wy} 0.25 children [ Shape {{ appearance PBRAppearance {{ baseColor {c} roughness 0.8 }} geometry Box {{ size 0.1 1.0 0.5 }} }} ] boundingObject Box {{ size 0.1 1.0 0.5 }} }}"
        else:
            wx, wy = x1, (y1 + y2) / 2.0
            t = f"Solid {{ translation {wx} {wy} 0.25 children [ Shape {{ appearance PBRAppearance {{ baseColor {c} roughness 0.8 }} geometry Box {{ size 1.0 0.1 0.5 }} }} ] boundingObject Box {{ size 1.0 0.1 0.5 }} }}"
        tile_group.importMFNodeFromString(-1, t)


def get_metal_signal(cx, cy, dx, dy):
    """
    Az irányított szenzor matematikai modellje takarási (occlusion) hatással.

    Args:
        cx (int): Aktuális x koordináta.
        cy (int): Aktuális y koordináta.
        dx (int): Érzékelési irány x vektora.
        dy (int): Érzékelési irány y vektora.

    Returns:
        float: A számított jelerősség.
    """
    nx, ny = cx + dx, cy + dy
    
    # Közvetlen fizikai akadály detektálása esetén blokkoló érték.
    if ((cx, cy), (nx, ny)) in game_walls:
        return 999.0

    signal = 0.0
    # Az érzékelési hatósugár kiértékelése maximum 3 egység távolságig.
    for i in range(1, 4):
        prev_x, prev_y = cx + dx * (i - 1), cy + dy * (i - 1)
        curr_x, curr_y = cx + dx * i, cy + dy * i

        # Ha akadály van a vonalon, az érzékelési sugár megszakad.
        if ((prev_x, prev_y), (curr_x, curr_y)) in game_walls:
            break

        if 0 <= curr_x <= 9 and 0 <= curr_y <= 9:
            # Inverz távolság-alapú jelcsillapodás modellje.
            if game_grid[curr_x][curr_y] == 1:
                if i == 1:
                    signal += 10.0
                elif i == 2:
                    signal += 3.0
                elif i == 3:
                    signal += 1.0
        else:
            # A munkatér határának elérése.
            if i == 1:
                signal += 100.0

    return signal


def print_final_statistics():
    """
    Aggregálja a CSV fájlban rögzített adatokat a kísérletsorozat végén.
    """
    import csv
    from collections import defaultdict

    stats = defaultdict(lambda: {"WIN": 0, "LOSS": 0, "TIMEOUT": 0, "STALL": 0,
                                 "total_steps": 0, "win_steps": 0, "runs": 0})

    try:
        with open('evaluation_results.csv', mode='r', encoding='utf-8') as file:
            reader = csv.DictReader(file)
            for row in reader:
                ctrl = row["Controller"]
                res = row["Result"]
                steps = int(row["Steps"])

                stats[ctrl]["runs"] += 1
                stats[ctrl][res] += 1
                stats[ctrl]["total_steps"] += steps
                if res == "WIN":
                    stats[ctrl]["win_steps"] += steps

        for ctrl, data in stats.items():
            runs = data["runs"]
            if runs == 0:
                continue
            win_rate = (data["WIN"] / runs) * 100
            avg_steps = data["total_steps"] / runs
            avg_win_steps = (data["win_steps"] / data["WIN"]) if data["WIN"] > 0 else 0

            print(f"\n>>> {ctrl.upper()} EREDMÉNYEK ({runs} pálya) <<<")
            print(f"Túlélési arány (WIN): {win_rate:.1f}% ({data['WIN']} nyert)")
            print(f"Felrobbant (LOSS):    {data['LOSS']} alkalommal")
            print(f"Elakadt/Időtúllépés:  {data['TIMEOUT'] + data['STALL']} alkalommal")
            print(f"Átlag lépésszám:      {avg_steps:.0f} (Sikereseknél: {avg_win_steps:.0f})")

    except Exception as e:
        print(f"Hiba a statisztikai adatok beolvasása során: {e}")


print(f"\n--- ÚJ KÖRNYEZET GENERÁLÁSA ({current_map_idx + 1}/{NUM_MAPS}) ---")
game_grid = generate_map()
spawn_everything(game_grid)

exploded = False
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")

# Fő szimulációs ciklus a folytonos idejű vezérléshez.
while robot.step(TIME_STEP) != -1:
    steps_taken += 1

    # Kommunikációs puffer aszinkron kiürítése.
    while receiver and receiver.getQueueLength() > 0:
        receiver.nextPacket()

    if robot_node and not exploded:
        # A robot egzakt térbeli állapotának kinyerése.
        pos = robot_node.getPosition()
        rot = robot_node.getOrientation()
        px, py = pos[0], pos[1]

        # Irányszög számítása a forgásmátrixból.
        yaw_angle = math.atan2(rot[3], rot[0])
        cx, cy = int(round(px)), int(round(py))

        # Terminálási feltételek kiértékelése.
        is_dead = (0 <= cx <= 9 and 0 <= cy <= 9 and game_grid[cx][cy] == 1)
        is_win = (cx >= 9 and cy >= 9)
        is_timeout = steps_taken > MAX_STEPS

        if is_dead or is_win or is_timeout:
            exploded = True
            sim_time_seconds = (steps_taken * TIME_STEP) / 1000.0

            if is_dead:
                outcome = "LOSS"
            elif is_win:
                outcome = "WIN"
            else:
                outcome = "TIMEOUT"

            # Az állapot átvitele a vezérlő algoritmus felé.
            if emitter:
                msg = f"{outcome if outcome != 'LOSS' else 'BOOM'}|{px:.2f}|{py:.2f}"
                emitter.send(msg.encode('utf-8'))

            # A futási adatok perzisztens rögzítése.
            with open('evaluation_results.csv', mode='a', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow([MAP_SEEDS[current_map_idx],
                                 CONTROLLERS[current_ctrl_idx],
                                 outcome, steps_taken, sim_time_seconds])

            print(f"Pálya: {current_map_idx + 1}/{NUM_MAPS} | Kontroller: {CONTROLLERS[current_ctrl_idx]} -> {outcome} ({sim_time_seconds:.2f} s)")

            # Átmenet a következő kísérleti iterációra.
            current_ctrl_idx += 1
            if current_ctrl_idx >= len(CONTROLLERS):
                current_ctrl_idx = 0
                current_map_idx += 1

                # Kísérletsorozat globális lezárása.
                if current_map_idx >= NUM_MAPS:
                    print_final_statistics()
                    print("\n=== KIÉRTÉKELÉSI FÁZIS LEZÁRVA ===")
                    robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                    break

                print(f"\n--- ÚJ KÖRNYEZET GENERÁLÁSA ({current_map_idx + 1}/{NUM_MAPS}) ---")

                game_grid = generate_map()
                spawn_everything(game_grid)

            # Az ágens felülírása a paramétertérben.
            controller_field.setSFString(CONTROLLERS[current_ctrl_idx])

            # Kinematikai állapot visszaállítása.
            for _ in range(10):
                if trans_field and rot_field:
                    trans_field.setSFVec3f(INITIAL_TRANS)
                    rot_field.setSFRotation(INITIAL_ROT)
                    robot_node.resetPhysics()

                    try:
                        robot_node.setVelocity([0, 0, 0, 0, 0, 0])
                    except Exception:
                        pass

                robot.step(TIME_STEP)

            # Aszinkron csatornák megtisztítása.
            while receiver and receiver.getQueueLength() > 0:
                receiver.nextPacket()

            steps_taken = 0
            exploded = False
            continue

        # Aktív szimulációs fázis: szenzoros értékek gyűjtése.
        sig_E = get_metal_signal(cx, cy, 1, 0)
        sig_N = get_metal_signal(cx, cy, 0, 1)
        sig_W = get_metal_signal(cx, cy, -1, 0)
        sig_S = get_metal_signal(cx, cy, 0, -1)

        # Szerializált adatátvitel az ágens felé.
        if emitter:
            msg = f"{sig_E:.2f}|{sig_N:.2f}|{sig_W:.2f}|{sig_S:.2f}|{px:.2f}|{py:.2f}|{yaw_angle:.2f}"
            emitter.send(msg.encode('utf-8'))