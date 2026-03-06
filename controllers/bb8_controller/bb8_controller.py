from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# A Pioneer 3-DX motorjai
left_motor = robot.getDevice('left wheel')
right_motor = robot.getDevice('right wheel')
rx = robot.getDevice('rx0')

if rx:
    rx.enable(timestep)
else:
    print("HIBA: Nem találtam 'rx0' Receivert a roboton!")

for m in [left_motor, right_motor]:
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

STATE_DECIDE = "DECIDE"
STATE_TURN = "TURN"
STATE_MOVE = "MOVE"

current_state = STATE_DECIDE
target_yaw = 0.0
target_gx, target_gy = 0.0, 0.0

print("🚀 Pioneer 3-DX Rendszer Inicializálva. Várakozás a radar adatokra...")

# Hozzáadunk egy memóriát az utolsó ismert adatokhoz, hogy ne vesszenek el!
last_dist_E, last_dist_N, last_dist_W, last_dist_S = 0.0, 0.0, 0.0, 0.0
last_curr_x, last_curr_y, last_current_yaw = 0.0, 0.0, 0.0

while robot.step(timestep) != -1:
    packet_received = False
    
    # Adatok olvasása és a memória frissítése
    while rx and rx.getQueueLength() > 0:
        data = rx.getString().split('|')
        last_dist_E, last_dist_N = float(data[0]), float(data[1])
        last_dist_W, last_dist_S = float(data[2]), float(data[3])
        last_curr_x, last_curr_y = float(data[4]), float(data[5])
        last_current_yaw = float(data[6])
        rx.nextPacket()
        packet_received = True

    # NAGYON FONTOS: Ha még SOHA nem kaptunk adatot, várunk!
    if last_dist_E == 0.0 and last_dist_N == 0.0 and last_dist_W == 0.0 and last_dist_S == 0.0:
        continue

    # A könnyebbség kedvéért átadjuk az aktuális értékeket a rövidebb változóknak
    dist_E, dist_N, dist_W, dist_S = last_dist_E, last_dist_N, last_dist_W, last_dist_S
    curr_x, curr_y, current_yaw = last_curr_x, last_curr_y, last_current_yaw

    if round(curr_x) == 9 and round(curr_y) == 9:
        print("🎉 CÉL ELÉRVE! (9,9) 🎉")
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break

    # --- 1. DÖNTÉS ---
    if current_state == STATE_DECIDE:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        # Lehetséges irányok: Szög, Távolság, X-lépés, Y-lépés, Név
        dirs = [
            (0.0, dist_E, 1, 0, "Kelet"),
            (math.pi/2, dist_N, 0, 1, "Észak"),
            (math.pi, dist_W, -1, 0, "Nyugat"),
            (-math.pi/2, dist_S, 0, -1, "Dél")
        ]
        
        # Sorba rakjuk aszerint, melyik visz közelebb a (9,9)-hez
        dirs.sort(key=lambda d: (9 - (round(curr_x) + d[2]))**2 + (9 - (round(curr_y) + d[3]))**2)
        
        chosen_dir = None
        for d in dirs:
            if d[1] > 0.8: # Ha szabad az út
                chosen_dir = d
                break
                
        if chosen_dir:
            target_yaw = chosen_dir[0]
            target_gx = round(curr_x) + chosen_dir[2]
            target_gy = round(curr_y) + chosen_dir[3]
            print(f"Mező: ({round(curr_x)}, {round(curr_y)}) -> Fordulás {chosen_dir[4]} felé, cél: ({target_gx}, {target_gy})")
            current_state = STATE_TURN
        else:
            print("CSAPDA! Nincs szabad útvonal.")
            break

    # --- 2. TÖKÉLETES FORGÁS HELYBEN ---
    elif current_state == STATE_TURN:
        error = target_yaw - current_yaw
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        
        # P-szabályozó a forgáshoz (bal kerék hátra, jobb kerék előre)
        turn_speed = max(-3.0, min(3.0, error * 2.0))
        left_motor.setVelocity(-turn_speed)
        right_motor.setVelocity(turn_speed)
        
        # Ha a szöghiba kisebb, mint 1 fok (0.02 radián)
        if abs(error) < 0.02:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            current_state = STATE_MOVE

    # --- 3. EGYENES GURULÁS A CSEMPÉRE ---
    elif current_state == STATE_MOVE:
        # Menet közben is korrigáljuk az egyenest, mint a sávtartó automatika
        error = target_yaw - current_yaw
        while error > math.pi: error -= 2*math.pi
        while error < -math.pi: error += 2*math.pi
        
        base_speed = 5.0
        correction = error * 2.0
        
        # Ha picit sodródna, a kerekek differenciálisan visszahúzzák egyenesbe
        left_motor.setVelocity(max(-5.2, min(5.2, base_speed - correction)))
        right_motor.setVelocity(max(-5.2, min(5.2, base_speed + correction)))
        
        # Ha beértünk a célpont (csempe közepe) 10 centis sugarába
        dist_to_center = math.sqrt((curr_x - target_gx)**2 + (curr_y - target_gy)**2)
        if dist_to_center < 0.1:
            print("Csempe közepe elérve. Megállás és újratervezés.")
            current_state = STATE_DECIDE