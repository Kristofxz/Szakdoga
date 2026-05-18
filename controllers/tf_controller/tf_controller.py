import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from controller import Robot

class MinesweeperEnv(gym.Env):
    def __init__(self):
        super(MinesweeperEnv, self).__init__()
        
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        
        self.rx = self.robot.getDevice('rx0')
        self.rx.enable(self.timestep)
        self.tx = self.robot.getDevice('tx0') 

        self.action_space = spaces.Discrete(4)
        
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(14,), dtype=np.float32)
        
        self.visited_tiles = {}
        self.max_steps = 300 
        self.current_step = 0
        self.global_steps = 0 
        self.last_data = None
        self.last_euclidean_dist = 999.0

    def _step_webots(self):
        if self.robot.step(self.timestep) == -1:
            exit(0)
            
        latest_data = None
        while self.rx.getQueueLength() > 0:
            data_str = self.rx.getString()
            self.rx.nextPacket()
            data = data_str.split('|')
            
            if data[0] in ["BOOM", "WIN", "STALL"]:
                latest_data = {"type": data[0]}
            elif len(data) >= 7:
                latest_data = {
                    "type": "STATE",
                    "sigs": [float(data[0]), float(data[1]), float(data[2]), float(data[3])],
                    "px": float(data[4]), "py": float(data[5]), "yaw": float(data[6])
                }
        return latest_data

    def _turn_to_angle(self, target_yaw, is_radar=False):
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        timeout = 0 
        max_turn_steps = 150 
        
        while timeout < max_turn_steps:
            timeout += 1
            data = self._step_webots()
            if data is None: continue
            if data["type"] != "STATE": return data 
            
            curr_yaw = data["yaw"]
            error = target_yaw - curr_yaw
            while error > math.pi: error -= 2*math.pi
            while error < -math.pi: error += 2*math.pi
            
            if abs(error) < 0.05:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                return data
                
            if is_radar:
                speed = 3.0  
            else:
                speed = max(2.0, min(10.0, abs(error) * 10.0))
                
            if error > 0:
                self.left_motor.setVelocity(-speed); self.right_motor.setVelocity(speed)
            else:
                self.left_motor.setVelocity(speed); self.right_motor.setVelocity(-speed)
                
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        return self.last_data 
        
    def _radar_sweep(self):
        angles = [0.0, math.pi/2, math.pi, -math.pi/2]
        for angle in angles:
            new_data = self._turn_to_angle(angle, is_radar=True)
            if new_data and new_data["type"] == "STATE":
                self.last_data = new_data
            for _ in range(5): 
                self._step_webots()
        return self.last_data

    def _get_euclidean_distances(self, rx, ry):
        target_x, target_y = 9, 9
        curr_dist = math.sqrt((target_x - rx)**2 + (target_y - ry)**2)
        
        dirs = [(1,0), (0,1), (-1,0), (0,-1)]
        neighbor_dists = []
        for dx, dy in dirs:
            nx, ny = rx + dx, ry + dy
            if 0 <= nx <= 9 and 0 <= ny <= 9:
                dist = math.sqrt((target_x - nx)**2 + (target_y - ny)**2)
                neighbor_dists.append(dist)
            else:
                neighbor_dists.append(999.0) 
                
        return curr_dist, neighbor_dists

    def _build_obs(self, data):
        rx_int = max(0, min(9, round(data["px"])))
        ry_int = max(0, min(9, round(data["py"])))
        
        dirs = [(1,0), (0,1), (-1,0), (0,-1)]
        scaled_sigs = []
        
        for i, (dx, dy) in enumerate(dirs):
            sig = data["sigs"][i]
            if sig >= 10.0:
                scaled_sig = 1.0  
            else:
                scaled_sig = sig / 10.0  
            scaled_sigs.append(scaled_sig)
            
        current_eucl_dist, eucl_neighbors = self._get_euclidean_distances(rx_int, ry_int)
        self.last_euclidean_dist = current_eucl_dist
        
        norm_eucl = [min(1.0, d / 15.0) for d in eucl_neighbors]
        
        visited_adj = []
        for dx, dy in dirs:
            nx, ny = rx_int + dx, ry_int + dy
            v = self.visited_tiles.get((nx, ny), 0)
            visited_adj.append(min(1.0, v / 10.0))
        
        obs = np.concatenate([
            [rx_int / 9.0, ry_int / 9.0],  
            scaled_sigs,                   
            norm_eucl,                     
            visited_adj                    
        ]).astype(np.float32)
        
        return obs, rx_int, ry_int

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.visited_tiles.clear()
        self.visited_tiles[(0, 0)] = 1
        self.current_step = 0
        self.last_euclidean_dist = 999.0
        
        level = min(5, (self.global_steps // 50000) + 1)
        if self.tx: 
            self.tx.send(f"RESET|{level}".encode('utf-8'))
            
        while True:
            data = self._step_webots()
            if data and data["type"] == "STATE":
                if data["px"] < 0.5 and data["py"] < 0.5:
                    self.last_data = data
                    self._radar_sweep() 
                    obs, _, _ = self._build_obs(self.last_data)
                    return obs, {}

    def _move_to_cell(self, target_gx, target_gy):
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        last_d = None
        for _ in range(250): 
            d = self._step_webots()
            if d: last_d = d
            if last_d and last_d["type"] != "STATE": break
            
            if last_d and last_d["type"] == "STATE":
                curr_x, curr_y, curr_yaw = last_d["px"], last_d["py"], last_d["yaw"]
                dist = math.sqrt((target_gx - curr_x)**2 + (target_gy - curr_y)**2)
                if dist < 0.15: break
                    
                target_angle = math.atan2(target_gy - curr_y, target_gx - curr_x)
                error = target_angle - curr_yaw
                while error > math.pi: error -= 2*math.pi
                while error < -math.pi: error += 2*math.pi
                
                base_speed = 6.0 
                correction = error * 8.0 
                
                left_speed = max(-10.0, min(10.0, base_speed - correction))
                right_speed = max(-10.0, min(10.0, base_speed + correction))
                
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        for _ in range(3):
            d = self._step_webots()
            if d: last_d = d
        return last_d

    def step(self, action):
        self.current_step += 1
        self.global_steps += 1 
        
        target_angles = [0.0, math.pi/2, math.pi, -math.pi/2] 
        target_yaw = target_angles[action]
        
        if self.last_data is None or self.last_data["type"] != "STATE":
            return np.zeros(14, dtype=np.float32), 0.0, True, False, {}
            
        data = self.last_data
        sig_for_action = data["sigs"][action]
        obs_old, rx_int, ry_int = self._build_obs(data)
        
        old_eucl_dist = self.last_euclidean_dist
        
        reward = -2.0 
        terminated = False
        truncated = False
        
        if sig_for_action >= 100.0: 
            reward -= 300.0 
            obs, _, _ = self._build_obs(self.last_data)
        else:
            self._turn_to_angle(target_yaw)
            target_gx = rx_int + round(math.cos(target_yaw))
            target_gy = ry_int + round(math.sin(target_yaw))
            
            final_data = self._move_to_cell(target_gx, target_gy)
            
            if final_data and final_data["type"] in ["BOOM", "WIN", "STALL"]:
                terminated = True
                if final_data["type"] == "WIN": reward += 2000.0
                elif final_data["type"] == "BOOM": reward -= 300.0 
                obs = np.zeros(14, dtype=np.float32)
            else:
                if final_data: self.last_data = final_data
                
                self._radar_sweep() 
                obs, new_rx, new_ry = self._build_obs(self.last_data)
                
                if new_rx == rx_int and new_ry == ry_int:
                    reward -= 300.0 
                    terminated = True 
                else:
                    new_eucl_dist = self.last_euclidean_dist
                    
                    progress = old_eucl_dist - new_eucl_dist
                    progress_clipped = max(-3.0, min(3.0, progress))
                    reward += progress_clipped * 10.0
                    
                    visits = self.visited_tiles.get((new_rx, new_ry), 0)
                    if visits == 0:
                        reward += 5.0 
                    else:
                        reward -= min(20.0, visits * 5.0) 
                        
                    self.visited_tiles[(new_rx, new_ry)] = visits + 1

        if self.current_step >= self.max_steps:
            truncated = True
            if not terminated: reward -= 1000.0 
            
        return obs, float(reward), terminated, truncated, {}

# if __name__ == "__main__":
    # env = MinesweeperEnv()
    
    # CHECKPOINT_DIR = r"C:\Users\Kristof\Documents\szakdolgozat\controllers\tf_controller\mentett_modellek"
    # MODEL_PATH = os.path.join(CHECKPOINT_DIR, "v8_final_model_complete.zip")
    
    # checkpoint_callback = CheckpointCallback(save_freq=5000, save_path=CHECKPOINT_DIR, name_prefix='v8_aggressive_fast')

    # if os.path.exists(MODEL_PATH):
        # print(f"🔄 Modell betöltése: {MODEL_PATH}")
        # model = PPO.load(MODEL_PATH, env=env)
    # else:
        # print("🚀 Új, Agresszív (v8) tanulás indítása...")
        # policy_kwargs = dict(net_arch=[dict(pi=[64, 64], vf=[64, 64])])
        # model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1, 
                    # learning_rate=0.001, ent_coef=0.1, clip_range=0.2, 
                    # n_steps=1024, batch_size=64)

    # print("Tanítás megkezdése...")
    # try:
        # model.learn(total_timesteps=500_000, reset_num_timesteps=False, callback=checkpoint_callback)
        # model.save(os.path.join(CHECKPOINT_DIR, "v8_final_model_complete"))
        # print("Tanítás befejezve és modell elmentve.")
        
    # except Exception as e:
        # print(f"\n❌ Tanítás megszakadt ({e}), utolsó állapot mentése...")
        # model.save(os.path.join(CHECKPOINT_DIR, "v8_interrupted_model"))
        
if __name__ == "__main__":
    env = MinesweeperEnv()
    
    CHECKPOINT_DIR = r"C:\Users\Kristof\Documents\szakdolgozat\controllers\tf_controller\mentett_modellek"
    MODEL_PATH = os.path.join(CHECKPOINT_DIR, "v8_final_model_complete.zip")
    
    if os.path.exists(MODEL_PATH):
        print(f"Kész modell betöltése teszteléshez: {MODEL_PATH}")
        model = PPO.load(MODEL_PATH, env=env)
    else:
        print("Nem található a betanított modell! Ellenőrizd az elérési utat.")
        exit()

    print(" Tesztelési (Kijátszási) mód indítása...")
    
    obs, info = env.reset()
    
    while True:
        action, _states = model.predict(obs, deterministic=False)
        
        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated or truncated:
            print(f"🏁 Epizód vége (Jutalom az utolsó lépésnél: {reward}). Új pálya generálása...")
            obs, info = env.reset()