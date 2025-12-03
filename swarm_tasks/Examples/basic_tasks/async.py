import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
import swarm_tasks
import time
from swarm_tasks.simulation import simulation as sim
from swarm_tasks.simulation import visualizer as viz
import swarm_tasks.controllers.potential_field as potf
from swarm_tasks.modules.dispersion import disp_field
import swarm_tasks.controllers.base_control as base_control
from swarm_tasks.modules.aggregation import aggr_centroid, aggr_field
from swarm_tasks.modules import exploration as exp
from swarm_tasks.tasks import area_coverage as cvg
from math import radians, sin, cos, sqrt, atan2, asin, degrees
from dronekit import connect, VehicleMode, LocationGlobalRelative
from navigate import NavigationGridGenerator
from groupsplitauto import AutoSplitMission
from search_grid import SearchGridGenerator
from groupsplitspecific import SpecificSplitMission
import socket, json, csv
import locatePosition
import netifaces, math
import asyncio
from concurrent.futures import ProcessPoolExecutor

# Globals and initial settings
goal_table = []
file_name = "hanumanthapuram"
master_num = -1
master_flag = False
cwd = os.getcwd()

same_height = 30
different_height = [300, 320, 340, 360, 380, 400, 420, 440, 460, 480]

disperse_multiple_goals = []
start_multiple_goals = []
return_multiple_goals = []
goal_points = []
agg_goal_point = []
origin = (12.582228, 79.865131)  # based on file_name

nextwaypoint = 0
num_bots = 10
vehicles = []
port_array = [14551, 14552, 14553, 14554, 14555, 14556, 14557, 14558, 14559, 14560]

port_dict = {
    1: 14551,
    2: 14552,
    3: 14553,
    4: 14554,
    5: 14555,
    6: 14556,
    7: 14557,
    8: 14558,
    9: 14559,
    10: 1460
}

pos_array = []
heartbeat_ip = ["192.168.6.210", "192.168.6.210", "192.168.6.210", "192.168.6.154", "192.168.6.155"]
heartbeat_ip_timeout = [30, 30, 30, 30, 30]

goal_path_csv_array = []
goal_path_csv_array_flag = False
skip_wp_flag = False
next_wp = 0
home_pos = []
uav_home_pos = []


def get_wifi_ip():
    valid_prefixes = ("eth", "enp", "ens", "wlan", "wlp")
    for iface in netifaces.interfaces():
        if iface.startswith(valid_prefixes):
            try:
                addrs = netifaces.ifaddresses(iface)
                ipv4_info = addrs.get(netifaces.AF_INET, [])
                for addr in ipv4_info:
                    ip = addr.get('addr')
                    if ip and ip.startswith("192.168."):
                        return ip
            except Exception as e:
                print(f"Error on interface {iface}: {e}")
    return None


def home_lock():
    global vehicles, home_pos_lat_lon, home_pos
    home_pos = []
    home_pos_lat_lon = []
    for i, vehicle in enumerate(vehicles):
        timeout = time.time() + 30  # 30 seconds timeout
        while not vehicle.home_location:
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not vehicle.home_location:
                print(" Waiting for home position...")
                time.sleep(1)
            if time.time() > timeout:
                print(f"Timeout waiting for home_location for vehicle {i}")
                break

        home = vehicle.home_location
        if home is None:
            continue

        x, y = locatePosition.geoToCart(origin, endDistance, [home.lat, home.lon])
        home_pos_lat_lon.append((home.lat, home.lon))
        home_pos.append((x / 2, y / 2))
    return 1


def CHECK_network_connection():
    global heartbeat_ip_timeout, heartbeat_ip
    for i, iter_follower in enumerate(heartbeat_ip_timeout):
        response = os.system('ping -c 1 ' + heartbeat_ip[i])
        if response == 0:
            heartbeat_ip_timeout[i] = 30
        else:
            print("link is down")
            # linkdown_flag = True # Define and use if needed
            heartbeat_ip_timeout[i] = 1
    print(" heartbeat_ip_timeout", heartbeat_ip_timeout)


async def vehicle_collision_monitor_receive(sock3):
    global master_flag, master_num, pos_array, home_pos, uav_home_pos, skip_wp_flag, next_wp, vehicles
    loop = asyncio.get_running_loop()
    while True:
        data, addr = await loop.sock_recvfrom(sock3, 1024)
        decoded_index = data.decode('utf-8')
        print("Received:", decoded_index)

        if decoded_index.startswith("master"):
            m, master_num_str = decoded_index.split("-")
            master_num = int(master_num_str)
            print("Master message:", m, master_num)
            if master_num == 0:
                if not master_flag:
                    master_flag = True
                    CHECK_network_connection()
                    # Here vehicle_connection() is blocking; run in executor if needed
                    await asyncio.get_running_loop().run_in_executor(None, vehicle_connection)
                    await asyncio.get_running_loop().run_in_executor(None, fetch_location)
                    await asyncio.get_running_loop().run_in_executor(None, home_lock)
                    # s = sim.Simulation(uav_home_pos, num_bots=len(vehicles), env_name=file_name)  # Assuming synchronous
            else:
                if master_flag:
                    for vehicle in vehicles:
                        vehicle.close()
                master_flag = False
            print("master_flag", master_flag)

        elif decoded_index.startswith("pos_array"):
            array_data = decoded_index[9:]
            pos_array = json.loads(array_data)
            print("pos_array updated:", pos_array)

        elif decoded_index.startswith("home_pos"):
            home_pos_data = decoded_index[8:]
            home_pos = json.loads(home_pos_data)
            print("home_pos updated:", home_pos)

        elif decoded_index.startswith("uav_home_pos"):
            if not master_flag:
                uav_data = decoded_index[12:]
                uav_home_pos = json.loads(uav_data)
                print("uav_home_pos updated:", uav_home_pos)

        elif decoded_index.startswith("skip_wp"):
            _, next_wp_str = decoded_index.split(",")
            next_wp = int(next_wp_str)
            skip_wp_flag = True
            print("skip_wp_flag set True with next_wp =", next_wp)


def vehicle_connection():
    global vehicles, pos_array, num_bots, heartbeat_ip_timeout
    pos_array = []
    vehicles = []
    num_bots = 0

    ip = get_wifi_ip()

    for i in range(min(num_bots, len(port_array))):
        port = port_array[i]
        try:
            vehicle = connect(f'udpin:{ip}:{port}', baud=115200, heartbeat_timeout=heartbeat_ip_timeout[min(i,len(heartbeat_ip_timeout)-1)])
            print(f'Drone {i + 1} connected on port {port}')
            vehicles.append(vehicle)
            pos_array.append(vehicle._master.target_system)
            num_bots += 1
        except Exception as e:
            print(f"Vehicle {i+1} connection failed: {e}")


def fetch_location():
	global vehicles,home_pos_lat_lon,home_pos,uav_home_pos
	global robots
	uav_home_pos=[]
	current_lat_lon=[]
	if master_flag:	
		try:
			home_lock()
		except:
			for i,vehicle in enumerate(vehicles):
				lat = vehicle.location.global_relative_frame.lat
				lon = vehicle.location.global_relative_frame.lon
				home_pos_lat_lon.append((lat,lon))			    
				x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
				home_pos.append((x / 2, y / 2)) 
				if i < len(robots):
					robots[i] = (x / 2, y / 2)
				print('home_pos',home_pos)
		
		for i,vehicle in enumerate(vehicles):
		    lat = vehicle.location.global_relative_frame.lat
		    lon = vehicle.location.global_relative_frame.lon
		    current_lat_lon.append((lat,lon))			    
		    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
		    print("x,y",x/2,y/2)
		    uav_home_pos.append((x / 2, y / 2))


async def main():
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address2 = ('', 12008)  #receive from .....rx.py
    sock2.bind(server_address2)
    
    sock3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock3.bind(('', 12002))
    sock3.setblocking(False)

    # Start the UDP receive task
    task = asyncio.create_task(vehicle_collision_monitor_receive(sock3))

    ip = get_wifi_ip()
    print("ip", ip)

    # Keep running (add other async tasks or logic here)
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())
