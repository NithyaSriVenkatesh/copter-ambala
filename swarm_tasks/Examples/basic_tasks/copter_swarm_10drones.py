import sys,os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__),'../../..')))
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
from search import SearchGridGenerator
from search import PolygonSearchGrid
from groupsplitspecific import SpecificSplitMission
from multipoly_grid import PolygonAutoSplit
from multipoly_specificgrid import PolygonSpecificSplit
import socket,json,csv,threading
import locatePosition
import netifaces,math

'''
file_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
file_server_address = ('', 12003)  #receive from .....rx.py
file_sock.bind(file_server_address)

graph_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
graph_server_address = ('{}', 12009)  #receive from .....rx.py
'''

sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address2 = ('', 12008)  #receive from .....rx.py
sock2.bind(server_address2)

sock3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address3 = ('', 12002)  #receive from .....rx.py
sock3.bind(server_address3)
'''
uav1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
uav1_server_address = ('192.168.6.151', 12002)  #receive from .....rx.py
uav2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
uav2_server_address = ('192.168.6.152', 12002)  #receive from .....rx.py
uav3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
uav3_server_address = ('192.168.6.153', 12002)
#uav4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#uav4_server_address = ('192.168.6.153', 12002)
#uav5 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#uav5_server_address = ('192.168.6.153', 12002)
'''
goal_table=[]
file_name=""
master_num=-1
master_flag=False
cwd = os.getcwd()

same_height=30
different_height=[300,320,340,360,380,400,420,440,460,480]

# try:
# 	index,address=sock2.recvfrom(1024)
# 	print("data",index)
# 	decoded_index = index.decode('utf-8') 
# 	data1, height,step = decoded_index.split(",")
# 	same_alt_flag=False
# 	for h in range(10):
# 		different_height[h] = int(height) + int(step) * h
# 	print("different_height",different_height)
# 	index="data"
# except Exception as e:
# 	print("Exception", e)
# 	pass
    
sock2.setblocking(0)
		
try:
	print("waiting for master data")
	index,address=sock3.recvfrom(1024)
	print("data",index)
	decoded_index = index.decode('utf-8') 
	print("decoded_index",decoded_index)
	m,master_num=decoded_index.split("-")
	print("m,master_num",m,master_num)
	if(int(master_num)==0):
		master_flag=True
	print("master_flag",master_flag)
		
except:
	pass
'''
while True:
	try:
		data,address=file_sock.recvfrom(1024)
		print("data",data)
		decoded_index = data.decode('utf-8') 
		print("decoded_index",decoded_index)
		file_name=decoded_index
		break
			
	except:
		pass
'''
# master_num==3
# master_flag=True
file_name="hanumanthapuram"

disperse_multiple_goals=[]
start_multiple_goals=[]
return_multiple_goals=[]
goal_points=[]
agg_goal_point=[]
origin=[]
removed_uav_homepos_array=[]

if(file_name=="Medur_"):
	origin=( 13.210665, 80.099739) #(13.308039, 80.146629)#medur_vtol
	start_multiple_goals=[(4533.359571946137,  4534.582162146959),(5377.564492281329,  4421.370703097977),(5604.1003862181815,  4653.543172566769),(5337.341100049475,  4250.772625520308),(4518.998921425327,  4397.592049580196),(4301.768995360449,  4593.206644467459),(4456.608688538883,  4258.870187083201),(5303.847437392898,  4100.500955291573),(5551.116514904548,  4321.971920374709),(5244.308833824743,  3959.7844666417163),(4461.759579157777,  4109.740202033495)]
	#plan2[(4533.359571946137,  4534.582162146959),(5541.529225925758,  4398.968678463051),(5799.291002080761,  4651.218705350108),(5574.9320336883175,  4202.708479825763),(4518.998921425327,  4397.592049580196),(4301.768995360449,  4593.206644467459),(4456.608688538883,  4258.870187083201),(5568.515525467282,  4043.4876256302314),(5832.067947226436,  4301.679395717466),(5568.300107458491,  3887.6764008874325),(4461.759579157777,  4109.740202033495)]
	#plan1[(4525.754124970843,  4540.247117864114),(7042.036495192134,  4081.498042240694),(7572.850233419122,  4397.902557185096),(7054.010568720503,  3704.257212401159),(4525.749341561368,  4133.590978521016),(4043.784895040774,  4386.932320881892),(4415.993330884787,  3743.163108678769),(7008.169948105923,  3213.684247611162),(7724.160895424688,  3593.132632301485),(6991.440577817013,  2718.308185229332),(4459.746148555264,  3106.0575243964504)]
	multiple_goals=start_multiple_goals
	goal_points=start_multiple_goals
		 
if(file_name=="dce_"):
	origin=(12.921654, 80.041917)#(12.923975, 80.042167) 

if(file_name=="hanumanthapuram"):
	origin=( 12.582228, 79.865131)
	

nextwaypoint=0
'''
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
server_address = ('{}', 12005)  #receive from .....rx.py

# Bind the socket to the port
remove_bot_server_address = ('{}', 12001)  #receive from .....rx.py

sock4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
server_address4 = ('', 12011)  #receive from .....rx.py
sock.bind(server_address4)
'''

num_bots=10
vehicles=[]
port_array = [14551, 14552, 14553, 14554, 14555, 14556, 14557, 14558, 14559, 14560]
'''
port_dict = {
    5:14551,
    7:14552,
    8:14553,
    11:14554,
    14:14555,
    15:14556,
    20:14557,
    22:14558,
    24:14559,
    25:14560
}
'''
port_dict = {
    1:14551,
    2:14552,
    3:14553,
    4:14554,
    5:14555,
    6:14556,
    7:14557,
    8:14558,
    9:14559,
    10:14560
}

# Print the dictionary to verify
print(port_dict)

pos_array=[]
heartbeat_ip=['192.168.6.210']*10
# heartbeat_ip=["192.168.6.210","192.168.6.210","192.168.6.210"]
#heartbeat_ip=["192.168.6.151","192.168.6.152","192.168.6.153","192.168.6.154","192.168.6.155","192.168.6.156","192.168.6.157","192.168.6.158","192.168.6.159","192.168.6.160"]
heartbeat_ip_timeout=[30]*10
goal_path_csv_array=[]
goal_path_csv_array_flag=False
skip_wp_flag=False
next_wp=0

def get_wifi_ip():
    # Common Linux interface name prefixes for Ethernet and Wi-Fi
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


def vehicle_collision_moniter_receive():	
        global index
        global vehicles
        global master_flag,master_num,pos_array,home_pos,uav_home_pos,skip_wp_flag,next_wp
        while 1:
        	index, address = sock3.recvfrom(1024)
        	print ("msg!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", index)    
        	decoded_index=index.decode('utf-8')
        	print("decoded_index",decoded_index)           
        	if decoded_index.startswith("master"):
        		m,master_num=decoded_index.split("-")
        		print("m,master_num",m,master_num)
        		msg='Drone 0 master_num '+str(master_num)+' data received'
        		if(int(master_num)==0):
        			if(master_flag):
        				pass
        			else:
        				master_flag=True       
        				CHECK_network_connection() 			
        				vehicle_connection()        				
        				fetch_location()
        				home_lock()
        				s=sim.Simulation(uav_home_pos,num_bots=len(vehicles),env_name=file_name)
        		else:
        			if(master_flag):
        				for vehicle in vehicles:
        					vehicle.close()	
        			
        			master_flag=False
        			
        		index="data"
        		data="data"
        		msg="master_num "+str(master_num)
        		print("master_flag",master_flag)
        	
        	if decoded_index.startswith("pos_array"):        		
        		message = decoded_index[:9]  # Assuming "home_pos" is 8 characters long
        		array_data = decoded_index[9:]	
        		print("pos_array",pos_array)
        		pos_array = json.loads(array_data)
        		print("pos_array",pos_array)        		
        		index="data"
        		msg="UAV 3 connected with "+str(len(pos_array))+" vehicles"
        		print("master_flag",master_flag)
        	if decoded_index.startswith("home_pos"):        		
        		message = decoded_index[:8]  # Assuming "home_pos" is 8 characters long
        		home_pos = decoded_index[8:]	
        		print("home_pos",home_pos)
        		home_pos = json.loads(home_pos)
        		print("home_pos",home_pos)
        		
        	if decoded_index.startswith("uav_home_pos"):
        		if(master_flag):
        			pass
        		else:        			
        			message = decoded_index[:12]  # Assuming "home_pos" is 8 characters long
        			uav_home_pos = decoded_index[12:]	
        			print("uav_home_pos",uav_home_pos)
        			uav_home_pos = json.loads(uav_home_pos)
        			print("uav_home_pos",uav_home_pos)
        	if decoded_index.startswith("skip_wp"):
        		print("decoded_index",decoded_index)
        		c,next_wp=decoded_index.split(",")
        		print("c,next_wp",c,next_wp)      
        		next_wp=int(next_wp)  	
        		print("next_wp",next_wp)								
        		skip_wp_flag=True
        		print("skip_wp_flag",skip_wp_flag)        		        
        			
collision_thread = threading.Thread(target=vehicle_collision_moniter_receive)
collision_thread.daemon=True
collision_thread.start()


def home_lock():
	global vehicles, home_pos_lat_lon, home_pos
	home_pos=[]
	home_pos_lat_lon=[]
	for i, vehicle in enumerate(vehicles):
		# Wait until vehicle.home_location is valid or timeout to avoid infinite loop
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

		# Assign home only if available
		home = vehicle.home_location
		if home is None:
			# print(f"Vehicle {i} has no home_location. Skipping.")
			continue  # skip to next vehicle

		# print(f"Vehicle {i} - Latitude: {home.lat}, Longitude: {home.lon}")
		x, y = locatePosition.geoToCart(origin, endDistance, [home.lat, home.lon])
		home_pos_lat_lon.append((home.lat, home.lon))
		# print("x,y", x / 2, y / 2)
		home_pos.append((x / 2, y / 2))
	return 1

def CHECK_network_connection():
    global heartbeat_ip_timeout,heartbeat_ip
    for i,iter_follower in enumerate(heartbeat_ip_timeout):
	    response = os.system('ping -c 1 ' + heartbeat_ip[i])
	    if response==0:
	    	heartbeat_ip_timeout[i]=30
	    	pass
	    else: # Link is down.
	    	print ("link is down")
	    	linkdown_flag=True
	    	#master_ip="192.168.6.153"
	    	#slave_heal_ip[i] = 'nolink'    	    	
	    	heartbeat_ip_timeout[i]=1
    print(" heartbeat_ip_timeout",heartbeat_ip_timeout)


ip = get_wifi_ip()
print("ip",ip)

def vehicle_connection():	
	global vehicles,pos_array,num_bots,heartbeat_ip_timeout
	pos_array=[]
	vehicles=[]
	num_bots=0
	
	try:
		vehicle1= connect('udpin:{}:14551'.format(ip),baud=115200, heartbeat_timeout=heartbeat_ip_timeout[0])
		print('Drone1')
		vehicles.append(vehicle1)
		pos_array.append(vehicle1._master.target_system)
		num_bots+=1
		msg="Drone1 Connected"
	except:	
		pass
		print(	"Vehicle 1 is lost")
	try:		
		vehicle2= connect('udpin:{}:14552'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[1])
		print('Drone2')
		num_bots+=1	
		vehicles.append(vehicle2)
		pos_array.append(vehicle2._master.target_system)
		msg="Drone2 Connected"
	except:	
		pass	
		print(	"Vehicle 2 is lost")
	
	try:
		vehicle3= connect('udpin:{}:14553'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[2])
		print('Drone3')
		num_bots+=1 
		vehicles.append(vehicle3)
		pos_array.append(vehicle3._master.target_system)
		msg="Drone3 Connected"
	except:		
		pass
		print(	"Vehicle 3 is lost")
	
	try:		
		vehicle4= connect('udpin:{}:14554'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[3])
		print('Drone4')
		num_bots+=1
		vehicles.append(vehicle4)
		pos_array.append(vehicle4._master.target_system)
		msg="Drone4 Connected"
	except:		
		pass
		print(	"Vehicle 4 is lost")
	try:		
		vehicle5= connect('udpin:{}:14555'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[4])
		print('Drone5')
		num_bots+=1
		vehicles.append(vehicle5)
		pos_array.append(vehicle5._master.target_system)
		msg="Drone5 Connected"
	except:	
		pass
		print(	"Vehicle 5 is lost")
	
	try:
		vehicle6= connect('udpin:{}:14556'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[5])
		print('Drone6')
		num_bots+=1
		vehicles.append(vehicle6)
		pos_array.append(vehicle6._master.target_system)
		msg="Drone6 Connected"
	except:		
		pass	
		print(	"Vehicle 6 is lost")
		
	try:
		vehicle7= connect('udpin:{}:14557'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[6])
		print('Drone7')
		num_bots+=1
		vehicles.append(vehicle7)
		pos_array.append(vehicle7._master.target_system)
		msg="Drone7 Connected"
	except:		
		pass
		print(	"Vehicle 7 is lost")	
	
	try:
		vehicle8= connect('udpin:{}:14558'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[7])
		print('Drone8')
		num_bots+=1
		vehicles.append(vehicle8)
		pos_array.append(vehicle8._master.target_system)
		msg="Drone8 Connected"
	except:	
		pass
		print(	"Vehicle 8 is lost")
	
	try:	
		vehicle9= connect('udpin:{}:14559'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[8])
		print('Drone9')
		num_bots+=1
		vehicles.append(vehicle9)
		pos_array.append(vehicle9._master.target_system)
		msg="Drone9 Connected"
	except:
		pass
		print(	"Vehicle 9 is lost")
	
	try:	
		vehicle10= connect('udpin:{}:14560'.format(ip),baud=115200,heartbeat_timeout=heartbeat_ip_timeout[9])
		print('Drone10')
		vehicles.append(vehicle10)
		pos_array.append(vehicle10._master.target_system)
		num_bots+=1
		msg="Drone10 Connected"
	except:
		pass
		print(	"Vehicle 10 is lost")
	
	print(len(vehicles))
	'''
	serialized_data = json.dumps(pos_array)
	serialized_data="pos_array" + serialized_data
	print("serialized_data",serialized_data)
	
	for f in range(len(vehicles)):
		uav1.sendto(serialized_data.encode(), uav1_server_address)
		time.sleep(0.2)
		uav2.sendto(serialized_data.encode(), uav2_server_address)
		time.sleep(0.2)
		uav3.sendto(serialized_data.encode(), uav3_server_address)
		time.sleep(0.2)
		#uav4.sendto(serialized_data.encode(), uav4_server_address)
		#time.sleep(0.2)
		#uav5.sendto(serialized_data.encode(), #uav5_server_address)
	'''
def calculate_drones_needed(remaining_points, points_per_drone):
    """
    Calculate the number of drones required to cover the remaining points.

    Parameters:
    remaining_points (int): Number of points that need to be covered.
    points_per_drone (int): Number of points each drone can cover.

    Returns:
    int: Number of drones needed to cover the remaining points.
    """
    if remaining_points <= 0:
        return 0
    return (remaining_points + points_per_drone - 1) // points_per_drone  # Ceiling division

def allocate_drones(total_points, covered_points, total_drones):
    """
    Allocate drones to uncovered areas based on the exact number of drones needed.

    Parameters:
    total_points (int): Total number of points to cover.
    covered_points (list of int): Points covered by each drone.
    total_drones (int): Total number of drones initially available.

    Returns:
    dict: A dictionary where keys are area indices and values are the number of drones allocated to each area.
    """
    # Define the number of points each drone can cover
    points_per_drone = int(total_points/2)

    # Calculate remaining points for each area
    remaining_points_list = [total_points - c for c in covered_points]
    print("remaining_points_list",remaining_points_list)
    # Filter out areas that are already fully covered
    uncovered_areas = [(i, points) for i, points in enumerate(remaining_points_list) if points > 0]

    # Calculate the number of drones needed for each uncovered area
    drones_needed = [calculate_drones_needed(points, points_per_drone) for _, points in uncovered_areas]

    # Initialize allocation with zero drones
    allocation = {i: 0 for i, _ in uncovered_areas}
    
    # Case 1: total_drones <= uncovered areas
    if total_drones <= len(uncovered_areas):
        # Allocate 1 drone per uncovered area until drones run out
        for i, _ in uncovered_areas:
            if total_drones <= 0:
                break
            allocation[i] = 1
            total_drones -= 1
    
    # Case 2: total_drones > uncovered areas
    else:
        for idx, (area_index, _) in enumerate(uncovered_areas):
            if total_drones <= 0:
                break
            # Calculate max drones we can allocate to this area
            required_drones = min(drones_needed[idx], total_drones)
            allocation[area_index] = required_drones
            total_drones -= required_drones

    # Ensure fully covered areas are zeroed out
    all_areas = {i: 0 for i in range(len(covered_points))}
    all_areas.update(allocation)    
    return all_areas,remaining_points_list

count=0
endDistance=500000
home_pos=[]
home_pos_lat_lon=[]
uav_home_pos=[]
current_lat_lon=[]
home_flag=False
home_flag1=False
search_flag=False
home_goto_flag=False
lost_vehicle_num=0
vehicle_lost_flag=False
aggregate_flag=False
disperse_flag=False

landing_flag=False
# Iterate over the list of vehicles
robots = [(0, 0)] * 10

heartbeat=[0]*num_bots

vehicle_uav_heartbeat_flag=False

sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address1 = ('192.168.6.210', 12010)
           
all_uav_csv_grid_array=[0]*num_bots
robot_positions = [([0, 0]) for _ in range(20)]
print("origin#########",origin)

sleep_times = {
    10: 0,
    9: 0.1,
    8: 0.11,
    7: 0.11,
    6: 0.12,
    5: 0.009,
    4: 0.009,
    3: 0.01,
    2: 0.015,
    1: 0.015
}

# sleep_time_profiles = {
# 	1: { 5: 0.1, 4: 0.1, 3: 0.1, 2: 0.1, 1: 0.1},
#     5: { 5: 0.008, 4: 0.008, 3: 0.01, 2: 0.008, 1: 0.008},
#     6: { 5: 0.008, 4: 0.008, 3: 0.013, 2: 0.01, 1: 0.01},
# 	7: { 5: 0.008, 4: 0.008, 3: 0.015, 2: 0.01, 1: 0.01},
#     8: { 5: 0.008, 4: 0.008, 3: 0.018, 2: 0.01, 1: 0.01},
#     9: { 5: 0.008, 4: 0.008, 3: 0.01, 2: 0.01, 1: 0.01},
# 	10: {5: 0.009, 4: 0.009, 3: 0.015, 2: 0.019, 1: 0.02},
#     11: {5: 0.008, 4: 0.008, 3: 0.009, 2: 0.01, 1: 0.08},
#     12: {5: 0.008, 4: 0.008, 3: 0.009, 2: 0.01, 1: 0.08},
# 	13: {5: 0.008, 4: 0.008, 3: 0.009, 2: 0.01, 1: 0.08},
#     14: {5: 0.008, 4: 0.008, 3: 0.009, 2: 0.01, 1: 0.08},
#     15: {5: 0.008, 4: 0.008, 3: 0.009, 2: 0.01, 1: 0.08},
    
# }
# speed = 10

def fetch_location():
	global vehicles,home_pos_lat_lon,home_pos,uav_home_pos
	global robots
	uav_home_pos=[]
	current_lat_lon=[]
	print("#####")
	if master_flag:	
		try:
			home_lock()
		except:
			for i,vehicle in enumerate(vehicles):
				lat = vehicle.location.global_relative_frame.lat
				lon = vehicle.location.global_relative_frame.lon
				#print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
				home_pos_lat_lon.append((lat,lon))			    
				x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
				#print("x,y",x/2,y/2)
				home_pos.append((x / 2, y / 2)) 
				if i < len(robots):
					robots[i] = (x / 2, y / 2)
				print('home_pos',home_pos)
		try:
			for i,vehicle in enumerate(vehicles):
				lat = vehicle.location.global_relative_frame.lat
				lon = vehicle.location.global_relative_frame.lon
				current_lat_lon.append((lat,lon))			    
				x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
				print("x,y",x/2,y/2)
				uav_home_pos.append((x / 2, y / 2)) 	       
		except:
			pass	
if master_flag:
	CHECK_network_connection()
	vehicle_connection()
	while True:
	    all_armed = [False]*len(vehicles)  # Assume all vehicles are armed initially
	    for i,vehicle in enumerate(vehicles):	        
	        if vehicle.armed:	        
	            all_armed[i] = True  # Set the flag to False  
	    if all(all_armed):
	        fetch_location()
	        break
	    time.sleep(0.1)
	    
	
def generate_points(lat, lon, num_points, radius,circle_direction):
    # List to store generated points
    points = []

    # Generate points in circular formation
    for i in range(num_points):
        bearing_sign = 1 if circle_direction == 1 else -1
        bearing = bearing_sign*(360 / num_points * i)

        # Calculate new latitude and longitude
        lat2 = asin(sin(radians(lat)) * cos(radius / 6371000) +
                    cos(radians(lat)) * sin(radius / 6371000) * cos(radians(bearing)))
        lon2 = radians(lon) + atan2(sin(radians(bearing)) * sin(radius / 6371000) * cos(radians(lat)),
                                    cos(radius / 6371000) - sin(radians(lat)) * sin(lat2))

        # Append the new point to the list
        points.append((degrees(lat2), degrees(lon2)))

    return points
    
def read_specific_line(csv_file_path, line_number):
    goal=[]    
    with open(csv_file_path, 'rt') as file:
        reader = csv.reader(file)
        for i in range(line_number):
            next(reader)
        # Read the desired line
        line = next(reader)
        goal.append((float(line[0]),float(line[1])))
        return goal


def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checcks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    time.sleep(3)
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)

data=""
same_alt_flag=False
index=0
pop_flag_arr=[1]*num_bots
pop_flag=False
specific_bot_goal_flag=False
pop_bot_index=None
goal_bot_num=None
start_flag=False
start_return_csv_flag=False
circle_formation_flag=False
radius_of_earth = 6378100.0 # in meters
uav_home_flag=False
remove_flag=False
group_goal_flag=False
circle_formation_count=0
uav_removed=True
grid_path_array=[1]*num_bots
remove_bot_flag=False
remove_bot_index=0
search_step=1
percentage=0			
removed_uav_grid=[]
removed_grid_path_length=[]
removed_grid_path_array=[0]*len(pos_array)
removed_grid_path_array_start_val=[0]*len(pos_array)
checkall_removed_grid_path_array_start_val=[0]*len(pos_array)
removed_grid_filename=[0]*num_bots
removed_grid_path_array_flag=False
remove_bot_flag=False
remove_bot_index=[]
remove_bot_array=[]
grid_completed_bot=[-1]*num_bots
uncovered_area_filename=[]
uncovered_area_points=[]
grid_completed_bot=[-1]*num_bots
remove_bot_num_array=[]
group_split_goal_pos=[0]*num_bots
group_split_flag_array=[False]*num_bots
specific_goal_pos=[0]*num_bots
specific_bot_goal_flag_array=[False]*num_bots
specific_goal_xy_index=[0]*num_bots
group_split_flag=False
search_flag_val=0
split_flag_val=0
split_flag=False
previous_task=""
previous_task_flag=False
#Initialize Simulation and GUI 
while True:
	if(uav_home_pos!=[]):
		print("num_bots",num_bots,uav_home_pos)
		s = sim.Simulation(uav_home_pos,num_bots=len(pos_array), env_name=file_name)
		break
	else:
		pass

def home_monitor_thread():
    global home_pos, pos_array
    while True:
        try:
            # Only refresh if home_pos is missing or out of sync
            if not home_pos or len(pos_array) != len(home_pos):
                print("[Home Monitor] Updating home positions...")
                home_lock()
        except Exception as e:
            print("[Home Monitor] Error while updating home positions:", e)
        time.sleep(5)  # Check every 5 seconds (tune as needed)

home_monitor_thread = threading.Thread(target=home_monitor_thread, daemon=True)
home_monitor_thread.daemon=True
home_monitor_thread.start()

		
def remove_vehicle():
	global pos_array
	global vehicle_lost_flag
	global lost_vehicle_num
	global pop_bot_index
	global same_alt_flag
	global same_height
	global different_height
	global origin,endDistance
	global vehicles
	index=pos_array[lost_vehicle_num-1]
	print(index,"index")
	print("lost_vehicle_num",lost_vehicle_num,index,pos_array)
	vehicle_lost_flag=False
	pop_flag=True
	#print ("msg", index)
	for l in range(0,len(pos_array)):
		if(int(index)==pos_array[l]):
			pop_bot_index=l
			print("pop_bot_index,l",pop_bot_index,l)
			break
	pos_array.pop(pop_bot_index)
	vehicles.pop(pop_bot_index)
	s.remove_bot(pop_bot_index)
	print(num_bots)
	print("!!!!!!!!!!!!pop_flag_arr!!!!!!!!!!!",pop_flag_arr)
	print("pop index",pop_bot_index)
	same_alt_flag=False
	uav_home_pos=[]
	for i,vehicle in enumerate(vehicles):
	    lat = vehicle.location.global_relative_frame.lat
	    lon = vehicle.location.global_relative_frame.lon	     
	    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
	    different_height[i]=different_height[i]+2
	    uav_home_pos.append((x / 2, y / 2))						
	print("uav_home_pos",uav_home_pos)    
	remove_flag=False
	uav_removed=True
	while True:
		for i, b in enumerate(s.swarm):
			cmd = potf.velocity(b.get_position(), b.sim, weights=potf.field_weights, order = 2, max_dist=5)
			cmd.exec(b)
			if master_flag:
				value=[b.x*2,b.y*2]			
				lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
				
				if same_alt_flag:
					point1 = LocationGlobalRelative(lat,lon,same_height)
				else:
					point1 = LocationGlobalRelative(lat,lon,different_height[i])
				vehicles[i].simple_goto(point1)
				alt=[0]*num_bots
				alt_count=[0]*num_bots
				for i,vehicle in enumerate(vehicles):
					print("vehicle",vehicle,num_bots)
					alt[i]=vehicle.location.global_relative_frame.alt 
					print("alt[vehicle]",alt[i])
					if different_height[i] - 1.5 <= alt[i] <= different_height[i]+1.5:
						alt_count[i]=1
						print(alt_count,"alt_count")
						if all(count==1 for count in alt_count):		
							print("Reached target altitude")
							return index

vehicles_thread=[]
while(1):
	if(master_flag):
		num_bots=len(vehicles)
	else:
		num_bots=len(pos_array)
	try:            
		data, address = sock2.recvfrom(1050)
		print ("!!msg", data)
		if(data==b"store_uav_pos"):
			if os.path.exists(csv_file_path):
    				os.remove(csv_file_path)
			
			with open(csv_file_path, mode='w', newline='') as csv_file:
				csv_writer = csv.writer(csv_file)
				csv_writer.writerow(['X', 'Y'])  # Write header
				csv_writer.writerows(home_pos)
				csv_file.close()		       
			     				
		if(data.startswith(b"takeoff")):
			decoded_index=data.decode('utf-8')
			print("decoded_index",decoded_index)
			data,takeoff_height=decoded_index.split(",")
			print("data,takeoff_height",data,takeoff_height)
			for i, vehicle in enumerate(vehicles):
			    print(i)
			    thread = threading.Thread(target=arm_and_takeoff, args=(vehicle, int(takeoff_height)))				
			    vehicles_thread.append(thread)
			    thread.start()

			for thread in vehicles_thread:
			    thread.join()
			home_pos=[]
			for i,vehicle in enumerate(vehicles):
			    lat = vehicle.location.global_relative_frame.lat
			    lon = vehicle.location.global_relative_frame.lon
			    #print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
			    home_pos_lat_lon.append((lat,lon))			    
			    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
			    #print("x,y",x/2,y/2)			    
			    home_pos.append((x / 2, y / 2))
			    if i < len(robots):
			        robots[i] = (x / 2, y / 2)
			    msg = ','.join([f"{robot[0]},{robot[1]}" for robot in robots])		       
			    			     					
		if(data.startswith(b"remove")) or (remove_flag):		
				print("bot_goal!!!!!!!!!!!!")
				decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
				f,remove_bot_num = decoded_index.split(",")
				print(f,remove_bot_num)
				print("remove_bot_num",remove_bot_num,pos_array)
				remove_bot_num_array.append(int(remove_bot_num))
										
				remove_bot_flag=True
				print ("msg", index)
				for l in range(0,len(pos_array)):	
					if(int(remove_bot_num)==pos_array[l]):
						pop_bot_index=l
						print(l)
						break
				if(pop_bot_index!=None):
				    remove_bot_index=pop_bot_index
				    print(pop_bot_index)
				    pos_array.pop(pop_bot_index)
				    remove_bot_array.append(pop_bot_index)
				    print(pos_array)
				    print("vehicles[pop_bot_index]",vehicles[pop_bot_index])
				    v = vehicles[pop_bot_index]
				    v.close()
				    print("vehicles!!!",vehicles)
				    vehicles.pop(pop_bot_index)
				    s.remove_bot(pop_bot_index)
				    print("*********",home_pos)
				    home_pos.pop(pop_bot_index)
				    print("home_pos",home_pos)
				    different_height.pop(pop_bot_index)
				    print(num_bots,"LLLLLL")
				    print("!!!!!!!!!!!!pop_flag_arr!!!!!!!!!!!",pop_flag_arr,vehicles)
				    print("pop index",pop_bot_index)				    
				    uav_home_pos=[]
				    for vehicle in vehicles:
				        lat = vehicle.location.global_relative_frame.lat
				        lon = vehicle.location.global_relative_frame.lon
				        x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
				        uav_home_pos.append((x / 2, y / 2))						
				    print("uav_home_pos",uav_home_pos)    
				    remove_flag=False
				    uav_removed=True
				    pop_bot_index=None	
				    num_bots=num_bots-1	
				    print("num_bots",num_bots)		
				else:
				    print("Not found")
				data=previous_task
				if previous_task!="":
					previous_task_flag=True
				else:
					previous_task_flag=False
				
				
		if(data.startswith(b"add")):		        
		        try:
			        decoded_index = data.decode('utf-8')
			        f,sys_id = decoded_index.split(",")	        
			        try:
			            system_id = [int(id) for id in pos_array].index(int(sys_id))
			            print("system_id",system_id)
			            if int(sys_id) in port_dict:
			                connection_str = f"udpin:{ip}:{port_dict[int(sys_id)]}"
			                print(f"Connection string for sys_id {int(sys_id)}: {connection_str}")
			            else:
			                print(f"sys_id {int(sys_id)} not found in port_dict.")

			            vehicles[system_id] = connect(connection_str,baud=115200,heartbeat_timeout=30)
			            print('vehicles[system_id]',vehicles[system_id],vehicles)
			        except:
			            print("sys id not found in pos_array")
			            if int(sys_id) in port_dict:
			                connection_str = f"udpin:{ip}:{port_dict[int(sys_id)]}"
			                print(f"Connection string for sys_id {int(sys_id)}: {connection_str}")
			            else:
			                print(f"sys_id {int(sys_id)} not found in port_dict.")
						
			            globals()[f"vehicle{sys_id}"] = connect(connection_str, baud=115200, heartbeat_timeout=30)
			            vehicle = globals()[f"vehicle{sys_id}"]
			            print(f"Created vehicle{sys_id}:", vehicle)
			            vehicles.append(vehicle)
			            num_bots=num_bots+1
			            print("num_bots",num_bots)
			            # vehicle11 = connect(connection_str,baud=115200,heartbeat_timeout=30)
			            # vehicles.append(vehicle11)
			            # print("KKK",len(vehicles),vehicles)
			            lat = vehicle.location.global_relative_frame.lat
			            lon = vehicle.location.global_relative_frame.lon
			            print("GGGG",lat,lon)
			            x,y = locatePosition.geoToCart (origin, endDistance, [float(lat),float(lon)])
			            print("x,y",x/2,y/2)
			            s.add_bot(len(pos_array),(x/2,y/2))
			            print("KKKK")
			            pos_array.append(int(sys_id))
			            home_lock()
			            print("vehicles",vehicles,home_pos)
			            print('remove_bot_num_array',remove_bot_num_array)
			            print('sys_id',sys_id,'remove_bot_num_array',remove_bot_num_array,'remove_bot_array',remove_bot_array)
							
			            if int(sys_id) in remove_bot_num_array:
			            	remove_bot_flag=False 						
			            	previous_task_flag=False
			        
		        except Exception as e:
		            pass
		            print("System array not found ",e )
		        
		
		# if data.startswith(b'specific_bot_goal'): 
		# 		index="data"
		# 		# specific_goal_pos=[0]*num_bots
		# 		# specific_bot_goal_flag_array=[False]*num_bots
		# 		try:
		# 			decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
		# 			f,uav, goal_lat,goal_lon = decoded_index.split(",")
		# 			goal_x,goal_y = locatePosition.geoToCart (origin, endDistance, [float(goal_lat),float(goal_lon)])	
		# 			goal_position=(goal_x/2,goal_y/2)									
		# 			for l in range(0,num_bots):	
		# 				if(int(uav)==pos_array[l]):
		# 					uav=l
		# 					print("uav,l",uav,l)
		# 					break
					
		# 			goal_bot_num=int(uav)
		# 			print("goal_bot_num",goal_bot_num)
		# 			specific_goal_pos[goal_bot_num]=goal_position
		# 			specific_bot_goal_flag_array[goal_bot_num]=True
		# 			print("specific_bot_goal_flag_array",specific_bot_goal_flag_array,specific_goal_pos)
		# 			while 1:
		# 				time.sleep(sleep_times.get(num_bots))
		# 				if(specific_bot_goal_flag):
		# 					specific_bot_goal_flag=False
		# 					break	
								
		# 				for i,b in enumerate(s.swarm):
		# 					current_position=[b.x,b.y]
		# 					if(specific_bot_goal_flag_array[i]):
		# 						dx=abs(specific_goal_pos[i][0]-current_position[0])
		# 						dy=abs(specific_goal_pos[i][1]-current_position[1])
		# 						if(dx<=1 and dy<=1):
		# 							specific_bot_goal_flag_array[i]=False
		# 							specific_goal_pos[i]=0
		# 							print("specific_bot_goal_flag_array",specific_bot_goal_flag_array,specific_goal_pos)
		# 						if all(flag==False for flag in specific_bot_goal_flag_array):
		# 							specific_bot_goal_flag=True
		# 							break					
		# 						else:
		# 							current_position = (b.x, b.y)
		# 							if(specific_bot_goal_flag_array[i]):
		# 							    b.set_goal(specific_goal_pos[i][0], specific_goal_pos[i][1])
		# 							    cmd = cvg.goal_area_cvg(i, b, specific_goal_pos[i])
		# 							    cmd.exec(b)
		# 						if master_flag:
		# 							current_position = (b.x*2, b.y*2)
		# 							lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
		# 							if same_alt_flag:
		# 								point1 = LocationGlobalRelative(lat,lon,same_height)
		# 							else:
		# 								point1 = LocationGlobalRelative(lat,lon,different_height[i])
		# 							vehicles[i].simple_goto(point1)
												       
		# 				if(index==b"stop"):
		# 					specific_bot_goal_flag=False
		# 					break	
		# 		except Exception as e:
		# 			print("exceptiiiooonnn",e)	
		# 			pass
					
		if data.startswith(b'specificbotgoal'): 
				index="data"
				try:
					decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
					msg_parts = decoded_index.split('_') 
					f = msg_parts[0]
					uav = msg_parts[1]
					goal_array = msg_parts[2]  # All other coordinates
					goal_latlon = json.loads(goal_array)
					goal_xy=[]
					bot_reached=[0]*num_bots
					for x in goal_latlon:
						x,y = locatePosition.geoToCart (origin, endDistance, [x[0],x[1]])
						goal_xy.append((x/2,y/2))
					print(goal_xy,"goal")
					goal = [0]*num_bots
					for l in range(0,num_bots):	
						if(int(uav)==pos_array[l]):
							uav=l
							print("uav,l",uav,l)
							specific_goal_xy_index[l]=0
							break
					
					goal_bot_num=int(uav)
					print("goal_bot_num",goal_bot_num)
					specific_goal_pos[goal_bot_num]=goal_xy
					specific_bot_goal_flag_array[goal_bot_num]=True
					print("specific_bot_goal_flag_array",specific_bot_goal_flag_array,specific_goal_pos)
					while 1:
						time.sleep(sleep_times.get(num_bots))
						if(specific_bot_goal_flag):
							specific_bot_goal_flag=False
							break	
								
						for i,b in enumerate(s.swarm):
							if(specific_bot_goal_flag_array[i]):
								goal[i] = specific_goal_pos[i][specific_goal_xy_index[i]]
								current_position=[b.x,b.y]
								if(specific_bot_goal_flag_array[i]):
									dx=abs(goal[i][0]-current_position[0])
									dy=abs(goal[i][1]-current_position[1])
									if(dx<=1 and dy<=1):
										specific_goal_xy_index[i]=specific_goal_xy_index[i]+1
										print('specific_goal_xy_index',specific_goal_xy_index)
										if specific_goal_xy_index[i] == len(specific_goal_pos[i]):
											specific_bot_goal_flag_array[i]=False
											specific_goal_pos[i]=0
											print("specific_bot_goal_flag_array",specific_bot_goal_flag_array,specific_goal_pos)
									if all(flag==False for flag in specific_bot_goal_flag_array):
										specific_bot_goal_flag=True
										break					
									else:
										current_position = (b.x, b.y)
										if(specific_bot_goal_flag_array[i]):
											b.set_goal(goal[i][0], goal[i][1])
											cmd = cvg.goal_area_cvg(i, b, goal[i])
											cmd.exec(b)
									if master_flag:
										current_position = (b.x*2, b.y*2)
										lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
										if same_alt_flag:
											point1 = LocationGlobalRelative(lat,lon,same_height)
										else:
											point1 = LocationGlobalRelative(lat,lon,different_height[i])
										vehicles[i].simple_goto(point1)
								
														
						if(index==b"stop"):
							specific_bot_goal_flag=False
							break	
				except Exception as e:
					print("exceptiiiooonnn",e)	
					pass
					
		if data.startswith(b'goal'):				
				index="data"				
				try:
					decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
					msg_parts = decoded_index.split('_')
					print('msg_parts',msg_parts,len(msg_parts))
					f = msg_parts[0]  # First coordinate pair
					goal_array = msg_parts[1]  # All other coordinates
					goal_latlon = json.loads(goal_array)
					goal_xy=[]
					bot_reached=[0]*num_bots
					for x in goal_latlon:
						x,y = locatePosition.geoToCart (origin, endDistance, [x[0],x[1]])
						goal_xy.append((x/2,y/2))
						print(goal_xy,"goal_xy")
					print(goal_xy,"goal")
					goal_xy_index=0
					if master_flag:
					    uav_home_pos=[]
					    index = "data"
					    for vehicle in vehicles:
					        lat = vehicle.location.global_relative_frame.lat
					        lon = vehicle.location.global_relative_frame.lon
					        x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
					        uav_home_pos.append((x / 2, y / 2))						
					    print("uav_home_pos",uav_home_pos)
					    '''
				        serialized_data = json.dumps(home_pos)
					    serialized_data="uav_home_pos" + serialized_data
					    for i in range(len(pos_array)):
					        uav1.sendto(serialized_data.encode(),uav1_server_address)
					        time.sleep(0.2)
					        uav2.sendto(serialized_data.encode(),uav2_server_address)
					        time.sleep(0.2)
					        uav3.sendto(serialized_data.encode(),uav3_server_address)
					        time.sleep(0.2)
					        #uav4.sendto(serialized_data.encode(), uav4_server_address)
					        #time.sleep(0.2)
					        #uav5.sendto(serialized_data.encode(), #uav5_server_address)
					    '''
					    s = sim.Simulation(uav_home_pos,num_bots=len(pos_array), env_name=file_name )
					previous_task_flag=False
					while 1:
						time.sleep(sleep_times.get(num_bots))
						# time.sleep(sleep_time_profiles.get(speed, {}).get(num_bots))
						if(group_goal_flag):
							group_goal_flag=False
							break
						goal_position=goal_xy[goal_xy_index]
						for i,b in enumerate(s.swarm):
							current_position=[b.x,b.y]
							dx=abs(goal_position[0]-current_position[0])
							dy=abs(goal_position[1]-current_position[1])
							
							if(dx<=5 and dy<=5):
								bot_reached[i]=1
								# print("GOal reached",goal_xy_index,goal_position,bot_reached,dx,dy)
								
								if any(element == 1 for element in bot_reached) and goal_xy_index!=len(goal_xy) - 1:
									print("One reached",bot_reached)
									bot_reached=[0]*num_bots
									if goal_xy_index == len(goal_xy) - 1:
										print('group_goal_flag',group_goal_flag)
										group_goal_flag = True
										break
									else:
										goal_xy_index+=1
								elif all(element == 1 for element in bot_reached) and goal_xy_index==len(goal_xy) - 1:
									bot_reached=[0]*num_bots
									if goal_xy_index == len(goal_xy) - 1:
										print('group_goal_flag',group_goal_flag)
										group_goal_flag = True
										break
									else:
										goal_xy_index+=1															   
							else:
								b.set_goal(goal_position[0], goal_position[1])
								cmd = cvg.goal_area_cvg(i, b, goal_position)
								cmd.exec(b) 	
							if master_flag:
								current_position = (b.x*2, b.y*2)	
								lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
								if same_alt_flag:
									point1 = LocationGlobalRelative(lat,lon,same_height)
								else:
									point1 = LocationGlobalRelative(lat,lon,different_height[i])
								vehicles[i].simple_goto(point1)
											       
						if(index==b"stop"):
							print("Data",data)
							group_goal_flag=False
							break	
				except Exception as e:
					print("exception",e)	
					pass
		
												
		if data.startswith(b'different'): 
				decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
				data1, height,step = decoded_index.split(",")
				same_alt_flag=False
				for h in range(num_bots):
					different_height[h] = int(height) + int(step) * h
				print("different_height",different_height)
				alt_count=[0]*num_bots	
				print("alt_count",alt_count)
				alt=[0]*num_bots
				diff_height_flag=False				
				alt_count1=0
				while True:	
					if(diff_height_flag):
						diff_height_flag=False
						break				
					for i, b in enumerate(s.swarm):
						cmd = potf.velocity(b.get_position(), b.sim, weights=potf.field_weights, order = 2, max_dist=5)
						cmd.exec(b)
						if master_flag:
							value=[b.x*2,b.y*2]						
							lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)
							alt[i]=vehicles[i].location.global_relative_frame.alt 
							if different_height[i] - 1.5 <= alt[i] <= different_height[i]+1.5:
								alt_count[i]=1								
								if all(count==1 for count in alt_count):		
									print("Reached target altitude")
									index="data"
									data="data"
									diff_height_flag=True
									break
						else:
							data="data"
							diff_height_flag=True
							break
													
					if(index==b"stop"):
						index="data"
						data="index"
						break
						
		
		if(data.startswith(b"navigate")) or (start_flag):
			print("data",data)
			if not previous_task_flag:
				decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
				f,center_lat,center_lon,num_uavs,grid_space,coverage_area = decoded_index.split(",")
				curve= NavigationGridGenerator(origin, float(center_lat), float(center_lon),int(num_uavs),int(grid_space), int(coverage_area))
				path = curve.navigate_grid()
				multiple_goals=path
				print("multiple_goals",multiple_goals)
				if master_flag:
					#time.sleep(0.1)
					start_flag=True
					uav_home_pos=[]
					index = "data"
					for vehicle in vehicles:
						lat = vehicle.location.global_relative_frame.lat
						lon = vehicle.location.global_relative_frame.lon
						x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
						uav_home_pos.append((x / 2, y / 2))						
					'''
					serialized_data = json.dumps(home_pos)
					serialized_data="uav_home_pos" + serialized_data				
					for i in range(len(pos_array)):
						uav1.sendto(serialized_data.encode(),uav1_server_address)
						time.sleep(0.2)
						uav2.sendto(serialized_data.encode(),uav2_server_address)
						time.sleep(0.2)
						uav3.sendto(serialized_data.encode(),uav3_server_address)
						time.sleep(0.2)
						#uav4.sendto(serialized_data.encode(), uav4_server_address)
						#time.sleep(0.2)
						#uav5.sendto(serialized_data.encode(), #uav5_server_address)
					'''
					s = sim.Simulation(uav_home_pos,num_bots=len(pos_array), env_name=file_name )
			previous_task_flag=False
			index="data"
						
			for b in s.swarm:				
				search_flag=False
				all_bot_reach_flag=False
				bot_array=[0]*num_bots
				ind=0
				while 1:					
					if not start_flag:
						start_flag=False
						break										
					time.sleep(sleep_times.get(num_bots))
					# time.sleep(sleep_time_profiles.get(speed, {}).get(num_bots))
						
					bot_array=[0]*num_bots
					for i,b in enumerate(s.swarm):
						current_position = [b.x,b.y]
						if(skip_wp_flag):
							with open(csv_path, 'a') as csvfile:
								print("next_wp",next_wp)
								next_wp=int(next_wp)-1
								start_return_csv_flag=True
								fieldnames = ['waypoint']
								writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
								writer.writerow({'waypoint':next_wp})
								goal_path_csv_array.append(next_wp)
								print("goal_path_csv_array",goal_path_csv_array)
								goal_path_csv_array_flag=True	
							for x,c in enumerate(s.swarm):
								goal_table[x]=next_wp
							print("goal_table",goal_table)
							skip_wp_flag=False
							ind=goal_table[i]							
						goal=multiple_goals[0][ind]
						#print("goal",goal)
						x,y = locatePosition.geoToCart (origin, endDistance, goal)
						goal=(x/2,y/2)
						cmd =cvg.goal_area_cvg(i,b,goal)
						cmd+= disp_field(b)
						cmd.exec(b)							
						dx=abs(goal[0]-current_position[0])
						dy=abs(goal[1]-current_position[1])						
						if(dx<=1 and dy<=1):
							ind+=1						
							bot_array[i]=ind							
							print("bot_array",bot_array)
							print("ind",ind)
						if all(element == 1 for n,element in enumerate(bot_array)):
					         if bot_array[n] == len(multiple_goals) - 1:
					            print('all_bot_reach_flag',all_bot_reach_flag)
					            all_bot_reach_flag = True
					            break
					         else:
					            ind+=1
					            print('ind',ind,multiple_goals[ind])

						if (all_bot_reach_flag==True):
							with open(csv_path, 'a') as csvfile:
								start_return_csv_flag=True
								fieldnames = ['waypoint']
								writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
								writer.writerow({'waypoint':multiple_goals.index(goal)})
								print("multiple_goals.index(goal)",multiple_goals.index(goal))
								goal_path_csv_array.append(multiple_goals.index(goal))
								print("goal_path_csv_array",goal_path_csv_array)
								goal_path_csv_array_flag=True	
							all_bot_reach_flag=False
							bot_array=[0]*num_bots
							if (ind==len(multiple_goals)-1):
								print("Break")
								start_flag=False
								break					
						if master_flag:
							current_position = [b.x*2,b.y*2]
							lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)
					
					if(index==b"stop"):	
						print("start_flag",start_flag,"circle_formation_flag",circle_formation_flag)
						start_flag=False
						circle_formation_flag=False
						previous_task=b'navigate'
						previous_task_flag=False
						break
		
		if(data.startswith(b"disperse")) or (disperse_flag):
			disperse_goal=[]					
			index="data"	
			print("Disperse!!!!!")
			disperse_flag=True		
			if data.startswith(b"dispersegoal"):
				decoded_index = data.decode('utf-8')
				f,disperse_latlon = decoded_index.split(",")
				disperse_latlon = json.loads(disperse_goal)
				disperse_goal_index=0
				disperse_goal=[]
				for x in disperse_latlon:
					x,y = locatePosition.geoToCart (origin, endDistance, [x[0],x[1]])
					disperse_goal.append((x/2,y/2))
					print(disperse_goal,"disperse_goal")
				print(disperse_goal,"goal")
			if master_flag:
				index="data"
				uav_home_pos=[]
				for vehicle in vehicles:
				    lat = vehicle.location.global_relative_frame.lat
				    lon = vehicle.location.global_relative_frame.lon
				    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
				    uav_home_pos.append((x / 2, y / 2))
				'''
				serialized_data = json.dumps(home_pos)
				serialized_data="uav_home_pos" + serialized_data
				
				for i in range(len(pos_array)):
					uav1.sendto(serialized_data.encode(),uav1_server_address)
					time.sleep(0.2)
					uav2.sendto(serialized_data.encode(),uav2_server_address)
					time.sleep(0.2)
					uav3.sendto(serialized_data.encode(),uav3_server_address)
					time.sleep(0.2)
					#uav4.sendto(serialized_data.encode(), uav4_server_address)
					#time.sleep(0.2)
					#uav5.sendto(serialized_data.encode(), #uav5_server_address)			
				'''
				print("file_name",file_name)
				s = sim.Simulation(uav_home_pos,num_bots=num_bots, env_name=file_name)			
				gui = viz.Gui(s)
				gui.update()
				gui.close()
			disperse_start_time = time.time()
			print("disperse_start_time",disperse_start_time)
			while True:					
				time.sleep(sleep_times.get(num_bots))
				# time.sleep(sleep_time_profiles.get(speed, {}).get(num_bots))
								
				try:
					data,address=sock2.recvfrom(1024)
					decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed					
					if(data==b"search") or search_flag:
						search_flag=True
						break
				except:
					if(vehicle_lost_flag):
						vehicle_lost_flag=True
						x=remove_vehicle()
						print(x)
					for i,b in enumerate(s.swarm):						
						print("disperse_start_time",disperse_start_time)
						if(disperse_goal==[]):
							cmd = base_control.exp_control(b)
							cmd+= disp_field(b)*15
							cmd+= base_control.exp_obstacle_avoidance(b)*30
						else:
							current_position = [b.x,b.y]
							cmd = base_control.base_control(i,b,disperse_goal[disperse_goal_index])
							cmd = disp_field(b)*2
							cmd+= base_control.obstacle_avoidance(i,b,disperse_goal[disperse_goal_index])
							dx=abs(disperse_goal[disperse_goal_index][0]-current_position[0])
							dy=abs(disperse_goal[disperse_goal_index][1]-current_position[1])
							if(dx<=1 and dy<=1):	
								new_length_arr[i]=new_length_arr[i]-1	
								disperse_bot_goal[i]=1
							
							if all(goal==1 for goal in disperse_bot_goal):
								disperse_flag=False
								search_flag=True
								print(search_flag,"search_flag")
								break		
						elapsed_time = time.time() - disperse_start_time
						print("Disperse elapsed_time",elapsed_time)						
						cmd.exec(b)
						if master_flag:
							current_position =[b.x*2,b.y*2]
							lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)							
					if(disperse_goal==[]) and (elapsed_time>10):
						disperse_flag=False
						search_flag=True
						break
					if(index==b"search") or (search_flag):
						disperse_flag=False
						search_flag=True
						break
					
					if(index==b"stop"):
						print("Disperse Stoped!!!")
						disperse_flag=False
						break
		
		if(data.startswith(b"search")) or (search_flag):
			print("data,previous_task_flag",data,previous_task_flag)
			index="data"
			if not previous_task_flag:
				decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
				print('decoded_index',decoded_index)
				
				if decoded_index.startswith("searchpolygon_"):
					obstacles_latlon=[]
					rotation_angle = 90
					f,polygon_latlon_array,num_uavs,grid_spacing = decoded_index.split("_")
					print('f,polygon_latlon_array,num_uavs,grid_spacing',f,polygon_latlon_array,num_uavs,grid_spacing)
					polygon_latlon = json.loads(polygon_latlon_array)
					print("polygon_latlon",type(polygon_latlon),polygon_latlon,origin,endDistance,num_uavs,grid_spacing,rotation_angle,obstacles_latlon)
					print("KKKKKKK")
					planner = PolygonSearchGrid(
						polygon_latlon=polygon_latlon,
						origin_gps=origin,
						endDistance=endDistance,
						num_drones=int(num_uavs),
						grid_spacing=int(grid_spacing),
						rotation_angle=rotation_angle,
						obstacles_latlon=obstacles_latlon						
					)
					print("@@@@@@@@@@")
					planner.generate_paths()
					print("$$$$$$$$$$$$")
					planner.save_paths()
					print("!!!!!!!!")
				else:
					print("search_grid")
					f,center_lat,center_lon,num_uavs,grid_space,coverage_area = decoded_index.split(",")
					print("!!!!!!!",f,center_lat,center_lon,num_uavs,grid_space,coverage_area)
					curve= SearchGridGenerator(origin, float(center_lat), float(center_lon),int(num_uavs),int(grid_space), int(coverage_area))
					val = curve.generate_grids()
					# try:
					# 	curve = SearchGridGenerator(origin, float(center_lat), float(center_lon), int(num_uavs), int(grid_space), int(coverage_area))
					# 	val = curve.generate_grids()
					# 	print("Returned val:", val)
					# except Exception as e:
					# 	print("Error occurred:", e)
										
				
				search_step=1
				all_uav_csv_grid_array=[0]*num_bots
				if master_flag:					
					uav_home_pos=[]
					for vehicle in vehicles:
						lat = vehicle.location.global_relative_frame.lat
						lon = vehicle.location.global_relative_frame.lon
						x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
						uav_home_pos.append((x / 2, y / 2))
					'''
					serialized_data = json.dumps(home_pos)
					serialized_data="uav_home_pos" + serialized_data
					
					for i in range(len(pos_array)):
						uav1.sendto(serialized_data.encode(),uav1_server_address)
						time.sleep(0.2)
						uav2.sendto(serialized_data.encode(),uav2_server_address)
						time.sleep(0.2)
						uav3.sendto(serialized_data.encode(),uav3_server_address)
						time.sleep(0.2)
						#uav4.sendto(serialized_data.encode(), uav4_server_address)
						#time.sleep(0.2)
						#uav5.sendto(serialized_data.encode(), #uav5_server_address)			
					'''			
					s = sim.Simulation(uav_home_pos,num_bots=num_bots, env_name=file_name)				
					gui = viz.Gui(s)
					gui.update()
					gui.close()
				print("Search Started")
				search_flag_val=0
				f=""
				num_lines=0
				goal_position=[]
				cwd = os.getcwd()
				print("search_flag_val",search_flag_val)
				grid_path_array=[0]*num_bots
				if search_flag_val==0:
					search_flag_val+=1
					csv_file_paths=[]
					for i in range(1,len(pos_array)+1):
							csv_file_paths.append( os.path.join(cwd,'searchgrid', f'd{i}.csv'))
					print("csv_file_paths",csv_file_paths)		
				removed_grid_path_array_index=0
				print('grid_path_array',grid_path_array)
				removed_uav_grid=[]
				removed_grid_path_length=[]
				uncovered_area_points=[]
				uncovered_area_filename=[]
				removed_grid_path_array_flag=False
				removed_grid_path_array_start_val=[0]*len(pos_array)
				checkall_removed_grid_path_array_start_val=[0]*len(pos_array)
			previous_task_flag=False
			while 1:
				time.sleep(sleep_times.get(num_bots))						
				# time.sleep(sleep_time_profiles.get(speed, {}).get(num_bots))
						
				if(vehicle_lost_flag):
					vehicle_lost_flag=True
					x=remove_vehicle()
					print(x)
				reader = csv.reader(open(csv_file_paths[0]))
				num_lines= len(list(reader))
				if(remove_bot_flag):
					print("remove_bot_flag,remove_bot_array",remove_bot_flag,remove_bot_array)
					for m in remove_bot_array:
					    removed_uav_grid.append(all_uav_csv_grid_array.pop(m))
					    removed_grid_path_length.append(grid_path_array.pop(m))
					remove_bot_array=[]
					remove_bot_flag=False
					print("removed_uav_grid,removed_grid_path_length,search_step,all_uav_csv_grid_array,grid_path_array",removed_uav_grid,removed_grid_path_length,search_step,all_uav_csv_grid_array,grid_path_array)		
				if(search_step==1):
					for i,b in enumerate(s.swarm):
						all_uav_csv_grid_array[i]=csv_file_paths[i]
					print("all_uav_csv_grid_array",all_uav_csv_grid_array)
					search_step+=1
				for i,b in enumerate(s.swarm):
					if(len(checkall_removed_grid_path_array_start_val)==len(pos_array)):
					    if all(c==1 for c in checkall_removed_grid_path_array_start_val):
						    previous_task_flag=False
						    removed_uav_grid=[]
						    removed_grid_path_length=[]
						    uncovered_area_points=[]
						    uncovered_area_filename=[]
						    removed_grid_path_array_flag=False
						    removed_grid_path_array_start_val=[0]*len(pos_array)
						    checkall_removed_grid_path_array_start_val=[0]*len(pos_array)
						    landing_flag=True
					if all(x >= int(num_lines) for x in grid_path_array) and removed_grid_path_length!=[] and not removed_grid_path_array_flag:						
						print("removed_grid_path_length",removed_grid_path_length)						
						allocation,remaining_points_list  = allocate_drones(int(num_lines), removed_grid_path_length, len(pos_array))
						print("allocation,remaining_points_list",allocation,remaining_points_list)						
						for x,v in enumerate(remaining_points_list):
						    print("x",x)
						    if(removed_grid_path_length[x]==1):
						        start_index=removed_grid_path_length[x]
						    else:
						        start_index=removed_grid_path_length[x]-1
						    print("start_index",start_index,removed_grid_path_length[x])
						    print("JJJ",allocation[x],removed_grid_path_length[x],int(num_lines))
						    if(allocation[x]==0) and removed_grid_path_length[x]!=int(num_lines):
						        uncovered_area_points.append(removed_grid_path_length[x])
						        uncovered_area_filename.append(removed_uav_grid[x])
						        print("uncovered_area_points",x,uncovered_area_points,uncovered_area_filename)
						        continue
						    elif(allocation[x]==0):
						        continue
						    add_points=math.ceil(remaining_points_list[x]/allocation[x])
						    print("add_points",math.ceil(add_points))
						    end_index=start_index+add_points+1
						    print("end_index",math.ceil(end_index))
						    for m in range(allocation[x]):
						        print('removed_grid_path_array_index',removed_grid_path_array_index)
						        if(m!=0):
						            end_index+=add_points
						        if(end_index>int(num_lines)):
						            end_index=int(num_lines)
						        removed_grid_path_array[removed_grid_path_array_index] = (start_index, end_index)
						        removed_grid_path_array_start_val[removed_grid_path_array_index] = start_index
						        removed_grid_filename[removed_grid_path_array_index]=removed_uav_grid[x]
						        print("removed_grid_path_array",removed_grid_path_array,removed_grid_path_array_start_val,removed_grid_filename)
						        start_index=end_index
						        removed_grid_path_array_index+=1
						print("removed_grid_path_array!!!!!",removed_grid_path_array,removed_grid_path_array_start_val,removed_grid_filename)
						removed_grid_path_array_flag=True
						
					if all(x >= int(num_lines) for x in grid_path_array) and not removed_grid_path_length!=[]:
						previous_task_flag=False
						removed_uav_grid=[]
						removed_grid_path_length=[]
						uncovered_area_points=[]
						uncovered_area_filename=[]
						removed_grid_path_array_flag=False
						removed_grid_path_array_start_val=[0]*len(pos_array)
						checkall_removed_grid_path_array_start_val=[0]*len(pos_array)
						
						landing_flag=True
					if(removed_grid_path_array_flag):						
						if(removed_grid_path_array_start_val[i]==0):
							checkall_removed_grid_path_array_start_val[i]=1
							continue
						if(removed_grid_path_array_start_val[i]==removed_grid_path_array[i][1]):							
							checkall_removed_grid_path_array_start_val[i]=1
							if(uncovered_area_points!=[]):  
							    print("uncovered_area_points",uncovered_area_points)
							    for u,uncovered_area_point in enumerate(uncovered_area_points):
							        removed_grid_path_array[i]=(uncovered_area_point,int(num_lines)+1)
							        print('removed_grid_path_array',removed_grid_path_array)
							        removed_grid_path_array_start_val[i]=uncovered_area_points[u]
							        removed_grid_filename[i]=uncovered_area_filename[u]
							        removed_grid_path_array[i]=(uncovered_area_points[u],int(num_lines))
							        print('removed_grid_path_array_start_val',removed_grid_path_array_start_val,removed_grid_filename)
							        checkall_removed_grid_path_array_start_val[i]=0
							        uncovered_area_points.pop(u)
							        uncovered_area_filename.pop(u)							
							else:							    
							    continue						
					if grid_path_array[i]>=int(num_lines) and not removed_grid_path_array_flag:
						continue
					if(removed_grid_path_array_flag):						
						goal_lat_lon = read_specific_line(removed_grid_filename[i], removed_grid_path_array_start_val[i])										
					else:					
						goal_lat_lon = read_specific_line(all_uav_csv_grid_array[i], grid_path_array[i])
					x,y = goal_lat_lon[0][0],goal_lat_lon[0][1]
					goal=(x,y)
					cmd =cvg.goal_area_cvg(i,b,goal)
					value=[b.x*2,b.y*2]
					current_position=[b.x,b.y]
					dx=abs(goal[0]-current_position[0])
					dy=abs(goal[1]-current_position[1])
					if(dx<=0.5 and dy<=0.5):
						if grid_path_array[i]>=int(num_lines) and not removed_grid_path_array_flag:
							continue
						if grid_path_array[i]>=int(num_lines) and removed_grid_path_array_flag:
							removed_grid_path_array_start_val[i]+=1
							print("removed_grid_path_array_start_val",removed_grid_path_array_start_val)
							
						else:
							grid_path_array[i]+=1
							print("grid_path_array",grid_path_array)				
					cmd.exec(b)
					if master_flag:
						if pop_flag_arr[i]==1:							
							lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)					
						
				s.time_elapsed += 1					
				if(index==b"stop")	:
					search_flag=False
					print("Search Stoped!!!")
					previous_task=b'search'
					previous_task_flag=False
					break
											
		if data.startswith((b"split", b"specificsplit", b"polyspecificsplit", b"polyautosplit")) or split_flag:
			index="data"
			print('previous_task_flag',previous_task_flag)
			if (data.startswith(b"specificsplit")) and not previous_task_flag:
			    try:
			        decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
			        msg_parts = decoded_index.split('_')
			        print('msg_parts',msg_parts)
			        f = msg_parts[0]  # First coordinate pair
			        print('f',f)
			        center_lat_lon_array = msg_parts[1]  # All other coordinates
			        center_lat_lon_array = json.loads(center_lat_lon_array)
			        print('center_lat_lon_array',center_lat_lon_array) 
			        uav_array = msg_parts[2]
			        uav_array = json.loads(uav_array)
			        grid_space = msg_parts[3]
			        grid_space = json.loads(grid_space)
			        print('grid_space',grid_space)
			        coverage_area=msg_parts[4]
			        coverage_area = json.loads(coverage_area)
			        print('coverage_area',coverage_area)
			        split = SpecificSplitMission(origin=origin,center_lat_lons=center_lat_lon_array, drone_array = uav_array, grid_spacing=grid_space,
                                 coverage_area=coverage_area)
			        isDone = split.GroupSplitting(
                        center_lat_lons=center_lat_lon_array,
                        drone_array=uav_array,
                        grid_spacing=grid_space,
                        coverage_area=coverage_area,
                    )
			        previous_task=b'specificsplit'
			        previous_task_flag=False
			    except Exception as e:
			        print('Exception',e)
			
			if(data.startswith(b"polyautosplit")) and not previous_task_flag:
			    decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
			    msg_parts = decoded_index.split('_')
			    print('msg_parts',msg_parts,len(msg_parts))
			    f = msg_parts[0]  # First coordinate pair
			    num_uavs=msg_parts[2]		
			    grid_space=msg_parts[3]			 
			    grid_space = json.loads(grid_space)		
			    coverage_area=msg_parts[4]		
			    coverage_area = json.loads(coverage_area)	
			    polygon_array = msg_parts[1]  # All other coordinates	
			    polygon_array = json.loads(polygon_array)			
			    split = PolygonAutoSplit(
							polygon_latlon_list = polygon_array,
							origin_gps= origin,
							endDistance=endDistance,       # as in your original code
							num_drones=int(num_uavs),
							grid_spacing=int(grid_space),
							rotation_angle=90,               # or -1 to auto-detect angle
							obstacles_latlon_list=[],
						)
			    print("split",split)
			    split.generate_paths()
			    split.save_paths()
			    previous_task=b'split'
			    previous_task_flag=False
			
			if(data.startswith(b"polyspecificsplit")) and not previous_task_flag:
			    decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
			    msg_parts = decoded_index.split('_')
			    print('msg_parts',msg_parts,len(msg_parts))
			    f = msg_parts[0]  # First coordinate pair
			    uav_array = msg_parts[2]
			    uav_array = json.loads(uav_array)			   
			    grid_space=msg_parts[3]			 
			    grid_space = json.loads(grid_space)			
			    coverage_area=msg_parts[4]		
			    coverage_area = json.loads(coverage_area)			   
			    polygon_array = msg_parts[1]  # All other coordinates			
			    polygon_array = json.loads(center_lat_lon_array)		
			    
			    num_drones = sum(len(sublist) for sublist in uav_array)
			    split = PolygonSpecificSplit(
				    polygon_latlon_list=polygon_array,
				    origin_gps=origin,
				    endDistance=endDistance,
				    num_drones=num_drones,
				    grid_spacing=grid_space,
				    rotation_angle=90,
				    obstacles_latlon_list=None,
				    drone_assignments=assignments
				)

			    paths = split.generate_paths()
			    split.save_paths()
			    previous_task=b'split'
			    previous_task_flag=False
			
			
			if(data.startswith(b"split")) and not previous_task_flag:
			    decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
			    msg_parts = decoded_index.split('_')
			    print('msg_parts',msg_parts,len(msg_parts))
			    f = msg_parts[0]  # First coordinate pair
			    num_uavs=msg_parts[2]			   
			    grid_space=msg_parts[3]			 
			    grid_space = json.loads(grid_space)			
			    coverage_area=msg_parts[4]		
			    coverage_area = json.loads(coverage_area)			   
			    center_lat_lon_array = msg_parts[1]  # All other coordinates			
			    center_lat_lon_array = json.loads(center_lat_lon_array)	
			    center_lat_lon_array = [coord[0] for coord in center_lat_lon_array]		
			    print('center_lat_lon_array',center_lat_lon_array)
			    split = AutoSplitMission(origin=origin,center_lat_lons=center_lat_lon_array, num_of_drones= int(num_uavs), grid_spacing=int(grid_space),
                             coverage_area=int(coverage_area))			
			    isDone = split.GroupSplitting(
                    center_lat_lons=center_lat_lon_array,
                    num_of_drones=int(num_uavs),
                    grid_spacing=int(grid_space),
                    coverage_area=int(coverage_area),
                )
			    previous_task=b'split'
			    previous_task_flag=False
			if not previous_task_flag:
				previous_task_flag=False
				split_flag_val=0   
				search_step=1
				all_uav_csv_grid_array=[0]*num_bots
				grid_path_array=[0]*num_bots
				if master_flag:
					index="data"
					uav_home_pos=[]
					for vehicle in vehicles:
						lat = vehicle.location.global_relative_frame.lat
						lon = vehicle.location.global_relative_frame.lon
						x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
						uav_home_pos.append((x / 2, y / 2))
					'''
					serialized_data = json.dumps(home_pos)
					serialized_data="uav_home_pos" + serialized_data				
					for i in range(len(pos_array)):
						uav1.sendto(serialized_data.encode(),uav1_server_address)
						time.sleep(0.2)
						uav2.sendto(serialized_data.encode(),uav2_server_address)
						time.sleep(0.2)
						uav3.sendto(serialized_data.encode(),uav3_server_address)
						time.sleep(0.2)
						#uav4.sendto(serialized_data.encode(), uav4_server_address)
						#time.sleep(0.2)
						#uav5.sendto(serialized_data.encode(), #uav5_server_address)			
					'''
					s = sim.Simulation(uav_home_pos,num_bots=num_bots, env_name=file_name)				
					gui = viz.Gui(s)
					gui.update()
					gui.close()
				print("Group Splitting Started")		
				f=""
				num_lines=[0]*num_bots
				print('num_lines',num_lines)
				goal_bot_num=0
				goal_position=[]
				cwd = os.getcwd()
				grid_path_array=[0]*num_bots
				print("split_flag_val",split_flag_val)
				if split_flag_val==0 and data.startswith(b"split") or data.startswith(b"polyautosplit"):
					split_flag_val+=1
					csv_file_paths=[]
					for i in range(1,len(pos_array)+1):
						csv_file_paths.append( os.path.join(cwd, 'group_split', f'grid_{i}.csv'))
						reader = csv.reader(open(csv_file_paths[i-1]))
						num_lines[i-1]= len(list(reader))
					print("csv_file_paths",csv_file_paths,num_lines)
				if split_flag_val==0 and data.startswith(b"specificsplit") or data.startswith(b"polyspecificsplit"):
					split_flag_val+=1
					csv_file_paths=[]
					for i,e in enumerate(pos_array):
						csv_file_paths.append( os.path.join(cwd, 'group_split', f'grid_{e}.csv'))
						reader = csv.reader(open(csv_file_paths[i-1]))
						num_lines[i-1]= len(list(reader))
					print("csv_file_paths",csv_file_paths,num_lines)		
				removed_grid_path_array_index=0
				print('grid_path_array',grid_path_array)
				removed_uav_grid=[]
				removed_grid_path_length=[]
				uncovered_area_points=[]
				uncovered_area_filename=[]
				removed_grid_path_array_flag=False
				removed_grid_path_array_start_val=[0]*len(pos_array)
				checkall_removed_grid_path_array_start_val=[0]*len(pos_array)
			previous_task_flag=False
			while 1:
				time.sleep(sleep_times.get(num_bots))
				# time.sleep(sleep_time_profiles.get(speed, {}).get(num_bots))
						
				if(vehicle_lost_flag):
					vehicle_lost_flag=True
					x=remove_vehicle()
					print(x)
				if(remove_bot_flag):
					print("remove_bot_flag",remove_bot_flag,remove_bot_array)
					for m in remove_bot_array:
					    print("LLLLLL",remove_bot_array,m)
					    removed_uav_grid.append(all_uav_csv_grid_array.pop(m))
					    removed_grid_path_length.append(grid_path_array.pop(m))
					    num_lines.pop(m)
					remove_bot_array=[]
					remove_bot_flag=False 
					
				if(search_step==1):
					for i,b in enumerate(s.swarm):
						all_uav_csv_grid_array[i]=csv_file_paths[i]
					print("all_uav_csv_grid_array",all_uav_csv_grid_array)
					search_step+=1
				for i,b in enumerate(s.swarm):
					if(len(checkall_removed_grid_path_array_start_val)==len(pos_array)):
					    if all(c==1 for c in checkall_removed_grid_path_array_start_val):
						    previous_task_flag=False
						    removed_uav_grid=[]
						    removed_grid_path_length=[]
						    uncovered_area_points=[]
						    uncovered_area_filename=[]
						    removed_grid_path_array_start_val=[0]*len(pos_array)
						    checkall_removed_grid_path_array_start_val=[0]*len(pos_array)
						    removed_grid_path_array_flag=False
						    landing_flag=True
					
					if all(	x >= int(num_lines[i]) for x in grid_path_array) and removed_grid_path_length!=[] and not removed_grid_path_array_flag:						
						print("removed_grid_path_length",removed_grid_path_length)						
						allocation,remaining_points_list  = allocate_drones(int(num_lines[i]), removed_grid_path_length, len(pos_array))
						print("allocation,remaining_points_list",allocation,remaining_points_list)						
						for x,v in enumerate(remaining_points_list):
						    print("x",x)
						    if(removed_grid_path_length[x]==1):
						        start_index=removed_grid_path_length[x]
						    else:
						        start_index=removed_grid_path_length[x]-1
						    print("start_index",start_index)
						    print("JJJ",allocation[x])
						    if(allocation[x]==0) and removed_grid_path_length[x]!=int(num_lines[i]):
						        uncovered_area_points.append(removed_grid_path_length[x])
						        uncovered_area_filename.append(removed_uav_grid[x])
						        print("uncovered_area_points",x,v,uncovered_area_points,uncovered_area_filename)
						        continue
						    elif(allocation[x]==0):
						        continue
						    add_points=math.ceil(remaining_points_list[x]/allocation[x])
						    print("add_points",math.ceil(add_points))
						    end_index=start_index+add_points+1
						    print("end_index",math.ceil(end_index))
						    for m in range(allocation[x]):
						        print('removed_grid_path_array_index',removed_grid_path_array_index)
						        if(m!=0):
						            end_index+=add_points
						        if(end_index>int(num_lines[i])):
						            end_index=int(num_lines[i])
						        removed_grid_path_array[removed_grid_path_array_index] = (start_index, end_index)
						        removed_grid_path_array_start_val[removed_grid_path_array_index] = start_index
						        removed_grid_filename[removed_grid_path_array_index]=removed_uav_grid[x]
						        print("removed_grid_path_array",removed_grid_path_array,removed_grid_path_array_start_val,removed_grid_filename)
						        start_index=end_index
						        removed_grid_path_array_index+=1
						print("removed_grid_path_array!!!!!",removed_grid_path_array,removed_grid_path_array_start_val,removed_grid_filename)
						removed_grid_path_array_flag=True
						
					if all(x >= int(num_lines[i]) for x in grid_path_array) and not removed_grid_path_length!=[]:						
						previous_task_flag=False
						removed_uav_grid=[]
						removed_grid_path_length=[]
						uncovered_area_points=[]
						uncovered_area_filename=[]
						removed_grid_path_array_flag=False
						removed_grid_path_array_start_val=[0]*len(pos_array)
						checkall_removed_grid_path_array_start_val=[0]*len(pos_array)
						landing_flag=True
					if(removed_grid_path_array_flag):						
						if(removed_grid_path_array_start_val[i]==0):
							checkall_removed_grid_path_array_start_val[i]=1
							#print("checkall_removed_grid_path_array_start_val",checkall_removed_grid_path_array_start_val)
							continue
						if(removed_grid_path_array_start_val[i]==removed_grid_path_array[i][1]):							
							checkall_removed_grid_path_array_start_val[i]=1
							if(uncovered_area_points!=[]):  
							    print("uncovered_area_points",uncovered_area_points)
							    for u,uncovered_area_point in enumerate(uncovered_area_points):
							        removed_grid_path_array[i]=(uncovered_area_point,int(num_lines[i])+1)
							        print('removed_grid_path_array',removed_grid_path_array)
							        removed_grid_path_array_start_val[i]=uncovered_area_points[u]
							        removed_grid_filename[i]=uncovered_area_filename[u]
							        removed_grid_path_array[i]=(uncovered_area_points[u],int(num_lines[i]))
							        print('removed_grid_path_array_start_val',removed_grid_path_array_start_val,removed_grid_filename)
							        checkall_removed_grid_path_array_start_val[i]=0
							        uncovered_area_points.pop(u)
							        uncovered_area_filename.pop(u)							
							else:							    
							    continue						
					if grid_path_array[i]>=int(num_lines[i]) and not removed_grid_path_array_flag:
						continue						
					if(removed_grid_path_array_flag) and grid_path_array[i]<int(num_lines[i]) :						
						goal_lat_lon = read_specific_line(all_uav_csv_grid_array[i], grid_path_array[i])						
					
					elif(removed_grid_path_array_flag) and grid_path_array[i]>=int(num_lines[i]) :						
						goal_lat_lon = read_specific_line(removed_grid_filename[i], removed_grid_path_array_start_val[i])						
					
					else:					
						goal_lat_lon = read_specific_line(all_uav_csv_grid_array[i], grid_path_array[i])
					x,y = goal_lat_lon[0][0],goal_lat_lon[0][1]
					goal=(x,y)
					cmd =cvg.goal_area_cvg(i,b,goal)
					value=[b.x*2,b.y*2]
					current_position=[b.x,b.y]
					dx=abs(goal[0]-current_position[0])
					dy=abs(goal[1]-current_position[1])
					if(dx<=1 and dy<=1):						
						if grid_path_array[i]>=int(num_lines[i]) and not removed_grid_path_array_flag:
							continue
						if grid_path_array[i]<int(num_lines[i]) and removed_grid_path_array_flag:
							grid_path_array[i]+=1
							print("grid_path_array!!!!!",grid_path_array,num_lines)						
						
						elif grid_path_array[i]>=int(num_lines[i]) and removed_grid_path_array_flag:
							removed_grid_path_array_start_val[i]+=1
							print("removed_grid_path_array_start_val",removed_grid_path_array_start_val,num_lines)						
						
						else:
							grid_path_array[i]+=1
							print("grid_path_array",grid_path_array)				
					cmd.exec(b)											
					if master_flag:
						if pop_flag_arr[i]==1:							
							lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)
						
				s.time_elapsed += 1					
				if(index==b"stop"):
					split_flag=False
					previous_task_flag=False
					break

		if(data.startswith(b"aggregate")) or (aggregate_flag):
			decoded_index = data.decode('utf-8')  # Assuming utf-8 encoding, adjust if needed
			print("decoded_index",decoded_index)
			f,agg_lat,agg_lon = decoded_index.split(",")	
			print("agg_lat,agg_lon",agg_lat,agg_lon)	
			search_flag=False
			aggregate_flag=True
			bot_reached=[0]*len(pos_array)
			x,y = locatePosition.geoToCart (origin, endDistance, [float(agg_lat),float(agg_lon)])
			agg_goal_point=(x / 2, y / 2)
			if master_flag:
				index="data"	
				uav_home_pos=[]
				for vehicle in vehicles:
				    lat = vehicle.location.global_relative_frame.lat
				    lon = vehicle.location.global_relative_frame.lon
				    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
				    uav_home_pos.append((x / 2, y / 2))
				'''
				serialized_data = json.dumps(home_pos)
				serialized_data="uav_home_pos" + serialized_data
				sent = graph_socket.sendto(serialized_data.encode(), graph_server_address)
				for i in range(len(pos_array)):
					uav1.sendto(serialized_data.encode(),uav1_server_address)
					time.sleep(0.2)
					uav2.sendto(serialized_data.encode(),uav2_server_address)
					time.sleep(0.2)
					uav3.sendto(serialized_data.encode(),uav3_server_address)
					time.sleep(0.2)
					#uav4.sendto(serialized_data.encode(), uav4_server_address)
					#time.sleep(0.2)
					#uav5.sendto(serialized_data.encode(), #uav5_server_address)
				'''
			if not start_return_csv_flag:
				s = sim.Simulation(uav_home_pos,num_bots=len(pos_array), env_name=file_name )
			print("Aggregate!!!!")
			# if not aggregate_flag:
			# 	break
			while True:
				time.sleep(sleep_times.get(num_bots))
				if(vehicle_lost_flag):
					vehicle_lost_flag=True
					x=remove_vehicle()
					print(x)
				if(aggregate_flag):
					index = "data"
					velocity_flag=True				
					time.sleep(0.01)
					if(index==b"stop"):
						index="data"
						aggregate_flag=True					
						break
					if not aggregate_flag:
						print("aggregate_flag####")
						break				
					if(vehicle_lost_flag):
						vehicle_lost_flag=True
						x=remove_vehicle()
						print(x)
						
					for i, b in enumerate(s.swarm):
						goal_position=agg_goal_point												
						current_position=[b.x,b.y]	
						dx=abs(goal_position[0]-current_position[0])
						dy=abs(goal_position[1]-current_position[1])
						#print("dx,dy",dx,dy)
						if(dx<=1 and dy<=1):
							bot_reached[i]=1
							# print("bot_reached",bot_reached)
							if all(element==1 for element in bot_reached):
								aggregate_flag=False
								break					
						else:
							current_position = (b.x, b.y)		
							b.set_goal(goal_position[0], goal_position[1])
							cmd = cvg.goal_area_cvg(i, b, goal_position)
							cmd.exec(b)
							
						if master_flag:
							value=[b.x*2,b.y*2]					
							lat,lon = locatePosition.cartToGeo (origin, endDistance, value)
							if same_alt_flag:
								point1 = LocationGlobalRelative(lat,lon,same_height)
							else:
								point1 = LocationGlobalRelative(lat,lon,different_height[i])
							vehicles[i].simple_goto(point1)
						if(index==b"stop"):						
							index="data"
							aggregate_flag=False					
							data="index"
							break				

														
		if(data==b"home") or (home_flag):
			if master_flag:
				uav_home_pos=[]
				for vehicle in vehicles:
				    lat = vehicle.location.global_relative_frame.lat
				    lon = vehicle.location.global_relative_frame.lon

				    # Process the lat and lon as needed
				    print(f"Vehicle - Latitude: {lat}, Longitude: {lon}")
				    x,y = locatePosition.geoToCart (origin, endDistance, [lat,lon])
				    #print("x,y",x/2,y/2)
				    uav_home_pos.append((x / 2, y / 2))
				s = sim.Simulation(uav_home_pos,num_bots=len(pos_array), env_name=file_name )
				home_flag=True
								
			bot_array_home=[0]*num_bots
			all_bot_reach_flag_home=False
			print("Home.......",home_pos)
			if(home_pos==[]):
				home_lock()
			index="data"
			while 1:				
				time.sleep(sleep_times.get(num_bots))
				if(home_flag1):
					home_flag1=False
					home_goto_flag=True
					break
				if(vehicle_lost_flag):
					vehicle_lost_flag=True
					x=remove_vehicle()
					print(x)
				for i,b in enumerate(s.swarm):					
					current_position = [b.x,b.y]
					cmd =cvg.home_area_cvg(i,b,home_pos[i])
					goal=home_pos[i]
					cmd.exec(b)
					dx=abs(goal[0]-current_position[0])
					dy=abs(goal[1]-current_position[1])						
					if(dx<=0.5 and dy<=0.5):
						bot_array_home[i]=1		
					count_home=0
					#print("count_home",count_home)
					for j in range(num_bots):						
						if(bot_array_home[j]==1):
							count_home+=1
						if(count_home==num_bots):
							count_home=0
							bot_array_home=[0]*num_bots
							print("all_bot_reach_flag_home",all_bot_reach_flag_home)
							all_bot_reach_flag_home=True

					if (all_bot_reach_flag_home==True):
						with open(csv_path, 'a') as csvfile:
							fieldnames = ['waypoint']
							writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
							writer.writerow({'waypoint':-1})	
						home_flag1=True
							
					if master_flag:										
						current_position = [b.x*2,b.y*2]						
						lat,lon = locatePosition.cartToGeo (origin, endDistance, current_position)						
						if same_alt_flag:
							point1 = LocationGlobalRelative(lat,lon,same_height)
						else:
							point1 = LocationGlobalRelative(lat,lon,different_height[i])
						vehicles[i].simple_goto(point1)						
									       
				if(index==b"stop"):
					home_flag1=True
					home_flag=False
		
		if(data==b"land"):			
			print("land!!!!!!!!!!!!!!!!")
			sent = graph_socket.sendto(str("land").encode(), graph_server_address)	
			for i,b in enumerate(s.swarm):
				if(pop_flag_arr[i]==0):
					i+=1	
				vehicles[i].mode = VehicleMode("LAND")
				vehicles[i].close()		
			break
	
		if(data==b"home_goto") or (home_goto_flag):
			bot_array_home=[0]*num_bots
			all_bot_reach_flag_home=False
			s = sim.Simulation(home_pos,num_bots=num_bots, env_name=file_name )

			if(vehicle_lost_flag):
				vehicle_lost_flag=True
				x=remove_vehicle()
				print(x)
			for i,b in enumerate(s.swarm):	
				if master_flag:
					for i,b in enumerate(s.swarm):
						lat,lon = locatePosition.cartToGeo (origin, endDistance, [home_pos[i][0]*2,home_pos[i][1]*2])
						if same_alt_flag:
							point1 = LocationGlobalRelative(lat,lon,same_height)
						else:
							point1 = LocationGlobalRelative(lat,lon,different_height[i])
						vehicles[i].simple_goto(point1)
						
			if master_flag:				      							
				sent = graph_socket.sendto(str(msg).encode(), graph_server_address)
				with open(csv_path, 'w') as csvfile:
						fieldnames = ['waypoint']
						writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
						writer.writerow({'waypoint':-1})	
	
			if(index==b"stop"):	
				for i in range(3):
					sent = graph_socket.sendto(str("stop").encode(), graph_server_address)
				home_goto_flag=False
				start_flag=False
				home_flag=False					
				break	
			
	except Exception as e:
		#print("exception",e)
		if(search_flag):
			search_flag=False
		if(split_flag):
			split_flag=False
		if(start_flag):
			start_flag=False
		if(circle_formation_flag):
			circle_formation_flag=False
		if(home_flag):
			home_flag=False
		if(home_goto_flag):
			home_goto_flag=False
		if(disperse_flag):
			disperse_flag=False
		if(aggregate_flag):
			aggregate_flag=False
		pass