import carla

client = carla.Client('localhost', 2000)

client.set_timeout(10.0) # seconds

# print(client.get_available_maps()) # will show all maps that we can use

# world = client.load_world('Town01') # change the map name you need
world = client.get_world()

waypoints = world.get_map().generate_waypoints(20)
for w in waypoints:
    w_location = w.transform.location
    print(w_location)
    text = '({},{})'.format(round(w_location.x, 2), round(w_location.y, 2))
    world.debug.draw_string(w_location, text , draw_shadow=True,life_time=999)

spectator = world.get_spectator()
transform = waypoints[0].transform
spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
carla.Rotation(pitch=-90)))