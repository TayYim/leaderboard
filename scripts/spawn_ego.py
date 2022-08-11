import carla
import random

town = "Town12"

x="-1251.0" 
y="1609.8" 
z="328.9"

x=float(x)
y=float(y)
z=float(z)

client = carla.Client('localhost', 2000)
client.set_timeout(9999)
world = client.load_world(town)

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
vehicle_bp.set_attribute('role_name', "hero")
transform = carla.Transform(carla.Location(x, y, z))
actor = world.spawn_actor(vehicle_bp, transform)

spectator = world.get_spectator()
transform = actor.get_transform()
spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
carla.Rotation(pitch=-90)))
