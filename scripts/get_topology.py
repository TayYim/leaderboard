import carla

client = carla.Client('localhost', 2000)

world = client.get_world()

m = world.get_map()

tps = m.get_topology()

for tp in tps:
    print("{}\n{}\n================".format(tp[0], tp[1]))
print("n of tp:{}".format(len(tps)))