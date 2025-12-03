import yaml


def read_origin(filepath):
    with open(filepath) as f:
        data = yaml.safe_load(f)

    origin = data.get("origin")

    # If the origin is provided as a string "(lat, lon)"
    if isinstance(origin, str):
        origin = origin.strip("()")
        lat, lon = origin.split(",")
        origin = (float(lat), float(lon))

    return origin


# Example use
file = r"D:\nithya\copter\swarm_tasks\envs\worlds\rectangles.yaml"
print(read_origin(file))
