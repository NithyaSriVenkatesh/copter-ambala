import numpy as np
import simplekml
import csv
import os
from shapely.geometry import Point, Polygon, LineString, MultiLineString, GeometryCollection, box, MultiPolygon
from shapely.errors import TopologicalError
from functools import cmp_to_key
from math import atan2, degrees, radians, cos, sin
from locatePosition import geoToCart, cartToGeo # Your custom GPS <-> image coordinate conversion
import matplotlib.pyplot as plt  # Optional visualization


class PolygonSearchGrid:
    class Rtf:
        def __init__(self, angle):
            self.angle = angle
            self.w = radians(90 - self.angle)
            self.irm = np.array([
                [cos(self.w), -sin(self.w), 0.0],
                [sin(self.w),  cos(self.w), 0.0],
                [0.0,          0.0,         1.0]
            ])

    class AreaPolygon:
        def __init__(self, coordinates, initial_pos, angle, interior=[], ft=5.0):
            self.P = Polygon(coordinates, interior)
            self.ft = ft
            if 0 <= angle <= 360:
                self.rtf = PolygonSearchGrid.Rtf(angle)
            else:
                self.rtf = self.rtf_longest_edge()
            self.rP = self.rotated_polygon()
            self.origin = self.get_furthest_point(self.P.exterior.coords, initial_pos)[0]

        def rtf_longest_edge(self):
            coords = list(self.P.exterior.coords)
            max_len = 0
            best_angle = 0
            for i in range(len(coords) - 1):
                dx = coords[i+1][0] - coords[i][0]
                dy = coords[i+1][1] - coords[i][1]
                length = (dx**2 + dy**2)**0.5
                if length > max_len:
                    max_len = length
                    best_angle = degrees(atan2(dy, dx))
            return PolygonSearchGrid.Rtf(best_angle)

        def rotate_points(self, points):
            new_points = []
            for point in points:
                point_mat = np.array([[point[0]], [point[1]], [0]], dtype='float64')
                new_point = self.rtf.irm @ point_mat
                new_points.append(new_point[:-1].flatten())
            return np.array(new_points)

        def rotate_from(self, points):
            irm_inv = np.linalg.inv(self.rtf.irm)
            new_points = []
            for point in points:
                point_mat = np.array([[point[0]], [point[1]], [0]], dtype='float64')
                new_point = irm_inv @ point_mat
                new_points.append(new_point[:-1].flatten())
            return np.array(new_points)

        def rotated_polygon(self):
            tf_points = self.rotate_points(np.array(self.P.exterior.coords))
            tf_holes = [self.rotate_points(np.array(list(hole.coords))) for hole in self.P.interiors]
            return Polygon(tf_points, tf_holes)

        def generate_path_lines(self):
            minx, miny, maxx, maxy = self.rP.bounds
            width = maxx - minx
            base_line = LineString([(minx, miny), (minx, maxy)])
            lines = []
            iterations = int(width / self.ft) + 2

            # Ensure polygon validity
            polygon = self.rP.buffer(0)

            for i in range(iterations):
                try:
                    offset_line = base_line.parallel_offset(i * self.ft, 'right')
                    if offset_line.is_empty:
                        continue
                    # Intersect with polygon (with holes)
                    intersection = polygon.intersection(offset_line)
                    if intersection.is_empty:
                        continue

                    # Flatten intersection geometries and filter LineStrings
                    if isinstance(intersection, (LineString, MultiLineString)):
                        for geom in getattr(intersection, 'geoms', [intersection]):
                            # Check if line is inside polygon but outside holes
                            # Holes are part of polygon interiors, so intersection handles this
                            if polygon.contains(geom) or polygon.touches(geom):
                                lines.append(geom)
                    elif isinstance(intersection, GeometryCollection):
                        for geom in intersection.geoms:
                            if isinstance(geom, (LineString, MultiLineString)):
                                if polygon.contains(geom) or polygon.touches(geom):
                                    lines.append(geom)

                except TopologicalError:
                    continue
            return lines

        def decompose_lines(self, lines):
            results = []
            reverse = False

            def leftmost_x(line):
                if isinstance(line, LineString):
                    return min(pt[0] for pt in line.coords)
                elif isinstance(line, MultiLineString):
                    return min(min(pt[0] for pt in l.coords) for l in line.geoms)
                return float('inf')

            lines.sort(key=leftmost_x)
            flat_lines = []
            for line in lines:
                if isinstance(line, LineString):
                    flat_lines.append(line)
                elif isinstance(line, MultiLineString):
                    flat_lines.extend(list(line.geoms))

            for line in flat_lines:
                coords = list(line.coords)
                if reverse:
                    coords.reverse()
                results.extend(coords)
                reverse = not reverse

            return results

        def get_furthest_point(self, points, origin):
            origin_pt = Point(*origin)

            def compare(x, y):
                dist_x = origin_pt.distance(Point(*x))
                dist_y = origin_pt.distance(Point(*y))
                return (dist_x > dist_y) - (dist_x < dist_y)

            return sorted(points, key=cmp_to_key(compare))

        def get_full_coverage_path(self):
            origin = self.rotate_points(np.array([self.origin]))[0].tolist()
            lines = self.generate_path_lines()
            ordered_points = self.decompose_lines(lines)
            tf_result = self.rotate_from(np.array(ordered_points))
            return tf_result

    def __init__(self, polygon_latlon, origin_gps, endDistance, num_drones=1, grid_spacing=5.0,
                 rotation_angle=90, obstacles_latlon=None):
        self.num_drones = num_drones
        self.grid_spacing = grid_spacing
        self.rotation_angle = rotation_angle
        self.origin_gps = origin_gps
        self.endDistance = endDistance
        self.output_dir = self._create_output_directory()
        # os.makedirs(self.output_dir, exist_ok=True)
        self.path = []
        if obstacles_latlon is None:
            obstacles_latlon = []

        self.polygon_latlon = polygon_latlon
        self.obstacles_latlon = obstacles_latlon

        # Convert polygon and obstacles from GPS to image coords
        self.outer_poly_img = self.gps_to_image_coords(polygon_latlon)
        self.obstacles_img = [self.gps_to_image_coords(obst) for obst in obstacles_latlon]

        # Create polygon with holes
        self.full_polygon = Polygon(self.outer_poly_img, holes=self.obstacles_img)

        # Buffer polygon to fix minor gaps
        self.buffer_dist = 0.5
        self.buffered_polygon = self.full_polygon.buffer(self.buffer_dist)
        self.buffered_polygon = self.buffered_polygon.buffer(-self.buffer_dist)

        # Rotation object
        if 0 <= rotation_angle <= 360:
            self.rtf = self.Rtf(rotation_angle)
        else:
            coords = list(self.buffered_polygon.exterior.coords)
            max_len = 0
            best_angle = 0
            for i in range(len(coords) - 1):
                dx = coords[i+1][0] - coords[i][0]
                dy = coords[i+1][1] - coords[i][1]
                length = (dx**2 + dy**2)**0.5
                if length > max_len:
                    max_len = length
                    best_angle = degrees(atan2(dy, dx))
            self.rtf = self.Rtf(best_angle)

        self.rotated_polygon = self.rotate_polygon(self.buffered_polygon, self.rtf)

    def _create_output_directory(self):
        base_dir = os.path.join(os.getcwd(), 'searchgrid')
        if not os.path.exists(base_dir):
            os.makedirs(base_dir)

        for filename in os.listdir(base_dir):
            file_path = os.path.join(base_dir, filename)
            try:
                if os.path.isfile(file_path):
                    os.remove(file_path)
            except Exception as e:
                print(f"Error deleting file {file_path}: {e}")
        
        return base_dir
    
    def gps_to_image_coords(self, gps_list):
        print('gps_list',gps_list,gps_list[0])
        # If gps_list contains floats instead of pairs, raise a clear error
        if len(gps_list) == 0:
            return []
        if isinstance(gps_list[0], float) or isinstance(gps_list[0], int):
            raise ValueError("Expected a list of (lat, lon) pairs, got a flat list of floats instead.")
        return [geoToCart(self.origin_gps, self.endDistance, (lat, lon)) for lat, lon in gps_list]

    def rotate_points(self, points, rtf):
        new_points = []
        for point in points:
            point_mat = np.array([[point[0]], [point[1]], [0]], dtype='float64')
            new_point = rtf.irm @ point_mat
            new_points.append(new_point[:-1].flatten())
        return np.array(new_points)

    def rotate_from(self, points, rtf):
        irm_inv = np.linalg.inv(rtf.irm)
        new_points = []
        for point in points:
            point_mat = np.array([[point[0]], [point[1]], [0]], dtype='float64')
            new_point = irm_inv @ point_mat
            new_points.append(new_point[:-1].flatten())
        return np.array(new_points)

    def rotate_polygon(self, polygon, rtf):
        rotated_exterior = self.rotate_points(np.array(polygon.exterior.coords), rtf)
        rotated_holes = [self.rotate_points(np.array(hole.coords), rtf) for hole in polygon.interiors]
        return Polygon(rotated_exterior, rotated_holes)

    def split_polygon_equally(self, polygon, num_parts):
        minx, miny, maxx, maxy = polygon.bounds
        width = maxx - minx
        slice_width = width / num_parts
        slices = []
        for i in range(num_parts):
            slice_minx = minx + i * slice_width
            slice_maxx = slice_minx + slice_width
            slicing_rect = box(slice_minx, miny, slice_maxx, maxy)
            sliced_part = polygon.intersection(slicing_rect)
            slices.append(sliced_part)
        return slices

    def generate_paths(self):
        split_polygons = self.split_polygon_equally(self.rotated_polygon, self.num_drones)
        drone_paths = []

        for i, poly in enumerate(split_polygons):
            if poly.is_empty:
                drone_paths.append([])
                continue
            polys_to_process = [poly]
            if isinstance(poly, MultiPolygon):
                polys_to_process = poly.geoms
            combined_path = []
            for sub_poly in polys_to_process:
                holes = [list(hole.coords) for hole in sub_poly.interiors]
                initial_pos = (sub_poly.centroid.x, sub_poly.centroid.y)

                sub_area = self.AreaPolygon(
                    list(sub_poly.exterior.coords),
                    initial_pos=initial_pos,
                    angle=self.rotation_angle,
                    interior=holes,
                    ft=self.grid_spacing
                )
                path = sub_area.get_full_coverage_path()
                if path is not None:
                    combined_path.extend(path)
            drone_paths.append(combined_path)
        self.drone_paths = drone_paths
        return drone_paths

    def save_paths(self):
        if not hasattr(self, 'drone_paths'):
            raise RuntimeError("Paths not generated yet. Call generate_paths() first.")

        for i, path in enumerate(self.drone_paths):
            if path is None or len(path) == 0:
                print(f"Drone {i+1} path is empty. Skipping.")
                continue

            kml = simplekml.Kml()
            ls = kml.newlinestring(name=f"Drone {i+1} Path")
            ls.altitudemode = simplekml.AltitudeMode.clamptoground
            ls.style.linestyle.color = simplekml.Color.red
            ls.style.linestyle.width = 2

            csv_data = []
            latlon=[]
            waypoint_number = 1
            for pt in path:
                lonlat = cartToGeo(self.origin_gps, self.endDistance, [pt[0], pt[1]])
                ls.coords.addcoordinates([(lonlat[1], lonlat[0])])
                x,y = geoToCart(self.origin_gps,500000,[lonlat[0], lonlat[1]])
                csv_data.append([x/2,y/2])
                latlon.append([float(lonlat[0]),float(lonlat[1])])
                kml.newpoint(name=str(waypoint_number), coords=[(lonlat[1], lonlat[0])])
                waypoint_number += 1
        
            kml_path = os.path.join(self.output_dir, f"drone{i+1}.kml")
            csv_path = os.path.join(self.output_dir, f"d{i+1}.csv")
            self.path.append(latlon)
            print('self.path',len(self.path))
            print(self.path)
            kml.save(kml_path)
            print(f"Saved KML for drone {i+1} to {kml_path}")

            with open(csv_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # writer.writerow(['latitude', 'longitude'])
                writer.writerows(csv_data)
            print(f"Saved CSV for drone {i+1} to {csv_path}")

    def visualize(self):
        fig, ax = plt.subplots(figsize=(10, 10))

        # Plot outer polygon
        x, y = self.outer_poly_img.xy if hasattr(self.outer_poly_img, 'xy') else zip(*self.outer_poly_img)
        if isinstance(self.outer_poly_img, Polygon):
            x, y = self.outer_poly_img.exterior.xy
        else:
            x, y = zip(*self.outer_poly_img)
        ax.plot(*zip(*self.outer_poly_img), 'k-', label='Outer Polygon')

        # Plot obstacles
        for obst in self.obstacles_img:
            if isinstance(obst, Polygon):
                ox, oy = obst.exterior.xy
                ax.fill(ox, oy, 'r', alpha=0.5, label='Obstacle')
            else:
                ox, oy = zip(*obst)
                ax.fill(ox, oy, 'r', alpha=0.5)

        # Plot coverage lines
        for path in getattr(self, 'drone_paths', []):
            if path:
                px, py = zip(*path)
                ax.plot(px, py, 'b-', label='Coverage Path')

        ax.set_aspect('equal')
        plt.legend()
        plt.title("Drone Coverage Planning")
        plt.show()

# # user params
num_of_drones = 3
grid_spacing = 20
rotation_angle = 90
origin = (12.921654, 80.041917)
endDistance = 500000

polygon_latlon = [
    (12.928780, 80.045609), (12.931230, 80.046191), (12.929966, 80.049102),
    (12.929860, 80.051336), (12.930454, 80.052037), (12.928564, 80.054035),
    (12.928856, 80.055802), (12.928656, 80.056840), (12.926818, 80.056999),
    (12.925478, 80.056915), (12.927396, 80.049188), (12.927710, 80.049136),
    (12.928780, 80.045609)
]
#obstacles_latlon= [ [(12.929028, 80.046939),( 12.929715, 80.046791),( 12.929245, 80.049105),( 12.928527, 80.049221),(12.929028, 80.046939)]]
obstacles_latlon=[]
planner = PolygonSearchGrid(
    polygon_latlon=polygon_latlon,
    origin_gps=origin,
    endDistance=endDistance,
    num_drones=num_of_drones,
    grid_spacing=grid_spacing,
    rotation_angle=rotation_angle,
    obstacles_latlon=obstacles_latlon
)

planner.generate_paths()
planner.save_paths()

