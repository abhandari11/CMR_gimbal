"""Sample Logic for CMR planning"""
# Author: Aabhash Bhandari

import math
import os
import pathlib


d2r = math.pi / 180
r2d = 1 / d2r


class CmrPlanning():
    """Main interface for the CMR Planning window."""

    def __init__(self, parent=None):
        super().__init__(parent=parent)

    def clicked_generate_waypoints(self):
        waypoints = {}

        wp1 = [(33.21589373771255, -87.56986696619138),
               (33.19992239477393, -87.54676703331124),
               (33.218759762150036, -87.512328099724)]

        # one side of the survey line
        wp1 = self.calculate_waypoints_S(1)

        # other side of the survey line
        wp2 = self.calculate_waypoints_S(-1)

        waypoints.update({1: {'coordinates': wp2, 'color': 'red'}})
        waypoints.update({2: {'coordinates': wp1, 'color': 'blue'}})

        self.write_waypoints_to_a_file(waypoints)

    def write_waypoints_to_a_file(self, waypoints):
        for key, value in waypoints.items():
            name = "{}_waypoints.txt".format(value['color'])
            list_of_waypoints = value['coordinates']

            filename = '/' + name
            if os.path.exists(filename):
                os.remove(filename)
            with open(filename, 'w') as f:
                f.write('\n'.join(f'{tup[0]}, {tup[1]}' for tup in list_of_waypoints))


    def calculate_waypoints_frontback(self, direction):
        """
        Generate Waypoints for CMR for single survey line.

        towards and away from a single common point.

        Parameters
        ----------
        direction : int
            values: (-1 or 1)
            waypoints on the cw direction of the line = 1
            waypoints on the ccw direction of the line = -1.

        Returns
        -------
        waypoints : list
            set of waypoints for either side of the line.

        """
        waypoints = []

        # Bearing of the Grid line
        line_bearing = self.get_bearing(self.start_lat, self.start_lon, self.end_lat, self.end_lon)
        line_length = self.get_distance(self.start_lat, self.start_lon, self.end_lat, self.end_lon)
        m = self.H * math.tan(math.radians(self.theta_max))
        n = self.H * math.tan(math.radians(self.theta_min))
        k = (m - n)

        # First waypoint
        wp1 = self.get_new_coordinates(self.start_lat, self.start_lon, m, line_bearing + direction * 90)
        waypoints.append(wp1)

        # 2nd survey towards the survey line
        wp = self.get_new_coordinates(self.start_lat, self.start_lon, n, line_bearing + direction * 90)
        waypoints.append(wp)

        total_s = 25
        new_stopping_wp = wp1

        while total_s < line_length:

            # away from the survey line
            waypoints.append(new_stopping_wp)

            # towards a new point
            wp_new = self.get_new_coordinates(new_stopping_wp[0], new_stopping_wp[1], self.s, line_bearing)
            waypoints.append(wp_new)

            # towards the survey line
            wp = self.get_new_coordinates(wp_new[0], wp_new[1], k, line_bearing - direction * 90)
            waypoints.append(wp)

            new_stopping_wp = wp_new
            total_s += self.s

        return waypoints

    def calculate_waypoints_S(self, direction):
        """
        Generate Waypoints for CMR for single survey line.

        S-shaped plan.

        Parameters
        ----------
        direction : int
            values: (-1 or 1)
            waypoints on the cw direction of the line = 1
            waypoints on the ccw direction of the line = -1.

        Returns
        -------
        waypoints : list
            set of waypoints for either side of the line.

        """
        waypoints = []

        # Bearing of the Grid line
        line_bearing = self.get_bearing(self.start_lat, self.start_lon, self.end_lat, self.end_lon)
        line_length = self.get_distance(self.start_lat, self.start_lon, self.end_lat, self.end_lon)
        m = self.H * math.tan(math.radians(self.theta_max))
        n = self.H * math.tan(math.radians(self.theta_min))
        k = (m - n)

        # First waypoint
        wp1 = self.get_new_coordinates(self.start_lat, self.start_lon, m, line_bearing + direction * 90)
        waypoints.append(wp1)

        # 2nd survey towards the survey line
        wp = self.get_new_coordinates(self.start_lat, self.start_lon, n, line_bearing + direction * 90)
        waypoints.append(wp)

        # Away from the survey line
        waypoints.append(wp1)

        total_s = 50
        new_stopping_wp = wp1

        while total_s < line_length:

            # to a new point
            wp = self.get_new_coordinates(new_stopping_wp[0], new_stopping_wp[1], self.s, line_bearing)
            waypoints.append(wp)

            # towards the survey line
            wp = self.get_new_coordinates(wp[0], wp[1], k, line_bearing - direction * 90)
            waypoints.append(wp)

            # to a new point
            wp = self.get_new_coordinates(wp[0], wp[1], self.s, line_bearing)
            waypoints.append(wp)

            # away from the survey line
            wp = self.get_new_coordinates(wp[0], wp[1], k, line_bearing + direction * 90)
            waypoints.append(wp)

            new_stopping_wp = wp
            total_s += self.s * 2

        return waypoints

    def get_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculates the distances between two points.

        Parameters
        ----------
        lat1 : float
            latitude of first point.
        lon1 : float
            longitude of first point.
        lat2 : float
            latitude of end point.
        lon2 : float
            longitude of end point.

        Returns
        -------
        d: float
            Distance between two points (in meters)
        """
        R = 6378100
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        del_lat = lat2 - lat1
        del_lon = lon2 - lon1
        a = (math.sin(del_lat / 2)) ** 2 + math.cos(lat1) * math.cos(lat2) * (math.sin(del_lon / 2)) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = R * c
        return d

    def get_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculates the bearing of a line with start and end points.

        Parameters
        ----------
        lat1 : float
            latitude of first point.
        lon1 : float
            longitude of first point.
        lat2 : float
            latitude of end point.
        lon2 : float
            longitude of end point.

        Returns
        -------
        bearing : float
            Bearing of the line in degrees.

        """
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
        theta = math.atan2(y, x)
        bearing = (theta * r2d + 360) % 360
        return bearing

    def get_new_coordinates(self, lat1, lon1, dis, angle):
        """
        Calculates new coordinate from start coordinate, distance, and bearing.

        Parameters
        ----------
        lat1 : float
            latitude of first point.
        lon1 : float
            longitude of first point.
        dis : float
            distance to the new point (in metres).
        angle : float
            bearing to the new point (in degrees).

        Returns
        -------
        lat2 : float
            latitude of new point.
        lon2 : float
            longitude of new point.

        """
        R = 6378100
        angle = (angle + 360) % 360
        angle = math.radians(angle)
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.asin(math.sin(lat1) * math.cos(dis / R) + math.cos(lat1) * math.sin(dis / R) * math.cos(angle))
        lon2 = lon1 + math.atan2(math.sin(angle) * math.sin(dis / R) * math.cos(lat1),
                                 math.cos(dis / R) - math.sin(lat1) * math.sin(lat2))
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)
        return (lat2, lon2)


    def set_default_values(self):
        self.grid_spacing = 100
        self.start_lat = 33.21496
        self.start_lon = -87.54508
        self.end_lat = 33.21502
        self.end_lon = -87.54220
        self.H = 75
        self.s = 45
        self.theta_max = 60
        self.theta_min = 15
