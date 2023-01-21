import xml.etree.ElementTree as ET
from track import Track, Start, Straight, Arc, Crosswalk, Intersection, Gap, ParkingArea, TrafficIsland


class ElementMissingException(Exception):
    def __init__(self, element_name: str, parent_element: ET.Element):
        self.element_name = element_name
        self.parent_element = parent_element

    def __str__(self):
        return f'ElementMissingException: missing element="{self.element_name}" in parent element "{self.parent_element.tag}"'


class AttributeMissingException(Exception):
    def __init__(self, attribute_name: str, element: ET.Element):
        self.attribute_name = attribute_name
        self.element = element

    def __str__(self):
        return f'AttributeMissingException: missing attribute="{self.attribute_name}" in element "{self.element.tag}""'


class ZeroLength(Exception):
    def __init__(self, attribute: str, segment_name: str):
        self.attribute = attribute
        self.segment_name = segment_name

    def __str__(self):
        return f'ZeroLengthException: {self.attribute} in Segment "{self.segment_name}" is Zero'


class ArcError(Exception):
    def __init__(self, attribute: str, segment_name: str, track: Track):
        self.attribute = attribute
        self.segment_name = segment_name
        self.track_width = track.track_width

    def __str__(self):
        return f'ArcError: "{self.attribute}" in Segment "{self.segment_name}" has to be bigger than half of track_width: {self.track_width / 2}'


class LaneError(Exception):
    def __init__(self, attribute: str, segment_name: str, track: Track):
        self.attribute = attribute
        self.segment_name = segment_name
        self.lanes = int(track.lanes)

    def __str__(self):
        return f'IntersectionError: Number of "{self.attribute}" in Segment "{self.segment_name}" must be between 0 and {self.lanes}'


class TrackCheck:
    def __init__(self):
        pass

    @staticmethod
    def check_crosswalk(segment: Crosswalk):
        if segment.length == 0:
            raise ZeroLength("length", "Crosswalk")

    @staticmethod
    def check_parking_area(segment: ParkingArea):
        if segment.length == 0:
            raise ZeroLength("length", "Parking Area")

    @staticmethod
    def check_straight(segment: Straight):
        if segment.length == 0:
            raise ZeroLength("length", "Straight")

    @staticmethod
    def check_arc(segment: Arc, track: Track):
        if segment.radian_angle == 0:
            raise ZeroLength("angle", "Turn")
        if segment.radius < track.track_width / 2:
            raise ArcError("radius", "Turn", track)

    @staticmethod
    def check_intersection(segment: Intersection, track: Track):
        if segment.length == 0:
            raise ZeroLength("length", "Intersection")
        if segment.stopped_lanes > track.lanes:
            raise LaneError("stopped_lanes", "Intersection", track)

    @staticmethod
    def check_traffic_island(segment: TrafficIsland, track: Track):
        if segment.curve_segment_length == 0:
            raise ZeroLength("length", "Straight")
        if segment.left_lanes > track.lanes:
            raise LaneError("left_lanes", "TrafficIsland", track)

    def check_track_segments(self, segment, track):
        if isinstance(segment, Start):
            pass
        elif isinstance(segment, Gap):
            pass
        elif isinstance(segment, Crosswalk):
            self.check_crosswalk(segment)
        elif isinstance(segment, ParkingArea):
            self.check_parking_area(segment)
        elif isinstance(segment, Straight):
            self.check_straight(segment)
        elif isinstance(segment, Arc):
            self.check_arc(segment, track)
        elif isinstance(segment, Intersection):
            self.check_intersection(segment, track)
        elif isinstance(segment, TrafficIsland):
            self.check_traffic_island(segment, track)
        else:
            raise RuntimeError()

    def check_track(self, track: Track):
        for segments in track.segments:
            self.check_track_segments(segments, track)
