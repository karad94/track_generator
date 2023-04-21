# Copyright (C) 2022 twyleg
import xml.etree.ElementTree as ET
from track_generator.track import *
from track_generator.errorhandling import ElementMissingException, AttributeMissingException


def read_track(xml_input_file_path: str) -> Track:
    tree = ET.parse(xml_input_file_path)
    root = tree.getroot()

    version = _read_root(root)
    width, height, lanes = _read_size(root)
    x, y = _read_origin(root)
    background_color, background_opacity = _read_background(root)
    segments = _read_segments(root)

    return Track(version, width, height, lanes, (x, y), background_color, background_opacity, segments)


def _read_root(root: ET.Element) -> str:
    version = root.get('version')

    if version is None:
        raise AttributeMissingException('version', root)

    return version


def _read_size(root: ET.Element) -> Tuple[float, float, float]:
    size_element = root.find('Size')

    if size_element is None:
        raise ElementMissingException('Size', root)

    width = size_element.get('width')
    height = size_element.get('height')
    lanes = size_element.get('lanes')

    if width is None:
        raise AttributeMissingException('width', size_element)

    if height is None:
        raise AttributeMissingException('height', size_element)

    if lanes is None:
        raise AttributeMissingException('lanes', size_element)

    return float(width), float(height), float(lanes)


def _read_origin(root: ET.Element) -> Tuple[float, float]:
    origin_element = root.find('Origin')

    if origin_element is None:
        raise ElementMissingException('Origin', root)

    x = origin_element.get('x')
    y = origin_element.get('y')

    if x is None:
        raise AttributeMissingException('x', origin_element)

    if y is None:
        raise AttributeMissingException('y', origin_element)

    return float(x), float(y)


def _read_background(root: ET.Element) -> Tuple[str, float]:
    background_element = root.find('Background')

    if background_element is None:
        raise ElementMissingException('Background', root)

    color = background_element.get('color')
    opacity = background_element.get('opacity')

    if color is None:
        raise AttributeMissingException('color', background_element)

    if opacity is None:
        raise AttributeMissingException('opacity', background_element)

    return color, float(opacity)


def _read_segments(root: ET.Element):
    segments_element = root.find("Segments")

    if segments_element is None:
        raise ElementMissingException('Segments', root)

    segments = []

    for segment_element in segments_element:
        if segment_element.tag == 'Start':
            segments.append(_read_start_element(segment_element))
        elif segment_element.tag == 'Straight':
            segments.append(_read_straight_element(segment_element))
        elif segment_element.tag == 'Turn':
            segments.append(_read_turn_element(segment_element))
        elif segment_element.tag == 'Crosswalk':
            segments.append(_read_crosswalk_element(segment_element))
        elif segment_element.tag == 'Intersection':
            segments.append(_read_intersection_element(segment_element))
        elif segment_element.tag == 'Gap':
            segments.append(_read_gap_element(segment_element))
        elif segment_element.tag == 'ParkingArea':
            segments.append(_read_parking_area_element(segment_element))
        elif segment_element.tag == 'TrafficIsland':
            segments.append(_read_traffic_island_element(segment_element))

    return segments


def _read_start_element(start_element: ET.Element):
    x = start_element.get('x')
    y = start_element.get('y')
    direction_angle = start_element.get('direction_angle')

    if x is None:
        raise AttributeMissingException('x', start_element)
    elif y is None:
        raise AttributeMissingException('y', start_element)
    elif direction_angle is None:
        raise AttributeMissingException('direction_angle', start_element)

    return Start(float(x), float(y), float(direction_angle))


def _read_straight_element(straight_element: ET.Element):
    length = straight_element.get('length')

    if length is None:
        raise AttributeMissingException('length', straight_element)

    return Straight(float(length))


def _read_turn_element(turn_element: ET.Element):
    direction = turn_element.get('direction')
    radius = turn_element.get('radius')
    radian = turn_element.get('radian')

    if direction is None:
        raise AttributeMissingException('direction', turn_element)
    elif radius is None:
        raise AttributeMissingException('radius', turn_element)
    elif radian is None:
        raise AttributeMissingException('radian', turn_element)

    cw = True if direction == 'right' else False

    return Arc(float(radius), float(radian), cw)


def _read_crosswalk_element(crosswalk_element: ET.Element):
    length = crosswalk_element.get('length')

    if length is None:
        raise AttributeMissingException('length', crosswalk_element)

    return Crosswalk(float(length))


def _read_intersection_element(intersection_element: ET.Element):
    length = intersection_element.get('length')
    oneway = intersection_element.get('oneway')
    stopped_lanes = intersection_element.get('stopped_lanes')

    if length is None:
        raise AttributeMissingException('length', intersection_element)
    if oneway is None:
        raise AttributeMissingException('oneway', intersection_element)
    if stopped_lanes is None and oneway != 'yes':
        raise AttributeMissingException('stopped_lanes', intersection_element)

    if oneway == 'yes' or oneway == 'Ja':
        ow = True
        stopped_lanes = 0

    else:
        ow = False

    return Intersection(float(length), ow, float(stopped_lanes))


def _read_gap_element(straight_element: ET.Element):
    length = straight_element.get('length')

    if length is None:
        raise AttributeMissingException('length', straight_element)

    return Gap(float(length))


def _read_spot_element(spot_element: ET.Element) -> ParkingArea.ParkingLot.Spot:
    type = spot_element.get('type')
    length = spot_element.get('length')
    return ParkingArea.ParkingLot.Spot(type, float(length))


def _read_parking_lot_element(parking_lot_element: ET.Element) -> ParkingArea.ParkingLot:
    start = parking_lot_element.get('start')
    depth = parking_lot_element.get('depth')
    opening_ending_angle = parking_lot_element.get('opening_ending_angle')
    spots: List[ParkingArea.ParkingLot.Spot] = []

    for spot_element in parking_lot_element:
        spot = _read_spot_element(spot_element)
        spots.append(spot)

    return ParkingArea.ParkingLot(float(start), float(depth), float(opening_ending_angle), spots)


def _read_parking_area_element(parking_area_element: ET.Element):
    length = parking_area_element.get('length')

    if length is None:
        raise AttributeMissingException('length', parking_area_element)

    right_lots: List[ParkingArea.ParkingLot] = []
    left_lots: List[ParkingArea.ParkingLot] = []

    right_lots_element = parking_area_element.find("RightLots")
    for parking_lot_element in right_lots_element:
        parking_lot = _read_parking_lot_element(parking_lot_element)
        right_lots.append(parking_lot)

    left_lots_element = parking_area_element.find("LeftLots")
    for parking_lot_element in left_lots_element:
        parking_lot = _read_parking_lot_element(parking_lot_element)
        left_lots.append(parking_lot)

    return ParkingArea(float(length), right_lots, left_lots)


def _read_traffic_island_element(traffic_island_element: ET.Element):
    island_width = traffic_island_element.get('island_width')
    crosswalk_length = traffic_island_element.get('crosswalk_length')
    curve_segment_length = traffic_island_element.get('curve_segment_length')
    curvature = traffic_island_element.get('curvature')
    left_lanes = traffic_island_element.get('left_lanes')

    if left_lanes is None:
        raise AttributeMissingException('left_lanes', traffic_island_element)

    return TrafficIsland(float(island_width), float(crosswalk_length), float(curve_segment_length), float(curvature),
                         float(left_lanes))
