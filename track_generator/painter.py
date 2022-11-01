# Copyright (C) 2022 twyleg
import os
import math
import pathlib
import drawSvg as draw

from typing import Optional
from track_generator.track import Track, Start, Straight, Arc, Crosswalk, Intersection, Gap
from track_generator.coordinate_system import Point2d

DEFAULT_LINE_WIDTH = 0.020
DEFAULT_TRACK_WIDTH = 0.800

class Painter:

    def __init__(self):
        self.d: Optional[draw.Drawing] = None

    def draw_point(self, p: Point2d):
        self.d.append(draw.Circle(p.x_w,
                                  p.y_w,
                                  0.010,
                                  fill='red', stroke_width=0, stroke='black'))

        self.d.append(draw.Text(f'({int(p.x_w)},{int(p.y_w)})', 0.1, p.x_w + 0.032, p.y_w, fill='red'))

    def draw_arc_center_point(self, p: Point2d, radian_angle, radius):
        self.d.append(draw.Circle(p.x_w,
                                  p.y_w,
                                  0.010,
                                  fill='red', stroke_width=0, stroke='black'))

        self.d.append(draw.Text(f'({int(p.x_w)},{int(p.y_w)})\nr={int(radius)}\na={int(radian_angle)}°', 0.1, p.x_w + 0.032, p.y_w, fill='red'))

    def draw_start_verbose(self, segment: Start):
        self.draw_point(segment.sp)
        self.draw_point(segment.slp)
        self.draw_point(segment.srp)

    def draw_straight(self, segment: Straight):
        self.d.append(draw.Line(segment.sp.x_w, segment.sp.y_w,
                            segment.ep.x_w, segment.ep.y_w,
                            fill='#eeee00',
                            stroke='black',
                            stroke_width=DEFAULT_TRACK_WIDTH))

        self.d.append(draw.Line(segment.sp.x_w, segment.sp.y_w, segment.ep.x_w, segment.ep.y_w,
                            stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none',
                            style="stroke-miterlimit:4;stroke-dasharray:0.16,0.16;stroke-dashoffset:0"))
        # stroke-miterlimit:4;stroke-dasharray:0.08,0.16;stroke-dashoffset:0;stroke-width:0.02

        self.d.append(draw.Line(segment.slp.x_w, segment.slp.y_w, segment.elp.x_w, segment.elp.y_w,
                            stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none'))

        self.d.append(draw.Line(segment.srp.x_w, segment.srp.y_w, segment.erp.x_w, segment.erp.y_w,
                            stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none'))

    def draw_straight_verbose(self, segment: Straight):
        self.draw_point(segment.sp)
        self.draw_point(segment.slp)
        self.draw_point(segment.srp)

    def draw_arc(self, segment: Arc):
        end_angle = segment.direction_angle
        start_angle = segment.start_direction_angle

        if segment.cw:
            final_end_angle = end_angle + 90
            final_start_angle = start_angle + 90
        else:
            final_end_angle = end_angle - 90
            final_start_angle = start_angle - 90

        self.d.append(draw.Arc(segment.cp.x_w, segment.cp.y_w, math.fabs(segment.radius), final_start_angle, final_end_angle,
                          cw=segment.cw, stroke='black', stroke_width=DEFAULT_TRACK_WIDTH, fill='none'))

        self.d.append(draw.Arc(segment.cp.x_w, segment.cp.y_w, math.fabs(segment.radius) - 0.380, final_start_angle,
                          final_end_angle, cw=segment.cw, stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none'))

        self.d.append(draw.Arc(segment.cp.x_w, segment.cp.y_w, math.fabs(segment.radius) + 0.380, final_start_angle,
                          final_end_angle, cw=segment.cw, stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none'))

        self.d.append(draw.Arc(segment.cp.x_w, segment.cp.y_w, math.fabs(segment.radius), final_start_angle, final_end_angle,
                          cw=segment.cw, stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none',
                          style="stroke-miterlimit:4;stroke-dasharray:0.160,0.160;stroke-dashoffset:0"))

    def draw_arc_verbose(self, segment: Arc):
        end_angle = segment.direction_angle
        start_angle = segment.start_direction_angle

        if segment.cw:
            final_end_angle = end_angle + 90
            final_start_angle = start_angle + 90
        else:
            final_end_angle = end_angle - 90
            final_start_angle = start_angle - 90

        p = draw.Path(fill='none', stroke='blue', stroke_width=DEFAULT_LINE_WIDTH)
        p.arc(segment.cp.x_w, segment.cp.y_w, math.fabs(segment.radius), final_end_angle, final_start_angle, cw=not segment.cw)
        p.arc(segment.cp.x_w, segment.cp.y_w, 0, final_start_angle, final_end_angle, cw=segment.cw, includeL=True)
        p.Z()
        self.d.append(p)

        self.draw_arc_center_point(segment.cp, segment.radian_angle, segment.radius)
        self.draw_point(segment.sp)
        self.draw_point(segment.slp)
        self.draw_point(segment.srp)

    def draw_crosswalk(self, segment: Crosswalk):
        self.d.append(draw.Line(segment.sp.x_w, segment.sp.y_w,
                            segment.ep.x_w, segment.ep.y_w,
                            fill='#eeee00',
                            stroke='black',
                            stroke_width=DEFAULT_TRACK_WIDTH))

        self.d.append(draw.Line(segment.slp.x_w, segment.slp.y_w, segment.elp.x_w, segment.elp.y_w,
                            stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none'))

        self.d.append(draw.Line(segment.srp.x_w, segment.srp.y_w, segment.erp.x_w, segment.erp.y_w,
                            stroke='white', stroke_width=DEFAULT_LINE_WIDTH, fill='none'))

        for line_point_pair in segment.line_point_pairs:
            p0 = line_point_pair[0]
            p1 = line_point_pair[1]
            self.d.append(draw.Line(p0.x_w, p0.y_w, p1.x_w, p1.y_w,
                                    stroke='white', stroke_width=0.03, fill='none'))

    def draw_intersection(self, segment: Intersection):
        full_template_file_path = os.path.join(pathlib.Path(__file__).parent.absolute(),
                                               'segment_templates/intersection.svg')
        self.draw_template_based_segment(segment, full_template_file_path)
    
    def draw_template_based_segment(self, segment, template_file_path: str):
        self.d.append(draw.Image(segment.sp.x_w - (segment.width / 2.0), segment.sp.y_w, segment.width, segment.height,
                      template_file_path,
                      embed=True,
                      transform=f'rotate({-(segment.direction_angle - 90.0)} , {segment.sp.x_w}, {-segment.sp.y_w})')
        )

    def draw_segment(self, segment):
        if isinstance(segment, Start):
            pass
        elif isinstance(segment, Crosswalk):
            self.draw_crosswalk(segment)
        elif isinstance(segment, Straight):
            self.draw_straight(segment)
        elif isinstance(segment, Arc):
            self.draw_arc(segment)
        elif isinstance(segment, Intersection):
            self.draw_intersection(segment)
        elif isinstance(segment, Gap):
            pass
        else:
            raise RuntimeError()

    def draw_segment_verbose(self, segment):
        if isinstance(segment, Start):
            pass
        elif isinstance(segment, Straight):
            self.draw_straight_verbose(segment)
        elif isinstance(segment, Arc):
            self.draw_arc_verbose(segment)
        # elif isinstance(segment, Crosswalk):
        #     self.draw_crosswalk_verbose(segment)
        # elif isinstance(segment, Intersection):
        #     self.draw_intersection_verbose(segment)
        # elif isinstance(segment, Gap):
        #     pass
        # else:
        #     raise RuntimeError()

    def draw_track(self, track: Track):
        self.d = draw.Drawing(track.width, track.height, origin=track.origin, displayInline=False)
        self.d.setPixelScale(1000)
        self.d.append(draw.Rectangle(0, 0, track.width, track.height, fill=track.background_color,
                                     fill_opacity=track.background_opacity))
        for segment in track.segments:
            self.draw_segment(segment)

    def draw_track_verbose(self, track: Track):
        for segment in track.segments:
            self.draw_segment_verbose(segment)

    def save_svg(self, track_name: str, output_directory: str, file_name_postfix: str = ''):
        output_file_path = os.path.join(output_directory, track_name)
        self.d.saveSvg(f'{output_file_path}{file_name_postfix}.svg')

    def save_png(self, track_name: str, output_directory: str):
        output_file_path = os.path.join(output_directory, track_name)
        self.d.setPixelScale(1000)
        self.d.savePng(f'{output_file_path}.png')
