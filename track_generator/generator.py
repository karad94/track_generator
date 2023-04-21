# Copyright (C) 2022 twyleg
import os
from typing import List

from track_generator import xml_reader
from track_generator.painter import Painter
from track_generator.gazebo_model_generator import GazeboModelGenerator
from track_generator.errorhandling import TrackCheck


def _create_output_directory_if_required(output_directory):
    os.makedirs(output_directory, exist_ok=True)


def _get_track_name_from_file_path(file_path: str) -> str:
    filename = os.path.basename(file_path)
    filename_without_extension, _ = os.path.splitext(filename)
    return filename_without_extension


def generate_track(track_files: List[str], root_output_directory: str, generate_png=False,
                   generate_gazebo_project=False) -> List[str]:
    """
    Generate tracks (SVG, Gazebo project, etc.) from given track files (XML)
    :param track_files: List of track files
    :param root_output_directory: The output directory to write results to. Subdirectories for every track will be
    generated.
    :param generate_png: Flag whether a png image should be created for the track
    :param generate_gazebo_project:Flag whether gazebo project files should be created for the track
    :return: List of output directories for the tracks
    """
    track_output_directories: List[str] = []
    for track_file in track_files:
        # Read track definition and start of coordinate calculation
        track = xml_reader.read_track(track_file)
        track.calc(track)

        # Check for errors in track definition
        check = TrackCheck()
        check.check_track(track)

        # generate output directory
        track_name = _get_track_name_from_file_path(track_file)
        track_output_directory = os.path.join(root_output_directory, track_name)
        _create_output_directory_if_required(track_output_directory)
        track_output_directories.append(track_output_directory)

        # generate svg output and gazebo model
        painter = Painter()
        painter.draw_track(track)
        painter.save_svg(track_name, track_output_directory)
        if generate_png:
            painter.save_png(track_name, track_output_directory)

        if generate_gazebo_project:
            gazebo_model_generator = GazeboModelGenerator(track_name, track_output_directory)
            gazebo_model_generator.generate_gazebo_model(track)

            painter.save_png(track_name, gazebo_model_generator.track_materials_textures_directory)

        painter.draw_track_verbose(track)
        painter.save_svg(track_name, track_output_directory, file_name_postfix='_verbose')
    return track_output_directories


def generate_trajectory():
    print('generate_trajectory not implemented yet')
