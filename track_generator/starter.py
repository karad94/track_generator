# Copyright (C) 2022 twyleg
import os
import argparse
import sys

from track_generator import generator

VERSION = '0.0.1'


def subcommand_generate_track():
    parser = argparse.ArgumentParser(description='Generates a track')
    parser.add_argument('track_files', metavar='track_file', type=str, nargs='+',
                        help='a track file (XML) to generate the track from')
    parser.add_argument('-o', '--output', dest='output', default=os.path.join(os.getcwd(), 'output'),
                        help='Output directory for generated tracks. Default="./"')
    parser.add_argument('--png', action='store_true',
                        help='Generate output PNG (SVG only by default).')
    parser.add_argument('--gazebo', action='store_true',
                        help='Generate Gazebo model for track.')
    args = parser.parse_args(sys.argv[2:])

    generator.generate_track(args.track_files, args.output, args.png, args.gazebo)


def subcommand_generate_trajectory():
    """ TODO: Implement """
    pass


def start():
    parser = argparse.ArgumentParser(usage='track_generator <command> [<args>] <track_file>')
    parser.add_argument('command', help='track_generator commands')
    parser.add_argument('-v', '--version', help='show version and exit', action='version', version=VERSION)
    args = parser.parse_args(sys.argv[1:2])

    if args.command == 'generate_track':
        subcommand_generate_track()
    elif args.command == 'generate_trajectory':
        subcommand_generate_trajectory()


if __name__ == '__main__':
    start()
