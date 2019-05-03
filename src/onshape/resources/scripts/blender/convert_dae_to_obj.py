# This script does not run as a standalone python script.
# It is meant to be run in the Blender scripting environment
# using its bundled Python interpreter and imports Blender libraries.
# It can also be passed in to a headless/background Blender instance
# using `blender --background --python <script file> -- [ARGS AND OPTIONS]`
#
# Script usage: `blender --background --python convert_dae_to_obj.py -- /path/to/input_file.dae /path/to/output_file.obj
import bpy
import sys

argv = sys.argv
argv = argv[argv.index("--") + 1:]  # get all args after "--"

dae_in = argv[0]
obj_out = argv[1]

bpy.ops.wm.collada_import(filepath=dae_in)
bpy.ops.export_scene.obj(filepath=obj_out, axis_forward='X', axis_up='Z')
