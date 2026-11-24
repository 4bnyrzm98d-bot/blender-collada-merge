# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

"""Blender UI: settings, operators and sidebar panels."""

import bpy
from bpy.props import PointerProperty

from . import operators, panels, properties

classes = (
    properties.UniteModelSettings,
    *operators.classes,
    *panels.classes,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.unite_model = PointerProperty(type=properties.UniteModelSettings)


def unregister():
    del bpy.types.Scene.unite_model
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
