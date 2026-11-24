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

"""Unite Model

Joins every mesh of a collection (typically one imported COLLADA house) into
a single object and repairs what double-sided SketchUp exports break:

1. Every surface is exported twice (front + back face, same vertices).
   "Merge by Distance" then deletes a random face of each pair, so the visible
   front side sometimes disappears. We remove the back face of each pair
   *before* welding, keeping the face with the front material.
2. Vertices are not shared between triangles. They are welded afterwards.
3. Some faces still point inside. Each face is tested with rays: the side
   with more free space around it is the outside. Faces the rays cannot
   decide inherit orientation from their decided neighbours.
4. Files often contain geometry that is not part of the house: map
   underlays, photo backdrops, stray parts far away and neighbour houses.
   These are found per connected part and moved to a hidden "_removed"
   object.

Every result is then tested with rays from outside and the scene is sorted
into the collections "Correct" and "To check".

The panel (3D View > Sidebar > Unite) imports .dae files with the Collada
Support importer, repairs and tests them and exports the results as glTF 2.0
(.glb).

Package layout:

    geometry/    pure bmesh steps (build, unrelated, fill, orient, rays)
    pipeline     collection -> one united object
    verify       automatic test of a result (Correct / To check)
    workflow     repair + test of one model with the panel settings
    scene        collections and layout of many models
    sorting      Correct / To check collections, export selection
    io_collada   COLLADA import (Collada Support extension)
    io_gltf      glTF 2.0 (.glb) export
    batch        folder -> GLB batch with CSV report
    ui/          settings, operators, panels
"""

import sys

# Blender re-runs this file when the add-on is enabled again or the scripts
# are reloaded; drop cached submodules so every module is loaded fresh.
for _name in [n for n in sys.modules if n.startswith(__name__ + ".")]:
    del sys.modules[_name]

from . import ui  # noqa: E402
# Public API used by the test tools (tools/batch_verify.py) and scripts.
from .io_collada import collada_importer_available, import_dae  # noqa: E402,F401
from .io_gltf import export_glb  # noqa: E402,F401
from .pipeline import unite_bmesh, unite_collection  # noqa: E402,F401
from .verify import judge, verify_bmesh  # noqa: E402,F401
from .workflow import repair_collection, test_collection  # noqa: E402,F401


def register():
    ui.register()


def unregister():
    ui.unregister()
