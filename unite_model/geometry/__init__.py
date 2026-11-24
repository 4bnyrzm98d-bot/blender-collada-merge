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

"""Geometry steps of the repair. Pure bmesh code: no operators, no scene.

build      - meshes -> one world-space bmesh, back faces, welding, slivers
unrelated  - parts that do not belong to the house
fill       - narrow slits and missing planar faces
orient     - outward normals, hidden inner faces, two-sided faces
rays       - ray casting helpers used by orient
"""
