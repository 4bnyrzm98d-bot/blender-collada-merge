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


"""Names of custom properties and bmesh layers shared by the modules."""

# objects created by the repair
RESULT_TAG = "unite_model_result"      # the united model
REMOVED_TAG = "unite_model_removed"    # hidden object with the removed parts

# bmesh face layers used inside the pipeline
TWO_SIDED_LAYER = "unite_two_sided"
FLOOR_LAYER = "unite_filled_floor"

# test result, stored on the model collection and on its result object
VERDICT_KEY = "unite_verdict"          # "CORRECT" or "TO_CHECK"
VERDICT_REASON_KEY = "unite_verdict_reason"

# collections the scene is sorted into, marked with this property
SORT_KEY = "unite_sort"


def is_generated(obj):
    """True for objects the add-on created (result or removed parts)."""
    return bool(obj.get(RESULT_TAG) or obj.get(REMOVED_TAG))
