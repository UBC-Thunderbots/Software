# Single source of truth for SSL practice field dimensions.
#
# Values flow at build time into:
#   - shared/practice_field_dims.h                          (constexpr for C++)
#   - shared/practice_field_dims.py                         (constants for Python)
#   - extlibs/er_force_sim/config/simulator/practice.txt    (sim geometry)
#
# Ratios versus Division B (9.0 x 6.0):
#   - 2/5 for field length/width, defense area, center circle
#   - 3/5 for goal width
#   - Goal depth and boundary buffer kept at Division B — tied to physical
#     goal hardware and surrounding carpet, not field size.

PRACTICE_FIELD_DIMS = {
    "field_x_length": "3.6",
    "field_y_length": "2.4",
    "defense_x_length": "0.4",
    "defense_y_length": "0.8",
    "goal_x_length": "0.18",
    "goal_y_length": "0.6",
    "boundary_buffer_size": "0.3",
    "center_circle_radius": "0.2",
}

def practice_field_substitutions():
    """Returns PRACTICE_FIELD_DIMS keyed for expand_template."""
    return {
        "@" + key.upper() + "@": value
        for key, value in PRACTICE_FIELD_DIMS.items()
    }
