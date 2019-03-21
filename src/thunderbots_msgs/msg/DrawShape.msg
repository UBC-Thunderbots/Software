# This message is for basic communication from AI to the visualizr
# regarding shape drawing

# Type of the shape
# Can be one of: "ellipse" | "rect" | "poly" | "arc" | "line"
string type

# Data associated with the shape; expect different form depending on type:
# If "ellipse": [center_x, center_y, r1, r2]
# If "rect":   [x, y, width, height]
# If "poly":   [x1, y1, x2, y2, ...]
# If "arc":    [center_x, center_y, radius, theta_start, theta_end]
# If "line":   [x1, y1, x2, y2]
float64[] data

# Fill of the shape, which is colour filled into the shape
string fill

# Stroke of the shape, which is the colour that outlines the shape
string stroke

# Weight of the stroke
uint8 stroke_weight

# Transformations
float64 transform_rotation
float64 transform_scale
