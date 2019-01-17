# This message is for basic communication from AI to the visualizr
# regarding shape drawing

# Type of the shape
# Can be one of: "circle" | "rect" | "poly" | "arc" | "line"
string type

# Data associated with the shape; expect different form depending on type:
# If "circle": [center_x, center_y, radius]
# If "rect":   [x, y, width, height]
# If "poly":   [x1, y1, x2, y2, ...]
# If "arc":    [center_x, center_y, radius, theta_start, theta_end]
# If "line":   [x1, y1, x2, y2]
float64[] data

# Fill of the shape
# Well-formed data should follow [r, g, b, a]
uint8[] fill

# Stroke of the shape
# Well-formed data should follow [r, g, b, a]
uint8[] stroke

# Weight of the stroke
uint8 strokeWeight
