# Control as an arbitrary-length vector.
float64[] u

# Include a priority between 0 and 1 for control mixing.
# Priority 1 means "apply this control only," e.g. at the edge of the TEB.
float64 priority