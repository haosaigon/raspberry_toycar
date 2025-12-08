# Helper to map value to pulse width (µs)
def map_value(x, in_min, in_max, out_min, out_max):
    value = int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    return value
