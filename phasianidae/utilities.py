def clamp(minimum, value, maximum):
    if minimum > value:
        return minimum
    if value > maximum:
        return maximum
    return value
