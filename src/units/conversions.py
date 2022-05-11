def kmh_to_mps(speed_in_kmh):
    """Convert from kilometers per hour to meters per second

    Aguments:
        speed_in_kmh: a speed to convert

    Returns:
        speed_in_mps: a speed in m/s
    """
    return speed_in_kmh * 1000.0 / 3600.0

def mps_to_kmh(speed_in_mps):
    """Convert from kilometers per hour to meters per second

    Aguments:
        speed_in_mps: a speed to convert

    Returns:
        speed_in_kmh: a speed in m/s
    """
    return speed_in_mps / 1000.0 * 3600.0

def C_to_K(temperature_in_C):
    """Convert from celsius temperature to kelvin

    Arguments:
        temperature_in_C: temperature in degrees celsius

    Returns:
        temperature_in_K: temperature in Kelvin
    """
    return temperature_in_C + 273.15

def K_to_C(temperature_in_K):
    """Convert from Kelvin temperature to Celsius

    Argumenets:
        temperature_in_K: temperature in Kelvin

    Returns:
        temperature_in_C: temperature in degrees Celsius
    """
    return temperature_in_K - 273.15


