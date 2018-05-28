import math
import orbit
import time

def calculate_landing_reference_frame(space_center, latitude, longitude, altitude):
    body = space_center.active_vessel.orbit.body
    create_relative = space_center.ReferenceFrame.create_relative
    return create_relative(
        create_relative(
            create_relative(
                body.reference_frame,
                body.surface_position(latitude, longitude, body.reference_frame),
                (
                    0,
                    math.sin(-longitude * 0.5 * math.pi / 180),
                    0,
                    math.cos(-longitude * 0.5 * math.pi / 180))),
            (0, 0, 0),
            (
                0,
                0,
                math.sin(latitude * 0.5 * math.pi / 180),
                math.cos(latitude * 0.5 * math.pi / 180))),
        (altitude, 0, 0))

def take_off():
    pass

def landing(space_center, landing_site, warp_altitude, center_of_mass_height, target_speed_at_warp_altitude):
    vessel = space_center.active_vessel

    # retro burn over landing site
    landing_reference_frame = calculate_landing_reference_frame(space_center, landing_site[0], landing_site[1], landing_site[2])
    landing_site_radius = landing_site[3]
    index = 0
    while True:
        print('head to retrograde over landing site, iteration {}'.format(index))
        index += 1
        orbit.target_direction_then_warp(space_center, closest_approach(space_center, landing_reference_frame), (0, -1, 0))
        distance = math.sqrt(sum(
            coordinate ** 2
            for coordinate in vessel.orbit.position_at(space_center.ut, landing_reference_frame)[1:]))
        print('distance to landing site: {}'.format(distance))
        if  distance < landing_site_radius:
            break
    flight = vessel.flight(vessel.orbit.body.reference_frame)
    orbit.burn(vessel, lambda vessel: flight.horizontal_speed, 0.1)

    # warp to warp_altitude
    body = vessel.orbit.body
    surface_offset = body.equatorial_radius + body.surface_height(flight.latitude, flight.longitude)
    z0 = surface_offset + flight.surface_altitude
    z1 = surface_offset + warp_altitude
    mu = space_center.g * body.mass
    w = 1.0 / z0 - flight.vertical_speed ** 2 / (2 * mu)
    def t(z, w):
        return (
            (math.asin(math.sqrt(w * z)) - math.sqrt(w * z * (1 - w * z)))
            / (math.sqrt(2 * mu) * w ** (3 / 2)))
    threshold_ut = space_center.ut + t(z0, w) - t(z1, w)
    print('head to retrograde')
    vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
    vessel.auto_pilot.target_direction = (0, -1, 0)
    wait_auto_pilot(vessel)
    print('warp to {} m, at {}'.format(warp_altitude, threshold_ut))
    space_center.warp_to(threshold_ut)

    # descent burn
    legs_deployed = False
    while True:
        target_speed = target_speed_at_warp_altitude * math.sqrt((flight.surface_altitude - center_of_mass_height) / warp_altitude)
        speed = -flight.vertical_speed
        print('\rspeed: {} / {}, altitude: {}'.format(speed, target_speed, flight.surface_altitude), end='', flush=True)
        vessel.control.throttle = clamp(
            0,
            (speed - target_speed) * 1,
            1)
        if flight.surface_altitude < 100 and not legs_deployed:
            legs_deployed = True
            vessel.parts.legs[0].deployed = True
        if vessel.situation == space_center.VesselSituation.landed:
            vessel.control.throttle = 0
            break
        time.sleep(0.1)

def harvest(space_center, enable_converters=(True, True, False, False)):
    vessel = space_center.active_vessel
    for resource_converter in vessel.parts.resource_converters:
        for index, enable in enumerate(enable_converters):
            if enable:
                resource_converter.start(index)
            else:
                resource_converter.stop(index)
    for resource_harvester in vessel.parts.resource_harvesters:
        resource_harvester.deployed = True
    for resource_harvester in vessel.parts.resource_harvesters:
        while resource_harvester.state != space_center.ResourceHarvesterState.deployed:
            time.sleep(0.1)
        resource_harvester.active = True
