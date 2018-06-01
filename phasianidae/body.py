import math
import time
from . import orbit
from . import utilities

def t(z, w, mu):
    return (
        (math.asin(math.sqrt(w * z)) - math.sqrt(w * z * (1 - w * z)))
        / (math.sqrt(2 * mu) * w ** (3 / 2)))

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

def take_off(connection, target_altitude, altitude_error, pitch_profile, undershoot, precedence, precedence_hysteresis, apoapsis_k, eccentricity_k, handle_message=print, handle_stage=None):
    space_center = connection.space_center
    vessel = space_center.active_vessel
    body = vessel.orbit.body
    flight = vessel.flight(body.reference_frame)
    flight = vessel.flight(calculate_landing_reference_frame(space_center, flight.latitude, flight.longitude, 0))
    vessel.auto_pilot.target_direction = (1, 0, 0)
    vessel.auto_pilot.engage()
    vessel.control.throttle = 1
    phase = 0
    thrust = vessel.available_thrust
    eccentricity = 1
    while True:

        # manage stagging
        if not handle_stage is None:
            handle_stage(connection)

        # manage thrust and throttle
        altitude = flight.mean_altitude
        if phase == 0:
            vessel.auto_pilot.target_pitch = clamp(0, sum(coefficient * altitude ** index for index, coefficient in enumerate(pitch_profile)), 90)
            vessel.control.throttle = clamp(0, (target_altitude - undershoot - vessel.orbit.apoapsis_altitude) * apoapsis_k, 1)
            handle_message()
            if target_altitude - undershoot - vessel.orbit.apoapsis_altitude < altitude_error:
                vessel.auto_pilot.target_pitch = 0
                vessel.auto_pilot.throttle = 0
                phase = 1
        elif phase == 1:
            if vessel.orbit.time_to_apoapsis < precedence:
                eccentricity = vessel.orbit.eccentricity
                vessel.control.throttle = 1
                phase = 2
        elif phase == 2:
            if vessel.orbit.time_to_apoapsis > precedence:
                vessel.control.throttle = 0
                phase = 3
            else:
                new_eccentricity = vessel.orbit.eccentricity
                if new_eccentricity < 0.001 or new_eccentricity > eccentricity:
                    vessel.control.throttle = 0
                    break
                else:
                    eccentricity = new_eccentricity
                    vessel.control.throttle = clamp(0, eccentricity * eccentricity_k, 1)
        elif phase == 3:
            connection.space_center.warp_to(connection.space_center.ut + vessel.orbit.time_to_apoapsis - (precedence - precedence_hysteresis))
            eccentricity = vessel.orbit.eccentricity
            vessel.control.throttle = clamp(0, eccentricity * eccentricity_k, 1)
            phase = 4
        elif phase == 4:
            if vessel.orbit.time_to_apoapsis > precedence:
                vessel.control.throttle = 0
                phase = 3
            else:
                new_eccentricity = vessel.orbit.eccentricity
                if new_eccentricity < 0.001 or new_eccentricity > eccentricity:
                    vessel.control.throttle = 0
                    break
                else:
                    eccentricity = new_eccentricity
                    vessel.control.throttle = clamp(0, eccentricity * eccentricity_k, 1)
        print('phase {}, altitude: {} m, apoapsis: {} m, eccentricity: {}, pitch: {}, throttle: {}'.format(phase, altitude, vessel.orbit.apoapsis_altitude, vessel.orbit.eccentricity, vessel.auto_pilot.target_pitch, vessel.control.throttle))
        time.sleep(0.1)
    vessel.auto_pilot.disengage()

def landing(connection, landing_site, warp_altitude, stop_altitude, speed_at_warp_altitude, speed_at_stop_altitude, deploy_altitude, handle_message=print):
    space_center = connection.space_center
    vessel = space_center.active_vessel

    # retro burn over landing site
    landing_reference_frame = calculate_landing_reference_frame(space_center, landing_site[0], landing_site[1], landing_site[2])
    landing_site_radius = landing_site[3]
    index = 0
    while True:
        handle_message('head to retrograde over landing site, iteration {}'.format(index))
        index += 1
        orbit.target_direction_then_warp(connection, orbit.closest_approach(space_center, landing_reference_frame, handle_message=handle_message), (0, -1, 0))
        distance = math.sqrt(sum(
            coordinate ** 2
            for coordinate in vessel.orbit.position_at(space_center.ut, landing_reference_frame)[1:]))
        handle_message('distance to landing site: {}'.format(distance))
        if  distance < landing_site_radius:
            break
    body = vessel.orbit.body
    flight = vessel.flight(body.reference_frame)
    orbit.burn(vessel, lambda vessel: flight.horizontal_speed, 0.1, handle_message=handle_message)

    # warp to warp_altitude
    handle_message('head to retrograde')
    vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
    vessel.auto_pilot.target_direction = (0, -1, 0)
    orbit.wait_auto_pilot(connection, handle_message=handle_message)
    z0 = body.equatorial_radius + flight.mean_altitude
    z1 = body.equatorial_radius + warp_altitude + body.surface_height(flight.latitude, flight.longitude)
    mu = space_center.g * body.mass
    w = 1.0 / z0 - (flight.vertical_speed ** 2) / (2 * mu)
    ut_delta = t(z0, w, mu) - t(z1, w, mu)
    handle_message('warp to {} m, in {} s'.format(warp_altitude, ut_delta))
    space_center.warp_to(space_center.ut + ut_delta)

    # descent burn
    stop = False
    legs_deployed = False
    while True:
        altitude = flight.mean_altitude - body.surface_height(flight.latitude, flight.longitude)
        if altitude < stop_altitude:
            if not stop:
                handle_message('\nstop reached, freezing speed and direction\n', end='', flush=True)
                stop = True
                vessel.auto_pilot.reference_frame = calculate_landing_reference_frame(space_center, flight.latitude, flight.longitude, 0)
                vessel.auto_pilot.target_direction = (1, 0, 0)
                target_speed = speed_at_stop_altitude
        else:
            ratio = math.sqrt((altitude - stop_altitude) / (warp_altitude - stop_altitude))
            target_speed = ratio * speed_at_warp_altitude + (1 - ratio) * speed_at_stop_altitude
        speed = -flight.vertical_speed
        handle_message=handle_message('\rspeed: {} / {}, altitude: {}'.format(speed, target_speed, altitude), end='', flush=True)
        vessel.control.throttle = utilities.clamp(0, speed - target_speed, 1)
        if flight.surface_altitude < deploy_altitude and not legs_deployed:
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
