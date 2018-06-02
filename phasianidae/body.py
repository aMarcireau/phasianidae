import math
import time
from . import orbit
from . import utilities

def deploy_legs(connection, index):
    space_center = connection.space_center
    if index is not None:
        legs = space_center.active_vessel.parts.legs
        if legs[index].state == space_center.LegState.retracted:
            legs[index].deployed = True
        index += 1
        if index < len(legs):
            return index
        return None

def retract_legs(connection, index):
    space_center = connection.space_center
    if index is not None:
        legs = space_center.active_vessel.parts.legs
        if legs[index].state == space_center.LegState.deployed:
            legs[index].deployed = False
        index += 1
        if index < len(legs):
            return index
        return None

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

def cleanup(connection):
    space_center = connection.space_center
    vessel = space_center.active_vessel
    space_center.rails_warp_factor = 0
    space_center.physics_warp_factor = 0
    vessel.control.remove_nodes()
    vessel.control.throttle = 0
    vessel.control.rcs = False
    vessel.auto_pilot.disengage()
    vessel.auto_pilot.target_roll = float('nan')
    vessel.auto_pilot.sas = False
    for light in vessel.parts.lights:
        light.active = True
    for solar_panel in vessel.parts.solar_panels:
        if solar_panel.deployable and not solar_panel.deployed:
            solar_panel.deployed = True
    for radiator in vessel.parts.radiators:
        if radiator.deployable and not radiator.deployed:
            radiator.deployed = True

def take_off(
    connection,
    target_altitude,
    pitch_profile,
    initial_time_to_apoapsis,
    final_time_to_apoapsis,
    eccentricity_error,
    apoapsis_gain,
    time_to_apoapsis_gain,
    handle_message=print,
    handle_stage=None):
    space_center = connection.space_center
    vessel = space_center.active_vessel
    body = vessel.orbit.body
    flight = vessel.flight(body.reference_frame)
    flight = vessel.flight(calculate_landing_reference_frame(space_center, flight.latitude, flight.longitude, 0))
    vessel.auto_pilot.target_direction = (1, 0, 0)
    vessel.auto_pilot.engage()
    vessel.control.throttle = 1
    leg_index = 0
    apoapsis_error = float('inf')
    time_to_apoapsis = None
    phase = 0
    while True:

        # manage legs
        leg_index = retract_legs(connection, leg_index)

        # manage stagging
        if handle_stage is not None:
            handle_stage(connection)

        # manage direction and throttle
        altitude = flight.mean_altitude
        if phase == 0:
            new_apoapsis_error = target_altitude - vessel.orbit.apoapsis_altitude
            handle_message('\rapoapsis error: {}'.format(new_apoapsis_error), end='', flush=True)
            if new_apoapsis_error > apoapsis_error and flight.atmosphere_density == 0:
                handle_message('\n', end='', flush=True)
                vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
                vessel.control.throttle = 0
                phase = 1
            else:
                angle = utilities.clamp(
                    0.0,
                    sum(coefficient * float(altitude ** index) for index, coefficient in enumerate(pitch_profile)),
                    90.0) / 180.0 *  math.pi
                vessel.auto_pilot.target_direction = (math.sin(angle), 0, math.cos(angle))
                vessel.control.throttle = utilities.clamp(0, new_apoapsis_error * apoapsis_gain, 1)
            apoapsis_error = new_apoapsis_error
        elif phase == 1:
            handle_message('head to prograde at apoapsis')
            orbit.target_direction_then_warp(
                connection,
                space_center.ut + vessel.orbit.time_to_apoapsis - initial_time_to_apoapsis,
                (0, 1, 0))
            time_to_apoapsis = vessel.orbit.time_to_apoapsis
            vessel.control.throttle = 1
            phase = 2
        elif phase == 2:
            new_time_to_apoapsis = vessel.orbit.time_to_apoapsis
            if new_time_to_apoapsis > time_to_apoapsis:
                vessel.control.throttle = 0
                handle_message('\rtime to apoapsis: {} s\nthrust is large enough, stabilizing near apoapsis\n'.format(new_time_to_apoapsis), end='', flush=True)
                space_center.warp_to(space_center.ut + vessel.orbit.time_to_apoapsis - final_time_to_apoapsis)
                phase = 3
            else:
                handle_message('\rtime to apoapsis: {} s'.format(new_time_to_apoapsis), end='', flush=True)
                time_to_apoapsis = new_time_to_apoapsis
        elif phase == 3:
            eccentricity = vessel.orbit.eccentricity
            time_to_apoapsis = vessel.orbit.time_to_apoapsis
            handle_message('\rtime to apoapsis: {} s, eccentricity: {}'.format(time_to_apoapsis, eccentricity), end='', flush=True)
            if eccentricity < eccentricity_error:
                vessel.control.throttle = 0
                handle_message('\n', end='', flush=True)
                break
            vessel.control.throttle = utilities.clamp(0, (final_time_to_apoapsis - time_to_apoapsis) * time_to_apoapsis_gain, 1)
        else:
            raise Exception('unknown phase')
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
    leg_index = 0
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
        handle_message('\rspeed: {} / {}, altitude: {}'.format(speed, target_speed, altitude), end='', flush=True)
        vessel.control.throttle = utilities.clamp(0, speed - target_speed, 1)
        if flight.surface_altitude < deploy_altitude:
            leg_index = deploy_legs(connection, leg_index)
        if vessel.situation == space_center.VesselSituation.landed:
            vessel.control.throttle = 0
            break
        time.sleep(0.1)

def harvest(connection):
    space_center = connection.space_center
    vessel = space_center.active_vessel
    for radiator in vessel.parts.radiators:
        if radiator.deployable and not radiator.deployed:
            radiator.deployed = True
    for resource_harvester in vessel.parts.resource_harvesters:
        resource_harvester.deployed = True
    for resource_harvester in vessel.parts.resource_harvesters:
        if resource_harvester.state != space_center.ResourceHarvesterState.active:
            while resource_harvester.state != space_center.ResourceHarvesterState.deployed:
                time.sleep(0.1)
            resource_harvester.active = True

def stop_harvest(connection):
    space_center = connection.space_center
    vessel = space_center.active_vessel
    for radiator in vessel.parts.radiators:
        if radiator.deployable and radiator.deployed:
            radiator.deployed = False
    for resource_harvester in vessel.parts.resource_harvesters:
        resource_harvester.active = False
        resource_harvester.deployed = False
    for resource_harvester in vessel.parts.resource_harvesters:
        while resource_harvester.state != space_center.ResourceHarvesterState.retracted:
            time.sleep(0.1)
