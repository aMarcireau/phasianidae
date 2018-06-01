import math
import scipy.optimize
import time
from . import utilities

def cleanup(space_center):
    vessel = space_center.active_vessel
    space_center.rails_warp_factor = 0
    space_center.physics_warp_factor = 0
    vessel.control.remove_nodes()
    vessel.control.throttle = 0
    vessel.control.rcs = False
    vessel.parts.legs[0].deployed = False
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_roll = float('nan')
    vessel.auto_pilot.sas = False
    for resource_harvester in vessel.parts.resource_harvesters:
        if resource_harvester.deployed:
            resource_harvester.deployed = False
    for light in vessel.parts.lights:
        light.active = True
    for solar_panel in vessel.parts.solar_panels:
        if solar_panel.deployable and not solar_panel.deployed:
            solar_panel.deployed = True
    for radiator in vessel.parts.radiators:
        if radiator.deployable and not radiator.deployed:
            radiator.deployed = True

def closest_approach(space_center, reference_frame):
    orbit = space_center.active_vessel.orbit
    now = space_center.ut
    optimum = scipy.optimize.minimize_scalar(
        lambda ut: math.sqrt(sum(coordinate ** 2 for coordinate in orbit.position_at(ut, reference_frame))),
        bounds=(now, now + orbit.period),
        method='bounded').x
    print('closest approach in [{}, {}]: {} (position: {})'.format(
        now,
        now + orbit.period,
        optimum,
        orbit.position_at(optimum, reference_frame)))
    return optimum

def wait_auto_pilot(connection, error=0.5):
    vessel = connection.space_center.active_vessel
    vessel.auto_pilot.attenuation_angle = (error, error, error)
    connection.space_center.physics_warp_factor = 3
    while abs(vessel.auto_pilot.error) > error * 10:
        print('\r{}'.format(abs(vessel.auto_pilot.error)), end='', flush=True)
        time.sleep(0.1)
    connection.space_center.physics_warp_factor = 0
    previous_error = abs(vessel.auto_pilot.error)
    ticks_left = 50
    while True:
        current_error = abs(vessel.auto_pilot.error)
        print('\r{}'.format(current_error), end='', flush=True)
        ticks_left -= 1
        if ticks_left == 0:
            if abs(previous_error - current_error) / current_error < 0.001:
                print('\nrelative variation in 5 s: {}'.format(
                    abs(previous_error - current_error) / current_error), end='', flush=True)
                break
            previous_error = current_error
            ticks_left = 50
        if current_error < error:
            break
        time.sleep(0.1)
    print('\n', end='', flush=True)

def target_direction_then_warp(connection, ut, direction, error=0.5):
    space_center = connection.space_center
    vessel = space_center.active_vessel
    vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
    node = vessel.control.add_node(ut, direction[1] * 100, direction[2] * 100, -direction[0] * 100)
    vessel.auto_pilot.target_direction = node.direction(vessel.auto_pilot.reference_frame)
    wait_auto_pilot(connection, error)
    space_center.warp_to(ut - 60)
    vessel.auto_pilot.target_direction = node.direction(vessel.auto_pilot.reference_frame)
    wait_auto_pilot(connection, error)
    space_center.warp_to(ut - 10)
    vessel.auto_pilot.target_direction = node.direction(vessel.auto_pilot.reference_frame)
    wait_auto_pilot(connection, error)
    space_center.warp_to(ut)
    node.remove()
    vessel.auto_pilot.target_direction = direction

def burn(vessel, error, gain, throttle_error=None):
    previous_error = error(vessel)
    while True:
        current_error = error(vessel)
        print('\r{}'.format(current_error), end='', flush=True)
        if current_error > previous_error:
            vessel.control.throttle = 0
            break
        previous_error = current_error
        vessel.control.throttle = utilities.clamp(0, (current_error if throttle_error is None else throttle_error(vessel)) * gain, 1)
        time.sleep(0.1)
    print('\n', end='', flush=True)

def transfer(connection, altitude, inclination_gain=100, altitude_gain=1, error=0.001, direction_error=0.5):
    space_center = connection.space_center
    vessel = space_center.active_vessel
    while True:
        inclination = vessel.orbit.inclination
        print('inclination: {}'.format(inclination))
        if inclination / (2 * math.pi) < error:
            break
        print('head to anti-normal at ascending node')
        target_direction_then_warp(
            connection,
            vessel.orbit.ut_at_true_anomaly(2 * math.pi - vessel.orbit.argument_of_periapsis),
            (0, 0, -1),
            direction_error)
        print('burn')
        burn(vessel, lambda vessel: vessel.orbit.inclination / (2 * math.pi), inclination_gain)
    while True:
        def altitude_error(orbit, altitude):
            return math.hypot((orbit.apoapsis_altitude - altitude) / altitude, (orbit.periapsis_altitude - altitude) / altitude)
        print('altitude error: {}'.format(altitude_error(vessel.orbit, altitude)))
        apoapsis_set = abs(vessel.orbit.apoapsis_altitude - altitude) / altitude < error
        periapsis_set = abs(vessel.orbit.periapsis_altitude - altitude) / altitude < error
        if apoapsis_set and periapsis_set:
            break
        if not apoapsis_set and not periapsis_set:
            if vessel.orbit.apoapsis_altitude > altitude:
                if vessel.orbit.periapsis_altitude > altitude:
                    print('head to retrograde at apoapsis')
                    target_direction_then_warp(
                        connection,
                        space_center.ut + vessel.orbit.time_to_apoapsis,
                        (0, -1, 0),
                        direction_error)
                else:
                    print('head to prograde at apoapsis')
                    target_direction_then_warp(
                        connection,
                        space_center.ut + vessel.orbit.time_to_apoapsis,
                        (0, 1, 0),
                        direction_error)
                print('burn')
                burn(
                    vessel,
                    lambda vessel: altitude_error(vessel.orbit, altitude),
                    altitude_gain,
                    lambda vessel: abs(vessel.orbit.periapsis_altitude - altitude) / altitude)
            else:
                print('head to retrograde at periapsis')
                target_direction_then_warp(
                    connection,
                    space_center.ut + vessel.orbit.time_to_periapsis,
                    (0, -1, 0),
                    direction_error)
                print('burn')
                burn(
                    vessel,
                    lambda vessel: altitude_error(vessel.orbit, altitude),
                    altitude_gain,
                    lambda vessel: abs(vessel.orbit.apoapsis_altitude - altitude) / altitude)
        elif apoapsis_set:
            print('head to prograde at apiapsis')
            target_direction_then_warp(
                connection,
                space_center.ut + vessel.orbit.time_to_apoapsis,
                (0, 1, 0),
                direction_error)
            print('burn')
            burn(
                vessel,
                lambda vessel: altitude_error(vessel.orbit, altitude),
                altitude_gain,
                lambda vessel: abs(vessel.orbit.periapsis_altitude - altitude) / altitude)
        else:
            print('head to retrograde at periapsis')
            target_direction_then_warp(
                connection,
                space_center.ut + vessel.orbit.time_to_periapsis,
                (0, -1, 0),
                direction_error)
            print('burn')
            burn(
                vessel,
                lambda vessel: altitude_error(vessel.orbit, altitude),
                altitude_gain,
                lambda vessel: abs(vessel.orbit.apoapsis_altitude - altitude) / altitude)
