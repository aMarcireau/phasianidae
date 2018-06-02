import krpc
import os
import scipy.interpolate
import sys
import time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import phasianidae

connection = krpc.connect()
space_center = connection.space_center
vessel = space_center.active_vessel
if vessel.name != 'moustique':
    raise RuntimeError('the vessel must be a moustique')
for engine in vessel.parts.engines:
    engine.active = (engine.part.name != 'engineLargeSkipper')
for resource_converter in vessel.parts.resource_converters:
    resource_converter.start(0)
    resource_converter.start(1)
    resource_converter.stop(2)
    resource_converter.stop(3)

phasianidae.orbit.cleanup(connection=connection)
phasianidae.orbit.transfer(connection=connection, altitude=12500, error=0.05, altitude_gain=0.5)
phasianidae.body.landing(
    connection=connection,
    landing_site=(0, -10, 0, 2000),
    warp_altitude=3000,
    stop_altitude=10,
    speed_at_warp_altitude=90,
    speed_at_stop_altitude=0.2,
    deploy_altitude=100)
phasianidae.body.cleanup(connection=connection)
phasianidae.body.harvest(connection=connection)
while True:
    ratios_and_names = [
        (round(vessel.resources.amount(name) / vessel.resources.max(name), 1), name)
        for name in ('Ore', 'LiquidFuel', 'Oxidizer', 'MonoPropellant')]
    print(
        '\r' + ', '.join([ratio_and_name[1] + ': ' + str(ratio_and_name[0]) for ratio_and_name in ratios_and_names]),
        end='',
        flush=True)
    if all(ratio_and_name[0] == 1 for ratio_and_name in ratios_and_names):
        print('\n', end='', flush=True)
        space_center.rails_warp_factor = 0
        break
    else:
        space_center.rails_warp_factor = 7
    time.sleep(0.1)
phasianidae.body.stop_harvest(connection=connection)
phasianidae.body.take_off(
    connection=connection,
    target_altitude=20000,
    pitch_profile=list(reversed(scipy.interpolate.lagrange([0, 500, 20000], [90, 45, 0]).coefficients.tolist())),
    initial_time_to_apoapsis=60,
    final_time_to_apoapsis=10,
    eccentricity_error=0.005,
    apoapsis_gain=0.001,
    time_to_apoapsis_gain=0.1)
phasianidae.orbit.cleanup(connection=connection)
phasianidae.orbit.transfer(connection=connection, altitude=2e6, altitude_gain=0.5)
