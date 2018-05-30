import krpc
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import phasianidae

connection = krpc.connect()
space_center = connection.space_center
vessel = space_center.active_vessel
if vessel.name != 'moskito':
    raise RuntimeError('the vessel must be a moskito')
phasianidae.orbit.cleanup(space_center)
for engine in vessel.parts.engines:
    engine.active = (engine.part.name != 'engineLargeSkipper')
phasianidae.orbit.transfer(connection, 12500)
phasianidae.body.landing(connection, (0, -10, 0, 2000), 3000, 10, 90, 0.2, 100)
phasianidae.body.harvest(space_center)
