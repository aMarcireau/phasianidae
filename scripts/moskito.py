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
phasianidae.orbit.cleanup()
for engine in vessel.parts.engines:
    engine.active = (engine.part.name != 'engineLargeSkipper')
phasianidae.orbit.transfer(space_center, 12500)
phasianidae.body.landing(space_center, (0, -10, 0, 1000), 3000, 8.2, 90)
phasianidae.body.harvest(space_center)
