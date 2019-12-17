import numpy as np
import geodesy.conversions as gc

kml_ref = np.array([-77.0001, 45.0001, 361])
kml = np.array([-77., 45., 360.])

lla_ref = gc.kml2lla(kml_ref)
lla = gc.kml2lla(kml)
print lla

ned = gc.lla2ned(lla, lla_ref)
print ned

lla_2 = gc.ned2lla(ned, lla_ref)
print lla_2

kml_a = np.vstack((kml, kml, kml, kml, kml))
print kml

print '\n\narray operations:\n'

lla_a = gc.kml2lla(kml_a)
print lla_a

ned_a = gc.lla2ned(lla_a, lla_ref)
print ned_a

lla_2_a = gc.ned2lla(ned_a, lla_ref)
print lla_2_a
