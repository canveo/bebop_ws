import numpy as np

# cuadrado
WAYPOINTS = [
    (0.0, 0.0, 1.0),
    (2.0, 0.0, 1.0),
    (2.0, 2.0, 1.0),
    (0.0, 2.0, 1.0),
    (0.0, 0.0, 1.0)
]

# cuadrado con giro
WAYPOINTS_YAW = [
    (0,   0,   1, np.deg2rad(270)),
    (0,   0,   1, np.deg2rad(0))  ,
    (2.0, 0,   1, np.deg2rad(0))  ,
    (2.0, 0,   1, np.deg2rad(90)) ,
    (2.0, 2.0, 1, np.deg2rad(90)) ,
    (2.0, 2.0, 1, np.deg2rad(180)),
    (0,   2.0, 1, np.deg2rad(180)),
    (0,   2.0, 1, np.deg2rad(270)),
    (0,   0 ,  1, np.deg2rad(270))
]