import numpy as np
import rospy


class LYAPUNOV:

     def __init__(self, posx, posy, posz):
        self.Initialize() 
        self.psi = 0  
        self.a = 0.3  # altura drone [m]
        self.b = 0.1  # distancia hacia el puntop de control [m]
        # ganancia K
        self.gain = 3
        # trajectory
        self.pxd = 3
        self.pyd = -2.5
        self.pzd = 3
        # integrales
        self.x1 = []
        self.y1 = []
        self.z1 = []
        self.x1_int = []
        self.y1_int = []
        self.z1_int = []

        self.hx = posx
        self.hy = posy
        self.hz = posz


     def set_posicion(self, posx, posy, posz):
        self.hx = posx
        self.hy = posy
        self.hz = posz

     def get_waypoint(self, wpx, wpy, wpz):
        self.pxd = wpx
        self.pyd = wpy
        self.pzd = wpz

     def Initialize(self):
        self.currtm = time.time()
        self.prevtm = self.currtm
          
     def compute(self):

        self.currtm = rospy.Time.now()      
        ts = (self.currtm - self.prevtm).to_sec()        

        # Error

        hxe = self.pxd - self.hx
        hye = self.pyd - self.hy
        hze = self.pzd - self.hz
        psie = self.psid - self.psi

        he = np.array([[hxe],[hye],[hze],[psie]]) # vector de errores (4x1)

        # Matriz Jacobiana
        J = np.array([[ np.cos(self.psi), -np.sin(self.psi)    , 0  , -self.b*np.sin(self.psi)],
                      [ np.sin(self.psi),  np.cos(self.psi)    , 0  ,  self.b*np.cos(self.psi)],
                      [ 0               ,  0                   , 1  ,  0                 ],
                      [ 0               ,  0                   , 0  ,  1                 ]])

        
        # Parametros de control

    

        K = self.gain * np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        
        # Ley de control
        qpRef = np.linalg.pinv(J).dot(K).dot(he)     # vector de acciones de control (3x1)

        ufRef  = qpRef[0][0]
        ulRef  = qpRef[1][0]
        uzRef  = qpRef[2][0]
        wRef   = qpRef[3][0]

        
        #################### APLICAR ACCIONES DE CONTROL #####################
        # # Integral numerica
        # self.psi_int = self.psi + ts * wRef

        # # Modelo cinematico
        # x1p = ufRef * np.cos(self.psi_int) - ulRef * np.sin(self.psi_int)
        # y1p = ufRef * np.sin(self.psi_int) + ulRef * np.cos(self.psi_int)
        # z1p = uzRef

        # # Integral numerica
        # self.x1_int = self.x1 + ts * x1p
        # self.y1_int = self.y1 + ts * y1p
        # self.z1_int = self.z1 + ts * z1p

        # # Cinematica directa
        # self._hx = self.x1_int + self.b * np.cos(psi[k+1])  
        # self._hy = self.y1_int + self.b * np.sin(psi[k+1])
        # self._hz = self.z1_int    + self.a

        self.prevtm = self.currtm

        return ufRef, ulRef, uzRef, wRef











     


