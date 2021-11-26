import vehicleState
import numpy as np

class CTRV():
    def __init__(self,YawRate,State,dt = 0.1):
        self.yaw_rate = YawRate
        self.dt = dt
        self.state = State
    
    def get_prediction(self):
        prediction = []
        v = (self.state.velocity.x **2 + self.state.velocity.y**2) **0.5
        
        location = self.state.transform.location
        yaw = np.deg2rad(self.state.transform.rotation.yaw)



        for _ in range(50):
            location.x += v * np.cos(yaw) * self.dt
            location.y += v * np.sin(yaw) * self.dt
            yaw += self.yaw_rate

            prediction.append([location.x,location.y,yaw])

        return prediction






