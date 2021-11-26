import numpy as np
from scipy.optimize import minimize, differential_evolution
import matplotlib.pyplot as plt





class state:
    def __init__(self, X=0, Y=0, YAW=0, V=0, DELTA = 0):
        self.x = X
        self.y = Y
        self.yaw = YAW
        self.v = V
        self.delta = DELTA

    def Show(self,):
        print(round(self.x,2), round(self.y,2), round(self.yaw,2), round(self.v,2), round(self.delta,2))

class inputs:
    def __init__(self, delta_rate=0, acceleration=0):
        self.delta_rate = delta_rate
        self.acceleration = acceleration


class OCP:
    def __init__(self,START_X,START_Y,FINAL_X,START_YAW,START_V,START_DELTA,Prediction):
        self.L = 4
        self.start_x = START_X
        self.start_y = START_Y
        self.final_x = FINAL_X
        self.start_v = START_V
        self.start_delta = START_DELTA
        self.init_state = state(START_X,START_Y,START_YAW,START_V,START_DELTA)
        self.prediction = Prediction




        self.T = 5
        self.time_step = 0.1
        self.total_step = int(self.T/self.time_step)

        bound1 = (-np.pi/2,np.pi/2)
        bound2 = (-6,1.2)

        self.bnds = (bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound1,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2,bound2)
    def getNextState(self,inputs, init_state, dt = 0.1, L = 4):

        next_state = state()
        
        ## find the final satte after dt of time ###
        next_state.x  = init_state.x  + init_state.v*np.cos(init_state.yaw)*dt
        next_state.y  = init_state.y  + init_state.v*np.sin(init_state.yaw)*dt
        # print("next state yaw ",type(next_state.yaw))
        # print("init state yaw ",type(init_state.yaw))
        # print("init state v ",type(initt_state.v))
        next_state.yaw = init_state.yaw + init_state.v * np.tan(init_state.delta)/ L * dt 
        next_state.v  = init_state.v  + inputs.acceleration*dt
        next_state.delta = init_state.delta + inputs.delta_rate*dt

        if abs(next_state.delta) > np.pi /3:
            if next_state.delta > 0:
                next_state.delta = np.pi/3
            else:
                next_state.delta = -np.pi/3
        if next_state.v < 0:
            next_state.v = 0


        return next_state

    def getResult(self,input):
        prev_state = self.init_state
        xs = []
        ys = []

        yaws = []
        vs = []
        deltas = []


        # arr_us = []
        # arr_vs = []
        for i in range(self.total_step):
            input_state = inputs()
            input_state.delta_rate = input[i]
            input_state.acceleration = input[i+self.total_step]
            next_state = self.getNextState(input_state,prev_state)
            # next_state.Show()
            xs.append(prev_state.x)
            ys.append(prev_state.y)
            yaws.append(prev_state.yaw)
            vs.append(prev_state.v)
            deltas.append(prev_state.delta)

            # arr_us.append(np.cos(prev_state.yaw))
            # arr_vs.append(np.sin(prev_state.yaw))

            prev_state = next_state
        # plt.quiver(xs,ys,arr_us,arr_vs)
        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.show()
        delta_rates = input[:self.total_step]
        accelerations = input[self.total_step+1:self.total_step * 2]

        return xs,ys,yaws,vs,deltas,delta_rates,accelerations



    def getR(self,delta):
        if delta == 0:
            return 1e9
        else:
            return self.L / np.tan(delta)

    def costFunction(self,x):
        prev_state = self.init_state
        cost = 0
        u = inputs()
        for i in range(self.total_step):
            u.delta_rate = x[i]
            u.acceleration = x[i+self.total_step]
            next_state = self.getNextState(u,prev_state,dt = self.time_step, L = 4)

            R = self.getR(next_state.delta)
            # total_acc = (u.acceleration **2 + next_state.v **2/R )** 0.5 
            cost += 5.0*abs(next_state.x-self.final_x) + 3*abs(next_state.v-5) + 30*abs(next_state.yaw + np.pi/2) + self.collisionCheck(next_state,self.prediction[i])
            
            prev_state = next_state
        return cost

    def calculate(self):
        init_point = np.zeros(2*self.total_step)

        optimum = minimize(self.costFunction, init_point, method='SLSQP',bounds=self.bnds)
        output = self.getResult(optimum.x)
        return output



    def costFunction2(self,x):
        prev_state = self.init_state
        cost = 0
        u = inputs()
        collision_position_difference = 0
        
        for i in range(self.total_step):
            u.delta_rate = x[i]
            u.acceleration = x[i+self.total_step]
            next_state = self.getNextState(u,prev_state,dt = self.time_step, L = 4)

            R = self.getR(next_state.delta)
            # total_acc = (u.acceleration **2 + next_state.v **2/R )** 0.5 
            collision_location = self.collisionCheck2(next_state,self.prediction[i])
            

            #  trial 1
            # if collision_position_difference == 0 and collision_location:   
            #     collision_position_difference = -collision_location[0] + collision_location[1]
            #     if 3< collision_position_difference < 6 or collision_position_difference < 0:
            #         cost += 0.3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-7.5) + 40*abs(next_state.yaw + np.pi/2) - 200000*(collision_position_difference - 5)
            #     else:
            #         cost += 0.3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-7.5) + 40*abs(next_state.yaw + np.pi/2) - 200000*(collision_position_difference - 5)
            # else:
            #     # if not collision_location:
            #     #     cost += 0.7*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-13) + 10*abs(next_state.yaw + np.pi/2) + 0.8*(next_state.y - self.prediction[i][1])
            #     # else:
            #     #     cost += 0.7*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-13) + 10*abs(next_state.yaw + np.pi/2) + 10
            #     cost += 0.3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-7.5) + 10*abs(next_state.yaw + np.pi/2) + 0.5*(next_state.y - self.prediction[i][1])
            
            #  trial 2
            # if collision_position_difference == 0:
            #     if collision_location:
            #         collision_position_difference = -collision_location[0] + collision_location[1]
            #         cost += 3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-5) + 100*abs(next_state.yaw + np.pi/2) - 40*(collision_position_difference - 5)
            #     else:
            #         cost += 3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-5) + 100*abs(next_state.yaw + np.pi/2) + 2*(next_state.y - self.prediction[i][1])

            #  trial 3
            #  before collision
            # if collision_position_difference == 0:
            #     if collision_location:
            #         collision_position_difference = -collision_location[0] + collision_location[1]
            #         cost += 3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-5) + 100*abs(next_state.yaw + np.pi/2) - 40*(collision_position_difference - 5)
            #     else:
            #         cost += 3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-5) + 100*abs(next_state.yaw + np.pi/2) + 2*(next_state.y - self.prediction[i][1])
            # #  after collision
            # else:
            #     if collision_location:
            #         collision_position_difference = -collision_location[0] + collision_location[1]
            #         cost += 3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-5) + 100*abs(next_state.yaw + np.pi/2) - 40*(collision_position_difference - 5)
            #     else:
            #         cost += 3*abs(next_state.x-self.final_x) + 0.2*abs(next_state.v-5) + 100*abs(next_state.yaw + np.pi/2)

            #  trial 4
            collision_position_difference = 4
            if collision_location:
                collision_position_difference = -collision_location[0] + collision_location[1]
            cost += 3.5*abs(next_state.x-self.final_x) + 1.0*abs(next_state.v-5) + 30*abs(next_state.yaw + np.pi/2) - 60*(collision_position_difference - 5)


        





            prev_state = next_state

        return cost

    def calculate2(self):
        init_point = np.zeros(2*self.total_step)

        optimum = minimize(self.costFunction2, init_point, method='SLSQP',bounds=self.bnds)
        output = self.getResult(optimum.x)
        return output
    
    
    def collisionCheck2(self,ego_state, opp_state):
        # R = 1.546 
        R = 1.35
        # bigger_R = 2.3
        dL = 2
        ego_dx = dL * np.cos(ego_state.yaw)
        ego_dy = dL * np.sin(ego_state.yaw)
        opp_dx = dL * np.cos(opp_state[2])
        opp_dy = dL * np.sin(opp_state[2])

        ego_collision_point = [[ego_state.x,ego_state.y],[ego_state.x + ego_dx ,ego_state.y +ego_dy],[ego_state.x - ego_dx,ego_state.y - ego_dy]]
        opp_collision_point = [[opp_state[0],opp_state[1]], [opp_state[0] + opp_dx, opp_state[1] + opp_dy], [opp_state[0] - opp_dx, opp_state[1] - opp_dy]]
        
        for ep in ego_collision_point:
            for op in opp_collision_point:
                dist = self.distance(ep,op)
                if dist < 2*R:
                    return [ep[1],op[1],ep[0],op[0]]
                    # return [ego_state.y,opp_state[1],ego_state.x,opp_state[0]]
                # if dist < bigger_R:
                #     return 1/dist - 1/bigger_R
        return False
    
    def collisionCheck(self,ego_state, opp_state):
        # R = 1.546 
        R = 1.45
        # bigger_R = 2.3
        dL = 2
        ego_dx = dL * np.cos(ego_state.yaw)
        ego_dy = dL * np.sin(ego_state.yaw)
        opp_dx = dL * np.cos(opp_state[2])
        opp_dy = dL * np.sin(opp_state[2])

        ego_collision_point = [[ego_state.x,ego_state.y],[ego_state.x + ego_dx ,ego_state.y +ego_dy],[ego_state.x - ego_dx,ego_state.y - ego_dy]]
        opp_collision_point = [[opp_state[0],opp_state[1]], [opp_state[0] + opp_dx, opp_state[1] + opp_dy], [opp_state[0] - opp_dx, opp_state[1] - opp_dy]]
        
        for ep in ego_collision_point:
            for op in opp_collision_point:
                dist = self.distance(ep,op)
                if dist < 2*R:
                    
                    return 999999999999999
        return False

    def distance(self,point_a,point_b):
        return ((point_a[0]-point_b[0])**2 + (point_a[1]-point_b[1])**2)**0.5
    



#output x,y,yaw,v,delta,deltarate,acceleration
def collisionCheck(ego_state, opp_state):
    # R = 1.546 
    R = 1.35
    # bigger_R = 2.3
    dL = 2
    ego_dx = dL * np.cos(ego_state[2])
    ego_dy = dL * np.sin(ego_state[2])
    opp_dx = dL * np.cos(opp_state[2])
    opp_dy = dL * np.sin(opp_state[2])

    ego_collision_point = [[ego_state[0],ego_state[1]],[ego_state[0] + ego_dx ,ego_state[1] +ego_dy],[ego_state[0] - ego_dx,ego_state[1] - ego_dy]]
    opp_collision_point = [[opp_state[0],opp_state[1]], [opp_state[0] + opp_dx, opp_state[1] + opp_dy], [opp_state[0] - opp_dx, opp_state[1] - opp_dy]]
    
    for ep in ego_collision_point:
        for op in opp_collision_point:
            dist = distance(ep,op)
            if dist < 2*R:
                return True,ep,op
            # if dist < bigger_R:
            #     return 1/dist - 1/bigger_R
    return False,ep,op

def collisionCheck2(ego_state, opp_state):
    # R = 1.546 
    R = 1.45
    # bigger_R = 2.3
    dL = 2
    ego_dx = dL * np.cos(ego_state[2])
    ego_dy = dL * np.sin(ego_state[2])
    opp_dx = dL * np.cos(opp_state[2])
    opp_dy = dL * np.sin(opp_state[2])

    ego_collision_point = [[ego_state[0],ego_state[1]],[ego_state[0] + ego_dx ,ego_state[1] +ego_dy],[ego_state[0] - ego_dx,ego_state[1] - ego_dy]]
    opp_collision_point = [[opp_state[0],opp_state[1]], [opp_state[0] + opp_dx, opp_state[1] + opp_dy], [opp_state[0] - opp_dx, opp_state[1] - opp_dy]]
    
    for ep in ego_collision_point:
        for op in opp_collision_point:
            dist = distance(ep,op)
            if dist < 2*R:
                return True,ep,op
            # if dist < bigger_R:
            #     return 1/dist - 1/bigger_R
    return False,ep,op

def distance(point_a,point_b):
    return ((point_a[0]-point_b[0])**2 + (point_a[1]-point_b[1])**2)**0.5



    
