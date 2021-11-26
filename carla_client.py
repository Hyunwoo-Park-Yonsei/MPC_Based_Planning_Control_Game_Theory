#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.



import glob
import os
import sys
import glob
import os
import sys
import argparse
import numpy as np
import random
import time
import math
import queue
import collections
import OCP
import OCP_upper_hand
import vehicleState
import CTRV
import time


try:
    sys.path.append(glob.glob('/opt/carla-simulator/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import math

delta_ratio = 1/(np.pi/3)
def ThrottleRatio(acc):
    if acc <= 0:
        return 0
    else:
        return acc/2 +0.4

def function_handler(event):
    actor_we_collide_against = event.other_actor
    impulse = event.normal_impulse
    intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)


def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-s', '--speed',
        metavar='FACTOR',
        default=1.0,
        type=float,
        help='rate at which the weather changes (default: 1.0)')
    args = argparser.parse_args()

    speed_factor = args.speed
    update_freq = 0.1 / speed_factor

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    map = world.get_map()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    actor_list = []

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('cybertruck')[0]
    spawnpoint = carla.Transform(carla.Location(x=-74.4,y=-8.0,z = 0),carla.Rotation(pitch = 0, yaw = 270, roll = 0))
    spawnpoint2 = carla.Transform(carla.Location(x=-77.85,y=-8.0,z = 0),carla.Rotation(pitch = 0, yaw = 270, roll = 0))

    ego_vehicle = world.spawn_actor(vehicle_bp, spawnpoint)
    vehicle1 = world.spawn_actor(vehicle_bp, spawnpoint2)
    actor_list.append(ego_vehicle)
    actor_list.append(vehicle1)
    # client.start_recorder("/home/recording01.log")

    prev_yaw = np.deg2rad(270)
    prev_ego_yaw = np.deg2rad(270)

    
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    
    tm.distance_to_leading_vehicle(ego_vehicle,5)
    tm.global_percentage_speed_difference(40)

    #collision

    blueprint_library = world.get_blueprint_library()
    collision_sensor = world.spawn_actor(blueprint_library.find('sensor.other.collision'),carla.Transform(), attach_to=ego_vehicle)
    collision_sensor.listen(lambda event: function_handler(event)) 



    
    # remove buffering
    ego_vehicle.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
    vehicle1.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
    world.tick()
    ego_vehicle.apply_control(carla.VehicleControl(manual_gear_shift=False))
    vehicle1.apply_control(carla.VehicleControl(manual_gear_shift=False))
    world.tick()
    # remove buffering
    start_time = time.time()
    state = 0

    fast = False
    while True:
        world.tick()


    

        v1state = vehicleState.VehicleState(vehicle1.get_transform(),vehicle1.get_velocity())
        ego_vehicle_state = vehicleState.VehicleState(ego_vehicle.get_transform(),ego_vehicle.get_velocity())
        yaw_rate = np.deg2rad(v1state.transform.rotation.yaw) - prev_yaw
        ego_yaw_rate = np.deg2rad(ego_vehicle_state.transform.rotation.yaw) -prev_ego_yaw
        if abs(yaw_rate) < 0.0001:
            yaw_rate = 0.0001
        if abs(ego_yaw_rate) < 0.0001:
            ego_yaw_rate

        Predict = CTRV.CTRV(yaw_rate,v1state)
        prediction = Predict.get_prediction()

        Predict2 = CTRV.CTRV(ego_yaw_rate,ego_vehicle_state)
        ego_vehicle_prediction = Predict2.get_prediction()

        for point in prediction:
            p = carla.Location()
            p.x = point[0]
            p.y = point[1]
            world.debug.draw_string(p, 'O', draw_shadow=False, color=carla.Color(r=250, g=0, b=0), life_time=0.01, persistent_lines=True)
        prev_yaw = np.deg2rad(v1state.transform.rotation.yaw)
        prev_ego_yaw = np.deg2rad(ego_vehicle_state.transform.rotation.yaw)


        

        


        #closest waypoint
        waypoints = world.get_map().get_waypoint(ego_vehicle.get_location())
        #number of waypoints.next() is number of directions that ego-vehicle can go
        # waypoints_arr = waypoints.next(10)
        if waypoints.lane_id == 3:

            '''
                1. planned path에 현재 차위치 더해서 첫번째 collision 구하기
                2. 첫번째 collision의 상대적 위치 차로 주도권 계산
                3. 주도권이 낮다면 주도권 높게 mpc
                차 선택은? 바로왼쪽차 차선변경 할지 그 뒤에차 차선변경 할지
                case 2개 나눠서해야할듯
                
            '''
            left_waypoint = waypoints.get_left_lane()
            location = ego_vehicle.get_location()
            vector_v = ego_vehicle.get_velocity()
            v = (vector_v.x **2 + vector_v.y **2)**0.5

            yaw = np.deg2rad(ego_vehicle.get_transform().rotation.yaw)


            delta = ego_vehicle.get_control().steer
            if state == 0:
                output = [0,0,0,0,[0,0.0],0,[0,0.65]]
                planned_points = []
                if time.time()-start_time > 5:
                    state = 1
                else:
                    print("it is state 0")


            elif state == 1:

                test_model = OCP.OCP(location.x, location.y, left_waypoint.transform.location.x, yaw, v, delta, prediction)
                output = test_model.calculate2()
                
                
                first_collision_point = carla.Location()
                first_collision_point.x = 0
                first_collision_point.y = 0
                collision_position_difference = 0
                collision_time = 10
                planned_points = []
                is_first = True
                for i in range(len(output[0])):
                    point = carla.Location()
                    point.x = output[0][i]
                    point.y = output[1][i]
                    planned_points.append(point)
                    new_output = [output[0][i],output[1][i],output[2][i]]
                    
                    #planning 된 경로와 상대차량의 예측 경로의 충돌 예측 지점
                    check_collision = OCP.collisionCheck(new_output,prediction[i])
                    if check_collision[0] > 0:
                        collision_point = carla.Location()
                        collision_point.x = (check_collision[1][0]+check_collision[2][0])/2
                        collision_point.y = (check_collision[1][1]+check_collision[2][1])/2
                        if is_first:
                            first_collision_point = collision_point
                            collision_position_difference = -new_output[1] + prediction[i][1]
                            collision_time = i*0.1
                            print( "collision position difference : ", collision_position_difference, "collision after ", collision_time , "seconds")
                            
                            is_first = False
                        else:
                            world.debug.draw_string(collision_point, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=0.00001, persistent_lines=True)
                world.debug.draw_string(first_collision_point, 'X', draw_shadow=False, color=carla.Color(r=255, g=0, b=255), life_time=0.000001, persistent_lines=True)

                


                # original collsion time 1.3
                if collision_position_difference > 4.3 and collision_time <= 0.91:
                    state = 2
            
            elif state == 2:


                current_vehicle1_x = vehicle1.get_location().x
                current_vehicle1_y = vehicle1.get_location().y
                current_vehicle1_yaw = np.deg2rad(-270)
                current_vehicle1_vel = vehicle1.get_velocity().y
                expected_collision_time = 5
                expected_collision_distance = 20
                predicted_vehicle1_acc = 0

                prediction3 = []

                for t in range(len(planned_points)):
                    current_vehicle1_y += current_vehicle1_vel*0.1
                    vehicle1_state = [current_vehicle1_x,current_vehicle1_y,current_vehicle1_yaw]
                    ego_vehicle_state = [output[0][i],output[1][i],output[2][i]]
                    check_collision = OCP.collisionCheck2(vehicle1_state,ego_vehicle_state)
                    if check_collision[0]:
                        expected_collision_time = t*0.1
                        break

                if expected_collision_time < 5:
                    expected_collision_distance =  -expected_collision_time* vehicle1.get_velocity().y
                    # for safe distance
                    if expected_collision_distance > 0.5:
                        expected_collision_distance -= 0.5
                    
                    if expected_collision_distance != 0:
                        predicted_vehicle1_acc = -4/expected_collision_distance
                    else:
                        predicted_vehicle1_acc = -10

                for t in range(len(planned_points)):
                    predicted_vehicle1_y =  vehicle1.get_location().y + current_vehicle1_vel*0.1*t - predicted_vehicle1_acc*0.5*(0.01*t*t)
                    if predicted_vehicle1_y >  vehicle1.get_location().y:
                        predicted_vehicle1_y =  vehicle1.get_location().y
                    prediction3.append([current_vehicle1_x,predicted_vehicle1_y,-270])


                    for p in prediction3:
                        t = carla.Location()
                        t.x = p[0]
                        t.y = p[1]
                        world.debug.draw_string(t, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=255), life_time=0.000001, persistent_lines=True)




                test_model = OCP.OCP(location.x, location.y, left_waypoint.transform.location.x, yaw, v, delta, prediction3)
                output = test_model.calculate()

                planned_points = []

                for i in range(len(output[0])):
                    point = carla.Location()
                    point.x = output[0][i]
                    point.y = output[1][i]
                    planned_points.append(point)

            
            print("ego vehicle, vehicle 1 veloicty",(ego_vehicle.get_velocity().x ** 2+ ego_vehicle.get_velocity().y ** 2)**0.5,(vehicle1.get_velocity().x ** 2+ vehicle1.get_velocity().y ** 2)**0.5)
            print("ego vehicle, vehicle 1 position difference ", -ego_vehicle.get_location().y + vehicle1.get_location().y)
            print("time ",time.time()-start_time)
            for p in planned_points:
                world.debug.draw_string(p, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=0.000001, persistent_lines=True)
                
            control = carla.VehicleControl()
            control.throttle = ThrottleRatio(output[6][1])
            control.steer = output[4][1] *delta_ratio
            if output[6][1] < 0:
                control.brake = -output[6][1]/10
                print(output[6][1],control.brake)
            else:
                control.brake = 0.0
            control.hand_brake = False
            control.manual_gear_shift = False

            ego_vehicle.apply_control(control)
            print("state",state)

        elif  waypoints.lane_id == 2:
            if state ==2:
                state =3
            location = ego_vehicle.get_location()
            vector_v = ego_vehicle.get_velocity()
            v = (vector_v.x **2 + vector_v.y **2)**0.5

            yaw = np.deg2rad(ego_vehicle.get_transform().rotation.yaw)
            delta = ego_vehicle.get_control().steer


            #output x,y,yaw,v,delta,deltarate,acceleration

            current_vehicle1_x = vehicle1.get_location().x
            current_vehicle1_y = vehicle1.get_location().y
            current_vehicle1_yaw = np.deg2rad(-270)
            current_vehicle1_vel = vehicle1.get_velocity().y
            expected_collision_time = 5
            expected_collision_distance = 20
            predicted_vehicle1_acc = 0

            prediction3 = []

            for t in range(len(planned_points)):
                current_vehicle1_y += current_vehicle1_vel*0.1
                vehicle1_state = [current_vehicle1_x,current_vehicle1_y,current_vehicle1_yaw]
                ego_vehicle_state = [output[0][i],output[1][i],output[2][i]]
                check_collision = OCP.collisionCheck2(vehicle1_state,ego_vehicle_state)
                if check_collision[0]:
                    expected_collision_time = t*0.1
                    break

            if expected_collision_time < 5:
                expected_collision_distance =  -expected_collision_time* vehicle1.get_velocity().y
                # for safe distance
                if expected_collision_distance > 0.5:
                    expected_collision_distance -= 0.5
                
                if expected_collision_distance != 0:
                    predicted_vehicle1_acc = -4/expected_collision_distance
                else:
                    predicted_vehicle1_acc = -10

            for t in range(len(planned_points)):
                predicted_vehicle1_y =  vehicle1.get_location().y + current_vehicle1_vel*0.1*t - predicted_vehicle1_acc*0.5*(0.01*t*t)
                if predicted_vehicle1_y >  vehicle1.get_location().y:
                    predicted_vehicle1_y =  vehicle1.get_location().y
                prediction3.append([current_vehicle1_x,predicted_vehicle1_y,-270])


                for p in prediction3:
                    t = carla.Location()
                    t.x = p[0]
                    t.y = p[1]
                    world.debug.draw_string(t, 'O', draw_shadow=False, color=carla.Color(r=0, g=255, b=255), life_time=0.000001, persistent_lines=True)


            test_model = OCP.OCP(location.x, location.y, left_waypoint.transform.location.x, yaw, v, delta, prediction3)
            output = test_model.calculate()

            planned_points = []

            for i in range(len(output[0])):
                point = carla.Location()
                point.x = output[0][i]
                point.y = output[1][i]
                planned_points.append(point)

            for p in planned_points:
                world.debug.draw_string(p, 'O', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=0.000001, persistent_lines=True)

            control = carla.VehicleControl()

            control.throttle = ThrottleRatio(output[6][1])
            control.steer = output[4][1] *delta_ratio
            if output[6][1] < 0:
                control.brake = -output[6][1]/10

            else:
                control.brake = 0.0
            
            control.hand_brake = False
            control.manual_gear_shift = False
            ego_vehicle.apply_control(control)



            

        # control vehicle1

        if state == 0:
            vehicle1_acc = 0.6

        else:
            vehicle1_acc = 0
            # vehicle velocity 4
            current_vehicle1_x = vehicle1.get_location().x
            current_vehicle1_y = vehicle1.get_location().y
            current_vehicle1_yaw = np.deg2rad(-270)
            current_vehicle1_vel = vehicle1.get_velocity().y
            expected_collision_time = 5
            expected_collision_distance = 20

  
            for t in range(len(planned_points)):
                current_vehicle1_y += current_vehicle1_vel*0.1
                vehicle1_state = [current_vehicle1_x,current_vehicle1_y,current_vehicle1_yaw]
                ego_vehicle_state = [ego_vehicle_prediction[t][0],ego_vehicle_prediction[t][1],ego_vehicle_prediction[t][2]]
                check_collision = OCP.collisionCheck2(vehicle1_state,ego_vehicle_state)

                if check_collision[0]:
                    expected_collision_time = t*0.1
                    break
            if expected_collision_time <= 1.2:
                expected_collision_distance = -expected_collision_time*vehicle1.get_location().y
                # for safe distance original 0.5
                if expected_collision_distance > 0.2:
                    expected_collision_distance -= 0.2
                else:
                    vehicle1_acc = -10

                if expected_collision_distance != 0:
                    # original gain -12
                    vehicle1_acc = -6/expected_collision_distance 
                else:
                    vehicle1_acc = -10
            else:
                if abs(vehicle1.get_velocity().y) < 3.8:
                    vehicle1_acc = 0.05
                elif abs(vehicle1.get_velocity().y) > 4.2:
                    vehicle1_acc = -0.05





            
        control1 = carla.VehicleControl()
        control1.throttle = ThrottleRatio(vehicle1_acc)
        control1.steer = 0
        if vehicle1_acc < 0:
            control1.brake = -vehicle1_acc/10

        else:
            control1.brake = 0.0
        
        control1.hand_brake = False
        control1.manual_gear_shift = False
        vehicle1.apply_control(control1)

        print()
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
    print("finished")
if __name__ == '__main__':
    main()