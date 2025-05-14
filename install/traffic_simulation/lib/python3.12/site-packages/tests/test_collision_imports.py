#!/usr/bin/env python3

"""
Simple test script to verify that the collision avoidance components can be imported.
"""

import sys
import os

def test_imports():
    try:
        print("Testing imports...")
        
        from traffic_simulation.sensors.sensor import Lidar, Radar, SensorReading
        print("✅ Successfully imported sensors")
        
        from traffic_simulation.communication.iot import IoTCommunicator, VehicleData
        print("✅ Successfully imported communication")
        
        from traffic_simulation.planning.collision_avoidance import CollisionAvoidance, CollisionStrategy
        print("✅ Successfully imported planning")
        
        from traffic_simulation.vehicles.vehicle import Vehicle, WhiteCar, SmartCar
        print("✅ Successfully imported vehicles")
        
        # Test creating an instance of each class
        sensor_reading = SensorReading(distance=100.0, angle=45.0)
        print("✅ Created SensorReading")
        
        lidar = Lidar("test_car")
        print("✅ Created Lidar")
        
        radar = Radar("test_car")
        print("✅ Created Radar")
        
        vehicle = Vehicle(0, 0, [(100, 100)])
        print("✅ Created Vehicle")
        
        white_car = WhiteCar(0, 0, [(100, 100)])
        print("✅ Created WhiteCar")
        
        collision_avoidance = CollisionAvoidance("test_car")
        print("✅ Created CollisionAvoidance")
        
        smart_car = SmartCar(0, 0, [(100, 100)])
        print("✅ Created SmartCar")
        
        print("\nAll tests passed!")
        return True
    
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        print(traceback.format_exc())
        return False

if __name__ == "__main__":
    test_imports() 