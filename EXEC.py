#!/usr/bin/env python3
import time
import board
import busio
import adafruit_bno055
from adafruit_servokit import ServoKit
from skyfield.api import load, wgs84, Star
from skyfield import almanac
from datetime import datetime, timezone
import math

class StarTracker:
    def __init__(self, latitude, longitude, elevation=0):
        print("Initializing BNO055...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        
        # Initialize servo controller (PCA9685)
        print("Initializing servos...")
        self.kit = ServoKit(channels=16)
        
        # Servo configuration
        self.pan_channel = 0  # Pan servo on channel 0
        self.tilt_channel = 1  # Tilt servo on channel 1
        
        # Servo angle ranges
        self.pan_min = 0
        self.pan_max = 180
        self.tilt_min = 0
        self.tilt_max = 180
        
        # Center servos at startup
        self.center_servos()
        
        # Initialize Skyfield
        print("Loading ephemeris data...")
        self.ts = load.timescale()
        self.eph = load('de421.bsp')  # Planetary ephemeris
        
        # Set observer location
        self.location = wgs84.latlon(latitude, longitude, elevation)
        
        # Calibration offset (from BNO055 to servo reference frame)
        self.azimuth_offset = 0
        self.altitude_offset = 0
        
        print("Star tracker initialized!")
        
    def center_servos(self):
        """Move servos to center position"""
        self.kit.servo[self.pan_channel].angle = 90
        self.kit.servo[self.tilt_channel].angle = 90
        time.sleep(1)
        
    def calibrate_orientation(self):
        """
        Calibrate the mount orientation using BNO055
        Point mount at known azimuth (e.g., North) before calling
        """
        print("Calibrating... Keep mount pointed North")
        time.sleep(2)
        
        euler = self.sensor.euler
        if euler[0] is not None:
            self.azimuth_offset = euler[0]
            print(f"Azimuth offset set to: {self.azimuth_offset:.2f}°")
        else:
            print("Warning: Could not read sensor for calibration")
            
    def get_current_orientation(self):
        """Get current mount orientation from BNO055"""
        euler = self.sensor.euler
        
        if euler[0] is None:
            return None, None
            
        # euler[0] = heading (0-360°)
        # euler[1] = roll
        # euler[2] = pitch
        
        azimuth = (euler[0] - self.azimuth_offset) % 360
        altitude = euler[2]  # Using pitch as altitude
        
        return azimuth, altitude
        
    def calculate_object_position(self, target):
        """
        Calculate azimuth and altitude for a celestial object
        
        Args:
            target: Skyfield object (planet, star, etc.)
            
        Returns:
            azimuth, altitude in degrees
        """
        t = self.ts.now()
        observer = self.eph['earth'] + self.location
        
        astrometric = observer.at(t).observe(target)
        alt, az, distance = astrometric.apparent().altaz()
        
        return az.degrees, alt.degrees
        
    def move_to_position(self, target_az, target_alt):
        """
        Move servos to point at target azimuth and altitude
        
        Args:
            target_az: Target azimuth in degrees (0-360)
            target_alt: Target altitude in degrees (-90 to 90)
        """
        # Convert azimuth (0-360) to servo angle (0-180)
        # Assuming 180° servo range covers 360° of rotation with gearing
        # Adjust this mapping based on your mechanical setup
        pan_angle = (target_az / 360.0) * 180.0
        
        # Convert altitude (-90 to 90) to servo angle (0-180)
        # 0° altitude = 90° servo, 90° altitude = 180° servo, -90° = 0° servo
        tilt_angle = target_alt + 90
        
        # Constrain to servo limits
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        # Move servos
        self.kit.servo[self.pan_channel].angle = pan_angle
        self.kit.servo[self.tilt_channel].angle = tilt_angle
        
        print(f"Moved to Az: {target_az:.2f}° (servo: {pan_angle:.1f}°), "
              f"Alt: {target_alt:.2f}° (servo: {tilt_angle:.1f}°)")
        
    def track_object(self, target, duration=60, update_interval=1):
        """
        Track a celestial object for a specified duration
        
        Args:
            target: Skyfield object to track
            duration: Tracking duration in seconds
            update_interval: Update rate in seconds
        """
        print(f"Tracking object for {duration} seconds...")
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            # Calculate object position
            az, alt = self.calculate_object_position(target)
            
            # Move to position
            if alt > 0:  # Only track if above horizon
                self.move_to_position(az, alt)
                
                # Display current orientation
                current_az, current_alt = self.get_current_orientation()
                if current_az is not None:
                    print(f"Current orientation: Az: {current_az:.2f}°, Alt: {current_alt:.2f}°")
            else:
                print(f"Object below horizon (altitude: {alt:.2f}°)")
                
            time.sleep(update_interval)
            
    def point_at_star(self, star_name, ra_hours, dec_degrees):
        """
        Point at a specific star by coordinates
        
        Args:
            star_name: Name of the star
            ra_hours: Right ascension in hours
            dec_degrees: Declination in degrees
        """
        star = Star(ra_hours=(ra_hours,), dec_degrees=(dec_degrees,))
        az, alt = self.calculate_object_position(star)
        
        print(f"Pointing at {star_name}")
        print(f"RA: {ra_hours}h, Dec: {dec_degrees}°")
        print(f"Azimuth: {az:.2f}°, Altitude: {alt:.2f}°")
        
        if alt > 0:
            self.move_to_position(az, alt)
        else:
            print(f"{star_name} is below the horizon")


def main():
    # Set your location (Atlanta, Georgia as example)
    LATITUDE = 33.7490  # degrees North
    LONGITUDE = -84.3880  # degrees West
    ELEVATION = 320  # meters
    
    # Initialize tracker
    tracker = StarTracker(LATITUDE, LONGITUDE, ELEVATION)
    
    # Optional: Calibrate by pointing North
    # tracker.calibrate_orientation()
    
    # Example 1: Track a planet (Mars)
    print("\n=== Tracking Mars ===")
    mars = tracker.eph['mars']
    tracker.track_object(mars, duration=30, update_interval=2)
    
    # Example 2: Point at a bright star (Sirius)
    print("\n=== Pointing at Sirius ===")
    tracker.point_at_star("Sirius", ra_hours=6.75, dec_degrees=-16.72)
    time.sleep(5)
    
    # Example 3: Point at Polaris (North Star)
    print("\n=== Pointing at Polaris ===")
    tracker.point_at_star("Polaris", ra_hours=2.53, dec_degrees=89.26)
    time.sleep(5)
    
    # Example 4: Track the Moon
    print("\n=== Tracking Moon ===")
    moon = tracker.eph['moon']
    tracker.track_object(moon, duration=30, update_interval=2)
    
    # Return to center
    print("\n=== Returning to center ===")
    tracker.center_servos()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTracking stopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
