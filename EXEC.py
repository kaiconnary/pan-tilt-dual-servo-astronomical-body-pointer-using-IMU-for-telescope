from skyfield.api import load, wgs84
from skyfield import almanac
from datetime import datetime, timezone
import time
import serial
import serial.tools.list_ports
import json
import re

class AstroTracker:
    def __init__(self, latitude, longitude, elevation=0, port=None, baudrate=9600):
        """
        Initialize the astronomical tracker with IMU feedback
        
        Args:
            latitude: Observer latitude in degrees
            longitude: Observer longitude in degrees
            elevation: Observer elevation in meters
            port: Serial port (e.g., 'COM3' or '/dev/ttyUSB0')
            baudrate: Serial communication speed
        """
        # Load ephemeris data
        print("Loading astronomical data...")
        self.ts = load.timescale()
        self.eph = load('de421.bsp')  # Planetary ephemeris
        
        # Set observer location
        self.location = wgs84.latlon(latitude, longitude, elevation)
        
        # Initialize serial connection to Arduino
        self.serial_conn = None
        if port:
            self.connect_arduino(port, baudrate)
        else:
            self.auto_connect_arduino(baudrate)
        
        # Wait for Arduino to initialize IMU
        time.sleep(3)
        
        # Check IMU calibration status
        self.check_imu_calibration()
    
    def list_serial_ports(self):
        """List all available serial ports"""
        ports = serial.tools.list_ports.comports()
        available_ports = []
        print("\nAvailable serial ports:")
        for port in ports:
            print(f"  {port.device} - {port.description}")
            available_ports.append(port.device)
        return available_ports
    
    def auto_connect_arduino(self, baudrate=9600):
        """Attempt to automatically connect to Arduino"""
        ports = serial.tools.list_ports.comports()
        arduino_keywords = ['Arduino', 'CH340', 'USB-SERIAL', 'ttyUSB', 'ttyACM']
        
        for port in ports:
            for keyword in arduino_keywords:
                if keyword.lower() in port.description.lower() or keyword in port.device:
                    try:
                        print(f"\nAttempting to connect to {port.device}...")
                        self.connect_arduino(port.device, baudrate)
                        return
                    except:
                        continue
        
        print("\nCouldn't auto-detect Arduino. Available ports:")
        self.list_serial_ports()
        print("Please specify port manually when creating AstroTracker")
    
    def connect_arduino(self, port, baudrate=9600):
        """Connect to Arduino via serial"""
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=2)
            time.sleep(2)  # Wait for Arduino to reset
            print(f"Connected to Arduino on {port}")
            
            # Read initial messages
            time.sleep(1)
            while self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode().strip()
                print(f"Arduino: {line}")
                
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")
            self.serial_conn = None
    
    def send_command(self, command, data=None):
        """Send command to Arduino and get response"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("Warning: No Arduino connection")
            return None
        
        try:
            # Format: COMMAND:data1,data2\n
            if data:
                cmd_str = f"{command}:{','.join(map(str, data))}\n"
            else:
                cmd_str = f"{command}\n"
            
            self.serial_conn.write(cmd_str.encode())
            time.sleep(0.1)  # Give Arduino time to process
            
            # Read response(s)
            responses = []
            timeout = time.time() + 1  # 1 second timeout
            while time.time() < timeout:
                if self.serial_conn.in_waiting:
                    response = self.serial_conn.readline().decode().strip()
                    if response:
                        responses.append(response)
                time.sleep(0.05)
            
            return responses if responses else None
            
        except Exception as e:
            print(f"Serial communication error: {e}")
        return None
    
    def check_imu_calibration(self):
        """Check BNO055 calibration status"""
        print("\nChecking IMU calibration status...")
        response = self.send_command('STATUS')
        if response:
            for line in response:
                if 'Calibration' in line or 'System' in line or 'Gyro' in line:
                    print(f"  {line}")
    
    def calibrate_imu(self):
        """Calibrate the IMU at current position"""
        print("\nCalibrating IMU...")
        print("Ensure mount is in a known orientation (e.g., pointing North at 45° elevation)")
        response = self.send_command('CALIBRATE')
        if response:
            for line in response:
                print(f"  {line}")
    
    def get_imu_orientation(self):
        """Get current orientation from IMU"""
        response = self.send_command('GET_ORIENTATION')
        if response:
            for line in response:
                if 'ORIENTATION:' in line:
                    # Parse: ORIENTATION: Az=123.45° Alt=45.67° Roll=0.12°
                    match = re.search(r'Az=([\d.]+)° Alt=([\d.]+)° Roll=([\d.]+)°', line)
                    if match:
                        return {
                            'azimuth': float(match.group(1)),
                            'altitude': float(match.group(2)),
                            'roll': float(match.group(3))
                        }
        return None
    
    def get_body_position(self, body_name, t=None):
        """
        Calculate alt-azimuth position of astronomical body
        
        Args:
            body_name: Name of body ('sun', 'moon', 'mars', etc.)
            t: Time object (uses current time if None)
        
        Returns:
            dict with 'altitude' and 'azimuth' in degrees
        """
        if t is None:
            t = self.ts.now()
        
        # Get the astronomical body
        bodies = {
            'sun': self.eph['sun'],
            'moon': self.eph['moon'],
            'mercury': self.eph['mercury'],
            'venus': self.eph['venus'],
            'mars': self.eph['mars'],
            'jupiter': self.eph['jupiter barycenter'],
            'saturn': self.eph['saturn barycenter'],
            'uranus': self.eph['uranus barycenter'],
            'neptune': self.eph['neptune barycenter']
        }
        
        if body_name.lower() not in bodies:
            raise ValueError(f"Unknown body: {body_name}")
        
        body = bodies[body_name.lower()]
        
        # Calculate position from observer location
        observer = self.eph['earth'] + self.location
        astrometric = observer.at(t).observe(body)
        alt, az, distance = astrometric.apparent().altaz()
        
        return {
            'altitude': alt.degrees,
            'azimuth': az.degrees,
            'distance': distance.au
        }
    
    def set_target_position(self, azimuth, altitude):
        """Send target position to Arduino (closed-loop with IMU)"""
        response = self.send_command('MOVE', [azimuth, altitude])
        return response
    
    def wait_for_position(self, timeout=30, tolerance=1.0):
        """Wait for mount to reach target position"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            response = self.send_command('STATUS')
            if response:
                for line in response:
                    if 'STATUS:' in line:
                        # Parse status to check if we're close to target
                        match = re.search(r'Target Az=([\d.]+)° Alt=([\d.]+)° \| Current Az=([\d.]+)° Alt=([\d.]+)°', line)
                        if match:
                            target_az = float(match.group(1))
                            target_alt = float(match.group(2))
                            current_az = float(match.group(3))
                            current_alt = float(match.group(4))
                            
                            az_error = abs(target_az - current_az)
                            # Handle 0/360 wraparound
                            if az_error > 180:
                                az_error = 360 - az_error
                            
                            alt_error = abs(target_alt - current_alt)
                            
                            print(f"  Position error: Az={az_error:.2f}° Alt={alt_error:.2f}°")
                            
                            if az_error < tolerance and alt_error < tolerance:
                                print("  Target reached!")
                                return True
            
            time.sleep(0.5)
        
        print("  Timeout waiting for position")
        return False
    
    def point_at_body(self, body_name, wait=True):
        """Point servos at specified astronomical body with IMU feedback"""
        pos = self.get_body_position(body_name)
        
        print(f"\n{'='*60}")
        print(f"{body_name.upper()} Position:")
        print(f"  Calculated Altitude: {pos['altitude']:.2f}°")
        print(f"  Calculated Azimuth: {pos['azimuth']:.2f}°")
        print(f"  Distance: {pos['distance']:.2f} AU")
        
        if pos['altitude'] < 0:
            print(f"  WARNING: {body_name} is below the horizon!")
            return False
        
        # Get current IMU orientation before moving
        current_orient = self.get_imu_orientation()
        if current_orient:
            print(f"\nCurrent IMU Orientation:")
            print(f"  Azimuth: {current_orient['azimuth']:.2f}°")
            print(f"  Altitude: {current_orient['altitude']:.2f}°")
        
        # Send target to Arduino
        print(f"\nSending target to Arduino...")
        response = self.set_target_position(pos['azimuth'], pos['altitude'])
        if response:
            for line in response:
                print(f"  Arduino: {line}")
        
        # Wait for positioning
        if wait:
            print("\nWaiting for mount to reach target...")
            self.wait_for_position(timeout=30, tolerance=0.5)
            
            # Verify final position
            final_orient = self.get_imu_orientation()
            if final_orient:
                print(f"\nFinal IMU Orientation:")
                print(f"  Azimuth: {final_orient['azimuth']:.2f}°")
                print(f"  Altitude: {final_orient['altitude']:.2f}°")
                print(f"  Pointing Error: Az={abs(pos['azimuth']-final_orient['azimuth']):.2f}° Alt={abs(pos['altitude']-final_orient['altitude']):.2f}°")
        
        print('='*60)
        return True
    
    def track_body(self, body_name, duration_minutes=60, update_interval=30):
        """
        Continuously track a body for specified duration
        
        Args:
            body_name: Name of body to track
            duration_minutes: How long to track in minutes
            update_interval: Seconds between position updates
        """
        print(f"\n{'='*60}")
        print(f"Starting tracking of {body_name.upper()}")
        print(f"Duration: {duration_minutes} minutes")
        print(f"Update interval: {update_interval} seconds")
        print('='*60)
        
        start_time = time.time()
        end_time = start_time + (duration_minutes * 60)
        
        try:
            while time.time() < end_time:
                self.point_at_body(body_name, wait=True)
                
                remaining = (end_time - time.time()) / 60
                print(f"\n⏱ Time remaining: {remaining:.1f} minutes\n")
                
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print("\n\nTracking stopped by user")
    
    def find_visible_bodies(self):
        """List all currently visible astronomical bodies"""
        t = self.ts.now()
        visible = []
        
        bodies = ['sun', 'moon', 'mercury', 'venus', 'mars', 
                  'jupiter', 'saturn', 'uranus', 'neptune']
        
        print("\n" + "="*60)
        print("Currently Visible Bodies:")
        print("="*60)
        
        for body in bodies:
            pos = self.get_body_position(body, t)
            if pos['altitude'] > 0:
                visible.append(body)
                print(f"{body.upper():12s} - Alt: {pos['altitude']:6.2f}°  Az: {pos['azimuth']:6.2f}°")
        
        if not visible:
            print("No bodies currently visible above horizon")
        
        print("="*60)
        return visible
    
    def home_position(self):
        """Move servos to home position"""
        print("\nMoving to home position...")
        response = self.send_command('HOME')
        if response:
            for line in response:
                print(f"  Arduino: {line}")
        time.sleep(2)
    
    def close(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial connection closed")


def main():
    """Example usage"""
    
    # Set your location (example: San Francisco)
    latitude = 37.7749    # degrees North
    longitude = -122.4194  # degrees West
    elevation = 52         # meters
    
    # Initialize tracker (will auto-detect Arduino)
    # Or specify port: tracker = AstroTracker(lat, lon, elev, port='/dev/ttyUSB0')
    print("="*60)
    print("ASTRONOMICAL TRACKER WITH BNO055 IMU")
    print("="*60)
    
    tracker = AstroTracker(latitude, longitude, elevation)
    
    if not tracker.serial_conn:
        print("ERROR: Could not connect to Arduino")
        return
    
    try:
        # Calibrate IMU (optional - do this if mount orientation is known)
        # tracker.calibrate_imu()
        
        # Move to home position
        tracker.home_position()
        
        # Show visible bodies
        tracker.find_visible_bodies()
        
        # Get current IMU orientation
        print("\nCurrent IMU reading:")
        orient = tracker.get_imu_orientation()
        if orient:
            print(f"  Azimuth: {orient['azimuth']:.2f}°")
            print(f"  Altitude: {orient['altitude']:.2f}°")
            print(f"  Roll: {orient['roll']:.2f}°")
        
        # Point at the moon with precise IMU feedback
        tracker.point_at_body('moon', wait=True)
        
        time.sleep(3)
        
        # Point at the sun
        tracker.point_at_body('sun', wait=True)
        
        # Uncomment to track continuously (updates every 30 seconds)
        # tracker.track_body('moon', duration_minutes=10, update_interval=30)
        
    except KeyboardInterrupt:
        print("\n\nProgram stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        tracker.close()


if __name__ == "__main__":
    main()
