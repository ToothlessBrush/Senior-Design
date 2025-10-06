#!/usr/bin/env python3
import bluetooth
import time
import sys

# HC-05 MAC address
HC05_MAC = "98:D3:21:F7:C3:E0"
RFCOMM_CHANNEL = 1

def connect_hc05():
    """Connect to HC-05 and return the socket"""
    print(f"Connecting to HC-05 at {HC05_MAC}...")
    
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    
    try:
        sock.connect((HC05_MAC, RFCOMM_CHANNEL))
        print("Connected successfully!")
        return sock
    except Exception as e:
        print(f"Connection failed: {e}")
        sys.exit(1)

def main():
    # Connect to HC-05
    sock = connect_hc05()
    
    try:
        print("\nListening for data from STM32...")
        print("Press Ctrl+C to exit\n")
        
        while True:
            # Check if data is available
            try:
                data = sock.recv(1024)
                if data:
                    print(f"{data.decode('utf-8', errors='ignore')}", end='')
            except bluetooth.BluetoothError as e:
                print(f"\nBluetooth error: {e}")
                break
            except KeyboardInterrupt:
                print("\n\nExiting...")
                break
                
    finally:
        sock.close()
        print("Connection closed.")

if __name__ == "__main__":
    # Install dependencies first if needed
    try:
        import bluetooth
    except ImportError:
        print("PyBluez not found. Install with:")
        print("  sudo pacman -S python-pybluez")
        sys.exit(1)
    
    main()
