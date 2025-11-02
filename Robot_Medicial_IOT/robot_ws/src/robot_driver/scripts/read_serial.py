import serial

def read_serial(port="/dev/stm32_uart", baudrate=115200):
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud")

        while True:
            if ser.in_waiting > 0:  # check if data is available
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"Received: {line}")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    read_serial()
