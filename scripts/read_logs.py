# Read logs from the device and output them to a CSV file.
import serial
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("log_path")
    args = parser.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=5)

    with open(args.log_path, "w") as f:
        f.write("Device Type, Ticks, Data0, Data1\n")
        try:
            while True:
                    line = ser.readline()
                    print(line)
                    f.write(str(line)[2:-6] + "\n") # really hacky, fix this
        except KeyboardInterrupt:
            pass