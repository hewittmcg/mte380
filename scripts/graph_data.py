import matplotlib.pyplot as plt
import argparse
import csv
from enum import Enum

class Reading(Enum):
    FORWARD_TOF = 0
    SIDE_TOFS = 1
    IMU = 2
    GENERAL = 3
    PHOTORESISTOR = 4
    IMU_TURN = 5
    TURN_STARTING = 6
    NUM_READINGS = 7

# Labels for each type of data stored
labels = {
    Reading.FORWARD_TOF.value: ["Forward ToF", None],
    Reading.SIDE_TOFS.value: ["Front Side ToF", "Rear Side ToF"],
    Reading.IMU.value: ["DPS X", "Accel Z"],
    Reading.GENERAL.value: ["Integrated X Tracked", None],
    Reading.PHOTORESISTOR.value: ["Photoresistor Voltage", None],
    Reading.IMU_TURN.value: ["Integrated Turn Degrees", "Current Degrees Turned"],
    Reading.TURN_STARTING.value: ["Unused", None],
}

TOF_MAX_READING = 2000

# Reading indices
IDX_SOURCE = 0
IDX_TIMESTAMP = 1
IDX_READING1 = 2
IDX_READING2 = 3

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("filepath")
    args = parser.parse_args()
        
    with open(args.filepath, "r") as f:
        reader = csv.reader(f)
        data = [[] for e in Reading]
        for row in reader:
            try:
                row = [int(ele) for ele in row]
                if len(row) == 4:
                    # First value is the source
                    source = Reading(row[0])

                    # Handle special cases (we don't want ToF values to be too high)
                    if source == Reading.FORWARD_TOF or source == Reading.SIDE_TOFS:
                        row[IDX_READING1] = min(row[IDX_READING1], TOF_MAX_READING)
                        row[IDX_READING2] = min(row[IDX_READING2], TOF_MAX_READING)

                    data[source.value].append(row)
            except Exception as e:
                print(f"Exception adding row to data: {e}")    

        fig, axs = plt.subplots(Reading.NUM_READINGS.value)
        fig.suptitle('Data Subplots')

        for idx in range(Reading.NUM_READINGS.value):
            to_plot = data[idx]
            x = [row[IDX_TIMESTAMP] for row in to_plot] 
            if labels[idx][0] is not None:
                y = [row[IDX_READING1] for row in to_plot]
                axs[idx].plot(x, y, label = labels[idx][0]) 
            if labels[idx][1] is not None:
                y = [row[IDX_READING2] for row in to_plot]
                axs[idx].plot(x, y, label = labels[idx][1]) 
            axs[idx].legend()
        
        plt.show()