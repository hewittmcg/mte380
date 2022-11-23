import matplotlib.pyplot as plt
import argparse
import csv
from enum import Enum
import numpy as np

class Reading(Enum):
    FORWARD_TOF = 0
    SIDE_TOFS = 1
    IMU = 2
    GENERAL = 3
    PIT_DETECT = 4
    IMU_TURN = 5
    TURN_STARTING = 6
    NUM_READINGS = 7

# Labels for each type of data stored
labels = {
    Reading.FORWARD_TOF.value: ["Forward ToF", None],
    Reading.SIDE_TOFS.value: ["Front Side ToF", "Rear Side ToF"],
    Reading.IMU.value: ["DPS X", "Accel Z"],
    Reading.GENERAL.value: ["General 1", "General 2"],
    Reading.PIT_DETECT.value: ["Pit Integrated X Tracked", "Photoresistor Voltage"],
    Reading.IMU_TURN.value: ["Integrated Turn Degrees", "Current Degrees Turned"],
    Reading.TURN_STARTING.value: ["Unused", None],
}

READINGS_TO_GRAPH = [Reading.FORWARD_TOF, Reading.SIDE_TOFS, Reading.IMU, Reading.PIT_DETECT, Reading.IMU_TURN]

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
        data_split = []
        data_max_min_time = []
        data = [[] for e in Reading]
        for row in reader:
            try:
                row = [int(ele) for ele in row]
                if len(row) == 4:
                    # First value is the source
                    source = Reading(row[0])
                    
                    # Track each section of the course in separate arrays
                    if source == Reading.TURN_STARTING:
                        data_split.append(data)
                        data = [[] for e in READINGS_TO_GRAPH]

                    # Handle special cases (we don't want ToF values to be too high)
                    if source == Reading.FORWARD_TOF or source == Reading.SIDE_TOFS:
                        row[IDX_READING1] = min(row[IDX_READING1], TOF_MAX_READING)
                        row[IDX_READING2] = min(row[IDX_READING2], TOF_MAX_READING)
                    if source in READINGS_TO_GRAPH:
                        arr_idx = READINGS_TO_GRAPH.index(source)
                        data[arr_idx].append(row)  
            except Exception as e:
                print(f"Exception adding row to data: {e}")
        # Append last data list
        data_split.append(data)

        for idx, data in enumerate(data_split):
            fig, axs = plt.subplots(len(READINGS_TO_GRAPH), sharex=True)
            fig.suptitle(f"Section {idx} Data")
            for idx in range(len(READINGS_TO_GRAPH)):
                to_plot = data[idx]
                x = [row[IDX_TIMESTAMP] for row in to_plot] 
                if labels[idx][0] is not None:
                    y = [row[IDX_READING1] for row in to_plot]
                    axs[idx].plot(x, y, 'o', label = labels[idx][0]) 
                if labels[idx][1] is not None:
                    y = [row[IDX_READING2] for row in to_plot]
                    axs[idx].plot(x, y, 'o', label = labels[idx][1]) 
                axs[idx].legend()
            
        plt.show()