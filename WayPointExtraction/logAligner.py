import numpy as np

def read_lines(file_path):
    with open(file_path, 'r') as file:
        lines = [line.strip() for line in file.readlines() if line.strip()]
    return lines


def convert_to_double_arrays(lines):
    """Convert a list of space-separated strings into an array of double arrays."""
    double_arrays = []
    for line in lines:
        # Split the line by spaces, filter out empty strings, and convert to floats
        double_array = [float(value) for value in line.split() if value]
        double_arrays.append(double_array)
    return double_arrays


def read_data_after_last_256(file_path):
    """Read data from a file and return lines starting from the line after the last 256. (Select)"""
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Find the index of the last occurrence of '256'
    last_256_index = -1
    for i, line in enumerate(lines):
        if '256' in line.split():
            last_256_index = i
    
    # Return the lines starting from the line after the last '256'
    if last_256_index != -1 and last_256_index + 1 < len(lines):
        return lines[last_256_index + 1:]
    else:
        return []
    

def main():
    gamepadLogsPath = "logs/z.gamepad.dat"
    pandaLogsPath = "logs/z.panda0.dat"
    
    pandaLines = read_lines(pandaLogsPath)
    gamepadData = convert_to_double_arrays(read_data_after_last_256(gamepadLogsPath))

    qReal = []
    qReal.extend([[float(elem) for elem in pandaLine.split(" ") if elem][:8] for pandaLine in pandaLines])

    selectTime = gamepadData[0][0]
    
    # start qReal from time when select was pressed
    qReal = qReal[int((int(selectTime*100)-1)/len(gamepadData)*len(qReal)):]

    print(len(qReal), len(gamepadData))
    print(qReal[:3], "\n\n")
    print(gamepadData[:3])

    np.save("logs/qReal", np.asarray(qReal))
    #np.save("logs/gamePadAfterSelect", np.asarray(gamepadData))


if __name__ == "__main__":
    main()
