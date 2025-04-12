import numpy as np

def read_gps_file(gps_file):
    """
    Reads a GPS file with format:
    timestamp tx ty tz
    Returns: numpy array of shape (N, 4)
    """
    data = np.loadtxt(gps_file)
    return data

def read_imu_file(imu_file):
    """
    Reads an IMU file with format:
    timestamp qx qy qz qw
    Returns: numpy array of shape (N, 5)
    """
    data = np.loadtxt(imu_file)
    return data

def write_tum_file(output_file, merged_data):
    """
    Writes the merged data to a TUM file with format:
    timestamp tx ty tz qx qy qz qw
    """
    with open(output_file, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")  # Optional header
        for row in merged_data:
            f.write("{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(*row))
    print(f"TUM file saved to: {output_file}")

def merge_gps_imu(gps_data, imu_data, tolerance=0.001):
    """
    Merges GPS and IMU data by timestamp.
    - gps_data: numpy array of shape (N, 4) -> [timestamp, tx, ty, tz]
    - imu_data: numpy array of shape (M, 5) -> [timestamp, qx, qy, qz, qw]
    - tolerance: maximum allowed time difference for merging
    
    Returns: numpy array of merged data with shape (K, 8)
    """
    merged_data = []
    gps_idx, imu_idx = 0, 0

    while gps_idx < len(gps_data) and imu_idx < len(imu_data):
        gps_time = gps_data[gps_idx, 0]
        imu_time = imu_data[imu_idx, 0]

        # If timestamps are close enough, merge the data
        if abs(gps_time - imu_time) < tolerance:
            merged_row = np.hstack((gps_data[gps_idx], imu_data[imu_idx, 1:]))  # Combine position and orientation
            merged_data.append(merged_row)
            gps_idx += 1
            imu_idx += 1
        elif gps_time < imu_time:
            # GPS timestamp is smaller, move to the next GPS entry
            gps_idx += 1
        else:
            # IMU timestamp is smaller, move to the next IMU entry
            imu_idx += 1

    return np.array(merged_data)

def main():
    # Input files
    gps_file = "/data/yhy/gps_trajectory.txt"
    imu_file = "/data/yhy/imu_trajectory.txt"

    # Output file
    output_file = "/data/yhy/groundtruth.txt"

    # Read GPS and IMU data
    gps_data = read_gps_file(gps_file)
    imu_data = read_imu_file(imu_file)

    # Merge GPS and IMU data
    merged_data = merge_gps_imu(gps_data, imu_data)

    # Write the merged data to a TUM file
    write_tum_file(output_file, merged_data)

if __name__ == "__main__":
    main()
