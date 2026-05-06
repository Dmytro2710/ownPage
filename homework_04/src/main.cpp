#include <cstdint>
#include <iostream>
#include <fstream>

#define MAX_STEP_TICKS 512

const int16_t ticks_per_revolution = 1024;
const double wheel_radius_m = 0.3;
const double wheelbase_m = 1.0;

int16_t fl_ticks[MAX_STEP_TICKS];
int16_t fr_ticks[MAX_STEP_TICKS];
int16_t bl_ticks[MAX_STEP_TICKS];
int16_t br_ticks[MAX_STEP_TICKS];
int32_t timestamp_ms[MAX_STEP_TICKS];
uint16_t num_samples = 0;

int16_t read_odometry_file(
    const char* filename,
    int16_t (&fl)[MAX_STEP_TICKS],
    int16_t (&fr)[MAX_STEP_TICKS],
    int16_t (&bl)[MAX_STEP_TICKS],
    int16_t (&br)[MAX_STEP_TICKS],
    int32_t (&ts)[MAX_STEP_TICKS],
    uint16_t& num_ticks
) {
    // Implementation for reading odometry file
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cout << "Error opening file: " << filename << std::endl;
        return -1;
    }
    for (int i = 0; i < MAX_STEP_TICKS; ++i) {
        if (!(infile >> ts[i] >> fl[i] >> fr[i] >> bl[i] >> br[i])) {
            break; // Stop reading if we reach the end of the file or encounter an error
        }
        num_ticks++;
    }
    infile.close();
    if (num_ticks == 0) {
        std::cout << "No valid data found in file: " << filename << std::endl;
        return -1;
    }
    for (int i = 0; i < num_ticks; ++i) {
        if (ts[i] < 0 || fl[i] < 0 || fr[i] < 0 || bl[i] < 0 || br[i] < 0) {
            std::cout << "Invalid data at line " << i + 1 << ": " 
                      << ts[i] << " " << fl[i] << " " << fr[i] << " " 
                      << bl[i] << " " << br[i] << std::endl;
            return -1;
        }
    }
    for (int i = 1; i < num_ticks; ++i) {
        if (ts[i] <= ts[i - 1]) {
            std::cout << "Timestamps are not in increasing order at line " << i + 1 << std::endl;
            return -1;
        }
    }
    for (int i = 0; i < num_ticks; ++i) {
        if (fl[i] != bl[i] || fr[i] != br[i]) {
            std::cout << "Incorrect tick counts at line " << i + 1 << ": " 
                      << "fl=" << fl[i] << ", fr=" << fr[i] 
                      << ", bl=" << bl[i] << ", br=" << br[i] << std::endl;
            return -1;
        }
    }
    return 0;
}


int main(int argc, char** argv) {
    // The program expects exactly one argument: a path to telemetry samples.
    if (argc != 2) {
        std::cerr << "usage: ugv_odometry <input_path>\n";
        return 1;
    }
    const char* input_path = argv[1];
    if (read_odometry_file(input_path, fl_ticks, fr_ticks, bl_ticks, br_ticks, timestamp_ms, num_samples) != 0) {
        return 1; // Error reading the file
    }

    // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
    //
    // Model parameters:
    //   ticks_per_revolution = 1024
    //   wheel_radius_m       = 0.3
    //   wheelbase_m          = 1.0
    //
    // Input: a text file with 5 whitespace-separated values per line:
    //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
    // Output: a table on stdout, starting from the second sample:
    //         timestamp_ms x y theta

    return 0;
}
