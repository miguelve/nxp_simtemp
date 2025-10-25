#include <iostream>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <cerrno>
#include <iomanip>
#include <stdexcept>
#include <thread>
#include <cstdlib>


// Define the struct exactly as in the driver (packed layout)
#pragma pack(push, 1)
struct simtemp_sample {
    uint64_t timestamp_ns; // monotonic timestamp in nanoseconds
    int32_t temp_mC;       // temperature in milli-degrees Celsius
    uint32_t flags;        // sample flags
};
#pragma pack(pop)


class TemperatureDevice {
public:
    explicit TemperatureDevice(const std::string& path)
        : temp_fd(::open(path.c_str(), O_RDONLY))
    {
        if (temp_fd < 0) {
            throw std::runtime_error("Failed to open device: " + std::string(std::strerror(errno)));
        }
    }

    ~TemperatureDevice() {
        if (temp_fd >= 0)
            ::close(temp_fd);
    }

    // Waits for the device to become readable
    bool waitForSample(int timeout_ms = -1) {
        struct pollfd pfd {
            .fd = temp_fd,
            .events = POLLIN
        };

        int ret = ::poll(&pfd, 1, timeout_ms);
        if (ret < 0) {
            throw std::runtime_error("poll() failed: " + std::string(std::strerror(errno)));
        }
        if (ret == 0) {
            std::cout << "Timeout reached (" << timeout_ms << " ms)\n";
            return false;
        }

        if (pfd.revents & POLLIN)
            return true;

        return false;
    }
    
    // Waits for threshold crossing
    bool waitForThresholdCrossing(int timeout_ms = -1) {
        struct pollfd pfd {
            .fd = temp_fd,
            .events = POLLPRI
        };

        int ret = ::poll(&pfd, 1, timeout_ms);
        if (ret < 0) {
            throw std::runtime_error("poll() failed: " + std::string(std::strerror(errno)));
        }
        if (ret == 0) {
            std::cout << "Timeout reached (" << timeout_ms << " ms)\n";
            return false;
        }

        if (pfd.revents & POLLPRI)
            return true;

        return false;
    }

    void readSample() {
        uint8_t buffer[128];
        ssize_t bytes = ::read(temp_fd, buffer, sizeof(buffer) - 1);
        if (bytes < 0) {
            throw std::runtime_error("read() failed: " + std::string(std::strerror(errno)));
        }
        
        parse_and_print_samples(buffer,bytes);

        return;
    }

private:
    int temp_fd;
    simtemp_sample sample;

    void parse_and_print_samples(const uint8_t* data, size_t len)
    {
        size_t sample_size = sizeof(simtemp_sample);

        if (len % sample_size != 0) {
            std::cerr << "Warning: buffer size (" << len 
                    << " bytes) is not a multiple of sample size (" 
                    << sample_size << " bytes)." << std::endl;
        }

        size_t count = len / sample_size;

        for (size_t i = 0; i < count; ++i) {
            simtemp_sample sample;
            std::memcpy(&sample, data + i * sample_size, sample_size);

            double temp_C = sample.temp_mC / 1000.0;
            double elapsed_s = sample.timestamp_ns / 1e9;

            std::cout << "Sample " << i << ":\n"
                    << "  Elapsed time:   " << std::fixed << std::setprecision(6)
                    << elapsed_s << " s\n"
                    << "  Temperature:    " << std::fixed << std::setprecision(3)
                    << temp_C << " °C\n"
                    << "  Flags:          0x" << std::hex << sample.flags 
                    << std::dec << "\n\n";
        }
    }
};

void set_test_mode(void) {
    std::cout << "Setting test mode in 3s...\n";
    system("sleep 3");
    system("echo -n test > /sys/devices/platform/nxp_simtemp@0/mode");
}


int main() {
    const std::string devPath = "/dev/nxp_simtemp0";
    TemperatureDevice tempDev(devPath);
    int i = 0;
    
    std::cout << "Setting sampling to 1000ms...\n";
    system("echo 1000 > /sys/devices/platform/nxp_simtemp@0/sampling");
    
    std::cout << "Sampling configuration:\n";
    system("cat /sys/devices/platform/nxp_simtemp@0/sampling");
    std::cout << "Mode configuration:\n";
    system("cat /sys/devices/platform/nxp_simtemp@0/mode");

    std::cout << "Reading 10 samples from " << devPath << "...\n";
    for(i = 0; i < 10; i++)
    {
        try {

            // Wait forever (-1) until driver signals readiness
            if (tempDev.waitForSample(-1)) {
                std::cout << "Data is ready! Reading...\n";
                tempDev.readSample();
            }
        } catch (const std::exception& ex) {
            std::cerr << "Error: " << ex.what() << '\n';
            return 1;
        }
    }
    
    std::cout << "Setting sampling to 100ms..\n";
    system("echo 100 > /sys/devices/platform/nxp_simtemp@0/sampling");
    
    std::cout << "Sampling configuration: \n";
    system("cat /sys/devices/platform/nxp_simtemp@0/sampling");

    std::cout << "Reading 10 samples from " << devPath << "...\n";
    for(i = 0; i < 10; i++)
    {
        try {

            // Wait forever (-1) until driver signals readiness
            if (tempDev.waitForSample(-1)) {
                std::cout << "Data is ready! Reading...\n";
                tempDev.readSample();
            }
        } catch (const std::exception& ex) {
            std::cerr << "Error: " << ex.what() << '\n';
            return 1;
        }
    }
    
    std::cout << "Setting ramp mode..\n";
    system("echo -n ramp > /sys/devices/platform/nxp_simtemp@0/mode");
    
    std::cout << "Mode configuration: \n";
    system("cat /sys/devices/platform/nxp_simtemp@0/mode");

    std::cout << "Reading 10 samples from with ramp mode" << devPath << "...\n";
    for(i = 0; i < 10; i++)
    {
        try {

            // Wait forever (-1) until driver signals readiness
            if (tempDev.waitForSample(-1)) {
                std::cout << "Data is ready! Reading...\n";
                tempDev.readSample();
            }
        } catch (const std::exception& ex) {
            std::cerr << "Error: " << ex.what() << '\n';
            return 1;
        }
    }
    
    std::cout << "Setting test mode in 3s and polling for threshold crossing with POLLPRI flag...\n";
    std::thread cmd_thread(set_test_mode);
    
    if (tempDev.waitForThresholdCrossing(-1)) {
        std::cout << "Got an alarm! Reading...\n";
        tempDev.readSample();
    }
    
    cmd_thread.join();

    return 0;
}
