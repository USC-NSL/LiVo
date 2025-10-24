import time
import csv
from pynvml import *

def log_nvenc_nvdec(interval, log_file):
    # Initialize NVML
    nvmlInit()
    device_count = nvmlDeviceGetCount()

    # Create/overwrite the CSV file with a header row
    with open(log_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp", "timestamp_ms", "gpu_index", "gpu_name",
                         "gpu_util_percent", "mem_util_percent",
                         "mem_used_mib", "mem_total_mib",
                         "temperature",
                         "encoder_util(%)", "decoder_util(%)", "enc_sampling_us", "dec_sampling_us"])

    try:
        while True:
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            # Get current time in milliseconds
            timestamp_ms = int(time.time() * 1000)
            for i in range(device_count):
                handle = nvmlDeviceGetHandleByIndex(i)
                name = nvmlDeviceGetName(handle)

                # GPU utilization
                util = nvmlDeviceGetUtilizationRates(handle)
                gpu_util = util.gpu  # Percent
                mem_util = util.memory  # Percent

                # Memory usage
                mem_info = nvmlDeviceGetMemoryInfo(handle)
                mem_used = mem_info.used // 1024**2  # MiB
                mem_total = mem_info.total // 1024**2  # MiB

                # Temperature
                temp = nvmlDeviceGetTemperature(handle, NVML_TEMPERATURE_GPU)

                # NVENC utilization
                enc_util, enc_sampling = nvmlDeviceGetEncoderUtilization(handle)
                # NVDEC utilization
                dec_util, dec_sampling = nvmlDeviceGetDecoderUtilization(handle)

                with open(log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        timestamp,
                        timestamp_ms,
                        i,
                        name,
                        gpu_util, mem_util,
                        mem_used, mem_total,
                        temp,
                        enc_util,       # % of time encoder was active
                        dec_util,       # % of time decoder was active
                        enc_sampling,   # sampling period in microseconds
                        dec_sampling
                    ])
            time.sleep(interval)

    except KeyboardInterrupt:
        pass
    finally:
        # Shut down NVML
        nvmlShutdown()

if __name__ == '__main__':
    # Log NVENC/NVDEC usage every second to 'nvenc_nvdec_log.csv'.
    # You can also use sub-second intervals (e.g., 0.1 for 100ms).

    # No decoding
    log_nvenc_nvdec(interval=0.1, log_file='gpu_log_no_decoding_py_log.csv')

    # With decoding
    # log_nvenc_nvdec(interval=0.1, log_file='gpu_log_with_decoding_py_log.csv')

# sudo python3.8 -m pip install nvidia-ml-py
# sudo python3.8 gpu_util_log.py
