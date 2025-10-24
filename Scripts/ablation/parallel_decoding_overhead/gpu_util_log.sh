#!/bin/bash

# NVENC encoder stats not working. Use python's pynvml to get the stats

# For base case without running livo_nocull

nvidia-smi \
            --query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu,encoder.stats.sessionCount,encoder.stats.averageFps,encoder.stats.averageLatency \
            --format=csv \
            -lms 100 \
            > gpu_log_base.csv

# Without parallel decoding 

# nvidia-smi \
#             --query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu,encoder.stats.sessionCount,encoder.stats.averageFps,encoder.stats.averageLatency \
#             --format=csv \
#             -lms 100 \
#             > gpu_log_no_decoding.csv

# For parallel decoding overhead

# nvidia-smi \
#             --query-gpu=index,name,utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu,encoder.stats.sessionCount,encoder.stats.averageFps,encoder.stats.averageLatency \
#             --format=csv \
#             -lms 100 \
#             > gpu_log_parallel_decoding.csv