# Copy this file to Scripts/livo-site.sh and edit it for both machines.
# Scripts/livo-site.sh is ignored by git.

export LIVO_SERVER_HOST="68.181.32.205"
export LIVO_CLIENT_HOST="68.181.32.215"
export LIVO_MAHIMAHI_HOST="100.64.0.2"

export LIVO_DATA_ROOT="/datassd/KinectStream/panoptic_data"
export LIVO_USER_TRACE_ROOT="${LIVO_ROOT}/data"
export LIVO_OUTPUT_ROOT="/datassd/pipeline_cpp"
export LIVO_TRACE_ROOT="${LIVO_ROOT}/Scripts/mahi_traces"

export LIVO_SERVER_CPUS="0-11"
export LIVO_CLIENT_CPUS="0-19"
export LIVO_DRACO_CPUS="0-3"
