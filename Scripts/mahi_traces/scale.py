import os

if __name__ == '__main__':
    traces = ["/usr/share/mahimahi/traces/Verizon-LTE-driving.up", "/usr/share/mahimahi/traces/Verizon-LTE-driving.down"]
    scale_factor = 10

    for trace in traces:

        in_file = open(trace, "r")
        out_file_name = trace.split(".")[0]
        if "up" in trace:
            out_file_name += "_scale.up"
        elif "down" in trace:
            out_file_name += "_scale.down"

        out_file = open(out_file_name, "w")
        lines = in_file.readlines()
        for line in lines:
            for i in range(scale_factor):
                out_file.write(line)
        
        in_file.close()
        out_file.close()