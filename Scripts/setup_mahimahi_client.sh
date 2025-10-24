sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables --table filter --policy FORWARD ACCEPT
sudo iptables -t nat -A PREROUTING -s 68.181.32.205 -j DNAT --to-destination 100.64.0.2
sudo iptables -A FORWARD -s 68.181.32.205 -d 100.64.0.2 -j ACCEPT