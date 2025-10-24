sudo ufw allow 8080/tcp
sudo ufw allow 5252/tcp
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables --table filter --policy FORWARD ACCEPT
sudo iptables -t nat -A PREROUTING -s 68.181.32.215 -j DNAT --to-destination 100.64.0.2
sudo iptables -A FORWARD -s 68.181.32.215 -d 100.64.0.2 -j ACCEPT