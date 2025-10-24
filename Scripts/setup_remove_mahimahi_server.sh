sudo iptables -D FORWARD -s 68.181.32.215 -d 100.64.0.2 -j ACCEPT
sudo iptables -t nat -D PREROUTING -s 68.181.32.215 -j DNAT --to-destination 100.64.0.2