Header header

# name of the wireless interface (e.g. wlan0, wlan1, etc.)
string iface

#network delay obtained using the ros service call (application level roung trip time without using ICMP echo request)
float32 delay

#Flag to check if the link is alive or dead
bool alive
