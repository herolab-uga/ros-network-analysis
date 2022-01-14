Header header

# name of the wireless interface (e.g. wlan0, wlan1, etc.)
string iface

# ssid of the access point (e.g. ROBOT0, ROBOT1, CommandStation, etc.)
string ssid

# Connection status (1 is connected and 0 is disconnected/error0
bool status 

# Received Signal Strength (RSS) in dBm
int32 txpower

# Received Signal Strength (RSS) in dBm
int32 rssi

# Link Quality of the wireless link in percentage (scale of 1 to 100)
float32 lqi

# Noise floor of the wireless link in dBm (only limited NICs provide this correctly)
int32 noise
