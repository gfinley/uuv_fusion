Header header

string sender_mac
string sender_ip

# gps data
float64 latitude
float64 longitude
float64 altitude
float64 speed
bool useGPS

# neighbor data
int8 num_neighbors
string[] neighbor_mac
string[] neighbor_ip
float64[] neighbor_snr
