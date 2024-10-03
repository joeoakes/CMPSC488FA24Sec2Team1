set -e

echo "
=============
Network Check
============="
echo "Getting IP addresses"
ip -o -f inet addr show | awk '{print $2, $4}'
echo "Checking network connectivity"
ping -c 3 google.com
