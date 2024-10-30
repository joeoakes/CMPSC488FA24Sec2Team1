set -e

echo "
===============
Resources Check
==============="
echo "Checking disk usage"
df -h | head -n 1
df -h | grep "/$"

echo "Checking memory usage"
free -h

echo "Current CPU Usage: $(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print 100 - $1}')%"

