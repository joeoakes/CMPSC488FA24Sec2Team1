set -e

echo "
============
Camera Check
============"
echo "Checking that camera is connected"
test -e /dev/video0
echo "Checking camera permissions"
test -rw /dev/video0
