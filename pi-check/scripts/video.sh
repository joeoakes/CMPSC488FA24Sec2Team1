set -e

echo "
============
Camera Check
============"
echo "Checking that camera is connected"
test -e /dev/video0
echo "Checking camera permissions"
test -w /dev/video0
test -r /dev/video0
