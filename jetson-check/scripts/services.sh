echo "
==============
Services Check
=============="
# List of services to check
services=("sshd" "NetworkManager")

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Initialize error flag
error_flag=0

# Check each service
for service in "${services[@]}"; do
    if systemctl is-active --quiet "$service"; then
        echo -e "${GREEN}$service is running${NC}"
    else
        echo -e "${RED}$service is NOT running${NC}"
        error_flag=1
    fi
done

# Exit with error code if any service failed
if [ $error_flag -ne 0 ]; then
    exit 1
else
    exit 0
fi
