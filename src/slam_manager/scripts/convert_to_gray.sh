convert $1.png -type Palette -colorspace Gray -alpha off -strip $1.png
YELLOW='\033[0;33m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
CYAN='\033[0;31m'
PURPLE='\033[0;35m'
LIGHT_GRAY='\033[0;37m'
NC='\033[0m' # No Color

echo -e "${BLUE}convert $1.png -type Palette -colorspace Gray -alpha off -strip $1.png${NC}"