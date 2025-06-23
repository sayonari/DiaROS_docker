#!/bin/bash

# DiaROS Docker Setup Script

set -e

echo "==================================="
echo "DiaROS Docker Setup"
echo "==================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo "Error: docker-compose is not installed. Please install docker-compose first."
    exit 1
fi

# Create .env file from template if it doesn't exist
if [ ! -f .env ]; then
    echo "Creating .env file from template..."
    cp .env.example .env
    
    # Detect OS and set appropriate DISPLAY variable
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Detected macOS. Setting DISPLAY for XQuartz..."
        sed -i '' 's/DISPLAY=:0/DISPLAY=host.docker.internal:0/' .env
        echo "Please ensure XQuartz is installed and running."
        echo "Run: xhost +localhost"
    fi
else
    echo ".env file already exists. Skipping..."
fi

# Create config directory structure
echo "Creating configuration directories..."
mkdir -p config
mkdir -p recordings
mkdir -p workspace

# Check for API credentials
echo ""
echo "Checking for API credentials..."
echo "--------------------------------"

if [ ! -f config/google_credentials.json ]; then
    echo "⚠️  Google Cloud credentials not found!"
    echo "   Please copy your credentials to: config/google_credentials.json"
fi

if [ ! -f config/a3rt_apikey.data ]; then
    echo "⚠️  A3RT API key not found!"
    echo "   Please copy your API key to: config/a3rt_apikey.data"
fi

# Build Docker image
echo ""
echo "Building Docker image..."
echo "This may take several minutes on first run..."
docker-compose build

echo ""
echo "==================================="
echo "Setup completed!"
echo "==================================="
echo ""
echo "Next steps:"
echo "1. Copy your DiaROS source code to ./workspace/"
echo "2. Place API credentials in ./config/"
echo "3. Run: ./scripts/run.sh"
echo ""
echo "To start the container:"
echo "  ./scripts/run.sh"
echo ""
echo "To monitor the system:"
echo "  ./scripts/monitor.sh"
echo ""