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

# Check if Docker daemon is running
if ! docker info &> /dev/null; then
    echo "Error: Docker daemon is not running."
    echo "Please start Docker Desktop and try again."
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

# Note about API credentials (optional)
echo ""
echo "Note: DiaROS now uses local models by default"
echo "API credentials are optional for backward compatibility"

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
echo "2. Run: ./scripts/run.sh"
echo ""
echo "To start the container:"
echo "  ./scripts/run.sh"
echo ""
echo "To monitor the system:"
echo "  ./scripts/monitor.sh"
echo ""