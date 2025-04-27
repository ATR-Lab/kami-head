#!/bin/bash

# Script to install requirements with delays between installations to avoid SSL issues
# Uses flags: --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org --no-cache-dir --retries 10 --timeout 120

# Default number of pip installation retries
DEFAULT_MAX_RETRIES=30

# Get the script's directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( dirname "$SCRIPT_DIR" )"

# Default virtual environment directory
DEFAULT_VENV_DIR="$REPO_ROOT/coffee_budy_venv"

# Initialize variables
VENV_DIR="$DEFAULT_VENV_DIR"
MAX_RETRIES="$DEFAULT_MAX_RETRIES"

# Function to print usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -v, --venv PATH       Path to virtual environment (default: $DEFAULT_VENV_DIR)"
    echo "  -r, --retries NUMBER  Maximum number of retries for failed pip installations (default: $DEFAULT_MAX_RETRIES)"
    echo "  -h, --help            Display this help message"
    exit 1
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -v|--venv)
            VENV_DIR="$2"
            shift 2
            ;;
        -r|--retries)
            MAX_RETRIES="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

echo "Using virtual environment: $VENV_DIR"
echo "Maximum retries for failed pip installations: $MAX_RETRIES"

REQ_FILES=(
    "$REPO_ROOT/requirements/core_requirements.txt"
    "$REPO_ROOT/requirements/audio_requirements.txt"
    "$REPO_ROOT/requirements/speech_requirements.txt"
    "$REPO_ROOT/requirements/ai_requirements.txt"
    "$REPO_ROOT/requirements/nlp_requirements.txt"
    "$REPO_ROOT/requirements/ui_requirements.txt"
    "$REPO_ROOT/requirements/utility_requirements.txt"
    "$REPO_ROOT/requirements/serial_requirements.txt"
)

PIP_FLAGS="--trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org --retries 10 --timeout 120"

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo "Virtual environment not found at $VENV_DIR"
    echo "Please make sure you have created the virtual environment"
    exit 1
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Install each requirements file with a delay
for req_file in "${REQ_FILES[@]}"; do
    echo "Installing packages from $req_file"
    
    # Initialize retry counter
    retry_count=0
    success=false
    
    # Retry loop
    while [ $retry_count -lt $MAX_RETRIES ] && [ "$success" = false ]; do
        if [ $retry_count -gt 0 ]; then
            echo "Retry attempt $retry_count of $MAX_RETRIES for $req_file"
            echo "Waiting for 30 seconds before retrying..."
            sleep 30
        fi
        
        python -m pip install $PIP_FLAGS -r "$req_file"
        
        if [ $? -eq 0 ]; then
            success=true
            echo "Successfully installed packages from $req_file"
        else
            echo "Error installing packages from $req_file"
            retry_count=$((retry_count + 1))
        fi
    done
    
    # Check if all retries failed
    if [ "$success" = false ]; then
        echo "All $MAX_RETRIES retries failed for $req_file"
        echo "Please check the requirements file and try again"
        exit 1
    fi
    
    echo "Sleeping for 20 seconds before installing next requirements..."
    sleep 20
done

echo "All requirements have been installed successfully!" 