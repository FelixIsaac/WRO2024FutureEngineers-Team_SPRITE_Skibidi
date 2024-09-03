#!/bin/bash

# The virtual environment directory (at the root level)
VENV_DIR="./"

# Check if the virtual environment directories exist
if [ ! -d "${VENV_DIR}bin" ] || [ ! -d "${VENV_DIR}lib" ] || [ ! -d "${VENV_DIR}include" ]; then
    echo "Virtual environment directories not found. Creating a new virtual environment..."
    python3 -m venv .
else
    echo "Virtual environment already exists."
fi

# Activate the virtual environment
source "${VENV_DIR}bin/activate"

# Upgrade pip
pip install --upgrade pip

# Install the required packages
if [ -f "requirements.txt" ]; then
    echo "Installing dependencies from requirements.txt..."
    pip install -r requirements.txt
else
    echo "requirements.txt not found. Please ensure the file exists."
    exit 1
fi

# Notify the user that the setup is complete
echo "Setup complete. Virtual environment is ready to use."
