#!/bin/bash

echo "🔧 Setting up RL Toolpath environment..."

# Step 1: Create virtual environment
python3.10 -m venv venv
echo "✅ Virtual environment created."

# Step 2: Activate virtual environment
source venv/bin/activate
echo "✅ Virtual environment activated."

# Step 3: Upgrade pip
pip install --upgrade pip

# Step 4: Install dependencies
pip install gymnasium[all] matplotlib numpy noise
echo "✅ Python dependencies installed."

# Step 5: Create outputs folder if it doesn't exist
mkdir -p outputs
echo "📁 'outputs/' folder is ready."

echo "🎉 Setup complete. To run your script:"
echo "source venv/bin/activate && python rl_toolpath_optimizer.py"
