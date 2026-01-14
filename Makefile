.PHONY: help install run run-headless clean

# Python interpreter
PYTHON := python

# Source directory
SRC_DIR := src

# Main script
MAIN_SCRIPT := $(SRC_DIR)/main.py

# Default target
.DEFAULT_GOAL := help

help: ## Show this help message
	@echo "Available targets:"
	@echo "  make install        - Install dependencies from requirements.txt"
	@echo "  make run            - Run simulation with visualization"
	@echo "  make run-headless   - Run simulation without GUI"
	@echo "  make clean          - Clean temporary files"
	@echo "  make help           - Show this help message"

install: ## Install Python dependencies
	@echo "Installing dependencies..."
	$(PYTHON) -m pip install --upgrade pip
	$(PYTHON) -m pip install -r requirements.txt
	@echo "Dependencies installed successfully!"

run: ## Run simulation with visualization
	@echo "Starting simulation with visualization..."
	$(PYTHON) $(MAIN_SCRIPT)

run-headless: ## Run simulation without GUI
	@echo "Starting simulation in headless mode..."
	$(PYTHON) $(MAIN_SCRIPT) --headless

clean: ## Clean temporary files and caches
	@echo "Cleaning temporary files..."
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
	find . -type f -name "*.pyo" -delete 2>/dev/null || true
	find . -type f -name "*.log" -delete 2>/dev/null || true
	@echo "Cleanup completed!"
