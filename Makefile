# PyBulletFleet Development Makefile
# Usage: make help

.DEFAULT_GOAL := help

.PHONY: help lint format typecheck test test-fast verify docs bench-smoke bench-full clean

help:  ## Show available targets
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

lint:  ## Run all pre-commit hooks (black, pyright, flake8)
	pre-commit run --all-files --show-diff-on-failure

format:  ## Auto-format code with black
	black pybullet_fleet tests examples

typecheck:  ## Run pyright type checker
	pyright

test:  ## Run tests with coverage (CI equivalent)
	pytest tests/ -v --tb=short --cov=pybullet_fleet --cov-report=term-missing --cov-fail-under=75

test-fast:  ## Quick test run (stop on first failure)
	pytest tests/ -x -q

verify: lint test  ## Full verification (lint + test, CI equivalent)

docs:  ## Build documentation (warnings as errors)
	$(MAKE) -C docs html SPHINXOPTS=-W

bench-smoke:  ## Quick benchmark (~10 seconds)
	python benchmark/run_benchmark.py --duration 10

bench-full:  ## Full benchmark sweep
	python benchmark/run_benchmark.py --sweep 100 500 1000

clean:  ## Remove build artifacts and caches
	rm -rf build/ dist/ .coverage htmlcov/ .pytest_cache/
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
