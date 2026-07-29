import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(__file__))

def test_environment_script_runs():
    result = subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'check_environment.py')], capture_output=True, text=True)
    assert 'ros2:' in result.stdout

def test_evaluator_writes_json():
    with tempfile.TemporaryDirectory() as directory:
        subprocess.run([sys.executable, os.path.join(ROOT, 'scripts', 'evaluate_run.py'), '--scenario', 'acc', '--output', directory], check=True)
        with open(os.path.join(directory, 'metrics.json')) as stream:
            assert json.load(stream)['scenario'] == 'acc'
