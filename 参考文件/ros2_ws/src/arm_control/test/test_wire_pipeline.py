"""FK -> IK -> Bezier -> production C620 Bus/mock plant -> encoder -> FK."""
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
import yaml
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from kinematics import Chain, error
from send_trajectory import bezier, validate_motion
config = yaml.safe_load((ROOT / 'config/robot_robomaster.yaml').read_text(encoding='utf-8'))
chain = Chain(config)
pose = chain.fk([.12, -.12, .12, -.12, .12, -.12])
goal = chain.ik(pose, [0.] * 6)
validate_motion(config['joints'], [0.] * 6, goal, 3.)
with tempfile.TemporaryDirectory() as directory:
    path = Path(directory) / 'trajectory.txt'
    path.write_text('\n'.join(' '.join(map(str, bezier([0.] * 6, goal, k / 600, 3.)[0])) for k in range(601)))
    output = subprocess.check_output([sys.argv[1], str(path)], text=True, timeout=20)
measured = np.fromstring(output, sep=' ')
assert measured.size == 6 and np.max(np.abs(measured - goal)) < .02, output
residual = error(pose, chain.fk(measured))
assert np.linalg.norm(residual[:3]) < .005 and np.linalg.norm(residual[3:]) < .03, residual
print('PASS: FK/IK -> Bezier -> C620 current frames -> six-axis plant -> encoder -> FK; residual:', residual)
