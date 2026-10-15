"""Generate isolated equal-precision benchmark source; production stays untouched."""
from pathlib import Path
import sys

source, destination = map(Path, sys.argv[1:])
text = source.read_text()
for old, new in [
    ('/(count*sigma*sigma*frames)', '/(count*config.camera_noise_sigma_m.front()*config.camera_noise_sigma_m.front()*frames)'),
    ('/(count*p.sigma*p.sigma)', '/(count*config.camera_noise_sigma_m.front()*config.camera_noise_sigma_m.front())'),
]:
    if text.count(old) != 1:
        raise ValueError(f'Instrumentation seam changed: {old}')
    text = text.replace(old, new)
destination.write_text(text)
