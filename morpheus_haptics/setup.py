import os
from glob import glob

package_name = 'morpheus_main'

setup(
    data_files=[
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ]
)