project = "RKO-LIO"
html_title = "Lidar-inertial odometry without sensor-specific modeling"
html_short_title = "Lidar-inertial odometry"

html_theme = "alabaster"
rosdoc2_settings = {"override_theme": False}
html_static_path = ["_static"]
html_css_files = [
    "custom.css",
]

html_sidebars = {
    "**": [
        "about.html",
        "searchfield.html",
        "navigation.html",
    ]
}

html_theme_options = {
    "description": "Lidar-inertial odometry",
    "github_user": "PRBonn",
    "github_repo": "rko_lio",
    "github_button": "true",
    "github_type": "star",
    "show_relbar_bottom": "true",
    "extra_nav_links": {
        "ROS Index": "https://index.ros.org/p/rko_lio/",
        "PyPI": "https://pypi.org/project/rko-lio/",
    },
}

autodoc_mock_imports = [
    "numpy",
    "pyquaternion",
    "typer",
    "pyyaml",
    "tqdm",
    "rko_lio_pybind",
    "rko_lio.rko_lio_pybind",
    "rko_lio.dataloaders.helipr_file_reader_pybind",
]
pkgs_to_mock = [
    "numpy",
    "pyquaternion",
    "typer",
    "pyyaml",
    "tqdm",
    "rko_lio_pybind",
    ".rko_lio_pybind",
    "rko_lio.rko_lio_pybind",
    "rko_lio.dataloaders.helipr_file_reader_pybind",
]

extensions = ["sphinx.ext.napoleon"]

import os
import subprocess
import sys


def run_apidoc(app):
    module_dir = sys.path[0]
    output_dir = os.path.join(app.srcdir, "python")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Run sphinx-apidoc command line tool
    subprocess.run(
        [
            "sphinx-apidoc",
            "-o",
            output_dir,
            module_dir,
            "--force",
            "--separate",
        ],
        check=True,
    )


def setup(app):
    app.connect("builder-inited", run_apidoc)
