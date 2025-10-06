# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# extensions = []

# templates_path = ['_templates']
# exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "alabaster"
rosdoc2_settings = {"override_theme": False}

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
