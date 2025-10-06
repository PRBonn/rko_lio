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
    ".rko_lio_pybind",
]
pkgs_to_mock = [
    "numpy",
    "pyquaternion",
    "typer",
    "pyyaml",
    "tqdm",
    "rko_lio_pybind",
    ".rko_lio_pybind",
]

extensions = []
extensions.append("sphinx.ext.autodoc")

import os
import sys

sys.path.insert(
    0,
    os.path.abspath(
        os.path.join("/home/meher/ros_ws/testing_ws/src/rko_lio/python/rko_lio", "..")
    ),
)
