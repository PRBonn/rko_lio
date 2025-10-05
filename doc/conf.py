# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
from pathlib import Path
project = "rko_lio"
copyright = "2025, Meher Malladi"
author = "Meher Malladi"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["breathe", "exhale"]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

root_doc = "index"

breathe_projects = {"My Project": "./_doxygen/xml"}
breathe_default_project = "My Project"
exhale_args = {
    # These arguments are required
    "containmentFolder": "./api",
    "rootFileName": "library_root.rst",
    # "doxygenStripFromPath": "../cpp/rko_lio/core",
    "doxygenStripFromPath": str(Path(__file__) / "../cpp/rko_lio"),
    # Heavily encouraged optional argument (see docs)
    "rootFileTitle": "Library API",
    # Suggested optional arguments
    "createTreeView": True,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": True,
    "exhaleDoxygenStdin": f"INPUT = {str(Path(__file__) / "../cpp/rko_lio/core")}",
}

primary_domain = "cpp"
highlight_language = "cpp"
# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "alabaster"
html_static_path = ["_static"]
