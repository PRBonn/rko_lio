import os
import sys

# Add Python source folder for autodoc if needed
sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "python"))
)

# Sphinx extensions
extensions = [
    "breathe",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
]

# Breathe configuration: Path to Doxygen XML output
breathe_projects = {
    "rko_lio": os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "doc/doxygen/xml")
    )
}
breathe_default_project = "rko_lio"

# General project info
project = "rko_lio"
author = "Your Name"

# Paths
templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# HTML output options (optional)
html_theme = "sphinx_rtd_theme"
