
from setuptools import setup, find_packages
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "gennav"))

# Package meta-data
NAME = "gennav"
DESCRIPTION = "Python Package for Robot Navigation Algorithms"
REPOSITORY = "https://github.com/ERC-BPGC/gennav"
EMAIL = "ercbitsgoa19@gmail.com"
AUTHOR = "ERC BITS Goa"
REQUIRES_PYTHON = ">=2.7.0"
VERSION = "0.1.0"
KEYWORDS = ("path planning", "robotics", "motion planning", "navigation", "algorithms")
CLASSIFIERS = (
        # Trove classifiers
        # Full list: https://pypi.python.org/pypi?%3Aaction=list_classifiers
        "Development Status :: 3 - Alpha",
        'Intended Audience :: Developers',
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Natural Language :: English",
        "Topic :: Software Development",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules"
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 2.7"
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
)
with open("README.md", "r") as f:
    LONG_DESCRIPTION = f.read()
PROJECT_URLS = { 
    'Bug Reports': 'https://github.com/ERC-BPGC/gennav/issues',
    'Source': 'https://github.com/ERC-BPGC/gennav',
    'About Us': 'http://erc-bpgc.github.io/',
}

# What packages are required for this module to be executed?
REQUIRED = (
    "descartes",
    "matplotlib",
    "numpy",
    "shapely"
)

# What packages are optional?
EXTRAS = {}

# Modules to exclude
EXCLUDES = ["tests", "*.tests", "*.tests.*", "tests.*"]

# Where the magic happens:
setup(
    name=NAME,
    version=VERSION,
    description=DESCRIPTION,
    long_description=LONG_DESCRIPTION,
    long_description_content_type="text/markdown",
    author=AUTHOR,
    author_email=EMAIL,
    python_requires=REQUIRES_PYTHON,
    url=REPOSITORY,
    packages=find_packages(exclude=EXCLUDES),
    tests_require=["pytest"],
    install_requires=REQUIRED,
    extras_require=EXTRAS,
    include_package_data=True,
    license="MIT",
    keywords=KEYWORDS,
    classifiers=CLASSIFIERS,
    project_urls=PROJECT_URLS
)
