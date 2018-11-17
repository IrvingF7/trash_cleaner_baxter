# Trash Cleaner Baxter

## Installation
Use Python 2.7.10

### Clone the Repo
```
git clone https://github.com/GrantMeAWish/trash_cleaner_baxter.git
```

### Virtual Environment Setup
Before installing dependencies, we strongly recommend that you setup a virtual environment in the folder that you clone the repo in. This makes sure that your dependencies are fully compatible with the python version we recommend (2.7).
```
virtualenv -p python2 trash_env
cd trash_env
source bin/activate
```

### Installing Dependencies
To install dependencies, we recommend cloning into the repo and installing the libraries using pip
```
pip install -r requirements.txt
```

### Installing Packages for Development
To install the packages, run the following script. Since this project is still in development, DO NOT run python setup.py install. Run this instead so that you can add scripts to the packages and have changes reflected in the packages.
```
python setup.py develop
```
