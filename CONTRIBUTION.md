# Contribution Guidelines

Follow this guide to start contributing to gennav


## Configuring

* Fork the repository to your github account by pressing the 'Fork' button on the top right corner of the screen when you open the repo

* Clone your fork to your computer using :
	
	```
	git clone https://github.com/your-github-username/gennav.git
	```

* Set the remotes :

	```bash
	git remote add origin github.com/your-github-username/gennav.git
	git remote add upstream github.com/ERC-BPGC/gennav.git
	```

* Updating the forks:
	```bash
	git fetch upstream
	```

* Installing dependencies:
	```bash
	python2 -m pip install -r requirements.txt
	pytohn3 -m pip install -r requirements.txt
	python3 -m pip install black 
	```

* Creating a new branch for your contributions:
	```bash
	git checkout -b name-of-your-bugfix-or-feature
	```

## Ways of Contributing

### 1. Openning new issues

You can open issues from the issues page of the repo. Issues can be opened if you:

* Have ideas for new things that can be implemented in the library

* Find something missing from the library 

* Have ideas for changing / optimising current implementations to make them more 
  efficient 

* Find any bug 

### 2. Solving Issues

You can start working on an unsolved issue by requesting to take it up in the [Issues Section](https://github.com/ERC-BPGC/gennav/issues) of the repo on github. The issue will then be assigned to it and you are good to go.

After the issue is assigned to you, you can start working on it by setting up a local
repo by following the steps giving in the configuration section.

* Remember to update your local repo before starting the work everytime by using :
	
	```
	git pull upstream master
	```

* You can create a new branch where you will make the changes using :
	```
	git checkout -b #branch_name
	```


This will push the changes to the forked remote repo. Once this is done, you can open 'Pull Request' (PR) to the repo which will then be reviwed and merged after making some changes (if any)


If you add any new code in the package, use [Google Style](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html) to write the documentation.


* To format code use the following:
	```bash
	isort -rc .
	black .
	flake8 .
	```

We are using pytest for testing the modules in the package. So, while writing the tests follow these rules:
1. Make sure that your file name matches the pattern: test_*.py or *_test.py.
2. Make sure that your function name starts with the test prefix.

You can find more about pytest [here](https://docs.pytest.org/en/latest/goodpractices.html).


Before sending the Pull Request make sure you have commented out the plotting functions in the tests so that builds can pass on travis-ci. 

* To perform local tests we are using [tox](https://tox.readthedocs.io/en/latest/). This tool can be used by running this in the root of the package:
	```bash
	tox	
	```


* After making the final changes, you can push the new changes using :
	```bash
	git add .
	git commit - m 'comments'
	git push origin #branch_name
	```
