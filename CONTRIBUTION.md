# Contribution Guidelines

Follow this guide to start contributing to gennav

## Methods of Contributing

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

* After making the final changes, you can push the new changes using :

	```
	git add .
	git commit - m 'comments'
	git push origin #branch_name
	```

This will push the changes to the forked remote repo. Once this is done, you can open 'Pull Request' (PR) to the repo which will then be reviwed and merged after making some changes (if any)

## Configuring

* Fork the repository to your github account by pressing the 'Fork' button on the top  	 right corner of the screen when you open the repo

* Clone your fork to your computer using :
	
	```
	git clone https://github.com/#github_username/gennav.git
	```

* Set the remotes :

	```
	git remote add origin github.com/#github_username/gennav.git
	git remote add upstream github.com/ERC-BPGC/gennav.git
	```
