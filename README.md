# FRC2021-Imported-Test-With-Thomas

1. Cloned FRC2021 code
2. Followed Thomas's magic steps:

##Thomas works his magic: An Explanation

###0. Need to get correct submodule from robototes
- get 2022_updates branch and pull the branch down

3 command line arguments:
	git init (to be able to do the other things)
	git submodule add https://github.com/robototes/2910Common.git common
		clones master branch
	git config -f .gitmodules submodule.common.branch 2022_update
		switches the branch to the 2022_update one
		may need to do a git fetch (?)
	git submodule update --remote common
		updates to 2022 branch

###1. build.gradle and settings.gradle in main robot project
[include 'common'] in settings.gradle (at bottom of page)


[implementation project(':common')] inside dependencies{} in build.gradle

###2. Set [id 'net.ltgt.errorprone' version '2.0.2'] in plugins{} function in the Common build.gradle
- we added version '2.0.2' where there previously was no version specification 
- net.ltgt.errorprone is a github repo with version # 2.0.2. the com.google.errorprone stuff below in dependencies{} is different 

this step may no longer be needed if 2910/robototes updates 

###3. Build 2910 Common stuff FIRST 
- The main project is dependent on the smaller project's jar file
- If the main project tries to compile before 2910 it's a problem because it needs the jar file from the 2910 compilation
- following steps manually determine build order

To fix:
- Put [gradlew build   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2022\jdk"] (or whatever VSCode terminal runs) into the command prompt. Do this inside the COMMON directory FIRST. 

- Then put the same prompt into the MAIN (robot code w/ subsystems/commands) NEXT.

- Should be able to build in VSCode fine after. 

Additional notes:
- there is probably a way to specify build order in gradle
