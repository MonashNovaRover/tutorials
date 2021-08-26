# tutorials
This package is for learning ROS 2 and can be combined with the Nuclino documentation. :notebook:

### Script Conversion Task
One of the first programs everyone on the software team has to do, prior to converting their first real script, is to convert the **heartbeat_sub.py** script. The original script can be found on GitHub in the **rover** repository [here](https://github.com/novarover/rover/blob/master/scripts/heartbeat_listener.py).

Here is the process you will need to follow:

1. Install this repository into the source folder of your current ROS 2 workspace. This can be a new one or an old one. :package:

2. Create a new branch called [name]. For example, you could call it *harrison* if you were me (I wonder who wrote this...).

3. Read through the previous script, understand what it does and the functionality it must provide. :scroll:

4. Create a new script in the new repository (in this case, this one) and set it to the correct name as per the naming conventions.

5. Replicate the script by programming the functionality using ROS 2, Python 3 and classes. Make sure all PEP8 coding conventions are followed. They can be found on [Nuclino](https://app.nuclino.com/Nova-Rover-Team/Software-2021/Python-1241f1a3-a1ee-45aa-8b68-e0f9e0d1b6ad). :snake:

6. Make sure it works as expected using the command line ROS interface. You do not need to turn this into a launch file; simply running the following line is sufficient:

    `python3 [file].py`

7. Once happy with it working, add all your changes to GitHub on your current branch and make a **pull request** to the **master** branch. Make sure to add both Liam Whittle and Harrison Verrios to the pull request. Do not click merge.

8. The two software STRs will then look through your code and provide feedback. :rocket:


If you have any questions at all, please do not hesistate to contact us and ask away! That is what the #software Slack channel is for. :)